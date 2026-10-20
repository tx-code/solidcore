// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Region/shell base-class checks over the actual arena graph and owning metadata.
//!
//! Upstream: `SmAObject.cpp::AssertValid` (940--998), `SmTopology.cpp`
//! (356--455), `SmOwningTopology.cpp` (137--209), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | `owning` | ordered AObject/Topology/OwningTopology checks |
//! Deviations: typed vectors encode reciprocal links/counts. Backpointers and
//! stored marks are checked independently. Ambiguous multi-list membership is
//! rejected rather than inventing native peer links; no recursion is used.

use super::{Brep, Object, Reporter};
use crate::error::{Result, SolidError};
use crate::geom::assert_array::labels;
use crate::geom::context::MarkType;
use crate::kernel::Kernel;
use crate::topology::shell::ShellMembers;

fn invalid() -> SolidError {
    SolidError::invalid_input(
        "topology::brep_validation::local_base",
        "ambiguous native list",
    )
}

fn attribute_base(b: &Brep, object: Object, report: &mut Reporter<'_>) -> Result<bool> {
    let mut valid = report.class_predicate(labels::A_OBJECT_TYPE, 0, object, true)?;
    // AttributeStore has a typed single/array container and shared context.
    // Reference-behavior exemptions therefore need no fabricated pointer graph.
    let attrs = b
        .notifications()
        .attributes
        .attributes(crate::geom::attribute::ObjectKey(object.key().0));
    if attrs.len() == 1 {
        valid &= report.class_predicate(labels::A_OBJECT_TYPE, 1, object, true)?;
    } else {
        for _ in &attrs {
            valid &= report.class_predicate(labels::A_OBJECT_TYPE, 2, object, true)?;
        }
    }
    Ok(valid)
}

// owner-present, on owner's list, has actual peer links, owner's list count,
// actual peer-ring count. Membership must not be inferred from a backpointer.
fn peer_state(b: &Brep, object: Object) -> Result<(bool, bool, bool, usize, usize)> {
    match object {
        Object::Region(id) => {
            let own = b.region(id)?.in_brep;
            let list = b.region_list();
            let occurrences = list.iter().filter(|entry| **entry == id).count();
            if occurrences > 1 {
                return Err(invalid());
            }
            let linked = occurrences != 0;
            Ok((
                own,
                own && linked,
                linked,
                list.len(),
                if linked { list.len() } else { 1 },
            ))
        }
        Object::Shell(id) => {
            let owner = b.shell(id)?.region;
            let owner_list = owner.map(|r| b.region(r)).transpose()?;
            let on_owner = owner_list.is_some_and(|r| r.shells.contains(&id));
            let mut actual_count = None;
            for (_, region) in b.regions().iter() {
                let occurrences = region.shells.iter().filter(|entry| **entry == id).count();
                if occurrences > 1 || (occurrences == 1 && actual_count.is_some()) {
                    return Err(invalid());
                }
                if occurrences == 1 {
                    actual_count = Some(region.shells.len());
                }
            }
            Ok((
                owner.is_some(),
                on_owner,
                actual_count.is_some(),
                owner_list.map_or(0, |r| r.shells.len()),
                actual_count.unwrap_or(1),
            ))
        }
        _ => Err(invalid()),
    }
}

fn topology_base(k: &Kernel, b: &Brep, object: Object, report: &mut Reporter<'_>) -> Result<bool> {
    let mut valid = attribute_base(b, object, report)?;
    let (owner, on_list, linked, owner_count, actual_count) = peer_state(b, object)?;
    let mut last_predicate;
    if owner {
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 0, object, on_list)?;
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 1, object, true)?;
        let metadata = b
            .notifications()
            .topology
            .get(&crate::geom::attribute::ObjectKey(object.key().0))
            .copied()
            .unwrap_or_default();
        last_predicate = true;
        for (test, mark) in [
            (2, MarkType::Mark),
            (3, MarkType::Mark2),
            (4, MarkType::Mark3),
        ] {
            last_predicate = metadata.get_mark(mark) <= k.context.current_mark(mark);
            valid &= report.class_predicate(labels::TOPOLOGY_TYPE, test, object, last_predicate)?;
        }
    } else {
        last_predicate = !linked;
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 5, object, last_predicate)?;
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 6, object, last_predicate)?;
    }
    // Source bOk deliberately carries the previous mark3/last-pointer predicate
    // into the ring check. Preserve its cascading test7 instead of resetting it.
    valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 7, object, last_predicate)?;
    valid &= report.class_predicate(
        labels::TOPOLOGY_TYPE,
        8,
        object,
        !owner || owner_count == actual_count,
    )?;
    Ok(valid)
}

fn child_head(b: &Brep, object: Object) -> Result<(Option<Object>, usize, bool)> {
    Ok(match object {
        Object::Region(id) => {
            let list = &b.region(id)?.shells;
            (list.first().copied().map(Object::Shell), list.len(), true)
        }
        Object::Shell(id) => {
            let shell = b.shell(id)?;
            match &shell.members {
                ShellMembers::Empty => (None, 0, shell.is_faceuse_shell()),
                ShellMembers::Faceuses(ids) => (
                    ids.first().copied().map(Object::Faceuse),
                    ids.len(),
                    shell.is_faceuse_shell(),
                ),
                // SmShell.h: count the head edge's radial list, not the whole
                // connected network. Reuse the model's native count adapter.
                // Neither wire nor vertex shells have typical owner-use lists.
                ShellMembers::Edgeuse(first) => (
                    Some(Object::Edgeuse(*first)),
                    crate::topology::shell::shell_list_size(b, id)?,
                    false,
                ),
                ShellMembers::Vertexuse(id) => (Some(Object::Vertexuse(*id)), 1, false),
            }
        }
        _ => return Err(invalid()),
    })
}

fn neighbors<T: Copy + PartialEq>(list: &[T], id: T) -> Result<(Option<T>, Option<T>)> {
    let Some(i) = list.iter().position(|entry| *entry == id) else {
        return Ok((None, None));
    };
    if list.iter().filter(|entry| **entry == id).count() != 1 {
        return Err(invalid());
    }
    Ok((
        list.get((i + 1) % list.len()).copied(),
        list.get((i + list.len() - 1) % list.len()).copied(),
    ))
}

fn child_links(b: &Brep, object: Object) -> Result<(Option<Object>, Option<Object>)> {
    Ok(match object {
        Object::Shell(id) => {
            let mut list = None;
            for (_, region) in b.regions().iter() {
                if region.shells.contains(&id) {
                    if list.is_some() {
                        return Err(invalid());
                    }
                    list = Some(region.shells.as_slice());
                }
            }
            let (next, last) = neighbors(list.unwrap_or_default(), id)?;
            (next.map(Object::Shell), last.map(Object::Shell))
        }
        Object::Faceuse(id) => {
            let (next, last) = crate::topology::shell::faceuse_sibling_links(b, id)?;
            (next.map(Object::Faceuse), last.map(Object::Faceuse))
        }
        Object::Edgeuse(id) => {
            let owner = b.edgeuse(id)?.edge.ok_or_else(invalid)?;
            let (next, last) = neighbors(&b.edge(owner)?.edgeuses, id)?;
            (next.map(Object::Edgeuse), last.map(Object::Edgeuse))
        }
        Object::Vertexuse(id) => {
            let owner = b.vertexuse(id)?.vertex.ok_or_else(invalid)?;
            let (next, last) = neighbors(&b.vertex(owner)?.vertexuses, id)?;
            (next.map(Object::Vertexuse), last.map(Object::Vertexuse))
        }
        _ => return Err(invalid()),
    })
}

/// The non-Given base chain, preserving static class report identities.
pub(super) fn owning(
    k: &Kernel,
    b: &Brep,
    object: Object,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = topology_base(k, b, object, report)?;
    let (head, declared, typical) = child_head(b, object)?;
    let Some(head) = head else {
        valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 5, object, declared == 0)?;
        return Ok(valid);
    };
    let mut child = head;
    // Native enters once even for a non-null head with declared count zero.
    // Subsequent visits stop at head or declared count, bounding corrupt rings.
    for i in 0..declared.max(1) {
        if i != 0 && child.key() == head.key() {
            break;
        }
        let (next, _) = child_links(b, child)?;
        let last_of_next = next
            .map(|next| child_links(b, next))
            .transpose()?
            .and_then(|(_, last)| last);
        let (Some(next), Some(last)) = (next, last_of_next) else {
            valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 0, object, false)?;
            break;
        };
        valid &= report.class_predicate(
            labels::OWNING_TOPOLOGY_TYPE,
            1,
            object,
            last.key() == child.key(),
        )?;
        if typical {
            let same_owner = match (object, child) {
                (Object::Region(r), Object::Shell(s)) => b.shell(s)?.region == Some(r),
                (Object::Shell(s), Object::Faceuse(fu)) => b.faceuse(fu)?.shell == Some(s),
                _ => return Err(invalid()),
            };
            valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 2, object, same_owner)?;
        }
        if declared.checked_sub(1) == Some(i) {
            valid &= report.class_predicate(
                labels::OWNING_TOPOLOGY_TYPE,
                3,
                object,
                next.key() == head.key(),
            )?;
        }
        valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 4, object, true)?;
        child = next;
    }
    Ok(valid)
}
