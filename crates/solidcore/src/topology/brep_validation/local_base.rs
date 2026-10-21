// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Topology base-class checks over the actual arena graph and owning metadata.
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

struct PeerState {
    owner: bool,
    on_owner_list: bool,
    owner_count: usize,
}

fn owner_state(b: &Brep, object: Object) -> Result<PeerState> {
    let (owner, on_owner_list, owner_count) = match object {
        Object::Region(id) => {
            let owner = b.region(id)?.in_brep;
            (
                owner,
                owner && b.region_list().contains(&id),
                b.region_list().len(),
            )
        }
        Object::Shell(id) => {
            let owner = b.shell(id)?.region.map(|r| b.region(r)).transpose()?;
            (
                owner.is_some(),
                owner.is_some_and(|r| r.shells.contains(&id)),
                owner.map_or(0, |r| r.shells.len()),
            )
        }
        Object::Face(id) => {
            b.face(id)?;
            let linked = b.face_list().contains(&id);
            // The model's face-list sentinel is intrinsic, distinct from Face.shell.
            (linked, linked, b.face_list().len())
        }
        Object::Edge(id) => {
            b.edge(id)?;
            let linked = b.edge_list().contains(&id);
            (linked, linked, b.edge_list().len())
        }
        Object::Vertex(id) => {
            b.vertex(id)?;
            let linked = b.vertex_list().contains(&id);
            (linked, linked, b.vertex_list().len())
        }
        Object::Edgeuse(id) => {
            let owner = b.edgeuse(id)?.edge.map(|edge| b.edge(edge)).transpose()?;
            (
                owner.is_some(),
                owner.is_some_and(|edge| edge.edgeuses.contains(&id)),
                owner.map_or(0, |edge| edge.edgeuses.len()),
            )
        }
        Object::Vertexuse(id) => {
            let owner = b
                .vertexuse(id)?
                .vertex
                .map(|vertex| b.vertex(vertex))
                .transpose()?;
            (
                owner.is_some(),
                owner.is_some_and(|vertex| vertex.vertexuses.contains(&id)),
                owner.map_or(0, |vertex| vertex.vertexuses.len()),
            )
        }
        Object::Loop(id) => {
            b.loop_(id)?;
            // SmLoop has no intrusive owner/list: its paired loopuses own their
            // separate links; Loop.face is only a convenience cache.
            (false, false, 0)
        }
        Object::Faceuse(id) => {
            let owner = b.faceuse(id)?.shell;
            let on_list = owner.map(|s| -> Result<bool> {
                Ok(matches!(&b.shell(s)?.members, ShellMembers::Faceuses(list) if list.contains(&id)))
            }).transpose()?.unwrap_or(false);
            (
                owner.is_some(),
                on_list,
                owner
                    .map(|s| crate::topology::shell::shell_list_size(b, s))
                    .transpose()?
                    .unwrap_or(0),
            )
        }
        Object::Loopuse(id) => {
            let owner = b.loopuse(id)?.faceuse.map(|fu| b.faceuse(fu)).transpose()?;
            (
                owner.is_some(),
                owner.is_some_and(|fu| fu.loopuses.contains(&id)),
                owner.map_or(0, |fu| fu.loopuses.len()),
            )
        }
        _ => return Err(invalid()),
    };
    Ok(PeerState {
        owner,
        on_owner_list,
        owner_count,
    })
}

fn containing_list<'a, T: Copy + PartialEq + 'a>(
    lists: impl Iterator<Item = &'a [T]>,
    id: T,
) -> Result<&'a [T]> {
    let mut found = None;
    for list in lists {
        let count = list.iter().filter(|entry| **entry == id).count();
        if count > 1 || (count == 1 && found.is_some()) {
            return Err(invalid());
        }
        if count == 1 {
            found = Some(list);
        }
    }
    Ok(found.unwrap_or_default())
}

/// Common topology base. Literal FU links remain observable; ordinary vector
/// reciprocal/count invariants are intrinsic. Marks come from real owned state.
pub(super) fn topology_base(
    k: &Kernel,
    b: &Brep,
    object: Object,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = attribute_base(b, object, report)?;
    let state = owner_state(b, object)?;
    let (next, last) = child_links(b, object)?;
    let mut ring_ok;
    if state.owner {
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 0, object, state.on_owner_list)?;
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 1, object, true)?;
        let metadata = b
            .notifications()
            .topology
            .get(&crate::geom::attribute::ObjectKey(object.key().0))
            .copied()
            .unwrap_or_default();
        ring_ok = true;
        for (test, mark) in [
            (2, MarkType::Mark),
            (3, MarkType::Mark2),
            (4, MarkType::Mark3),
        ] {
            ring_ok = metadata.get_mark(mark) <= k.context.current_mark(mark);
            valid &= report.class_predicate(labels::TOPOLOGY_TYPE, test, object, ring_ok)?;
        }
    } else {
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 5, object, next.is_none())?;
        ring_ok = last.is_none();
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 6, object, ring_ok)?;
    }
    // SmTopology.cpp's exact two-speed ring walk. bOk carries the last mark3/
    // last-pointer result. Do not reset it or silently ignore literal FU damage.
    let mut current = next;
    let mut slow = current;
    let mut move_slow = false;
    let mut count = 1;
    while let Some(node) = current {
        if node.key() == object.key() {
            break;
        }
        count += 1;
        let (next, last) = child_links(b, node)?;
        let (Some(next), Some(last)) = (next, last) else {
            return Err(invalid());
        };
        ring_ok &= child_links(b, next)?.1.map(Object::key) == Some(node.key());
        ring_ok &= child_links(b, last)?.0.map(Object::key) == Some(node.key());
        current = Some(next);
        if move_slow {
            slow = slow
                .map(|node| child_links(b, node))
                .transpose()?
                .and_then(|links| links.0);
        }
        move_slow = !move_slow;
        if slow.map(Object::key) == current.map(Object::key) {
            ring_ok = false;
            break;
        }
    }
    valid &= report.class_predicate(labels::TOPOLOGY_TYPE, 7, object, ring_ok)?;
    valid &= report.class_predicate(
        labels::TOPOLOGY_TYPE,
        8,
        object,
        !state.owner || state.owner_count == count,
    )?;
    Ok(valid)
}

fn child_head(b: &Brep, object: Object) -> Result<(Option<Object>, usize, bool)> {
    Ok(match object {
        Object::Region(id) => {
            let list = &b.region(id)?.shells;
            (list.first().copied().map(Object::Shell), list.len(), true)
        }
        Object::Faceuse(id) => {
            let list = &b.faceuse(id)?.loopuses;
            (list.first().copied().map(Object::Loopuse), list.len(), true)
        }
        Object::Edge(id) => {
            let list = &b.edge(id)?.edgeuses;
            (list.first().copied().map(Object::Edgeuse), list.len(), true)
        }
        Object::Vertex(id) => {
            let list = &b.vertex(id)?.vertexuses;
            (
                list.first().copied().map(Object::Vertexuse),
                list.len(),
                true,
            )
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
        Object::Region(id) => {
            b.region(id)?;
            let (next, last) = neighbors(b.region_list(), id)?;
            (next.map(Object::Region), last.map(Object::Region))
        }
        Object::Shell(id) => {
            b.shell(id)?;
            let list = containing_list(b.regions().iter().map(|(_, r)| r.shells.as_slice()), id)?;
            let (next, last) = neighbors(list, id)?;
            (next.map(Object::Shell), last.map(Object::Shell))
        }
        Object::Face(id) => {
            b.face(id)?;
            let (next, last) = neighbors(b.face_list(), id)?;
            (next.map(Object::Face), last.map(Object::Face))
        }
        Object::Edge(id) => {
            b.edge(id)?;
            let (next, last) = neighbors(b.edge_list(), id)?;
            (next.map(Object::Edge), last.map(Object::Edge))
        }
        Object::Vertex(id) => {
            b.vertex(id)?;
            let (next, last) = neighbors(b.vertex_list(), id)?;
            (next.map(Object::Vertex), last.map(Object::Vertex))
        }
        Object::Loop(id) => {
            b.loop_(id)?;
            (None, None)
        }
        Object::Faceuse(id) => {
            if let Some(links) = b.faceuse(id)?.sibling_links {
                (links.0.map(Object::Faceuse), links.1.map(Object::Faceuse))
            } else {
                let list = containing_list(
                    b.shells().iter().filter_map(|(_, s)| match &s.members {
                        ShellMembers::Faceuses(ids) => Some(ids.as_slice()),
                        _ => None,
                    }),
                    id,
                )?;
                let (next, last) = neighbors(list, id)?;
                (next.map(Object::Faceuse), last.map(Object::Faceuse))
            }
        }
        Object::Loopuse(id) => {
            b.loopuse(id)?;
            let list = containing_list(
                b.faceuses().iter().map(|(_, fu)| fu.loopuses.as_slice()),
                id,
            )?;
            let (next, last) = neighbors(list, id)?;
            (next.map(Object::Loopuse), last.map(Object::Loopuse))
        }
        Object::Edgeuse(id) => {
            b.edgeuse(id)?;
            let list = containing_list(
                b.edges().iter().map(|(_, edge)| edge.edgeuses.as_slice()),
                id,
            )?;
            let (next, last) = neighbors(list, id)?;
            (next.map(Object::Edgeuse), last.map(Object::Edgeuse))
        }
        Object::Vertexuse(id) => {
            b.vertexuse(id)?;
            let list = containing_list(
                b.vertices()
                    .iter()
                    .map(|(_, vertex)| vertex.vertexuses.as_slice()),
                id,
            )?;
            let (next, last) = neighbors(list, id)?;
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
                (Object::Faceuse(fu), Object::Loopuse(lu)) => b.loopuse(lu)?.faceuse == Some(fu),
                (Object::Edge(edge), Object::Edgeuse(eu)) => b.edgeuse(eu)?.edge == Some(edge),
                (Object::Vertex(vertex), Object::Vertexuse(vu)) => {
                    b.vertexuse(vu)?.vertex == Some(vertex)
                }
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
