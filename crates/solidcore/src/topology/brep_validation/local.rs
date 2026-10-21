// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual Region and Shell local assertions, including native connectivity walks.
//!
//! Upstream: `SmRegion.cpp::AssertValid` (441--478), `SmShell.cpp`
//! (1586--1846), maintained source `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_region`] | `SmRegion::AssertValid` |
//! | [`assert_shell`] | `SmShell::AssertValid` |
//! Deviations: checked ids replace unsafe casts/dereferences; missing non-root
//! GetSubTopology remains explicit. Shared traversal algorithms retain their
//! source order and bound input-driven walks. No geometry is approximated.

use super::{local_base, AssertValidOptions, Brep, CheckLevel, Object, Reporter};
use crate::error::{Result, SolidError};
use crate::geom::assert_array::AssertArray;
use crate::geom::context::{with_mark_lock, MarkLock, ALL_MARKS};
use crate::kernel::Kernel;
use crate::topology::edgeuse::p1 as eu;
use crate::topology::shell::{self, ShellMembers, ShellType};
use crate::topology::topology_traverser::{self as traverser, Marks};
use crate::topology::{
    vertex, vertexuse, EdgeuseOwner, LoopuseStart, RegionId, ShellId, VertexuseOwner,
};

fn required<T>(value: Option<T>) -> Result<T> {
    value.ok_or_else(|| SolidError::null_pointer("topology::brep_validation::local"))
}

pub(super) fn ignore_native_status<T>(result: Result<T>) -> Result<Option<T>> {
    match result {
        Ok(value) => Ok(Some(value)),
        // Native callers ignore SmStatus; Rust arena/representation failures
        // have no safe native status counterpart and must not become success.
        Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

pub(super) fn root_shells(k: &mut Kernel, b: &mut Brep) -> Result<Vec<ShellId>> {
    let mut marks = Marks::from_brep(b)?;
    let result = super::bp::shells_with_marks(k, b, None, &mut marks);
    marks.store_in_brep(b)?;
    result
}

/// Root branch of `GetSubTopology`, `SmTopologyTraverser.cpp`1720--1790: use
/// actual stored root lists and face-derived loops before validating ANY child.
/// Missing local/geometry calls stop the walk explicitly, never count as valid.
pub(super) fn walk_root(
    k: &mut Kernel,
    b: &mut Brep,
    level: CheckLevel<'_>,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let regions = super::bp::regions(b, None)?;
    let shells = root_shells(k, b)?;
    let faces = super::bp::faces(b, None)?;
    let mut loops = Vec::new();
    for face in &faces {
        loops.extend(crate::topology::face::p2::loops(b, *face, false, None)?);
    }
    let edges = super::bp::edges(b, None)?;
    let vertices = super::bp::vertices(b, None)?;
    // Native passes the level but no root request array to child AssertValid.
    let level = if matches!(level, CheckLevel::Given(_)) {
        CheckLevel::Given(&[])
    } else {
        level
    };
    let options = AssertValidOptions { level, walk: false };
    let mut valid = true;
    for region in regions {
        valid &= assert_region(k, b, region, options, report.reports.as_deref_mut())?;
    }
    for shell in shells {
        valid &= assert_shell(k, b, shell, options, report.reports.as_deref_mut())?;
    }
    for face in faces {
        valid &= super::assert_face(k, b, face, options, report.reports.as_deref_mut())?;
    }
    for loop_ in loops {
        valid &= super::assert_loop(k, b, loop_, options, report.reports.as_deref_mut())?;
    }
    for edge in edges {
        valid &= super::assert_edge(k, b, edge, options, report.reports.as_deref_mut())?;
    }
    for vertex in vertices {
        valid &= super::assert_vertex(k, b, vertex, options, report.reports.as_deref_mut())?;
    }
    Ok(valid)
}

pub(super) fn non_root_walk() -> SolidError {
    // UNPORTED(topology.topology_traverser): non-root GetSubTopology; do not
    // replace its source mark/order semantics with an arbitrary graph scan.
    SolidError::unported(
        "SmTopologyTraverser::GetSubTopology (non-root)",
        "topology.topology_traverser",
    )
}

/// Run the native Region base chain. The source has no additional Region-local
/// predicates; walk requests require its descendant traversal, never a fake pass.
pub fn assert_region(
    k: &mut Kernel,
    b: &mut Brep,
    region: RegionId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.region(region)?;
    let valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::owning(k, b, Object::Region(region), &mut Reporter { reports })?
    };
    if options.walk {
        return Err(non_root_walk());
    }
    Ok(valid)
}

/// Execute native Shell assertions over actual model membership and connectivity.
/// Retains mark effects even on ordinary failure/error; no repair is authorized.
pub fn assert_shell(
    k: &mut Kernel,
    b: &mut Brep,
    shell: ShellId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.shell(shell)?;
    let mut report = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::owning(k, b, Object::Shell(shell), &mut report)?
    };
    let mut marks = Marks::from_brep(b)?;
    let result = shell_checks(k, b, shell, &mut marks, &mut report);
    // Source writes native marks in place. Restore partial traversal effects
    // before propagating errors, without touching ownership or geometry slots.
    marks.store_in_brep(b)?;
    let (local_valid, early_return) = result?;
    valid &= local_valid;
    if !early_return && options.walk {
        return Err(non_root_walk());
    }
    Ok(valid)
}

fn pred(report: &mut Reporter<'_>, shell: ShellId, test: u64, passed: bool) -> Result<bool> {
    report.class_predicate(16013, test, Object::Shell(shell), passed)
}

fn shell_vertices(b: &Brep, shell: ShellId) -> Result<Vec<crate::topology::VertexId>> {
    required(shell::shell_brep(b, shell)?)?;
    let mut out = Vec::new();
    for id in super::bp::vertices(b, None)? {
        if vertex::is_shell_vertex(b, id)? {
            let vu = required(vertex::primary_vertexuse(b, id)?)?;
            if vertexuse::shell(b, vu)? == Some(shell) {
                out.push(id);
            }
        }
    }
    Ok(out)
}

fn shell_faceuses(b: &Brep, shell: ShellId) -> Result<Vec<crate::topology::FaceuseId>> {
    required(shell::shell_brep(b, shell)?)?;
    let mut out = Vec::new();
    for id in super::bp::faces(b, None)? {
        let (first, second) = crate::topology::face::p2::faceuses(b, id)?;
        for fu in [first, second].into_iter().flatten() {
            if b.faceuse(fu)?.shell == Some(shell) {
                out.push(fu);
            }
        }
    }
    Ok(out)
}

fn connected_edgeuses(
    k: &mut Kernel,
    b: &Brep,
    marks: &mut Marks,
    start: crate::topology::EdgeuseId,
) -> Result<Vec<crate::topology::EdgeuseId>> {
    // SmEdgeuse.cpp5434--5468: loop uses follow their loop; wires collect edges
    // with the supplied NEW mark, then append each edge's complete radial list.
    if let EdgeuseOwner::Loopuse(owner) = b.edgeuse(start)?.owner {
        return eu::loopuse_edgeuses(b, required(owner)?);
    }
    with_mark_lock(k, ALL_MARKS, |k, mark| {
        let mut edges = Vec::new();
        // The Shell caller ignores GetConnectedEdgeuses' ordinary status. Its
        // callee still appends collected edges before returning that status.
        ignore_native_status(traverser::collect_wire_edges(
            k,
            b,
            marks,
            b.edgeuse(start)?.edge,
            mark,
            &mut edges,
            None,
        ))?;
        let mut out = Vec::new();
        for edge in edges {
            out.extend(b.edge(edge)?.edgeuses.iter().copied());
        }
        Ok(out)
    })?
}

#[allow(clippy::too_many_lines)]
fn shell_checks(
    k: &mut Kernel,
    b: &Brep,
    id: ShellId,
    marks: &mut Marks,
    report: &mut Reporter<'_>,
) -> Result<(bool, bool)> {
    let shell = b.shell(id)?;
    let has_head = match &shell.members {
        ShellMembers::Empty => false,
        ShellMembers::Faceuses(list) => !list.is_empty(),
        _ => true,
    };
    let mut valid = pred(report, id, 0, has_head)?;
    if !has_head {
        return Ok((false, true));
    }
    // The three independent collections are intentional: scanning owner
    // backpointers must not be replaced with the shell's own declared list.
    let vertices = shell_vertices(b, id)?;
    let wires = shell::wire_edges_with_marks(k, b, id, marks)?;
    let faces = shell_faceuses(b, id)?;
    match shell.shell_type {
        ShellType::Vertex => {
            let ShellMembers::Vertexuse(vu) = shell.members else {
                return Err(SolidError::null_pointer("vertex shell head"));
            };
            valid &= pred(
                report,
                id,
                1,
                vertices.len() == 1 && vertices.first().copied() == shell.vertex(b)?,
            )?;
            valid &= pred(report, id, 2, wires.is_empty())?;
            valid &= pred(report, id, 3, faces.is_empty())?;
            b.vertexuse(vu)?;
            valid &= pred(report, id, 4, true)?;
            valid &= pred(report, id, 5, vertexuse::is_shell_vertexuse(b, vu)?)?;
            valid &= pred(
                report,
                id,
                6,
                b.vertexuse(vu)?.owner == VertexuseOwner::Shell(Some(id)),
            )?;
        }
        ShellType::Wire => {
            let ShellMembers::Edgeuse(start) = shell.members else {
                return Err(SolidError::null_pointer("wire shell head"));
            };
            b.edgeuse(start)?;
            valid &= pred(report, id, 7, true)?;
            valid &= pred(
                report,
                id,
                8,
                matches!(b.edgeuse(start)?.owner, EdgeuseOwner::Shell(_)),
            )?;
            valid &= pred(
                report,
                id,
                9,
                b.edgeuse(start)?.owner == EdgeuseOwner::Shell(Some(id)),
            )?;
            let first = connected_edgeuses(k, b, marks, start)?;
            valid &= pred(report, id, 10, vertices.is_empty())?;
            valid &= pred(report, id, 11, faces.is_empty())?;
            let mut second = Vec::new();
            for wire in wires {
                second.extend(b.edge(wire)?.edgeuses.iter().copied());
            }
            valid &= pred(report, id, 12, first.len() == second.len())?;
            for (first, second) in first.iter().copied().zip(second.iter().copied()) {
                b.edgeuse(first)?;
                b.edgeuse(second)?;
                valid &= pred(report, id, 13, true)?;
                valid &= pred(report, id, 14, true)?;
                valid &= pred(
                    report,
                    id,
                    15,
                    matches!(b.edgeuse(first)?.owner, EdgeuseOwner::Shell(_)),
                )?;
                valid &= pred(
                    report,
                    id,
                    16,
                    b.edgeuse(first)?.owner == EdgeuseOwner::Shell(Some(id)),
                )?;
                valid &= pred(
                    report,
                    id,
                    17,
                    matches!(b.edgeuse(second)?.owner, EdgeuseOwner::Shell(_)),
                )?;
                valid &= pred(
                    report,
                    id,
                    18,
                    b.edgeuse(second)?.owner == EdgeuseOwner::Shell(Some(id)),
                )?;
            }
        }
        ShellType::Faceuse => {
            let first = shell.get_faceuses()?;
            let fu = required(first.first().copied())?;
            b.faceuse(fu)?;
            valid &= pred(report, id, 19, true)?;
            valid &= pred(report, id, 20, b.faceuse(fu)?.shell == Some(id))?;
            let Some(lu) = b.faceuse(fu)?.loopuses.first().copied() else {
                valid &= pred(report, id, 34, false)?;
                return Ok((valid, true));
            };
            valid &= pred(
                report,
                id,
                21,
                matches!(b.loopuse(lu)?.start, LoopuseStart::Edgeuse(_)),
            )?;
            let LoopuseStart::Edgeuse(start) = b.loopuse(lu)?.start else {
                return Err(SolidError::null_pointer("face shell loop head"));
            };
            let start = required(start)?;
            let mut second = Vec::new();
            if let Some(vu) = b.edgeuse(start)?.vertexuse {
                let vertex = required(b.vertexuse(vu)?.vertex)?;
                let mut lock = MarkLock::new(std::slice::from_mut(&mut k.context), 0, ALL_MARKS)?;
                let result = traverser::shell_traversal(
                    k,
                    b,
                    marks,
                    vertex,
                    id,
                    None,
                    Some(&mut second),
                    &lock,
                );
                lock.release(std::slice::from_mut(&mut k.context))?;
                // Native Shell ignores this status and inspects partial output.
                // Missing dependencies and unsafe graph access still propagate.
                ignore_native_status(result)?;
            }
            valid &= pred(report, id, 22, first.len() == second.len())?;
            valid &= pred(report, id, 23, faces.len() == second.len())?;
            // Native reuses index24 for both size and context predicates.
            valid &= pred(report, id, 24, vertices.is_empty())?;
            if first.len() != second.len() {
                for (left, right) in [(&first, &second), (&second, &first)] {
                    for fu in left.iter().copied().filter(|fu| !right.contains(fu)) {
                        // Original diagnostic path still calls GetNeighbors even
                        // when debug drawing is disabled. Preserve that access.
                        eu::faceuse_neighbors(b, fu, false)?;
                    }
                }
            }
            for (first, second) in first.iter().copied().zip(second.iter().copied()) {
                valid &= pred(report, id, 24, true)?;
                valid &= pred(report, id, 25, b.faceuse(first)?.shell == Some(id))?;
                valid &= pred(report, id, 26, b.faceuse(second)?.shell == Some(id))?;
            }
        }
        _ => valid = false,
    }
    Ok((valid, false))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::EntityRef;
    use crate::geom::context::MarkType;

    fn no_walk() -> AssertValidOptions<'static> {
        AssertValidOptions {
            level: CheckLevel::Zero,
            walk: false,
        }
    }

    #[test]
    fn given_region_preserves_native_base_skip_without_hiding_it_in_normal_levels() {
        let (mut b, _, _, _) = super::super::tests::fixture();
        let region = b.infinite_region().unwrap();
        b.region_mut(region).unwrap().in_brep = false;
        let mut reports = AssertArray::new();
        assert!(!assert_region(
            &mut Kernel::default(),
            &mut b,
            region,
            no_walk(),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports
                .reports()
                .iter()
                .map(|r| r.test_index)
                .collect::<Vec<_>>(),
            vec![5, 6, 7]
        );
        assert!(assert_region(
            &mut Kernel::default(),
            &mut b,
            region,
            AssertValidOptions {
                level: CheckLevel::Given(&[0]),
                walk: false
            },
            None
        )
        .unwrap());
    }

    #[test]
    fn shell_traversal_keeps_real_marks_and_preserves_other_owned_metadata() {
        let (mut b, face, _, _) = super::super::tests::fixture();
        let fu = b.face(face).unwrap().faceuse.unwrap();
        let shell = b.faceuse(fu).unwrap().shell.unwrap();
        b.notification_topology_mut(EntityRef::Faceuse(fu))
            .unwrap()
            .set_all_flags(37);
        let mut k = Kernel::default();
        assert!(assert_shell(&mut k, &mut b, shell, no_walk(), None).unwrap());
        let key = crate::topology::brep::p2::object_ref(&b, EntityRef::Faceuse(fu))
            .unwrap()
            .key;
        let record = b.notifications().topology.get(&key).unwrap();
        assert_eq!(record.all_flags(), 37);
        assert!(record.is_marked(&k.context, MarkType::Mark));
        let before = k.context.current_mark(MarkType::Mark);
        assert!(assert_shell(&mut k, &mut b, shell, no_walk(), None).unwrap());
        assert!(k.context.current_mark(MarkType::Mark) > before);
    }

    #[test]
    fn missing_loop_head_reports_34_and_returns_before_a_requested_walk() {
        let (mut b, face, _, _) = super::super::tests::fixture();
        let fu = b.face(face).unwrap().faceuse.unwrap();
        let shell = b.faceuse(fu).unwrap().shell.unwrap();
        b.faceuse_mut(fu).unwrap().loopuses.clear();
        let mut reports = AssertArray::new();
        assert!(!assert_shell(
            &mut Kernel::default(),
            &mut b,
            shell,
            AssertValidOptions::default(),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports
                .reports()
                .iter()
                .map(|r| (r.reporting_type, r.test_index))
                .collect::<Vec<_>>(),
            vec![(16013, 34)]
        );
    }

    #[test]
    fn shell_vertex_and_wire_network_checks_use_topology_without_requiring_a_curve() {
        let mut b = Brep::new_with_infinite_region().unwrap();
        let (shell, vertex) =
            crate::topology::brep::p2::make_shell_vertex_topology(&mut b, None).unwrap();
        let mut k = Kernel::default();
        assert!(assert_shell(&mut k, &mut b, shell, no_walk(), None).unwrap());
        let (edge, end) =
            crate::topology::brep::p2::make_edge_vertex_topology(&mut b, None, vertex).unwrap();
        crate::topology::brep::p2::make_edge_vertex_topology(&mut b, None, end).unwrap();
        crate::topology::brep::p2::make_edge_vertex_topology(&mut b, None, vertex).unwrap();
        let mut reports = AssertArray::new();
        assert!(assert_shell(&mut k, &mut b, shell, no_walk(), Some(&mut reports)).unwrap());
        assert!(reports.is_empty());
        assert!(b.edge(edge).unwrap().curve.is_none());
        let key = crate::topology::brep::p2::object_ref(&b, EntityRef::Edge(edge))
            .unwrap()
            .key;
        assert!(b
            .notifications()
            .topology
            .get(&key)
            .unwrap()
            .is_marked(&k.context, MarkType::Mark));
    }

    #[test]
    fn root_walk_keeps_base_request_selection_while_reaching_real_face_checks() {
        for level in [CheckLevel::Zero, CheckLevel::Given(&[8])] {
            let (mut b, _, _, _) = super::super::tests::fixture();
            let region = b.infinite_region().unwrap();
            let mut contexts = [crate::geom::context::Context::default()];
            let mut lock = MarkLock::new(&mut contexts, 0, MarkType::Mark3.code()).unwrap();
            b.notification_topology_mut(EntityRef::Region(region))
                .unwrap()
                .mark(contexts.first(), MarkType::Mark3);
            lock.release(&mut contexts).unwrap();
            let mut reports = AssertArray::new();
            let valid = super::super::assert_valid(
                &mut Kernel::default(),
                &mut b,
                AssertValidOptions { level, walk: true },
                Some(&mut reports),
            )
            .unwrap();
            assert_eq!(valid, !matches!(level, CheckLevel::Zero));
            let actual = reports
                .reports()
                .iter()
                .map(|r| (r.reporting_type, r.test_index))
                .collect::<Vec<_>>();
            if matches!(level, CheckLevel::Zero) {
                assert_eq!(actual, vec![(16001, 4), (16001, 7)]);
            } else {
                assert!(actual.is_empty());
            }
        }
    }

    #[test]
    fn checked_traversal_error_persists_marks_and_releases_context_lock() {
        let (mut b, face, _, _) = super::super::tests::fixture();
        let fu = b.face(face).unwrap().faceuse.unwrap();
        let shell = b.faceuse(fu).unwrap().shell.unwrap();
        let vertex = b.vertex_list()[0];
        b.vertex_mut(vertex)
            .unwrap()
            .vertexuses
            .push(crate::topology::VertexuseId::from_index(u32::MAX));
        let mut k = Kernel::default();
        assert!(assert_shell(&mut k, &mut b, shell, no_walk(), None).is_err());
        // A leaked traversal lock would select a different mark here.
        let mut lock = MarkLock::new(std::slice::from_mut(&mut k.context), 0, ALL_MARKS).unwrap();
        assert_eq!(lock.mark_type(), MarkType::Mark);
        lock.release(std::slice::from_mut(&mut k.context)).unwrap();
        assert!(!b.notifications().topology.is_empty());
    }
}
