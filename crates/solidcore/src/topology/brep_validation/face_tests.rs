// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual geometric predicates used by maintained Face assertions.
//!
//! Upstream: `SmFace.cpp` tests32--39 (19783--20309), coincidence helpers
//! (16498--16795), `HasSeamProblem` early branches (15629--15715), `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | `coincident_vertices`, `coincident_edges`, `partially_coincident_edges` | tests32/33/34 |
//! | `nested_loops`, `disjoint_loops`, `distinct_loop_vertices` | tests37/38/39 |
//! Deviations: only the assertion call conventions are implemented; optional
//! external accumulation APIs are not claimed. Missing failed coincidence outputs
//! and periodic seam classification remain explicit. Sample-based inconclusive
//! source predicates do not establish continuous import acceptance evidence.

use super::{local::ignore_native_status, Brep};
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::point_set::PointContainment;
use crate::geom::solution_array::SolutionType;
use crate::kernel::Kernel;
use crate::math::{Point2, EFF_ZERO_SQ};
use crate::surface::surface::p1::map_to_2d;
use crate::topology::{
    edge,
    edgeuse::{p1 as eu, p3 as eu3},
    face, loopuse, topology_traverser, FaceId, VertexId,
};

fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::face_tests")
}

pub(super) fn coincident_vertices(
    b: &Brep,
    f: FaceId,
    all: bool,
) -> Result<Vec<(VertexId, VertexId)>> {
    let vertices = crate::query::face::p2::face_vertices(b, f, false, None)?;
    let mut pairs = Vec::new();
    for (i, survivor) in vertices.iter().copied().enumerate() {
        if pairs.iter().any(|(_, deleted)| *deleted == survivor) {
            continue;
        }
        for deleted in vertices.iter().skip(i + 1).copied() {
            if b.vertex(survivor)?
                .point
                .distance_squared(b.vertex(deleted)?.point)
                < EFF_ZERO_SQ
            {
                pairs.push((survivor, deleted));
            }
        }
        // Source quit-early applies to the OUTER loop only, retaining all pairs
        // of the first surviving vertex. Do not return inside the inner loop.
        if !all && !pairs.is_empty() {
            break;
        }
    }
    Ok(pairs)
}

fn coincident_group(vertex: VertexId, pairs: &[(VertexId, VertexId)]) -> Vec<VertexId> {
    let mut result = vec![vertex];
    let target = if let Some((survivor, _)) = pairs.iter().find(|(_, deleted)| *deleted == vertex) {
        result.push(*survivor);
        *survivor
    } else {
        vertex
    };
    for (survivor, deleted) in pairs {
        if *survivor == target {
            result.push(*deleted);
        }
    }
    result
}

fn coincidence(
    k: &Kernel,
    b: &Brep,
    a: crate::topology::EdgeId,
    c: crate::topology::EdgeId,
    tol: f64,
) -> Result<crate::curve::curve::p6::Coincidence<EntityRef>> {
    let first = b.edge(a)?;
    let second = b.edge(c)?;
    let outcome = first
        .curve
        .as_ref()
        .ok_or_else(missing)?
        .global_coincidence_checker_with_parameter_intervals(
            k,
            &first.interval,
            second.curve.as_ref().ok_or_else(missing)?,
            &second.interval,
            tol,
            true,
            [Some(EntityRef::Edge(a)), Some(EntityRef::Edge(c))],
            &|solution, parameter| {
                // SmFace.cpp's coincidence assertions pass actual edge objects;
                // SmSolutionArray.cpp::GetParameterInterval reads each edge's
                // full restriction, including when the checker subdivides it.
                // Resolve the same arena borrowed for this validation, without
                // substituting the defining curve's larger natural interval.
                if let Some((object_index, 0)) =
                    solution.object_index_for_parameter_index(parameter)?
                {
                    if let Some(Some(EntityRef::Edge(id))) = solution.objects.get(object_index) {
                        return edge::interval(b, *id).map(Some);
                    }
                }
                solution.parameter_interval(parameter)
            },
        );
    match outcome {
        Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => {
            // UNPORTED(fork.topology_validation): native callers inspect partial
            // outputs even after failure; current Result API cannot expose them.
            Err(SolidError::unported(
                "SmCurve::GlobalCoincidenceChecker failed out-parameters",
                "fork.topology_validation",
            ))
        }
        other => other,
    }
}

pub(super) fn coincident_edges(k: &Kernel, b: &Brep, f: FaceId) -> Result<bool> {
    let edges = topology_traverser::face_edges(b, f)?;
    let pairs = coincident_vertices(b, f, true)?;
    let mut found = false;
    let mut deleted = Vec::new();
    for (i, survivor) in edges.iter().copied().enumerate() {
        if deleted.contains(&survivor) {
            continue;
        }
        let start = coincident_group(
            edge::start_vertex(b, survivor)?.ok_or_else(missing)?,
            &pairs,
        );
        let end = coincident_group(edge::end_vertex(b, survivor)?.ok_or_else(missing)?, &pairs);
        for other in edges.iter().skip(i + 1).copied() {
            let v0 = edge::start_vertex(b, other)?.ok_or_else(missing)?;
            let v1 = edge::end_vertex(b, other)?.ok_or_else(missing)?;
            if (start.contains(&v0) && end.contains(&v1))
                || (end.contains(&v0) && start.contains(&v1))
            {
                let tol = crate::geom::tol::xsect_tol_3d_from_zones(
                    k,
                    b.edge(survivor)?.tolerance,
                    b.edge(other)?.tolerance,
                );
                let checked = coincidence(k, b, survivor, other, tol)?;
                let ranges = checked
                    .solutions
                    .iter()
                    .all(|s| s.solution_type == SolutionType::RangeOfValues);
                if !checked.additional_work_needed
                    || ((checked.solutions.len() == 1 || checked.solutions.len() == 2) && ranges)
                {
                    found = true;
                    deleted.push(other);
                }
            }
        }
        if found {
            break;
        }
    }
    Ok(found)
}

pub(super) fn partially_coincident_edges(k: &Kernel, b: &Brep, f: FaceId) -> Result<bool> {
    let edges = topology_traverser::face_edges(b, f)?;
    for (i, first) in edges.iter().copied().enumerate() {
        for second in edges.iter().skip(i + 1).copied() {
            let a = b.edge(first)?.tolerance;
            let c = b.edge(second)?.tolerance;
            let checked = coincidence(k, b, first, second, if a > c { a } else { c })?;
            if checked
                .solutions
                .iter()
                .any(|s| s.solution_type == SolutionType::RangeOfValues)
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

pub(super) fn nested_loops(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<bool> {
    let loops = face::p2::loops(b, f, false, None)?;
    if loops.len() < 2 {
        return Ok(true);
    }
    let first = *loops.first().ok_or_else(missing)?;
    let tol = b.face(f)?.tolerance;
    for (i, inner) in loops.iter().copied().enumerate().skip(1) {
        let mut sample = None;
        for edgeuse in loopuse::loop_edgeuses(b, inner)? {
            if let Some(point) = ignore_native_status(eu3::normalized_evaluate(
                k, b, edgeuse, 0.5, true, false, true,
            ))? {
                sample = Some(Point2::new(point.point.x, point.point.y));
                break;
            }
        }
        let Some(sample) = sample else {
            continue;
        }; // Native inconclusive, not proof.
        let surface = b.face(f)?.surface.as_ref().ok_or_else(missing)?;
        let uv_tol = map_to_2d(k, tol, sample, surface)?;
        let containment = ignore_native_status(crate::query::r#loop::contains_uv_point(
            k, b, first, None, sample, uv_tol,
        ))?;
        if containment.is_some_and(|c| {
            matches!(
                c.containment,
                PointContainment::Outside | PointContainment::OnBoundary
            )
        }) {
            return Ok(false);
        }
        for (j, other) in loops.iter().copied().enumerate().skip(1) {
            if i == j {
                continue;
            }
            if ignore_native_status(crate::query::r#loop::contains_uv_point(
                k, b, other, None, sample, uv_tol,
            ))?
            .is_some_and(|c| c.containment == PointContainment::Inside)
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

pub(super) fn disjoint_loops(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<bool> {
    let loops = face::p2::loops(b, f, false, None)?;
    if loops.len() < 2 {
        return Ok(true);
    }
    for (i, first) in loops.iter().copied().enumerate() {
        for second in loops.iter().skip(i + 1).copied() {
            let first_uses = loopuse::loop_edgeuses(b, first)?;
            let second_uses = loopuse::loop_edgeuses(b, second)?;
            let surface = b.face(f)?.surface.as_ref().ok_or_else(missing)?;
            let tol = crate::topology::surface_cache::working_surface_tolerance(k, b, surface)?;
            let uv_tol = map_to_2d(k, tol, surface.natural_uv_domain().mid(), surface)?;
            for u0 in first_uses.iter().copied() {
                let fetched = eu3::get_or_create_uv_trim_curve(k, b, u0, false, false, true)?;
                if ignore_native_status(fetched.status)?.is_none()
                    || eu::uv_trim_curve(b, u0)?.0.is_none()
                {
                    continue;
                }
                let interval0 = eu::uv_trim_curve(b, u0)?
                    .0
                    .ok_or_else(missing)?
                    .natural_interval();
                for u1 in second_uses.iter().copied() {
                    let fetched = eu3::get_or_create_uv_trim_curve(k, b, u1, false, false, true)?;
                    if ignore_native_status(fetched.status)?.is_none()
                        || eu::uv_trim_curve(b, u1)?.0.is_none()
                    {
                        continue;
                    }
                    // Reborrow both after the mutable fetch. Shared physical UV
                    // curves remain the SAME object for self-intersection dispatch.
                    let c0 = eu::uv_trim_curve(b, u0)?.0.ok_or_else(missing)?;
                    let c1 = eu::uv_trim_curve(b, u1)?.0.ok_or_else(missing)?;
                    let solutions = ignore_native_status(c0.global_curve_intersect::<EntityRef>(
                        k,
                        &interval0,
                        c1,
                        &c1.natural_interval(),
                        uv_tol,
                        &crate::curve::curve::p8::CurveIntersectOptions::default(),
                    ))?;
                    if solutions.is_some_and(|s| !s.is_empty()) {
                        return Ok(false);
                    }
                }
            }
        }
    }
    Ok(true)
}

pub(super) fn distinct_loop_vertices(b: &Brep, f: FaceId) -> Result<bool> {
    let loops = face::p2::loops(b, f, false, None)?;
    for (i, first) in loops.iter().copied().enumerate() {
        let vertices = loopuse::loop_vertices(b, first)?;
        for other in loops.iter().skip(i + 1).copied() {
            if loopuse::loop_vertices(b, other)?
                .iter()
                .any(|v| vertices.contains(v))
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

pub(super) fn seam_problem(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<face::p1::SeamProblem> {
    face::p4::owning_seam_problem(k, b, f)
}
