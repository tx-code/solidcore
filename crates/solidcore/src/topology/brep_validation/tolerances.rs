// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Gap/tolerance validation with original order, thresholds and side effects.
//!
//! Upstream: `SmBrep.cpp::ValidateAndUpdateTolerances`, `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`validate_and_update_tolerances`] | `SmBrep::ValidateAndUpdateTolerances` |
//! | [`ToleranceReport`] | scalar and optional object out-parameters |
//!
//! Deviations: outputs are owned values; check mode restores the original
//! model flag on every result path. Native unused maxima remain zero/None; the
//! shadowed edge/face-maximum comparison is preserved, not silently corrected.
use super::{Brep, CheckLevel, Object, Reporter};
use crate::error::{ignored, Result, SolidError};
use crate::geom::assert_array::AssertArray;
use crate::geom::context::{with_mark_lock, ALL_MARKS};
use crate::geom::extent3d::Extent3d;
use crate::heal::gap::Gap;
use crate::kernel::Kernel;
use crate::query::brep::{calculate_bounding_box, BoundingBoxOptions};
use crate::topology::{edge, edgeuse::p1 as eu, vertex, vertexuse, EdgeuseId, VertexuseId};
use std::collections::BTreeSet;

const OP: &str = "topology::brep_validation::validate_and_update_tolerances";
/// Original output values. Maxima describe calculated SDK gaps; they are not
/// continuous source-to-target error bounds or import repair permissions.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ToleranceReport {
    /// Source violation/update flag (threshold differs from report thresholds).
    pub violated_or_updated: bool,
    /// Unused native output, initialized to zero.
    pub max_edge_face_trim_curve_gap: f64,
    /// Largest vertex/edge gap, strict comparison retains first tied witness.
    pub max_vertex_edge_gap: f64,
    /// Largest vertex/surface gap.
    pub max_vertex_face_gap: f64,
    /// Original optional output; native shadowed comparison normally leaves zero.
    pub max_edge_face_gap: f64,
    /// Unused native output, initialized to zero.
    pub max_vertex_face_trim_curve_gap: f64,
    /// Original vertex use of the largest vertex/edge gap.
    pub vertex_edge_source: Option<VertexuseId>,
    /// Original vertex use of the largest vertex/face gap.
    pub vertex_face_source: Option<VertexuseId>,
    /// Unused native witness.
    pub edge_face_trim_curve_source: Option<EdgeuseId>,
    /// Native code does not assign this optional witness.
    pub edge_face_source: Option<EdgeuseId>,
    /// Unused native witness.
    pub vertex_face_trim_curve_source: Option<EdgeuseId>,
}

/// Check gaps, or explicitly request the native working-tolerance adjustments.
/// `validate_only=true` does not grant import acceptance and is not guaranteed to
/// leave lazy caches untouched. Working tolerances stay in the model's units.
/// Original level/request input is retained; Release value reports run at level 0.
pub fn validate_and_update_tolerances(
    k: &mut Kernel,
    b: &mut Brep,
    validate_only: bool,
    _level: CheckLevel<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<ToleranceReport> {
    let old = b.validate_only();
    if validate_only {
        b.set_validate_only(true);
    }
    let mut report = Reporter { reports };
    let outcome = (|| -> Result<ToleranceReport> {
        let faces = b.face_list().to_vec();
        // Native ordering is GetFaces, CalculateBoundingBox, then mark locking.
        let bounds = ignored(calculate_bounding_box(k, b, BoundingBoxOptions::default()))?
            .unwrap_or_else(Extent3d::new);
        let size = bounds.size().length();
        with_mark_lock(k, ALL_MARKS, |k, _mark| {
            check(k, b, validate_only, &mut report, &faces, size)
        })?
    })();
    b.set_validate_only(old);
    outcome
}

fn check(
    k: &Kernel,
    b: &mut Brep,
    validate_only: bool,
    report: &mut Reporter<'_>,
    faces: &[crate::topology::FaceId],
    size: f64,
) -> Result<ToleranceReport> {
    let mut out = ToleranceReport::default();
    let mut visited = BTreeSet::new();
    for face in faces {
        let fu = b.face(*face)?.faceuse.ok_or(SolidError::null_pointer(OP))?;
        for id in eu::faceuse_edgeuses(b, fu, false, None)? {
            if visited.contains(&id) {
                continue;
            }
            let mate = eu::mate(b, id)?.ok_or(SolidError::null_pointer(OP))?;
            visited.insert(id);
            visited.insert(mate);
            let start = b
                .edgeuse(id)?
                .vertexuse
                .ok_or(SolidError::null_pointer(OP))?;
            let end = b
                .edgeuse(mate)?
                .vertexuse
                .ok_or(SolidError::null_pointer(OP))?;
            let start_vertex = b
                .vertexuse(start)?
                .vertex
                .ok_or(SolidError::null_pointer(OP))?;
            let end_vertex = b
                .vertexuse(end)?
                .vertex
                .ok_or(SolidError::null_pointer(OP))?;
            // Original getter order matters: each may populate shared caches.
            let start_edge = vertexuse::vertex_edge_gap(k, b, start, false)?.map(Gap::from);
            let start_face = vertexuse::vertex_face_gap(k, b, start, false)?.map(Gap::from);
            let end_edge = vertexuse::vertex_edge_gap(k, b, end, false)?.map(Gap::from);
            let end_face = vertexuse::vertex_face_gap(k, b, end, false)?.map(Gap::from);
            for (v, g1, g2) in [
                (start_vertex, start_edge, start_face),
                (end_vertex, end_edge, end_face),
            ] {
                let tol = b.vertex(v)?.tolerance;
                if g1.is_some_and(|g| g.length() > tol) || g2.is_some_and(|g| g.length() > tol) {
                    out.violated_or_updated = true;
                    if !validate_only {
                        let a = g1.map_or(0., |g| g.length());
                        let other = g2.map_or(0., |g| g.length());
                        let max = if a > other { a } else { other };
                        vertex::set_tolerance(b, v, 2. * max, true, true)?;
                    }
                }
            }
            for (test, v, gap) in [
                (0, start_vertex, start_edge),
                (1, start_vertex, start_face),
                (2, end_vertex, end_edge),
                (3, end_vertex, end_face),
            ] {
                let tol = b.vertex(v)?.tolerance / 2.;
                let passed = gap.is_none_or(|g| g.length() <= tol);
                // Failed predicates imply a non-null gap, so the native value
                // expression's null dereference is never evaluated on None.
                report.list_value(
                    2,
                    test,
                    Object::Edgeuse(id),
                    passed,
                    tol,
                    gap.map_or(0., |g| g.length()),
                )?;
            }
            for gap in [start_edge, end_edge].into_iter().flatten() {
                if gap.length() > out.max_vertex_edge_gap {
                    out.max_vertex_edge_gap = gap.length();
                    out.vertex_edge_source = gap.vertexuse();
                }
            }
            for gap in [start_face, end_face].into_iter().flatten() {
                if gap.length() > out.max_vertex_face_gap {
                    out.max_vertex_face_gap = gap.length();
                    out.vertex_face_source = gap.vertexuse();
                }
            }
            let gap = eu::max_edge_face_gap(k, b, id, false)?;
            let maximum = gap.map_or(0., |g| Gap::from(g).length());
            let edge_id = eu::edge(b, id)?.ok_or(SolidError::null_pointer(OP))?;
            if maximum > b.edge(edge_id)?.tolerance {
                out.violated_or_updated = true;
                if !validate_only && maximum < size / 800. {
                    edge::set_tolerance(b, edge_id, 2. * maximum, true, true)?;
                }
            }
            let tol = b.edge(edge_id)?.tolerance;
            report.list_value(2, 4, Object::Edgeuse(id), maximum <= tol, tol, maximum)?;
            // The C++ local dMaxEdgeFaceGap shadows its intended accumulator.
            // Its self-comparison cannot select a maximum; preserve zero/None.
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Point3, Vec3};
    fn shifted(delta: f64) -> (Brep, crate::topology::VertexId) {
        let (mut b, _, _, _) = super::super::tests::fixture();
        let vertex = b
            .vertex_list()
            .iter()
            .copied()
            .find(|id| b.vertex(*id).unwrap().point == Point3::ZERO)
            .unwrap();
        b.vertex_mut(vertex).unwrap().point = Vec3::new(0., 0., delta);
        (b, vertex)
    }
    #[test]
    fn reports_can_exist_below_the_native_violation_flag_threshold() {
        let delta = 0.75e-5;
        let (mut b, v) = shifted(delta);
        let old = b.vertex(v).unwrap().tolerance;
        let mut reports = AssertArray::new();
        let r = validate_and_update_tolerances(
            &mut Kernel::default(),
            &mut b,
            true,
            CheckLevel::Zero,
            Some(&mut reports),
        )
        .unwrap();
        assert!(!r.violated_or_updated);
        assert!(!reports.is_empty());
        assert_eq!(r.max_vertex_edge_gap, delta);
        assert_eq!(b.vertex(v).unwrap().tolerance, old);
        assert!(!b.validate_only());
    }
    #[test]
    fn explicit_update_changes_working_tolerance_and_restores_prior_flag() {
        let delta = 2e-5;
        let (mut b, v) = shifted(delta);
        b.set_validate_only(true);
        let r = validate_and_update_tolerances(
            &mut Kernel::default(),
            &mut b,
            false,
            CheckLevel::Given(&[]),
            None,
        )
        .unwrap();
        assert!(r.violated_or_updated);
        assert_eq!(b.vertex(v).unwrap().tolerance, delta * 2.);
        assert!(b.validate_only());
    }
    #[test]
    fn check_mode_preserves_native_uv_creation_tolerance_side_effects() {
        let (mut b, vertex) = shifted(0.01);
        let root_tol = b.tolerance();
        let result = validate_and_update_tolerances(
            &mut Kernel::default(),
            &mut b,
            true,
            CheckLevel::Zero,
            None,
        )
        .unwrap();
        // Native CreateUVTrimCurves(TRUE) raises the working vertex tolerance
        // before the root comparison. Check mode does not undo that side effect.
        assert!(!result.violated_or_updated);
        assert_eq!(result.max_vertex_edge_gap, 0.01);
        assert_eq!(result.max_vertex_face_gap, 0.01);
        assert_eq!(b.vertex(vertex).unwrap().tolerance, 0.02);
        assert_eq!(b.tolerance(), root_tol);
        assert!(!b.validate_only());
        assert!(!b.editing_enabled());
    }

    #[test]
    fn unavailable_general_projection_restores_both_model_flags_and_owner() {
        use crate::surface::surface::{Surface, SurfaceKind};
        let (mut b, _) = shifted(0.01);
        let face = b.face_list()[0];
        let original = b.face(face).unwrap().surface.as_ref().unwrap();
        let owner = original.owner();
        let SurfaceKind::Plane(plane) = original.kind() else {
            panic!("fixture plane")
        };
        let mut surface = Surface::from(plane.base_with_nurb().unwrap().clone_exact());
        surface.set_owner(owner);
        b.face_mut(face).unwrap().surface = Some(surface);
        let error = validate_and_update_tolerances(
            &mut Kernel::default(),
            &mut b,
            true,
            CheckLevel::Zero,
            None,
        )
        .unwrap_err();
        // Live owner tolerance is now resolved. The actual next dependency is
        // UV projection; that failure must still restore every temporary flag.
        assert!(
            matches!(
                error,
                SolidError::Unported {
                    upstream: "SmSurface::DropCurve",
                    unit: "intersect.surface_drop_lib_p3"
                }
            ),
            "{error:?}"
        );
        assert!(!b.validate_only());
        assert!(!b.editing_enabled());
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
    }
}
