// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Viewer queries against the same owning native candidate, never mesh picks.
//!
//! Contract: this crate's README, Ownership and responsibilities. Native queries
//! can update derived caches and invalidate earlier numerical observations;
//! source definitions, topology correspondence and import qualification stay put.

use crate::{Candidate, Error, Result};
use solidcore::{math::Vec3, EntityRef};

/// Closest actual native boundary hit, in the candidate's millimeter coordinates.
/// IDs are valid only while this candidate lives and are not source record IDs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NativeRayHit {
    /// Actual mapped native face, edge or vertex, including recorded chart cuts.
    pub entity: EntityRef,
    /// Point on the forward ray in millimeters.
    pub point_mm: Vec3,
    /// Unsquared forward distance from the origin in millimeters.
    pub distance_mm: f64,
}

impl Candidate {
    /// Query this model's actual trimmed B-rep with its executing kernel/cache.
    /// Direction need not be normalized. Explicit tolerance and an optional
    /// maximum distance are in millimeters and must be positive and finite.
    /// A query does not establish accepted import or continuous qualification.
    ///
    /// # Errors
    /// Invalid inputs, unmapped native results or an actual kernel/dependency
    /// failure. `Ok(None)` means a completed native query found no forward hit.
    /// Before native execution, final general trim/material observations are
    /// invalidated even if the native operation later fails; malformed input
    /// rejected before execution leaves those observations intact.
    pub fn ray_hit(
        &mut self,
        origin_mm: Vec3,
        direction: Vec3,
        tolerance_mm: f64,
        maximum_distance_mm: Option<f64>,
    ) -> Result<Option<NativeRayHit>> {
        let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if !finite(origin_mm)
            || !finite(direction)
            || direction == Vec3::ZERO
            || !tolerance_mm.is_finite()
            || tolerance_mm <= 0.
            || maximum_distance_mm.is_some_and(|limit| !limit.is_finite() || limit <= 0.)
        {
            return Err(Error::Invalid("invalid explicit native ray input"));
        }
        #[cfg(feature = "general-development")]
        if let crate::SourcePlan::General(plan) = &mut self.plan {
            // Like classify_face_uv, actual ray leaves may materialize UVs and
            // sector caches on error. Old observations describe the old state.
            plan.invalidate_final_trim_measurements();
        }
        let hit = solidcore::query::api_queries::brep_ray_fire(
            &self.kernel,
            &mut self.brep,
            origin_mm,
            direction,
            tolerance_mm,
            maximum_distance_mm,
        )?;
        let Some(hit) = hit else { return Ok(None) };
        if !self.ray_entity_is_mapped(hit.entity) {
            return Err(Error::Invalid("unmapped native ray-hit entity"));
        }
        Ok(Some(NativeRayHit {
            entity: hit.entity,
            point_mm: hit.point,
            distance_mm: hit.distance,
        }))
    }

    fn ray_entity_is_mapped(&self, entity: EntityRef) -> bool {
        match entity {
            EntityRef::Face(face) => self.faces.values().any(|mapped| *mapped == face),
            EntityRef::Edge(edge) => {
                self.edges.values().flatten().any(|mapped| *mapped == edge)
                    || self.ray_edge_is_chart_cut(edge)
            }
            EntityRef::Vertex(vertex) => {
                self.vertices.values().any(|mapped| *mapped == vertex)
                    || self
                        .derived_vertices()
                        .iter()
                        .any(|derived| derived.native_vertex == vertex)
            }
            _ => false,
        }
    }

    fn ray_edge_is_chart_cut(&self, edge: solidcore::topology::EdgeId) -> bool {
        #[cfg(feature = "general-development")]
        if let Some(report) = self.general_report() {
            return report.chart_cuts.iter().any(|(_, mapped)| *mapped == edge);
        }
        let _ = edge;
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{construct_planar, Options};
    use solidcore_step::{inspect, planar as p, semantics as s, Limits};

    const PLANE: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/plane.step");
    const BOX: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/material-box.step");

    fn planar(bytes: &[u8]) -> Candidate {
        let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
        let source = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                check_part_and_closure: true,
                check_geometry_dependencies: true,
                ..s::Options::default()
            },
        );
        let planar = p::check(&source, p::Quality::planar_fixture(), p::Limits::fixtures());
        construct_planar(
            &planar,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap()
    }

    fn close(actual: Vec3, expected: Vec3) {
        assert!(actual.x.is_finite() && actual.y.is_finite() && actual.z.is_finite());
        assert!(
            (actual - expected).length() <= 1e-7,
            "actual={actual:?}, expected={expected:?}"
        );
    }

    #[test]
    fn actual_trimmed_plane_hit_distance_limit_and_no_hit_are_distinct() {
        // The public rotated rectangle lies at z=5, with centre (3.4,5.2,5).
        // (9,12,5) lies inside its AABB but outside the actual trimmed region.
        let mut model = planar(PLANE);
        let origin = Vec3::new(3.4, 5.2, 20.);
        let hit = model
            .ray_hit(origin, -2. * Vec3::Z, 1e-6, None)
            .unwrap()
            .unwrap();
        assert!(matches!(hit.entity, EntityRef::Face(_)));
        close(hit.point_mm, Vec3::new(3.4, 5.2, 5.));
        assert!((hit.distance_mm - 15.).abs() < 1e-7);
        assert!(model
            .ray_hit(origin, -Vec3::Z, 1e-6, Some(10.))
            .unwrap()
            .is_none());
        assert!(model
            .ray_hit(origin, Vec3::Z, 1e-6, None)
            .unwrap()
            .is_none());
        assert!(model
            .ray_hit(Vec3::new(9., 12., 20.), -Vec3::Z, 1e-6, None)
            .unwrap()
            .is_none());
        assert_eq!(model.source().source().bytes(), PLANE);
    }

    #[test]
    fn nearest_native_face_occludes_the_back_face_in_a_material_box() {
        let mut model = planar(BOX);
        // Original public #100..#107 coordinates independently give
        // x=[2,12], y=[-3,17], z=[5,35], not an origin-centred dimensions box.
        let hit = model
            .ray_hit(Vec3::new(5., 10., 100.), -Vec3::Z, 1e-6, None)
            .unwrap()
            .unwrap();
        close(hit.point_mm, Vec3::new(5., 10., 35.));
        assert!((hit.distance_mm - 65.).abs() < 1e-7);
        assert!(matches!(hit.entity, EntityRef::Face(_)));
    }

    #[test]
    fn same_arena_ids_in_alternating_models_never_share_ray_context_or_cache() {
        let shifted = std::str::from_utf8(PLANE)
            .unwrap()
            .replace(",5.))", ",15.))");
        let mut first = planar(PLANE);
        let mut second = planar(shifted.as_bytes());
        assert_eq!(first.faces, second.faces);
        assert_eq!(first.edges, second.edges);
        let origin = Vec3::new(3.4, 5.2, 30.);
        for _ in 0..3 {
            let a = first
                .ray_hit(origin, -Vec3::Z, 1e-6, None)
                .unwrap()
                .unwrap();
            let b = second
                .ray_hit(origin, -Vec3::Z, 1e-6, None)
                .unwrap()
                .unwrap();
            close(a.point_mm, Vec3::new(3.4, 5.2, 5.));
            close(b.point_mm, Vec3::new(3.4, 5.2, 15.));
            assert!((a.distance_mm - 25.).abs() < 1e-7);
            assert!((b.distance_mm - 15.).abs() < 1e-7);
            assert!(first.brep.notifications().spatial_cache.is_some());
            assert!(second.brep.notifications().spatial_cache.is_some());
        }
        assert_eq!(first.source().source().bytes(), PLANE);
        assert_eq!(second.source().source().bytes(), shifted.as_bytes());
    }

    #[test]
    fn invalid_ray_inputs_and_real_kernel_failure_leave_model_execution_state_usable() {
        let mut model = planar(PLANE);
        let origin = Vec3::new(3.4, 5.2, 20.);
        let bad = [
            (Vec3::new(f64::NAN, 0., 0.), -Vec3::Z, 1e-6, None),
            (origin, Vec3::new(f64::INFINITY, 0., 0.), 1e-6, None),
            (origin, Vec3::ZERO, 1e-6, None),
            (origin, -Vec3::Z, f64::NAN, None),
            (origin, -Vec3::Z, 0., None),
            (origin, -Vec3::Z, -1., None),
            (origin, -Vec3::Z, 1e-6, Some(f64::INFINITY)),
            (origin, -Vec3::Z, 1e-6, Some(0.)),
        ];
        for (origin, direction, tol, limit) in bad {
            assert!(matches!(
                model.ray_hit(origin, direction, tol, limit),
                Err(Error::Invalid(_))
            ));
        }
        assert!(model.brep.notifications().spatial_cache.is_none());
        assert!(model
            .ray_hit(origin, -Vec3::Z, 1e-6, None)
            .unwrap()
            .is_some());
        let cache = model.brep.notifications().spatial_cache.clone();
        let editing = model.brep.editing_enabled();
        let face = *model.faces.values().next().unwrap();
        let surface = model.brep.face_mut(face).unwrap().surface.take();
        assert!(matches!(
            model.ray_hit(origin, -Vec3::Z, 1e-6, None),
            Err(Error::Kernel(_))
        ));
        assert_eq!(model.brep.notifications().spatial_cache, cache);
        assert_eq!(model.brep.editing_enabled(), editing);
        model.brep.face_mut(face).unwrap().surface = surface;
        assert!(model
            .ray_hit(origin, -Vec3::Z, 1e-6, None)
            .unwrap()
            .is_some());
    }

    #[cfg(feature = "general-development")]
    fn general(bytes: &[u8]) -> Candidate {
        let quality = crate::GeneralQuality {
            id: "public-native-ray-controls".into(),
            definition_mm: p::Rational::decimal("0.0000001").unwrap(),
            consistency_mm: p::Rational::decimal("0.000001").unwrap(),
            derived_uv_mm: p::Rational::decimal("0.000001").unwrap(),
            query_mm: p::Rational::decimal("0.000001").unwrap(),
            maximum_work_tolerance_mm: p::Rational::decimal("0.00001").unwrap(),
            maximum_tolerance_growth_mm: p::Rational::ZERO,
        };
        let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
        crate::construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
            quality,
        )
        .unwrap()
    }

    #[cfg(feature = "general-development")]
    #[test]
    fn native_cylinder_and_cone_hits_follow_analytic_definitions() {
        let cylinder = include_bytes!("../tests/fixtures/cylinder.step");
        let cone = include_bytes!("../../solidcore-step/tests/fixtures/cone-pole.step");
        for (bytes, origin, direction, expected) in [
            (
                cylinder.as_slice(),
                Vec3::new(8., 0., 3.),
                -Vec3::X,
                Vec3::new(3., 0., 3.),
            ),
            (
                cone.as_slice(),
                Vec3::new(0., 8., 1.5),
                -Vec3::Y,
                Vec3::new(0., 1.5, 1.5),
            ),
        ] {
            let mut model = general(bytes);
            let hit = model
                .ray_hit(origin, direction, 1e-6, None)
                .unwrap()
                .unwrap();
            close(hit.point_mm, expected);
            assert!((hit.distance_mm - (origin - expected).length()).abs() < 1e-7);
            assert!(matches!(hit.entity, EntityRef::Face(_)));
            assert_eq!(model.source().source().bytes(), bytes);
        }
    }

    #[cfg(feature = "general-development")]
    #[test]
    fn ray_queries_invalidate_old_measurements_but_retain_derived_source_receipts() {
        let bytes = include_bytes!("../tests/fixtures/cylinder-phase-rings.step");
        let mut model = general(bytes)
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        let before: Vec<_> = model
            .derived_vertices()
            .iter()
            .map(|v| (v.source_edge, v.native_vertex, v.native_parameter))
            .collect();
        assert!(!before.is_empty());
        let operations = model.operations();
        assert!(model.ray_hit(Vec3::ZERO, Vec3::ZERO, 1e-6, None).is_err());
        let report = model.general_report().unwrap();
        assert!(report.final_trim_measurements.is_some());
        assert!(report.material_measurements.is_some());
        // +Y on the upper ring is a computational vertex, absent from the
        // original source point map. The nearest native hit must retain it.
        let hit = model
            .ray_hit(Vec3::new(0., 8., 7.), -Vec3::Y, 1e-6, None)
            .unwrap()
            .unwrap();
        close(hit.point_mm, Vec3::new(0., 3., 7.));
        assert!(
            matches!(hit.entity, EntityRef::Vertex(vertex) if before.iter().any(|(_, id, _)| *id == vertex))
        );
        let report = model.general_report().unwrap();
        assert!(report.final_trim_measurements.is_none());
        assert!(report.material_measurements.is_none());
        assert_eq!(model.operations(), operations);
        assert_eq!(model.source().source().bytes(), bytes);
        assert_eq!(
            model
                .derived_vertices()
                .iter()
                .map(|v| (v.source_edge, v.native_vertex, v.native_parameter))
                .collect::<Vec<_>>(),
            before
        );
        // Measurement can be repeated explicitly after a query; qualification
        // is never inferred from the hit, the mesh, or the previous observation.
        let mut model = model
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        let face = *model.faces.values().next().unwrap();
        model.brep.face_mut(face).unwrap().surface = None;
        assert!(model
            .ray_hit(Vec3::new(0., 8., 7.), -Vec3::Y, 1e-6, None)
            .is_err());
        let report = model.general_report().unwrap();
        assert!(report.final_trim_measurements.is_none());
        assert!(report.material_measurements.is_none());
    }
}
