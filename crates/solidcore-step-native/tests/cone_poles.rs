// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public cone-pole controls with independently authored analytic truth.
//!
//! The apex is (0,0,0), the base is z=3 and radius 3 millimeters. The source
//! `VERTEX_LOOP` remains a point-boundary receipt at the shared native pole; it
//! must not become a second native loop sharing the chart's corner. The derived
//! meridian adds an edge, not a point or a source edge. These are numerical/native
//! candidate checkpoints, never a complete general-import acceptance claim.
#![cfg(feature = "general-development")]

#[cfg(test)]
mod tests {
    use solidcore::curve::curve::CurveKind;
    use solidcore::math::Vec3;
    use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
    use solidcore::surface::surface::SurfaceKind;
    use solidcore::topology::{edge, face};
    use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};
    use solidcore_step_native::{
        construct_general, Candidate, Error, GeneralFailureKind, GeneralQuality, Options,
    };
    use std::collections::BTreeSet;
    use std::f64::consts::PI;

    const SOURCE: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/cone-pole.step");
    const OPERATIONS: usize = 100_000;
    const ACCURACY: f64 = 1e-7;

    fn quality() -> Result<GeneralQuality, Error> {
        Ok(GeneralQuality {
            id: "Q-Public-Cone-Pole-Checkpoints".into(),
            definition_mm: Rational::decimal("0.0000001")?,
            consistency_mm: Rational::decimal("0.000001")?,
            derived_uv_mm: Rational::decimal("0.000001")?,
            query_mm: Rational::decimal("0.000001")?,
            maximum_work_tolerance_mm: Rational::decimal("0.00001")?,
            maximum_tolerance_growth_mm: Rational::ZERO,
        })
    }

    fn build(bytes: &[u8], operations: usize) -> Result<Candidate, Error> {
        let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
        construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations,
                work_tolerance_mm: 1e-6,
            },
            quality()?,
        )
    }

    fn complete(bytes: &[u8], operations: usize) -> Result<Candidate, Error> {
        build(bytes, operations)?
            .validate_native()?
            .measure_general_trims()?
            .measure_general_material(ACCURACY)
    }

    fn record(source: &s::Snapshot, id: &str) -> usize {
        source
            .source()
            .records()
            .iter()
            .position(|r| r.id == id)
            .unwrap()
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "actual={actual:.17e}, independent={expected:.17e}, tolerance={tolerance:.17e}"
        );
    }

    fn assert_source_and_topology(model: &Candidate, bytes: &[u8], orientation: bool) {
        assert_eq!(model.source().source().bytes(), bytes);
        assert_eq!(
            model.source().geometry_scope(),
            s::GeometryScope::GeneralBrep
        );
        assert_eq!(
            model.source().part_definition().unwrap().shape_role,
            "MaterialSolid"
        );
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (2, 1, 2)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (2, 2, 2)
        );
        let source = model.source();
        assert_eq!(source.units().length_unit, Some(record(source, "10")));
        assert_eq!(source.units().plane_angle_unit, Some(record(source, "11")));
        let pole = model.vertices()[&record(source, "120")];
        let anchor = model.vertices()[&record(source, "121")];
        let cap = model.faces()[&record(source, "180")];
        let side = model.faces()[&record(source, "182")];
        assert_ne!(pole, anchor);
        assert_eq!(model.brep().vertex(pole).unwrap().point, Vec3::ZERO);
        assert_eq!(
            model.brep().vertex(anchor).unwrap().point,
            Vec3::new(3., 0., 3.)
        );
        assert!(
            matches!(model.brep().face(side).unwrap().surface.as_ref().unwrap().kind(),SurfaceKind::Cone(c) if !c.is_cylinder_class())
        );

        let report = model.general_report().unwrap();
        assert_eq!(report.length_scale_mm, Rational::ONE);
        assert!(report.derived_vertices.is_empty());
        assert!(model.derived_vertices().is_empty());
        assert_eq!(report.loops.len(), 3);
        let point = report
            .loops
            .iter()
            .find(|l| l.loop_record == record(source, "163"))
            .unwrap();
        assert_eq!(
            (point.face, point.bound, point.source_vertex),
            (
                record(source, "182"),
                record(source, "173"),
                Some(record(source, "120"))
            )
        );
        assert!(!point.declared_outer);
        assert_eq!(point.bound_orientation, orientation);
        assert!(point.uses.is_empty());
        assert_eq!(
            report
                .loops
                .iter()
                .filter(|l| l.source_vertex.is_none())
                .count(),
            2
        );
        assert!(report
            .loops
            .iter()
            .filter(|l| l.source_vertex.is_none())
            .all(|l| l.uses.len() == 1 && l.bound_orientation));

        // A point-boundary receipt must identify the ACTUAL outer chart loop and
        // original pole, rather than silently disappearing or creating a second
        // native loop sharing that pole (which violates native face-loop incidence).
        assert_eq!(report.vertex_bounds.len(), 1);
        let binding = &report.vertex_bounds[0];
        assert_eq!(
            (
                binding.source_face,
                binding.source_bound,
                binding.source_loop,
                binding.source_vertex
            ),
            (
                record(source, "182"),
                record(source, "173"),
                record(source, "163"),
                record(source, "120")
            )
        );
        assert_eq!((binding.native_face, binding.native_vertex), (side, pole));
        assert_eq!(
            model.brep().face(side).unwrap().loops,
            vec![binding.native_loop]
        );
        assert_eq!(model.brep().loop_(binding.native_loop).unwrap().face, side);
        assert_eq!(report.chart_cuts.len(), 1);
        assert_eq!(report.chart_cuts[0].0, record(source, "182"));
        let meridian = report.chart_cuts[0].1;
        let native = model.brep().edge(meridian).unwrap();
        assert_eq!(
            native.vertices.into_iter().collect::<BTreeSet<_>>(),
            BTreeSet::from([pole, anchor])
        );
        assert_eq!(
            edge::faces(model.brep(), meridian, None)
                .unwrap()
                .into_iter()
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([side])
        );
        let uses = face::p4::upward_edgeuses(model.brep(), side, false, None).unwrap();
        assert_eq!(uses.len(), 3);
        let seam_uses: Vec<_> = uses
            .iter()
            .filter_map(|id| {
                let use_ = model.brep().edgeuse(*id).unwrap();
                (use_.edge == Some(meridian)).then_some(use_.orientation)
            })
            .collect();
        assert_eq!(seam_uses.len(), 2);
        assert!(seam_uses.contains(&OrientType::Same) && seam_uses.contains(&OrientType::Opposite));
        assert_eq!(model.brep().face(cap).unwrap().loops.len(), 1);
    }

    #[test]
    fn source_point_boundary_keeps_its_role_and_the_circle_keeps_its_complete_definition() {
        let model = build(SOURCE, OPERATIONS).unwrap();
        assert_source_and_topology(&model, SOURCE, true);
        let source = model.source();
        let pieces = &model.edges()[&record(source, "130")];
        assert_eq!(pieces.len(), 1);
        let edge = model.brep().edge(pieces[0]).unwrap();
        let basis = edge.curve.as_ref().unwrap();
        let CurveKind::Ellipse(circle) = basis.kind() else {
            panic!("source circle replaced")
        };
        assert!(circle.is_circle_class());
        close(circle.radius(), 3., 0.);
        close(circle.step_interval().length(), 360., 0.);
        assert_eq!(edge.interval, basis.natural_interval());
        let report = model.general_report().unwrap();
        let mapping = report
            .parameters
            .iter()
            .find(|m| m.source_use == record(source, "130") && m.kind == "ConicRationalParameter")
            .unwrap();
        assert_eq!(mapping.source_geometry, record(source, "110"));
        let original = &mapping
            .parameters
            .iter()
            .find(|(name, _)| name == "native_natural_interval")
            .unwrap()
            .1;
        assert_eq!(
            [edge.interval.min(), edge.interval.max()],
            [original[0], original[1]]
        );
        // Quarter-turn coordinates are authored independently. In particular the
        // native circle parameter is converted from each degree angle, never assumed
        // to be a STEP radian or shortened to a meridian-to-meridian arc.
        for (angle, expected) in [
            (0., Vec3::new(3., 0., 3.)),
            (90., Vec3::new(0., 3., 3.)),
            (180., Vec3::new(-3., 0., 3.)),
            (270., Vec3::new(0., -3., 3.)),
            (360., Vec3::new(3., 0., 3.)),
        ] {
            let t = circle
                .convert_t_from_step_to_nurbs(model.kernel(), angle)
                .unwrap();
            assert!((basis.evaluate_point(model.kernel(), t).unwrap() - expected).length() < 1e-12);
        }
    }

    #[test]
    fn pole_cone_completes_real_native_trim_and_independent_material_checkpoints() {
        let model = complete(SOURCE, OPERATIONS).unwrap();
        assert_source_and_topology(&model, SOURCE, true);
        let native = model.native_checks().unwrap();
        assert_eq!((native.pointer_reports, native.deep_reports), (0, 0));
        let report = model.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 4); // cap ring + side ring + both meridian incidences
        assert_eq!(
            trims
                .uses
                .iter()
                .filter(|u| u.source_edge == Some(record(model.source(), "130")))
                .count(),
            2
        );
        assert_eq!(
            trims
                .uses
                .iter()
                .filter(|u| u.source_edge.is_none())
                .count(),
            2
        );
        assert!(trims.samples >= 4 * 9 && trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            (2, 2, 2)
        );
        assert_eq!(material.requested_relative_accuracy, ACCURACY);
        assert_eq!(material.applied_relative_accuracy, ACCURACY);
        // Elementary right-cone truth: slant=3*sqrt(2), total area=9*pi*(1+sqrt(2)),
        // volume=9*pi, and the volume centroid is 3h/4 above the apex. These numerical
        // observation screens are separate from source/UV/work-tolerance budgets.
        let root2 = 2_f64.sqrt();
        let area = 9. * PI * (1. + root2);
        let volume = 9. * PI;
        close(material.area_mm2, area, area * 1e-4);
        close(material.material_volume_mm3, volume, volume * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 2.25)).length() < 1e-5);
        close(material.boundary_mm, 6. * PI + 3. * root2, 1e-5);
        for (actual, expected) in [
            (material.bounds_mm.min(), Vec3::new(-3., -3., 0.)),
            (material.bounds_mm.max(), Vec3::new(3., 3., 3.)),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        let source_edge = record(model.source(), "130");
        let source_face = record(model.source(), "182");
        let ring: Vec<_> = material
            .edges
            .iter()
            .filter(|e| e.source_edge == Some(source_edge))
            .collect();
        let seams: Vec<_> = material
            .edges
            .iter()
            .filter(|e| e.source_edge.is_none())
            .collect();
        assert_eq!((ring.len(), seams.len()), (1, 1));
        close(ring[0].length_mm, 6. * PI, 1e-5);
        assert_eq!(seams[0].derived_from_face, Some(source_face));
        close(seams[0].length_mm, 3. * root2, 1e-5);
        assert_eq!(seams[0].native_edge, report.chart_cuts[0].1);
        for result in &material.faces {
            if result.source_face == source_face {
                close(result.area_mm2, 9. * PI * root2, area * 1e-4);
                assert!((result.centroid_mm - Vec3::new(0., 0., 2.)).length() < 1e-5);
                assert_eq!((result.edges, result.vertices), (2, 2));
            } else {
                assert_eq!(result.source_face, record(model.source(), "180"));
                close(result.area_mm2, 9. * PI, area * 1e-4);
                assert!((result.centroid_mm - Vec3::new(0., 0., 3.)).length() < 1e-5);
                assert_eq!((result.edges, result.vertices), (1, 1));
            }
        }
    }

    #[test]
    fn original_vertex_bound_orientation_is_retained_after_chart_normalization() {
        let text = std::str::from_utf8(SOURCE).unwrap().replace(
            "#173=FACE_BOUND('',#163,.T.);",
            "#173=FACE_BOUND('',#163,.F.);",
        );
        assert_ne!(text.as_bytes(), SOURCE);
        let model = build(text.as_bytes(), OPERATIONS).unwrap();
        assert_source_and_topology(&model, text.as_bytes(), false);
        let field = model
            .source()
            .field(record(model.source(), "173"), None, "orientation")
            .unwrap();
        assert_eq!(field.value().unwrap().text, "F");
    }

    #[test]
    fn off_pole_and_wrong_ring_support_are_rejected_without_changing_the_source_cone() {
        let original = std::str::from_utf8(SOURCE).unwrap();
        // The pole references a NEW point: changing #100 would also move the cone's
        // frame and would cease to be a single off-pole defect. The radius negative
        // keeps its ring anchor on its own circle, so endpoint checks must still pass.
        let off_pole = original.replace(
            "#120=VERTEX_POINT('',#100);",
            "#190=CARTESIAN_POINT('',(0.,0.,0.001));\n#120=VERTEX_POINT('',#190);",
        );
        let radius = original
            .replace("#110=CIRCLE('',#107,3.);", "#110=CIRCLE('',#107,4.);")
            .replace(
                "#102=CARTESIAN_POINT('',(3.,0.,3.));",
                "#102=CARTESIAN_POINT('',(4.,0.,3.));",
            );
        for (text, key) in [
            (off_pole, "GeneralVertexLoopOffPole"),
            (radius, "GeneralVertexLoopRingSupport"),
        ] {
            let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
            let checked = s::check(
                &raw,
                s::Limits::fixtures(),
                &s::Options {
                    geometry_scope: s::GeometryScope::GeneralBrep,
                    check_geometry_dependencies: true,
                    check_part_and_closure: true,
                    ..s::Options::default()
                },
            );
            let source = checked.snapshot.unwrap();
            assert_eq!(source.source().bytes(), text.as_bytes());
            let baseline = inspect(SOURCE.to_vec(), Limits::planar_fixtures());
            let baseline = baseline.source.unwrap();
            for id in ["100", "106", "142"] {
                let actual = source.source();
                assert_eq!(
                    baseline.raw(baseline.find(id).unwrap().span).unwrap(),
                    actual.raw(actual.find(id).unwrap().span).unwrap()
                );
            }
            let Err(error) = build(text.as_bytes(), OPERATIONS) else {
                panic!("invalid pole/support was accepted")
            };
            assert!(
                matches!(error,Error::General {key:actual,record:owner,kind:GeneralFailureKind::Postcheck} if actual==key && owner==record(&source,"182")),
                "{error:?}"
            );
        }
        let replay = complete(SOURCE, OPERATIONS).unwrap();
        assert_source_and_topology(&replay, SOURCE, true);
    }

    #[test]
    fn every_construction_prefix_fails_without_publishing_and_exact_budget_replays_cleanly() {
        let full = build(SOURCE, OPERATIONS).unwrap();
        let required = full.operations();
        assert!(required > 0);
        drop(full);
        let raw = inspect(SOURCE.to_vec(), Limits::planar_fixtures());
        let original_records = raw.source.as_ref().unwrap().records().to_vec();
        let run = |limit| {
            construct_general(
                &raw,
                s::Limits::fixtures(),
                &s::Options::default(),
                Options {
                    operations: limit,
                    work_tolerance_mm: 1e-6,
                },
                quality().unwrap(),
            )
        };
        // Discover the completed boundary, then exercise EVERY lower budget. This
        // covers meridian construction and the source point-boundary handoff without
        // copying a private charging formula or treating partial candidates as success.
        for limit in 0..required {
            assert!(
                matches!(run(limit), Err(Error::ResourceLimit)),
                "prefix {limit}/{required}"
            );
            assert_eq!(raw.captured().unwrap(), SOURCE);
            assert_eq!(raw.source.as_ref().unwrap().records(), original_records);
        }
        let exact = run(required).unwrap();
        assert_eq!(exact.operations(), required);
        assert_source_and_topology(&exact, SOURCE, true);
        let replay = build(SOURCE, required).unwrap();
        assert_eq!(replay.operations(), required);
        assert_source_and_topology(&replay, SOURCE, true);
    }

    #[test]
    fn final_checkpoint_n_minus_one_fails_and_n_retains_both_complete_receipts() {
        let full = complete(SOURCE, OPERATIONS).unwrap();
        let required = full.operations();
        drop(full);
        assert!(matches!(
            complete(SOURCE, required - 1),
            Err(Error::ResourceLimit)
        ));
        let exact = complete(SOURCE, required).unwrap();
        assert_eq!(exact.operations(), required);
        assert_source_and_topology(&exact, SOURCE, true);
        let report = exact.general_report().unwrap();
        assert_eq!(report.final_trim_measurements.unwrap().edge_uses, 4);
        assert_eq!(report.material_measurements.unwrap().vertices, 2);
        let replay = complete(SOURCE, required).unwrap();
        assert_eq!(replay.operations(), required);
        assert_eq!(replay.source().source().bytes(), SOURCE);
    }

    #[test]
    fn explicit_angle_dimensions_proof_keeps_strict_refusal_and_all_cone_pole_receipts() {
        // Same right cone, now expressed as 45 source angle units. The name carries
        // no scale information: only the complete measure -> SI radian dependency
        // graph establishes the conversion, using the original factor lexeme. The
        // referenced dimensions are the one deliberate AP214 derived-slot defect.
        let text = std::str::from_utf8(SOURCE)
        .unwrap()
        .replace(
            "#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#11=(CONVERSION_BASED_UNIT('not a scale inference',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());",
        )
        .replace(
            "#142=CONICAL_SURFACE('',#106,0.,0.7853981633974483);",
            "#142=CONICAL_SURFACE('',#106,0.,45.);",
        )
        .replace(
            "ENDSEC;\nEND-ISO",
            "#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#901);\n#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n#902=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND-ISO",
        );
        assert_ne!(text.as_bytes(), SOURCE);
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        let strict_options = s::Options {
            geometry_scope: s::GeometryScope::GeneralBrep,
            check_geometry_dependencies: true,
            check_part_and_closure: true,
            ..s::Options::default()
        };
        let strict = s::check(&raw, s::Limits::fixtures(), &strict_options);
        assert_eq!(strict.status, s::Status::Rejected);
        assert!(strict.snapshot.is_none());
        assert!(strict
            .diagnostics
            .iter()
            .any(|d| d.source.key == "DerivedSlotMarkerRequired"));
        let Err(Error::Source {
            status,
            diagnostics,
            development_unit_diagnostics,
            ..
        }) = build(text.as_bytes(), OPERATIONS)
        else {
            panic!("strict native construction accepted an explicit derived-slot marker")
        };
        assert_eq!(status, s::Status::Rejected);
        assert_eq!(
            serde_json::to_value(diagnostics).unwrap(),
            serde_json::to_value(&strict.diagnostics).unwrap()
        );
        assert!(development_unit_diagnostics.is_empty());

        let options = s::Options {
            development_unit_policy: s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions,
            ..strict_options
        };
        let checked = s::check(&raw, s::Limits::fixtures(), &options);
        assert_eq!(checked.status, s::Status::CheckedWithDiagnostics);
        assert_eq!(checked.source_policy(), s::SourcePolicy::Strict);
        assert_eq!(checked.interoperability, "NotRun");
        assert!(checked.diagnostics.is_empty());
        assert_eq!(
            serde_json::to_value(checked.development_unit_diagnostics()).unwrap(),
            serde_json::to_value(&strict.diagnostics).unwrap()
        );
        let source = checked.snapshot.as_ref().unwrap();
        let unit = record(source, "11");
        let receipt = source.development_unit_receipt().unwrap();
        assert!(!receipt.source_syntax_verified());
        assert_eq!(receipt.affected_units(), [unit]);
        assert_eq!(receipt.scale_to_radians(unit), Some(0.0174532925199433));
        assert_eq!(receipt.unit_definitions().len(), 2);
        assert_eq!(
            receipt
                .unit_definitions()
                .iter()
                .find(|definition| definition.record_index() == unit)
                .unwrap()
                .factor_lexeme(),
            Some("0.0174532925199433")
        );
        assert_eq!(
            serde_json::to_value(receipt.original_diagnostics()).unwrap(),
            serde_json::to_value(&strict.diagnostics).unwrap()
        );
        let dimensions = source
            .field(unit, Some("NAMED_UNIT"), "dimensions")
            .unwrap();
        assert!(dimensions.declaration().unwrap().derived_slot);
        assert_eq!(
            dimensions.value().unwrap().kind,
            solidcore_step::ValueKind::Reference
        );
        assert_eq!(
            source
                .field(record(source, "142"), None, "semi_angle")
                .unwrap()
                .value()
                .unwrap()
                .text,
            "45."
        );
        assert_eq!(source.source().bytes(), text.as_bytes());

        // This crossing requires actual native, trim and material completion. A
        // development source receipt alone cannot qualify a built model or erase
        // the original syntax diagnostic, even when all numerical checks succeed.
        let model = construct_general(
            &raw,
            s::Limits::fixtures(),
            &options,
            Options {
                operations: OPERATIONS,
                work_tolerance_mm: 1e-6,
            },
            quality().unwrap(),
        )
        .unwrap()
        .validate_native()
        .unwrap()
        .measure_general_trims()
        .unwrap()
        .measure_general_material(ACCURACY)
        .unwrap();
        assert_source_and_topology(&model, text.as_bytes(), true);
        assert_eq!(model.native_checks().unwrap().deep_reports, 0);
        let report = model.general_report().unwrap();
        assert_eq!(report.source.status, s::Status::CheckedWithDiagnostics);
        assert_eq!(
            serde_json::to_value(report.source.development_unit_diagnostics()).unwrap(),
            serde_json::to_value(&strict.diagnostics).unwrap()
        );
        assert!(!model
            .source()
            .development_unit_receipt()
            .unwrap()
            .source_syntax_verified());
        assert_eq!(report.vertex_bounds.len(), 1);
        assert_eq!(report.final_trim_measurements.unwrap().edge_uses, 4);
        assert!(report.final_trim_measurements.unwrap().maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        let area = 9. * PI * (1. + 2_f64.sqrt());
        close(material.area_mm2, area, area * 1e-4);
        close(material.material_volume_mm3, 9. * PI, 9. * PI * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 2.25)).length() < 1e-5);
        close(material.boundary_mm, 6. * PI + 3. * 2_f64.sqrt(), 1e-5);
        assert_eq!(raw.captured().unwrap(), text.as_bytes());
    }

    fn assert_metamorphic_material(model: &Candidate, scale_mm: f64) {
        let checks = model.native_checks().unwrap();
        assert_eq!((checks.pointer_reports, checks.deep_reports), (0, 0));
        let report = model.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 4);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            (2, 2, 2)
        );
        let area = 9. * PI * (1. + 2_f64.sqrt()) * scale_mm * scale_mm;
        let volume = 9. * PI * scale_mm * scale_mm * scale_mm;
        close(material.area_mm2, area, area * 1e-4);
        close(material.material_volume_mm3, volume, volume * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 2.25 * scale_mm)).length() < 1e-5);
        close(
            material.boundary_mm,
            (6. * PI + 3. * 2_f64.sqrt()) * scale_mm,
            1e-5,
        );
        for (actual, expected) in [
            (
                material.bounds_mm.min(),
                Vec3::new(-3. * scale_mm, -3. * scale_mm, 0.),
            ),
            (
                material.bounds_mm.max(),
                Vec3::new(3. * scale_mm, 3. * scale_mm, 3. * scale_mm),
            ),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        let edge = model
            .brep()
            .edge(model.edges()[&record(model.source(), "130")][0])
            .unwrap();
        let basis = edge.curve.as_ref().unwrap();
        let CurveKind::Ellipse(circle) = basis.kind() else {
            panic!("complete source circle was replaced")
        };
        close(circle.radius(), 3. * scale_mm, 0.);
        close(circle.step_interval().length(), 360., 1e-12);
        assert_eq!(basis.natural_interval(), edge.interval);
        assert_eq!(report.chart_cuts.len(), 1);
        assert!(report.derived_vertices.is_empty());
        assert_eq!(report.vertex_bounds.len(), 1);
        let bound = &report.vertex_bounds[0];
        assert_eq!(bound.source_vertex, record(model.source(), "120"));
        assert_eq!(bound.native_vertex, model.vertices()[&bound.source_vertex]);
        assert_eq!(
            model.brep().face(bound.native_face).unwrap().loops,
            vec![bound.native_loop]
        );
    }

    #[test]
    fn rotated_source_frames_keep_the_phased_circle_and_pole_through_all_checkpoints() {
        // All analytic frames rotate together while the source anchor stays at +X.
        // The physical cone remains unchanged; the original circle/meridian chart
        // starts at -90 degrees instead of zero. This exercises parameter phase,
        // not a transformed or refitted boundary created by the test.
        let text = std::str::from_utf8(SOURCE).unwrap().replace(
            "#105=DIRECTION('',(1.,0.,0.));",
            "#105=DIRECTION('',(0.,1.,0.));",
        );
        assert_ne!(text.as_bytes(), SOURCE);
        let model = complete(text.as_bytes(), OPERATIONS).unwrap();
        assert_source_and_topology(&model, text.as_bytes(), true);
        assert_metamorphic_material(&model, 1.);
        let ring = model
            .brep()
            .edge(model.edges()[&record(model.source(), "130")][0])
            .unwrap();
        let CurveKind::Ellipse(circle) = ring.curve.as_ref().unwrap().kind() else {
            panic!("source circle")
        };
        assert_eq!(circle.position().x_axis(), Vec3::Y);
        assert_eq!(circle.position().y_axis(), -Vec3::X);
        close(circle.step_interval().min(), -90., 1e-12);
        close(circle.step_interval().max(), 270., 1e-12);
        assert_eq!(
            model
                .source()
                .field(record(model.source(), "130"), None, "same_sense")
                .unwrap()
                .value()
                .unwrap()
                .text,
            "T"
        );
    }

    #[test]
    fn reversed_circle_normal_and_source_edge_sense_keep_physical_traversal_and_pole() {
        // Only the circle's frame reverses; the plane still requires #107, so none
        // of the original geometry becomes an orphan. Reversing EDGE_CURVE's sense
        // while preserving both OEs is the same physical boundary traversal.
        let text = std::str::from_utf8(SOURCE).unwrap()
        .replace("#110=CIRCLE('',#107,3.);", "#190=DIRECTION('',(0.,0.,-1.));\n#191=AXIS2_PLACEMENT_3D('',#101,#190,#105);\n#110=CIRCLE('',#191,3.);")
        .replace("#130=EDGE_CURVE('',#121,#121,#110,.T.);", "#130=EDGE_CURVE('',#121,#121,#110,.F.);");
        assert_ne!(text.as_bytes(), SOURCE);
        let model = complete(text.as_bytes(), OPERATIONS).unwrap();
        assert_source_and_topology(&model, text.as_bytes(), true);
        assert_metamorphic_material(&model, 1.);
        let source = model.source();
        for (id, field, expected) in [
            ("130", "same_sense", "F"),
            ("150", "orientation", "T"),
            ("152", "orientation", "F"),
        ] {
            assert_eq!(
                source
                    .field(record(source, id), None, field)
                    .unwrap()
                    .value()
                    .unwrap()
                    .text,
                expected
            );
        }
        assert_eq!(
            source
                .field(record(source, "140"), None, "position")
                .unwrap()
                .reference_target(),
            Some(record(source, "107"))
        );
        let edge = model.edges()[&record(source, "130")][0];
        let basis = model.brep().edge(edge).unwrap().curve.as_ref().unwrap();
        let CurveKind::Ellipse(circle) = basis.kind() else {
            panic!("source circle")
        };
        assert_eq!(circle.position().y_axis(), -Vec3::Y);
        let quarter = circle
            .convert_t_from_step_to_nurbs(model.kernel(), 90.)
            .unwrap();
        assert!(
            (basis.evaluate_point(model.kernel(), quarter).unwrap() - Vec3::new(0., -3., 3.))
                .length()
                < 1e-12
        );
        for (face_id, forward) in [("180", false), ("182", true)] {
            let native_face = model.faces()[&record(source, face_id)];
            let uses: Vec<_> = face::p4::upward_edgeuses(model.brep(), native_face, false, None)
                .unwrap()
                .into_iter()
                .filter_map(|id| {
                    let u = model.brep().edgeuse(id).unwrap();
                    (u.edge == Some(edge)).then_some(u.orientation)
                })
                .collect();
            assert_eq!(
                uses,
                vec![if forward {
                    OrientType::Same
                } else {
                    OrientType::Opposite
                }]
            );
        }
    }

    #[test]
    fn centimeter_source_keeps_original_numbers_and_builds_the_independent_thirty_mm_cone() {
        // Changing the declared length unit alone gives r=h=30 mm. Neither source
        // coordinates nor quality allowances are multiplied or rewritten. Independent
        // truth is A=900*pi*(1+sqrt(2)), V=9000*pi, centroid z=22.5 mm.
        let text = std::str::from_utf8(SOURCE)
            .unwrap()
            .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)");
        assert_ne!(text.as_bytes(), SOURCE);
        let model = complete(text.as_bytes(), OPERATIONS).unwrap();
        assert_eq!(model.source().source().bytes(), text.as_bytes());
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (2, 1, 2)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (2, 2, 2)
        );
        let source = model.source();
        assert_eq!(
            model
                .brep()
                .vertex(model.vertices()[&record(source, "120")])
                .unwrap()
                .point,
            Vec3::ZERO
        );
        assert_eq!(
            model
                .brep()
                .vertex(model.vertices()[&record(source, "121")])
                .unwrap()
                .point,
            Vec3::new(30., 0., 30.)
        );
        assert_eq!(
            source
                .field(record(source, "110"), None, "radius")
                .unwrap()
                .value()
                .unwrap()
                .text,
            "3."
        );
        let report = model.general_report().unwrap();
        assert_eq!(report.length_scale_mm, Rational::decimal("10").unwrap());
        assert_eq!(
            report.quality.consistency_mm,
            quality().unwrap().consistency_mm
        );
        assert_eq!(
            report.quality.derived_uv_mm,
            quality().unwrap().derived_uv_mm
        );
        let point = report
            .loops
            .iter()
            .find(|bound| bound.loop_record == record(source, "163"))
            .unwrap();
        assert_eq!(point.source_vertex, Some(record(source, "120")));
        assert!(!point.declared_outer && point.bound_orientation && point.uses.is_empty());
        assert_metamorphic_material(&model, 10.);
    }

    fn assert_rigid_cone_checkpoints(
        text: &str,
        anchor: Vec3,
        centroid: Vec3,
        bounds: [Vec3; 2],
        interior_cap: bool,
    ) {
        let model = complete(text.as_bytes(), OPERATIONS).unwrap();
        assert_eq!(model.source().source().bytes(), text.as_bytes());
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (2, 1, 2)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (2, 2, 2)
        );
        let source = model.source();
        let pole = model.vertices()[&record(source, "120")];
        assert_eq!(model.brep().vertex(pole).unwrap().point, Vec3::ZERO);
        let native_anchor = model
            .brep()
            .vertex(model.vertices()[&record(source, "121")])
            .unwrap()
            .point;
        assert!((native_anchor - anchor).length() < 1e-12);
        let checked = model.native_checks().unwrap();
        assert_eq!((checked.pointer_reports, checked.deep_reports), (0, 0));
        let report = model.general_report().unwrap();
        assert_eq!(report.length_scale_mm, Rational::ONE);
        assert_eq!(
            report.quality.consistency_mm,
            quality().unwrap().consistency_mm
        );
        assert_eq!(report.chart_cuts.len(), 1);
        assert!(report.derived_vertices.is_empty());
        let point = report
            .loops
            .iter()
            .find(|bound| bound.loop_record == record(source, "163"))
            .unwrap();
        assert_eq!(point.source_vertex, Some(record(source, "120")));
        assert!(!point.declared_outer && point.bound_orientation && point.uses.is_empty());
        assert_eq!(report.vertex_bounds.len(), 1);
        let mapping = &report.vertex_bounds[0];
        assert_eq!(mapping.native_vertex, pole);
        assert_eq!(mapping.source_vertex, record(source, "120"));
        assert_eq!(
            model.brep().face(mapping.native_face).unwrap().loops,
            vec![mapping.native_loop]
        );
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 4);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            (2, 2, 2)
        );
        // A rigid transform changes neither scalar measure nor the original budgets.
        // Centroid and extrema below come directly from the authored rotation matrix.
        let area = 9. * PI * (1. + 2_f64.sqrt());
        close(material.area_mm2, area, area * 1e-4);
        close(material.material_volume_mm3, 9. * PI, 9. * PI * 1e-4);
        close(material.boundary_mm, 6. * PI + 3. * 2_f64.sqrt(), 1e-5);
        assert!((material.centroid_mm - centroid).length() < 1e-5);
        for (actual, expected) in [
            (material.bounds_mm.min(), bounds[0]),
            (material.bounds_mm.max(), bounds[1]),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        let ring = model.edges()[&record(source, "130")][0];
        let edge = model.brep().edge(ring).unwrap();
        let basis = edge.curve.as_ref().unwrap();
        let CurveKind::Ellipse(circle) = basis.kind() else {
            panic!("rotated original circle was replaced")
        };
        close(circle.radius(), 3., 0.);
        close(circle.step_interval().length(), 360., 1e-12);
        assert_eq!(basis.natural_interval(), edge.interval);
        if interior_cap {
            // The 45Y world AABB contains off-circle corners. Projecting those
            // corners onto the cone axis produces a lawful computational height
            // beyond the cap. Its trim is therefore INTERIOR to the natural V range;
            // assuming every cap is on natural Vmax must fail this public control.
            let side = model.faces()[&record(source, "182")];
            let face = model.brep().face(side).unwrap();
            let domain = face.surface.as_ref().unwrap().natural_uv_domain();
            let use_ = face::p4::upward_edgeuses(model.brep(), side, false, None)
                .unwrap()
                .into_iter()
                .find(|id| model.brep().edgeuse(*id).unwrap().edge == Some(ring))
                .unwrap();
            let (uv, _) =
                solidcore::topology::edgeuse::p1::uv_trim_curve(model.brep(), use_).unwrap();
            let point = uv
                .unwrap()
                .evaluate_point(model.kernel(), edge.interval.mid())
                .unwrap();
            let fraction = (point.y - domain.min().y) / (domain.max().y - domain.min().y);
            assert!(
                fraction > 0.25 && fraction < 0.75,
                "interior cap fraction={fraction}"
            );
        }
    }

    #[test]
    fn whole_cone_rotated_ninety_degrees_about_x_keeps_the_same_material_and_pole() {
        let text = std::str::from_utf8(SOURCE)
            .unwrap()
            .replace(
                "#101=CARTESIAN_POINT('',(0.,0.,3.));",
                "#101=CARTESIAN_POINT('',(0.,-3.,0.));",
            )
            .replace(
                "#102=CARTESIAN_POINT('',(3.,0.,3.));",
                "#102=CARTESIAN_POINT('',(3.,-3.,0.));",
            )
            .replace(
                "#104=DIRECTION('',(0.,0.,1.));",
                "#104=DIRECTION('',(0.,-1.,0.));",
            );
        assert_ne!(text.as_bytes(), SOURCE);
        assert_rigid_cone_checkpoints(
            &text,
            Vec3::new(3., -3., 0.),
            Vec3::new(0., -2.25, 0.),
            [Vec3::new(-3., -3., -3.), Vec3::new(3., 0., 3.)],
            false,
        );
    }

    #[test]
    fn whole_cone_rotated_forty_five_degrees_about_y_keeps_its_interior_cap_trim() {
        // Round-trip decimal encoding of sqrt(1/2) introduces only binary64 source
        // rounding. The original millimeter acceptance/work budgets remain unchanged;
        // the actual reader, construction and all final gates must establish them.
        let c = 0.5_f64.sqrt();
        let text = std::str::from_utf8(SOURCE)
            .unwrap()
            .replace(
                "#101=CARTESIAN_POINT('',(0.,0.,3.));",
                &format!("#101=CARTESIAN_POINT('',({},0.,{}));", 3. * c, 3. * c),
            )
            .replace(
                "#102=CARTESIAN_POINT('',(3.,0.,3.));",
                &format!("#102=CARTESIAN_POINT('',({},0.,0.));", 6. * c),
            )
            .replace(
                "#104=DIRECTION('',(0.,0.,1.));",
                &format!("#104=DIRECTION('',({c},0.,{c}));"),
            )
            .replace(
                "#105=DIRECTION('',(1.,0.,0.));",
                &format!("#105=DIRECTION('',({c},0.,{}));", -c),
            );
        assert_ne!(text.as_bytes(), SOURCE);
        assert_rigid_cone_checkpoints(
            &text,
            Vec3::new(6. * c, 0., 0.),
            Vec3::new(2.25 * c, 0., 2.25 * c),
            [
                Vec3::new(0., -3., 0.),
                Vec3::new(3. * 2_f64.sqrt(), 3., 3. * 2_f64.sqrt()),
            ],
            true,
        );
    }
}
