// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public linked-wireframe/supporting-geometry controls from the maintained
//! `STEP_IMPORT` tests at `200aa2f`: StepImportWireframeTests.cpp and
//! StepImportSupportingTests.cpp. The two fixtures retain those public bytes.
//!
//! Independent truth: the body is [2,12] x [-3,17] x [5,35] mm, V=6000 mm^3;
//! auxiliary points, finite curves, frames and infinite planes are separate
//! source roles. Numerical query/material observations do not certify import.
#![cfg(feature = "general-development")]

#[cfg(test)]
mod tests {
    use solidcore::math::{Vec2, Vec3};
    use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};
    use solidcore_step_native::{
        construct_general, AssociatedDefinitionRef, AssociatedGeometry,
        AssociatedRepresentationItem, AssociatedRole, Candidate, Error, GeneralFailureKind,
        GeneralQuality, Options,
    };

    const WIRE: &str = include_str!("fixtures/linked-wireframe.step");
    const SUPPORT: &str = include_str!("fixtures/supporting-geometry.step");
    const OPERATIONS: usize = 100_000;

    fn quality() -> GeneralQuality {
        GeneralQuality {
            id: "Q-Public-Associated-Geometry".into(),
            definition_mm: Rational::decimal("0.0000001").unwrap(),
            consistency_mm: Rational::decimal("0.000001").unwrap(),
            derived_uv_mm: Rational::decimal("0.000001").unwrap(),
            query_mm: Rational::decimal("0.000001").unwrap(),
            maximum_work_tolerance_mm: Rational::decimal("0.00001").unwrap(),
            maximum_tolerance_growth_mm: Rational::ZERO,
        }
    }

    fn build(text: &str, operations: usize) -> Result<Candidate, Error> {
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations,
                work_tolerance_mm: 1e-6,
            },
            quality(),
        )
    }

    fn complete(text: &str) -> Candidate {
        build(text, OPERATIONS)
            .unwrap()
            .validate_native_detailed()
            .unwrap()
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap()
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "actual={actual:.17e}, expected={expected:.17e}, tolerance={tolerance:.17e}"
        );
    }

    fn point(actual: Vec3, expected: Vec3) {
        close(actual.x, expected.x, 1e-9);
        close(actual.y, expected.y, 1e-9);
        close(actual.z, expected.z, 1e-9);
    }

    fn record(model: &Candidate, id: &str) -> usize {
        model
            .source()
            .source()
            .records()
            .iter()
            .position(|record| record.id == id)
            .unwrap()
    }

    fn curve_index(view: &AssociatedGeometry<'_>, id: &str) -> usize {
        view.curve_definitions()
            .iter()
            .position(|definition| definition.source_id == id)
            .unwrap()
    }

    fn assert_body(model: &Candidate, source: &str) {
        assert_eq!(model.source().source().bytes(), source.as_bytes());
        assert_eq!(
            model.source().part_definition().unwrap().shape_role,
            "MaterialSolid"
        );
        assert_eq!(
            (
                model.faces().len(),
                model.edges().len(),
                model.vertices().len()
            ),
            (6, 12, 8)
        );
        assert!(model.edges().values().all(|pieces| pieces.len() == 1));
        assert_eq!(
            (
                model.brep().face_list().len(),
                model.brep().edge_list().len(),
                model.brep().vertex_list().len()
            ),
            (6, 12, 8)
        );
        let report = model.general_report().unwrap();
        let source_check = report.source;
        assert_eq!(source_check.status, s::Status::Checked);
        assert!(source_check.diagnostics.is_empty());
        assert_eq!(
            (
                source_check.binding,
                source_check.types,
                source_check.unit_dependencies,
                source_check.geometry_dependencies,
                source_check.si_units,
                source_check.part
            ),
            ("Passed", "Passed", "Passed", "Passed", "Passed", "Passed")
        );
        assert!(report.chart_cuts.is_empty());
        assert!(report.derived_vertices.is_empty());
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 24);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.faces.len(),
                material.edges.len(),
                material.vertices
            ),
            (6, 12, 8)
        );
        // Elementary box integrals; these screens do not change the selected
        // source/UV/working budgets or claim a certified integration error.
        close(material.material_volume_mm3, 6000., 6000. * 1e-7);
        close(material.area_mm2, 2200., 2200. * 1e-7);
        close(material.boundary_mm, 240., 1e-7);
        point(material.centroid_mm, Vec3::new(7., 7., 20.));
        point(material.bounds_mm.min(), Vec3::new(2., -3., 5.));
        point(material.bounds_mm.max(), Vec3::new(12., 17., 35.));
        for id in ["911", "912", "940", "904"] {
            let original = record(model, id);
            assert!(!model.vertices().contains_key(&original));
            assert!(!model.edges().contains_key(&original));
            assert!(!model.faces().contains_key(&original));
        }
    }

    fn assert_curves(view: &AssociatedGeometry<'_>, line_y: f64) {
        assert_eq!(view.curve_definitions().len(), 2);
        for (id, basis, first, last, middle) in [
            (
                "912",
                "913",
                Vec3::new(0., line_y, 0.),
                Vec3::new(5., line_y, 0.),
                Vec3::new(2.5, line_y, 0.),
            ),
            (
                "940",
                "941",
                Vec3::new(2., 0., 0.),
                Vec3::new(0., 2., 0.),
                Vec3::new(2_f64.sqrt(), 2_f64.sqrt(), 0.),
            ),
        ] {
            let index = curve_index(view, id);
            let info = &view.curve_definitions()[index];
            assert_eq!(info.basis_id, basis);
            assert!(info.source_forward_in_native);
            let [lo, hi] = info.parameter_domain;
            assert!(lo.is_finite() && hi.is_finite() && hi > lo);
            // Native conic parameters need not be source radians. Evaluate the
            // actual retained interval rather than treating pi/2 as its max.
            point(view.evaluate_curve(index, lo).unwrap(), first);
            point(view.evaluate_curve(index, hi).unwrap(), last);
            point(view.evaluate_curve(index, (lo + hi) / 2.).unwrap(), middle);
            assert!(view.evaluate_curve(index, f64::NAN).is_err());
            assert!(view.evaluate_curve(index, f64::INFINITY).is_err());
            assert!(view.evaluate_curve(index, lo - (hi - lo)).is_err());
            assert!(view.evaluate_curve(index, hi + (hi - lo)).is_err());
        }
        assert!(view.evaluate_curve(usize::MAX, 0.).is_err());
    }

    fn assert_wireframe(model: &Candidate) {
        let view = model.associated_geometry().unwrap();
        assert_eq!(view.groups().len(), 1);
        let group = &view.groups()[0];
        assert_eq!(group.role, AssociatedRole::Wireframe);
        assert_eq!(
            (
                group.representation_id.as_str(),
                group.relationship_id.as_str(),
                group.context_id.as_str()
            ),
            ("920", "931", "12")
        );
        assert_eq!(group.set_id.as_deref(), Some("910"));
        assert_eq!(
            group
                .items
                .iter()
                .map(|item| item.source_id.as_str())
                .collect::<Vec<_>>(),
            ["911", "912", "940"]
        );
        let bounds = group.bounds_mm.unwrap();
        point(bounds[0], Vec3::new(0., 0., 0.));
        point(bounds[1], Vec3::new(7., 8., 9.));
        assert_eq!(view.point_definitions().len(), 1);
        assert_eq!(view.point_definitions()[0].source_id, "911");
        point(view.point_definitions()[0].point_mm, Vec3::new(7., 8., 9.));
        assert_eq!(view.frame_definitions().len(), 1);
        assert!(view.plane_definitions().is_empty());
        let representation = &view.representations()[0];
        assert_eq!(representation.role, AssociatedRole::Wireframe);
        assert!(
            matches!(&representation.items[0], AssociatedRepresentationItem::CurveSet { source_id, group_index: 0, .. } if source_id == "910")
        );
        assert!(
            matches!(&representation.items[1], AssociatedRepresentationItem::Definition(item) if item.source_id == "904" && matches!(item.definition, AssociatedDefinitionRef::Frame(_)))
        );
        assert_curves(&view, 0.);
        for (query, expected, distance) in [
            (Vec3::new(7., 8., 9.), Vec3::new(7., 8., 9.), 0.),
            (Vec3::new(-1., 0., 0.), Vec3::new(0., 0., 0.), 1.),
            (
                Vec3::new(2_f64.sqrt(), 2_f64.sqrt(), 0.),
                Vec3::new(2_f64.sqrt(), 2_f64.sqrt(), 0.),
                0.,
            ),
        ] {
            let hit = view.closest_wireframe(0, query).unwrap();
            point(hit.point_mm, expected);
            close(hit.distance_mm, distance, 1e-9);
        }
        assert!(view.closest_wireframe(usize::MAX, Vec3::ZERO).is_err());
        assert!(view
            .closest_wireframe(0, Vec3::new(f64::NAN, 0., 0.))
            .is_err());
    }

    fn assert_support(model: &Candidate) {
        let view = model.associated_geometry().unwrap();
        assert_eq!(view.groups().len(), 1);
        let group = &view.groups()[0];
        assert_eq!(group.role, AssociatedRole::Supporting);
        assert_eq!(
            (
                group.representation_id.as_str(),
                group.relationship_id.as_str(),
                group.context_id.as_str()
            ),
            ("960", "961", "12")
        );
        assert!(group.set_id.is_none());
        assert!(group.bounds_mm.is_none());
        assert_eq!(
            group
                .items
                .iter()
                .map(|item| item.source_id.as_str())
                .collect::<Vec<_>>(),
            ["904", "970", "972", "911", "912", "940"]
        );
        assert_eq!(
            (
                view.frame_definitions().len(),
                view.plane_definitions().len(),
                view.point_definitions().len()
            ),
            (2, 1, 1)
        );
        assert_curves(&view, 0.);
        let (frame_index, frame) = view
            .frame_definitions()
            .iter()
            .enumerate()
            .find(|(_, frame)| frame.source_id == "970")
            .unwrap();
        assert_eq!(view.coordinate_frame(frame_index).unwrap(), frame.frame);
        point(frame.frame.origin_mm, Vec3::new(30., 40., 50.));
        point(frame.frame.x, Vec3::new(0., 1., 0.));
        point(frame.frame.y, Vec3::new(0., 0., 1.));
        point(frame.frame.z, Vec3::new(1., 0., 0.));
        let plane = &view.plane_definitions()[0];
        assert_eq!(
            (plane.source_id.as_str(), plane.position_id.as_str()),
            ("972", "970")
        );
        assert_eq!(plane.position_record, frame.source_record);
        assert_eq!(plane.frame, frame.frame);
        point(
            view.evaluate_plane(0, Vec2::new(2., 3.)).unwrap(),
            Vec3::new(30., 42., 53.),
        );
        let closest = view.closest_plane(0, Vec3::new(35., 42., 53.)).unwrap();
        point(closest.point_mm, Vec3::new(30., 42., 53.));
        close(closest.distance_mm, 5., 1e-9);
        let [umin, vmin, umax, vmax] = plane.parameter_domain;
        assert!(umin.is_finite() && vmin.is_finite() && umax.is_finite() && vmax.is_finite());
        assert!(view
            .evaluate_plane(0, Vec2::new(umin - (umax - umin), 0.))
            .is_err());
        assert!(view
            .evaluate_plane(0, Vec2::new(0., vmax + (vmax - vmin)))
            .is_err());
        assert!(view.evaluate_plane(0, Vec2::new(f64::NAN, 0.)).is_err());
        assert!(view.evaluate_plane(usize::MAX, Vec2::ZERO).is_err());
        assert!(view.coordinate_frame(usize::MAX).is_err());
        assert!(view.closest_plane(usize::MAX, Vec3::ZERO).is_err());
        assert!(view
            .closest_plane(0, Vec3::new(0., f64::INFINITY, 0.))
            .is_err());
        assert!(view.closest_wireframe(0, Vec3::ZERO).is_err());
    }

    fn replace(text: &str, before: &str, after: &str) -> String {
        assert_eq!(
            text.matches(before).count(),
            1,
            "public fixture edit must be unique: {before}"
        );
        text.replacen(before, after, 1)
    }

    fn append(text: &str, records: &str) -> String {
        replace(
            text,
            "ENDSEC;\nEND-ISO-10303-21;",
            &format!("{records}ENDSEC;\nEND-ISO-10303-21;"),
        )
    }

    fn shared() -> String {
        // Fixed C++ SharedSource: #914 is the LINE origin. Its 1e-7 mm offset
        // leaves the second source trim point off the defining line by 1e-7;
        // neither original bytes nor that admitted residual may be reset on reuse.
        let text = append(SUPPORT,
            "#910=GEOMETRIC_CURVE_SET('',(#911,#912,#940,#918));\n#918=CARTESIAN_POINT('',(7.,8.,9.));\n#920=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#910,#904,#919),#12);\n#931=SHAPE_REPRESENTATION_RELATIONSHIP('','',#900,#920);\n#919=GEOMETRIC_CURVE_SET('',(#912,#911,#940));\n");
        let text = replace(
            &text,
            "#914=CARTESIAN_POINT('',(0.,0.,0.));",
            "#914=CARTESIAN_POINT('',(0.,0.0000001,0.));",
        );
        replace(
            &text,
            "(#904,#970,#972,#911,#912,#940)",
            "(#912,#904,#911,#972,#940,#970)",
        )
    }

    fn centimeters(text: &str) -> String {
        // Exact decimal scaling of this known synthetic carrier only: lengths
        // are divided by ten, never directions or angular PARAMETER_VALUEs.
        // Every generated REAL retains a decimal point (valid Part 21 spelling).
        fn scaled(value: &str) -> String {
            assert!(value.contains('.') && !value.contains(['e', 'E']));
            format!("{value}E-1")
        }
        let mut result = String::new();
        for line in text.lines() {
            let edited = if line.contains("=CARTESIAN_POINT(") {
                let (prefix, coordinates) = line.split_once(",(").unwrap();
                let coordinates = coordinates.strip_suffix("));").unwrap();
                format!(
                    "{prefix},({}));",
                    coordinates
                        .split(',')
                        .map(scaled)
                        .collect::<Vec<_>>()
                        .join(",")
                )
            } else if line.contains("=VECTOR(") || line.contains("=CIRCLE(") {
                let (prefix, length) = line.rsplit_once(',').unwrap();
                format!("{prefix},{});", scaled(length.strip_suffix(");").unwrap()))
            } else {
                line.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)")
            };
            result.push_str(&edited);
            result.push('\n');
        }
        result
    }

    #[test]
    fn linked_wireframe_has_real_queries_without_polluting_material_topology() {
        let mut model = complete(WIRE);
        assert_body(&model, WIRE);
        assert_wireframe(&model);
        // #911 lies inside the box. The body-only ray must ignore the exact
        // auxiliary point hit at its origin and return the box bottom instead.
        let hit = model
            .ray_hit(
                Vec3::new(7., 8., 9.),
                Vec3::new(0., 0., -1.),
                1e-6,
                Some(10.),
            )
            .unwrap()
            .unwrap();
        point(hit.point_mm, Vec3::new(7., 8., 5.));
        close(hit.distance_mm, 4., 1e-9);
        assert_eq!(model.source().source().bytes(), WIRE.as_bytes());
    }

    #[test]
    fn supporting_geometry_retains_actual_frames_unbounded_plane_and_finite_curves() {
        let model = complete(SUPPORT);
        assert_body(&model, SUPPORT);
        assert_support(&model);
    }

    #[test]
    fn centimetre_source_preserves_physical_body_and_associated_geometry() {
        for (source, wire) in [(WIRE, true), (SUPPORT, false)] {
            let text = centimeters(source);
            assert!(text.contains("SI_UNIT(.CENTI.,.METRE.)"));
            assert!(text.contains("PARAMETER_VALUE(1.5707963267948966)"));
            let model = complete(&text);
            assert_body(&model, &text);
            if wire {
                assert_wireframe(&model);
            } else {
                assert_support(&model);
            }
        }
    }

    #[test]
    fn same_source_definitions_are_shared_across_roles_but_equal_points_keep_identity() {
        let text = shared();
        let model = complete(&text);
        assert_body(&model, &text);
        let view = model.associated_geometry().unwrap();
        assert_eq!(view.groups().len(), 3);
        assert_eq!(view.curve_definitions().len(), 2);
        assert_eq!(view.point_definitions().len(), 2);
        assert_eq!(view.frame_definitions().len(), 2);
        assert_eq!(view.plane_definitions().len(), 1);
        assert_curves(&view, 1e-7);
        for (index, group) in view.groups().iter().enumerate() {
            assert_eq!(
                group.role,
                if index < 2 {
                    AssociatedRole::Wireframe
                } else {
                    AssociatedRole::Supporting
                }
            );
            for id in ["912", "940", "911"] {
                let original = view.groups()[0]
                    .items
                    .iter()
                    .find(|item| item.source_id == id)
                    .unwrap();
                let occurrence = group
                    .items
                    .iter()
                    .find(|item| item.source_id == id)
                    .unwrap();
                assert_eq!(original.definition, occurrence.definition);
                assert_eq!(original.source_record, occurrence.source_record);
                assert_eq!(occurrence.source_record, record(&model, id));
                assert_eq!(original.source_types, occurrence.source_types);
            }
            close(group.source_residual_mm.unwrap(), 1e-7, 1e-14);
        }
        assert_eq!(
            view.groups()[2]
                .items
                .iter()
                .map(|item| item.source_id.as_str())
                .collect::<Vec<_>>(),
            ["912", "904", "911", "972", "940", "970"]
        );
        let first = view.groups()[0]
            .items
            .iter()
            .find(|item| item.source_id == "911")
            .unwrap();
        let equal = view.groups()[0]
            .items
            .iter()
            .find(|item| item.source_id == "918")
            .unwrap();
        assert_ne!(first.definition, equal.definition);
        assert_ne!(first.source_record, equal.source_record);
        point(
            view.point_definitions()[0].point_mm,
            view.point_definitions()[1].point_mm,
        );
        close(
            view.curve_definitions()[curve_index(&view, "912")]
                .source_residual_mm
                .unwrap(),
            1e-7,
            1e-14,
        );
    }

    #[test]
    fn reversed_source_trim_sense_keeps_definition_domain_and_explicit_direction() {
        for source in [WIRE, SUPPORT] {
            let text = replace(
                source,
                "(#914),(#915),.T.,.CARTESIAN.",
                "(#915),(#914),.F.,.CARTESIAN.",
            );
            let original = complete(source);
            let reversed = complete(&text);
            assert_body(&reversed, &text);
            let a = original.associated_geometry().unwrap();
            let b = reversed.associated_geometry().unwrap();
            let ai = curve_index(&a, "912");
            let bi = curve_index(&b, "912");
            let normal = &a.curve_definitions()[ai];
            let reverse = &b.curve_definitions()[bi];
            assert!(normal.source_forward_in_native);
            assert!(!reverse.source_forward_in_native);
            assert_eq!(normal.parameter_domain, reverse.parameter_domain);
            assert_eq!(reverse.basis_id, "913");
            for parameter in [reverse.parameter_domain[0], reverse.parameter_domain[1]] {
                point(
                    a.evaluate_curve(ai, parameter).unwrap(),
                    b.evaluate_curve(bi, parameter).unwrap(),
                );
            }
            assert!(text.contains("(#915),(#914),.F.,.CARTESIAN."));
        }
    }

    #[test]
    fn irrational_support_frame_projects_in_its_actual_binary64_basis() {
        let text = replace(
            SUPPORT,
            "#973=DIRECTION('',(1.,0.,0.));",
            "#973=DIRECTION('',(1.,1.,1.));",
        );
        let model = complete(&text);
        assert_body(&model, &text);
        let view = model.associated_geometry().unwrap();
        // The support plane is x+y+z=120. Its orthogonal projection is analytic
        // and independent of native basis normalization or its UV coordinates.
        let closest = view.closest_plane(0, Vec3::new(31., 42., 53.)).unwrap();
        point(closest.point_mm, Vec3::new(29., 40., 51.));
        close(closest.distance_mm, 12_f64.sqrt(), 1e-9);
        let frame = view.plane_definitions()[0].frame;
        let native = view.evaluate_plane(0, Vec2::new(2., 3.)).unwrap();
        point(native, frame.origin_mm + frame.x * 2. + frame.y * 3.);
        close(native.x + native.y + native.z, 120., 1e-9);
    }

    #[test]
    fn associated_construction_budget_is_all_or_nothing_with_unchanged_source_replay() {
        for text in [WIRE.to_owned(), SUPPORT.to_owned(), shared()] {
            let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
            let original_records = raw.source.as_ref().unwrap().records().to_vec();
            let run = |operations| {
                construct_general(
                    &raw,
                    s::Limits::fixtures(),
                    &s::Options::default(),
                    Options {
                        operations,
                        work_tolerance_mm: 1e-6,
                    },
                    quality(),
                )
            };
            let full = run(OPERATIONS).unwrap();
            let needed = full.operations();
            assert!(needed > 0);
            assert!(matches!(run(needed - 1), Err(Error::ResourceLimit)));
            assert_eq!(raw.source.as_ref().unwrap().bytes(), text.as_bytes());
            assert_eq!(raw.source.as_ref().unwrap().records(), original_records);
            let exact = run(needed).unwrap();
            assert_eq!(exact.operations(), needed);
            assert_eq!(exact.source().source().bytes(), text.as_bytes());
            let replay = run(OPERATIONS).unwrap();
            assert_eq!(replay.operations(), needed);
            assert_eq!(
                (
                    replay.faces().len(),
                    replay.edges().len(),
                    replay.vertices().len()
                ),
                (6, 12, 8)
            );
            let expected = full.associated_geometry().unwrap();
            for model in [&exact, &replay] {
                let actual = model.associated_geometry().unwrap();
                assert_eq!(actual.groups().len(), expected.groups().len());
                assert_eq!(
                    actual.curve_definitions().len(),
                    expected.curve_definitions().len()
                );
                assert_eq!(
                    actual.point_definitions().len(),
                    expected.point_definitions().len()
                );
                for (a, b) in actual.groups().iter().zip(expected.groups()) {
                    assert_eq!(a.role, b.role);
                    assert_eq!(
                        a.items
                            .iter()
                            .map(|item| (&item.source_id, item.definition))
                            .collect::<Vec<_>>(),
                        b.items
                            .iter()
                            .map(|item| (&item.source_id, item.definition))
                            .collect::<Vec<_>>()
                    );
                }
            }
        }
    }

    #[test]
    fn invalid_or_unsupported_required_curve_is_not_dropped_or_repaired() {
        let source_check = |text: &str| {
            let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
            s::check(
                &raw,
                s::Limits::fixtures(),
                &s::Options {
                    geometry_scope: s::GeometryScope::GeneralBrep,
                    check_geometry_dependencies: true,
                    check_part_and_closure: true,
                    ..s::Options::default()
                },
            )
        };
        let bare_line = replace(WIRE, "(#911,#912,#940)", "(#911,#912,#940,#913)");
        // A LINE passes the SELECT type, but the fixed source wireframe WR3
        // additionally requires a bounded/closed admissible curve. Preserve
        // that earlier source rejection; native scope cannot override it.
        let invalid = source_check(&bare_line);
        let source_key =
            "SourceRuleViolation:GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION.WR3";
        assert_eq!(invalid.status, s::Status::Rejected);
        assert_eq!(invalid.diagnostics.len(), 1);
        assert_eq!(invalid.diagnostics[0].source.key, source_key);
        assert_eq!(invalid.diagnostics[0].field, "elements");
        match build(&bare_line, OPERATIONS) {
            Err(Error::Source {
                status,
                diagnostics,
                ..
            }) => {
                assert_eq!(status, s::Status::Rejected);
                assert_eq!(diagnostics.len(), 1);
                assert_eq!(diagnostics[0].source.key, source_key);
            }
            Err(other) => panic!("source rejection changed layer: {other:?}"),
            Ok(_) => panic!("invalid source wireframe silently accepted"),
        }
        for (source, before, after, expected_key) in [
            (
                WIRE,
                "(#911,#912,#940)",
                "(#911,#912,#940,#980)",
                "GeneralWireframeGeometryUnsupported",
            ),
            (
                SUPPORT,
                "(#904,#970,#972,#911,#912,#940)",
                "(#904,#970,#972,#911,#912,#940,#980)",
                "GeneralSupportingGeometryUnsupported",
            ),
        ] {
            let text = append(
                &replace(source, before, after),
                "#980=COMPOSITE_CURVE('',(#981),.F.);\n#981=COMPOSITE_CURVE_SEGMENT(.DISCONTINUOUS.,.T.,#912);\n",
            );
            let checked = source_check(&text);
            assert_eq!(checked.status, s::Status::Checked);
            assert_eq!(checked.part, "Passed");
            assert!(checked.diagnostics.is_empty());
            // A one-segment bounded composite with its optional final
            // discontinuity satisfies the source composite and wireframe rules;
            // this native associated increment does not construct composites.
            // It must refuse the complete request rather than omit this member.
            match build(&text, OPERATIONS) {
                Err(Error::General {
                    key,
                    kind: GeneralFailureKind::Unsupported,
                    ..
                }) => assert_eq!(key, expected_key),
                Err(other) => {
                    panic!(
                        "wrong failure for legal but unsupported associated composite: {other:?}"
                    )
                }
                Ok(_) => panic!("required source member silently omitted"),
            }
            let replay = complete(source);
            assert_body(&replay, source);
        }
    }

    fn shared_family(definitions: &str) -> String {
        // Same source definition #950 appears in both physical roles; the
        // source control points remain definition dependencies, not body V's.
        let text = replace(
            SUPPORT,
            "(#904,#970,#972,#911,#912,#940)",
            "(#904,#970,#972,#911,#912,#940,#950)",
        );
        append(
            &text,
            &format!(
                "{definitions}#910=GEOMETRIC_CURVE_SET('',(#911,#912,#940,#950));\n#920=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#910,#904),#12);\n#931=SHAPE_REPRESENTATION_RELATIONSHIP('','',#900,#920);\n"
            ),
        )
    }

    fn assert_shared_family(model: &Candidate, basis_id: &str) -> usize {
        let view = model.associated_geometry().unwrap();
        assert_eq!(view.groups().len(), 2);
        assert_eq!(view.curve_definitions().len(), 3);
        let wire = view
            .groups()
            .iter()
            .find(|group| group.role == AssociatedRole::Wireframe)
            .unwrap();
        let support = view
            .groups()
            .iter()
            .find(|group| group.role == AssociatedRole::Supporting)
            .unwrap();
        let a = wire
            .items
            .iter()
            .find(|item| item.source_id == "950")
            .unwrap();
        let b = support
            .items
            .iter()
            .find(|item| item.source_id == "950")
            .unwrap();
        assert_eq!(a.definition, b.definition);
        assert_eq!(a.source_record, b.source_record);
        assert_eq!(a.source_record, record(model, "950"));
        let index = curve_index(&view, "950");
        assert_eq!(a.definition, AssociatedDefinitionRef::Curve(index));
        let info = &view.curve_definitions()[index];
        assert_eq!(info.basis_id, basis_id);
        assert_eq!(info.basis_record, record(model, basis_id));
        let mappings: Vec<_> = model
            .general_report()
            .unwrap()
            .parameters
            .iter()
            .filter(|mapping| mapping.source_geometry == a.source_record)
            .collect();
        // Reusing the owner does not manufacture a second interpretation or
        // replace the original source/basis identities with a body edge ID.
        assert_eq!(mappings.len(), 1);
        assert_eq!(mappings[0].source_use, a.source_record);
        assert_eq!(mappings[0].source_basis_geometry, Some(info.basis_record));
        for id in ["950", "951", "952", "953"] {
            if let Some(source) = model
                .source()
                .source()
                .records()
                .iter()
                .position(|source| source.id == id)
            {
                assert!(!model.vertices().contains_key(&source));
                assert!(!model.edges().contains_key(&source));
                assert!(!model.faces().contains_key(&source));
            }
        }
        index
    }

    #[test]
    fn complete_conics_and_explicit_knot_splines_have_independent_internal_values_across_roles() {
        // Public CPP CurveFamilies/SharedDefinitions at 200aa2f, including
        // nonuniform rational weights. This is independent elementary Bezier
        // truth, not a second implementation of the B-spline evaluator.
        let families = [
            "#950=CIRCLE('',#904,2.);\n",
            "#950=ELLIPSE('',#904,3.,2.);\n",
            "#950=B_SPLINE_CURVE_WITH_KNOTS('',2,(#914,#951,#952),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);\n#951=CARTESIAN_POINT('',(1.,1.,0.));\n#952=CARTESIAN_POINT('',(2.,0.,0.));\n",
            "#950=(BOUNDED_CURVE() B_SPLINE_CURVE(2,(#914,#951,#952),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,2.,1.)) REPRESENTATION_ITEM(''));\n#951=CARTESIAN_POINT('',(1.,1.,0.));\n#952=CARTESIAN_POINT('',(2.,0.,0.));\n",
        ];
        for (family, definitions) in families.into_iter().enumerate() {
            let text = shared_family(definitions);
            let model = complete(&text);
            assert_body(&model, &text);
            let index = assert_shared_family(&model, "950");
            let view = model.associated_geometry().unwrap();
            let info = &view.curve_definitions()[index];
            assert!(info.source_forward_in_native);
            assert!(info.source_residual_mm.is_none());
            let [lo, hi] = info.parameter_domain;
            if family < 2 {
                let rx = if family == 0 { 2. } else { 3. };
                for (fraction, expected) in [
                    (0., Vec3::new(rx, 0., 0.)),
                    (0.25, Vec3::new(0., 2., 0.)),
                    (0.5, Vec3::new(-rx, 0., 0.)),
                    (0.75, Vec3::new(0., -2., 0.)),
                    (1., Vec3::new(rx, 0., 0.)),
                ] {
                    point(
                        view.evaluate_curve(index, lo + (hi - lo) * fraction)
                            .unwrap(),
                        expected,
                    );
                }
                let wire_index = view
                    .groups()
                    .iter()
                    .position(|group| group.role == AssociatedRole::Wireframe)
                    .unwrap();
                let closest = view
                    .closest_wireframe(wire_index, Vec3::new(-rx - 1., 0., 0.))
                    .unwrap();
                // The original line and quarter arc have x>=0, so the new
                // complete conic is the unique closest member at this query.
                point(closest.point_mm, Vec3::new(-rx, 0., 0.));
                close(closest.distance_mm, 1., 1e-9);
                assert!(info.endpoint_error_mm.is_some());
            } else {
                assert_eq!([lo, hi], [0., 1.]);
                assert!(info.endpoint_error_mm.is_none());
                let expected = if family == 2 {
                    [(0., 0.), (0.5, 0.375), (1., 0.5), (1.5, 0.375), (2., 0.)]
                } else {
                    // P(t)=[4t(1-t)*(1,1)+t^2*(2,0)]/[1+2t(1-t)].
                    [
                        (0., 0.),
                        (7. / 11., 6. / 11.),
                        (1., 2. / 3.),
                        (15. / 11., 6. / 11.),
                        (2., 0.),
                    ]
                };
                for ((x, y), t) in expected.into_iter().zip([0., 0.25, 0.5, 0.75, 1.]) {
                    point(view.evaluate_curve(index, t).unwrap(), Vec3::new(x, y, 0.));
                }
                let source = model.source();
                let curve = record(&model, "950");
                assert_eq!(
                    source
                        .field(curve, None, "degree")
                        .unwrap()
                        .value()
                        .unwrap()
                        .text,
                    "2"
                );
                for (field, expected) in [
                    ("control_points_list", vec!["914", "951", "952"]),
                    ("knot_multiplicities", vec!["3", "3"]),
                    ("knots", vec!["0.", "1."]),
                ] {
                    let value = source.field(curve, None, field).unwrap();
                    assert_eq!(
                        value
                            .value()
                            .unwrap()
                            .children
                            .iter()
                            .map(|value| value.text.as_str())
                            .collect::<Vec<_>>(),
                        expected
                    );
                }
                if family == 3 {
                    let weights = source.field(curve, None, "weights_data").unwrap();
                    assert_eq!(
                        weights
                            .value()
                            .unwrap()
                            .children
                            .iter()
                            .map(|value| value.text.as_str())
                            .collect::<Vec<_>>(),
                        ["1.", "2.", "1."]
                    );
                }
                let mapping = model
                    .general_report()
                    .unwrap()
                    .parameters
                    .iter()
                    .find(|mapping| mapping.source_geometry == curve)
                    .unwrap();
                assert_eq!(mapping.kind, "WireframeSplineIdentity");
                assert!(mapping.equation.contains("source_t=native_t"));
            }
        }
    }

    #[test]
    fn finite_ellipse_preserves_source_angle_parameters_and_reversed_trim_direction() {
        let definitions = "#950=TRIMMED_CURVE('',#953,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.5707963267948966)),.T.,.PARAMETER.);\n#953=ELLIPSE('',#904,3.,2.);\n";
        let normal = shared_family(definitions);
        let reversed = replace(&normal,
            "#950=TRIMMED_CURVE('',#953,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.5707963267948966)),.T.,.PARAMETER.);",
            "#950=TRIMMED_CURVE('',#953,(PARAMETER_VALUE(1.5707963267948966)),(PARAMETER_VALUE(0.)),.F.,.PARAMETER.);");
        let mut original_domain = None;
        for (text, forward) in [(&normal, true), (&reversed, false)] {
            let model = complete(text);
            assert_body(&model, text);
            let index = assert_shared_family(&model, "953");
            let view = model.associated_geometry().unwrap();
            let info = &view.curve_definitions()[index];
            assert_eq!(info.source_forward_in_native, forward);
            assert!(info.source_residual_mm.is_none());
            let [lo, hi] = info.parameter_domain;
            if let Some(domain) = original_domain {
                assert_eq!([lo, hi], domain);
            } else {
                original_domain = Some([lo, hi]);
            }
            for (t, expected) in [
                (lo, Vec3::new(3., 0., 0.)),
                (
                    (lo + hi) / 2.,
                    Vec3::new(3. / 2_f64.sqrt(), 2_f64.sqrt(), 0.),
                ),
                (hi, Vec3::new(0., 2., 0.)),
            ] {
                point(view.evaluate_curve(index, t).unwrap(), expected);
            }
            let mapping = model
                .general_report()
                .unwrap()
                .parameters
                .iter()
                .find(|mapping| mapping.source_geometry == record(&model, "950"))
                .unwrap();
            assert_eq!(mapping.kind, "WireframeConicRationalParameter");
            let parameters = |name| {
                &mapping
                    .parameters
                    .iter()
                    .find(|(key, _)| key == name)
                    .unwrap()
                    .1
            };
            let angles = if forward {
                vec![0., std::f64::consts::FRAC_PI_2]
            } else {
                vec![std::f64::consts::FRAC_PI_2, 0.]
            };
            assert_eq!(parameters("basis_trim_rad"), &angles);
            assert_eq!(
                parameters("source_forward_in_native"),
                &vec![if forward { 1. } else { -1. }]
            );
            let trim = model
                .source()
                .field(record(&model, "950"), None, "trim_1")
                .unwrap();
            assert_eq!(
                trim.value().unwrap().children[0].children[0].text,
                if forward { "0." } else { "1.5707963267948966" }
            );
        }
    }
}
