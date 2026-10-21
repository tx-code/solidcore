// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independently authored public periodic-ring controls; no private CAD input.
//!
//! Radius 3 mm, z in [0,7] mm. The lower closed edge starts at +Y, the
//! upper at +X. A meridian through the lower source vertex therefore needs
//! one derived vertex on the upper full circle and two exact upper intervals.
//! The circles and their two source vertices remain unchanged. Every upper
//! interval belongs to both the cap and the cylindrical face.
#![cfg(feature = "general-development")]

#[cfg(test)]
mod tests {
    use solidcore::curve::curve::CurveKind;
    use solidcore::math::Vec3;
    use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
    use solidcore::topology::{edge, face};
    use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};
    use solidcore_step_native::{
        construct_general, Candidate, Error, GeneralFailureKind, GeneralQuality, Options,
    };
    use std::collections::BTreeSet;
    use std::f64::consts::PI;

    const SOURCE: &[u8] = include_bytes!("fixtures/cylinder-phase-rings.step");

    fn quality() -> Result<GeneralQuality, Error> {
        Ok(GeneralQuality {
            id: "Q-Public-Periodic-Rings".into(),
            definition_mm: Rational::decimal("0.0000001")?,
            consistency_mm: Rational::decimal("0.000001")?,
            derived_uv_mm: Rational::decimal("0.000001")?,
            query_mm: Rational::decimal("0.000001")?,
            maximum_work_tolerance_mm: Rational::decimal("0.00001")?,
            maximum_tolerance_growth_mm: Rational::ZERO,
        })
    }

    fn build(bytes: &[u8]) -> Result<Candidate, Error> {
        build_with_operations(bytes, 100_000)
    }

    fn build_with_operations(bytes: &[u8], operations: usize) -> Result<Candidate, Error> {
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

    fn record(model: &Candidate, id: &str) -> usize {
        model
            .source()
            .source()
            .records()
            .iter()
            .position(|r| r.id == id)
            .unwrap()
    }

    fn close(actual: f64, expected: f64, absolute: f64) {
        assert!(actual.is_finite());
        assert!(
            (actual - expected).abs() <= absolute,
            "actual={actual:.17e}, expected={expected:.17e}, tolerance={absolute:.17e}"
        );
    }

    #[test]
    fn different_ring_phases_preserve_source_and_propagate_exact_pieces_to_the_cap() {
        let model = build(SOURCE).unwrap();
        assert_eq!(model.source().source().bytes(), SOURCE);
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (2, 2, 3)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (3, 4, 3)
        );
        assert_eq!(
            model.source().part_definition().unwrap().shape_role,
            "MaterialSolid"
        );

        let lower = record(&model, "130");
        let upper = record(&model, "131");
        let side = record(&model, "182");
        let upper_cap = *model.faces().get(&record(&model, "181")).unwrap();
        let side_face = *model.faces().get(&side).unwrap();
        let lower_vertex = *model.vertices().get(&record(&model, "120")).unwrap();
        let upper_vertex = *model.vertices().get(&record(&model, "121")).unwrap();
        assert_eq!(
            model.brep().vertex(lower_vertex).unwrap().point,
            Vec3::new(0., 3., 0.)
        );
        assert_eq!(
            model.brep().vertex(upper_vertex).unwrap().point,
            Vec3::new(3., 0., 7.)
        );
        assert_eq!(model.edges().get(&lower).unwrap().len(), 1);
        let pieces = model.edges().get(&upper).unwrap();
        assert_eq!(pieces.len(), 2);
        let first = model.brep().edge(pieces[0]).unwrap();
        let second = model.brep().edge(pieces[1]).unwrap();
        assert!(first.interval.length() > 0. && second.interval.length() > 0.);
        assert_eq!(first.interval.max(), second.interval.min());
        for piece in pieces {
            let faces = edge::faces(model.brep(), *piece, None)
                .unwrap()
                .into_iter()
                .collect::<BTreeSet<_>>();
            assert_eq!(faces, BTreeSet::from([upper_cap, side_face]));
        }

        let report = model.general_report().unwrap();
        let source_loops = report
            .loops
            .iter()
            .filter(|l| l.face == side)
            .collect::<Vec<_>>();
        assert_eq!(source_loops.len(), 2);
        assert!(source_loops
            .iter()
            .all(|l| !l.declared_outer && l.uses.len() == 1 && l.bound_orientation));
        assert_eq!(report.chart_cuts.len(), 1);
        assert_eq!(report.chart_cuts[0].0, side);
        assert_eq!(report.derived_vertices.len(), 1);
        let derived = &report.derived_vertices[0];
        assert_eq!(derived.source_edge, upper);
        assert_ne!(derived.native_vertex, lower_vertex);
        assert_ne!(derived.native_vertex, upper_vertex);
        let native_point = model.brep().vertex(derived.native_vertex).unwrap().point;
        assert!((native_point - Vec3::new(0., 3., 7.)).length() < 1e-12);
        assert_eq!(derived.native_parameter, first.interval.max());
        let basis = first.curve.as_ref().unwrap();
        assert!(
            (basis
                .evaluate_point(model.kernel(), derived.native_parameter)
                .unwrap()
                - native_point)
                .length()
                < 1e-12
        );

        // Each target piece retains the COMPLETE defining circle. Only Edge.interval
        // changes; slicing its NURB or replacing it by an arc would fail these checks.
        let CurveKind::Ellipse(first_circle) = basis.kind() else {
            panic!("source circle kind changed")
        };
        assert!(first_circle.is_circle_class());
        close(first_circle.radius(), 3., 0.);
        close(first_circle.step_interval().length(), 360., 0.);
        let basis_data = first_circle.base().gw_nurb().unwrap();
        let mapping = report
            .parameters
            .iter()
            .find(|m| m.source_use == upper && m.kind == "ConicRationalParameter")
            .unwrap();
        let natural = &mapping
            .parameters
            .iter()
            .find(|(name, _)| name == "native_natural_interval")
            .unwrap()
            .1;
        assert_eq!(
            (
                basis.natural_interval().min(),
                basis.natural_interval().max()
            ),
            (natural[0], natural[1])
        );
        assert_eq!(
            (first.interval.min(), second.interval.max()),
            (natural[0], natural[1])
        );
        let CurveKind::Ellipse(second_circle) = second.curve.as_ref().unwrap().kind() else {
            panic!("source circle kind changed")
        };
        assert!(second_circle.is_circle_class());
        assert_eq!(second_circle.step_interval(), first_circle.step_interval());
        let other_data = second_circle.base().gw_nurb().unwrap();
        assert_eq!(other_data.degree(), basis_data.degree());
        assert_eq!(other_data.knots(), basis_data.knots());
        assert_eq!(other_data.cpts(), basis_data.cpts());
    }

    #[test]
    fn phase_split_cylinder_completes_native_uv_and_independent_material_observations() {
        let model = build(SOURCE)
            .unwrap()
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        let report = model.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 8); // Side: five incidences; lower/upper caps: one/two.
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            (3, 4, 3)
        );
        // Analytic truth from elementary cylinder formulas, independent of all
        // construction, tessellation and numerical integration implementations.
        close(material.area_mm2, 60. * PI, 60. * PI * 1e-4);
        close(material.material_volume_mm3, 63. * PI, 63. * PI * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 3.5)).length() < 1e-5);
        close(material.boundary_mm, 12. * PI + 7., 1e-5);
        for (actual, expected) in [
            (material.bounds_mm.min(), Vec3::new(-3., -3., 0.)),
            (material.bounds_mm.max(), Vec3::new(3., 3., 7.)),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        let lower = record(&model, "130");
        let upper = record(&model, "131");
        assert_eq!(
            material
                .edges
                .iter()
                .filter(|e| e.source_edge == Some(lower))
                .count(),
            1
        );
        assert_eq!(
            material
                .edges
                .iter()
                .filter(|e| e.source_edge == Some(upper))
                .count(),
            2
        );
        assert_eq!(
            material
                .edges
                .iter()
                .filter(|e| e.source_edge.is_none())
                .count(),
            1
        );
        assert_eq!(model.source().source().bytes(), SOURCE);
    }

    #[test]
    fn incompatible_radius_and_axis_are_refused_without_moving_source_vertices() {
        let original = std::str::from_utf8(SOURCE).unwrap();
        let radius = original
            .replace("#111=CIRCLE('',#107,3.);", "#111=CIRCLE('',#107,4.);")
            .replace(
                "#103=CARTESIAN_POINT('',(3.,0.,7.));",
                "#103=CARTESIAN_POINT('',(4.,0.,7.));",
            );
        let offset = original.replace("#111=CIRCLE('',#107,3.);",
        "#108=CARTESIAN_POINT('',(1.,0.,7.));\n#109=AXIS2_PLACEMENT_3D('',#108,#104,#105);\n#111=CIRCLE('',#109,3.);")
        .replace("#103=CARTESIAN_POINT('',(3.,0.,7.));", "#103=CARTESIAN_POINT('',(4.,0.,7.));");
        for text in [radius, offset] {
            assert_ne!(text.as_bytes(), SOURCE);
            let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
            let source = s::check(
                &raw,
                s::Limits::fixtures(),
                &s::Options {
                    geometry_scope: s::GeometryScope::GeneralBrep,
                    check_part_and_closure: true,
                    check_geometry_dependencies: true,
                    ..Default::default()
                },
            );
            let source = source.snapshot.unwrap();
            assert_eq!(source.source().bytes(), text.as_bytes());
            let upper = source
                .source()
                .records()
                .iter()
                .position(|r| r.id == "131")
                .unwrap();
            let Err(failure) = build(text.as_bytes()) else {
                panic!("invalid periodic support accepted");
            };
            assert!(
                matches!(failure, Error::General { key: "GeneralImplicitRingSupport", record, kind: GeneralFailureKind::Postcheck } if record == upper),
                "{failure:?}"
            );
        }
        // Failed requests cannot rewrite the immutable source or contaminate a
        // subsequent owning request on the exact original public bytes.
        let replay = build(SOURCE).unwrap();
        assert_eq!(replay.source().source().bytes(), SOURCE);
    }

    /// Stable source/piece identity of upward uses; native arena numbers are not a
    /// comparison oracle across two independently owned construction requests.
    fn ring_directions(model: &Candidate, face_id: &str, edge_id: &str) -> Vec<(usize, bool)> {
        let native_face = model.faces()[&record(model, face_id)];
        let pieces = &model.edges()[&record(model, edge_id)];
        face::p4::upward_edgeuses(model.brep(), native_face, false, None)
            .unwrap()
            .iter()
            .filter_map(|id| {
                let use_ = model.brep().edgeuse(*id).unwrap();
                let native_edge = use_.edge.unwrap();
                pieces.iter().position(|p| *p == native_edge).map(|piece| {
                    assert!(matches!(
                        use_.orientation,
                        OrientType::Same | OrientType::Opposite
                    ));
                    (piece, use_.orientation == OrientType::Same)
                })
            })
            .collect()
    }

    #[test]
    fn every_phase_split_construction_budget_prefix_fails_cleanly_and_exact_budget_replays() {
        let complete = build(SOURCE).unwrap();
        let operations = complete.operations();
        assert!(operations > 0);
        assert_eq!(complete.derived_vertices().len(), 1);
        assert_eq!(complete.edges()[&record(&complete, "131")].len(), 2);
        drop(complete);

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
        // Enumerating the public budget prefixes covers failure before the split,
        // added vertex, complete-basis copy, topology and final handoff. The boundary
        // comes from the completed receipt, not a copy of private charging formulas.
        for limit in 0..operations {
            assert!(
                matches!(run(limit), Err(Error::ResourceLimit)),
                "construction published before the complete budget at {limit}/{operations}"
            );
            assert_eq!(raw.captured().unwrap(), SOURCE);
            let source = raw.source.as_ref().unwrap();
            assert_eq!(source.bytes(), SOURCE);
            assert_eq!(source.records(), original_records);
        }
        // In particular N-1 fails above, while exactly N owns the complete result.
        // Its counts distinguish a complete partition from a partial or first-piece
        // publication. A fresh parse/request must independently replay the same N.
        let exact = run(operations).unwrap();
        assert_eq!(exact.operations(), operations);
        assert_eq!(exact.source().source().bytes(), SOURCE);
        assert_eq!(exact.edges()[&record(&exact, "131")].len(), 2);
        assert_eq!(exact.derived_vertices().len(), 1);
        assert_eq!(
            (
                exact.brep().vertex_list().len(),
                exact.brep().edge_list().len(),
                exact.brep().face_list().len()
            ),
            (3, 4, 3)
        );
        let replay = build_with_operations(SOURCE, operations).unwrap();
        assert_eq!(replay.operations(), operations);
        assert_eq!(replay.source().source().bytes(), SOURCE);
        assert_eq!(
            ring_directions(&replay, "181", "131"),
            ring_directions(&exact, "181", "131")
        );
        assert_eq!(
            ring_directions(&replay, "182", "131"),
            ring_directions(&exact, "182", "131")
        );
    }

    #[test]
    fn reversing_edge_same_sense_and_both_occurrences_preserves_target_directions() {
        // On this CLOSED edge, swapping edge_start/edge_end is vacuous. Inverting
        // EDGE_CURVE.same_sense and BOTH ORIENTED_EDGE orientations leaves the
        // effective traversal of the unchanged defining circle mathematically equal.
        let text = std::str::from_utf8(SOURCE)
            .unwrap()
            .replace(
                "#131=EDGE_CURVE('',#121,#121,#111,.T.);",
                "#131=EDGE_CURVE('',#121,#121,#111,.F.);",
            )
            .replace(
                "#151=ORIENTED_EDGE('',*,*,#131,.T.);",
                "#151=ORIENTED_EDGE('',*,*,#131,.F.);",
            )
            .replace(
                "#154=ORIENTED_EDGE('',*,*,#131,.F.);",
                "#154=ORIENTED_EDGE('',*,*,#131,.T.);",
            );
        assert_ne!(text.as_bytes(), SOURCE);
        let baseline = build(SOURCE).unwrap();
        let reversed = build(text.as_bytes()).unwrap();
        assert_eq!(reversed.source().source().bytes(), text.as_bytes());
        assert_eq!(baseline.operations(), reversed.operations());
        for (id, name, expected) in [
            ("131", "same_sense", "F"),
            ("151", "orientation", "F"),
            ("154", "orientation", "T"),
        ] {
            let field = reversed
                .source()
                .field(record(&reversed, id), None, name)
                .unwrap();
            assert_eq!(field.value().unwrap().text, expected);
        }
        // Only these three declared senses changed. Every defining geometry and
        // source vertex record remains byte-identical to its independent input.
        for id in ["102", "103", "106", "107", "110", "111", "120", "121"] {
            let a = baseline.source().source();
            let b = reversed.source().source();
            assert_eq!(
                a.raw(a.find(id).unwrap().span).unwrap(),
                b.raw(b.find(id).unwrap().span).unwrap()
            );
        }
        let a = baseline.general_report().unwrap();
        let b = reversed.general_report().unwrap();
        assert_eq!(a.loops.len(), b.loops.len());
        for (original, other) in a.loops.iter().zip(b.loops) {
            assert_eq!(original.face, other.face);
            assert_eq!(original.bound, other.bound);
            assert_eq!(original.loop_record, other.loop_record);
            assert_eq!(original.bound_orientation, other.bound_orientation);
            assert_eq!(original.declared_outer, other.declared_outer);
            assert_eq!(original.uses, other.uses);
        }
        for face_id in ["181", "182"] {
            assert_eq!(
                ring_directions(&baseline, face_id, "131"),
                ring_directions(&reversed, face_id, "131")
            );
        }
        let upper = record(&reversed, "131");
        let pieces = &reversed.edges()[&upper];
        assert_eq!(pieces.len(), 2);
        let mut cap_uses = ring_directions(&reversed, "181", "131");
        // A closed cycle has no distinguished head; EndLoop retains its last
        // inserted edge. Rotate to piece zero while preserving directed order.
        let start = cap_uses.iter().position(|(piece, _)| *piece == 0).unwrap();
        cap_uses.rotate_left(start);
        assert_eq!(cap_uses, vec![(0, true), (1, true)]);
        let side_uses = ring_directions(&reversed, "182", "131");
        assert_eq!(side_uses.len(), 2);
        assert!(side_uses.iter().all(|(_, forward)| !forward));
        for (original, other) in baseline.edges()[&record(&baseline, "131")]
            .iter()
            .zip(pieces)
        {
            let a = baseline.brep().edge(*original).unwrap();
            let b = reversed.brep().edge(*other).unwrap();
            assert_eq!(a.interval, b.interval);
            let ca = a.curve.as_ref().unwrap();
            let cb = b.curve.as_ref().unwrap();
            assert_eq!(ca.natural_interval(), cb.natural_interval());
            for t in [b.interval.min(), b.interval.max()] {
                assert!(
                    (ca.evaluate_point(baseline.kernel(), t).unwrap()
                        - cb.evaluate_point(reversed.kernel(), t).unwrap())
                    .length()
                        < 1e-12
                );
            }
        }
        let checked = reversed
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap();
        assert_eq!(checked.source().source().bytes(), text.as_bytes());
        let trims = checked
            .general_report()
            .unwrap()
            .final_trim_measurements
            .unwrap();
        assert_eq!(trims.edge_uses, 8);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
    }

    #[test]
    fn opposite_circle_frame_keeps_source_normal_and_compensated_physical_traversal() {
        // Reversing the CIRCLE normal reverses its positive parameter direction.
        // Compensating with EDGE_CURVE.same_sense preserves the physical boundary;
        // the cap/cylinder placements and all ORIENTED_EDGE declarations stay fixed.
        // A new placement is necessary because the original also defines the cap.
        let text = std::str::from_utf8(SOURCE)
        .unwrap()
        .replace(
            "#111=CIRCLE('',#107,3.);",
            "#108=DIRECTION('',(0.,0.,-1.));\n#109=AXIS2_PLACEMENT_3D('',#101,#108,#105);\n#111=CIRCLE('',#109,3.);",
        )
        .replace(
            "#131=EDGE_CURVE('',#121,#121,#111,.T.);",
            "#131=EDGE_CURVE('',#121,#121,#111,.F.);",
        );
        let model = build(text.as_bytes()).unwrap();
        assert_eq!(model.source().source().bytes(), text.as_bytes());
        let upper = record(&model, "131");
        let circle_id = record(&model, "111");
        assert_eq!(
            model
                .source()
                .field(circle_id, None, "position")
                .unwrap()
                .reference_target(),
            Some(record(&model, "109"))
        );
        assert_eq!(
            model
                .source()
                .field(upper, None, "same_sense")
                .unwrap()
                .value()
                .unwrap()
                .text,
            "F"
        );
        let pieces = &model.edges()[&upper];
        assert_eq!(pieces.len(), 2);
        for piece in pieces {
            let native = model.brep().edge(*piece).unwrap();
            let curve = native.curve.as_ref().unwrap();
            let CurveKind::Ellipse(circle) = curve.kind() else {
                panic!("source circle kind changed")
            };
            let canonical = circle.canonical();
            assert_eq!(canonical.position.origin(), Vec3::new(0., 0., 7.));
            assert_eq!(canonical.position.x_axis(), Vec3::new(1., 0., 0.));
            assert_eq!(canonical.position.z_axis(), Vec3::new(0., 0., -1.));
            close(circle.radius(), 3., 0.);
            close(circle.step_interval().length(), 360., 0.);
        }
        let report = model.general_report().unwrap();
        assert_eq!(report.derived_vertices.len(), 1);
        let derived = &report.derived_vertices[0];
        assert_eq!(derived.source_edge, upper);
        assert!(
            (model.brep().vertex(derived.native_vertex).unwrap().point - Vec3::new(0., 3., 7.))
                .length()
                < 1e-12
        );
        let upper_vertex = model.vertices()[&record(&model, "121")];
        assert_eq!(
            model.brep().vertex(upper_vertex).unwrap().point,
            Vec3::new(3., 0., 7.)
        );
        let first = model.brep().edge(pieces[0]).unwrap();
        let second = model.brep().edge(pieces[1]).unwrap();
        assert_eq!(first.interval.max(), second.interval.min());
        assert_eq!(derived.native_parameter, first.interval.max());
        let curve = first.curve.as_ref().unwrap();
        assert_eq!(first.interval.min(), curve.natural_interval().min());
        assert_eq!(second.interval.max(), curve.natural_interval().max());
        assert!(
            (curve
                .evaluate_point(model.kernel(), derived.native_parameter)
                .unwrap()
                - Vec3::new(0., 3., 7.))
            .length()
                < 1e-12
        );
        // Actual target directions reverse relative to the now reversed circle
        // basis, so the cap still travels CCW in world XY and the side clockwise.
        let cap_uses = ring_directions(&model, "181", "131");
        let side_uses = ring_directions(&model, "182", "131");
        assert_eq!((cap_uses.len(), side_uses.len()), (2, 2));
        assert!(cap_uses.iter().all(|(_, forward)| !forward));
        assert!(side_uses.iter().all(|(_, forward)| *forward));
        let checked = model
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        assert_eq!(checked.source().source().bytes(), text.as_bytes());
        let report = checked.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 8);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        close(material.area_mm2, 60. * PI, 60. * PI * 1e-4);
        close(material.material_volume_mm3, 63. * PI, 63. * PI * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 3.5)).length() < 1e-5);
        close(material.boundary_mm, 12. * PI + 7., 1e-5);
    }

    fn replace_public_record(text: &str, original: &str, replacement: &str) -> String {
        assert_eq!(
            text.matches(original).count(),
            1,
            "public fixture spelling changed"
        );
        text.replace(original, replacement)
    }

    fn swap_public_ring_set(text: &str) -> String {
        // ADVANCED_FACE.bounds is a SET. Its textual order carries no boundary
        // role, traversal or geometric meaning; no sense is compensated here.
        replace_public_record(
            text,
            "#182=ADVANCED_FACE('',(#172,#173),#142,.T.);",
            "#182=ADVANCED_FACE('',(#173,#172),#142,.T.);",
        )
    }

    fn rotate_public_cylinder_x_90() -> String {
        // Independent exact rigid transform R_x(+90deg): (x,y,z)->(x,-z,y).
        // Both cap placements, their circles and the cylinder share these source
        // records. The origin #100 and reference X direction #105 stay fixed.
        let mut text = std::str::from_utf8(SOURCE).unwrap().to_owned();
        for (original, replacement) in [
            (
                "#101=CARTESIAN_POINT('',(0.,0.,7.));",
                "#101=CARTESIAN_POINT('',(0.,-7.,0.));",
            ),
            (
                "#102=CARTESIAN_POINT('',(0.,3.,0.));",
                "#102=CARTESIAN_POINT('',(0.,0.,3.));",
            ),
            (
                "#103=CARTESIAN_POINT('',(3.,0.,7.));",
                "#103=CARTESIAN_POINT('',(3.,-7.,0.));",
            ),
            (
                "#104=DIRECTION('',(0.,0.,1.));",
                "#104=DIRECTION('',(0.,-1.,0.));",
            ),
        ] {
            text = replace_public_record(&text, original, replacement);
        }
        text
    }

    fn assert_public_changed_records(bytes: &[u8], changed: &[&str]) {
        let baseline = inspect(SOURCE.to_vec(), Limits::planar_fixtures());
        let variant = inspect(bytes.to_vec(), Limits::planar_fixtures());
        let baseline = baseline.source.as_ref().unwrap();
        let variant = variant.source.as_ref().unwrap();
        assert_eq!(baseline.records().len(), variant.records().len());
        for original in baseline.records() {
            let other = variant.find(&original.id).unwrap();
            let original_bytes = baseline.raw(original.span).unwrap();
            let variant_bytes = variant.raw(other.span).unwrap();
            if changed.contains(&original.id.as_str()) {
                assert_ne!(
                    original_bytes, variant_bytes,
                    "unchanged declared record #{}",
                    original.id
                );
            } else {
                assert_eq!(
                    original_bytes, variant_bytes,
                    "unexpected change to #{}",
                    original.id
                );
            }
        }
    }

    /// Material traversal uses the face side bounding the non-void region.
    /// The primary face use always follows the surface normal, including the
    /// lower cap whose primary side belongs to the exterior void. Comparing two
    /// primary uses directly would mistake that source `same_sense` for a crack.
    fn material_ring_directions(
        model: &Candidate,
        face_id: &str,
        edge_id: &str,
    ) -> Vec<(usize, bool)> {
        let face = model.faces()[&record(model, face_id)];
        let primary = model.brep().face(face).unwrap().faceuse.unwrap();
        let shell = model.brep().faceuse(primary).unwrap().shell.unwrap();
        let region = model.brep().shell(shell).unwrap().region.unwrap();
        let primary_bounds_material = model.brep().region(region).unwrap().is_void == 0;
        ring_directions(model, face_id, edge_id)
            .into_iter()
            .map(|(piece, same)| (piece, same == primary_bounds_material))
            .collect()
    }

    fn assert_order_independent_ring_partition(model: &Candidate, rotated: bool) {
        let transform = |point: Vec3| {
            if rotated {
                Vec3::new(point.x, -point.z, point.y)
            } else {
                point
            }
        };
        assert_transformed_ring_partition(model, transform, transform, true);
    }

    fn assert_transformed_ring_partition(
        model: &Candidate,
        transform: impl Fn(Vec3) -> Vec3,
        transform_axis: impl Fn(Vec3) -> Vec3,
        exact_axes: bool,
    ) {
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (2, 2, 3)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (3, 4, 3)
        );
        assert_eq!(
            model.source().part_definition().unwrap().shape_role,
            "MaterialSolid"
        );
        for (source, point) in [
            ("120", Vec3::new(0., 3., 0.)),
            ("121", Vec3::new(3., 0., 7.)),
        ] {
            let vertex = model.vertices()[&record(model, source)];
            assert_eq!(model.brep().vertex(vertex).unwrap().point, transform(point));
        }
        let report = model.general_report().unwrap();
        let side_source = record(model, "182");
        let side_face = model.faces()[&side_source];
        let loops = report
            .loops
            .iter()
            .filter(|mapping| mapping.face == side_source)
            .collect::<Vec<_>>();
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|mapping| !mapping.declared_outer
            && mapping.uses.len() == 1
            && mapping.bound_orientation));
        assert_eq!(report.chart_cuts.len(), 1);
        assert_eq!(report.chart_cuts[0].0, side_source);
        assert_eq!(report.derived_vertices.len(), 1);
        let derived = &report.derived_vertices[0];
        assert!([record(model, "130"), record(model, "131")].contains(&derived.source_edge));
        assert!(!model
            .vertices()
            .values()
            .any(|vertex| *vertex == derived.native_vertex));
        let mut piece_counts = Vec::new();
        let mut ring_native_edges = BTreeSet::new();
        for (source_edge, cap_id) in [("130", "180"), ("131", "181")] {
            let source = record(model, source_edge);
            let pieces = &model.edges()[&source];
            piece_counts.push(pieces.len());
            let cap = model.faces()[&record(model, cap_id)];
            let mapping = report
                .parameters
                .iter()
                .find(|mapping| {
                    mapping.source_use == source && mapping.kind == "ConicRationalParameter"
                })
                .unwrap();
            let natural = &mapping
                .parameters
                .iter()
                .find(|(name, _)| name == "native_natural_interval")
                .unwrap()
                .1;
            let original_interval = &mapping
                .parameters
                .iter()
                .find(|(name, _)| name == "native_edge_interval")
                .unwrap()
                .1;
            assert_eq!(natural.len(), 2);
            assert_eq!(original_interval.len(), 2);
            let mut previous_end = original_interval[0];
            let mut basis_data = None;
            for piece in pieces {
                assert!(ring_native_edges.insert(*piece));
                let native = model.brep().edge(*piece).unwrap();
                assert!(native.interval.length() > 0.);
                assert_eq!(
                    native.interval.min(),
                    previous_end,
                    "incomplete or overlapping source ring #{source_edge}"
                );
                previous_end = native.interval.max();
                let curve = native.curve.as_ref().unwrap();
                assert_eq!(
                    (
                        curve.natural_interval().min(),
                        curve.natural_interval().max()
                    ),
                    (natural[0], natural[1])
                );
                let CurveKind::Ellipse(circle) = curve.kind() else {
                    panic!("complete source circle kind changed")
                };
                assert!(circle.is_circle_class());
                close(circle.radius(), 3., 0.);
                close(circle.step_interval().length(), 360., 0.);
                let center = if source_edge == "130" {
                    Vec3::ZERO
                } else {
                    Vec3::new(0., 0., 7.)
                };
                assert_eq!(circle.canonical().position.origin(), transform(center));
                let actual = circle.canonical().position;
                if exact_axes {
                    // Preserve the existing identity/exact-quarter-turn assertions.
                    assert_eq!(actual.z_axis(), transform_axis(Vec3::Z));
                    assert_eq!(actual.x_axis(), transform_axis(Vec3::X));
                } else {
                    // Decimal rotation coefficients are normalized by source
                    // build_axes. Their rounding remains far below the unchanged
                    // millimeter budgets; never alter native geometry to match them.
                    assert!((actual.z_axis() - transform_axis(Vec3::Z)).length() < 1e-12);
                    assert!((actual.x_axis() - transform_axis(Vec3::X)).length() < 1e-12);
                }
                let data = circle.base().gw_nurb().unwrap();
                if let Some((degree, knots, points)) = &basis_data {
                    assert_eq!(&data.degree(), degree);
                    assert_eq!(data.knots(), knots);
                    assert_eq!(data.cpts(), points);
                } else {
                    basis_data = Some((data.degree(), data.knots().to_vec(), data.cpts().to_vec()));
                }
                assert_eq!(
                    edge::faces(model.brep(), *piece, None)
                        .unwrap()
                        .into_iter()
                        .collect::<BTreeSet<_>>(),
                    BTreeSet::from([cap, side_face])
                );
            }
            assert_eq!(
                previous_end, original_interval[1],
                "source ring #{source_edge} lost its final interval"
            );
            // Every source piece is reused once by its cap and once by the side;
            // compare incidence identity and opposite traversal, not list heads.
            let cap_uses = material_ring_directions(model, cap_id, source_edge);
            let side_uses = material_ring_directions(model, "182", source_edge);
            assert_eq!(
                (cap_uses.len(), side_uses.len()),
                (pieces.len(), pieces.len())
            );
            for index in 0..pieces.len() {
                let cap_direction = cap_uses
                    .iter()
                    .filter(|(piece, _)| *piece == index)
                    .collect::<Vec<_>>();
                let side_direction = side_uses
                    .iter()
                    .filter(|(piece, _)| *piece == index)
                    .collect::<Vec<_>>();
                assert_eq!((cap_direction.len(), side_direction.len()), (1, 1));
                assert_ne!(cap_direction[0].1, side_direction[0].1);
            }
            if source == derived.source_edge {
                assert_eq!(pieces.len(), 2);
                let first = model.brep().edge(pieces[0]).unwrap();
                let second = model.brep().edge(pieces[1]).unwrap();
                assert_eq!(derived.native_parameter, first.interval.max());
                assert_eq!(derived.native_parameter, second.interval.min());
                let point = model.brep().vertex(derived.native_vertex).unwrap().point;
                assert!(
                    (first
                        .curve
                        .as_ref()
                        .unwrap()
                        .evaluate_point(model.kernel(), derived.native_parameter)
                        .unwrap()
                        - point)
                        .length()
                        < 1e-12
                );
            }
        }
        piece_counts.sort_unstable();
        assert_eq!(piece_counts, vec![1, 2]);
        assert_eq!(ring_native_edges.len(), 3);
        assert!(!ring_native_edges.contains(&report.chart_cuts[0].1));
        let cut_uses = face::p4::upward_edgeuses(model.brep(), side_face, false, None)
            .unwrap()
            .into_iter()
            .filter_map(|id| {
                let use_ = model.brep().edgeuse(id).unwrap();
                (use_.edge == Some(report.chart_cuts[0].1)).then_some(use_.orientation)
            })
            .collect::<Vec<_>>();
        assert_eq!(cut_uses.len(), 2);
        assert_ne!(cut_uses[0], cut_uses[1]);
    }

    fn assert_order_independent_cylinder_workflow(bytes: &[u8], rotated: bool) {
        let candidate = build(bytes).unwrap();
        assert_eq!(candidate.source().source().bytes(), bytes);
        // Complete public reports remain available if a new metamorphic control
        // exposes a genuine native rejection; no report or check is skipped.
        let checked = candidate.validate_native_detailed().unwrap_or_else(|failure| {
            panic!("public periodic cylinder validation: stage={:?}, completed={}, error={:?}, reports={:#?}", failure.stage, failure.completed, failure.error, failure.reports)
        }).measure_general_trims().unwrap().measure_general_material(1e-7).unwrap();
        assert_eq!(checked.source().source().bytes(), bytes);
        assert_order_independent_ring_partition(&checked, rotated);
        let report = checked.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 8);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            (3, 4, 3)
        );
        close(material.area_mm2, 60. * PI, 60. * PI * 1e-4);
        close(material.material_volume_mm3, 63. * PI, 63. * PI * 1e-4);
        close(material.boundary_mm, 12. * PI + 7., 1e-5);
        let (centroid, lower, upper) = if rotated {
            (
                Vec3::new(0., -3.5, 0.),
                Vec3::new(-3., -7., -3.),
                Vec3::new(3., 0., 3.),
            )
        } else {
            (
                Vec3::new(0., 0., 3.5),
                Vec3::new(-3., -3., 0.),
                Vec3::new(3., 3., 7.),
            )
        };
        assert!((material.centroid_mm - centroid).length() < 1e-5);
        for (actual, expected) in [
            (material.bounds_mm.min(), lower),
            (material.bounds_mm.max(), upper),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        assert_eq!(
            material
                .edges
                .iter()
                .filter(|edge| edge.source_edge.is_none())
                .count(),
            1
        );
        for source in [record(&checked, "130"), record(&checked, "131")] {
            assert_eq!(
                material
                    .edges
                    .iter()
                    .filter(|edge| edge.source_edge == Some(source))
                    .count(),
                checked.edges()[&source].len()
            );
        }
    }

    #[test]
    fn face_bound_set_order_preserves_complete_periodic_cylinder_workflow() {
        let text = swap_public_ring_set(std::str::from_utf8(SOURCE).unwrap());
        assert_public_changed_records(text.as_bytes(), &["182"]);
        assert_order_independent_cylinder_workflow(text.as_bytes(), false);
    }

    #[test]
    fn exact_x_quarter_turn_preserves_periodic_cylinder_workflow() {
        let text = rotate_public_cylinder_x_90();
        assert_public_changed_records(text.as_bytes(), &["101", "102", "103", "104"]);
        assert_order_independent_cylinder_workflow(text.as_bytes(), true);
    }

    #[test]
    fn face_bound_set_order_after_x_quarter_turn_preserves_periodic_cylinder_workflow() {
        let text = swap_public_ring_set(&rotate_public_cylinder_x_90());
        assert_public_changed_records(text.as_bytes(), &["101", "102", "103", "104", "182"]);
        assert_order_independent_cylinder_workflow(text.as_bytes(), true);
    }

    fn assert_rigid_cylinder_workflow(
        bytes: &[u8],
        transform_point: impl Fn(Vec3) -> Vec3,
        transform_axis: impl Fn(Vec3) -> Vec3,
        bounds: [Vec3; 2],
    ) {
        let checked = build(bytes).unwrap().validate_native_detailed().unwrap_or_else(|failure| {
            panic!("public rigid cylinder validation: stage={:?}, completed={}, error={:?}, reports={:#?}", failure.stage, failure.completed, failure.error, failure.reports)
        }).measure_general_trims().unwrap().measure_general_material(1e-7).unwrap();
        assert_eq!(checked.source().source().bytes(), bytes);
        assert_transformed_ring_partition(&checked, &transform_point, transform_axis, false);
        let report = checked.general_report().unwrap();
        assert_eq!(report.length_scale_mm, Rational::ONE);
        assert_eq!(
            report.quality.consistency_mm,
            quality().unwrap().consistency_mm
        );
        assert_eq!(
            report.quality.derived_uv_mm,
            quality().unwrap().derived_uv_mm
        );
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 8);
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        close(material.area_mm2, 60. * PI, 60. * PI * 1e-4);
        close(material.material_volume_mm3, 63. * PI, 63. * PI * 1e-4);
        close(material.boundary_mm, 12. * PI + 7., 1e-5);
        assert!((material.centroid_mm - transform_point(Vec3::new(0., 0., 3.5))).length() < 1e-5);
        for (actual, expected) in [
            (material.bounds_mm.min(), bounds[0]),
            (material.bounds_mm.max(), bounds[1]),
        ] {
            close(actual.x, expected.x, 1e-6);
            close(actual.y, expected.y, 1e-6);
            close(actual.z, expected.z, 1e-6);
        }
        // The bootstrap may choose either source ring. The derived point must
        // still be the opposite ring's physical phase, not a prescribed native ID
        // or an assumption that the textual first/last bound owns the split.
        let derived = &report.derived_vertices[0];
        let original_cut = if derived.source_edge == record(&checked, "130") {
            Vec3::new(3., 0., 0.)
        } else {
            assert_eq!(derived.source_edge, record(&checked, "131"));
            Vec3::new(0., 3., 7.)
        };
        let actual = checked.brep().vertex(derived.native_vertex).unwrap().point;
        assert!((actual - transform_point(original_cut)).length() < 1e-12);
    }

    #[test]
    fn whole_periodic_cylinder_y_forty_five_rotation_keeps_complete_native_checkpoints() {
        // R_y(+45): (x,y,z)->(c*x+c*z,y,-c*x+c*z). The cylinder's support
        // extrema are NOT a rotated axis-aligned box: each x/z radial extent is
        // 3/sqrt(2), while the axis runs from 0 to 7/sqrt(2).
        let c = 0.5_f64.sqrt();
        let rotate = |point: Vec3| {
            Vec3::new(
                c * point.x + c * point.z,
                point.y,
                -c * point.x + c * point.z,
            )
        };
        let center = rotate(Vec3::new(0., 0., 7.));
        let anchor = rotate(Vec3::new(3., 0., 7.));
        let mut text = std::str::from_utf8(SOURCE).unwrap().to_owned();
        for (original, replacement) in [
            (
                "#101=CARTESIAN_POINT('',(0.,0.,7.));",
                format!("#101=CARTESIAN_POINT('',({},0.,{}));", center.x, center.z),
            ),
            (
                "#103=CARTESIAN_POINT('',(3.,0.,7.));",
                format!("#103=CARTESIAN_POINT('',({},0.,{}));", anchor.x, anchor.z),
            ),
            (
                "#104=DIRECTION('',(0.,0.,1.));",
                format!("#104=DIRECTION('',({c},0.,{c}));"),
            ),
            (
                "#105=DIRECTION('',(1.,0.,0.));",
                format!("#105=DIRECTION('',({c},0.,{}));", -c),
            ),
        ] {
            text = replace_public_record(&text, original, &replacement);
        }
        // #100=(0,0,0) and #102=(0,3,0) are fixed by this authored rotation.
        assert_public_changed_records(text.as_bytes(), &["101", "103", "104", "105"]);
        assert_rigid_cylinder_workflow(
            text.as_bytes(),
            rotate,
            rotate,
            [
                Vec3::new(-3. * c, -3., -3. * c),
                Vec3::new(10. * c, 3., 10. * c),
            ],
        );
    }

    #[test]
    fn whole_periodic_cylinder_integer_translation_keeps_complete_native_checkpoints() {
        // Independent non-origin placement T=(11,-5,13) mm. Only point records
        // translate: directions, radii, units, source senses and budgets stay fixed.
        let shift = Vec3::new(11., -5., 13.);
        let mut text = std::str::from_utf8(SOURCE).unwrap().to_owned();
        for (original, replacement) in [
            (
                "#100=CARTESIAN_POINT('',(0.,0.,0.));",
                "#100=CARTESIAN_POINT('',(11.,-5.,13.));",
            ),
            (
                "#101=CARTESIAN_POINT('',(0.,0.,7.));",
                "#101=CARTESIAN_POINT('',(11.,-5.,20.));",
            ),
            (
                "#102=CARTESIAN_POINT('',(0.,3.,0.));",
                "#102=CARTESIAN_POINT('',(11.,-2.,13.));",
            ),
            (
                "#103=CARTESIAN_POINT('',(3.,0.,7.));",
                "#103=CARTESIAN_POINT('',(14.,-5.,20.));",
            ),
        ] {
            text = replace_public_record(&text, original, replacement);
        }
        assert_public_changed_records(text.as_bytes(), &["100", "101", "102", "103"]);
        assert_rigid_cylinder_workflow(
            text.as_bytes(),
            |point| point + shift,
            |axis| axis,
            [Vec3::new(8., -8., 13.), Vec3::new(14., -2., 20.)],
        );
    }
}
