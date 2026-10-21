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
}
