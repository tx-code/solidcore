// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public two-ring frustum controls, with independently specified physical truth.
//!
//! Radius r(z)=2+z millimeters, z=[0,4]. Exact elementary integrals give
//! V=208*pi/3, volume centroid z=34/13, and total area=(40+32*sqrt(2))*pi.
//! These are numerical observation screens, not source acceptance allowances.
//! Circles, source vertices and raw definitions survive equivalent chart cuts.
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

    const SOURCE: &str = include_str!("fixtures/cone-band.step");
    const OPERATIONS: usize = 100_000;

    fn quality() -> GeneralQuality {
        GeneralQuality {
            id: "Q-Public-Conical-Two-Rings".into(),
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
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap()
    }

    fn phase_split() -> String {
        SOURCE.replace(
            "#103=CARTESIAN_POINT('',(0.,6.,4.));",
            "#103=CARTESIAN_POINT('',(6.,0.,4.));",
        )
    }

    fn reverse_cone_axis(text: &str) -> String {
        // The same physical support: h'=-h, alpha'=-alpha, hence R'(h')=2+h.
        // Both angular and axial source parameters reverse, retaining the support
        // normal and the original FACE.same_sense. Circle definitions stay intact.
        text.replace("#142=CONICAL_SURFACE('',#106,2.,0.7853981633974483);",
        "#108=DIRECTION('',(0.,0.,-1.));\n#109=AXIS2_PLACEMENT_3D('',#100,#108,#105);\n#142=CONICAL_SURFACE('',#109,2.,-0.7853981633974483);")
    }

    fn apex_reference(text: &str) -> String {
        // Equivalent whole support with origin at the apex z=-2 and radius=0:
        // h=z+2, R(h)=h. Circle centres, radii, anchors and face senses stay put.
        text.replace("#142=CONICAL_SURFACE('',#106,2.,0.7853981633974483);",
        "#108=CARTESIAN_POINT('',(0.,0.,-2.));\n#109=AXIS2_PLACEMENT_3D('',#108,#104,#105);\n#142=CONICAL_SURFACE('',#109,0.,0.7853981633974483);")
    }

    fn paired_source_reversal(text: &str) -> String {
        // Reverse each source EDGE sense and both of its OE senses together.
        // Physical directed boundaries and all closed source vertex IDs stay put.
        text.replace(
            "#130=EDGE_CURVE('',#120,#120,#110,.T.);",
            "#130=EDGE_CURVE('',#120,#120,#110,.F.);",
        )
        .replace(
            "#131=EDGE_CURVE('',#121,#121,#111,.T.);",
            "#131=EDGE_CURVE('',#121,#121,#111,.F.);",
        )
        .replace(
            "#150=ORIENTED_EDGE('',*,*,#130,.F.);",
            "#150=ORIENTED_EDGE('',*,*,#130,.T.);",
        )
        .replace(
            "#152=ORIENTED_EDGE('',*,*,#130,.T.);",
            "#152=ORIENTED_EDGE('',*,*,#130,.F.);",
        )
        .replace(
            "#151=ORIENTED_EDGE('',*,*,#131,.T.);",
            "#151=ORIENTED_EDGE('',*,*,#131,.F.);",
        )
        .replace(
            "#154=ORIENTED_EDGE('',*,*,#131,.F.);",
            "#154=ORIENTED_EDGE('',*,*,#131,.T.);",
        )
    }

    fn centimeters(text: &str) -> String {
        // Change the source length carrier and every defining length coherently;
        // angles and selected millimeter accuracy/working budgets are unchanged.
        text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)")
            .replace(
                "#101=CARTESIAN_POINT('',(0.,0.,4.));",
                "#101=CARTESIAN_POINT('',(0.,0.,0.4));",
            )
            .replace(
                "#102=CARTESIAN_POINT('',(0.,2.,0.));",
                "#102=CARTESIAN_POINT('',(0.,0.2,0.));",
            )
            .replace(
                "#103=CARTESIAN_POINT('',(0.,6.,4.));",
                "#103=CARTESIAN_POINT('',(0.,0.6,0.4));",
            )
            .replace(
                "#103=CARTESIAN_POINT('',(6.,0.,4.));",
                "#103=CARTESIAN_POINT('',(0.6,0.,0.4));",
            )
            .replace("#110=CIRCLE('',#106,2.);", "#110=CIRCLE('',#106,0.2);")
            .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,0.6);")
            .replace(
                "#142=CONICAL_SURFACE('',#106,2.,0.7853981633974483);",
                "#142=CONICAL_SURFACE('',#106,0.2,0.7853981633974483);",
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

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "actual={actual:.17e}, expected={expected:.17e}, tolerance={tolerance:.17e}"
        );
    }

    fn directions(
        model: &Candidate,
        f: solidcore::topology::FaceId,
        e: solidcore::topology::EdgeId,
    ) -> Vec<OrientType> {
        // GetUpwardEdgeuses follows the surface-normal primary side, not
        // necessarily the material side. In this fixture the lower plane has +Z
        // support and FACE.same_sense=false, so its primary bounds exterior void.
        // Compare the actual non-void region traversal, as the public periodic-ring
        // controls do; primary-vs-primary would invent a shared-boundary crack.
        let primary = model.brep().face(f).unwrap().faceuse.unwrap();
        let shell = model.brep().faceuse(primary).unwrap().shell.unwrap();
        let region = model.brep().shell(shell).unwrap().region.unwrap();
        let primary_bounds_material = model.brep().region(region).unwrap().is_void == 0;
        face::p4::upward_edgeuses(model.brep(), f, false, None)
            .unwrap()
            .into_iter()
            .filter_map(|id| {
                let incidence = model.brep().edgeuse(id).unwrap();
                if incidence.edge != Some(e) {
                    return None;
                }
                Some(if primary_bounds_material {
                    incidence.orientation
                } else {
                    match incidence.orientation {
                        OrientType::Same => OrientType::Opposite,
                        OrientType::Opposite => OrientType::Same,
                        OrientType::Unknown => {
                            panic!("native material boundary has unknown orientation")
                        }
                    }
                })
            })
            .collect()
    }

    fn assert_correspondence(model: &Candidate, text: &str, split: bool, upper_at_x: bool) {
        assert_eq!(model.source().source().bytes(), text.as_bytes());
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
            (2, 2, 3)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            if split { (3, 4, 3) } else { (2, 3, 3) }
        );
        let side_source = record(model, "182");
        let side = model.faces()[&side_source];
        let actual_surface = model.brep().face(side).unwrap().surface.as_ref().unwrap();
        assert!(matches!(actual_surface.kind(), SurfaceKind::Cone(c) if !c.is_cylinder_class()));
        let report = model.general_report().unwrap();
        assert_eq!(report.chart_cuts.len(), 1);
        assert_eq!(report.chart_cuts[0].0, side_source);
        assert_eq!(report.derived_vertices.len(), usize::from(split));
        assert!(report.vertex_bounds.is_empty());
        let source_loops: Vec<_> = report
            .loops
            .iter()
            .filter(|l| l.face == side_source)
            .collect();
        assert_eq!(source_loops.len(), 2);
        assert!(source_loops
            .iter()
            .all(|l| l.source_vertex.is_none() && l.uses.len() == 1));
        let source_lower = model.vertices()[&record(model, "120")];
        let source_upper = model.vertices()[&record(model, "121")];
        assert_eq!(
            model.brep().vertex(source_lower).unwrap().point,
            Vec3::new(0., 2., 0.)
        );
        assert_eq!(
            model.brep().vertex(source_upper).unwrap().point,
            if upper_at_x {
                Vec3::new(6., 0., 4.)
            } else {
                Vec3::new(0., 6., 4.)
            }
        );
        let mut piece_counts = Vec::new();
        for (source_id, cap_id, radius, height) in [("130", "180", 2., 0.), ("131", "181", 6., 4.)]
        {
            let source = record(model, source_id);
            let cap = model.faces()[&record(model, cap_id)];
            let pieces = &model.edges()[&source];
            piece_counts.push(pieces.len());
            let mapping = report
                .parameters
                .iter()
                .find(|m| m.source_use == source && m.kind == "ConicRationalParameter")
                .unwrap();
            let parameter = |name: &str| {
                &mapping
                    .parameters
                    .iter()
                    .find(|(key, _)| key == name)
                    .unwrap()
                    .1
            };
            let original = parameter("native_edge_interval");
            let natural = parameter("native_natural_interval");
            let mut end = original[0];
            let mut basis = None;
            for piece in pieces {
                let native = model.brep().edge(*piece).unwrap();
                assert_eq!(native.interval.min(), end);
                assert!(native.interval.length() > 0.);
                end = native.interval.max();
                let defining = native.curve.as_ref().unwrap();
                assert_eq!(
                    (
                        defining.natural_interval().min(),
                        defining.natural_interval().max()
                    ),
                    (natural[0], natural[1])
                );
                let CurveKind::Ellipse(circle) = defining.kind() else {
                    panic!("source circle was replaced")
                };
                assert!(circle.is_circle_class());
                close(circle.radius(), radius, 1e-12);
                close(circle.step_interval().length(), 360., 1e-12);
                assert_eq!(
                    circle.canonical().position.origin(),
                    Vec3::new(0., 0., height)
                );
                let data = circle.base().gw_nurb().unwrap();
                let actual = (data.degree(), data.knots().to_vec(), data.cpts().to_vec());
                if let Some(before) = &basis {
                    assert_eq!(&actual, before);
                } else {
                    basis = Some(actual);
                }
                assert_eq!(
                    edge::faces(model.brep(), *piece, None)
                        .unwrap()
                        .into_iter()
                        .collect::<BTreeSet<_>>(),
                    BTreeSet::from([cap, side])
                );
                let cap_direction = directions(model, cap, *piece);
                let side_direction = directions(model, side, *piece);
                assert_eq!((cap_direction.len(), side_direction.len()), (1, 1));
                assert_ne!(cap_direction[0], side_direction[0]);
            }
            assert_eq!(end, original[1], "the source ring lost an interval");
        }
        piece_counts.sort_unstable();
        assert_eq!(piece_counts, if split { vec![1, 2] } else { vec![1, 1] });
        let seam = report.chart_cuts[0].1;
        let uses = directions(model, side, seam);
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0], uses[1]);
        for derived in report.derived_vertices {
            assert!(!model
                .vertices()
                .values()
                .any(|id| *id == derived.native_vertex));
            let pieces = &model.edges()[&derived.source_edge];
            assert_eq!(pieces.len(), 2);
            let a = model.brep().edge(pieces[0]).unwrap();
            let b = model.brep().edge(pieces[1]).unwrap();
            assert_eq!(a.interval.max(), derived.native_parameter);
            assert_eq!(b.interval.min(), derived.native_parameter);
            let point = model.brep().vertex(derived.native_vertex).unwrap().point;
            let expected = if derived.source_edge == record(model, "130") {
                Vec3::new(2., 0., 0.)
            } else {
                Vec3::new(0., 6., 4.)
            };
            assert!((point - expected).length() < 1e-7);
            assert!(
                (a.curve
                    .as_ref()
                    .unwrap()
                    .evaluate_point(model.kernel(), derived.native_parameter)
                    .unwrap()
                    - point)
                    .length()
                    < 1e-7
            );
        }
    }

    fn assert_measurements(model: &Candidate, split: bool) {
        let checks = model.native_checks().unwrap();
        assert_eq!((checks.pointer_reports, checks.deep_reports), (0, 0));
        let report = model.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, if split { 8 } else { 6 });
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let material = report.material_measurements.unwrap();
        let area = (40. + 32. * 2_f64.sqrt()) * PI;
        let volume = 208. * PI / 3.;
        close(material.area_mm2, area, area * 1e-4);
        close(material.material_volume_mm3, volume, volume * 1e-4);
        assert!((material.centroid_mm - Vec3::new(0., 0., 34. / 13.)).length() < 1e-5);
        close(material.boundary_mm, 16. * PI + 4. * 2_f64.sqrt(), 1e-5);
        assert!((material.bounds_mm.min() - Vec3::new(-6., -6., 0.)).length() < 1e-6);
        assert!((material.bounds_mm.max() - Vec3::new(6., 6., 4.)).length() < 1e-6);
        assert_eq!(
            (
                material.vertices,
                material.edges.len(),
                material.faces.len()
            ),
            if split { (3, 4, 3) } else { (2, 3, 3) }
        );
        assert_eq!(
            material
                .edges
                .iter()
                .filter(|e| e.source_edge.is_none())
                .count(),
            1
        );
        assert_eq!(material.requested_relative_accuracy, 1e-7);
        assert_eq!(material.applied_relative_accuracy, 1e-7);
    }

    #[test]
    fn same_phase_and_split_conical_bands_retain_sources_and_complete_independent_checkpoints() {
        for (text, split) in [(SOURCE.to_owned(), false), (phase_split(), true)] {
            let mut model = complete(&text);
            assert_correspondence(&model, &text, split, split);
            assert_measurements(&model, split);
            // Independent section z=2 has r=4. A seam hit may legitimately be an
            // edge; either kind must use actual mapped native topology.
            let hit = model
                .ray_hit(Vec3::new(10., 0., 2.), -Vec3::X, 1e-6, None)
                .unwrap()
                .unwrap();
            assert!((hit.point_mm - Vec3::new(4., 0., 2.)).length() < 1e-6);
            close(hit.distance_mm, 6., 1e-6);
        }
    }

    #[test]
    fn conical_bound_set_and_face_set_order_preserve_cap_piece_identity() {
        let text = phase_split()
            .replace(
                "#182=ADVANCED_FACE('',(#172,#173),#142,.T.);",
                "#182=ADVANCED_FACE('',(#173,#172),#142,.T.);",
            )
            .replace(
                "#22=CLOSED_SHELL('',(#180,#181,#182));",
                "#22=CLOSED_SHELL('',(#182,#181,#180));",
            );
        let model = complete(&text);
        assert_correspondence(&model, &text, true, true);
        assert_measurements(&model, true);
    }

    #[test]
    fn conical_axis_and_paired_source_reversals_preserve_geometry_and_material_role() {
        for (input, split) in [(SOURCE.to_owned(), false), (phase_split(), true)] {
            for text in [reverse_cone_axis(&input), paired_source_reversal(&input)] {
                let model = complete(&text);
                assert_correspondence(&model, &text, split, split);
                assert_measurements(&model, split);
            }
        }
    }

    #[test]
    fn zero_reference_radius_at_the_apex_preserves_the_same_positive_two_ring_band() {
        for (input, split) in [(SOURCE.to_owned(), false), (phase_split(), true)] {
            let text = apex_reference(&input);
            let model = complete(&text);
            assert_correspondence(&model, &text, split, split);
            assert_measurements(&model, split);
            assert!(std::str::from_utf8(model.source().source().bytes())
                .unwrap()
                .contains("#142=CONICAL_SURFACE('',#109,0.,0.7853981633974483);"));
        }
    }

    #[test]
    fn the_same_closed_band_as_a_legal_surface_model_does_not_gain_material_role() {
        for input in [SOURCE.to_owned(), phase_split()] {
            // Change the actual representation and item together, as a legal STEP
            // surface-model carrier requires. Keeping ADVANCED_BREP while changing
            // only its item would be a source type error rather than a role control.
            let text = input
                .replace(
                    "#20=ADVANCED_BREP_SHAPE_REPRESENTATION('body',(#21),#12);",
                    "#20=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('body',(#21),#12);",
                )
                .replace(
                    "#21=MANIFOLD_SOLID_BREP('',#22);",
                    "#21=SHELL_BASED_SURFACE_MODEL('',(#22));",
                );
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
            // Ownership::shape explicitly defers GeneralBrep region evidence for
            // a FACE_BOUND-only face (semantics/ownership.rs), after product and
            // dependency closure succeeded. Checker::run preserves that in part
            // and the final status. This is not a type/unit failure, nor permission
            // to report Checked before that independent region evidence exists.
            assert_eq!(checked.status, s::Status::Deferred);
            assert_eq!(checked.part, "Deferred");
            assert!(checked.diagnostics.is_empty());
            assert_eq!(
                (
                    checked.binding,
                    checked.types,
                    checked.unit_dependencies,
                    checked.geometry_dependencies,
                    checked.si_units,
                ),
                ("Passed", "Passed", "Passed", "Passed", "Passed")
            );
            assert!(checked.unit_dependency_checks.total > 0);
            assert!(checked.geometry_dependency_checks.total > 0);
            let source = checked.snapshot.as_ref().unwrap();
            assert_eq!(source.source().bytes(), text.as_bytes());
            let index = |id: &str| {
                source
                    .source()
                    .records()
                    .iter()
                    .position(|record| record.id == id)
                    .unwrap()
            };
            let part = source.part_definition().unwrap();
            assert_eq!(part.shape_role, "ClosedSurfaceModel");
            assert_eq!(part.deferred_region_faces, vec![index("182")]);
            assert_eq!(source.is_a(index("172"), "FACE_OUTER_BOUND"), Some(false));
            assert_eq!(source.is_a(index("173"), "FACE_OUTER_BOUND"), Some(false));
            assert_eq!(source.is_a(index("172"), "FACE_BOUND"), Some(true));
            assert_eq!(source.is_a(index("173"), "FACE_BOUND"), Some(true));
            assert_eq!(
                source.is_a(index("20"), "MANIFOLD_SURFACE_SHAPE_REPRESENTATION"),
                Some(true)
            );
            assert_eq!(
                source.is_a(index("21"), "SHELL_BASED_SURFACE_MODEL"),
                Some(true)
            );
            assert_eq!(source.is_a(index("21"), "MANIFOLD_SOLID_BREP"), Some(false));
            assert_eq!(source.is_a(index("22"), "CLOSED_SHELL"), Some(true));
            for (name, id) in [
                ("representation", "20"),
                ("shape", "21"),
                ("shell", "22"),
                ("unit_context", "12"),
            ] {
                assert_eq!(
                    part.roles
                        .iter()
                        .find(|role| role.name == name)
                        .unwrap()
                        .record_index,
                    index(id)
                );
            }
            // The source ownership/RI-filtered closure has completed independently
            // of the one deferred face region. Nothing required was discarded to
            // manufacture a legal surface-model carrier.
            for id in ["21", "22", "180", "181", "182", "110", "111", "142"] {
                assert_eq!(source.used_by_representation(index(id)), Some(true));
            }
            assert_eq!(source.units().context, Some(index("12")));
            assert_eq!(source.units().length_unit, Some(index("10")));
            assert_eq!(source.units().plane_angle_unit, Some(index("11")));
            assert_eq!(source.units().length_power10_mm, Some(0));
            let error = build(&text, OPERATIONS)
                .err()
                .expect("surface model gained material role");
            assert!(
            matches!(
                error,
                Error::General {
                    key: "GeneralRequiresMaterialSolid",
                    kind: GeneralFailureKind::Postcheck,
                    ..
                }
            ),
            "the source carrier must be legal before the explicit native role refusal: {error:?}"
        );
        }
    }

    #[test]
    fn centimeter_conical_band_keeps_raw_source_units_and_the_same_millimeter_truth() {
        for split in [false, true] {
            let input = if split {
                phase_split()
            } else {
                SOURCE.to_owned()
            };
            let text = centimeters(&input);
            let model = complete(&text);
            assert_correspondence(&model, &text, split, split);
            assert_measurements(&model, split);
            assert_eq!(
                model.general_report().unwrap().length_scale_mm,
                Rational::new(10, 1).unwrap()
            );
            assert!(std::str::from_utf8(model.source().source().bytes())
                .unwrap()
                .contains("SI_UNIT(.CENTI.,.METRE.)"));
        }
    }

    #[test]
    fn full_circle_support_rejects_wrong_radius_offset_tilt_zero_band_and_opposite_sheet() {
        let phase = phase_split();
        // Each coherent altered circle retains its own anchor. The sole semantic
        // defect is its relation to the unchanged conical support, not a bad point
        // reference or mismatched CIRCLE/EDGE vertex used to trigger an earlier gate.
        let cases = [
            (
                phase
                    .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,5.);")
                    .replace(
                        "#103=CARTESIAN_POINT('',(6.,0.,4.));",
                        "#103=CARTESIAN_POINT('',(5.,0.,4.));",
                    ),
                "GeneralImplicitRingSupport",
            ),
            (
                phase
                    .replace(
                        "#101=CARTESIAN_POINT('',(0.,0.,4.));",
                        "#101=CARTESIAN_POINT('',(0.01,0.,4.));",
                    )
                    .replace(
                        "#103=CARTESIAN_POINT('',(6.,0.,4.));",
                        "#103=CARTESIAN_POINT('',(6.01,0.,4.));",
                    ),
                "GeneralImplicitRingSupport",
            ),
            (
                phase.replace(
                    "#107=AXIS2_PLACEMENT_3D('',#101,#104,#105);",
                    "#108=DIRECTION('',(0.,1.,1.));\n#107=AXIS2_PLACEMENT_3D('',#101,#108,#105);",
                ),
                "GeneralImplicitRingSupport",
            ),
            (
                SOURCE
                    .replace(
                        "#101=CARTESIAN_POINT('',(0.,0.,4.));",
                        "#101=CARTESIAN_POINT('',(0.,0.,0.));",
                    )
                    .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,2.);")
                    .replace(
                        "#103=CARTESIAN_POINT('',(0.,6.,4.));",
                        "#103=CARTESIAN_POINT('',(0.,2.,0.));",
                    ),
                "GeneralImplicitSeamHeight",
            ),
            (
                phase
                    .replace(
                        "#101=CARTESIAN_POINT('',(0.,0.,4.));",
                        "#101=CARTESIAN_POINT('',(0.,0.,-4.));",
                    )
                    .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,2.);")
                    .replace(
                        "#103=CARTESIAN_POINT('',(6.,0.,4.));",
                        "#103=CARTESIAN_POINT('',(2.,0.,-4.));",
                    ),
                "GeneralImplicitRingSupport",
            ),
        ];
        for (text, expected) in cases {
            let original = text.clone();
            let error = build(&text, OPERATIONS)
                .err()
                .expect("invalid support published a candidate");
            assert!(
                matches!(error, Error::General { key, kind: GeneralFailureKind::Postcheck, .. } if key == expected),
                "expected {expected}: {error:?}"
            );
            assert_eq!(text, original);
        }
        // A failed request neither changes the source fixture nor poisons a replay.
        let replay = complete(&phase);
        assert_correspondence(&replay, &phase, true, true);
    }

    #[test]
    fn conical_aligned_and_split_construction_charge_the_exact_budget_and_replay_cleanly() {
        for (text, split) in [(SOURCE.to_owned(), false), (phase_split(), true)] {
            let baseline = build(&text, OPERATIONS).unwrap();
            let required = baseline.operations();
            assert!(required > 0);
            assert!(matches!(
                build(&text, required - 1),
                Err(Error::ResourceLimit)
            ));
            let exact = build(&text, required).unwrap();
            assert_eq!(exact.operations(), required);
            assert_correspondence(&exact, &text, split, split);
            let replay = complete(&text);
            assert_correspondence(&replay, &text, split, split);
            assert_measurements(&replay, split);
        }
    }
}
