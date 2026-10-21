// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public exact torus-latitude carriers with independent physical region truth.
//!
//! R=10,r=5,z=3 cuts circles rho=14 and rho=6. For beta=acos(3/5), the
//! crown has A=(160+200 beta) pi, V=(500 beta-240) pi, and centroid
//! z=(2560/3)/(500 beta-240). Source-directed long arcs select the complement,
//! not a guessed shortest meridian. Numerical screens are not import certificates.
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

    const SOURCE: &str = include_str!("fixtures/torus-latitude-band.step");
    const OPERATIONS: usize = 100_000;

    fn quality() -> GeneralQuality {
        GeneralQuality {
            id: "Q-Public-Torus-Latitude-Band".into(),
            definition_mm: Rational::decimal("0.0000001").unwrap(),
            consistency_mm: Rational::decimal("0.000001").unwrap(),
            derived_uv_mm: Rational::decimal("0.000001").unwrap(),
            query_mm: Rational::decimal("0.000001").unwrap(),
            maximum_work_tolerance_mm: Rational::decimal("0.00001").unwrap(),
            maximum_tolerance_growth_mm: Rational::ZERO,
        }
    }

    fn build(text: &str, operations: usize) -> Result<Candidate, Error> {
        let source = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        construct_general(
            &source,
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

    fn reverse_uses(text: &str) -> String {
        text.replace(
            "#150=ORIENTED_EDGE('',*,*,#130,.F.);",
            "#150=ORIENTED_EDGE('',*,*,#130,.T.);",
        )
        .replace(
            "#151=ORIENTED_EDGE('',*,*,#131,.T.);",
            "#151=ORIENTED_EDGE('',*,*,#131,.F.);",
        )
        .replace(
            "#152=ORIENTED_EDGE('',*,*,#130,.T.);",
            "#152=ORIENTED_EDGE('',*,*,#130,.F.);",
        )
        .replace(
            "#154=ORIENTED_EDGE('',*,*,#131,.F.);",
            "#154=ORIENTED_EDGE('',*,*,#131,.T.);",
        )
    }

    fn paired_edges_and_bounds() -> String {
        // Reverse source EDGE sense and each FACE_BOUND orientation together,
        // leaving OE declarations unchanged. These are separate source uses;
        // their combination retains every physical directed material boundary.
        SOURCE
            .replace(
                "#130=EDGE_CURVE('',#120,#120,#110,.T.);",
                "#130=EDGE_CURVE('',#120,#120,#110,.F.);",
            )
            .replace(
                "#131=EDGE_CURVE('',#121,#121,#111,.T.);",
                "#131=EDGE_CURVE('',#121,#121,#111,.F.);",
            )
            .replace(
                "#170=FACE_OUTER_BOUND('',#160,.T.);",
                "#170=FACE_OUTER_BOUND('',#160,.F.);",
            )
            .replace(
                "#171=FACE_BOUND('',#161,.T.);",
                "#171=FACE_BOUND('',#161,.F.);",
            )
            .replace(
                "#172=FACE_BOUND('',#162,.T.);",
                "#172=FACE_BOUND('',#162,.F.);",
            )
            .replace(
                "#173=FACE_BOUND('',#163,.T.);",
                "#173=FACE_BOUND('',#163,.F.);",
            )
    }

    fn torus_reference_y() -> String {
        // Only the torus's original #106 frame changes; #107 still owns the
        // cap/circle placement. No obsolete geometry is left orphaned.
        SOURCE.replace(
            "#106=AXIS2_PLACEMENT_3D('',#100,#104,#105);",
            "#108=DIRECTION('',(0.,1.,0.));\n#106=AXIS2_PLACEMENT_3D('',#100,#104,#108);",
        )
    }

    fn reversed_circle_frames() -> String {
        // Reversing only circle normals reverses their positive parameters.
        // Compensate by EDGE senses while retaining the cap's original frame
        // and all OE/BOUND declarations. The new frame is used by both circles.
        SOURCE.replace("#110=CIRCLE('',#107,14.);",
            "#108=DIRECTION('',(0.,0.,-1.));\n#109=AXIS2_PLACEMENT_3D('',#101,#108,#105);\n#110=CIRCLE('',#109,14.);")
            .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#109,6.);")
            .replace("#130=EDGE_CURVE('',#120,#120,#110,.T.);", "#130=EDGE_CURVE('',#120,#120,#110,.F.);")
            .replace("#131=EDGE_CURVE('',#121,#121,#111,.T.);", "#131=EDGE_CURVE('',#121,#121,#111,.F.);")
    }

    fn centimeters() -> String {
        SOURCE
            .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)")
            .replace(
                "#101=CARTESIAN_POINT('',(0.,0.,3.));",
                "#101=CARTESIAN_POINT('',(0.,0.,0.3));",
            )
            .replace(
                "#102=CARTESIAN_POINT('',(14.,0.,3.));",
                "#102=CARTESIAN_POINT('',(1.4,0.,0.3));",
            )
            .replace(
                "#103=CARTESIAN_POINT('',(6.,0.,3.));",
                "#103=CARTESIAN_POINT('',(0.6,0.,0.3));",
            )
            .replace("#110=CIRCLE('',#107,14.);", "#110=CIRCLE('',#107,1.4);")
            .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,0.6);")
            .replace(
                "#142=TOROIDAL_SURFACE('',#106,10.,5.);",
                "#142=TOROIDAL_SURFACE('',#106,1.,0.5);",
            )
    }

    fn lower_long_region() -> String {
        // Reverse the cap normal and all four OE uses, while retaining the
        // torus's SAME normal. The closed material is now z<=3: selecting the
        // shorter meridian would silently replace it with the crown.
        reverse_uses(SOURCE).replace(
            "#180=ADVANCED_FACE('',(#170,#171),#140,.F.);",
            "#180=ADVANCED_FACE('',(#170,#171),#140,.T.);",
        )
    }

    fn material_directions(
        model: &Candidate,
        f: solidcore::topology::FaceId,
        e: solidcore::topology::EdgeId,
    ) -> Vec<OrientType> {
        // Upward queries always use the surface-normal side. Compare actual
        // material-region walks, especially the crown's downward annular cap.
        let primary = model.brep().face(f).unwrap().faceuse.unwrap();
        let shell = model.brep().faceuse(primary).unwrap().shell.unwrap();
        let region = model.brep().shell(shell).unwrap().region.unwrap();
        let primary_material = model.brep().region(region).unwrap().is_void == 0;
        face::p4::upward_edgeuses(model.brep(), f, false, None)
            .unwrap()
            .into_iter()
            .filter_map(|id| {
                let use_ = model.brep().edgeuse(id).unwrap();
                if use_.edge != Some(e) {
                    return None;
                }
                Some(if primary_material {
                    use_.orientation
                } else {
                    match use_.orientation {
                        OrientType::Same => OrientType::Opposite,
                        OrientType::Opposite => OrientType::Same,
                        OrientType::Unknown => {
                            panic!("unknown native material boundary orientation")
                        }
                    }
                })
            })
            .collect()
    }

    fn assert_correspondence(model: &Candidate, text: &str, circles_reversed: bool) {
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
            (2, 2, 2)
        );
        assert_eq!(
            (
                model.brep().vertex_list().len(),
                model.brep().edge_list().len(),
                model.brep().face_list().len()
            ),
            (2, 3, 2)
        );
        assert!(model
            .brep()
            .vertex_list()
            .iter()
            .all(|v| model.vertices().values().any(|source| source == v)));
        for (id, expected) in [
            ("120", Vec3::new(14., 0., 3.)),
            ("121", Vec3::new(6., 0., 3.)),
        ] {
            assert_eq!(
                model
                    .brep()
                    .vertex(model.vertices()[&record(model, id)])
                    .unwrap()
                    .point,
                expected
            );
        }
        let cap = model.faces()[&record(model, "180")];
        let torus = model.faces()[&record(model, "182")];
        assert!(matches!(
            model
                .brep()
                .face(torus)
                .unwrap()
                .surface
                .as_ref()
                .unwrap()
                .kind(),
            SurfaceKind::Torus(_)
        ));
        let report = model.general_report().unwrap();
        assert!(report.derived_vertices.is_empty());
        assert!(report.vertex_bounds.is_empty());
        assert_eq!(report.chart_cuts.len(), 1);
        assert_eq!(report.chart_cuts[0].0, record(model, "182"));
        let loops: Vec<_> = report
            .loops
            .iter()
            .filter(|l| l.face == record(model, "182"))
            .collect();
        assert_eq!(loops.len(), 2);
        assert!(loops
            .iter()
            .all(|l| l.source_vertex.is_none() && l.uses.len() == 1));
        let mut original_edges = BTreeSet::new();
        for (id, radius) in [("130", 14.), ("131", 6.)] {
            let source = record(model, id);
            let pieces = &model.edges()[&source];
            assert_eq!(
                pieces.len(),
                1,
                "aligned-only normalization must preserve the original ring"
            );
            let edge_id = pieces[0];
            assert!(original_edges.insert(edge_id));
            let native = model.brep().edge(edge_id).unwrap();
            let curve = native.curve.as_ref().unwrap();
            let CurveKind::Ellipse(circle) = curve.kind() else {
                panic!("source circle was replaced")
            };
            assert!(circle.is_circle_class());
            close(circle.radius(), radius, 1e-12);
            close(circle.step_interval().length(), 360., 1e-12);
            assert_eq!(circle.canonical().position.origin(), Vec3::new(0., 0., 3.));
            assert_eq!(
                circle.canonical().position.z_axis(),
                if circles_reversed { -Vec3::Z } else { Vec3::Z }
            );
            let mapping = report
                .parameters
                .iter()
                .find(|m| m.source_use == source && m.kind == "ConicRationalParameter")
                .unwrap();
            let natural = &mapping
                .parameters
                .iter()
                .find(|(key, _)| key == "native_natural_interval")
                .unwrap()
                .1;
            let interval = &mapping
                .parameters
                .iter()
                .find(|(key, _)| key == "native_edge_interval")
                .unwrap()
                .1;
            assert_eq!(
                (
                    curve.natural_interval().min(),
                    curve.natural_interval().max()
                ),
                (natural[0], natural[1])
            );
            assert_eq!(
                (native.interval.min(), native.interval.max()),
                (interval[0], interval[1])
            );
            assert_eq!(
                edge::faces(model.brep(), edge_id, None)
                    .unwrap()
                    .into_iter()
                    .collect::<BTreeSet<_>>(),
                BTreeSet::from([cap, torus])
            );
            let cap_use = material_directions(model, cap, edge_id);
            let side_use = material_directions(model, torus, edge_id);
            assert_eq!((cap_use.len(), side_use.len()), (1, 1));
            assert_ne!(cap_use[0], side_use[0]);
        }
        let meridian = report.chart_cuts[0].1;
        assert!(!original_edges.contains(&meridian));
        assert_eq!(
            edge::faces(model.brep(), meridian, None)
                .unwrap()
                .into_iter()
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([torus])
        );
        let seam_uses = material_directions(model, torus, meridian);
        assert_eq!(seam_uses.len(), 2);
        assert_ne!(seam_uses[0], seam_uses[1]);
    }

    fn assert_minor_circle_cut(model: &Candidate, lower: bool) {
        let beta = (3_f64 / 5.).acos();
        let id = model.general_report().unwrap().chart_cuts[0].1;
        let native = model.brep().edge(id).unwrap();
        let curve = native.curve.as_ref().unwrap();
        let CurveKind::Ellipse(circle) = curve.kind() else {
            panic!("toroidal cut is not its analytic minor circle")
        };
        assert!(circle.is_circle_class());
        close(circle.radius(), 5., 1e-12);
        let frame = circle.canonical().position;
        assert!((frame.origin() - Vec3::new(10., 0., 0.)).length() < 1e-7);
        close(
            circle.step_interval().length(),
            (if lower {
                2. * PI - 2. * beta
            } else {
                2. * beta
            }) * 180.
                / PI,
            1e-8,
        );
        assert_eq!(native.interval, curve.natural_interval());
        let desired = Vec3::new(10., 0., if lower { -5. } else { 5. });
        let delta = desired - frame.origin();
        let geometric_angle =
            delta.dot(frame.y_axis()).atan2(delta.dot(frame.x_axis())) * 180. / PI;
        let angular = circle.step_interval();
        let angle = geometric_angle + 360. * ((angular.min() - geometric_angle) / 360.).ceil();
        assert!(angular.min() < angle && angle < angular.max());
        // Convert the physical polar angle through the actual conic mapping;
        // a NURBS parameter is never assumed to equal degrees or arc fraction.
        let parameter = circle
            .convert_t_from_step_to_nurbs(model.kernel(), angle)
            .unwrap();
        assert!(native.interval.min() < parameter && parameter < native.interval.max());
        assert!(
            (curve.evaluate_point(model.kernel(), parameter).unwrap() - desired).length() < 1e-7
        );
        let mut ends = Vec::new();
        for fraction in [0., 0.25, 0.5, 0.75, 1.] {
            let point = curve
                .evaluate_point(model.kernel(), native.interval.evaluate(fraction))
                .unwrap();
            close(point.y, 0., 1e-7);
            close((point - Vec3::new(10., 0., 0.)).length(), 5., 1e-7);
            if lower {
                assert!(point.z <= 3. + 1e-7);
            } else {
                assert!(point.z >= 3. - 1e-7);
            }
            if fraction == 0. || fraction == 1. {
                ends.push(point);
            }
        }
        for point in &ends {
            assert!(
                (*point - Vec3::new(14., 0., 3.)).length() < 1e-7
                    || (*point - Vec3::new(6., 0., 3.)).length() < 1e-7
            );
        }
        close((ends[0] - ends[1]).length(), 8., 1e-7);
    }

    fn assert_material(model: &Candidate, lower: bool) {
        let beta = (3_f64 / 5.).acos();
        let checks = model.native_checks().unwrap();
        assert_eq!((checks.pointer_reports, checks.deep_reports), (0, 0));
        let report = model.general_report().unwrap();
        let trims = report.final_trim_measurements.unwrap();
        assert_eq!(trims.edge_uses, 6); // annular cap: two; torus: two rings + paired meridian
        assert!(trims.maximum_mm.is_finite() && trims.maximum_mm <= 1e-6);
        let actual = report.material_measurements.unwrap();
        let crown_volume = (500. * beta - 240.) * PI;
        let volume = if lower {
            500. * PI * PI - crown_volume
        } else {
            crown_volume
        };
        let area = if lower {
            200. * PI * PI - 200. * beta * PI + 160. * PI
        } else {
            (160. + 200. * beta) * PI
        };
        let moment_z = 2560. * PI / 3.;
        let centroid_z = if lower {
            -moment_z / volume
        } else {
            moment_z / volume
        };
        // Independent numerical truth screens: the requested 1e-7 integration
        // control is not a certified error bound. Keep these dimensionless
        // observation screens separate from the unchanged mm-based source/UV
        // budgets, and from continuous import qualification.
        close(actual.material_volume_mm3, volume, volume * 1e-6);
        close(actual.area_mm2, area, area * 1e-6);
        assert!((actual.centroid_mm - Vec3::new(0., 0., centroid_z)).length() < 1e-5);
        close(
            actual.boundary_mm,
            40. * PI + if lower { 10. * (PI - beta) } else { 10. * beta },
            1e-5,
        );
        let (radius, z_min, z_max) = if lower { (15., -5., 3.) } else { (14., 3., 5.) };
        assert!((actual.bounds_mm.min() - Vec3::new(-radius, -radius, z_min)).length() < 1e-6);
        assert!((actual.bounds_mm.max() - Vec3::new(radius, radius, z_max)).length() < 1e-6);
        assert_eq!(
            (actual.vertices, actual.edges.len(), actual.faces.len()),
            (2, 3, 2)
        );
        assert_eq!(
            actual
                .edges
                .iter()
                .filter(|edge| edge.source_edge.is_none())
                .count(),
            1
        );
        assert_eq!(actual.requested_relative_accuracy, 1e-7);
        assert_eq!(actual.applied_relative_accuracy, 1e-7);
    }

    fn source_role(text: &str, role: &str) -> s::Snapshot {
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        let outcome = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                geometry_scope: s::GeometryScope::GeneralBrep,
                check_geometry_dependencies: true,
                check_part_and_closure: true,
                ..s::Options::default()
            },
        );
        assert_eq!(
            outcome.status,
            s::Status::Deferred,
            "{:?}",
            outcome.diagnostics
        );
        assert_eq!(outcome.part, "Deferred");
        assert_eq!(
            (
                outcome.binding,
                outcome.types,
                outcome.unit_dependencies,
                outcome.geometry_dependencies,
                outcome.si_units
            ),
            ("Passed", "Passed", "Passed", "Passed", "Passed")
        );
        assert!(outcome.diagnostics.is_empty());
        let source = outcome.snapshot.unwrap();
        assert_eq!(source.source().bytes(), text.as_bytes());
        let side = source
            .source()
            .records()
            .iter()
            .position(|r| r.id == "182")
            .unwrap();
        assert_eq!(
            source.part_definition().unwrap().deferred_region_faces,
            vec![side]
        );
        assert_eq!(source.part_definition().unwrap().shape_role, role);
        assert_eq!(source.used_by_representation(side), Some(true));
        source
    }

    fn meridian_quarter_tube() -> String {
        // Two genuine torus meridians: radius 5 centred at +10X / +10Y,
        // in XZ / YZ planes, plus their two correctly directed disk caps.
        // This is outside the latitude-only normalization, not an invalid
        // annulus obtained by tilting a circle away from the torus support.
        SOURCE.replace("#22=CLOSED_SHELL('',(#180,#182));", "#22=CLOSED_SHELL('',(#180,#181,#182));")
            .replace("#101=CARTESIAN_POINT('',(0.,0.,3.));", "#101=CARTESIAN_POINT('',(10.,0.,0.));")
            .replace("#102=CARTESIAN_POINT('',(14.,0.,3.));", "#102=CARTESIAN_POINT('',(15.,0.,0.));")
            .replace("#103=CARTESIAN_POINT('',(6.,0.,3.));", "#103=CARTESIAN_POINT('',(0.,15.,0.));")
            .replace("#107=AXIS2_PLACEMENT_3D('',#101,#104,#105);",
                "#108=DIRECTION('',(0.,1.,0.));\n#109=CARTESIAN_POINT('',(0.,10.,0.));\n#112=AXIS2_PLACEMENT_3D('',#109,#105,#108);\n#107=AXIS2_PLACEMENT_3D('',#101,#108,#105);")
            .replace("#110=CIRCLE('',#107,14.);", "#110=CIRCLE('',#107,5.);")
            .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#112,5.);")
            .replace("#140=PLANE('',#107);", "#140=PLANE('',#107);\n#141=PLANE('',#112);")
            .replace("#151=ORIENTED_EDGE('',*,*,#131,.T.);", "#151=ORIENTED_EDGE('',*,*,#131,.F.);")
            .replace("#154=ORIENTED_EDGE('',*,*,#131,.F.);", "#154=ORIENTED_EDGE('',*,*,#131,.T.);")
            .replace("#171=FACE_BOUND('',#161,.T.);", "#171=FACE_OUTER_BOUND('',#161,.T.);")
            .replace("#180=ADVANCED_FACE('',(#170,#171),#140,.F.);",
                "#180=ADVANCED_FACE('',(#170),#140,.F.);\n#181=ADVANCED_FACE('',(#171),#141,.F.);")
    }

    #[test]
    fn toroidal_latitude_crown_keeps_original_circles_and_independent_material_truth() {
        let model = complete(SOURCE);
        assert_correspondence(&model, SOURCE, false);
        assert_minor_circle_cut(&model, false);
        assert_material(&model, false);
    }

    #[test]
    fn toroidal_bound_set_order_and_torus_only_reference_axis_preserve_the_same_region() {
        let order = SOURCE
            .replace(
                "#182=ADVANCED_FACE('',(#172,#173),#142,.T.);",
                "#182=ADVANCED_FACE('',(#173,#172),#142,.T.);",
            )
            .replace(
                "#180=ADVANCED_FACE('',(#170,#171),#140,.F.);",
                "#180=ADVANCED_FACE('',(#171,#170),#140,.F.);",
            );
        for text in [order, torus_reference_y()] {
            let model = complete(&text);
            assert_correspondence(&model, &text, false);
            assert_minor_circle_cut(&model, false);
            assert_material(&model, false);
        }
    }

    #[test]
    fn paired_edge_bound_and_circle_axis_reversals_preserve_physical_material_walks() {
        for (text, opposite_circle_axes) in [
            (paired_edges_and_bounds(), false),
            (reversed_circle_frames(), true),
        ] {
            let model = complete(&text);
            assert_correspondence(&model, &text, opposite_circle_axes);
            assert_minor_circle_cut(&model, false);
            assert_material(&model, false);
        }
    }

    #[test]
    fn toroidal_centimeter_source_keeps_its_raw_units_and_millimeter_truth() {
        let text = centimeters();
        let model = complete(&text);
        assert_correspondence(&model, &text, false);
        assert_minor_circle_cut(&model, false);
        assert_material(&model, false);
        assert_eq!(
            model.general_report().unwrap().length_scale_mm,
            Rational::new(10, 1).unwrap()
        );
        assert!(std::str::from_utf8(model.source().source().bytes())
            .unwrap()
            .contains("SI_UNIT(.CENTI.,.METRE.)"));
    }

    #[test]
    fn source_directed_long_v_arc_preserves_the_large_lower_torus_region() {
        let text = lower_long_region();
        let model = complete(&text);
        assert_correspondence(&model, &text, false);
        assert_minor_circle_cut(&model, true);
        assert_material(&model, true);
    }

    #[test]
    fn closed_toroidal_surface_model_cannot_gain_a_material_role() {
        let text = SOURCE
            .replace(
                "#20=ADVANCED_BREP_SHAPE_REPRESENTATION('body',(#21),#12);",
                "#20=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('body',(#21),#12);",
            )
            .replace(
                "#21=MANIFOLD_SOLID_BREP('',#22);",
                "#21=SHELL_BASED_SURFACE_MODEL('',(#22));",
            );
        let source = source_role(&text, "ClosedSurfaceModel");
        let item = source
            .source()
            .records()
            .iter()
            .position(|r| r.id == "21")
            .unwrap();
        assert_eq!(source.is_a(item, "SHELL_BASED_SURFACE_MODEL"), Some(true));
        assert_eq!(source.is_a(item, "MANIFOLD_SOLID_BREP"), Some(false));
        let error = build(&text, OPERATIONS)
            .err()
            .expect("surface model was promoted to material");
        assert!(matches!(
            error,
            Error::General {
                key: "GeneralRequiresMaterialSolid",
                kind: GeneralFailureKind::Postcheck,
                ..
            }
        ));
    }

    #[test]
    fn unaligned_latitudes_and_real_meridians_are_explicit_unsupported_scope() {
        let phase = SOURCE.replace(
            "#103=CARTESIAN_POINT('',(6.,0.,3.));",
            "#103=CARTESIAN_POINT('',(0.,6.,3.));",
        );
        for (text, key) in [
            (phase, "GeneralToroidalRingPhaseNotYetConstructed"),
            (
                meridian_quarter_tube(),
                "GeneralToroidalNonLatitudeRingNotYetConstructed",
            ),
        ] {
            source_role(&text, "MaterialSolid");
            let error = build(&text, OPERATIONS)
                .err()
                .expect("unsupported region published a candidate");
            assert!(
                matches!(error, Error::General { key: actual, kind: GeneralFailureKind::Unsupported, .. } if actual == key),
                "{error:?}"
            );
        }
    }

    #[test]
    fn coherent_circle_with_wrong_torus_support_is_refused_without_repair() {
        let text = SOURCE
            .replace("#111=CIRCLE('',#107,6.);", "#111=CIRCLE('',#107,7.);")
            .replace(
                "#103=CARTESIAN_POINT('',(6.,0.,3.));",
                "#103=CARTESIAN_POINT('',(7.,0.,3.));",
            );
        source_role(&text, "MaterialSolid");
        let original = text.clone();
        let error = build(&text, OPERATIONS)
            .err()
            .expect("off-support circle was silently changed");
        assert!(matches!(
            error,
            Error::General {
                key: "GeneralToroidalRingSupport",
                kind: GeneralFailureKind::Postcheck,
                ..
            }
        ));
        assert_eq!(text, original);
    }

    #[test]
    fn toroidal_construction_budget_n_minus_one_n_and_clean_replay_keep_source_bytes() {
        let baseline = build(SOURCE, OPERATIONS).unwrap();
        let required = baseline.operations();
        assert!(required > 0);
        assert!(matches!(
            build(SOURCE, required - 1),
            Err(Error::ResourceLimit)
        ));
        let exact = build(SOURCE, required).unwrap();
        assert_eq!(exact.operations(), required);
        assert_correspondence(&exact, SOURCE, false);
        assert_minor_circle_cut(&exact, false);
        let replay = complete(SOURCE);
        assert_correspondence(&replay, SOURCE, false);
        assert_material(&replay, false);
    }
}
