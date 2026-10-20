// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent source parameter/graph truth, not derived from C++ observations.
use solidcore_step::{inspect, semantics, Limits};
use std::fmt::Write;
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
fn source(data: &str) -> solidcore_step::Outcome {
    inspect(format!("ISO-10303-21;HEADER;FILE_DESCRIPTION(('public synthetic'),'2;1');FILE_NAME('public.step','',(''),(''),'','','');FILE_SCHEMA(('{AP214}'));ENDSEC;DATA;{data}ENDSEC;END-ISO-10303-21;").into_bytes(),Limits::planar_fixtures())
}
fn run(data: &str) -> (solidcore_step::Outcome, semantics::Outcome) {
    let raw = source(data);
    let result = semantics::check(
        &raw,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_geometry_dependencies: true,
            ..semantics::Options::default()
        },
    );
    (raw, result)
}
const POSITION: &str = "#2=AXIS2_PLACEMENT_3D('',#3,$,$);#3=CARTESIAN_POINT('',(0.,0.,0.));";
const SPLINE:&str="#1=B_SPLINE_CURVE_WITH_KNOTS('source',1,(#2,#3),.UNSPECIFIED.,.F.,.U.,(2,2),(0.,1.),.UNSPECIFIED.);#2=CARTESIAN_POINT('',(0.,0.,0.));#3=CARTESIAN_POINT('',(10.,0.,0.));";
const SURFACE:&str="#1=(BOUNDED_SURFACE() B_SPLINE_SURFACE(1,1,((#2,#2),(#2,#2)),.UNSPECIFIED.,.F.,.F.,.U.) B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.,2.),(3.,4.))) REPRESENTATION_ITEM('source rational') SURFACE());#2=CARTESIAN_POINT('',(0.,0.,0.));";

#[test]
fn source_radius_predicates_are_exact_and_do_not_clamp_cone_angles() {
    let (_, circle) = run(&format!("#1=CIRCLE('',#2,-0.);{POSITION}"));
    assert_eq!(circle.status, semantics::Status::Rejected);
    let d = &circle.diagnostics[0];
    assert_eq!(
        d.source.key,
        "SourceRuleViolation:POSITIVE_LENGTH_MEASURE.WR1"
    );
    assert_eq!(d.field, "radius");
    assert_eq!(d.source.parameter_path, [3]);
    assert_eq!(d.actual, "-0.");
    assert_eq!(circle.geometry_dependency_checks.type_predicates, 1);
    let (_, cone) = run(&format!(
        "#1=CONICAL_SURFACE('',#2,0.,1.E99999999999999999999);{POSITION}"
    ));
    assert_eq!(cone.status, semantics::Status::Checked);
    assert_eq!(cone.geometry_dependency_checks.entity_predicates, 1);
    let (_, bad) = run(&format!("#1=CONICAL_SURFACE('',#2,-1.,0.5);{POSITION}"));
    assert_eq!(
        bad.diagnostics[0].source.key,
        "SourceRuleViolation:CONICAL_SURFACE.WR1"
    );
    let (_, tiny) = run(&format!(
        "#1=ELLIPSE('',#2,1.E-99999999999999999999,2.);{POSITION}"
    ));
    assert_eq!(tiny.status, semantics::Status::Checked);
}

#[test]
fn spline_parameters_stay_original_and_fail_at_their_own_slots() {
    let (raw, result) = run(SPLINE);
    assert_eq!(result.status, semantics::Status::Checked);
    assert_eq!(result.geometry_dependencies, "Passed");
    assert_eq!(result.part, "NotRun");
    assert_eq!(
        result.snapshot.unwrap().source().bytes(),
        raw.captured().unwrap()
    );
    for (from, to, key, path) in [
        (
            "'source',1,",
            "'source',0,",
            "SourceRuleViolation:B_SPLINE_CURVE_WITH_KNOTS.WR1",
            vec![2],
        ),
        (
            "(2,2),(0.,1.)",
            "(2,2),(0.,0.5,1.)",
            "SourceRuleViolation:B_SPLINE_CURVE_WITH_KNOTS.WR2",
            vec![7],
        ),
        (
            "(2,2),(0.,1.)",
            "(3,1),(0.,1.)",
            "SourceRuleViolation:B_SPLINE_CURVE_WITH_KNOTS.WR1",
            vec![7, 1],
        ),
        (
            "(2,2),(0.,1.)",
            "(2,2),(1.,0.)",
            "SourceRuleViolation:B_SPLINE_CURVE_WITH_KNOTS.WR1",
            vec![8, 2],
        ),
        (
            "(2,2),(0.,1.)",
            "(2,2),(0.,0.E99)",
            "SourceRuleViolation:B_SPLINE_CURVE_WITH_KNOTS.WR1",
            vec![8, 2],
        ),
    ] {
        let (_, bad) = run(&SPLINE.replace(from, to));
        let d = &bad.diagnostics[0];
        assert_eq!(d.source.key, key);
        assert_eq!(d.source.parameter_path, path);
        assert!(bad.snapshot.is_none());
    }
}

#[test]
fn rational_surface_rows_weights_and_unknown_logical_are_never_rewritten() {
    let (_, good) = run(SURFACE);
    assert_eq!(good.status, semantics::Status::Checked);
    let snapshot = good.snapshot.unwrap();
    assert_eq!(
        snapshot
            .field(0, None, "self_intersect")
            .unwrap()
            .value()
            .unwrap()
            .text,
        "U"
    );
    assert_eq!(
        snapshot
            .field(0, None, "weights_data")
            .unwrap()
            .value()
            .unwrap()
            .children[1]
            .children[1]
            .text,
        "4."
    );
    for (from, to, key, path) in [
        (
            "((#2,#2),(#2,#2))",
            "((#2,#2),(#2,#2,#2))",
            "SourceGeometryDependencyViolation",
            vec![3, 2],
        ),
        (
            "((1.,2.),(3.,4.))",
            "((1.,2.,5.),(3.,4.))",
            "SourceRuleViolation:RATIONAL_B_SPLINE_SURFACE.WR1",
            vec![1, 1],
        ),
        (
            "((1.,2.),(3.,4.))",
            "((1.,2.),(3.,4.,5.))",
            "SourceGeometryDependencyViolation",
            vec![1, 2],
        ),
        (
            "((1.,2.),(3.,4.))",
            "((1.,2.),(3.,-0.))",
            "SourceRuleViolation:RATIONAL_B_SPLINE_SURFACE.WR2",
            vec![1, 2, 2],
        ),
    ] {
        let (raw, bad) = run(&SURFACE.replace(from, to));
        let d = &bad.diagnostics[0];
        assert_eq!(d.source.key, key);
        assert_eq!(d.source.parameter_path, path);
        assert!(bad.snapshot.is_none());
        assert_eq!(raw.source.unwrap().find("1").unwrap().id, "1");
    }
    let (_, tiny) = run(&SURFACE.replace("(3.,4.)", "(3.,1.E-999999999999999999999999)"));
    assert_eq!(tiny.status, semantics::Status::Checked);
}

#[test]
fn trim_sets_and_definition_cycles_fail_before_ownership() {
    let base =
        "#1=TRIMMED_CURVE('',#1,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);";
    let (raw, cycle) = run(base);
    assert_eq!(cycle.geometry_dependencies, "Rejected");
    assert_eq!(cycle.part, "NotRun");
    assert_eq!(
        cycle.diagnostics[0].source.key,
        "SourceGeometryDependencyViolation"
    );
    assert_eq!(cycle.diagnostics[0].source.parameter_path, [2]);
    assert_eq!(
        cycle.diagnostics[0].responsibility,
        "project necessary geometry dependency closure"
    );
    assert_eq!(
        raw.source.unwrap().find("1").unwrap().components[0].parameters[1].text,
        "1"
    );
    let same="#1=TRIMMED_CURVE('',#2,(PARAMETER_VALUE(0.),PARAMETER_VALUE(1.)),(PARAMETER_VALUE(2.)),.T.,.PARAMETER.);#2=LINE('',#3,#4);#3=CARTESIAN_POINT('',(0.,0.,0.));#4=VECTOR('',#5,1.);#5=DIRECTION('',(1.,0.,0.));";
    let (_, bad) = run(same);
    assert_eq!(
        bad.diagnostics[0].source.key,
        "SourceRuleViolation:TRIMMED_CURVE.WR1"
    );
    assert_eq!(bad.diagnostics[0].source.parameter_path, [3]);
}

#[test]
fn composite_cycle_uses_real_parent_path_and_related_segment_occurrence() {
    let data="#1=COMPOSITE_CURVE('',(#2),.U.);#2=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#3);#3=TRIMMED_CURVE('',#1,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);";
    let (raw, bad) = run(data);
    let d = &bad.diagnostics[0];
    assert_eq!(d.source.key, "SourceGeometryDependencyViolation");
    assert_eq!(d.source.record_id, "2");
    assert_eq!(d.field, "parent_curve");
    assert_eq!(d.source.parameter_path, [3]);
    let snapshot = raw.source.unwrap();
    assert_eq!(
        d.source.related,
        [snapshot.find("1").unwrap().components[0].parameters[1].children[0].span]
    );
    assert!(bad.snapshot.is_none());
}

#[test]
fn composite_transition_and_inverse_use_are_source_declarations() {
    let parent="#4=TRIMMED_CURVE('',#5,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);#5=LINE('',#6,#7);#6=CARTESIAN_POINT('',(0.,0.,0.));#7=VECTOR('',#8,1.);#8=DIRECTION('',(1.,0.,0.));";
    let (_,bad)=run(&format!("#1=COMPOSITE_CURVE('',(#2,#3),.U.);#2=COMPOSITE_CURVE_SEGMENT(.DISCONTINUOUS.,.T.,#4);#3=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.F.,#4);{parent}"));
    assert_eq!(
        bad.diagnostics[0].source.key,
        "SourceRuleViolation:COMPOSITE_CURVE.WR1"
    );
    let (_, orphan) = run(&format!(
        "#2=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.F.,#4);{parent}"
    ));
    assert_eq!(
        orphan.diagnostics[0].source.key,
        "SourceRuleViolation:COMPOSITE_CURVE_SEGMENT.using_curves"
    );
    assert_eq!(orphan.diagnostics[0].source.parameter_path, [3]);
    let (_,bare)=run(&format!("#1=COMPOSITE_CURVE('',(#4),.U.);#4=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#5);#5=CIRCLE('',#2,1.);{POSITION}"));
    assert_eq!(
        bare.diagnostics[0].source.key,
        "SourceRuleViolation:COMPOSITE_CURVE_SEGMENT.WR1"
    );
}

#[test]
fn wireframe_reference_hops_do_not_fabricate_nested_parameter_paths() {
    let data="#1=LINE('',#2,#3);#2=CARTESIAN_POINT('',(0.,0.,0.));#3=VECTOR('',#4,1.);#4=DIRECTION('',(1.,0.,0.));#10=GEOMETRIC_CURVE_SET('',(#1));#11=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#10),#20);#20=REPRESENTATION_CONTEXT('','');";
    let (raw, bad) = run(data);
    let d = &bad.diagnostics[0];
    assert_eq!(
        d.source.key,
        "SourceRuleViolation:GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION.WR3"
    );
    assert_eq!(d.source.record_id, "10");
    assert_eq!(d.source.parameter_path, [2, 1]);
    let snapshot = raw.source.unwrap();
    assert_eq!(
        d.source.related,
        [snapshot.find("11").unwrap().components[0].parameters[1].children[0].span]
    );
}

#[test]
fn surface_role_requires_a_direct_surface_bearing_set() {
    let data=format!("#1=PLANE('',#2);{POSITION}#10=GEOMETRIC_SET('',(#1));#11=GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION('',(#10),#20);#20=REPRESENTATION_CONTEXT('','');");
    let (_, good) = run(&data);
    assert_eq!(good.status, semantics::Status::Checked);
    let (_, bad) = run(&data.replace("GEOMETRIC_SET('',(#1))", "GEOMETRIC_SET('',(#3))"));
    assert_eq!(
        bad.diagnostics[0].source.key,
        "SourceRuleViolation:GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION.WR7"
    );
    let (_, curve_set) = run(&data.replace("GEOMETRIC_SET('", "GEOMETRIC_CURVE_SET('"));
    assert_eq!(
        curve_set.diagnostics[0].source.key,
        "SourceRuleViolation:GEOMETRIC_CURVE_SET.WR1"
    );
}

#[test]
fn budgets_and_report_scope_preserve_unrun_geometry_consistency() {
    let (mut raw, good) = run(SURFACE);
    let visits = good.dependency_visits;
    let options = semantics::Options {
        check_geometry_dependencies: true,
        ..semantics::Options::default()
    };
    assert_eq!(
        semantics::check(
            &raw,
            semantics::Limits {
                dependency_visits: visits,
                preserved_diagnostics: 1024,
            },
            &options
        )
        .status,
        semantics::Status::Checked
    );
    let bad = semantics::check(
        &raw,
        semantics::Limits {
            dependency_visits: visits - 1,
            preserved_diagnostics: 1024,
        },
        &options,
    );
    assert_eq!(bad.status, semantics::Status::ResourceLimit);
    assert_eq!(bad.geometry_dependencies, "ResourceLimit");
    assert_eq!(bad.dependency_visits, visits);
    assert!(bad.snapshot.is_none());
    let report: serde_json::Value =
        serde_json::from_slice(&semantics::compile_report(&raw, &good).unwrap()).unwrap();
    assert_eq!(report["geometry_dependencies"], "Passed");
    assert_eq!(report["geometry_consistency"], "NotRun");
    assert_eq!(report["planar_rules"], "NotRun");
    assert_eq!(report["full_import"], "NotRun");
    assert_eq!(report["native_build"], "NotRun");
    raw.limits.report_bytes = 1;
    assert_eq!(
        semantics::compile_report(&raw, &good),
        Err(semantics::ReportError::Output(
            solidcore_step::report::Error::ReportByteLimit
        ))
    );
}

#[test]
fn dependency_depth_is_iterative_and_full_mode_matches_p0_scope() {
    let mut data=String::from("#1=CIRCLE('',#9001,1.);#9001=AXIS2_PLACEMENT_3D('',#9002,$,$);#9002=CARTESIAN_POINT('',(0.,0.,0.));");
    for id in 2..1500 {
        write!(data,"#{id}=TRIMMED_CURVE('',#{},(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);",id-1).unwrap();
    }
    let (_, good) = run(&data);
    assert_eq!(good.status, semantics::Status::Checked);
    assert!(good.dependency_visits < 100_000);
    let raw = inspect(
        include_bytes!("fixtures/plane.step").to_vec(),
        Limits::planar_fixtures(),
    );
    let result = semantics::check(
        &raw,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_geometry_dependencies: true,
            check_part_and_closure: true,
            ..semantics::Options::default()
        },
    );
    assert_eq!(result.status, semantics::Status::Checked);
    assert_eq!(result.geometry_dependencies, "NotRun");
    assert_eq!(result.part, "Passed");
}
