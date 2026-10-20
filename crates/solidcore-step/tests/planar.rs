// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent analytic truth for bounded arithmetic, source rules and regions.
use p::{ExactError, Logical, Rational as R, Status};
use solidcore_step::{inspect, planar as p, semantics as s, Limits};

fn data() -> String {
    include_str!("fixtures/plane.step").into()
}
fn check(
    text: &str,
    quality: p::Quality,
    limits: p::Limits,
) -> (solidcore_step::Outcome, s::Outcome, p::Outcome) {
    let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
    let sem = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    let result = p::check(&sem, quality, limits);
    (raw, sem, result)
}
fn fixture(text: &str) -> p::Outcome {
    check(text, p::Quality::planar_fixture(), p::Limits::fixtures()).2
}
fn append(text: &str, rows: &str) -> String {
    text.replace("ENDSEC;\nEND-ISO", &format!("{rows}\nENDSEC;\nEND-ISO"))
}
fn hole() -> String {
    append(&data().replace("ADVANCED_FACE('',(#301),#134", "ADVANCED_FACE('',(#301,#499),#134"),
        "#400=CARTESIAN_POINT('',(2.6,0.8,5.));\n#401=CARTESIAN_POINT('',(1.,2.,5.));\n#402=CARTESIAN_POINT('',(2.2,3.6,5.));\n#403=CARTESIAN_POINT('',(3.8,2.4,5.));
#410=VERTEX_POINT('',#400);\n#411=VERTEX_POINT('',#401);\n#412=VERTEX_POINT('',#402);\n#413=VERTEX_POINT('',#403);
#420=DIRECTION('',(-0.8,0.6,0.));\n#421=DIRECTION('',(0.6,0.8,0.));\n#422=DIRECTION('',(0.8,-0.6,0.));\n#423=DIRECTION('',(-0.6,-0.8,0.));
#430=VECTOR('',#420,2.);\n#431=VECTOR('',#421,2.);\n#432=VECTOR('',#422,2.);\n#433=VECTOR('',#423,2.);
#440=LINE('',#400,#430);\n#441=LINE('',#401,#431);\n#442=LINE('',#402,#432);\n#443=LINE('',#403,#433);
#450=EDGE_CURVE('',#410,#411,#440,.T.);\n#451=EDGE_CURVE('',#411,#412,#441,.T.);\n#452=EDGE_CURVE('',#412,#413,#442,.T.);\n#453=EDGE_CURVE('',#413,#410,#443,.T.);
#460=ORIENTED_EDGE('',*,*,#450,.T.);\n#461=ORIENTED_EDGE('',*,*,#451,.T.);\n#462=ORIENTED_EDGE('',*,*,#452,.T.);\n#463=ORIENTED_EDGE('',*,*,#453,.T.);
#498=EDGE_LOOP('',(#460,#461,#462,#463));\n#499=FACE_BOUND('',#498,.T.);")
}

#[test]
fn fractions_are_canonical_read_only_and_overflow_never_rounds() {
    assert_eq!(R::decimal("-12.50E-1").unwrap(), R::new(-5, 4).unwrap());
    assert_eq!(R::new(2, 4).unwrap().numerator(), 1);
    assert_eq!(R::new(0, 12).unwrap().denominator(), 1);
    assert_eq!(
        R::new(1, 3)
            .unwrap()
            .checked_add(R::new(1, 6).unwrap())
            .unwrap(),
        R::new(1, 2).unwrap()
    );
    assert_eq!(
        R::new(2, 3)
            .unwrap()
            .divide(R::new(-4, 5).unwrap())
            .unwrap(),
        R::new(-5, 6).unwrap()
    );
    for (n, d) in [(1, 0), (1, -1), (i64::MIN, 1)] {
        assert_eq!(R::new(n, d), Err(ExactError::Capacity));
    }
    assert_eq!(
        R::new(i64::MAX, 1).unwrap().checked_add(R::ONE),
        Err(ExactError::Capacity)
    );
    assert_eq!(R::ONE.divide(R::ZERO), Err(ExactError::UndefinedDivision));
    assert_eq!(R::decimal("1E-19"), Err(ExactError::Capacity));
    // The planar parser checks exponent capacity before accepting a zero mantissa.
    assert_eq!(R::decimal("0.E10000000"), Err(ExactError::Capacity));
    assert_eq!(R::decimal("-0.E100000"), Ok(R::ZERO));
    assert_eq!(R::decimal("1..2"), Err(ExactError::InvalidDecimal));
}

#[test]
fn exact_comparison_agrees_with_independent_wide_cross_products() {
    let m = i64::MAX;
    for (a, b, c, d) in [
        (m - 1, m, m, m - 1),
        (-m, m - 1, -m + 1, m),
        (-1, 7, -1, 8),
        (0, 1, -1, m),
        (1, 3, 2, 6),
        (m, 1, m, 2),
    ] {
        let x = R::new(a, b).unwrap();
        let y = R::new(c, d).unwrap();
        assert_eq!(
            x.cmp(&y),
            (i128::from(a) * i128::from(d)).cmp(&(i128::from(c) * i128::from(b)))
        );
    }
}

#[test]
fn logical_unknown_is_computed_and_query_does_not_select_it() {
    assert_eq!(Logical::Unknown.not(), Logical::Unknown);
    assert!(!Logical::Unknown.query_selects());
    assert!(!Logical::False.query_selects());
    assert!(Logical::True.query_selects());
    let result = fixture(&data());
    let rule = result
        .evidence
        .rules
        .iter()
        .find(|r| r.name == "restrict_representation_for_surface_condition")
        .unwrap();
    assert_eq!(rule.value, Some(Logical::True));
    assert_eq!(rule.facts["correlation_UNKNOWN_count"], "1");
    assert_eq!(rule.facts["failure_QUERY_count"], "0");
}

#[test]
fn plane_metrics_and_dimensionless_line_parameters_have_analytic_truth() {
    let result = fixture(&data());
    assert_eq!(result.status, Status::Checked);
    assert_eq!(result.work, 364);
    assert_eq!(result.evidence.rules.len(), 207);
    let face = &result.evidence.faces[0];
    assert_eq!(face.area_squared_mm, R::new(9216, 1).unwrap());
    assert_eq!(
        face.centroid_mm,
        [
            R::new(17, 5).unwrap(),
            R::new(26, 5).unwrap(),
            R::new(5, 1).unwrap()
        ]
    );
    assert_eq!(face.source_length_scale_mm, R::ONE);
    let edge = &result.evidence.edges[0];
    assert_eq!(edge.t_start_coefficient, R::ZERO);
    assert_eq!(edge.t_end_coefficient, R::ONE);
    assert_eq!(edge.length_squared_mm, R::new(144, 1).unwrap());
    assert!(face.source_uv_formula.contains("source_length_scale_mm"));
}

#[test]
fn global_populations_include_subtypes_and_do_not_invent_unknown_public_types() {
    let result = fixture(&data());
    let mut classes = std::collections::BTreeMap::new();
    for rule in &result.evidence.rules {
        if rule.kind == "GLOBAL" {
            *classes.entry(rule.applicability).or_insert(0) += 1;
        }
    }
    assert_eq!(
        classes,
        std::collections::BTreeMap::from([("Mandatory", 5), ("Conditional", 1), ("Guarded", 41)])
    );
    let rule = result
        .evidence
        .rules
        .iter()
        .find(|r| r.name == "validate_dependently_instantiable_entity_data_types")
        .unwrap();
    assert_eq!(rule.populations["LOOP"].len(), 1);
    let source = result.evidence.source.as_ref().unwrap();
    assert!(source.instances("NOT_REGISTERED").is_none());
    assert_eq!(
        source.source().records()[rule.populations["LOOP"][0]].id,
        "300"
    );
}

#[test]
fn irrational_axis_normalization_retains_exact_radicals_and_original_geometry() {
    let changed = data().replace(
        "#132=DIRECTION('',(0.6,0.8,0.));",
        "#132=DIRECTION('',(1.,1.,0.));",
    );
    let result = fixture(&changed);
    assert_eq!(result.status, Status::Checked);
    let face = &result.evidence.faces[0];
    assert_eq!(face.x_norm_squared, R::new(2, 1).unwrap());
    assert_eq!(face.metric_squared, R::new(4, 1).unwrap());
    assert_eq!(face.area_squared_mm, R::new(9216, 1).unwrap());
    assert_eq!(
        face.loops[0].signed_area_coefficient,
        R::new(48, 1).unwrap()
    );
    assert_eq!(
        result.evidence.source.as_ref().unwrap().source().bytes(),
        changed.as_bytes()
    );
}

#[test]
fn holes_subtract_area_and_first_moments_without_changing_source_topology() {
    let text = hole();
    let result = fixture(&text);
    assert_eq!(result.status, Status::Checked);
    let face = &result.evidence.faces[0];
    assert_eq!(face.loops.len(), 2);
    assert_eq!(
        face.loops[1].signed_area_coefficient,
        R::new(-4, 1).unwrap()
    );
    assert_eq!(face.area_squared_mm, R::new(8464, 1).unwrap());
    assert_eq!(
        face.centroid_mm,
        [
            R::new(396, 115).unwrap(),
            R::new(613, 115).unwrap(),
            R::new(5, 1).unwrap()
        ]
    );
    assert_eq!(
        result.evidence.source.as_ref().unwrap().source().bytes(),
        text.as_bytes()
    );
    let bad = fixture(&text.replace(
        "#499=FACE_BOUND('',#498,.T.);",
        "#499=FACE_BOUND('',#498,.F.);",
    ));
    assert_eq!(bad.status, Status::Rejected);
    assert_eq!(
        bad.diagnostics[0].source.key,
        "InnerBoundaryOrientationMismatch"
    );
}

#[test]
fn closed_surface_geometry_retains_surface_role_distinct_from_material_box() {
    for (text, role) in [
        (include_str!("fixtures/material-box.step"), "MaterialSolid"),
        (
            include_str!("fixtures/surface-box.step"),
            "ClosedSurfaceModel",
        ),
    ] {
        let result = fixture(text);
        assert_eq!(result.status, Status::Checked);
        assert_eq!(result.evidence.faces.len(), 6);
        assert_eq!(
            result
                .evidence
                .source
                .as_ref()
                .unwrap()
                .part_definition()
                .unwrap()
                .shape_role,
            role
        );
        let incidence = result
            .evidence
            .metrics
            .iter()
            .find(|m| m.kind == "shell_incidence")
            .unwrap();
        assert_eq!(incidence.exact["boundary_edges"], R::ZERO);
        assert_eq!(incidence.exact["paired_edges"], R::new(12, 1).unwrap());
    }
}

#[test]
fn failure_stages_and_partial_rows_follow_actual_entry_order() {
    let local = fixture(&data().replace(
        "#200=DIRECTION('',(0.6,0.8,0.));",
        "#200=DIRECTION('',(0.,0.,0.));",
    ));
    assert_eq!(
        (local.local_rules, local.global_rules, local.geometry),
        ("Failed", "NotRun", "NotRun")
    );
    assert_eq!(
        local.evidence.rules.last().unwrap().value,
        Some(Logical::False)
    );
    let global = fixture(&data().replace(
        "'ap203_configuration_controlled_3d_design_of_mechanical_parts_and_assemblies'",
        "'different source claim'",
    ));
    assert_eq!(
        (global.local_rules, global.global_rules, global.geometry),
        ("Passed", "Failed", "NotRun")
    );
    assert_eq!(
        global.diagnostics[0].source.key,
        "SourceRuleViolation:application_protocol_definition_required.WR1"
    );
    assert_eq!(global.diagnostics[0].source.parameter_path, [2]);
    let geometry = fixture(&data().replace(
        "#203=EDGE_CURVE('',#110,#111,#202,.T.);",
        "#203=EDGE_CURVE('',#110,#111,#202,.F.);",
    ));
    assert_eq!(
        (
            geometry.local_rules,
            geometry.global_rules,
            geometry.geometry
        ),
        ("Passed", "Passed", "Failed")
    );
    assert_eq!(geometry.diagnostics[0].source.key, "EdgeCurveSenseMismatch");
    assert_eq!(geometry.evidence.edges.len(), 1);
}

#[test]
fn exact_quality_boundary_is_inclusive_and_a_tighter_named_configuration_rejects() {
    let text = data().replace(
        "#130=CARTESIAN_POINT('',(3.,-2.,5.));",
        "#130=CARTESIAN_POINT('',(3.,-2.,5.0000001));",
    );
    assert_eq!(fixture(&text).status, Status::Checked);
    let strict = p::Quality {
        id: "explicit-stricter".into(),
        consistency_mm: R::new(1, 20_000_000).unwrap(),
    };
    let result = check(&text, strict, p::Limits::fixtures()).2;
    assert_eq!(result.status, Status::Rejected);
    assert_eq!(result.diagnostics[0].source.key, "EdgeSurfaceMismatch");
    assert_eq!(
        result.evidence.source.as_ref().unwrap().source().bytes(),
        text.as_bytes()
    );
}

#[test]
fn optional_where_success_does_not_invent_undefined_placement_axes() {
    let changed = data().replace(
        "AXIS2_PLACEMENT_3D('',#130,#131,#132)",
        "AXIS2_PLACEMENT_3D('',#130,$,#132)",
    );
    let changed = changed
        .replace(
            "#132=DIRECTION('',(0.6,0.8,0.));",
            "#132=DIRECTION('',(0.,0.,1.));",
        )
        .lines()
        .filter(|line| !line.starts_with("#131="))
        .collect::<Vec<_>>()
        .join("\n");
    let result = fixture(&changed);
    assert_eq!(result.status, Status::Indeterminate);
    assert_eq!(
        (result.local_rules, result.global_rules, result.geometry),
        ("Passed", "Passed", "Indeterminate")
    );
    assert_eq!(
        result.diagnostics[0].source.key,
        "DerivedPlacementAxesIndeterminate"
    );
    assert!(result.evidence.edges.is_empty());
}

#[test]
fn numeric_capacity_retains_unestablished_rows_instead_of_source_false() {
    let changed = data().replace(
        "#201=VECTOR('',#200,12.);",
        "#201=VECTOR('',#200,0.E10000000);",
    );
    let result = fixture(&changed);
    assert_eq!(result.status, Status::Indeterminate);
    assert_eq!(result.local_rules, "Indeterminate");
    assert_eq!(result.global_rules, "NotRun");
    let last = result.evidence.rules.last().unwrap();
    assert_eq!(last.name, "VECTOR.WR1");
    assert_eq!(last.value, None);
    assert_eq!(last.evaluation, "Indeterminate");
    assert_eq!(result.diagnostics[0].source.stage, "D3");
    assert_eq!(
        result.diagnostics[0].source.key,
        "ExactArithmeticCapacityExceeded"
    );
}

#[test]
fn exact_work_budget_does_not_charge_a_denied_visit_or_qualify_partial_evidence() {
    let good = fixture(&data());
    let n = good.work;
    assert_eq!(
        check(&data(), p::Quality::planar_fixture(), p::Limits { work: n })
            .2
            .status,
        Status::Checked
    );
    let limited = check(
        &data(),
        p::Quality::planar_fixture(),
        p::Limits { work: n - 1 },
    )
    .2;
    assert_eq!(limited.status, Status::ResourceLimit);
    assert_eq!(limited.work, n - 1);
    assert!(!limited.evidence.faces.is_empty());
    assert_eq!(limited.geometry, "ResourceLimit");
    let zero = check(&data(), p::Quality::planar_fixture(), p::Limits { work: 0 }).2;
    assert_eq!(
        (zero.local_rules, zero.global_rules, zero.geometry),
        ("NotRun", "NotRun", "NotRun")
    );
    assert!(zero.evidence.rules.is_empty());
    let records = good
        .evidence
        .source
        .as_ref()
        .unwrap()
        .source()
        .records()
        .len();
    let stop = check(
        &data(),
        p::Quality::planar_fixture(),
        p::Limits { work: records },
    )
    .2;
    assert_eq!(stop.local_rules, "ResourceLimit");
    assert!(stop.evidence.rules.is_empty());
}

#[test]
fn quality_identity_and_complete_semantic_handoff_are_required() {
    let mut quality = p::Quality::planar_fixture();
    quality.consistency_mm = R::new(1, 20_000_000).unwrap();
    assert_eq!(
        check(&data(), quality, p::Limits::fixtures()).2.status,
        Status::InvalidRequest
    );
    assert_eq!(
        check(
            &data(),
            p::Quality::planar_fixture(),
            p::Limits { work: 10_000_001 }
        )
        .2
        .status,
        Status::InvalidRequest
    );
    let raw = inspect(data().into_bytes(), Limits::planar_fixtures());
    let partial = s::check(&raw, s::Limits::fixtures(), &s::Options::default());
    assert_eq!(partial.status, s::Status::Checked);
    assert_eq!(
        p::check(
            &partial,
            p::Quality::planar_fixture(),
            p::Limits::fixtures()
        )
        .status,
        Status::InvalidRequest
    );
}

#[test]
fn qualified_other_schema_never_acquires_planar_rule_evidence() {
    let text=data().replace("AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}","AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}");
    let (_, sem, result) = check(&text, p::Quality::planar_fixture(), p::Limits::fixtures());
    assert_eq!(sem.status, s::Status::Checked);
    assert_eq!(result.status, Status::Unsupported);
    assert_eq!(result.diagnostics[0].source.key, "UnsupportedRuleProfile");
    assert_eq!(result.work, 0);
    assert!(result.evidence.rules.is_empty());
}

#[test]
fn evidence_owns_source_after_outcome_drop_and_report_is_bound_and_atomic() {
    let (raw, sem, result) = check(&data(), p::Quality::planar_fixture(), p::Limits::fixtures());
    let bytes = p::compile_report(&raw, &sem, &result).unwrap();
    let json: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    assert!(json["model"].is_null());
    assert_eq!(json["checks"]["native_build"], "NotRun");
    assert_eq!(json["qualified_source_evidence"], true);
    let other = inspect(data().into_bytes(), Limits::planar_fixtures());
    assert_eq!(
        p::compile_report(&other, &sem, &result),
        Err(p::ReportError::SourceMismatch)
    );
    let evidence = result.evidence;
    drop(raw);
    drop(sem);
    assert!(evidence
        .source
        .as_ref()
        .unwrap()
        .source()
        .find("300")
        .is_some());
    assert_eq!(evidence.faces[0].area_squared_mm, R::new(9216, 1).unwrap());
    let mut limits = Limits::planar_fixtures();
    limits.report_bytes = 1;
    let raw = inspect(data().into_bytes(), limits);
    let sem = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            check_part_and_closure: true,
            ..Default::default()
        },
    );
    let result = p::check(&sem, p::Quality::planar_fixture(), p::Limits::fixtures());
    assert_eq!(result.status, Status::Checked);
    assert_eq!(
        p::compile_report(&raw, &sem, &result),
        Err(p::ReportError::Output(
            solidcore_step::report::Error::ReportByteLimit
        ))
    );
}
