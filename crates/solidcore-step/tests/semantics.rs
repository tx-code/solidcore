// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent analytic source contracts; no C++ output supplies expected truth.
use solidcore_step::{inspect, semantics, Limits, ValueKind};

const AP203:&str="AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
fn file(data: &str, schema: &str) -> Vec<u8> {
    format!("ISO-10303-21;\nHEADER;FILE_DESCRIPTION(('synthetic'),'2;1');FILE_NAME('public.step','',(''),(''),'','','');FILE_SCHEMA(('{schema}'));ENDSEC;DATA;{data}ENDSEC;END-ISO-10303-21;").into_bytes()
}
fn run(data: &str) -> (solidcore_step::Outcome, semantics::Outcome) {
    let source = inspect(file(data, AP214), Limits::planar_fixtures());
    let result = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options::default(),
    );
    (source, result)
}
const LENGTH: &str = "#1=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));";

const RATIONAL_SURFACE:&str="#1=(BOUNDED_SURFACE() B_SPLINE_SURFACE(1,1,((#2,#2),(#2,#2)),.UNSPECIFIED.,.F.,.F.,.U.) B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.,1.),(1.,1.))) REPRESENTATION_ITEM('') SURFACE());#2=CARTESIAN_POINT('',(0.,0.,0.));";

#[test]
fn nested_rational_grid_and_unknown_logical_preserve_source_shape() {
    let (_, result) = run(RATIONAL_SURFACE);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    let grid = snapshot
        .field(0, Some("B_SPLINE_SURFACE"), "control_points_list")
        .unwrap();
    assert_eq!(grid.declaration().unwrap().nested_aggregates.len(), 1);
    assert_eq!(grid.declaration().unwrap().nested_aggregates[0].minimum, 2);
    let rows = &grid.value().unwrap().children;
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[0].children.len(), 2);
    assert_eq!(rows[0].children[0].text, "2");
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
        "1."
    );
    let (_, bad) = run(&RATIONAL_SURFACE.replace("((#2,#2),(#2,#2))", "((#2,#2),(#2,0))"));
    assert_eq!(bad.diagnostics[0].source.key, "AttributeTypeMismatch");
    assert_eq!(bad.diagnostics[0].source.parameter_path, [3, 2, 2]);
    assert!(bad.snapshot.is_none());
}

#[test]
fn converted_length_stays_deferred_and_retains_raw_factor() {
    let plane = String::from_utf8(include_bytes!("fixtures/plane.step").to_vec()).unwrap();
    let replacement="#10=(CONVERSION_BASED_UNIT('inch',#901) LENGTH_UNIT() NAMED_UNIT(#902));\n#900=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.));\n#901=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.0254),#900);\n#902=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);";
    let bytes = plane
        .replace(
            "#10=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));",
            replacement,
        )
        .into_bytes();
    for scope in [
        semantics::GeometryScope::ExplicitOuterBounds,
        semantics::GeometryScope::GeneralBrep,
    ] {
        let source = inspect(bytes.clone(), Limits::planar_fixtures());
        let result = semantics::check(
            &source,
            semantics::Limits::fixtures(),
            &semantics::Options {
                geometry_scope: scope,
                physical_context: Some("12".into()),
                ..semantics::Options::default()
            },
        );
        assert_eq!(result.status, semantics::Status::Deferred);
        assert_eq!(result.si_units, "Deferred");
        let snapshot = result.snapshot.unwrap();
        assert!(snapshot.units().length_power10_mm.is_none());
        assert_eq!(snapshot.source().bytes(), bytes);
        let factor = snapshot.source().find("901").unwrap();
        assert_eq!(
            factor.components[0].parameters[0].children[0].text,
            "0.0254"
        );
    }
}

#[test]
fn typed_marker_and_mixed_trim_select_keep_their_original_branches() {
    let (_,style)=run("#1=POINT_STYLE('marker',MARKER_TYPE(.RING.),POSITIVE_LENGTH_MEASURE(1.),#2);#2=COLOUR_RGB('colour',1.,0.,0.);");
    assert_eq!(style.status, semantics::Status::Checked);
    let snapshot = style.snapshot.unwrap();
    assert_eq!(
        snapshot
            .field(0, None, "marker")
            .unwrap()
            .value()
            .unwrap()
            .text,
        "MARKER_TYPE"
    );
    let data="#1=LINE('',#2,#4);#2=CARTESIAN_POINT('',(0.,0.,0.));#4=VECTOR('',#5,1.);#5=DIRECTION('',(1.,0.,0.));#3=TRIMMED_CURVE('',#1,(PARAMETER_VALUE(-0.),#2),(PARAMETER_VALUE(1.),#2),.T.,.PARAMETER.);";
    let (_, trim) = run(data);
    assert_eq!(trim.status, semantics::Status::Checked);
    let snapshot = trim.snapshot.unwrap();
    let last = snapshot.source().records().len() - 1;
    let field = snapshot.field(last, None, "trim_1").unwrap();
    assert_eq!(field.value().unwrap().children[0].children[0].text, "-0.");
    assert_eq!(
        field.value().unwrap().children[1].kind,
        ValueKind::Reference
    );
    let (_, duplicate) = run(&data.replace(
        "PARAMETER_VALUE(-0.),#2",
        "PARAMETER_VALUE(-0.),PARAMETER_VALUE(0.E99)",
    ));
    assert_eq!(duplicate.diagnostics[0].source.key, "SetDuplicateMember");
}

#[test]
fn retains_source_and_binds_inherited_fields_without_geometry_claims() {
    let bytes = include_bytes!("fixtures/plane.step").to_vec();
    let source = inspect(bytes.clone(), Limits::planar_fixtures());
    let options = semantics::Options {
        physical_context: Some("12".into()),
        ..semantics::Options::default()
    };
    let result = semantics::check(&source, semantics::Limits::fixtures(), &options);
    assert_eq!(result.status, semantics::Status::Checked);
    assert_eq!(result.unit_dependencies, "NotRun");
    assert_eq!(result.si_units, "Passed");
    let snapshot = result.snapshot.as_ref().unwrap();
    assert_eq!(snapshot.source().bytes(), bytes);
    assert!(std::ptr::eq(
        snapshot.source().records(),
        source.source.as_ref().unwrap().records()
    ));
    assert_eq!(snapshot.units().length_power10_mm, Some(0));
    assert_eq!(snapshot.units().si_definitions[0].si_name, "METRE");
    assert_eq!(snapshot.units().si_definitions[0].prefix, "MILLI");
    assert_eq!(
        snapshot.units().si_definitions[0].dimensions,
        [1, 0, 0, 0, 0, 0, 0]
    );
    assert!(snapshot.instances("BORDERLINE_UNKNOWN").is_none());
    assert_eq!(snapshot.instances("MASS_UNIT"), Some([].as_slice()));
    assert_eq!(snapshot.is_a(9, "SI_UNIT"), Some(true));
    assert_eq!(snapshot.is_a(9, "MASS_UNIT"), Some(false));
    let marker = snapshot.field(9, Some("NAMED_UNIT"), "dimensions").unwrap();
    assert!(marker.declaration().unwrap().derived_slot);
    assert_eq!(marker.value().unwrap().kind, ValueKind::Derived);
    let dimension = snapshot
        .field(
            11,
            Some("GEOMETRIC_REPRESENTATION_CONTEXT"),
            "coordinate_space_dimension",
        )
        .unwrap();
    assert_eq!(dimension.value().unwrap().text, "3");
    drop(result);
    drop(source);
    assert_eq!(dimension.value().unwrap().text, "3"); // owning field outlives every caller-owned result
}

#[test]
fn exact_unit_metadata_does_not_rewrite_coordinates_or_choose_quality() {
    let original = String::from_utf8(include_bytes!("fixtures/plane.step").to_vec()).unwrap();
    for (prefix, power) in [
        ("$", 3),
        (".EXA.", 21),
        (".KILO.", 6),
        (".MILLI.", 0),
        (".MICRO.", -3),
        (".ATTO.", -15),
    ] {
        let bytes = original
            .replace(
                "SI_UNIT(.MILLI.,.METRE.)",
                &format!("SI_UNIT({prefix},.METRE.)"),
            )
            .into_bytes();
        let source = inspect(bytes.clone(), Limits::planar_fixtures());
        let result = semantics::check(
            &source,
            semantics::Limits::fixtures(),
            &semantics::Options {
                physical_context: Some("12".into()),
                ..semantics::Options::default()
            },
        );
        assert_eq!(result.status, semantics::Status::Checked);
        let snapshot = result.snapshot.as_ref().unwrap();
        assert_eq!(snapshot.units().length_power10_mm, Some(power));
        assert_eq!(snapshot.source().bytes(), bytes);
        let report: serde_json::Value =
            serde_json::from_slice(&semantics::compile_report(&source, &result).unwrap()).unwrap();
        for stage in [
            "part",
            "geometry_dependencies",
            "native_build",
            "interoperability",
        ] {
            assert_eq!(report[stage], "NotRun");
        }
        assert_eq!(report["repair"], "None");
    }
}

#[test]
fn diagnostics_retain_type_path_target_related_span_and_declaration() {
    let (source, result) =
        run("#1=LINE('',#2,#3);#2=DIRECTION('',(1.,0.,0.));#3=VECTOR('',#2,1.);");
    assert_eq!(result.status, semantics::Status::Rejected);
    assert!(result.snapshot.is_none());
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "ReferenceTypeMismatch");
    assert_eq!(d.source.stage, "D1");
    assert_eq!(d.classification, "SourceInvalid");
    assert_eq!(d.source.record_id, "1");
    assert_eq!(d.source.component, "LINE");
    assert_eq!(d.source.parameter_path, [2]);
    assert_eq!(d.source.target_id, "2");
    assert_eq!(d.field, "pnt");
    assert_eq!(d.declared_in, "LINE");
    assert_eq!(d.declared_type, "CARTESIAN_POINT");
    let raw = source.source.as_ref().unwrap();
    assert_eq!(raw.raw(d.source.primary), Some(b"#2".as_slice()));
    assert_eq!(d.source.related, [raw.find("2").unwrap().id_span]);
    let (_, result) = run("#1=CARTESIAN_POINT($,(0.,0.,0.));");
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "RequiredAttributeMissing");
    assert_eq!(d.field, "name");
    assert_eq!(d.declared_in, "REPRESENTATION_ITEM");
    assert_eq!(d.actual, "$");
    let (_, result) = run("#1=CARTESIAN_POINT('',(0,0.,0.));");
    assert_eq!(result.diagnostics[0].source.key, "AttributeTypeMismatch");
    assert_eq!(result.diagnostics[0].source.parameter_path, [2, 1]);
}

#[test]
fn complex_transport_guards_do_not_repair_or_reorder_source() {
    for (data,key) in [
        ("#1=(NAMED_UNIT(*) LENGTH_UNIT() SI_UNIT(.MILLI.,.METRE.));","ComplexConstituentOrderInvalid"),
        ("#1=(LENGTH_UNIT() SI_UNIT(.MILLI.,.METRE.));","MissingComplexConstituent"),
        ("#1=(LENGTH_UNIT() NAMED_UNIT(*) NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));","DuplicateComplexConstituent"),
        ("#1=(LENGTH_UNIT() MASS_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));","ComplexSubtypeConflict"),
        ("#1=(LENGTH_UNIT() NAMED_UNIT(#2) SI_UNIT(.MILLI.,.METRE.));#2=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);","DerivedSlotMarkerRequired"),
    ] {let (source,result)=run(data);assert_eq!(result.diagnostics[0].source.key,key);assert!(result.snapshot.is_none());assert_eq!(source.captured(),Some(file(data,AP214).as_slice()));}
    let (_,result)=run("#1=PRODUCT('','',$,(#2,#02));#2=PRODUCT_CONTEXT('',#3,'mechanical');#3=APPLICATION_CONTEXT('');");
    assert_eq!(result.diagnostics[0].source.key, "SetDuplicateMember");
    assert_eq!(result.diagnostics[0].source.parameter_path, [4, 2]);
    assert_eq!(result.diagnostics[0].source.related.len(), 1);
}

#[test]
fn schema_identity_and_implementation_are_separate_and_ordered() {
    for (schema, scope, status, profile) in [
        (
            AP203,
            semantics::GeometryScope::ExplicitOuterBounds,
            semantics::Status::Checked,
            Some("203e2-P0-Planar-0.1"),
        ),
        (
            AP203,
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Checked,
            Some("203e2-G0-Source-0.1"),
        ),
        (
            AP214,
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Checked,
            Some("214e3-P0-Source-0.1"),
        ),
        (
            "CONFIG_CONTROL_DESIGN",
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Checked,
            Some("203-N916-Source-0.1"),
        ),
        (
            "AUTOMOTIVE_DESIGN",
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Unsupported,
            None,
        ),
        (
            "AUTOMOTIVE_DESIGN {01 0 10303 214 3 1 1}",
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Unsupported,
            None,
        ),
        (
            "AUTOMOTIVE_DESIGN {1 0 10303 214 2 1 1}",
            semantics::GeometryScope::GeneralBrep,
            semantics::Status::Unsupported,
            None,
        ),
    ] {
        let source = inspect(
            file("#1=APPLICATION_CONTEXT('');", schema),
            Limits::planar_fixtures(),
        );
        let result = semantics::check(
            &source,
            semantics::Limits::fixtures(),
            &semantics::Options {
                geometry_scope: scope,
                physical_context: None,
                ..semantics::Options::default()
            },
        );
        assert_eq!(result.status, status, "{schema}");
        let selection = result.schema_selection.unwrap();
        assert_eq!(selection.identifiers[0].literal, schema);
        assert_eq!(selection.qualified.map(|p| p.profile), profile);
    }
}

#[test]
fn exact_derived_units_keep_fractional_exponents_and_nominal_roles() {
    let data = format!("{LENGTH}#2=DERIVED_UNIT_ELEMENT(#1,0.5);#3=DERIVED_UNIT((#2));");
    let (source, result) = run(&data);
    assert_eq!(result.status, semantics::Status::Checked);
    assert_eq!(result.unit_dependencies, "Passed");
    assert_eq!(result.si_units, "NotRun");
    let snapshot = result.snapshot.as_ref().unwrap();
    let definition = &snapshot.units().dimension_definitions[1];
    assert_eq!(definition.nominal, "Derived");
    assert_eq!(definition.dimensions[0].numerator, "1");
    assert_eq!(definition.dimensions[0].denominator, "2");
    assert_eq!(snapshot.is_a(2, "LENGTH_UNIT"), Some(false));
    assert_eq!(snapshot.source().bytes(), source.captured().unwrap());
    let (_, result) = run(&format!(
        "{LENGTH}#2=DERIVED_UNIT_ELEMENT(#1,9223372036854775808.E-1);#3=DERIVED_UNIT((#2));"
    ));
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    let dim = &snapshot.units().dimension_definitions[1].dimensions[0];
    assert_eq!(
        (&*dim.numerator, &*dim.denominator),
        ("4611686018427387904", "5")
    );
}

#[test]
fn rational_capacity_is_resource_failure_and_zero_does_not_remove_dependencies() {
    let (_, result) = run(&format!(
        "{LENGTH}#2=DERIVED_UNIT_ELEMENT(#1,1.E-20);#3=DERIVED_UNIT((#2));"
    ));
    assert_eq!(result.status, semantics::Status::ResourceLimit);
    assert_eq!(
        result.diagnostics[0].source.key,
        "UnitDimensionCapacityExceeded"
    );
    assert_eq!(result.diagnostics[0].classification, "ResourceLimit");
    assert!(result.snapshot.is_none());
    let (_,result)=run("#1=(MASS_UNIT() NAMED_UNIT(*) SI_UNIT($,.GRAM.));#2=DERIVED_UNIT_ELEMENT(#1,0.);#3=DERIVED_UNIT((#2));");
    assert_eq!(
        result.diagnostics[0].source.key,
        "SourceRuleViolation:SI_UNIT.WR1"
    );
    assert_eq!(result.diagnostics[0].source.record_id, "1");
    let (_,result)=run("#1=(CONVERSION_BASED_UNIT('cycle',#2) LENGTH_UNIT() NAMED_UNIT(*));#2=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.),#1);");
    assert_eq!(
        result.diagnostics[0].source.key,
        "SourceUnitDependencyCycle"
    );
    assert!(result.snapshot.is_none());
}

#[test]
fn signed_zero_huge_exponents_and_dimension_conflicts_are_not_coerced() {
    for value in ["0.", "-0.", "-1.E-99999999999999999999999999"] {
        let (_, result) = run(&format!(
            "{LENGTH}#2=MEASURE_WITH_UNIT(POSITIVE_LENGTH_MEASURE({value}),#1);"
        ));
        assert_eq!(
            result.diagnostics[0].source.key,
            "SourceRuleViolation:POSITIVE_LENGTH_MEASURE.WR1"
        );
    }
    let (_, result) = run(&format!(
        "{LENGTH}#2=MEASURE_WITH_UNIT(POSITIVE_LENGTH_MEASURE(1.E-99999999999999999999999999),#1);"
    ));
    assert_eq!(result.status, semantics::Status::Checked);
    let (_, result) = run(&format!(
        "{LENGTH}#2=MEASURE_WITH_UNIT(RATIO_MEASURE(1.),#1);"
    ));
    assert_eq!(
        result.diagnostics[0].source.key,
        "SourceRuleViolation:MEASURE_WITH_UNIT.WR1"
    );
    let (_, result) = run("#1=VALUE_REPRESENTATION_ITEM('unaltered',COUNT_MEASURE(-7));");
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert!(snapshot.units().length_unit.is_none());
    assert_eq!(
        snapshot
            .field(0, None, "value_component")
            .unwrap()
            .value()
            .unwrap()
            .children[0]
            .text,
        "-7"
    );
}

#[test]
fn report_binding_budgets_and_source_failure_remain_atomic() {
    let (mut source, result) = run(LENGTH);
    let (_, other) = run(LENGTH);
    assert_eq!(
        semantics::compile_report(&source, &other),
        Err(semantics::ReportError::SourceMismatch)
    );
    source.limits.report_bytes = 1;
    assert_eq!(
        semantics::compile_report(&source, &result),
        Err(semantics::ReportError::Output(
            solidcore_step::report::Error::ReportByteLimit
        ))
    );
    let source = inspect(
        file(
            "#1=APPLICATION_CONTEXT('');#1=APPLICATION_CONTEXT('');",
            AP203,
        ),
        Limits::planar_fixtures(),
    );
    let result = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options::default(),
    );
    assert_eq!(result.status, semantics::Status::SourceFailure);
    assert_eq!(result.binding, "NotRun");
    assert_eq!(
        result.diagnostics[0].classification,
        "SourceInspectionFailure"
    );
    assert_eq!(result.diagnostics[0].source.key, "DuplicateInstanceId");
    assert!(result.snapshot.is_none());
}

#[test]
fn exact_visit_budget_and_explicit_context_selection_never_default() {
    let source = inspect(
        include_bytes!("fixtures/plane.step").to_vec(),
        Limits::planar_fixtures(),
    );
    let options = semantics::Options {
        physical_context: Some("12".into()),
        ..semantics::Options::default()
    };
    let good = semantics::check(&source, semantics::Limits::fixtures(), &options);
    let visits = good.dependency_visits;
    assert!(visits > 0);
    assert_eq!(
        semantics::check(
            &source,
            semantics::Limits {
                dependency_visits: visits,
                preserved_diagnostics: 1024,
            },
            &options
        )
        .status,
        semantics::Status::Checked
    );
    let failure = semantics::check(
        &source,
        semantics::Limits {
            dependency_visits: visits - 1,
            preserved_diagnostics: 1024,
        },
        &options,
    );
    assert_eq!(failure.status, semantics::Status::ResourceLimit);
    assert_eq!(failure.dependency_visits, visits);
    assert_eq!(failure.diagnostics[0].source.key, "DependencyVisitLimit");
    assert!(failure.snapshot.is_none());
    let failure = semantics::check(
        &source,
        semantics::Limits {
            dependency_visits: 10_000_001,
            preserved_diagnostics: 1024,
        },
        &options,
    );
    assert_eq!(failure.dependency_visits, 0);
    assert_eq!(failure.diagnostics[0].source.key, "InvalidSemanticLimits");
    let failure = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options {
            physical_context: Some("1".into()),
            ..semantics::Options::default()
        },
    );
    assert_eq!(failure.status, semantics::Status::InvalidRequest);
    assert!(failure.snapshot.is_none());
    let passed = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options::default(),
    );
    assert_eq!(passed.si_units, "NotRun");
    assert!(passed.snapshot.unwrap().units().length_power10_mm.is_none());
}
