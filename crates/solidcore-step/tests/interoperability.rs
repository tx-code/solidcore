// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent synthetic source ownership, exclusion and qualification truth.
use solidcore_step::{
    inspect, inspect_with_options, semantics as s, CarrierInterpretation, DataStringPolicy,
    InspectOptions, Limits, ValueKind,
};

const AP203: &str = "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
const MBX: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 1 1 1 1}";
const EIGHT: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1 1}";
fn plane(schema: &str) -> String {
    let source = include_str!("fixtures/plane.step").replace(AP203, schema);
    if schema != "CONFIG_CONTROL_DESIGN" {
        return source;
    }
    // Author a legal N916 fixture: these TEXT slots are required in that fixed
    // schema. This is literal test input construction, never loader normalization.
    source
        .lines()
        .map(|line| {
            if ["#4=", "#5=", "#7=", "#8="]
                .iter()
                .any(|p| line.starts_with(p))
            {
                line.replace('$', "''")
            } else {
                line.into()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}
fn append(source: &str, data: &str) -> String {
    source.replace("ENDSEC;\nEND-ISO", &format!("{data}\nENDSEC;\nEND-ISO"))
}
fn options() -> s::Options {
    s::Options {
        source_policy: s::SourcePolicy::PreserveNonGeometric,
        geometry_scope: s::GeometryScope::GeneralBrep,
        ..Default::default()
    }
}
fn check(
    data: &str,
    options: &s::Options,
    limits: s::Limits,
) -> (solidcore_step::Outcome, s::Outcome) {
    let source = inspect(data.as_bytes().to_vec(), Limits::planar_fixtures());
    let result = s::check(&source, limits, options);
    (source, result)
}
const DENSITY: &str = "#900=(MASS_UNIT() NAMED_UNIT(*) SI_UNIT(.KILO.,.GRAM.));
#901=DERIVED_UNIT_ELEMENT(#900,1.);
#902=DERIVED_UNIT_ELEMENT(#10,-3.);
#903=DERIVED_UNIT((#901,#902));
#950=PROPERTY_DEFINITION('source density',$,#7);
#951=PROPERTY_DEFINITION_REPRESENTATION(#950,#952);
#952=REPRESENTATION('metadata',(#953),#12);
#953=MEASURE_REPRESENTATION_ITEM('density',POSITIVE_RATIO_MEASURE(1.),#903);";
const MASS: &str = "#900=(MASS_UNIT() NAMED_UNIT(*) SI_UNIT(.KILO.,.GRAM.));
#904=MASS_MEASURE_WITH_UNIT(MASS_MEASURE(2.),#900);
#905=(CONVERSION_BASED_UNIT('source mass',#904) MASS_UNIT() NAMED_UNIT(#960));
#960=DIMENSIONAL_EXPONENTS(0.,1.,0.,0.,0.,0.,0.);
#950=PROPERTY_DEFINITION('mass property',$,#7);
#951=PROPERTY_DEFINITION_REPRESENTATION(#950,#952);
#952=REPRESENTATION('metadata',(#953),#12);
#953=MEASURE_REPRESENTATION_ITEM('mass',MASS_MEASURE(1.),#905);";

#[test]
fn attached_density_keeps_exact_bad_value_and_original_strict_diagnostic() {
    let data = append(&plane(AP214), DENSITY);
    let (_, strict) = check(
        &data,
        &s::Options {
            geometry_scope: s::GeometryScope::GeneralBrep,
            check_geometry_dependencies: true,
            check_part_and_closure: true,
            ..Default::default()
        },
        s::Limits::fixtures(),
    );
    assert_eq!(strict.status, s::Status::Rejected);
    let (source, outcome) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(outcome.status, s::Status::CheckedWithDiagnostics);
    assert_eq!(outcome.interoperability, "Passed");
    assert!(outcome.diagnostics.is_empty());
    assert_eq!(outcome.unit_dependencies, "CheckedWithDiagnostics");
    assert_eq!(
        serde_json::to_value(outcome.preserved_diagnostics()).unwrap(),
        serde_json::to_value(&strict.diagnostics).unwrap()
    );
    let snapshot = outcome.snapshot.as_ref().unwrap();
    assert_eq!(snapshot.geometry_scope(), s::GeometryScope::GeneralBrep);
    assert_eq!(snapshot.source().bytes(), data.as_bytes());
    assert_eq!(
        snapshot.source().find("953").unwrap().components[0].parameters[1].text,
        "POSITIVE_RATIO_MEASURE"
    );
    assert_eq!(
        snapshot.source().find("953").unwrap().components[0].parameters[1].children[0].text,
        "1."
    );
    assert_eq!(
        snapshot
            .part_definition()
            .unwrap()
            .property_attachments
            .len(),
        1
    );
    let json: serde_json::Value =
        serde_json::from_slice(&s::compile_report(&source, &outcome).unwrap()).unwrap();
    assert_eq!(json["native_build"], "NotRun");
    assert_eq!(json["full_import"], "NotRun");
    assert_eq!(json["limits"]["preserved_diagnostics"], 1024);
}

#[test]
fn mass_marker_exception_never_hides_conflicting_dimensions_or_physical_use() {
    let data = append(&plane(AP214), MASS);
    let (_, good) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(good.status, s::Status::CheckedWithDiagnostics);
    assert_eq!(
        good.preserved_diagnostics()[0].source.key,
        "DerivedSlotMarkerRequired"
    );
    let source = good.snapshot.as_ref().unwrap().source();
    assert_eq!(
        source.find("905").unwrap().components[2].parameters[0].kind,
        ValueKind::Reference
    );
    let conflicting = data.replace(
        "DIMENSIONAL_EXPONENTS(0.,1.,0.,0.,0.,0.,0.)",
        "DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)",
    );
    let (_, bad) = check(&conflicting, &options(), s::Limits::fixtures());
    assert_eq!(bad.status, s::Status::Rejected);
    assert_eq!(
        bad.diagnostics[0].source.key,
        "MetadataMassUnitDimensionsConflict"
    );
    assert!(bad.preserved_diagnostics().is_empty());
    assert!(bad.snapshot.is_none());
    let physical = data.replace(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,#11))",
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,#11,#905))",
    );
    let (_, bad) = check(&physical, &options(), s::Limits::fixtures());
    assert_eq!(bad.status, s::Status::Rejected);
    assert_eq!(bad.diagnostics[0].source.key, "DerivedSlotMarkerRequired");
    assert_eq!(bad.interoperability, "Rejected");
    assert_eq!(bad.types, "Rejected");
    assert!(bad.snapshot.is_none());
    assert!(bad.preserved_diagnostics().is_empty());
}

#[test]
fn missing_application_protocol_is_diagnosed_without_a_synthetic_role() {
    for schema in [MBX, "CONFIG_CONTROL_DESIGN"] {
        let data = plane(schema)
            .lines()
            .filter(|line| !line.starts_with("#2="))
            .collect::<Vec<_>>()
            .join("\n");
        let (_, result) = check(&data, &options(), s::Limits::fixtures());
        assert_eq!(result.status, s::Status::CheckedWithDiagnostics);
        let part = result.snapshot.as_ref().unwrap().part_definition().unwrap();
        assert!(!part.application_protocol_present);
        assert!(part.apd_status.is_empty() && part.apd_year.is_empty());
        assert!(!part.roles.iter().any(|r| r.name == "application_protocol"));
        assert_eq!(
            result.preserved_diagnostics()[0].source.key,
            "UnsupportedSinglePartGraph"
        );
    }
    let data = plane(AP214)
        .lines()
        .filter(|line| !line.starts_with("#2="))
        .collect::<Vec<_>>()
        .join("\n");
    let (_, result) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(result.status, s::Status::Unsupported);
    assert!(result.snapshot.is_none());
}

#[test]
fn distinct_application_contexts_keep_reference_identities_and_reject_an_extra_context() {
    let data = append(
        &plane(AP214)
            .replace("PRODUCT_CONTEXT('',#1", "PRODUCT_CONTEXT('',#901")
            .replace(
                "PRODUCT_DEFINITION_CONTEXT('part definition',#1",
                "PRODUCT_DEFINITION_CONTEXT('part definition',#902",
            ),
        "#901=APPLICATION_CONTEXT('product app');\n#902=APPLICATION_CONTEXT('definition app');",
    );
    let (_, result) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(result.status, s::Status::CheckedWithDiagnostics);
    let snapshot = result.snapshot.as_ref().unwrap();
    let roles = &snapshot.part_definition().unwrap().roles;
    assert_eq!(
        snapshot.source().records()[roles
            .iter()
            .find(|r| r.name == "product_application_context")
            .unwrap()
            .record_index]
            .id,
        "901"
    );
    assert_eq!(
        snapshot.source().records()[roles
            .iter()
            .find(|r| r.name == "definition_application_context")
            .unwrap()
            .record_index]
            .id,
        "902"
    );
    let extra = append(&data, "#903=APPLICATION_CONTEXT('unselected extra');");
    let (_, bad) = check(&extra, &options(), s::Limits::fixtures());
    assert_eq!(bad.status, s::Status::Unsupported);
    assert_eq!(bad.diagnostics[0].source.key, "UnsupportedSinglePartGraph");
    assert!(bad.snapshot.is_none());
}

#[test]
fn exact_schema_choices_retain_unregistered_identity_and_original_refusal() {
    for (schema, interpretation) in [
        (EIGHT, s::SchemaInterpretation::UnregisteredAp214As2010),
        (
            "AUTOMOTIVE_DESIGN",
            s::SchemaInterpretation::Ap214NameOnlyAs2010,
        ),
    ] {
        let data = plane(schema);
        let (_, strict) = check(&data, &options(), s::Limits::fixtures());
        assert_eq!(strict.status, s::Status::Unsupported);
        let opt = s::Options {
            schema_interpretation: interpretation,
            ..options()
        };
        let (_, result) = check(&data, &opt, s::Limits::fixtures());
        assert_eq!(result.status, s::Status::CheckedWithDiagnostics);
        assert_eq!(result.binding, "InterpretedUnverified");
        assert_eq!(result.applied_interpretation(), interpretation);
        assert_eq!(
            result.schema_selection.as_ref().unwrap().identifiers[0].literal,
            schema
        );
        assert_eq!(
            serde_json::to_value(result.interpretation_diagnostics()).unwrap(),
            serde_json::to_value(&strict.diagnostics).unwrap()
        );
        assert_eq!(
            result.snapshot.as_ref().unwrap().profile().profile,
            "214-MBx2010-Source-0.1"
        );
    }
    for schema in [
        "AUTOMOTIVE_DESIGN_CC2",
        "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1 2}",
        "AUTOMOTIVE_DESIGN {01 0 10303 214 3 1 1 1}",
    ] {
        let (_, result) = check(
            &plane(schema),
            &s::Options {
                schema_interpretation: s::SchemaInterpretation::UnregisteredAp214As2010,
                ..options()
            },
            s::Limits::fixtures(),
        );
        assert_eq!(result.status, s::Status::Unsupported);
        assert_eq!(
            result.applied_interpretation(),
            s::SchemaInterpretation::DeclaredOnly
        );
        assert!(result.snapshot.is_none());
    }
}

#[test]
fn schema_choices_are_not_applied_to_registered_bindings_or_without_explicit_policy() {
    let opt = s::Options {
        schema_interpretation: s::SchemaInterpretation::UnregisteredAp214As2010,
        ..options()
    };
    let (_, good) = check(&plane(AP214), &opt, s::Limits::fixtures());
    assert_eq!(good.status, s::Status::Checked);
    assert_eq!(
        good.applied_interpretation(),
        s::SchemaInterpretation::DeclaredOnly
    );
    for bad in [
        s::Options {
            source_policy: s::SourcePolicy::Strict,
            ..opt.clone()
        },
        s::Options {
            geometry_scope: s::GeometryScope::ExplicitOuterBounds,
            ..opt
        },
    ] {
        let (_, result) = check(&plane(EIGHT), &bad, s::Limits::fixtures());
        assert_eq!(result.status, s::Status::InvalidRequest);
        assert!(result.snapshot.is_none());
    }
}

#[test]
fn schema_assumption_and_metadata_share_capacity_without_partial_approval() {
    let opt = s::Options {
        schema_interpretation: s::SchemaInterpretation::UnregisteredAp214As2010,
        ..options()
    };
    let data = append(&plane(EIGHT), DENSITY);
    for (capacity, expected) in [
        (0, s::Status::ResourceLimit),
        (1, s::Status::ResourceLimit),
        (2, s::Status::CheckedWithDiagnostics),
    ] {
        let (_, result) = check(
            &data,
            &opt,
            s::Limits {
                preserved_diagnostics: capacity,
                ..s::Limits::fixtures()
            },
        );
        assert_eq!(result.status, expected);
        if capacity < 2 {
            assert_eq!(result.diagnostics[0].source.key, "PreservedDiagnosticLimit");
            assert!(result.snapshot.is_none());
            assert!(result.preserved_diagnostics().is_empty());
        }
        if capacity == 0 {
            assert!(result
                .schema_selection
                .as_ref()
                .unwrap()
                .qualified
                .is_none());
            assert_eq!(
                result.applied_interpretation(),
                s::SchemaInterpretation::DeclaredOnly
            );
        } else {
            assert_eq!(result.interpretation_diagnostics().len(), 1);
        }
    }
}

#[test]
fn orphan_palette_stays_unattached_and_does_not_hide_extra_geometry() {
    let data = append(&plane(AP214), "#970=COLOUR_RGB('unassigned',0.2,0.4,0.6);");
    let (_, result) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(result.status, s::Status::CheckedWithDiagnostics);
    let snapshot = result.snapshot.as_ref().unwrap();
    assert_eq!(snapshot.unattached_non_geometric().len(), 1);
    let r = snapshot.unattached_non_geometric()[0];
    assert_eq!(snapshot.source().records()[r].id, "970");
    assert!(!snapshot
        .part_definition()
        .unwrap()
        .roles
        .iter()
        .any(|role| role.record_index == r));
    assert_eq!(
        result.preserved_diagnostics()[0].source.key,
        "UnownedOrOrphanProfileContent"
    );
    let (_, bad) = check(
        &append(&data, "#971=CARTESIAN_POINT('extra geometry',(0.,0.,0.));"),
        &options(),
        s::Limits::fixtures(),
    );
    assert_eq!(bad.status, s::Status::Unsupported);
    assert!(bad.snapshot.is_none());
    assert!(bad.unattached_non_geometric().is_empty());
}

#[test]
fn opaque_selected_id_is_unavailable_text_and_preservation_needs_exact_field_proof() {
    for (marker, expected) in [
        ("PRODUCT('", s::Status::CheckedWithDiagnostics),
        (
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION('",
            s::Status::Unsupported,
        ),
    ] {
        let text = plane(AP214);
        let position = text.find(marker).unwrap() + marker.len();
        let mut bytes = text.into_bytes();
        bytes.insert(position, 0xff);
        let source = inspect_with_options(
            bytes.clone(),
            Limits::planar_fixtures(),
            InspectOptions {
                data_string_policy: DataStringPolicy::PreserveOpaqueBytes,
                ..Default::default()
            },
        );
        let outcome = s::check(&source, s::Limits::fixtures(), &options());
        assert_eq!(outcome.status, expected);
        if expected == s::Status::CheckedWithDiagnostics {
            let snapshot = outcome.snapshot.as_ref().unwrap();
            let index = snapshot.instances("PRODUCT").unwrap()[0];
            assert_eq!(
                snapshot
                    .field(index, None, "id")
                    .unwrap()
                    .value()
                    .unwrap()
                    .interpreted_text(),
                None
            );
            assert_eq!(snapshot.source().bytes(), bytes);
        } else {
            assert_eq!(
                outcome.diagnostics[0].source.key,
                "OpaqueDataTextUnestablished"
            );
            assert!(outcome.snapshot.is_none());
        }
    }
}

#[test]
fn legacy_carrier_qualifies_only_under_explicit_general_interoperability() {
    let bytes = plane(AP214).replacen("'2;1'", "'1'", 1).into_bytes();
    let source = inspect_with_options(
        bytes.clone(),
        Limits::planar_fixtures(),
        InspectOptions {
            carrier_interpretation: CarrierInterpretation::Level1AsCorrectedEncoding,
            ..Default::default()
        },
    );
    let result = s::check(&source, s::Limits::fixtures(), &options());
    assert_eq!(result.status, s::Status::CheckedWithDiagnostics);
    assert!(result.preserved_diagnostics().is_empty());
    assert!(result
        .snapshot
        .as_ref()
        .unwrap()
        .source()
        .carrier_assumption()
        .is_some());
    assert_eq!(result.snapshot.as_ref().unwrap().source().bytes(), bytes);
    let bad = s::check(&source, s::Limits::fixtures(), &s::Options::default());
    assert_eq!(bad.status, s::Status::InvalidRequest);
}

#[test]
fn owning_qualification_and_views_survive_outcomes_and_preserve_source_binding() {
    let (mut raw, mut result) = check(
        &append(&plane(AP214), DENSITY),
        &options(),
        s::Limits::fixtures(),
    );
    let snapshot = result.snapshot.as_ref().unwrap().clone();
    let index = snapshot.instances("MEASURE_REPRESENTATION_ITEM").unwrap()[0];
    let view = snapshot.field(index, None, "value_component").unwrap();
    result.diagnostics.clear();
    raw.diagnostics.clear();
    assert_eq!(result.preserved_diagnostics().len(), 1);
    let unrelated = inspect(plane(AP214).into_bytes(), Limits::planar_fixtures());
    assert!(matches!(
        s::compile_report(&unrelated, &result),
        Err(s::ReportError::SourceMismatch)
    ));
    drop(result);
    drop(raw);
    drop(snapshot);
    assert_eq!(view.value().unwrap().children[0].text, "1.");
}

#[test]
fn approval_and_report_budgets_never_publish_partial_qualification() {
    let data = append(&plane(AP214), DENSITY);
    let (_, good) = check(&data, &options(), s::Limits::fixtures());
    let visits = good.dependency_visits;
    let (_, exact) = check(
        &data,
        &options(),
        s::Limits {
            dependency_visits: visits,
            ..s::Limits::fixtures()
        },
    );
    assert_eq!(exact.status, s::Status::CheckedWithDiagnostics);
    let (_, limited) = check(
        &data,
        &options(),
        s::Limits {
            dependency_visits: visits - 1,
            ..s::Limits::fixtures()
        },
    );
    assert_eq!(limited.status, s::Status::ResourceLimit);
    assert_eq!(limited.interoperability, "ResourceLimit");
    assert!(limited.preserved_diagnostics().is_empty());
    assert!(limited.snapshot.is_none());
    let mut limits = Limits::planar_fixtures();
    limits.report_bytes = 1;
    let source = inspect(data.into_bytes(), limits);
    let outcome = s::check(&source, s::Limits::fixtures(), &options());
    assert!(matches!(
        s::compile_report(&source, &outcome),
        Err(s::ReportError::Output(
            solidcore_step::report::Error::ReportByteLimit
        ))
    ));
}

#[test]
fn interoperability_does_not_skip_a_defining_geometry_failure() {
    let (_, result) = check(
        &append(&plane(AP214), "#901=CIRCLE('',#133,-1.);"),
        &options(),
        s::Limits::fixtures(),
    );
    assert_eq!(result.status, s::Status::Rejected);
    assert_eq!(
        result.diagnostics[0].source.key,
        "SourceRuleViolation:POSITIVE_LENGTH_MEASURE.WR1"
    );
    assert_eq!(result.part, "NotRun");
    assert_eq!(result.interoperability, "NotRun");
    assert!(result.snapshot.is_none());
}

#[test]
fn unavailable_raw_lexemes_keep_every_byte_without_replacement_decoding() {
    for (raw, expected) in [
        (b"'\xff'".as_slice(), "'ÿ'"),
        (b"'\xc3\xa9'".as_slice(), "'Ã©'"),
    ] {
        let text = plane(AP214);
        let prefix = "#203=EDGE_CURVE('',#110,#111,#202,";
        let position = text.find(prefix).unwrap() + prefix.len();
        let mut bytes = text.into_bytes();
        bytes.splice(position..position + 3, raw.iter().copied());
        let source = inspect_with_options(
            bytes,
            Limits::planar_fixtures(),
            InspectOptions {
                data_string_policy: DataStringPolicy::PreserveOpaqueBytes,
                ..Default::default()
            },
        );
        let result = s::check(&source, s::Limits::fixtures(), &options());
        assert_eq!(result.status, s::Status::Rejected);
        assert_eq!(result.diagnostics[0].source.key, "BooleanValueInvalid");
        assert_eq!(result.diagnostics[0].source.parameter_path, [5]);
        assert_eq!(result.diagnostics[0].actual, expected);
        assert_eq!(
            source
                .source
                .as_ref()
                .unwrap()
                .raw(result.diagnostics[0].source.primary),
            Some(raw)
        );
        assert!(result.snapshot.is_none());
    }
}

#[test]
fn decoded_unicode_in_a_schema_refusal_stays_unicode_with_original_source_span() {
    let data = plane("A {1\\X2\\4E2D\\X0\\}");
    let (source, result) = check(&data, &options(), s::Limits::fixtures());
    assert_eq!(result.status, s::Status::Unsupported);
    assert_eq!(result.diagnostics[0].actual, "A {1中}");
    assert_eq!(
        source
            .source
            .as_ref()
            .unwrap()
            .raw(result.diagnostics[0].source.primary),
        Some(b"'A {1\\X2\\4E2D\\X0\\}'".as_slice())
    );
    assert!(result.snapshot.is_none());
}
