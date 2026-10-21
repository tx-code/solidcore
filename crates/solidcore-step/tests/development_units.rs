// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public synthetic truth for the independent, sealed development-unit policy.
use solidcore_step::{inspect, semantics as s, Limits, ValueKind};

const AP203: &str = "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
const SI_ANGLE: &str = "#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));";
const EXPLICIT: &str =
    "#11=(CONVERSION_BASED_UNIT('arbitrary angle name',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());";
const EXTRA: &str =
    "#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#901);
#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));
#902=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);";

fn source(marker: &str) -> String {
    // Literal fixture authoring: a correct '*' does not reference #902, so
    // including that record would introduce an independent orphan defect.
    // The reference-valued single-defect fixture includes its real target.
    let extra = if marker == "*" {
        EXTRA
            .lines()
            .filter(|line| !line.starts_with("#902="))
            .collect::<Vec<_>>()
            .join("\n")
    } else {
        EXTRA.to_owned()
    };
    include_str!("fixtures/plane.step")
        .replace(AP203, AP214)
        .replace(
            SI_ANGLE,
            &EXPLICIT.replace("NAMED_UNIT(#902)", &format!("NAMED_UNIT({marker})")),
        )
        .replace("ENDSEC;\nEND-ISO", &format!("{extra}\nENDSEC;\nEND-ISO"))
}
fn options(enabled: bool) -> s::Options {
    s::Options {
        geometry_scope: s::GeometryScope::GeneralBrep,
        check_geometry_dependencies: true,
        check_part_and_closure: true,
        development_unit_policy: if enabled {
            s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions
        } else {
            s::DevelopmentUnitPolicy::Disabled
        },
        ..Default::default()
    }
}
fn check(data: &str, enabled: bool) -> (solidcore_step::Outcome, s::Outcome) {
    let raw = inspect(data.as_bytes().to_vec(), Limits::planar_fixtures());
    let checked = s::check(&raw, s::Limits::fixtures(), &options(enabled));
    (raw, checked)
}

#[test]
fn independent_policy_keeps_original_source_error_and_seals_the_exact_source_proof() {
    let data = source("#902");
    let (_, strict) = check(&data, false);
    assert_eq!(strict.status, s::Status::Rejected);
    assert_eq!(
        strict.diagnostics[0].source.key,
        "DerivedSlotMarkerRequired"
    );
    let (raw, mut out) = check(&data, true);
    assert_eq!(out.status, s::Status::CheckedWithDiagnostics);
    assert_eq!(out.source_policy(), s::SourcePolicy::Strict);
    assert_eq!(out.interoperability, "NotRun");
    assert!(out.preserved_diagnostics().is_empty());
    assert!(out.diagnostics.is_empty());
    assert_eq!(
        serde_json::to_value(out.development_unit_diagnostics()).unwrap(),
        serde_json::to_value(&strict.diagnostics).unwrap()
    );
    let snapshot = out.snapshot.clone().unwrap();
    assert_eq!(snapshot.source().bytes(), data.as_bytes());
    let unit = snapshot.units().plane_angle_unit.unwrap();
    let field = snapshot
        .field(unit, Some("NAMED_UNIT"), "dimensions")
        .unwrap();
    assert_eq!(field.value().unwrap().kind, ValueKind::Reference);
    assert!(field.declaration().unwrap().derived_slot);
    let receipt = snapshot.development_unit_receipt().unwrap();
    assert_eq!(receipt.affected_units(), [unit]);
    assert!(!receipt.source_syntax_verified());
    assert_eq!(receipt.scale_to_radians(unit), Some(0.0174532925199433));
    assert_eq!(receipt.unit_definitions().len(), 2);
    assert_eq!(
        receipt
            .unit_definitions()
            .iter()
            .find(|d| d.record_index() == unit)
            .unwrap()
            .factor_lexeme(),
        Some("0.0174532925199433")
    );
    // Clearing resettable progress/diagnostics never upgrades the sealed receipt.
    out.status = s::Status::Checked;
    out.types = "Passed";
    out.diagnostics.clear();
    let json: serde_json::Value =
        serde_json::from_slice(&s::compile_report(&raw, &out).unwrap()).unwrap();
    assert_eq!(json["schema"], "solidcore.step.development-unit-source.v1");
    assert_eq!(
        json["development_unit_receipt"]["sourceSyntaxVerified"],
        false
    );
    assert_eq!(
        json["development_unit_receipt"]["formalAPValidity"],
        "Unestablished"
    );
    assert_eq!(
        json["development_unit_diagnostics"][0]["source"]["key"],
        "DerivedSlotMarkerRequired"
    );
    assert_eq!(json["native_build"], "NotRun");
    assert_eq!(json["full_import"], "NotRun");
    drop(out);
    drop(raw);
    assert_eq!(
        snapshot
            .development_unit_receipt()
            .unwrap()
            .affected_units(),
        [unit]
    );
    assert_eq!(field.value().unwrap().kind, ValueKind::Reference);
}

#[test]
fn the_marker_is_the_only_source_defect_and_default_reports_remain_strict() {
    let data = source("*");
    let (raw, strict) = check(&data, false);
    assert_eq!(strict.status, s::Status::Checked);
    let (_, selected) = check(&data, true);
    assert_eq!(selected.status, s::Status::Checked);
    assert!(selected.development_unit_receipt().is_none());
    assert!(selected.development_unit_diagnostics().is_empty());
    assert_eq!(strict.dependency_visits, selected.dependency_visits);
    assert_eq!(
        serde_json::to_value(strict.unit_dependency_checks).unwrap(),
        serde_json::to_value(selected.unit_dependency_checks).unwrap()
    );
    let again = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            development_unit_policy: s::DevelopmentUnitPolicy::Disabled,
            ..options(false)
        },
    );
    let first = s::compile_report(&raw, &strict).unwrap();
    assert_eq!(first, s::compile_report(&raw, &again).unwrap());
    let json: serde_json::Value = serde_json::from_slice(&first).unwrap();
    assert_eq!(json["schema"], "solidcore.step.source-semantics.v1");
    assert!(!json
        .as_object()
        .unwrap()
        .contains_key("development_unit_policy"));
    assert!(!json
        .as_object()
        .unwrap()
        .contains_key("source_syntax_verified"));
    let bad = source("#902");
    let (raw_bad, strict_bad) = check(&bad, false);
    assert_eq!(raw_bad.status, solidcore_step::Status::Inspected);
    assert_eq!(strict_bad.status, s::Status::Rejected);
    assert_eq!(
        strict_bad.diagnostics[0].source.key,
        "DerivedSlotMarkerRequired"
    );
    let nongeometric = s::check(
        &raw_bad,
        s::Limits::fixtures(),
        &s::Options {
            source_policy: s::SourcePolicy::PreserveNonGeometric,
            ..options(false)
        },
    );
    assert_eq!(nongeometric.status, s::Status::Rejected);
    assert_eq!(
        nongeometric.diagnostics[0].source.key,
        "DerivedSlotMarkerRequired"
    );
}

#[test]
fn each_nonzero_dimension_is_rejected_exactly_including_tiny_values() {
    for index in 0..7 {
        let mut dimensions = ["0."; 7];
        dimensions[index] = "1.E-18";
        let bad = source("#902").replace(
            "DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.)",
            &format!("DIMENSIONAL_EXPONENTS({})", dimensions.join(",")),
        );
        let (_, out) = check(&bad, true);
        assert_eq!(out.status, s::Status::Rejected, "dimension {index}");
        assert!(out.snapshot.is_none() && out.development_unit_receipt().is_none());
        assert_eq!(out.diagnostics[0].source.key, "DerivedSlotMarkerRequired");
        assert!(out
            .diagnostics
            .iter()
            .any(|d| d.source.key == "DevelopmentPlaneAngleDimensionsMustBeZero"));
    }
}

#[test]
fn factor_kind_nominal_dependency_sign_range_and_cycles_are_not_waived() {
    let good = source("#902");
    let cases = [
        good.replace("PLANE_ANGLE_MEASURE(0.0174532925199433)", "RATIO_MEASURE(0.0174532925199433)"),
        good.replace("0.0174532925199433", "0."),
        good.replace("0.0174532925199433", "-1."),
        good.replace("0.0174532925199433", "1.E400"),
        good.replace("0.0174532925199433", "1.E-400"),
        good.replace("0.0174532925199433),#901", "0.0174532925199433),#10"),
        good.replace("#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#901=(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT());"),
        good.replace("#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#901=(CONVERSION_BASED_UNIT('cycle',#903) NAMED_UNIT(*) PLANE_ANGLE_UNIT());\n#903=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(1.),#11);"),
    ];
    for bad in cases {
        let (_, out) = check(&bad, true);
        assert!(matches!(
            out.status,
            s::Status::Rejected | s::Status::Unsupported
        ));
        assert!(out.snapshot.is_none() && out.development_unit_receipt().is_none());
        assert!(out
            .diagnostics
            .iter()
            .any(|d| d.source.key == "DerivedSlotMarkerRequired"));
        assert!(!out.development_unit_diagnostics().is_empty());
    }
}

#[test]
fn every_conversion_level_and_original_factor_is_proved_without_name_inference() {
    let data = source("#902")
        .replace("0.0174532925199433", "0.5")
        .replace("arbitrary angle name", "DEGREE")
        .replace("#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#901=(CONVERSION_BASED_UNIT('RADIAN',#903) NAMED_UNIT(*) PLANE_ANGLE_UNIT());\n#903=MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(2.),#904);\n#904=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));");
    let (_, out) = check(&data, true);
    assert_eq!(out.status, s::Status::CheckedWithDiagnostics);
    let snapshot = out.snapshot.as_ref().unwrap();
    let unit = snapshot.units().plane_angle_unit.unwrap();
    let receipt = snapshot.development_unit_receipt().unwrap();
    assert_eq!(receipt.scale_to_radians(unit), Some(1.));
    assert_eq!(receipt.unit_definitions().len(), 3);
    let bad = data.replace("PLANE_ANGLE_MEASURE(2.)", "PLANE_ANGLE_MEASURE(-2.)");
    let (_, rejected) = check(&bad, true);
    assert_eq!(rejected.status, s::Status::Rejected);
    assert!(rejected.development_unit_receipt().is_none());
}

#[test]
fn resource_and_later_ownership_failures_never_publish_a_partial_receipt() {
    let data = source("#902");
    let (raw, passed) = check(&data, true);
    let visits = passed.dependency_visits;
    assert!(passed.snapshot.is_some());
    let exact = s::check(
        &raw,
        s::Limits {
            dependency_visits: visits,
            ..s::Limits::fixtures()
        },
        &options(true),
    );
    assert_eq!(exact.status, s::Status::CheckedWithDiagnostics);
    let short = s::check(
        &raw,
        s::Limits {
            dependency_visits: visits - 1,
            ..s::Limits::fixtures()
        },
        &options(true),
    );
    assert_eq!(short.status, s::Status::ResourceLimit);
    assert!(short.snapshot.is_none() && short.development_unit_receipt().is_none());
    let quota = s::check(
        &raw,
        s::Limits {
            preserved_diagnostics: 0,
            ..s::Limits::fixtures()
        },
        &options(true),
    );
    assert_eq!(quota.status, s::Status::ResourceLimit);
    assert_eq!(quota.diagnostics[0].source.key, "DerivedSlotMarkerRequired");
    let orphan = data.replace(
        "ENDSEC;\nEND-ISO",
        "#999=CARTESIAN_POINT('',(0.,0.,0.));\nENDSEC;\nEND-ISO",
    );
    let (_, out) = check(&orphan, true);
    assert!(out.snapshot.is_none() && out.development_unit_receipt().is_none());
    assert!(out
        .diagnostics
        .iter()
        .any(|d| d.source.key == "DerivedSlotMarkerRequired"));
}

#[test]
fn explicit_general_scope_and_complete_source_ownership_are_required() {
    let data = source("#902");
    let raw = inspect(data.into_bytes(), Limits::planar_fixtures());
    let wrong_scope = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            development_unit_policy: s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions,
            ..Default::default()
        },
    );
    assert_eq!(wrong_scope.status, s::Status::InvalidRequest);
    let full = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            check_geometry_dependencies: false,
            check_part_and_closure: false,
            ..options(true)
        },
    );
    assert_eq!(full.status, s::Status::CheckedWithDiagnostics);
    assert_ne!(full.part, "NotRun");
    assert_ne!(full.geometry_dependencies, "NotRun");
    let ap203 = inspect(
        source("#902").replace(AP214, AP203).into_bytes(),
        Limits::planar_fixtures(),
    );
    let unsupported = s::check(&ap203, s::Limits::fixtures(), &options(true));
    assert_eq!(unsupported.status, s::Status::Unsupported);
    assert!(unsupported.snapshot.is_none());
}

#[test]
fn a_genuine_receipt_cannot_be_transplanted_into_another_source_report() {
    let (raw_a, mut a) = check(&source("#902"), true);
    let (_, b) = check(&source("#902").replace("0.0174532925199433", "0.5"), true);
    a.snapshot = b.snapshot;
    assert_eq!(
        s::compile_report(&raw_a, &a),
        Err(s::ReportError::SourceMismatch)
    );
}

#[test]
#[cfg(feature = "cli")]
fn explicit_semantic_cli_option_reports_the_independent_development_scope() {
    use std::process::Command;
    let file = std::env::temp_dir().join(format!(
        "solidcore-step-angle-policy-{}.step",
        std::process::id()
    ));
    std::fs::write(&file, source("#902")).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args([
            "--fixture-limits",
            "--general-brep-source",
            "--development-ap214-angle-dimensions",
        ])
        .arg(&file)
        .output()
        .unwrap();
    let invalid = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args(["--fixture-limits", "--development-ap214-angle-dimensions"])
        .arg(&file)
        .output()
        .unwrap();
    std::fs::remove_file(&file).unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let json: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(json["schema"], "solidcore.step.development-unit-source.v1");
    assert_eq!(json["development_unit_policy_applied"], true);
    assert_eq!(
        json["development_unit_receipt"]["sourceSyntaxVerified"],
        false
    );
    assert_eq!(
        json["development_unit_receipt"]["formalAPValidity"],
        "Unestablished"
    );
    assert_eq!(invalid.status.code(), Some(64));
    assert!(invalid.stdout.is_empty());
}
