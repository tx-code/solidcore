// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual strict checkpoint CLI publication and request-boundary contracts.
#![cfg(feature = "cli")]
use std::path::PathBuf;
use std::process::{Command, Output};
fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plane.step")
}
fn invoke(options: &[&str]) -> std::io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args(options)
        .arg(fixture())
        .output()
}

#[test]
fn requested_context_and_unrequested_stages_are_explicit_in_real_cli() {
    let output = invoke(&["--fixture-limits", "--unit-context", "12"]).unwrap();
    assert!(output.status.success());
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["schema"], "solidcore.step.types-units.v1");
    assert_eq!(json["si_units"], "Passed");
    assert_eq!(json["snapshot"]["units"]["length_power10_mm"], 0);
    for phase in [
        "part",
        "geometry_dependencies",
        "native_build",
        "interoperability",
    ] {
        assert_eq!(json[phase], "NotRun");
    }
    let output = invoke(&["--fixture-limits"]).unwrap();
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["si_units"], "NotRun");
    assert!(json["snapshot"]["units"]["length_power10_mm"].is_null());
}

#[test]
fn report_limit_and_semantic_budget_never_publish_a_partial_typed_snapshot() {
    let output = invoke(&["--fixture-limits", "--report-bytes", "1"]).unwrap();
    assert_eq!(output.status.code(), Some(5));
    assert!(output.stdout.is_empty());
    let output = invoke(&["--fixture-limits", "--dependency-visits", "0"]).unwrap();
    assert_eq!(output.status.code(), Some(3));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert!(json["snapshot"].is_null());
    assert_eq!(
        json["diagnostics"][0]["source"]["key"],
        "DependencyVisitLimit"
    );
    assert_eq!(json["source"]["status"], "Inspected");
    let output = invoke(&["--fixture-limits", "--input-bytes", "1"]).unwrap();
    assert_eq!(output.status.code(), Some(1));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["status"], "SourceFailure");
    assert_eq!(json["source"]["status"], "ResourceLimit");
    assert!(json["snapshot"].is_null());
}

#[test]
fn invalid_requests_and_unqualified_interpretation_flags_are_refused() {
    for options in [
        vec![],
        vec!["--fixture-limits", "--unit-context", "1"],
        vec!["--fixture-limits", "--unit-context", "012"],
        vec!["--fixture-limits", "--dependency-visits", "-1"],
        vec!["--fixture-limits", "--unknown-policy"],
        vec!["--fixture-limits", "--interpret-unregistered-ap214-2010"],
        vec!["--fixture-limits", "--interpret-level1-as-corrected"],
        vec!["--fixture-limits", "--interpret-level1-with-lowercase-hex"],
    ] {
        let output = invoke(&options).unwrap();
        assert_eq!(output.status.code(), Some(64));
        assert!(output.stdout.is_empty());
    }
    let missing = fixture().join("absent.step");
    for flag in [
        "--interpret-level1-as-corrected",
        "--interpret-unregistered-ap214-2010",
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
            .args(["--fixture-limits", flag])
            .arg(&missing)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(64));
        assert!(output.stdout.is_empty());
    }
}

#[test]
fn opaque_capture_cannot_turn_unavailable_product_text_into_checked_ownership() {
    let directory = std::env::temp_dir().join(format!(
        "solidcore-opaque-semantic-cli-{}",
        std::process::id()
    ));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("public.step");
    let original = std::fs::read(fixture()).unwrap();
    for (marker, expected) in [(b"FILE_NAME('".as_slice(), 0), (b"PRODUCT('".as_slice(), 2)] {
        let position = original
            .windows(marker.len())
            .position(|w| w == marker)
            .unwrap()
            + marker.len();
        let mut bytes = original.clone();
        bytes.insert(position, 0xff);
        std::fs::write(&path, &bytes).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
            .args([
                "--fixture-limits",
                "--source-semantics",
                "--preserve-opaque-header-name",
                "--preserve-opaque-data-strings",
            ])
            .arg(&path)
            .output()
            .unwrap();
        assert_eq!(output.status.code(), Some(expected));
        let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(json["source"]["status"], "Inspected");
        assert_eq!(json["native_build"], "NotRun");
        assert_eq!(json["interoperability"], "NotRun");
        if expected == 2 {
            assert_eq!(
                json["diagnostics"][0]["source"]["key"],
                "OpaqueDataTextUnestablished"
            );
            assert!(json["snapshot"].is_null());
            assert_eq!(json["part"], "NotRun");
        } else {
            assert_eq!(json["part"], "Passed");
        }
    }
    std::fs::remove_file(path).unwrap();
    std::fs::remove_dir(directory).unwrap();
}

#[test]
fn explicit_schema_cli_preserves_unverified_identity_and_refuses_zero_capacity() {
    let directory =
        std::env::temp_dir().join(format!("solidcore-schema-cli-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("public.step");
    let bytes = std::fs::read_to_string(fixture()).unwrap().replace(
        "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}",
        "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1 1}");
    std::fs::write(&path, &bytes).unwrap();
    let command = [
        "--fixture-limits",
        "--general-brep-source",
        "--interop-nongeometric",
        "--interpret-unregistered-ap214-2010",
    ];
    let good = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args(command)
        .arg(&path)
        .output()
        .unwrap();
    assert_eq!(good.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&good.stdout).unwrap();
    assert_eq!(json["status"], "CheckedWithDiagnostics");
    assert_eq!(json["binding"], "InterpretedUnverified");
    assert_eq!(json["source_binding_verified"], false);
    assert_eq!(
        json["interpretation_diagnostics"][0]["source"]["key"],
        "UnsupportedSchemaBinding"
    );
    assert_eq!(json["source"]["raw"], bytes);
    assert_eq!(json["native_build"], "NotRun");
    let limited = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args(command)
        .args(["--preserved-diagnostics", "0"])
        .arg(&path)
        .output()
        .unwrap();
    assert_eq!(limited.status.code(), Some(3));
    let json: serde_json::Value = serde_json::from_slice(&limited.stdout).unwrap();
    assert!(json["snapshot"].is_null());
    assert!(json["schema_selection"]["qualified"].is_null());
    assert_eq!(
        json["diagnostics"][0]["source"]["key"],
        "PreservedDiagnosticLimit"
    );
    std::fs::remove_file(path).unwrap();
    std::fs::remove_dir(directory).unwrap();
}

#[test]
fn ownership_mode_selects_its_proved_context_and_keeps_unrun_qualification_explicit() {
    let output = invoke(&["--fixture-limits", "--part-and-closure"]).unwrap();
    assert_eq!(output.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["schema"], "solidcore.step.part-closure.v1");
    assert_eq!(json["part"], "Passed");
    assert_eq!(json["si_units"], "Passed");
    assert_eq!(
        json["snapshot"]["part_definition"]["shape_role"],
        "OpenSurfaceModel"
    );
    assert_eq!(json["native_build"], "NotRun");
    assert_eq!(json["geometry_dependencies"], "NotRun");
    let output = invoke(&[
        "--fixture-limits",
        "--part-and-closure",
        "--unit-context",
        "1",
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(64));
    assert!(output.stdout.is_empty());
    let output = invoke(&[
        "--fixture-limits",
        "--part-and-closure",
        "--report-bytes",
        "1",
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(5));
    assert!(output.stdout.is_empty());
    let output = invoke(&[
        "--fixture-limits",
        "--part-and-closure",
        "--input-bytes",
        "1",
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(3));
}

#[test]
fn deferred_ownership_mode_retains_the_cpp_exit_code_without_changing_old_mode() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plane.step");
    let directory =
        std::env::temp_dir().join(format!("solidcore-part-deferred-{}", std::process::id()));
    std::fs::create_dir(&directory).unwrap();
    let input = directory.join("deferred.step");
    std::fs::write(
        &input,
        std::fs::read_to_string(path)
            .unwrap()
            .replace("FACE_OUTER_BOUND(", "FACE_BOUND("),
    )
    .unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_step-semantic-inspect"))
        .args([
            "--fixture-limits",
            "--part-and-closure",
            "--general-brep-source",
        ])
        .arg(&input)
        .output()
        .unwrap();
    std::fs::remove_file(input).unwrap();
    std::fs::remove_dir(directory).unwrap();
    assert_eq!(output.status.code(), Some(7));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["status"], "Deferred");
    assert_eq!(json["part"], "Deferred");
}

#[test]
fn full_source_mode_keeps_p0_geometry_scope_and_reports_unrun_native_work() {
    let output = invoke(&["--fixture-limits", "--source-semantics"]).unwrap();
    assert_eq!(output.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["schema"], "solidcore.step.source-semantics.v1");
    assert_eq!(json["geometry_dependencies"], "NotRun");
    assert_eq!(json["part"], "Passed");
    assert_eq!(json["full_import"], "NotRun");
    let output = invoke(&[
        "--fixture-limits",
        "--source-semantics",
        "--general-brep-source",
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["geometry_dependencies"], "Passed");
    assert_eq!(json["geometry_consistency"], "NotRun");
    assert_eq!(json["planar_rules"], "NotRun");
    assert_eq!(json["native_build"], "NotRun");
    let output = invoke(&[
        "--fixture-limits",
        "--source-semantics",
        "--general-brep-source",
        "--report-bytes",
        "1",
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(5));
    assert!(output.stdout.is_empty());
}
