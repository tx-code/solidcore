// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Real CLI publication and capture tests, using public synthetic input only.
#![cfg(feature = "cli")]

use std::path::PathBuf;
use std::process::{Command, Output};

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plane.step")
}

fn invoke(args: &[&str]) -> std::io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_step-inspect"))
        .args(args)
        .output()
}

#[test]
fn requires_explicit_limits_and_rejects_unsupported_options() {
    for args in [
        vec![],
        vec!["missing.step"],
        vec!["--fixture-limits"],
        vec!["--fixture-limits", "--guess-data-encoding", "file.step"],
        vec!["--fixture-limits", "--records", "-1", "file.step"],
        vec!["--fixture-limits", "--records", "+1", "file.step"],
        vec!["--fixture-limits", "one.step", "two.step"],
        vec![
            "--fixture-limits",
            "--interpret-level1-as-corrected",
            "--interpret-level1-with-lowercase-hex",
            "file.step",
        ],
    ] {
        let output = invoke(&args).unwrap();
        assert_eq!(output.status.code(), Some(64));
        assert!(output.stdout.is_empty());
    }
    let help = invoke(&["--help"]).unwrap();
    assert!(help.status.success());
    assert!(String::from_utf8(help.stdout).unwrap().contains("NOT RUN"));
}

#[test]
fn explicit_unused_carrier_options_are_reported_without_granting_qualification() {
    let file = fixture();
    let output = invoke(&[
        "--fixture-limits",
        "--preserve-opaque-header-name",
        "--preserve-opaque-data-strings",
        "--interpret-level1-as-corrected",
        file.to_str().unwrap(),
    ])
    .unwrap();
    assert_eq!(output.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["header_name_policy"], "PreserveOpaqueBytes");
    assert_eq!(json["data_string_policy"], "PreserveOpaqueBytes");
    assert_eq!(json["carrier_interpretation"]["status"], "NotApplied");
    assert_eq!(
        json["carrier_interpretation"]["declaration_verified"],
        false
    );
    assert_eq!(json["scope"]["native_build"], "NotRun");
}

#[test]
fn report_limit_publishes_no_json_and_input_limit_publishes_no_partial_graph() {
    let file = fixture();
    let path = file.to_str().unwrap();
    let good = invoke(&["--fixture-limits", path]).unwrap();
    assert_eq!(good.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&good.stdout).unwrap();
    assert_eq!(json["status"], "Inspected");
    assert!(good.stdout.ends_with(b"\n"));
    assert!(!good.stdout.ends_with(b"\r\n"));
    assert_eq!(
        json["raw"].as_str().unwrap().as_bytes(),
        std::fs::read(&file).unwrap()
    );
    let limited = invoke(&["--fixture-limits", "--report-bytes", "1", path]).unwrap();
    assert_eq!(limited.status.code(), Some(5));
    assert!(limited.stdout.is_empty());
    let limited = invoke(&["--fixture-limits", "--input-bytes", "1", path]).unwrap();
    assert_eq!(limited.status.code(), Some(3));
    let json: serde_json::Value = serde_json::from_slice(&limited.stdout).unwrap();
    assert_eq!(json["diagnostics"][0]["key"], "InputByteLimit");
    assert!(json["records"].is_null());
    assert!(json["sha256"].is_null());
}

#[test]
fn capture_failure_and_invalid_limits_are_distinct() {
    let missing = fixture().join("absent.step");
    let path = missing.to_str().unwrap();
    let output = invoke(&["--fixture-limits", path]).unwrap();
    assert_eq!(output.status.code(), Some(4));
    assert!(output.stdout.is_empty());
    assert!(!String::from_utf8(output.stderr).unwrap().contains(path));
    // Invalid limits are diagnosed before opening even a nonexistent path.
    let output = invoke(&["--fixture-limits", "--nesting", "129", path]).unwrap();
    assert_eq!(output.status.code(), Some(3));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["diagnostics"][0]["key"], "InvalidLimits");
}

#[test]
fn unicode_paths_and_whitespace_are_preserved_at_cli_boundary() {
    let suffix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let directory =
        std::env::temp_dir().join(format!("solidcore-step-{}-{suffix}", std::process::id()));
    std::fs::create_dir(&directory).unwrap();
    let path = directory.join("公开 合成.step");
    std::fs::copy(fixture(), &path).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_step-inspect"))
        .arg("--fixture-limits")
        .arg("--")
        .arg(&path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    std::fs::remove_dir(directory).unwrap();
    assert_eq!(output.status.code(), Some(0));
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["scope"]["native_build"], "NotRun");
}
