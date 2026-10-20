// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual planar CLI request, publication and custom-quality boundaries.
#![cfg(feature = "cli")]
use std::path::PathBuf;
use std::process::{Command, Output};
fn file() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/plane.step")
}
fn invoke(flags: &[&str]) -> std::io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_step-planar-inspect"))
        .args(flags)
        .arg(file())
        .output()
}

#[test]
fn explicit_quality_and_limits_are_required_and_frozen_quality_cannot_be_redefined() {
    for flags in [
        vec![],
        vec!["--fixture-limits"],
        vec!["--quality", "Q-P0-mm-01"],
        vec!["--fixture-limits", "--quality", "custom"],
        vec![
            "--fixture-limits",
            "--quality",
            "Q-P0-mm-01",
            "--consistency-mm",
            "1/20000000",
        ],
        vec![
            "--fixture-limits",
            "--quality",
            "custom",
            "--consistency-mm",
            "0",
        ],
        vec![
            "--fixture-limits",
            "--quality",
            "custom",
            "--consistency-mm",
            "1/0",
        ],
        vec![
            "--fixture-limits",
            "--quality",
            "Q-P0-mm-01",
            "--planar-work",
            "10000001",
        ],
    ] {
        let out = invoke(&flags).unwrap();
        assert_eq!(out.status.code(), Some(64));
        assert!(out.stdout.is_empty());
    }
    assert!(invoke(&["--help"]).unwrap().status.success());
}

#[test]
fn report_budget_counts_final_lf_and_never_publishes_partial_json() {
    let base = ["--fixture-limits", "--quality", "Q-P0-mm-01"];
    let good = invoke(&base).unwrap();
    assert_eq!(good.status.code(), Some(0));
    assert!(good.stdout.ends_with(b"\n"));
    assert!(!good.stdout.ends_with(b"\r\n"));
    let json: serde_json::Value = serde_json::from_slice(&good.stdout).unwrap();
    assert_eq!(json["qualified_source_evidence"], true);
    assert!(json["model"].is_null());
    for key in [
        "native_build",
        "full_import",
        "target_uv",
        "material_volume",
        "shell_self_intersection",
    ] {
        assert_eq!(json["checks"][key], "NotRun");
    }
    let needle = b"\"report_bytes\":16777216";
    let copies = good
        .stdout
        .windows(needle.len())
        .filter(|w| *w == needle)
        .count();
    assert_eq!(copies, 1);
    let mut limit = good.stdout.len();
    for _ in 0..4 {
        limit = good.stdout.len() + limit.to_string().len() - "16777216".len();
    }
    let exact = invoke(&[
        "--fixture-limits",
        "--quality",
        "Q-P0-mm-01",
        "--report-bytes",
        &limit.to_string(),
    ])
    .unwrap();
    assert_eq!(exact.status.code(), Some(0));
    assert_eq!(exact.stdout.len(), limit);
    let short = invoke(&[
        "--fixture-limits",
        "--quality",
        "Q-P0-mm-01",
        "--report-bytes",
        &(limit - 1).to_string(),
    ])
    .unwrap();
    assert_eq!(short.status.code(), Some(5));
    assert!(short.stdout.is_empty());
    let input = invoke(&[
        "--fixture-limits",
        "--quality",
        "Q-P0-mm-01",
        "--input-bytes",
        "1",
    ])
    .unwrap();
    assert_eq!(input.status.code(), Some(3));
    let json: serde_json::Value = serde_json::from_slice(&input.stdout).unwrap();
    assert_eq!(json["status"], "UpstreamFailure");
    assert_eq!(json["qualified_source_evidence"], false);
}

#[test]
fn named_custom_quality_controls_source_consistency_without_rewriting_source() {
    let directory =
        std::env::temp_dir().join(format!("solidcore-planar-quality-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("public.step");
    let text = std::fs::read_to_string(file()).unwrap().replace(
        "#130=CARTESIAN_POINT('',(3.,-2.,5.));",
        "#130=CARTESIAN_POINT('',(3.,-2.,5.0000001));",
    );
    std::fs::write(&path, &text).unwrap();
    for (flags, code) in [
        (vec!["--quality", "Q-P0-mm-01"], 0),
        (
            vec![
                "--quality",
                "custom-tight",
                "--consistency-mm",
                "1/20000000",
            ],
            1,
        ),
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_step-planar-inspect"))
            .arg("--fixture-limits")
            .args(flags)
            .arg(&path)
            .output()
            .unwrap();
        assert_eq!(out.status.code(), Some(code));
        let json: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
        assert_eq!(json["source_semantics"]["source"]["raw"], text);
        assert_eq!(json["checks"]["native_build"], "NotRun");
        if code == 1 {
            assert_eq!(
                json["diagnostics"][0]["source"]["key"],
                "EdgeSurfaceMismatch"
            );
        }
    }
    std::fs::remove_file(path).unwrap();
    std::fs::remove_dir(directory).unwrap();
}
