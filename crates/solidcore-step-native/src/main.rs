// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit synthetic construction checkpoint; not a full-import success CLI.
use std::io::Read;

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 2 || args.first().is_none_or(|a| a != "--fixture-limits") {
        return Err("usage: step-native-import --fixture-limits FILE (construction checkpoint, not accepted import)".into());
    }
    let path = args.get(1).ok_or("missing file")?;
    let limits = solidcore_step::Limits::planar_fixtures();
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(u64::try_from(limits.input_bytes)?.saturating_add(1))
        .read_to_end(&mut bytes)?;
    let raw = solidcore_step::inspect(bytes, limits);
    let sem = solidcore_step::semantics::check(
        &raw,
        solidcore_step::semantics::Limits::fixtures(),
        &solidcore_step::semantics::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    let planar = solidcore_step::planar::check(
        &sem,
        solidcore_step::planar::Quality::planar_fixture(),
        solidcore_step::planar::Limits::fixtures(),
    );
    let model = solidcore_step_native::construct_planar(
        &planar,
        solidcore_step_native::Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )?
    .validate_native()?;
    println!(
        "{}",
        serde_json::json!({"status":"NativeConstructed","accepted_import":false,
        "native_kernel":"Rust solidcore","vertices":model.vertices().len(),"edges":model.edges().len(),
        "faces":model.faces().len(),"operations":model.operations(),
        "native_pointer_validation":"Passed","native_deep_validation":"Passed",
        "unrun":["continuous_source_correspondence","error_budgets","final_publication"],"repair":"None"})
    );
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
