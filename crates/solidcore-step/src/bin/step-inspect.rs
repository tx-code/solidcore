// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! CLI for the source-only Rust STEP migration.
//!
//! Upstream: `source/STEP_SOURCE/app/step_inspect.cpp`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | `run` | `main` |
//!
//! Deviations: filesystem lives exclusively in this opt-in binary. Requires an
//! explicit test limit preset; emits solidcore.step.source.v1, not C++ JSON.

#![forbid(unsafe_code)]

use solidcore_step::{inspect, report, Limits, Status};
use std::ffi::OsString;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "step-inspect --fixture-limits [--input-bytes N] [--records N] [--values N] [--references N] [--nesting N] [--string-bytes N] [--record-bytes N] [--report-bytes N] [--id-digits N] [--] FILE\nSource inspection only. Schema semantics, units, geometry and native construction are NOT RUN.\nThe preset is for synthetic tests, not a production import or quality policy.";

fn arguments(args: impl Iterator<Item = OsString>) -> Result<Option<(PathBuf, Limits)>, ()> {
    let mut args = args.peekable();
    let mut limits = Limits::planar_fixtures();
    let mut explicit = false;
    let mut path = None;
    let mut options = true;
    while let Some(arg) = args.next() {
        if options && arg == "--help" {
            return Ok(None);
        }
        if options && arg == "--fixture-limits" {
            explicit = true;
            continue;
        }
        if options && arg == "--" {
            options = false;
            continue;
        }
        if options && arg.to_str().is_some_and(|s| s.starts_with('-')) {
            let option = arg.to_str().ok_or(())?;
            let value = args
                .next()
                .ok_or(())?
                .to_str()
                .filter(|s| !s.is_empty() && s.bytes().all(|c| c.is_ascii_digit()))
                .ok_or(())?
                .parse::<usize>()
                .map_err(|_| ())?;
            match option {
                "--input-bytes" => limits.input_bytes = value,
                "--records" => limits.records = value,
                "--values" => limits.values = value,
                "--references" => limits.references = value,
                "--nesting" => limits.nesting = value,
                "--string-bytes" => limits.string_bytes = value,
                "--record-bytes" => limits.record_bytes = value,
                "--report-bytes" => limits.report_bytes = value,
                "--id-digits" => limits.id_digits = value,
                _ => return Err(()),
            }
        } else if path.replace(PathBuf::from(arg)).is_some() {
            return Err(());
        }
    }
    if !explicit {
        return Err(());
    }
    Ok(Some((path.ok_or(())?, limits)))
}

fn run() -> u8 {
    let (path, limits) = match arguments(std::env::args_os().skip(1)) {
        Ok(Some(request)) => request,
        Ok(None) => {
            println!("{USAGE}");
            return 0;
        }
        Err(()) => {
            eprintln!("{USAGE}");
            return 64;
        }
    };
    // Check limits before I/O; at most input_bytes + 1 are captured, even for a
    // pipe or a file that grows. A size from file metadata never drives allocation.
    let mut bytes = Vec::new();
    if limits.valid() {
        let input = File::open(path).and_then(|file| {
            file.take(limits.input_bytes as u64 + 1)
                .read_to_end(&mut bytes)
        });
        if input.is_err() {
            eprintln!("InputReadFailure: complete source capture unavailable");
            return 4;
        }
    }
    let outcome = inspect(bytes, limits);
    let report = match report::compile(&outcome) {
        Ok(bytes) => bytes,
        Err(report::Error::ReportByteLimit) => {
            eprintln!("ReportByteLimit: no JSON published");
            return 5;
        }
        Err(report::Error::Serialization) => {
            eprintln!("ReportSerializationFailure: no JSON published");
            return 6;
        }
    };
    if std::io::stdout().lock().write_all(&report).is_err() {
        eprintln!("ReportWriteFailure");
        return 6;
    }
    match outcome.status {
        Status::Inspected => 0,
        Status::Rejected => 1,
        Status::Unsupported => 2,
        Status::ResourceLimit => 3,
    }
}

fn main() -> ExitCode {
    ExitCode::from(run())
}
