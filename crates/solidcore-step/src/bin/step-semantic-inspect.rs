// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Opt-in strict source checkpoint CLI; filesystem stays outside the library.
//! Upstream: `source/STEP_SEMANTICS/app/step_semantics.cpp` (source capture and checks).
//! Deviations: only selected migrated phases; no geometry/native claims or
//! interoperability/legacy-carrier interpretation options.
#![forbid(unsafe_code)]
use solidcore_step::{
    inspect_with_options, report, semantics, DataStringPolicy, HeaderNamePolicy, InspectOptions,
    Limits,
};
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;

const USAGE:&str="step-semantic-inspect --fixture-limits [--general-brep-source] [--source-semantics | --geometry-dependencies | --part-and-closure] [--preserve-opaque-header-name] [--preserve-opaque-data-strings] [--unit-context ID] [--dependency-visits N] [--input-bytes N] [--report-bytes N] FILE\n--source-semantics runs the maintained strict source phases, including geometry prerequisites and source ownership. Continuous geometry/planar qualification and native build are NOT RUN. Opaque source retention does not approve unavailable typed text.";
struct Request {
    path: PathBuf,
    source_limits: Limits,
    source_options: InspectOptions,
    limits: semantics::Limits,
    options: semantics::Options,
}
fn number(value: &std::ffi::OsStr) -> Result<usize, ()> {
    let s = value.to_str().ok_or(())?;
    if s.is_empty() || !s.bytes().all(|c| c.is_ascii_digit()) {
        return Err(());
    }
    s.parse().map_err(|_| ())
}
fn arguments() -> Result<Option<Request>, ()> {
    let mut args = std::env::args_os().skip(1);
    let mut path = None;
    let mut source_limits = Limits::planar_fixtures();
    let mut source_options = InspectOptions::default();
    let mut limits = semantics::Limits::fixtures();
    let mut options = semantics::Options::default();
    let mut explicit = false;
    let mut flags = true;
    while let Some(arg) = args.next() {
        if flags && arg == "--help" {
            return Ok(None);
        }
        if flags && arg == "--" {
            flags = false;
            continue;
        }
        if flags && arg == "--fixture-limits" {
            explicit = true;
            continue;
        }
        if flags && arg == "--general-brep-source" {
            options.geometry_scope = semantics::GeometryScope::GeneralBrep;
            continue;
        }
        if flags && arg == "--preserve-opaque-header-name" {
            source_options.header_name_policy = HeaderNamePolicy::PreserveOpaqueBytes;
            continue;
        }
        if flags && arg == "--preserve-opaque-data-strings" {
            source_options.data_string_policy = DataStringPolicy::PreserveOpaqueBytes;
            continue;
        }
        if flags && arg == "--part-and-closure" {
            options.check_part_and_closure = true;
            continue;
        }
        if flags && arg == "--geometry-dependencies" {
            options.check_geometry_dependencies = true;
            continue;
        }
        if flags && arg == "--source-semantics" {
            options.check_geometry_dependencies = true;
            options.check_part_and_closure = true;
            continue;
        }
        if flags && arg == "--unit-context" {
            let value = args.next().ok_or(())?.into_string().map_err(|_| ())?;
            if value.is_empty()
                || value.starts_with('0')
                || !value.bytes().all(|c| c.is_ascii_digit())
            {
                return Err(());
            }
            options.physical_context = Some(value);
            continue;
        }
        if flags && arg.to_str().is_some_and(|s| s.starts_with('-')) {
            let value = number(&args.next().ok_or(())?)?;
            match arg.to_str().ok_or(())? {
                "--dependency-visits" => limits.dependency_visits = value,
                "--input-bytes" => source_limits.input_bytes = value,
                "--report-bytes" => source_limits.report_bytes = value,
                _ => return Err(()),
            }
        } else if path.replace(PathBuf::from(arg)).is_some() {
            return Err(());
        }
    }
    if !explicit {
        return Err(());
    }
    Ok(Some(Request {
        path: path.ok_or(())?,
        source_limits,
        source_options,
        limits,
        options,
    }))
}
fn run() -> u8 {
    let request = match arguments() {
        Ok(Some(r)) => r,
        Ok(None) => {
            println!("{USAGE}");
            return 0;
        }
        Err(()) => {
            eprintln!("{USAGE}");
            return 64;
        }
    };
    let mut bytes = Vec::new();
    if request.source_limits.valid()
        && File::open(request.path)
            .and_then(|f| {
                f.take(request.source_limits.input_bytes as u64 + 1)
                    .read_to_end(&mut bytes)
            })
            .is_err()
    {
        eprintln!("InputReadFailure");
        return 4;
    }
    let source = inspect_with_options(bytes, request.source_limits, request.source_options);
    let outcome = semantics::check(&source, request.limits, &request.options);
    if outcome.status == semantics::Status::InvalidRequest {
        eprintln!("InvalidPhysicalContextSelection");
        return 64;
    }
    let bytes = match semantics::compile_report(&source, &outcome) {
        Ok(b) => b,
        Err(semantics::ReportError::Output(report::Error::ReportByteLimit)) => {
            eprintln!("ReportByteLimit: no JSON published");
            return 5;
        }
        Err(_) => {
            eprintln!("ReportInfrastructureFailure: no JSON published");
            return 6;
        }
    };
    if std::io::stdout().lock().write_all(&bytes).is_err() {
        return 6;
    }
    match outcome.status {
        semantics::Status::Checked => 0,
        semantics::Status::Deferred => {
            if request.options.check_part_and_closure {
                7
            } else {
                0
            }
        }
        semantics::Status::SourceFailure if request.options.check_part_and_closure => {
            match source.status {
                solidcore_step::Status::Rejected => 1,
                solidcore_step::Status::Unsupported => 2,
                solidcore_step::Status::ResourceLimit => 3,
                solidcore_step::Status::Inspected => 6,
            }
        }
        semantics::Status::Rejected | semantics::Status::SourceFailure => 1,
        semantics::Status::Unsupported => 2,
        semantics::Status::ResourceLimit => 3,
        semantics::Status::InvalidRequest => 64,
        semantics::Status::InfrastructureFailure => 6,
    }
}
fn main() -> ExitCode {
    ExitCode::from(run())
}
