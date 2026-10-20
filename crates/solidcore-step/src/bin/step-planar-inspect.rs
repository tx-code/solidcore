// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit planar source CLI, without native model construction or repair.
//! Upstream: `source/STEP_PLANAR/app/step_planar.cpp` at `d8ed45a`.
//! Deviations: host I/O outside the library, versioned Rust JSON and explicit
//! request/infrastructure statuses; original source/semantic/planar budgets apply.
#![forbid(unsafe_code)]
use solidcore_step::{inspect, planar, semantics, Limits};
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;
const USAGE:&str="step-planar-inspect --fixture-limits --quality Q-P0-mm-01 [--planar-work N] [--dependency-visits N] [source budget overrides] FILE\nSource rules and directed regions only. Native build, full import, repair, target UV, material volume and shell self-intersection are NOT RUN. The frozen quality is a synthetic candidate, not a production default. A custom --quality NAME requires --consistency-mm N/D (or an exact decimal); no quality is inferred.\nSource overrides: --input-bytes, --records, --values, --references, --nesting, --string-bytes, --record-bytes, --report-bytes, --id-digits.";
struct Request {
    path: PathBuf,
    source: Limits,
    semantic: semantics::Limits,
    planar: planar::Limits,
    quality: planar::Quality,
}
fn arguments() -> Result<Option<Request>, ()> {
    let mut args = std::env::args_os().skip(1);
    let mut path = None;
    let mut source = Limits::planar_fixtures();
    let mut semantic = semantics::Limits::fixtures();
    let mut planar = planar::Limits::fixtures();
    let mut explicit = false;
    let mut quality = None;
    let mut consistency = None;
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
        if flags && arg == "--quality" {
            quality = Some(
                args.next()
                    .and_then(|v| v.into_string().ok())
                    .filter(|v| !v.is_empty())
                    .ok_or(())?,
            );
            continue;
        }
        if flags && arg == "--consistency-mm" {
            let value = args.next().and_then(|v| v.into_string().ok()).ok_or(())?;
            consistency = Some(if let Some((n, d)) = value.split_once('/') {
                planar::Rational::new(n.parse().map_err(|_| ())?, d.parse().map_err(|_| ())?)
                    .map_err(|_| ())?
            } else {
                planar::Rational::decimal(&value).map_err(|_| ())?
            });
            continue;
        }
        if flags && arg.to_str().is_some_and(|s| s.starts_with('-')) {
            let name = arg.to_str().ok_or(())?;
            let raw = args.next().ok_or(())?;
            let text = raw
                .to_str()
                .filter(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))
                .ok_or(())?;
            let value = text.parse::<usize>().map_err(|_| ())?;
            match name {
                "--input-bytes" => source.input_bytes = value,
                "--records" => source.records = value,
                "--values" => source.values = value,
                "--references" => source.references = value,
                "--nesting" => source.nesting = value,
                "--string-bytes" => source.string_bytes = value,
                "--record-bytes" => source.record_bytes = value,
                "--report-bytes" => source.report_bytes = value,
                "--id-digits" => source.id_digits = value,
                "--dependency-visits" => semantic.dependency_visits = value,
                "--planar-work" => planar.work = value,
                _ => return Err(()),
            }
        } else if path.replace(PathBuf::from(arg)).is_some() {
            return Err(());
        }
    }
    if !explicit || planar.work > 10_000_000 {
        return Err(());
    }
    let id = quality.ok_or(())?;
    let frozen = planar::Quality::planar_fixture();
    let consistency_mm = consistency
        .or_else(|| (id == frozen.id).then_some(frozen.consistency_mm))
        .ok_or(())?;
    if consistency_mm <= planar::Rational::ZERO
        || (id == frozen.id && consistency_mm != frozen.consistency_mm)
    {
        return Err(());
    }
    Ok(Some(Request {
        path: path.ok_or(())?,
        source,
        semantic,
        planar,
        quality: planar::Quality { id, consistency_mm },
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
    if request.source.valid()
        && File::open(&request.path)
            .and_then(|f| {
                f.take(request.source.input_bytes as u64 + 1)
                    .read_to_end(&mut bytes)
            })
            .is_err()
    {
        eprintln!("InputReadFailure");
        return 4;
    }
    let source = inspect(bytes, request.source);
    let sem = semantics::check(
        &source,
        request.semantic,
        &semantics::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    let result = planar::check(&sem, request.quality, request.planar);
    if result.status == planar::Status::InvalidRequest {
        eprintln!("InvalidPlanarConfigurationOrHandoff");
        return 64;
    }
    let output = match planar::compile_report(&source, &sem, &result) {
        Ok(bytes) => bytes,
        Err(planar::ReportError::Output(solidcore_step::report::Error::ReportByteLimit)) => {
            eprintln!("ReportByteLimit: no JSON published");
            return 5;
        }
        Err(_) => {
            eprintln!("ReportInfrastructureFailure: no JSON published");
            return 6;
        }
    };
    if std::io::stdout().lock().write_all(&output).is_err() {
        return 6;
    }
    match result.status {
        planar::Status::Checked => 0,
        planar::Status::Rejected => 1,
        planar::Status::Unsupported => 2,
        planar::Status::ResourceLimit => 3,
        planar::Status::Deferred => 7,
        planar::Status::Indeterminate => 8,
        planar::Status::InvalidRequest => 64,
        planar::Status::InfrastructureFailure => 6,
        planar::Status::UpstreamFailure => {
            if sem.status == semantics::Status::Unsupported
                || source.status == solidcore_step::Status::Unsupported
            {
                2
            } else if sem.status == semantics::Status::ResourceLimit
                || source.status == solidcore_step::Status::ResourceLimit
            {
                3
            } else {
                1
            }
        }
    }
}
fn main() -> ExitCode {
    ExitCode::from(run())
}
