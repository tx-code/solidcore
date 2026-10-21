// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Opt-in source checkpoint and interoperability CLI; host I/O stays outside the library.
//! Upstream: `source/STEP_SEMANTICS/app/step_semantics.cpp` (source capture and checks).
//! Deviations: selected source phases and explicit interoperability remain separate
//! from planar/global qualification and native construction.
#![forbid(unsafe_code)]
use solidcore_step::{
    inspect_with_options, report, semantics, DataStringPolicy, HeaderNamePolicy, InspectOptions,
    Limits,
};
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "step-semantic-inspect --fixture-limits [--general-brep-source] [--development-ap214-angle-dimensions] [--source-semantics | --geometry-dependencies | --part-and-closure] [--interop-nongeometric] [--interpret-unregistered-ap214-2010 | --interpret-ap214-name-only-as-2010] [--interpret-level1-as-corrected | --interpret-level1-with-lowercase-hex] [--preserve-opaque-header-name] [--preserve-opaque-data-strings] [--unit-context ID] [--dependency-visits N] [--preserved-diagnostics N] [source budget overrides] FILE\n--interop-nongeometric runs the complete source pipeline and final exclusion proof. Schema/carrier interpretations require explicit general interoperability. Development angle-dimensions policy is independent, requires general scope and full source checks, retains the source syntax defect and never establishes formal AP validity.\nSource overrides: --input-bytes, --records, --values, --references, --nesting, --string-bytes, --record-bytes, --report-bytes, --id-digits.\nPlanar/global qualification, continuous geometry consistency, native build and repair are NOT RUN.";

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
        if flags && arg == "--development-ap214-angle-dimensions" {
            options.development_unit_policy =
                semantics::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions;
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
        if flags && arg == "--interop-nongeometric" {
            options.source_policy = semantics::SourcePolicy::PreserveNonGeometric;
            continue;
        }
        if flags
            && matches!(
                arg.to_str(),
                Some("--interpret-unregistered-ap214-2010" | "--interpret-ap214-name-only-as-2010")
            )
        {
            let interpretation = if arg == "--interpret-unregistered-ap214-2010" {
                semantics::SchemaInterpretation::UnregisteredAp214As2010
            } else {
                semantics::SchemaInterpretation::Ap214NameOnlyAs2010
            };
            if options.schema_interpretation != semantics::SchemaInterpretation::DeclaredOnly
                && options.schema_interpretation != interpretation
            {
                return Err(());
            }
            options.schema_interpretation = interpretation;
            continue;
        }
        if flags
            && matches!(
                arg.to_str(),
                Some("--interpret-level1-as-corrected" | "--interpret-level1-with-lowercase-hex")
            )
        {
            let interpretation = if arg == "--interpret-level1-as-corrected" {
                solidcore_step::CarrierInterpretation::Level1AsCorrectedEncoding
            } else {
                solidcore_step::CarrierInterpretation::Level1AsCorrectedEncodingWithLowercaseHex
            };
            if source_options.carrier_interpretation
                != solidcore_step::CarrierInterpretation::DeclaredOnly
                && source_options.carrier_interpretation != interpretation
            {
                return Err(());
            }
            source_options.carrier_interpretation = interpretation;
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
                "--preserved-diagnostics" => limits.preserved_diagnostics = value,
                "--input-bytes" => source_limits.input_bytes = value,
                "--report-bytes" => source_limits.report_bytes = value,
                "--records" => source_limits.records = value,
                "--values" => source_limits.values = value,
                "--references" => source_limits.references = value,
                "--nesting" => source_limits.nesting = value,
                "--string-bytes" => source_limits.string_bytes = value,
                "--record-bytes" => source_limits.record_bytes = value,
                "--id-digits" => source_limits.id_digits = value,
                _ => return Err(()),
            }
        } else if path.replace(PathBuf::from(arg)).is_some() {
            return Err(());
        }
    }
    if !explicit {
        return Err(());
    }
    if options.development_unit_policy != semantics::DevelopmentUnitPolicy::Disabled
        && options.geometry_scope != semantics::GeometryScope::GeneralBrep
    {
        return Err(());
    }
    // Preserve Check's invalid-request precedence even if capture would fail.
    // The library repeats this guard for direct callers and immutable receipts.
    let assumption = options.schema_interpretation != semantics::SchemaInterpretation::DeclaredOnly
        || source_options.carrier_interpretation
            != solidcore_step::CarrierInterpretation::DeclaredOnly;
    if assumption
        && !(options.source_policy == semantics::SourcePolicy::PreserveNonGeometric
            && options.geometry_scope == semantics::GeometryScope::GeneralBrep)
    {
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
        semantics::Status::Checked | semantics::Status::CheckedWithDiagnostics => 0,
        semantics::Status::Deferred => {
            if request.options.check_part_and_closure
                || request.options.source_policy == semantics::SourcePolicy::PreserveNonGeometric
            {
                7
            } else {
                0
            }
        }
        semantics::Status::SourceFailure
            if request.options.check_part_and_closure
                || request.options.source_policy
                    == semantics::SourcePolicy::PreserveNonGeometric =>
        {
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
