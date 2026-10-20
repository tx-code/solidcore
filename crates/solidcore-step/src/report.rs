// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded, atomic source report compilation.
//!
//! Upstream: `source/STEP_SOURCE/src/StepInspectJson.cpp`, `StepSourceReport.cpp`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`compile`] | `step_source::InspectJson` |
//!
//! Deviations: a new versioned Rust report schema; serde emits borrowed source
//! values directly. Complete bytes are compiled before external publication.

use crate::{Component, Diagnostic, Limits, Outcome, Record, Reference, Statistics, Status};
use serde::Serialize;
use std::io::{self, Write};

/// Versioned Rust report contract, independent of the C++ JSON byte format.
pub const SCHEMA: &str = "solidcore.step.source.v1";

#[derive(Serialize)]
struct Scope {
    source: &'static str,
    schema_semantics: &'static str,
    units: &'static str,
    geometry: &'static str,
    native_build: &'static str,
    repair: &'static str,
}

#[derive(Serialize)]
struct Report<'a> {
    schema: &'static str,
    status: Status,
    sha256: &'a Option<String>,
    captured_bytes: Option<usize>,
    raw: Option<&'a str>,
    scope: Scope,
    limits: Limits,
    statistics: Statistics,
    diagnostics: &'a [Diagnostic],
    header: Option<&'a [Component]>,
    records: Option<&'a [Record]>,
    references: Option<&'a [Reference]>,
}

struct Bounded {
    bytes: Vec<u8>,
    limit: usize,
    exceeded: bool,
}
impl Write for Bounded {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if bytes.len() > self.limit.saturating_sub(self.bytes.len()) {
            self.exceeded = true;
            return Err(io::Error::other("ReportByteLimit"));
        }
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// A complete report could not be compiled; callers must publish no partial JSON.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Actual JSON bytes (including escapes and final LF) exceeded the budget.
    ReportByteLimit,
    /// Internal serialization failure, distinct from source rejection.
    Serialization,
}

/// Compile a deterministic complete report, including its final LF, within budget.
///
/// Reports contain source strings and parameters and may contain confidential
/// information. Choosing where to publish is the caller's responsibility.
pub fn compile(outcome: &Outcome) -> Result<Vec<u8>, Error> {
    let source = outcome.source.as_ref();
    let report = Report {
        schema: SCHEMA,
        status: outcome.status,
        sha256: &outcome.sha256,
        captured_bytes: outcome.captured.as_ref().map(|b| b.len()),
        raw: source.and_then(|s| std::str::from_utf8(s.bytes()).ok()),
        scope: Scope {
            source: if source.is_some() {
                "Inspected"
            } else {
                "NotEstablished"
            },
            schema_semantics: "NotRun",
            units: "NotRun",
            geometry: "NotRun",
            native_build: "NotRun",
            repair: "None",
        },
        limits: outcome.limits,
        statistics: outcome.statistics,
        diagnostics: &outcome.diagnostics,
        header: source.map(crate::Snapshot::header),
        records: source.map(crate::Snapshot::records),
        references: source.map(crate::Snapshot::references),
    };
    let mut writer = Bounded {
        bytes: Vec::new(),
        limit: outcome.limits.report_bytes,
        exceeded: false,
    };
    if serde_json::to_writer(&mut writer, &report).is_err() {
        return Err(if writer.exceeded {
            Error::ReportByteLimit
        } else {
            Error::Serialization
        });
    }
    writer
        .write_all(b"\n")
        .map_err(|_| Error::ReportByteLimit)?;
    Ok(writer.bytes)
}
