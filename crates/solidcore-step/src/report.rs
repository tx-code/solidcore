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

use crate::{
    CarrierInterpretation, Component, DataStringPolicy, Diagnostic, HeaderNamePolicy, Limits,
    Outcome, Record, Reference, Statistics, Status, StringHexInterpretation,
};
use serde::Serialize;
use std::fmt::{self, Write as _};
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
pub(crate) struct Report<'a> {
    schema: &'static str,
    status: Status,
    #[serde(skip_serializing_if = "Option::is_none")]
    header_name_policy: Option<HeaderNamePolicy>,
    #[serde(skip_serializing_if = "Option::is_none")]
    data_string_policy: Option<DataStringPolicy>,
    #[serde(skip_serializing_if = "Option::is_none")]
    carrier_interpretation: Option<CarrierView<'a>>,
    sha256: &'a Option<String>,
    captured_bytes: Option<usize>,
    raw: Option<ByteText<'a>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    raw_encoding: Option<&'static str>,
    scope: Scope,
    limits: Limits,
    statistics: Statistics,
    diagnostics: &'a [Diagnostic],
    header: Option<&'a [Component]>,
    records: Option<&'a [Record]>,
    references: Option<&'a [Reference]>,
}

#[derive(Serialize)]
struct CarrierView<'a> {
    requested: CarrierInterpretation,
    applied: bool,
    status: &'static str,
    interpretation: CarrierInterpretation,
    declared_level: Option<&'a str>,
    declaration_verified: bool,
    original_diagnostics: &'a [Diagnostic],
    #[serde(skip_serializing_if = "slice_empty")]
    string_hex_interpretations: &'a [StringHexInterpretation],
}
fn slice_empty<T>(items: &&[T]) -> bool {
    items.is_empty()
}

/// Byte projection, never a guessed character encoding. Every original byte maps
/// to U+0000..U+00FF, matching `STEP_SOURCE`'s raw-field contract after JSON decoding.
/// Bounded chunks avoid a second input-sized allocation before the report budget.
struct ByteText<'a>(&'a [u8]);
impl fmt::Display for ByteText<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut text = String::new();
        for chunk in self.0.chunks(1024) {
            text.clear();
            for byte in chunk {
                text.write_char(char::from(*byte))?;
            }
            f.write_str(&text)?;
        }
        Ok(())
    }
}
impl Serialize for ByteText<'_> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if self.0.is_ascii() {
            return serializer
                .serialize_str(std::str::from_utf8(self.0).map_err(serde::ser::Error::custom)?);
        }
        serializer.collect_str(self)
    }
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
    compile_value(&view(outcome), outcome.limits.report_bytes)
}

pub(crate) fn view(outcome: &Outcome) -> Report<'_> {
    let source = outcome.source.as_ref();
    let options = outcome.options();
    let receipt = outcome.carrier_assumption();
    let carrier_interpretation = (options.carrier_interpretation
        != CarrierInterpretation::DeclaredOnly
        || receipt.is_some())
    .then(|| CarrierView {
        requested: options.carrier_interpretation,
        applied: receipt.is_some(),
        status: if receipt.is_some() {
            "InterpretedUnverified"
        } else {
            "NotApplied"
        },
        interpretation: receipt.map_or(CarrierInterpretation::DeclaredOnly, |c| c.interpretation),
        declared_level: receipt.map(|c| c.declared_level.as_str()).or_else(|| {
            source
                .and_then(|s| s.header().first())
                .and_then(|h| h.parameters.get(1))
                .map(|v| v.text.as_str())
        }),
        declaration_verified: false,
        original_diagnostics: receipt.map_or(&[], |c| c.original_diagnostics.as_slice()),
        string_hex_interpretations: receipt
            .map_or(&[], |c| c.string_hex_interpretations.as_slice()),
    });
    Report {
        schema: SCHEMA,
        status: outcome.status,
        header_name_policy: (options.header_name_policy != HeaderNamePolicy::StrictAscii)
            .then_some(options.header_name_policy),
        data_string_policy: (options.data_string_policy != DataStringPolicy::StrictAscii)
            .then_some(options.data_string_policy),
        carrier_interpretation,
        sha256: &outcome.sha256,
        captured_bytes: outcome.captured.as_ref().map(|b| b.len()),
        raw: source.map(|s| ByteText(s.bytes())),
        raw_encoding: source
            .filter(|s| !s.bytes().is_ascii())
            .map(|_| "ByteValuesU0000To00FF"),
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
    }
}

pub(crate) fn compile_value(value: &impl Serialize, limit: usize) -> Result<Vec<u8>, Error> {
    let mut writer = Bounded {
        bytes: Vec::new(),
        limit,
        exceeded: false,
    };
    if serde_json::to_writer(&mut writer, value).is_err() {
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
