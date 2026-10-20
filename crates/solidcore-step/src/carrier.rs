// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit source interpretation policies and immutable qualification receipts.
//!
//! Upstream: `source/STEP_SOURCE/inc/StepSource.h`, `src/StepSource.cpp` at `d8ed45a`.
//! `InspectOptions` groups the three C++ arguments; `CarrierAssumption` and
//! `StringHexInterpretation` preserve their original diagnostic and token evidence.
//! Deviations: safe enum options, private receipt owners; opaque JSON text is null.

use crate::{Diagnostic, Outcome, Span, Status, Value, ValueKind};
use serde::{ser::SerializeStruct, Serialize, Serializer};

/// Source policy for exactly `HEADER.FILE_NAME.name`; no code page is inferred.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub enum HeaderNamePolicy {
    /// Keep the ordinary ASCII carrier and Part 21 decoder.
    #[default]
    StrictAscii,
    /// Preserve high bytes in this token with unestablished text and encoding.
    PreserveOpaqueBytes,
}

/// Source policy for DATA STRING tokens; does not approve any typed field.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub enum DataStringPolicy {
    /// Keep the ordinary ASCII carrier and Part 21 decoder.
    #[default]
    StrictAscii,
    /// Preserve non-ASCII DATA STRING tokens as uninterpreted original bytes.
    PreserveOpaqueBytes,
}

/// Explicit unverified alternative to the declared carrier interpretation.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub enum CarrierInterpretation {
    /// Only corrected implementation declaration `2;1` is supported.
    #[default]
    DeclaredOnly,
    /// Read declaration `1` with the existing corrected grammar, without rewriting.
    Level1AsCorrectedEncoding,
    /// Additionally interpret lowercase DATA X/X2/X4 hex digits after applying `1`.
    Level1AsCorrectedEncodingWithLowercaseHex,
}

/// Explicit independent source options; defaults preserve strict behavior.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub struct InspectOptions {
    /// Limited HEADER name byte preservation.
    pub header_name_policy: HeaderNamePolicy,
    /// DATA byte preservation, without field-specific semantic acceptance.
    pub data_string_policy: DataStringPolicy,
    /// Requested interpretation, which can be enabled but remain unapplied.
    pub carrier_interpretation: CarrierInterpretation,
}

/// Exact source evidence for a token that required lowercase-hex interpretation.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct StringHexInterpretation {
    /// Complete quoted token, or the consumed prefix on failure.
    pub token_span: Span,
    /// Original strict `InvalidStringEncoding` range at the first lowercase digit.
    pub first_strict_span: Span,
    /// Number of interpreted lowercase digits, not inferred Unicode characters.
    pub lowercase_digits: usize,
    /// True only after the closing quote is consumed successfully.
    pub complete: bool,
}

/// Immutable evidence of an applied assumption; never a verified declaration.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct CarrierAssumption {
    /// Alternative explicitly chosen by the caller.
    pub interpretation: CarrierInterpretation,
    /// Original declaration text; never normalized to `2;1`.
    pub declared_level: String,
    /// Original declaration STRING token.
    pub declared_span: Span,
    /// The strict-path diagnostic which this assumption bypassed.
    pub original_diagnostics: Vec<Diagnostic>,
    /// Per-token hex evidence, including consumed prefixes on later failure.
    pub string_hex_interpretations: Vec<StringHexInterpretation>,
}

pub(super) fn serialize_value<S: Serializer>(
    value: &Value,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    // Preserve strict report layout and bytes. Opaque storage must never serialize
    // as an invented empty label; kind plus null explicitly withholds text.
    let mut out = serializer.serialize_struct("Value", 4)?;
    out.serialize_field("kind", &value.kind)?;
    out.serialize_field("span", &value.span)?;
    out.serialize_field("text", &value.interpreted_text())?;
    out.serialize_field("children", &value.children)?;
    out.end()
}

pub(super) fn notice(
    key: &str,
    message: &str,
    primary: Span,
    component: &str,
    path: &[usize],
) -> Diagnostic {
    Diagnostic {
        key: key.into(),
        stage: "D0".into(),
        message: message.into(),
        primary,
        related: Vec::new(),
        record_id: String::new(),
        component: component.into(),
        parameter_path: path.to_vec(),
        target_id: String::new(),
    }
}

pub(super) fn append_notices(outcome: &mut Outcome) {
    if outcome.status != Status::Inspected {
        return;
    }
    if let Some(span) = outcome
        .carrier_assumption()
        .map(|receipt| receipt.declared_span)
    {
        outcome.diagnostics.push(notice(
            "UnverifiedCarrierInterpretation",
            "Declared level1 interpreted using corrected grammar; original encoding conformance is unverified",
            span, "FILE_DESCRIPTION", &[2],
        ));
    }
    if let Some(span) = outcome
        .source
        .as_ref()
        .and_then(|source| source.header().get(1))
        .and_then(|header| header.parameters.first())
        .filter(|name| name.kind == ValueKind::OpaqueString)
        .map(|name| name.span)
    {
        outcome.diagnostics.push(notice(
            "OpaqueHeaderFileName",
            "Nonstandard FILE_NAME.name retained as original bytes; text and encoding are unestablished",
            span, "FILE_NAME", &[1],
        ));
    }
}
