// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded, lossless STEP source inspection; never a native `BRep` acceptance claim.
//!
//! Upstream: `source/STEP_SOURCE/inc/StepSource.h`, `src/StepSource.cpp` in the
//! usd-brep STEP fork (source snapshot `d8ed45a`).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`inspect`] | `step_source::InspectBytes` |
//! | [`Snapshot`] | `step_source::SourceSnapshot` |
//!
//! Deviations: safe Rust ownership, checked spans, SHA-256 via sha2. This first
//! migration supports the strict ASCII, corrected `2;1` carrier only. Explicit
//! legacy/opaque interpretation options are not silently inherited. Input-driven
//! recursion is bounded at 128 (including reference collection and serialization).

#![forbid(unsafe_code)]

mod parser;
pub mod report;
pub mod semantics;

use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::sync::Arc;

/// Half-open byte range in the immutable input, zero based.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub struct Span {
    /// Inclusive start byte.
    pub begin: usize,
    /// Exclusive end byte.
    pub end: usize,
}

/// One-based line and byte column; CRLF is one newline.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub struct Location {
    /// One-based line.
    pub line: usize,
    /// One-based byte column.
    pub column: usize,
}

/// Explicit request limits. No production default is selected by this crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub struct Limits {
    /// Maximum complete input bytes.
    pub input_bytes: usize,
    /// Maximum DATA records.
    pub records: usize,
    /// Maximum values, including HEADER values and nested aggregates.
    pub values: usize,
    /// Maximum reference values.
    pub references: usize,
    /// Maximum value nesting depth.
    pub nesting: usize,
    /// Maximum original STRING token bytes, including delimiters.
    pub string_bytes: usize,
    /// Maximum original record bytes, including delimiters.
    pub record_bytes: usize,
    /// Maximum serialized report bytes, including its final newline.
    pub report_bytes: usize,
    /// Maximum decimal digits per instance ID, before canonicalization.
    pub id_digits: usize,
}

impl Limits {
    /// Test-only preset matching `Limits::PlanarFixtures`; not a quality policy.
    pub const fn planar_fixtures() -> Self {
        Self {
            input_bytes: 1024 * 1024,
            records: 10_000,
            values: 200_000,
            references: 100_000,
            nesting: 64,
            string_bytes: 32_769,
            record_bytes: 65_536,
            report_bytes: 16 * 1024 * 1024,
            id_digits: 128,
        }
    }

    /// Independent implementation ceilings. A caller may always choose less.
    pub const fn valid(self) -> bool {
        self.input_bytes <= 64 * 1024 * 1024
            && self.records <= 1_000_000
            && self.values <= 2_000_000
            && self.references <= 2_000_000
            && self.nesting <= 128
            && self.string_bytes <= 1024 * 1024
            && self.record_bytes <= 16 * 1024 * 1024
            && self.report_bytes <= 256 * 1024 * 1024
            && self.id_digits <= 1024
    }
}

/// Source grammar value kind. No EXPRESS type or unit is inferred here.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub enum ValueKind {
    /// Decimal integer lexeme, not narrowed to a machine integer.
    Integer,
    /// Decimal real lexeme, not rounded to floating point.
    Real,
    /// Decoded UTF-8 string with original source span retained.
    String,
    /// Enumeration without the full-stop delimiters.
    Enumeration,
    /// Canonical positive decimal instance ID.
    Reference,
    /// Explicit unset marker `$`.
    Unset,
    /// Derived marker `*`.
    Derived,
    /// Ordered aggregate, including an empty aggregate.
    Aggregate,
    /// Named typed parameter with exactly one child.
    Typed,
}

/// Source parameter, retaining its exact byte span and ordered children.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Value {
    /// Grammar kind.
    pub kind: ValueKind,
    /// Original span, including delimiters.
    pub span: Span,
    /// Decoded string, exact numeric lexeme, or canonical reference ID.
    pub text: String,
    /// Ordered aggregate or typed children.
    pub children: Vec<Value>,
}

/// Entity component, also used for the three required HEADER records.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Component {
    /// Uninterpreted uppercase entity/type keyword.
    pub name: String,
    /// Keyword through closing parenthesis.
    pub span: Span,
    /// Parameters in source order.
    pub parameters: Vec<Value>,
}

/// DATA instance. A complex instance keeps every constituent in source order.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Record {
    /// Canonical positive decimal ID, without `#` or leading zeros.
    pub id: String,
    /// Original ID token, including `#` and leading zeros.
    pub id_span: Span,
    /// Complete DATA record through its semicolon.
    pub span: Span,
    /// Whether external complex mapping was used.
    pub complex: bool,
    /// Ordered components; no schema-dependent flattening occurs.
    pub components: Vec<Component>,
}

/// Resolved DATA reference. Forward references and cycles are legal at this layer.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Reference {
    /// Canonical target ID.
    pub target_id: String,
    /// Original reference token span.
    pub span: Span,
    /// Zero-based owning record index.
    pub record_index: usize,
    /// Zero-based owning component index.
    pub component_index: usize,
    /// One-based nested parameter indices.
    pub parameter_path: Vec<usize>,
    /// Zero-based resolved record index.
    pub target_record_index: usize,
}

/// Deterministic source diagnostic, distinct from native or numerical failures.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Diagnostic {
    /// Stable diagnostic identity from the source contract.
    pub key: String,
    /// `D0` lexical/structural or `D1` reference-resolution stage.
    pub stage: String,
    /// Human-readable reason; not used as a stable discriminator.
    pub message: String,
    /// Primary original byte span.
    pub primary: Span,
    /// Other relevant original spans, such as the first duplicate occurrence.
    pub related: Vec<Span>,
    /// Owning canonical ID, if known.
    pub record_id: String,
    /// Owning component keyword, if known.
    pub component: String,
    /// One-based nested parameter path.
    pub parameter_path: Vec<usize>,
    /// Target ID for duplicate/unresolved reference diagnostics.
    pub target_id: String,
}

/// Source inspection status. `Inspected` does not mean imported geometry.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub enum Status {
    /// Complete source graph inspected; semantics and native build have not run.
    Inspected,
    /// Malformed source or an unresolved reference.
    Rejected,
    /// Explicitly outside this migration's carrier scope.
    Unsupported,
    /// A configured limit or implementation ceiling was exceeded.
    ResourceLimit,
}

/// Resource accounting in source traversal order.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub struct Statistics {
    /// Completed DATA records.
    pub records: usize,
    /// Visited HEADER and DATA values.
    pub values: usize,
    /// Visited reference values.
    pub references: usize,
    /// Maximum visited value depth.
    pub nesting: usize,
    /// Largest completed original STRING token.
    pub max_string_bytes: usize,
    /// Largest completed HEADER or DATA record.
    pub max_record_bytes: usize,
}

/// Owning immutable source snapshot; only a complete graph can be published.
#[derive(Clone, Debug)]
pub struct Snapshot {
    data: Arc<SourceData>,
}

#[derive(Debug)]
struct SourceData {
    bytes: Arc<[u8]>,
    header: Vec<Component>,
    records: Vec<Record>,
    references: Vec<Reference>,
    index: BTreeMap<String, usize>,
    incoming: BTreeMap<String, Vec<usize>>,
    lines: Vec<usize>,
}

impl Snapshot {
    /// Exact, immutable original bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.data.bytes
    }
    /// Required HEADER records, in source order.
    pub fn header(&self) -> &[Component] {
        &self.data.header
    }
    /// DATA records in source order, without schema-dependent reordering.
    pub fn records(&self) -> &[Record] {
        &self.data.records
    }
    /// Resolved references in source traversal order.
    pub fn references(&self) -> &[Reference] {
        &self.data.references
    }
    /// Look up a canonical ID without integer narrowing or sparse allocation.
    pub fn find(&self, canonical_id: &str) -> Option<&Record> {
        self.data
            .index
            .get(canonical_id)
            .and_then(|i| self.data.records.get(*i))
    }
    /// Indices into [`Self::references`] for incoming references to an ID.
    pub fn incoming(&self, canonical_id: &str) -> &[usize] {
        self.data
            .incoming
            .get(canonical_id)
            .map_or(&[], Vec::as_slice)
    }
    /// Checked access to original bytes. Reversed/out-of-range spans return `None`.
    pub fn raw(&self, span: Span) -> Option<&[u8]> {
        self.data.bytes.get(span.begin..span.end)
    }
    /// Locate a byte offset, including EOF. Invalid offsets return `None`.
    pub fn locate(&self, offset: usize) -> Option<Location> {
        if offset > self.data.bytes.len() {
            return None;
        }
        let line = self.data.lines.partition_point(|start| *start <= offset);
        self.data
            .lines
            .get(line.checked_sub(1)?)
            .map(|start| Location {
                line,
                column: offset - start + 1,
            })
    }
}

/// Inspection receipt. Rejection retains capture but never publishes a partial graph.
#[derive(Debug)]
pub struct Outcome {
    // Keep the Arc private: handing an owned Arc to callers would allow
    // Arc::get_mut/make_mut to change a capture independently of its receipt.
    captured: Option<Arc<[u8]>>,
    /// SHA-256 of the complete capture, absent without a capture.
    pub sha256: Option<String>,
    /// Published complete source graph, present only for `Inspected`.
    pub source: Option<Snapshot>,
    /// Final source status.
    pub status: Status,
    /// Counters, including the last attempted value for budget failures.
    pub statistics: Statistics,
    /// Explicit limits actually used.
    pub limits: Limits,
    /// Deterministic diagnostics; the first fatal source failure stops parsing.
    pub diagnostics: Vec<Diagnostic>,
}

impl Outcome {
    /// Exact complete capture, absent when capture limits were exceeded.
    ///
    /// Only a borrowed byte view is exposed, so even a rejected capture cannot
    /// be changed through `Arc::get_mut` or detached from its content identity.
    pub fn captured(&self) -> Option<&[u8]> {
        self.captured.as_deref()
    }
}

/// Inspect owned bytes without filesystem, kernel calls, repairs or hidden defaults.
pub fn inspect(bytes: Vec<u8>, limits: Limits) -> Outcome {
    let mut outcome = Outcome {
        captured: None,
        sha256: None,
        source: None,
        status: Status::ResourceLimit,
        statistics: Statistics::default(),
        limits,
        diagnostics: Vec::new(),
    };
    let capture_failure = if !limits.valid() {
        Some("InvalidLimits")
    } else if bytes.len() > limits.input_bytes {
        Some("InputByteLimit")
    } else {
        None
    };
    if let Some(key) = capture_failure {
        outcome.diagnostics.push(Diagnostic {
            key: key.into(),
            stage: "D0".into(),
            message: "Input capture or requested limits exceed the allowed budget".into(),
            primary: Span::default(),
            related: Vec::new(),
            record_id: String::new(),
            component: String::new(),
            parameter_path: Vec::new(),
            target_id: String::new(),
        });
        return outcome;
    }
    outcome.sha256 = Some(format!("{:x}", Sha256::digest(&bytes)));
    let bytes: Arc<[u8]> = bytes.into();
    outcome.captured = Some(Arc::clone(&bytes));
    let mut parser = parser::Parser::new(bytes, limits);
    let result = parser.parse();
    outcome.statistics = parser.statistics();
    match result {
        Ok(source) => {
            outcome.source = Some(source);
            outcome.status = Status::Inspected;
        }
        Err(failure) => {
            outcome.status = failure.status;
            outcome.diagnostics.push(failure.diagnostic);
        }
    }
    outcome
}
