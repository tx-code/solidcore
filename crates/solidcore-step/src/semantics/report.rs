// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Borrowed, bounded types/unit checkpoint reports with immutable source binding.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemanticJson.cpp` (typed observations).
//! Deviations: independent checkpoint scope; no part/geometry claims. Reuses the
//! source writer budget and traverses records without a second report object graph.
use super::{
    Attribute, Diagnostic, Limits, Outcome, ReportError, SchemaSelection, Snapshot, Status,
    UnitChecks, Units,
};
use crate::Outcome as RawOutcome;
use serde::ser::SerializeSeq;
use serde::{Serialize, Serializer};
use std::sync::Arc;

struct Names<'a>(&'a Snapshot);
impl Serialize for Names<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(None)?;
        for name in self.0.registered_types() {
            seq.serialize_element(name)?;
        }
        seq.end()
    }
}
#[derive(Serialize)]
struct RecordView<'a> {
    id: &'a str,
    types: Option<Vec<&'static str>>,
    attributes: Option<&'a [Attribute]>,
}
struct Records<'a>(&'a Snapshot);
impl Serialize for Records<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(Some(self.0.source().records().len()))?;
        for (r, record) in self.0.source().records().iter().enumerate() {
            seq.serialize_element(&RecordView {
                id: &record.id,
                types: self.0.types_of(r),
                attributes: self.0.attributes(r),
            })?;
        }
        seq.end()
    }
}
#[derive(Serialize)]
struct SnapshotView<'a> {
    registered_types: Names<'a>,
    records: Records<'a>,
    units: &'a Units,
}
#[derive(Serialize)]
struct Report<'a> {
    schema: &'static str,
    scope: &'static str,
    status: Status,
    schema_selection: &'a Option<SchemaSelection>,
    binding: &'a str,
    types: &'a str,
    unit_dependencies: &'a str,
    si_units: &'a str,
    part: &'static str,
    geometry_dependencies: &'static str,
    native_build: &'static str,
    interoperability: &'static str,
    repair: &'static str,
    dependency_visits: usize,
    unit_dependency_checks: UnitChecks,
    limits: Limits,
    diagnostics: &'a [Diagnostic],
    snapshot: Option<SnapshotView<'a>>,
    source: crate::report::Report<'a>,
}
pub(super) fn compile(source: &RawOutcome, outcome: &Outcome) -> Result<Vec<u8>, ReportError> {
    let bound = match (&source.captured, &outcome.source_capture) {
        (Some(a), Some(b)) => Arc::ptr_eq(a, b),
        (None, None) => true,
        _ => false,
    };
    if !bound {
        return Err(ReportError::SourceMismatch);
    }
    let report = Report {
        schema: "solidcore.step.types-units.v1",
        scope: "TypesAndUnitsCheckpoint",
        status: outcome.status,
        schema_selection: &outcome.schema_selection,
        binding: outcome.binding,
        types: outcome.types,
        unit_dependencies: outcome.unit_dependencies,
        si_units: outcome.si_units,
        part: "NotRun",
        geometry_dependencies: "NotRun",
        native_build: "NotRun",
        interoperability: "NotRun",
        repair: "None",
        dependency_visits: outcome.dependency_visits,
        unit_dependency_checks: outcome.unit_dependency_checks,
        limits: outcome.limits,
        diagnostics: &outcome.diagnostics,
        snapshot: outcome.snapshot.as_ref().map(|s| SnapshotView {
            registered_types: Names(s),
            records: Records(s),
            units: s.units(),
        }),
        source: crate::report::view(source),
    };
    crate::report::compile_value(&report, source.limits.report_bytes).map_err(ReportError::Output)
}
