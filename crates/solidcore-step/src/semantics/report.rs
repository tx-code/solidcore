// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Borrowed, bounded source checkpoint reports with immutable source binding.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemanticJson.cpp` (typed observations).
//! Deviations: explicit checkpoint scope; no geometry/native claims. Reuses the
//! source writer budget and traverses records without a second report object graph.
use super::{
    Attribute, Diagnostic, GeometryChecks, Limits, Outcome, ReportError, SchemaSelection, Snapshot,
    Status, UnitChecks, Units,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    used_by_representation: Option<bool>,
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
                used_by_representation: self.0.used_by_representation(r),
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
    #[serde(skip_serializing_if = "Option::is_none")]
    part_definition: Option<&'a super::Part>,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    property_attachment_checks: Option<usize>,
    unit_dependency_checks: UnitChecks,
    #[serde(skip_serializing_if = "Option::is_none")]
    geometry_dependency_checks: Option<GeometryChecks>,
    #[serde(skip_serializing_if = "Option::is_none")]
    geometry_consistency: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    planar_rules: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    full_import: Option<&'static str>,
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
        schema: if outcome.check_geometry_dependencies && outcome.check_part_and_closure {
            "solidcore.step.source-semantics.v1"
        } else if outcome.check_geometry_dependencies {
            "solidcore.step.geometry-dependencies.v1"
        } else if outcome.check_part_and_closure {
            "solidcore.step.part-closure.v1"
        } else {
            "solidcore.step.types-units.v1"
        },
        scope: if outcome.check_geometry_dependencies && outcome.check_part_and_closure {
            "StrictSourceSemantics"
        } else if outcome.check_geometry_dependencies {
            "GeometryDependenciesCheckpoint"
        } else if outcome.check_part_and_closure {
            "PartAndClosureCheckpoint"
        } else {
            "TypesAndUnitsCheckpoint"
        },
        status: outcome.status,
        schema_selection: &outcome.schema_selection,
        binding: outcome.binding,
        types: outcome.types,
        unit_dependencies: outcome.unit_dependencies,
        si_units: outcome.si_units,
        part: outcome.part,
        geometry_dependencies: outcome.geometry_dependencies,
        native_build: "NotRun",
        interoperability: "NotRun",
        repair: "None",
        dependency_visits: outcome.dependency_visits,
        property_attachment_checks: outcome
            .check_part_and_closure
            .then_some(outcome.property_attachment_checks),
        unit_dependency_checks: outcome.unit_dependency_checks,
        geometry_dependency_checks: outcome
            .check_geometry_dependencies
            .then_some(outcome.geometry_dependency_checks),
        geometry_consistency: outcome.check_geometry_dependencies.then_some("NotRun"),
        planar_rules: outcome.check_geometry_dependencies.then_some("NotRun"),
        full_import: outcome.check_geometry_dependencies.then_some("NotRun"),
        limits: outcome.limits,
        diagnostics: &outcome.diagnostics,
        snapshot: outcome.snapshot.as_ref().map(|s| SnapshotView {
            registered_types: Names(s),
            records: Records(s),
            units: s.units(),
            part_definition: s.part_definition(),
        }),
        source: crate::report::view(source),
    };
    crate::report::compile_value(&report, source.limits.report_bytes).map_err(ReportError::Output)
}
