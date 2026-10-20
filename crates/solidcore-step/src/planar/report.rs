// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Borrowed source-bound planar diagnostics, compiled atomically under one budget.
//! Upstream: `StepPlanarJson.cpp` at `d8ed45a`.
//! Deviations: versioned Rust JSON uses snapshot-local record indices, with the
//! original semantic/source report embedded exactly once; no second report graph.
use super::{Limits, Outcome, Quality, ReportError, Status};
use crate::{semantics, Outcome as RawOutcome};
use serde::Serialize;
use std::sync::Arc;
#[derive(Serialize)]
struct Checks {
    local_rules: &'static str,
    global_rules_47: &'static str,
    planar_regions: &'static str,
    conversion_units: &'static str,
    native_build: &'static str,
    full_import: &'static str,
    target_uv: &'static str,
    tolerance_setting: &'static str,
    shell_self_intersection: &'static str,
    material_volume: &'static str,
}
#[derive(Serialize)]
struct QualityView<'a> {
    #[serde(flatten)]
    quality: &'a Quality,
    production_default: bool,
    repair: &'static str,
}
#[derive(Serialize)]
struct Report<'a> {
    schema: &'static str,
    scope: &'static str,
    status: Status,
    source_semantics: semantics::report::Report<'a>,
    quality: QualityView<'a>,
    limits: Limits,
    work: usize,
    checks: Checks,
    model: Option<()>,
    qualified_source_evidence: bool,
    arithmetic: &'static str,
    source_index_basis: &'static str,
    rules: &'a [super::RuleEvidence],
    metrics: &'a [super::Metric],
    edges: &'a [super::EdgeEvidence],
    faces: &'a [super::FaceEvidence],
    unit_conversions: &'a [super::UnitConversion],
    diagnostics: &'a [semantics::Diagnostic],
}
pub(super) fn compile(
    raw: &RawOutcome,
    sem: &semantics::Outcome,
    planar: &Outcome,
) -> Result<Vec<u8>, ReportError> {
    let bound = match (&raw.captured, &planar.source_capture) {
        (Some(a), Some(b)) => Arc::ptr_eq(a, b),
        (None, None) => true,
        _ => false,
    };
    if !bound {
        return Err(ReportError::SourceMismatch);
    }
    if let Some(source) = &planar.evidence.source {
        if !raw
            .captured
            .as_ref()
            .is_some_and(|bytes| Arc::ptr_eq(bytes, &source.source().data.bytes))
        {
            return Err(ReportError::SourceMismatch);
        }
    }
    let preceding = semantics::report::view(raw, sem).map_err(|_| ReportError::SourceMismatch)?;
    let report=Report{schema:"solidcore.step.planar.v1",scope:"PlanarSourceRulesAndRegions",status:planar.status,source_semantics:preceding,
        quality:QualityView{quality:&planar.quality,production_default:false,repair:"None"},limits:planar.limits,work:planar.work,
        checks:Checks{local_rules:planar.local_rules,global_rules_47:planar.global_rules,planar_regions:planar.geometry,conversion_units:planar.conversion_units,
            native_build:"NotRun",full_import:"NotRun",target_uv:"NotRun",tolerance_setting:"NotRun",shell_self_intersection:"NotRun",material_volume:"NotRun"},
        model:None,qualified_source_evidence:planar.status==Status::Checked,
        arithmetic:"Reduced signed-63-bit rational; all decisions exact, capacity exhaustion Indeterminate; radicals retained algebraically",
        source_index_basis:"source_semantics.source.records; original indices, never portable IDs",
        rules:&planar.evidence.rules,metrics:&planar.evidence.metrics,edges:&planar.evidence.edges,faces:&planar.evidence.faces,unit_conversions:&planar.evidence.conversions,diagnostics:&planar.diagnostics};
    crate::report::compile_value(&report, raw.limits.report_bytes).map_err(ReportError::Output)
}
