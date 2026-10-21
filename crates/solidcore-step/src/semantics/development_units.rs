// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit development-unit proof ownership, separate from nongeometric approval.
//! The fixed AP214 catalog still requires `NAMED_UNIT.dimensions='*'`. This policy
//! retains that source defect; it never claims a legal AP encoding or repairs bytes.
//! Dimensional/type/scale interpretation stays in the existing unit dependency pass.

use super::checker::{internal, Checked, Checker, Failure};
use super::{Attribute, Diagnostic, GeometryScope, Status};
use crate::{Value, ValueKind};
use serde::Serialize;
use std::sync::Arc;

/// Independently selected development compatibility; never an AP source policy.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub enum DevelopmentUnitPolicy {
    /// Strict marker/type rules remain unchanged.
    #[default]
    Disabled,
    /// Prove the exact zero dimensions and finite positive angle-unit dependency
    /// closure of AP214's explicit-reference derived-slot defect.
    Ap214ExplicitPlaneAngleDimensions,
}

/// One original angle-unit definition in the proved dependency closure.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DevelopmentAngleUnit {
    pub(super) record_index: usize,
    pub(super) dimensions_record: Option<usize>,
    pub(super) conversion_factor_record: Option<usize>,
    pub(super) dependency_unit: Option<usize>,
    pub(super) factor_measure: Option<String>,
    pub(super) factor_lexeme: Option<String>,
    pub(super) si_prefix: Option<String>,
    pub(super) scale_to_radians: f64,
}
impl DevelopmentAngleUnit {
    /// Original source index; it is meaningful only with the owning snapshot.
    pub fn record_index(&self) -> usize {
        self.record_index
    }
    /// Original conversion-factor token; no degree or pi value is inferred.
    pub fn factor_lexeme(&self) -> Option<&str> {
        self.factor_lexeme.as_deref()
    }
    /// Positive finite binary64 conversion observation, not a continuous bound.
    pub fn scale_to_radians(&self) -> f64 {
        self.scale_to_radians
    }
}

/// Immutable successful development proof attached only to its source snapshot.
#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DevelopmentUnitReceipt {
    policy: DevelopmentUnitPolicy,
    source_syntax_verified: bool,
    #[serde(rename = "formalAPValidity")]
    formal_ap_validity: &'static str,
    qualification: &'static str,
    affected_units: Vec<usize>,
    original_diagnostics: Vec<Diagnostic>,
    unit_definitions: Vec<DevelopmentAngleUnit>,
}
impl DevelopmentUnitReceipt {
    /// Exactly the units whose original derived-slot errors were retained.
    pub fn affected_units(&self) -> &[usize] {
        &self.affected_units
    }
    /// The original errors, with their original source spans and classification.
    pub fn original_diagnostics(&self) -> &[Diagnostic] {
        &self.original_diagnostics
    }
    /// All proved original definitions, in deterministic source order.
    pub fn unit_definitions(&self) -> &[DevelopmentAngleUnit] {
        &self.unit_definitions
    }
    /// A source-owned proved conversion observation for a unit in this closure.
    pub fn scale_to_radians(&self, record: usize) -> Option<f64> {
        self.unit_definitions
            .iter()
            .find(|u| u.record_index == record)
            .map(|u| u.scale_to_radians)
    }
    /// This policy retains a source marker defect and never verifies its syntax.
    pub fn source_syntax_verified(&self) -> bool {
        self.source_syntax_verified
    }
}

#[derive(Clone, Default)]
pub(super) struct Qualification {
    pub requested: DevelopmentUnitPolicy,
    pub original: Vec<Diagnostic>,
    pub receipt: Option<Arc<DevelopmentUnitReceipt>>,
}
impl Qualification {
    pub fn diagnostics(&self) -> &[Diagnostic] {
        self.receipt
            .as_deref()
            .map_or(&self.original, DevelopmentUnitReceipt::original_diagnostics)
    }
}

#[derive(Default)]
pub(super) struct Working {
    pub candidates: Vec<usize>,
    pub definitions: Vec<DevelopmentAngleUnit>,
    pub units_checked: bool,
}

impl Checker<'_> {
    pub fn development_dimensions_candidate(&self, r: usize, a: &Attribute, v: &Value) -> bool {
        if self.outcome.development_units.requested == DevelopmentUnitPolicy::Disabled
            || self.scope != GeometryScope::GeneralBrep
            || !self.catalog.is_some_and(super::catalog::Catalog::ap214)
            || a.name != "dimensions"
            || a.declared_in != "NAMED_UNIT"
            || v.kind != ValueKind::Reference
        {
            return false;
        }
        // Exact external-mapping facets, not names or a dimensionless substitute.
        self.source.records().get(r).is_some_and(|record| {
            record.complex
                && record.components.len() == 3
                && record.components.iter().map(|c| c.name.as_str()).eq([
                    "CONVERSION_BASED_UNIT",
                    "NAMED_UNIT",
                    "PLANE_ANGLE_UNIT",
                ])
        })
    }
    pub fn defer_development_dimensions(&mut self, r: usize, failure: Failure) -> Checked<()> {
        let span = failure.diagnostic.source.primary;
        self.development_units.candidates.push(r);
        self.outcome
            .development_units
            .original
            .push(failure.diagnostic);
        let count = self
            .outcome
            .development_units
            .original
            .len()
            .saturating_add(self.pending.len())
            .saturating_add(self.outcome.qualification.interpretation.len());
        if count > self.outcome.limits.preserved_diagnostics {
            return Err(self.failure(
                Status::ResourceLimit,
                "DevelopmentUnitDiagnosticLimit",
                "D2",
                "Development source diagnostic capacity exceeded",
                span,
                Some(r),
                0,
                &[],
                "configured retained diagnostic capacity",
                &count.to_string(),
                "separate explicit development policy; original diagnostics are never discarded",
            ));
        }
        Ok(())
    }
    pub fn publish_development_units(&mut self) -> Checked<()> {
        if self.outcome.development_units.requested == DevelopmentUnitPolicy::Disabled {
            return Ok(());
        }
        if !self.development_units.units_checked {
            return Err(internal());
        }
        for record in &self.development_units.candidates {
            if self
                .development_units
                .definitions
                .binary_search_by_key(record, |d| d.record_index)
                .is_err()
            {
                return Err(internal());
            }
        }
        let count = self
            .outcome
            .development_units
            .original
            .len()
            .saturating_add(self.outcome.preserved_diagnostics().len())
            .saturating_add(self.outcome.interpretation_diagnostics().len());
        if count > self.outcome.limits.preserved_diagnostics {
            let r = *self
                .development_units
                .candidates
                .first()
                .ok_or_else(internal)?;
            return Err(self.failure(Status::ResourceLimit, "DevelopmentUnitDiagnosticLimit", "D2",
                "Development source diagnostic capacity exceeded", self.record(r)?.span,
                Some(r), 0, &[], "configured retained diagnostic capacity", &count.to_string(),
                "shared retained-diagnostic capacity includes independent development and nongeometric evidence"));
        }
        if self.development_units.candidates.is_empty() {
            return Ok(());
        }
        self.outcome.development_units.receipt = Some(Arc::new(DevelopmentUnitReceipt {
            policy: self.outcome.development_units.requested,
            source_syntax_verified: false,
            formal_ap_validity: "Unestablished",
            qualification: "ExplicitDevelopmentUnitCompatibilityOnly",
            affected_units: std::mem::take(&mut self.development_units.candidates),
            original_diagnostics: std::mem::take(&mut self.outcome.development_units.original),
            unit_definitions: std::mem::take(&mut self.development_units.definitions),
        }));
        self.outcome.types = "CheckedWithDevelopmentDiagnostics";
        self.outcome.unit_dependencies = "CheckedWithDevelopmentDiagnostics";
        Ok(())
    }
}
