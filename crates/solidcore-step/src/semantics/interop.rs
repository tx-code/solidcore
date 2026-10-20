// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Provisional original diagnostics and final nongeometric source exclusion.
//! Upstream: `StepSemantics.cpp` Preserve/ApprovePreservedMetadata at `d8ed45a`.
//! Deviations: immutable Rust qualification owners and checked source indices.
//! No repair, encoding inference, numeric coercion or partial qualification.

mod approval;

use super::checker::{Checked, Checker, Failure};
use super::{Diagnostic, GeometryScope, Options, SchemaInterpretation, SourcePolicy, Status};
use std::collections::BTreeSet;

#[derive(Clone, Default)]
pub(super) struct Qualification {
    pub policy: SourcePolicy,
    pub scope: GeometryScope,
    pub requested: SchemaInterpretation,
    pub applied: SchemaInterpretation,
    pub interpretation: Vec<Diagnostic>,
    pub preserved: Vec<Diagnostic>,
    pub unattached: Vec<usize>,
}
impl Qualification {
    pub fn new(options: &Options) -> Self {
        Self {
            policy: options.source_policy,
            scope: options.geometry_scope,
            requested: options.schema_interpretation,
            ..Self::default()
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Proof {
    ExistingMetadata,
    MissingApplicationProtocol,
    CurveStyleWidth,
    OpaqueDisplayText,
    OpaquePartIdentity,
    OpaqueShapeCaption,
    OpaqueAdministrativeLabel,
    DistinctApplicationContexts,
    ContextApplicationProtocols,
    ForeignRealItem,
}
pub(super) struct Pending {
    pub failure: Box<Failure>,
    pub proof: Proof,
}
pub(super) struct OwnershipEvidence {
    pub outgoing: Vec<Vec<usize>>,
    pub presentation_roots: Vec<usize>,
    pub administrative: Vec<bool>,
    pub style_overrides: BTreeSet<usize>,
}

impl Checker<'_> {
    pub fn interoperability(&self) -> bool {
        self.outcome.qualification.policy == SourcePolicy::PreserveNonGeometric
            && self.catalog.is_some_and(|c| c.kind != "Ap203e2Planar")
    }
    pub fn general_interoperability(&self) -> bool {
        self.interoperability() && self.scope == GeometryScope::GeneralBrep
    }
    pub fn can_preserve_missing_protocol(&self) -> bool {
        self.catalog
            .is_some_and(|c| matches!(c.kind, "Ap214MbxSource" | "Ap203N916Source"))
    }
    pub fn borrowed_real(&self, r: usize) -> bool {
        self.catalog.is_some_and(|c| c.kind == "Ap214MbxSource")
            && self.is_a(r, "REAL_REPRESENTATION_ITEM")
    }
    pub fn non_geometric_style(&self, r: usize) -> bool {
        if self.catalog.is_some_and(super::catalog::Catalog::ap214)
            && self.is_a(r, "PRE_DEFINED_COLOUR")
        {
            return true;
        }
        [
            "PRESENTATION_STYLE_ASSIGNMENT",
            "SURFACE_STYLE_USAGE",
            "SURFACE_SIDE_STYLE",
            "SURFACE_STYLE_FILL_AREA",
            "FILL_AREA_STYLE",
            "FILL_AREA_STYLE_COLOUR",
            "COLOUR_RGB",
            "SURFACE_STYLE_RENDERING",
            "SURFACE_STYLE_TRANSPARENT",
            "CURVE_STYLE",
            "PRE_DEFINED_CURVE_FONT",
            "POINT_STYLE",
            "PRE_DEFINED_POINT_MARKER_SYMBOL",
        ]
        .iter()
        .any(|name| self.is_a(r, name))
    }
    pub fn preserve(&mut self, failure: Box<Failure>, proof: Proof) -> Checked<()> {
        // Original source error is provisional. Approval must run after physical
        // context and closure checks, before ANY semantic snapshot can escape.
        let span = failure.diagnostic.source.primary;
        self.pending.push(Pending { failure, proof });
        if self
            .pending
            .len()
            .saturating_add(self.outcome.qualification.interpretation.len())
            > self.outcome.limits.preserved_diagnostics
        {
            return Err(self.failure(
                Status::ResourceLimit,
                "PreservedDiagnosticLimit",
                "D2",
                "Interoperability diagnostic capacity exceeded",
                span,
                None,
                0,
                &[],
                "",
                "",
                "",
            ));
        }
        Ok(())
    }
    pub fn publish_qualification_progress(&mut self) {
        if self.outcome.binding == "Passed"
            && self.outcome.qualification.applied != SchemaInterpretation::DeclaredOnly
        {
            self.outcome.binding = "InterpretedUnverified";
        }
        for issue in &self.outcome.qualification.preserved {
            match issue.source.key.as_str() {
                "RequiredAttributeMissing"
                | "AttributeTypeMismatch"
                | "ReferenceTypeMismatch"
                | "DerivedSlotMarkerRequired"
                | "UnsupportedEntityType"
                | "OpaqueDataTextUnestablished" => self.outcome.types = "CheckedWithDiagnostics",
                "UnsupportedProductCategory"
                | "UnsupportedDefinitionContextRole"
                | "UnownedOrOrphanProfileContent"
                | "UnsupportedSinglePartGraph" => self.outcome.part = "CheckedWithDiagnostics",
                _ => self.outcome.unit_dependencies = "CheckedWithDiagnostics",
            }
        }
        if matches!(
            self.outcome.part,
            "Passed" | "CheckedWithDiagnostics" | "Deferred"
        ) && self
            .part
            .as_ref()
            .is_some_and(|p| !p.deferred_region_faces.is_empty())
        {
            self.outcome.part = "Deferred";
        }
    }
    pub fn approve_preserved(&mut self) -> Checked<()> {
        approval::run(self)
    }
}
