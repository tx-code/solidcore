// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Final source-ownership and physical-exclusion proof for provisional errors.
//! Upstream: `StepSemantics.cpp::ApprovePreservedMetadata` at `d8ed45a`.
//! Deviations: checked indices and the shared iterative source reach helper;
//! every diagnostic remains provisional until the complete proof succeeds.

use super::{OwnershipEvidence, Pending, Proof};
use crate::semantics::checker::{internal, Checked, Checker, Failure};
use crate::semantics::{GeometryScope, Part, Status};
use crate::ValueKind;

fn flag(values: &[bool], r: usize) -> Checked<bool> {
    values.get(r).copied().ok_or_else(internal)
}
fn selected(part: &Part, r: usize, names: &[&str]) -> bool {
    part.roles
        .iter()
        .any(|role| role.record_index == r && names.contains(&role.name.as_str()))
}
fn role(part: &Part, name: &str) -> Checked<usize> {
    part.roles
        .iter()
        .find(|r| r.name == name)
        .map(|r| r.record_index)
        .ok_or_else(internal)
}
fn progress(failure: &Failure) -> &'static str {
    match failure.status {
        Status::Rejected => "Rejected",
        Status::Unsupported => "Unsupported",
        Status::ResourceLimit => "ResourceLimit",
        _ => "InfrastructureFailure",
    }
}
fn type_failure(c: &mut Checker<'_>, failure: &Failure) {
    c.outcome.types = progress(failure);
}
fn reach(c: &mut Checker<'_>, e: &OwnershipEvidence, roots: Vec<usize>) -> Checked<Vec<bool>> {
    crate::semantics::ownership::reach(c, &e.outgoing, roots, false)
}
fn use_graph<'a>(
    c: &mut Checker<'_>,
    e: &OwnershipEvidence,
    graph: &'a mut Option<Vec<bool>>,
    roots: &[usize],
) -> Checked<&'a [bool]> {
    if graph.is_none() {
        *graph = Some(reach(c, e, roots.to_vec())?);
    }
    graph.as_deref().ok_or_else(internal)
}

pub(super) fn run(c: &mut Checker<'_>) -> Checked<()> {
    if c.pending.is_empty() {
        return Ok(());
    }
    let part = c.part.take().ok_or_else(internal)?;
    let evidence = c.ownership_evidence.take().ok_or_else(internal)?;
    let issues = std::mem::take(&mut c.pending);
    let result = approve(c, &part, &evidence, &issues);
    // Restore owning source facts on both paths. No snapshot/report can treat
    // partially approved issues as accepted; promotion is one final operation.
    c.part = Some(part);
    if result.is_ok() {
        c.outcome.qualification.preserved = issues
            .into_iter()
            .map(|issue| issue.failure.diagnostic)
            .collect();
    }
    result
}

fn approve(
    c: &mut Checker<'_>,
    part: &Part,
    evidence: &OwnershipEvidence,
    issues: &[Pending],
) -> Checked<()> {
    let mut roots = vec![role(part, "representation")?, role(part, "unit_context")?];
    for name in ["wireframe_representation", "construction_representation"] {
        if let Some(r) = part.roles.iter().find(|r| r.name == name) {
            roots.push(r.record_index);
        }
    }
    let physical = reach(c, evidence, roots)?;
    let mut presentation_use = None;
    let mut property_use = None;
    let mut selected_use = None;
    let mut property_items = Vec::new();
    let mut metadata = vec![false; c.source.records().len()];
    for attachment in &part.property_attachments {
        for value in &c.field(attachment.representation, "items")?.1.children {
            c.visit(value.span)?;
            let target = c.target(value)?;
            *metadata.get_mut(target).ok_or_else(internal)? = true;
            property_items.push(target);
        }
    }
    // Preserve the original final-quota timing, AFTER physical/property traversal.
    // UnitDependencies deliberately has its separate, earlier per-phase quota.
    if issues
        .len()
        .saturating_add(c.outcome.qualification.interpretation.len())
        > c.outcome.limits.preserved_diagnostics
    {
        let span = issues
            .last()
            .ok_or_else(internal)?
            .failure
            .diagnostic
            .source
            .primary;
        return Err(c.failure(
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
    for issue in issues {
        let failure = &issue.failure;
        let d = &failure.diagnostic;
        c.visit(d.source.primary)?;
        if issue.proof == Proof::MissingApplicationProtocol {
            if !c.can_preserve_missing_protocol()
                || part.application_protocol_present
                || !c.population("APPLICATION_PROTOCOL_DEFINITION")?.is_empty()
                || !matches!(c.outcome.part, "Passed" | "Deferred")
            {
                return Err(clone_failure(failure));
            }
            continue;
        }
        let r = c
            .source
            .data
            .index
            .get(&d.source.record_id)
            .copied()
            .ok_or_else(internal)?;
        let key = d.source.key.as_str();
        if issue.proof == Proof::ContextApplicationProtocols {
            if c.scope != GeometryScope::GeneralBrep {
                return Err(clone_failure(failure));
            }
            for protocol in c.population("APPLICATION_PROTOCOL_DEFINITION")? {
                if !selected(
                    part,
                    protocol,
                    &["application_protocol", "definition_application_protocol"],
                ) || flag(&physical, protocol)?
                {
                    return Err(clone_failure(failure));
                }
            }
            continue;
        }
        if issue.proof == Proof::DistinctApplicationContexts {
            if c.scope != GeometryScope::GeneralBrep {
                return Err(clone_failure(failure));
            }
            for context in c.population("APPLICATION_CONTEXT")? {
                if !selected(
                    part,
                    context,
                    &[
                        "application_context",
                        "product_application_context",
                        "definition_application_context",
                    ],
                ) || flag(&physical, context)?
                {
                    return Err(clone_failure(failure));
                }
            }
            continue;
        }
        if matches!(
            issue.proof,
            Proof::OpaquePartIdentity
                | Proof::OpaqueShapeCaption
                | Proof::OpaqueAdministrativeLabel
        ) {
            let approved = if issue.proof == Proof::OpaquePartIdentity {
                d.field == "id"
                    && c.population("PRODUCT_DEFINITION")?.len() == 1
                    && c.population("PRODUCT_DEFINITION_FORMATION")?.len() == 1
                    && !flag(&physical, r)?
                    && ((d.declared_in == "PRODUCT_DEFINITION"
                        && selected(part, r, &["definition"]))
                        || (d.declared_in == "PRODUCT_DEFINITION_FORMATION"
                            && selected(part, r, &["formation"])))
            } else if issue.proof == Proof::OpaqueShapeCaption {
                d.declared_in == "REPRESENTATION_ITEM"
                    && d.field == "name"
                    && c.is_a(r, "MANIFOLD_SOLID_BREP")
                    && selected(part, r, &["shape"])
            } else {
                let roots: Vec<usize> = part.roles.iter().map(|r| r.record_index).collect();
                let used = use_graph(c, evidence, &mut selected_use, &roots)?;
                !flag(&physical, r)?
                    && flag(used, r)?
                    && ((d.declared_in == "APPROVAL"
                        && d.field == "level"
                        && c.is_a(r, "APPROVAL"))
                        || (d.declared_in == "PERSON" && d.field == "id" && c.is_a(r, "PERSON")))
            };
            if c.scope != GeometryScope::GeneralBrep
                || c.field(r, &d.field)?.1.kind != ValueKind::OpaqueString
                || !approved
            {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
            continue;
        }
        if issue.proof == Proof::OpaqueDisplayText {
            let product = d.declared_in == "PRODUCT"
                && matches!(d.field.as_str(), "id" | "name")
                && selected(part, r, &["product"]);
            let layer = d.declared_in == "PRESENTATION_LAYER_ASSIGNMENT"
                && d.field == "name"
                && evidence.administrative.get(r) == Some(&true)
                && !flag(&physical, r)?;
            if c.scope != GeometryScope::GeneralBrep
                || c.field(r, &d.field)?.1.kind != ValueKind::OpaqueString
                || !(product || layer)
            {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
            continue;
        }
        if issue.proof == Proof::CurveStyleWidth {
            let used = use_graph(
                c,
                evidence,
                &mut presentation_use,
                &evidence.presentation_roots,
            )?;
            if !c.schema()?.ap214()
                || c.scope != GeometryScope::GeneralBrep
                || !c.is_a(r, "CURVE_STYLE")
                || c.field(r, "curve_width")?.1.kind != ValueKind::String
                || !flag(used, r)?
                || flag(&physical, r)?
            {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
            continue;
        }
        if issue.proof == Proof::ForeignRealItem {
            let used = use_graph(c, evidence, &mut property_use, &property_items)?;
            if !c.borrowed_real(r)
                || c.scope != GeometryScope::GeneralBrep
                || !flag(&metadata, r)?
                || !flag(used, r)?
                || flag(&physical, r)?
                || !evidence.outgoing.get(r).ok_or_else(internal)?.is_empty()
            {
                c.outcome.types = "Unsupported";
                return Err(clone_failure(failure));
            }
            continue;
        }
        if key == "UnsupportedEntityType" {
            let context = (selected(part, r, &["product_context"])
                && c.is_a(r, "MECHANICAL_CONTEXT"))
                || (selected(part, r, &["definition_context"]) && c.is_a(r, "DESIGN_CONTEXT"));
            let layer = c.schema()?.kind == "Ap203N916Source"
                && c.is_a(r, "PRESENTATION_LAYER_ASSIGNMENT")
                && evidence.administrative.get(r) == Some(&true);
            if !(layer || c.schema()?.kind == "Ap214MbxSource" && context) || flag(&physical, r)? {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
        }
        if matches!(
            key,
            "DerivedSlotMarkerRequired" | "SourceRuleViolation:SI_UNIT.WR1"
        ) {
            let used = use_graph(c, evidence, &mut property_use, &property_items)?;
            if !flag(used, r)? || flag(&physical, r)? {
                if key == "DerivedSlotMarkerRequired" {
                    type_failure(c, failure);
                } else {
                    c.outcome.unit_dependencies = progress(failure);
                }
                return Err(clone_failure(failure));
            }
        }
        if key == "RequiredAttributeMissing" && d.field == "rendering_method" {
            let used = use_graph(
                c,
                evidence,
                &mut presentation_use,
                &evidence.presentation_roots,
            )?;
            if !c.is_a(r, "SURFACE_STYLE_RENDERING") || !flag(used, r)? || flag(&physical, r)? {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
        }
        if key == "ReferenceTypeMismatch" {
            if d.field == "over_ridden_style" {
                let used = use_graph(
                    c,
                    evidence,
                    &mut presentation_use,
                    &evidence.presentation_roots,
                )?;
                if c.schema()?.kind != "Ap214MbxSource"
                    || !evidence.style_overrides.contains(&r)
                    || !flag(used, r)?
                    || flag(&physical, r)?
                {
                    type_failure(c, failure);
                    return Err(clone_failure(failure));
                }
                continue;
            }
            let target = c
                .source
                .data
                .index
                .get(&d.source.target_id)
                .copied()
                .ok_or_else(internal)?;
            if !selected(part, target, &["definition", "formation"])
                || evidence.administrative.get(r) != Some(&true)
                || flag(&physical, r)?
            {
                type_failure(c, failure);
                return Err(clone_failure(failure));
            }
        }
        if matches!(
            key,
            "SourceRuleViolation:MEASURE_WITH_UNIT.WR1"
                | "SourceRuleViolation:POSITIVE_RATIO_MEASURE.WR1"
        ) && (!flag(&metadata, r)? || flag(&physical, r)?)
        {
            c.outcome.unit_dependencies = progress(failure);
            return Err(clone_failure(failure));
        }
    }
    Ok(())
}
fn clone_failure(failure: &Failure) -> Box<Failure> {
    Box::new(Failure {
        status: failure.status,
        diagnostic: failure.diagnostic.clone(),
    })
}
