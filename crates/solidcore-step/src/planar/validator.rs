// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Ordered planar admission/phases and source-bound failures.
//! Upstream: `source/STEP_PLANAR/src/StepPlanar.cpp` at `d8ed45a`.
//! Deviations: checked source access and explicit Result propagation; partial
//! evidence survives only as diagnostics, with no native model qualification.

use super::{Evidence, ExactError, Limits, Outcome, Quality, Rational as R, Status, Vector};
use crate::{semantics as s, Diagnostic as RawDiagnostic, Record, Span, Value, ValueKind};

#[derive(Clone, Copy)]
pub(super) enum Failure {
    Stop(Status),
    Numeric(&'static str),
    Infrastructure,
}
pub(super) type Checked<T> = Result<T, Failure>;
impl From<ExactError> for Failure {
    fn from(error: ExactError) -> Self {
        match error {
            ExactError::Capacity => Self::Numeric("ExactArithmeticCapacityExceeded"),
            ExactError::UndefinedDivision => Self::Numeric("UndefinedExactDivision"),
            ExactError::InvalidDecimal => Self::Infrastructure,
        }
    }
}
#[derive(Clone, Copy)]
pub(super) enum Phase {
    Local,
    Global,
    Geometry,
}
pub(super) struct Validator<'o, 's> {
    pub out: &'o mut Outcome,
    pub sem: &'s s::Snapshot,
    pub current: usize,
    pub phase: Option<Phase>,
}
impl<'s> Validator<'_, 's> {
    pub fn record(&self, r: usize) -> Checked<&'s Record> {
        self.sem
            .source()
            .records()
            .get(r)
            .ok_or(Failure::Infrastructure)
    }
    pub fn field(&self, r: usize, name: &str) -> Checked<&'s Value> {
        // Bindings/ownership remain semantic-owned. No second field or ID map.
        let bound = self
            .sem
            .field(r, None, name)
            .ok_or(Failure::Infrastructure)?;
        let attr = bound.declaration().ok_or(Failure::Infrastructure)?;
        self.record(r)?
            .components
            .get(attr.component)
            .and_then(|c| c.parameters.get(attr.parameter))
            .ok_or(Failure::Infrastructure)
    }
    pub fn target(&self, value: &Value) -> Checked<usize> {
        if value.kind != ValueKind::Reference {
            return Err(Failure::Infrastructure);
        }
        self.sem
            .source()
            .data
            .index
            .get(&value.text)
            .copied()
            .ok_or(Failure::Infrastructure)
    }
    pub fn reference(&self, r: usize, name: &str) -> Checked<usize> {
        self.target(self.field(r, name)?)
    }
    pub fn refs(&self, r: usize, name: &str) -> Checked<Vec<usize>> {
        self.field(r, name)?
            .children
            .iter()
            .map(|v| self.target(v))
            .collect()
    }
    pub fn population(&self, name: &str) -> Checked<Vec<usize>> {
        self.sem
            .instances(name)
            .map(<[usize]>::to_vec)
            .ok_or(Failure::Infrastructure)
    }
    pub fn is(&self, r: usize, name: &str) -> bool {
        self.sem.is_a(r, name) == Some(true)
    }
    pub fn has(&self, r: usize, name: &str) -> Checked<bool> {
        // Closed-signature exclusions use actual TYPEOF lists, not a fabricated
        // empty public population for unknown types.
        Ok(self
            .sem
            .types_of(r)
            .ok_or(Failure::Infrastructure)?
            .contains(&name))
    }
    pub fn count_types(&self, r: usize, names: &[&str]) -> Checked<usize> {
        let mut count = 0;
        for name in names {
            count += usize::from(self.has(r, name)?);
        }
        Ok(count)
    }
    pub fn sense(&self, r: usize, name: &str) -> Checked<bool> {
        Ok(self.field(r, name)?.text == "T")
    }
    pub fn role(&self, name: &str) -> Checked<usize> {
        self.sem
            .part_definition()
            .and_then(|p| p.roles.iter().find(|r| r.name == name))
            .map(|r| r.record_index)
            .ok_or(Failure::Infrastructure)
    }
    pub fn tick(&mut self, owner: usize) -> Checked<()> {
        self.current = owner;
        if self.out.work >= self.out.limits.work {
            return Err(self.fail(
                "PlanarWorkLimit",
                "Resource",
                owner,
                "",
                "bounded source-rule and polygon work",
                Status::ResourceLimit,
                &[],
                "",
                "",
            ));
        }
        self.out.work += 1;
        Ok(())
    }
    pub fn fail(
        &mut self,
        key: &str,
        stage: &str,
        owner: usize,
        field: &str,
        responsibility: &str,
        status: Status,
        related: &[usize],
        expected: &str,
        actual: &str,
    ) -> Failure {
        let Ok(record) = self.record(owner) else {
            return Failure::Infrastructure;
        };
        let mut d = diagnostic(
            key,
            stage,
            responsibility,
            match status {
                Status::Rejected => "SourceInvalid",
                Status::Unsupported => "ProfileUnsupported",
                Status::ResourceLimit => "ResourceLimit",
                _ => "Indeterminate",
            },
        );
        d.source.record_id.clone_from(&record.id);
        d.source.primary = record.span;
        d.field = field.into();
        if !field.is_empty() {
            let Some(view) = self.sem.field(owner, None, field) else {
                return Failure::Infrastructure;
            };
            let (Some(a), Some(value)) = (view.declaration(), view.value()) else {
                return Failure::Infrastructure;
            };
            d.source.primary = value.span;
            let Some(component) = record.components.get(a.component) else {
                return Failure::Infrastructure;
            };
            d.source.component.clone_from(&component.name);
            d.source.parameter_path = vec![a.parameter + 1];
            d.declared_in = a.declared_in.into();
            d.declared_type = a.declared_type.into();
        }
        d.expected = expected.into();
        d.actual = actual.into();
        for witness in related {
            if *witness != owner {
                let Ok(record) = self.record(*witness) else {
                    return Failure::Infrastructure;
                };
                d.source.related.push(record.span);
            }
        }
        self.out.diagnostics.push(d);
        Failure::Stop(status)
    }
    pub fn reject(
        &mut self,
        key: &str,
        stage: &str,
        owner: usize,
        field: &str,
        reason: &str,
    ) -> Failure {
        self.fail(
            key,
            stage,
            owner,
            field,
            reason,
            Status::Rejected,
            &[],
            "",
            "",
        )
    }
    pub fn components(&self, r: usize, name: &str) -> Checked<Vector> {
        let source = &self.field(r, name)?.children;
        if source.len() != 3 {
            return Err(Failure::Numeric("ThreeDimensionalComponentsRequired"));
        }
        let mut values = [R::ZERO; 3];
        for (a, value) in values.iter_mut().zip(source) {
            *a = R::decimal(&value.text)?;
        }
        Ok(values)
    }
    pub fn point(&self, r: usize) -> Checked<Vector> {
        self.components(r, "coordinates")
    }
    pub fn direction(&self, r: usize) -> Checked<Vector> {
        self.components(r, "direction_ratios")
    }
    pub fn ends(&self, oe: usize) -> Checked<(usize, usize)> {
        let ec = self.reference(oe, "edge_element")?;
        if !self.is(ec, "EDGE_CURVE") {
            return Err(Failure::Numeric("UnsupportedOrientedEdgeElement"));
        }
        let start = self.reference(ec, "edge_start")?;
        let end = self.reference(ec, "edge_end")?;
        Ok(if self.sense(oe, "orientation")? {
            (start, end)
        } else {
            (end, start)
        })
    }
    pub fn numeric_diagnostic(&self, key: &str) -> Checked<s::Diagnostic> {
        let mut d = diagnostic(
            key,
            if matches!(self.phase, Some(Phase::Local)) {
                "D3"
            } else {
                "D4"
            },
            "Required exact evidence is unestablished; no rounded or invented fallback",
            "Indeterminate",
        );
        d.source
            .record_id
            .clone_from(&self.record(self.current)?.id);
        d.source.primary = self.record(self.current)?.span;
        d.responsibility = "numeric/derived evidence, not a proved source violation".into();
        Ok(d)
    }
}
pub(super) fn diagnostic(
    key: &str,
    stage: &str,
    message: &str,
    classification: &str,
) -> s::Diagnostic {
    s::Diagnostic {
        source: RawDiagnostic {
            key: key.into(),
            stage: stage.into(),
            message: message.into(),
            primary: Span::default(),
            related: Vec::new(),
            record_id: String::new(),
            component: String::new(),
            parameter_path: Vec::new(),
            target_id: String::new(),
        },
        classification: classification.into(),
        field: String::new(),
        declared_in: String::new(),
        declared_type: String::new(),
        expected: String::new(),
        actual: String::new(),
        responsibility: message.into(),
    }
}
pub(super) fn add(a: Vector, b: Vector) -> Result<Vector, ExactError> {
    let mut out = a;
    for (x, y) in out.iter_mut().zip(b) {
        *x = x.checked_add(y)?;
    }
    Ok(out)
}
pub(super) fn subtract(a: Vector, b: Vector) -> Result<Vector, ExactError> {
    let mut out = a;
    for (x, y) in out.iter_mut().zip(b) {
        *x = x.subtract(y)?;
    }
    Ok(out)
}
pub(super) fn times(a: Vector, b: R) -> Result<Vector, ExactError> {
    let mut out = a;
    for x in &mut out {
        *x = x.multiply(b)?;
    }
    Ok(out)
}
pub(super) fn dot(a: Vector, b: Vector) -> Result<R, ExactError> {
    let [a0, a1, a2] = a;
    let [b0, b1, b2] = b;
    a0.multiply(b0)?
        .checked_add(a1.multiply(b1)?)?
        .checked_add(a2.multiply(b2)?)
}
pub(super) fn cross(a: Vector, b: Vector) -> Result<Vector, ExactError> {
    let [a0, a1, a2] = a;
    let [b0, b1, b2] = b;
    Ok([
        a1.multiply(b2)?.subtract(a2.multiply(b1)?)?,
        a2.multiply(b0)?.subtract(a0.multiply(b2)?)?,
        a0.multiply(b1)?.subtract(a1.multiply(b0)?)?,
    ])
}
pub(super) fn squared(a: Vector) -> Result<R, ExactError> {
    dot(a, a)
}

pub(super) fn check(upstream: &s::Outcome, quality: Quality, limits: Limits) -> Outcome {
    let mut out = Outcome {
        source_capture: upstream.captured_owner(),
        status: Status::UpstreamFailure,
        quality,
        limits,
        work: 0,
        local_rules: "NotRun",
        global_rules: "NotRun",
        geometry: "NotRun",
        conversion_units: "NotRun",
        evidence: Evidence {
            source: None,
            rules: Vec::new(),
            metrics: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            conversions: Vec::new(),
        },
        diagnostics: Vec::new(),
    };
    if out.quality.id.is_empty()
        || out.quality.consistency_mm <= R::ZERO
        || limits.work > 10_000_000
        || (out.quality.id == "Q-P0-mm-01" && out.quality.consistency_mm != R::FIXTURE_MM)
    {
        out.status = Status::InvalidRequest;
        out.diagnostics.push(diagnostic(
            "InvalidPlanarConfiguration",
            "D3",
            "Explicit positive quality and bounded work configuration required",
            "InvalidRequest",
        ));
        return out;
    }
    if !matches!(upstream.status, s::Status::Checked | s::Status::Deferred) {
        out.diagnostics.clone_from(&upstream.diagnostics);
        return out;
    }
    let Some(sem) = upstream.snapshot.as_ref() else {
        out.status = Status::InvalidRequest;
        out.diagnostics.push(diagnostic(
            "MissingSemanticSnapshot",
            "D3",
            "Missing complete semantic snapshot",
            "InvalidRequest",
        ));
        return out;
    };
    out.evidence.source = Some(sem.clone());
    if upstream.status == s::Status::Deferred
        && (upstream.binding != "Passed"
            || upstream.types != "Passed"
            || upstream.part != "Passed"
            || upstream.si_units != "Deferred"
            || !upstream.diagnostics.is_empty())
    {
        out.status = Status::InvalidRequest;
        out.diagnostics.push(diagnostic(
            "InvalidDeferredHandoff",
            "D3",
            "Only the complete typed conversion-unit handoff can be resumed",
            "InvalidRequest",
        ));
        return out;
    }
    if sem.profile().kind != "Ap203e2Planar" {
        let mut d = diagnostic("UnsupportedRuleProfile", "D3", "Source declarations were checked; this schema's local/global rules and geometry are not implemented", "ProfileUnsupported");
        if let Some(v) = sem
            .source()
            .header()
            .get(2)
            .and_then(|h| h.parameters.first())
            .and_then(|v| v.children.first())
        {
            d.source.primary = v.span;
        }
        d.actual = sem.profile().profile.into();
        d.responsibility =
            "Protocol-specific rule implementation required before geometric validation".into();
        out.status = Status::Unsupported;
        out.diagnostics.push(d);
        return out;
    }
    if sem.part_definition().is_none() || sem.units().context.is_none() {
        out.status = Status::InvalidRequest;
        out.diagnostics.push(diagnostic(
            "IncompleteSemanticHandoff",
            "D3",
            "Complete source ownership and physical-unit checking is required",
            "InvalidRequest",
        ));
        return out;
    }
    let mut validator = Validator {
        out: &mut out,
        sem,
        current: 0,
        phase: None,
    };
    let result = (|| -> Checked<()> {
        for r in 0..sem.source().records().len() {
            validator.tick(r)?;
        }
        validator.phase = Some(Phase::Local);
        validator.out.local_rules = "Running";
        validator.local()?;
        validator.out.local_rules = "Passed";
        validator.phase = Some(Phase::Global);
        validator.out.global_rules = "Running";
        validator.global_rules()?;
        validator.out.global_rules = "Passed";
        validator.phase = Some(Phase::Geometry);
        validator.out.geometry = "Running";
        validator.geometry()?;
        validator.out.geometry = "Passed";
        validator.phase = None;
        Ok(())
    })();
    validator.out.status = match result {
        Ok(()) => Status::Checked,
        Err(Failure::Stop(status)) => status,
        Err(Failure::Numeric(key)) => {
            if let Ok(d) = validator.numeric_diagnostic(key) {
                validator.out.diagnostics.push(d);
            } else {
                validator.out.status = Status::InfrastructureFailure;
                return out;
            }
            Status::Indeterminate
        }
        Err(Failure::Infrastructure) => {
            validator.out.diagnostics.push(diagnostic(
                "PlanarInvariantFailure",
                "Infrastructure",
                "Internal source binding or registry invariant failed",
                "InfrastructureFailure",
            ));
            Status::InfrastructureFailure
        }
    };
    let progress = match validator.out.status {
        Status::Indeterminate => "Indeterminate",
        Status::ResourceLimit => "ResourceLimit",
        _ => "Failed",
    };
    for state in [
        &mut validator.out.local_rules,
        &mut validator.out.global_rules,
        &mut validator.out.geometry,
        &mut validator.out.conversion_units,
    ] {
        if *state == "Running" {
            *state = progress;
        }
    }
    if validator.out.status == Status::Indeterminate {
        if let Some(last) = validator
            .out
            .evidence
            .rules
            .last_mut()
            .filter(|r| r.value.is_none())
        {
            last.evaluation = "Indeterminate";
        }
    }
    out
}
