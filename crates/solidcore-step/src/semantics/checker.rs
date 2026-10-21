// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Strict typed source mapping and physical-context prerequisites.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemantics.cpp` (Binding/Mapping/Fields/UnitContext).
//! Rust Checker corresponds to the C++ Checker with explicit phase selection.
//! Deviations: checked indexing, owning immutable source sharing and explicit
//! checkpoint selection; nested aggregates keep the source depth ceiling.

use super::catalog::{Catalog, Field, Kind, Membership, Unique};
use super::{
    generated::CATALOGS, number, schema, Attribute, Data, Diagnostic, Limits, Options, Outcome,
    Profile, SiUnit, Snapshot, Status, UnitChecks, Units,
};
use crate::{
    Diagnostic as RawDiagnostic, Outcome as RawOutcome, Record, Snapshot as RawSnapshot, Span,
    Value, ValueKind,
};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

pub(super) type Checked<T> = Result<T, Box<Failure>>;
pub(super) struct Failure {
    pub status: Status,
    pub diagnostic: Diagnostic,
}
pub(super) fn internal() -> Box<Failure> {
    simple_failure(
        Status::InfrastructureFailure,
        "SemanticInvariantFailure",
        "InfrastructureFailure",
        "internal descriptor/binding invariant",
    )
}
fn simple_failure(status: Status, key: &str, classification: &str, message: &str) -> Box<Failure> {
    Box::new(Failure {
        status,
        diagnostic: Diagnostic {
            source: RawDiagnostic {
                key: key.into(),
                stage: "D2".into(),
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
            responsibility: String::new(),
        },
    })
}
pub(super) struct Checker<'a> {
    pub source: &'a RawSnapshot,
    pub catalog: Option<&'static Catalog>,
    pub profile: Option<Profile>,
    pub types: Vec<Membership>,
    pub populations: Vec<Vec<usize>>,
    pub attributes: Vec<Vec<Attribute>>,
    pub units: Units,
    pub part: Option<super::Part>,
    pub representation_use: Option<Vec<bool>>,
    pub scope: super::GeometryScope,
    pub pending: Vec<super::interop::Pending>,
    pub ownership_evidence: Option<super::interop::OwnershipEvidence>,
    pub development_units: super::development_units::Working,
    pub outcome: Outcome,
}
impl<'a> Checker<'a> {
    pub fn schema(&self) -> Checked<&'static Catalog> {
        self.catalog.ok_or_else(internal)
    }
    pub fn record(&self, r: usize) -> Checked<&'a Record> {
        self.source.records().get(r).ok_or_else(internal)
    }
    pub fn value(&self, r: usize, a: &Attribute) -> Checked<&'a Value> {
        self.record(r)?
            .components
            .get(a.component)
            .and_then(|c| c.parameters.get(a.parameter))
            .ok_or_else(internal)
    }
    pub fn field(&self, r: usize, name: &str) -> Checked<(Attribute, &'a Value)> {
        let mut found = self
            .attributes
            .get(r)
            .ok_or_else(internal)?
            .iter()
            .filter(|a| a.name == name);
        let a = found.next().ok_or_else(internal)?;
        if found.next().is_some() {
            return Err(internal());
        }
        Ok((a.clone(), self.value(r, a)?))
    }
    pub fn target(&self, v: &Value) -> Checked<usize> {
        self.source
            .data
            .index
            .get(&v.text)
            .copied()
            .ok_or_else(internal)
    }
    pub fn is_a(&self, r: usize, name: &str) -> bool {
        self.catalog
            .zip(self.types.get(r))
            .is_some_and(|(c, m)| c.is_a(*m, name))
    }
    pub fn population(&self, name: &str) -> Checked<Vec<usize>> {
        let id = self.schema()?.find(name).ok_or_else(internal)?;
        self.populations.get(id).cloned().ok_or_else(internal)
    }
    pub fn raw(&self, span: Span) -> String {
        self.source
            .raw(span)
            .map_or_else(String::new, |v| v.iter().copied().map(char::from).collect())
    }
    fn borrowed(&self, r: usize) -> bool {
        self.catalog.is_some_and(|c| c.kind == "Ap203N916Source")
            && self.is_a(r, "PRESENTATION_LAYER_ASSIGNMENT")
            || self.catalog.is_some_and(|c| c.kind == "Ap214MbxSource")
                && self.is_a(r, "REAL_REPRESENTATION_ITEM")
    }
    fn authority(&self, r: usize) -> &'static str {
        if self.catalog.is_some_and(|c| c.kind == "Ap203N916Source")
            && self.is_a(r, "PRESENTATION_LAYER_ASSIGNMENT")
        {
            return "N2635 borrowed foreign-layer transport (not a N916 declaration)";
        }
        if self.catalog.is_some_and(|c| c.kind == "Ap214MbxSource")
            && self.is_a(r, "REAL_REPRESENTATION_ITEM")
        {
            return "N2635 borrowed foreign REAL-item transport (not an AP214 declaration)";
        }
        self.catalog.map_or("", Catalog::authority)
    }
    pub fn failure(
        &self,
        mut status: Status,
        key: &str,
        stage: &str,
        message: &str,
        span: Span,
        owner: Option<usize>,
        component: usize,
        path: &[usize],
        expected: &str,
        actual: &str,
        responsibility: &str,
    ) -> Box<Failure> {
        let mut responsibility = responsibility.to_owned();
        let mut stage = stage;
        if status == Status::Rejected && owner.is_some_and(|r| self.borrowed(r)) {
            status = Status::Unsupported;
            stage = "D2";
            if let Some(r) = owner {
                let authority = self.authority(r);
                if !responsibility.contains(authority) {
                    responsibility = format!("{authority}: {responsibility}");
                }
            }
        }
        let classification = match status {
            Status::Unsupported => "ProfileUnsupported",
            Status::ResourceLimit => "ResourceLimit",
            Status::InvalidRequest => "InvalidRequest",
            Status::InfrastructureFailure => "InfrastructureFailure",
            _ => "SourceInvalid",
        };
        let mut failure = simple_failure(status, key, classification, message);
        let d = &mut failure.diagnostic;
        d.source.stage = stage.into();
        d.source.primary = span;
        d.source.parameter_path = path.to_vec();
        d.expected = expected.into();
        d.actual = actual.into();
        d.responsibility = responsibility;
        if let Some(r) = owner {
            if let Some(record) = self.source.records().get(r) {
                d.source.record_id.clone_from(&record.id);
                if let Some(c) = record.components.get(component) {
                    d.source.component.clone_from(&c.name);
                }
                if let Some(p) = path.first().and_then(|p| p.checked_sub(1)) {
                    if let Ok(a) = self.attribute(r, component, p) {
                        d.field = a.name.into();
                        d.declared_in = a.declared_in.into();
                        d.declared_type = a.declared_type.into();
                    }
                }
            }
        }
        failure
    }
    pub fn visit(&mut self, span: Span) -> Checked<()> {
        self.outcome.dependency_visits += 1;
        if self.outcome.dependency_visits > self.outcome.limits.dependency_visits {
            return Err(self.failure(
                Status::ResourceLimit,
                "DependencyVisitLimit",
                "D2",
                "Configured semantic traversal budget exceeded",
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
    pub fn field_failure(
        &self,
        status: Status,
        key: &str,
        stage: &str,
        r: usize,
        name: &str,
        expected: &str,
        actual: &str,
        responsibility: &str,
    ) -> Box<Failure> {
        match self.field(r, name) {
            Ok((a, v)) => self.failure(
                status,
                key,
                stage,
                "Source field check failed",
                v.span,
                Some(r),
                a.component,
                &[a.parameter + 1],
                expected,
                actual,
                responsibility,
            ),
            Err(failure) => failure,
        }
    }
    fn attribute(&self, r: usize, c: usize, p: usize) -> Checked<Attribute> {
        let catalog = self.schema()?;
        let record = self.record(r)?;
        let part = record.components.get(c).ok_or_else(internal)?;
        let t = catalog.find(&part.name).ok_or_else(internal)?;
        let (origin, local) = if record.complex {
            (t, p)
        } else {
            *catalog
                .declarations
                .get(t)
                .and_then(|d| d.flattened.get(p))
                .ok_or_else(internal)?
        };
        let m = *self.types.get(r).ok_or_else(internal)?;
        let (f, redeclared_by, derived_slot) =
            catalog.field(m, origin, local).ok_or_else(internal)?;
        let aggregate = f.kind == Kind::Aggregate;
        Ok(Attribute {
            component: c,
            parameter: p,
            name: f.name,
            declared_in: catalog.declarations.get(origin).ok_or_else(internal)?.name,
            declared_type: f.declared_type,
            redeclared_by,
            optional: f.optional,
            derived_slot,
            aggregate: if !aggregate {
                ""
            } else if f.unique == Unique::Set {
                "SET"
            } else {
                "LIST"
            },
            minimum: if aggregate { f.minimum } else { 0 },
            maximum: if aggregate { f.maximum } else { None },
            unique_elements: aggregate && f.unique != Unique::None,
            nested_aggregates: f
                .nested
                .iter()
                .map(|d| super::AggregateDimension {
                    aggregate: if d.set { "SET" } else { "LIST" },
                    minimum: d.minimum,
                    maximum: d.maximum,
                    unique_elements: d.unique,
                })
                .collect(),
        })
    }
    fn binding(&mut self, options: &Options) -> Checked<()> {
        let selection = schema::select(self.source, options.geometry_scope);
        self.outcome.schema_selection = Some(selection);
        let selection = self
            .outcome
            .schema_selection
            .as_ref()
            .ok_or_else(internal)?;
        let schema_header = self.source.header().get(2).ok_or_else(internal)?;
        if selection.identifiers.len() != 1 {
            return Err(self.failure(
                Status::Unsupported,
                "UnsupportedSchemaBinding",
                "D2",
                "One precise schema name/OID is required",
                schema_header.span,
                None,
                0,
                &[],
                "one N2635 identifier",
                &selection.identifiers.len().to_string(),
                "profile registry",
            ));
        }
        let identity = selection.identifiers.first().ok_or_else(internal)?;
        let profile = if let Some(profile) = selection.qualified.clone() {
            profile
        } else {
            let failure = self.failure(
                Status::Unsupported,
                "UnsupportedSchemaBinding",
                "D2",
                "HEADER identifier is not bound to the fixed N2635 registry entry",
                identity.span,
                None,
                0,
                &[],
                schema::AP203_EXPECTED,
                &identity.literal,
                "profile registry; no APD/year inference",
            );
            let Some(profile) = schema::explicit(selection, options.schema_interpretation) else {
                return Err(failure);
            };
            if self.outcome.limits.preserved_diagnostics == 0 {
                return Err(self.failure(
                    Status::ResourceLimit,
                    "PreservedDiagnosticLimit",
                    "D2",
                    "Source interpretation diagnostic capacity exceeded",
                    identity.span,
                    None,
                    0,
                    &[],
                    "",
                    "",
                    "",
                ));
            }
            self.outcome
                .qualification
                .interpretation
                .push(failure.diagnostic);
            self.outcome.qualification.applied = options.schema_interpretation;
            let selection = self
                .outcome
                .schema_selection
                .as_mut()
                .ok_or_else(internal)?;
            selection.qualified = Some(profile.clone());
            selection.applied_interpretation = options.schema_interpretation;
            profile
        };
        self.catalog = CATALOGS.iter().find(|c| c.kind == profile.kind);
        self.profile = Some(profile);
        self.schema()?;
        Ok(())
    }
    fn mapping(&mut self) -> Checked<()> {
        let catalog = self.schema()?;
        self.types = vec![Membership::default(); self.source.records().len()];
        self.populations = catalog.declarations.iter().map(|_| Vec::new()).collect();
        for (r, record) in self.source.records().iter().enumerate() {
            self.visit(record.span)?;
            let mut unique = BTreeSet::new();
            let mut explicit = Vec::new();
            let mut bits = Membership::default();
            for (c, part) in record.components.iter().enumerate() {
                if !unique.insert(part.name.as_str()) {
                    return Err(self.failure(
                        Status::Rejected,
                        "DuplicateComplexConstituent",
                        "D1",
                        "A partial entity occurs twice in one instance",
                        part.span,
                        Some(r),
                        c,
                        &[],
                        "unique constituent",
                        &part.name,
                        "external mapping",
                    ));
                }
                let Some(t) = catalog.find(&part.name) else {
                    return Err(self.failure(
                        Status::Unsupported,
                        if record.complex {
                            "UnsupportedComplexConstituent"
                        } else {
                            "UnsupportedEntityType"
                        },
                        "D2",
                        "Required content is outside the closed source profile",
                        part.span,
                        Some(r),
                        c,
                        &[],
                        "registered profile entity",
                        &part.name,
                        "closed type guard; unknown content is not skipped",
                    ));
                };
                let d = catalog.declarations.get(t).ok_or_else(internal)?;
                bits.union(d.ancestors);
                *self.types.get_mut(r).ok_or_else(internal)? = bits;
                explicit.push(part.name.as_str());
                if !record.complex && !d.simple {
                    return Err(self.failure(
                        Status::Unsupported,
                        if part.name == "FACE_SURFACE" {
                            "UnsupportedFaceSubtype"
                        } else {
                            "UnsupportedEntityMapping"
                        },
                        "D2",
                        "This declaration is not an admitted simple leaf",
                        part.span,
                        Some(r),
                        c,
                        &[],
                        "admitted simple leaf",
                        &part.name,
                        "profile mapping",
                    ));
                }
            }
            if record.complex {
                if catalog
                    .exclusive
                    .iter()
                    .any(|group| group.iter().filter(|id| bits.contains(**id)).count() > 1)
                {
                    return Err(self.failure(
                        Status::Rejected,
                        "ComplexSubtypeConflict",
                        "D1",
                        "Audited mutually exclusive subtype branches occur together",
                        record.span,
                        Some(r),
                        0,
                        &[],
                        "compatible partial types",
                        "ONEOF conflict",
                        &format!("{} audited subtype groups", self.authority(r)),
                    ));
                }
                if !explicit.windows(2).all(|w| w.first() <= w.get(1)) {
                    return Err(self.failure(
                        Status::Rejected,
                        "ComplexConstituentOrderInvalid",
                        "D1",
                        "External mapping facets must follow ascending type-name order",
                        record.span,
                        Some(r),
                        0,
                        &[],
                        "ascending type names",
                        "source order",
                        "Part21 external mapping",
                    ));
                }
                for (t, d) in catalog.declarations.iter().enumerate() {
                    if bits.contains(t) && !unique.contains(d.name) {
                        return Err(self.failure(
                            Status::Rejected,
                            "MissingComplexConstituent",
                            "D1",
                            "An inherited partial entity is missing",
                            record.span,
                            Some(r),
                            0,
                            &[],
                            d.name,
                            "absent",
                            "Part21 external mapping",
                        ));
                    }
                }
                if !catalog.complex.contains(&explicit.as_slice()) {
                    let expected = match catalog.kind {
                        "Ap203N916Source" => "registered modified N916 source signature",
                        "Ap203e2GeneralSource" => "registered N2635 general source signature",
                        _ if catalog.ap214() => "registered AP214 source signature",
                        _ => "one of six signatures",
                    };
                    return Err(self.failure(
                        Status::Unsupported,
                        "UnsupportedComplexSignature",
                        "D2",
                        "Complete complex mapping is outside the selected source signatures",
                        record.span,
                        Some(r),
                        0,
                        &[],
                        expected,
                        "other signature",
                        "profile mapping",
                    ));
                }
            }
            for (t, _) in catalog.declarations.iter().enumerate() {
                if bits.contains(t) {
                    self.populations.get_mut(t).ok_or_else(internal)?.push(r);
                }
            }
        }
        Ok(())
    }
    fn foreign(&mut self) -> Checked<()> {
        // vertex-loop-v1 explicitly adds N916's verified VERTEX_LOOP transport;
        // it does not admit any borrowed N2635 descriptors below. The separate
        // manifest identity records the addition without rewriting the C++ base.
        const N916: &[&str] = &[
            "DERIVED_UNIT",
            "DERIVED_UNIT_ELEMENT",
            "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",
            "STYLED_ITEM",
            "PRESENTATION_STYLE_ASSIGNMENT",
            "SURFACE_STYLE_USAGE",
            "SURFACE_SIDE_STYLE",
            "SURFACE_STYLE_FILL_AREA",
            "FILL_AREA_STYLE",
            "FILL_AREA_STYLE_COLOUR",
            "COLOUR",
            "COLOUR_SPECIFICATION",
            "COLOUR_RGB",
            "MEASURE_REPRESENTATION_ITEM",
            "DESCRIPTIVE_REPRESENTATION_ITEM",
            "VALUE_REPRESENTATION_ITEM",
            "PRESENTATION_LAYER_ASSIGNMENT",
            "PRE_DEFINED_ITEM",
            "PRE_DEFINED_COLOUR",
            "DRAUGHTING_PRE_DEFINED_COLOUR",
        ];
        let kind = self.schema()?.kind;
        let names = match kind {
            "Ap203N916Source" => N916,
            "Ap214MbxSource" => &[
                "MECHANICAL_CONTEXT",
                "DESIGN_CONTEXT",
                "REAL_REPRESENTATION_ITEM",
            ],
            _ => return Ok(()),
        };
        for name in names {
            for r in self.population(name)? {
                let record = self.record(r)?;
                if *name == "REAL_REPRESENTATION_ITEM" {
                    let failure = self.failure(
                        Status::Unsupported,
                        "UnsupportedEntityType",
                        "D2",
                        "Required content is outside the closed source profile",
                        record.components.first().ok_or_else(internal)?.span,
                        Some(r),
                        0,
                        &[],
                        "registered profile entity",
                        name,
                        "closed type guard; unknown content is not skipped",
                    );
                    if !self.general_interoperability() {
                        return Err(failure);
                    }
                    self.preserve(failure, super::interop::Proof::ForeignRealItem)?;
                    continue;
                }
                if kind == "Ap203N916Source" && !record.components.iter().any(|c| c.name == *name) {
                    continue;
                }
                self.visit(record.span)?;
                let failure = self.failure(Status::Unsupported,"UnsupportedEntityType","D2","Required content is outside the selected source declarations",record.span,Some(r),0,&[],if kind=="Ap203N916Source"{"modified N916 declaration"}else{"AP214E3_2010.exp declaration"},name,if kind=="Ap203N916Source"{"modified N916 source type membership; borrowed transport does not admit an entity"}else{"AP214 source type membership"});
                if !self.general_interoperability()
                    || (kind == "Ap203N916Source" && *name != "PRESENTATION_LAYER_ASSIGNMENT")
                {
                    return Err(failure);
                }
                self.preserve(failure, super::interop::Proof::ExistingMetadata)?;
            }
        }
        Ok(())
    }
    fn mismatch(
        &self,
        value: &Value,
        field: Field,
        r: usize,
        c: usize,
        path: &[usize],
    ) -> Box<Failure> {
        self.failure(
            Status::Rejected,
            "AttributeTypeMismatch",
            "D1",
            "Serialized value does not match the declared field type",
            value.span,
            Some(r),
            c,
            path,
            field.declared_type,
            &format!("{:?}", value.kind),
            &format!("{} explicit attribute", self.authority(r)),
        )
    }
    fn scalar(
        &mut self,
        v: &Value,
        kind: Kind,
        f: Field,
        r: usize,
        c: usize,
        path: &[usize],
    ) -> Checked<()> {
        let catalog = self.schema()?;
        if kind == Kind::String && v.kind == ValueKind::OpaqueString {
            let failure = self.failure(
                Status::Unsupported,
                "OpaqueDataTextUnestablished",
                "D1",
                "DATA text interpretation remains unestablished",
                v.span,
                Some(r),
                c,
                path,
                f.declared_type,
                "OpaqueString",
                "project DATA carrier policy; not established source STRING semantics",
            );
            let issue = &failure.diagnostic;
            let display = (issue.declared_in == "PRODUCT"
                && matches!(issue.field.as_str(), "id" | "name"))
                || (issue.declared_in == "PRESENTATION_LAYER_ASSIGNMENT" && issue.field == "name");
            let identity = issue.field == "id"
                && matches!(
                    issue.declared_in.as_str(),
                    "PRODUCT_DEFINITION_FORMATION" | "PRODUCT_DEFINITION"
                );
            let caption = issue.declared_in == "REPRESENTATION_ITEM"
                && issue.field == "name"
                && self.is_a(r, "MANIFOLD_SOLID_BREP");
            let administrative = (issue.declared_in == "APPROVAL" && issue.field == "level")
                || (issue.declared_in == "PERSON" && issue.field == "id");
            if !self.general_interoperability()
                || !(display || identity || caption || administrative)
                || path.len() != 1
            {
                return Err(failure);
            }
            let proof = if display {
                super::interop::Proof::OpaqueDisplayText
            } else if identity {
                super::interop::Proof::OpaquePartIdentity
            } else if caption {
                super::interop::Proof::OpaqueShapeCaption
            } else {
                super::interop::Proof::OpaqueAdministrativeLabel
            };
            return self.preserve(failure, proof);
        }
        let valid = match kind {
            Kind::String => v.kind == ValueKind::String,
            Kind::Real => v.kind == ValueKind::Real,
            Kind::Integer => v.kind == ValueKind::Integer,
            Kind::Number => matches!(v.kind, ValueKind::Integer | ValueKind::Real),
            Kind::Boolean | Kind::Logical => {
                let valid = v.kind == ValueKind::Enumeration
                    && (v.text == "T" || v.text == "F" || (kind == Kind::Logical && v.text == "U"));
                if !valid {
                    let label = if kind == Kind::Logical {
                        "LOGICAL"
                    } else {
                        "BOOLEAN"
                    };
                    return Err(self.failure(
                        Status::Rejected,
                        if kind == Kind::Logical {
                            "LogicalValueInvalid"
                        } else {
                            "BooleanValueInvalid"
                        },
                        "D1",
                        "Invalid BOOLEAN/LOGICAL value",
                        v.span,
                        Some(r),
                        c,
                        path,
                        if kind == Kind::Logical {
                            "LOGICAL T/F/U"
                        } else {
                            "BOOLEAN T/F"
                        },
                        &self.raw(v.span),
                        &format!("{} {label}", self.authority(r)),
                    ));
                }
                true
            }
            Kind::Enumeration => {
                if v.kind != ValueKind::Enumeration {
                    false
                } else if !catalog.enumeration(f.declared_type, &v.text) {
                    return Err(self.failure(
                        Status::Rejected,
                        "EnumerationValueInvalid",
                        "D1",
                        "Enumeration token is absent from the fixed source type",
                        v.span,
                        Some(r),
                        c,
                        path,
                        f.declared_type,
                        &v.text,
                        &format!("fixed {} enumeration", self.authority(r)),
                    ));
                } else {
                    true
                }
            }
            Kind::Reference => {
                if v.kind == ValueKind::Reference {
                    let target = self.target(v)?;
                    let bits = *self.types.get(target).ok_or_else(internal)?;
                    if !catalog.matches(bits, f.declared_type) {
                        let record = self.record(target)?;
                        let mut failure = self.failure(
                            Status::Rejected,
                            "ReferenceTypeMismatch",
                            "D1",
                            "Reference target does not instantiate the declared type/SELECT",
                            v.span,
                            Some(r),
                            c,
                            path,
                            f.declared_type,
                            &record.components.first().ok_or_else(internal)?.name,
                            &format!("{} explicit reference type", self.authority(r)),
                        );
                        failure.diagnostic.source.target_id.clone_from(&v.text);
                        failure.diagnostic.source.related.push(record.id_span);
                        let approval = catalog.kind == "Ap203e2GeneralSource"
                            && f.declared_type == "APPROVED_ITEM"
                            && self.is_a(r, "CC_DESIGN_APPROVAL")
                            && (self.is_a(target, "PRODUCT_DEFINITION")
                                || self.is_a(target, "PRODUCT_DEFINITION_FORMATION"));
                        let style = catalog.kind == "Ap214MbxSource"
                            && self.scope == super::GeometryScope::GeneralBrep
                            && f.name == "over_ridden_style"
                            && self.is_a(r, "OVER_RIDING_STYLED_ITEM")
                            && self.is_a(target, "CLOSED_SHELL");
                        if !self.interoperability() || (!approval && !style) {
                            return Err(failure);
                        }
                        self.preserve(failure, super::interop::Proof::ExistingMetadata)?;
                    }
                    true
                } else {
                    false
                }
            }
            Kind::Measure => {
                if v.kind != ValueKind::Typed || v.children.len() != 1 {
                    false
                } else {
                    if !catalog.measures.contains(&v.text.as_str()) {
                        return Err(self.failure(
                            Status::Unsupported,
                            "UnsupportedMeasureBranch",
                            "D2",
                            "Measure SELECT branch is outside the source profile",
                            v.span,
                            Some(r),
                            c,
                            path,
                            if catalog.ap214() {
                                "registered typed measure"
                            } else {
                                "typed length measure"
                            },
                            &v.text,
                            "profile measure SELECT guard",
                        ));
                    }
                    let child = v.children.first().ok_or_else(internal)?;
                    if v.text == "DESCRIPTIVE_MEASURE" && catalog.ap214() {
                        if child.kind == ValueKind::OpaqueString {
                            self.scalar(child, Kind::String, f, r, c, path)?;
                        }
                        child.kind == ValueKind::String
                    } else {
                        child.kind == ValueKind::Real
                            || (catalog.kind != "Ap203e2Planar"
                                && matches!(v.text.as_str(), "COUNT_MEASURE" | "NUMERIC_MEASURE")
                                && child.kind == ValueKind::Integer)
                    }
                }
            }
            Kind::SizeSelect | Kind::TrimmingSelect | Kind::MarkerSelect => {
                if v.kind == ValueKind::Reference {
                    let mut reference = f;
                    reference.kind = Kind::Reference;
                    if kind == Kind::SizeSelect {
                        reference.declared_type = "MEASURE_WITH_UNIT";
                    }
                    self.scalar(v, Kind::Reference, reference, r, c, path)?;
                    true
                } else if kind == Kind::MarkerSelect
                    && v.kind == ValueKind::Typed
                    && v.text == "MARKER_TYPE"
                    && v.children.len() == 1
                {
                    let mut marker = f;
                    marker.kind = Kind::Enumeration;
                    marker.declared_type = "MARKER_TYPE";
                    self.scalar(
                        v.children.first().ok_or_else(internal)?,
                        Kind::Enumeration,
                        marker,
                        r,
                        c,
                        path,
                    )?;
                    true
                } else if v.kind == ValueKind::Typed && v.children.len() == 1 {
                    let child = v.children.first().ok_or_else(internal)?;
                    if kind == Kind::TrimmingSelect {
                        v.text == "PARAMETER_VALUE" && child.kind == ValueKind::Real
                    } else {
                        kind == Kind::SizeSelect
                            && ((v.text == "POSITIVE_LENGTH_MEASURE"
                                && child.kind == ValueKind::Real)
                                || (v.text == "DESCRIPTIVE_MEASURE"
                                    && child.kind == ValueKind::String))
                    }
                } else {
                    false
                }
            }
            Kind::Aggregate => return Err(internal()),
        };
        if valid {
            Ok(())
        } else {
            Err(self.mismatch(v, f, r, c, path))
        }
    }
    fn aggregate(
        &mut self,
        v: &Value,
        f: Field,
        r: usize,
        c: usize,
        path: &[usize],
        declared_in: &str,
        depth: usize,
    ) -> Checked<()> {
        if depth > 128 {
            return Err(internal());
        }
        let inner = depth.checked_sub(1).and_then(|i| f.nested.get(i));
        let min = inner.map_or(f.minimum, |d| d.minimum);
        let max = inner.map_or(f.maximum, |d| d.maximum);
        let unique = inner.map_or(f.unique != Unique::None, |d| d.unique);
        let set = inner.map_or(f.unique == Unique::Set, |d| d.set);
        if v.kind != ValueKind::Aggregate {
            return Err(self.failure(
                Status::Rejected,
                "AttributeTypeMismatch",
                "D1",
                "Expected an aggregate value",
                v.span,
                Some(r),
                c,
                path,
                "aggregate",
                &format!("{:?}", v.kind),
                &format!("{} aggregate", self.authority(r)),
            ));
        }
        if v.children.len() < min || max.is_some_and(|m| v.children.len() > m) {
            return Err(self.failure(
                Status::Rejected,
                "AggregateCardinalityViolation",
                "D1",
                "Aggregate cardinality violates the declared bounds",
                v.span,
                Some(r),
                c,
                path,
                "declared bounds",
                &v.children.len().to_string(),
                &format!("{} aggregate", self.authority(r)),
            ));
        }
        let mut members = BTreeMap::new();
        for (i, child) in v.children.iter().enumerate() {
            let mut child_path = path.to_vec();
            child_path.push(i + 1);
            self.visit(child.span)?;
            if depth < f.nested.len() {
                self.aggregate(child, f, r, c, &child_path, declared_in, depth + 1)?;
            } else {
                self.scalar(child, f.element, f, r, c, &child_path)?;
            }
            let mut duplicate = None;
            if unique && f.element == Kind::TrimmingSelect {
                for other in v.children.iter().take(i) {
                    self.visit(other.span)?;
                    if child.kind == other.kind
                        && (if child.kind == ValueKind::Reference {
                            self.target(child)? == self.target(other)?
                        } else {
                            number::compare(
                                &child.children.first().ok_or_else(internal)?.text,
                                &other.children.first().ok_or_else(internal)?.text,
                            ) == 0
                        })
                    {
                        duplicate = Some(other.span);
                        break;
                    }
                }
            } else if unique {
                if let Some(previous) = members.get(child.text.as_str()) {
                    duplicate = Some(*previous);
                } else {
                    members.insert(child.text.as_str(), child.span);
                }
            }
            if let Some(previous) = duplicate {
                let mut failure = self.failure(
                    Status::Rejected,
                    if set {
                        "SetDuplicateMember"
                    } else {
                        "AggregateUniqueViolation"
                    },
                    "D1",
                    "Aggregate repeats an entity identity",
                    child.span,
                    Some(r),
                    c,
                    &child_path,
                    "unique instance identity",
                    &child.text,
                    &format!("{declared_in}.{}", f.name),
                );
                failure.diagnostic.source.related.push(previous);
                return Err(failure);
            }
        }
        Ok(())
    }
    fn fields(&mut self) -> Checked<()> {
        let catalog = self.schema()?;
        self.attributes = self.source.records().iter().map(|_| Vec::new()).collect();
        for (r, record) in self.source.records().iter().enumerate() {
            for (c, part) in record.components.iter().enumerate() {
                let t = catalog.find(&part.name).ok_or_else(internal)?;
                let d = catalog.declarations.get(t).ok_or_else(internal)?;
                let count = if record.complex {
                    d.fields.len()
                } else {
                    d.flattened.len()
                };
                if count != part.parameters.len() {
                    return Err(self.failure(
                        Status::Rejected,
                        "AttributeCountMismatch",
                        "D1",
                        "Explicit attribute slot count does not match this mapping",
                        part.span,
                        Some(r),
                        c,
                        &[],
                        &count.to_string(),
                        &part.parameters.len().to_string(),
                        &format!("{} attribute layout", self.authority(r)),
                    ));
                }
                for (p, v) in part.parameters.iter().enumerate() {
                    let a = self.attribute(r, c, p)?;
                    let binding = if record.complex {
                        (t, p)
                    } else {
                        *d.flattened.get(p).ok_or_else(internal)?
                    };
                    let (f, _, _) = catalog
                        .field(
                            *self.types.get(r).ok_or_else(internal)?,
                            binding.0,
                            binding.1,
                        )
                        .ok_or_else(internal)?;
                    self.visit(v.span)?;
                    self.attributes
                        .get_mut(r)
                        .ok_or_else(internal)?
                        .push(a.clone());
                    let path = [p + 1];
                    if a.derived_slot {
                        if v.kind != ValueKind::Derived {
                            let failure = self.failure(
                                Status::Rejected,
                                "DerivedSlotMarkerRequired",
                                "D1",
                                "Redeclared explicit slot must contain '*'",
                                v.span,
                                Some(r),
                                c,
                                &path,
                                "*",
                                &self.raw(v.span),
                                "explicit-to-derived redeclaration",
                            );
                            if self.development_dimensions_candidate(r, &a, v) {
                                self.defer_development_dimensions(r, *failure)?;
                                self.scalar(v, Kind::Reference, f, r, c, &path)?;
                                continue;
                            }
                            if !self.interoperability()
                                || !catalog.ap214()
                                || f.name != "dimensions"
                                || v.kind != ValueKind::Reference
                                || !self.is_a(r, "CONVERSION_BASED_UNIT")
                                || !self.is_a(r, "MASS_UNIT")
                            {
                                return Err(failure);
                            }
                            self.scalar(v, Kind::Reference, f, r, c, &path)?;
                            self.preserve(failure, super::interop::Proof::ExistingMetadata)?;
                        }
                        continue;
                    }
                    if v.kind == ValueKind::Unset {
                        if f.optional {
                            continue;
                        }
                        let failure = self.failure(
                            Status::Rejected,
                            "RequiredAttributeMissing",
                            "D1",
                            "A required attribute is missing; empty string is a distinct value",
                            v.span,
                            Some(r),
                            c,
                            &path,
                            f.declared_type,
                            "$",
                            &format!("{} required attribute", self.authority(r)),
                        );
                        let label = f.name == "name"
                            && f.kind == Kind::String
                            && (self.is_a(r, "REPRESENTATION_ITEM")
                                || self.is_a(r, "REPRESENTATION")
                                || (self.non_geometric_style(r)
                                    && !self.is_a(r, "PRE_DEFINED_ITEM")));
                        let rendering = f.name == "rendering_method"
                            && f.kind == Kind::Enumeration
                            && self.is_a(r, "SURFACE_STYLE_RENDERING");
                        if !self.interoperability() || (!label && !rendering) {
                            return Err(failure);
                        }
                        self.preserve(failure, super::interop::Proof::ExistingMetadata)?;
                        continue;
                    }
                    if f.kind == Kind::Aggregate {
                        self.aggregate(v, f, r, c, &path, a.declared_in, 0)?;
                    } else if let Err(failure) = self.scalar(v, f.kind, f, r, c, &path) {
                        if !self.general_interoperability()
                            || !catalog.ap214()
                            || !self.is_a(r, "CURVE_STYLE")
                            || f.name != "curve_width"
                            || f.kind != Kind::SizeSelect
                            || v.kind != ValueKind::String
                            || failure.diagnostic.source.key != "AttributeTypeMismatch"
                        {
                            return Err(failure);
                        }
                        self.preserve(failure, super::interop::Proof::CurveStyleWidth)?;
                    }
                }
            }
        }
        Ok(())
    }
    fn context(&mut self, id: &str) -> Checked<()> {
        let Some(r) = self
            .source
            .data
            .index
            .get(id)
            .copied()
            .filter(|r| self.is_a(*r, "GEOMETRIC_REPRESENTATION_CONTEXT"))
        else {
            return Err(simple_failure(
                Status::InvalidRequest,
                "InvalidPhysicalContextSelection",
                "InvalidRequest",
                "Explicit context ID must identify a typed geometric representation context",
            ));
        };
        if !self.is_a(r, "GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            return Err(self.failure(
                Status::Unsupported,
                "UnsupportedPhysicalUnitContext",
                "D2",
                "Selected physical representation requires explicit unit assignment",
                self.record(r)?.span,
                Some(r),
                0,
                &[],
                "",
                "",
                "project physical unit guard; no defaults",
            ));
        }
        let (_, dimension) = self.field(r, "coordinate_space_dimension")?;
        if number::compare(&dimension.text, "0") <= 0 {
            return Err(self.field_failure(
                Status::Rejected,
                "SourceRuleViolation:DIMENSION_COUNT.WR1",
                "D3",
                r,
                "coordinate_space_dimension",
                ">0",
                &dimension.text,
                "DIMENSION_COUNT.WR1",
            ));
        }
        if number::index(&dimension.text, 3) != Some(3) {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedCoordinateDimension",
                "D2",
                r,
                "coordinate_space_dimension",
                "3",
                &dimension.text,
                "profile dimension guard",
            ));
        }
        let (_, assignments) = self.field(r, "units")?;
        let mut lengths = Vec::new();
        let mut angles = Vec::new();
        let mut solids = Vec::new();
        for unit in &assignments.children {
            let target = self.target(unit)?;
            if self.is_a(target, "LENGTH_UNIT") {
                lengths.push(target);
            } else if self.is_a(target, "PLANE_ANGLE_UNIT") {
                angles.push(target);
            } else if self.is_a(target, "SOLID_ANGLE_UNIT") {
                solids.push(target);
            }
        }
        if lengths.len() != 1 || angles.len() != 1 || solids.len() > 1 {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedUnitAssignment",
                "D2",
                r,
                "units",
                "length=1 angle=1 solid-angle<=1",
                "different nominal quantities/counts",
                "profile unit guard; no defaults",
            ));
        }
        self.units.context = Some(r);
        self.units.length_unit = lengths.first().copied();
        self.units.plane_angle_unit = angles.first().copied();
        self.units.solid_angle_unit = solids.first().copied();
        for u in self.population("SI_UNIT")? {
            self.visit(self.record(u)?.span)?;
            let (_, name) = self.field(u, "name")?;
            let (_, prefix) = self.field(u, "prefix")?;
            let length = self.is_a(u, "LENGTH_UNIT");
            let angle = self.is_a(u, "PLANE_ANGLE_UNIT");
            let mass = self.is_a(u, "MASS_UNIT");
            let quantity = if length {
                "Length"
            } else if angle {
                "PlaneAngle"
            } else if mass {
                "Mass"
            } else {
                "SolidAngle"
            };
            let expected = if length {
                "METRE"
            } else if angle {
                "RADIAN"
            } else if mass {
                "GRAM"
            } else {
                "STERADIAN"
            };
            if name.text != expected {
                return Err(self.field_failure(
                    Status::Unsupported,
                    "UnsupportedSiUnitKind",
                    "D2",
                    u,
                    "name",
                    expected,
                    &name.text,
                    "profile nominal SI-unit guard",
                ));
            }
            let prefix = if prefix.kind == ValueKind::Unset {
                ""
            } else {
                prefix.text.as_str()
            };
            let power = self
                .schema()?
                .prefixes
                .iter()
                .find(|p| p.0 == prefix)
                .ok_or_else(internal)?
                .1
                + if length {
                    3
                } else if mass {
                    -3
                } else {
                    0
                };
            let mut dimensions = [0; 7];
            if length {
                if let Some(d) = dimensions.get_mut(0) {
                    *d = 1;
                }
            }
            if mass {
                if let Some(d) = dimensions.get_mut(1) {
                    *d = 1;
                }
            }
            if Some(u) == self.units.length_unit {
                self.units.length_power10_mm = Some(power);
            }
            self.units.si_definitions.push(SiUnit {
                record_index: u,
                quantity: quantity.into(),
                si_name: name.text.clone(),
                prefix: prefix.into(),
                dimensions,
                canonical_power10: power,
            });
        }
        Ok(())
    }
    fn run(&mut self, options: &Options) -> Checked<()> {
        let result = self.binding(options);
        self.outcome.binding = progress(&result);
        result?;
        if options.development_unit_policy != super::DevelopmentUnitPolicy::Disabled
            && !self.schema()?.ap214()
        {
            return Err(simple_failure(
                Status::Unsupported,
                "DevelopmentUnitPolicyRequiresAp214",
                "ProfileUnsupported",
                "Explicit angle-dimensions development policy applies only to AP214 source catalogs",
            ));
        }
        let result = self
            .mapping()
            .and_then(|()| self.foreign())
            .and_then(|()| self.fields());
        self.outcome.types = progress(&result);
        result?;
        if self.schema()?.kind != "Ap203e2Planar" {
            let result = super::units::check(self);
            self.outcome.unit_dependencies = progress(&result);
            result?;
            if options.check_geometry_dependencies {
                let result = super::geometry::check(self);
                self.outcome.geometry_dependencies = progress(&result);
                result?;
            }
        }
        let mut selected_context = None;
        if options.check_part_and_closure {
            let result = super::ownership::check(self, options.geometry_scope);
            self.outcome.part = progress(&result);
            selected_context = Some(result?);
            if self
                .part
                .as_ref()
                .is_some_and(|p| !p.deferred_region_faces.is_empty())
            {
                self.outcome.part = "Deferred";
            }
        }
        if let Some(context) = selected_context.as_ref() {
            if options
                .physical_context
                .as_ref()
                .is_some_and(|requested| requested != context)
            {
                return Err(simple_failure(
                    Status::InvalidRequest,
                    "PhysicalContextSelectionMismatch",
                    "InvalidRequest",
                    "Explicit context differs from the proved selected-part context",
                ));
            }
        }
        if let Some(context) = selected_context
            .as_ref()
            .or(options.physical_context.as_ref())
        {
            let result = self.context(context);
            self.outcome.si_units = progress(&result);
            result?;
            if self.units.length_power10_mm.is_none() {
                self.outcome.si_units = "Deferred";
            }
        }
        if self.interoperability() {
            let result = self.approve_preserved();
            self.outcome.interoperability = progress(&result);
            result?;
        }
        self.publish_development_units()?;
        Ok(())
    }
}
fn progress<T>(result: &Checked<T>) -> &'static str {
    match result {
        Ok(_) => "Passed",
        Err(e) => match e.status {
            Status::Rejected => "Rejected",
            Status::Unsupported => "Unsupported",
            Status::ResourceLimit => "ResourceLimit",
            Status::InvalidRequest => "InvalidRequest",
            _ => "InfrastructureFailure",
        },
    }
}
pub(super) fn check(source: &RawOutcome, limits: Limits, options: &Options) -> Outcome {
    let mut effective = options.clone();
    // Original Check always executes the full source pipeline for this policy;
    // no shortened checkpoint may publish provisionally preserved source errors.
    if options.source_policy == super::SourcePolicy::PreserveNonGeometric
        || options.development_unit_policy != super::DevelopmentUnitPolicy::Disabled
    {
        effective.check_geometry_dependencies = true;
        effective.check_part_and_closure = true;
    }
    let options = &effective;
    let mut outcome = Outcome {
        development_units: super::development_units::Qualification {
            requested: options.development_unit_policy,
            ..Default::default()
        },
        qualification: super::interop::Qualification::new(options),
        source_capture: source.captured.clone(),
        check_part_and_closure: options.check_part_and_closure,
        check_geometry_dependencies: options.check_geometry_dependencies,
        status: Status::SourceFailure,
        snapshot: None,
        schema_selection: None,
        binding: "NotRun",
        types: "NotRun",
        unit_dependencies: "NotRun",
        geometry_dependencies: "NotRun",
        geometry_dependency_checks: super::GeometryChecks::default(),
        si_units: "NotRun",
        part: "NotRun",
        interoperability: "NotRun",
        property_attachment_checks: 0,
        dependency_visits: 0,
        unit_dependency_checks: UnitChecks::default(),
        limits,
        diagnostics: Vec::new(),
    };
    if options.development_unit_policy != super::DevelopmentUnitPolicy::Disabled
        && options.geometry_scope != super::GeometryScope::GeneralBrep
    {
        outcome.status = Status::InvalidRequest;
        outcome.diagnostics.push(simple_failure(
            Status::InvalidRequest, "DevelopmentUnitsRequireGeneralSource", "InvalidRequest",
            "Development angle-unit policy requires explicitly selected GeneralBrep source checks",
        ).diagnostic);
        return outcome;
    }
    // STEP_SEMANTICS::Check requires explicit general interoperability even for
    // an enabled-but-unused legacy option. Consult the immutable owner, never
    // resettable notices; shortened strict checkpoints cannot acquire approval.
    let general_interop = options.source_policy == super::SourcePolicy::PreserveNonGeometric
        && options.geometry_scope == super::GeometryScope::GeneralBrep;
    if options.schema_interpretation != super::SchemaInterpretation::DeclaredOnly
        && !general_interop
    {
        outcome.status = Status::InvalidRequest;
        outcome.diagnostics.push(
            simple_failure(
                Status::InvalidRequest,
                "UnverifiedSchemaRequiresInteroperability",
                "InvalidRequest",
                "Unverified schema interpretation requires explicit general interoperability",
            )
            .diagnostic,
        );
        return outcome;
    }
    if (source.options().carrier_interpretation != crate::CarrierInterpretation::DeclaredOnly
        || source
            .source
            .as_ref()
            .is_some_and(|s| s.carrier_assumption().is_some()))
        && !general_interop
    {
        outcome.status = Status::InvalidRequest;
        outcome.diagnostics.push(
            simple_failure(
                Status::InvalidRequest,
                "UnverifiedCarrierRequiresInteroperability",
                "InvalidRequest",
                "Unverified carrier interpretation requires explicit general interoperability",
            )
            .diagnostic,
        );
        return outcome;
    }
    let Some(raw) = source
        .source
        .as_ref()
        .filter(|_| source.status == crate::Status::Inspected)
    else {
        for d in &source.diagnostics {
            let mut wrapped = simple_failure(
                Status::SourceFailure,
                &d.key,
                "SourceInspectionFailure",
                &d.message,
            )
            .diagnostic;
            wrapped.source = d.clone();
            wrapped.responsibility = "ticket 02 syntax/index/reference check".into();
            outcome.diagnostics.push(wrapped);
        }
        return outcome;
    };
    if limits.dependency_visits > 10_000_000 {
        outcome.status = Status::ResourceLimit;
        outcome.diagnostics.push(
            simple_failure(
                Status::ResourceLimit,
                "InvalidSemanticLimits",
                "ResourceLimit",
                "Semantic visit budget exceeds implementation ceiling",
            )
            .diagnostic,
        );
        return outcome;
    }
    let mut checker = Checker {
        source: raw,
        catalog: None,
        profile: None,
        types: Vec::new(),
        populations: Vec::new(),
        attributes: Vec::new(),
        units: Units::default(),
        part: None,
        representation_use: None,
        scope: options.geometry_scope,
        pending: Vec::new(),
        ownership_evidence: None,
        development_units: super::development_units::Working::default(),
        outcome,
    };
    let result = checker.run(options);
    checker.publish_qualification_progress();
    match result {
        Ok(()) => {
            checker.outcome.status = if (options.physical_context.is_some()
                || options.check_part_and_closure)
                && checker.units.length_power10_mm.is_none()
                || checker.outcome.part == "Deferred"
            {
                Status::Deferred
            } else {
                Status::Checked
            };
            if checker.outcome.status == Status::Checked
                && (!checker.outcome.preserved_diagnostics().is_empty()
                    || !checker.outcome.interpretation_diagnostics().is_empty()
                    || checker.outcome.development_units.receipt.is_some()
                    || raw.carrier_assumption().is_some())
            {
                checker.outcome.status = Status::CheckedWithDiagnostics;
            }
            if let (Some(catalog), Some(profile)) = (checker.catalog, checker.profile.take()) {
                checker.outcome.snapshot = Some(Snapshot {
                    data: Arc::new(Data {
                        development_units: checker.outcome.development_units.clone(),
                        qualification: checker.outcome.qualification.clone(),
                        source: raw.clone(),
                        catalog,
                        profile,
                        types: checker.types,
                        populations: checker.populations,
                        attributes: checker.attributes,
                        units: checker.units,
                        part: checker.part,
                        representation_use: checker.representation_use,
                    }),
                });
            } else {
                checker.outcome.status = Status::InfrastructureFailure;
                checker.outcome.diagnostics.push(internal().diagnostic);
            }
        }
        Err(failure) => {
            checker.outcome.status = failure.status;
            if !checker.outcome.development_unit_diagnostics().is_empty() {
                // Provisional development checks never erase the original D1
                // source error, including on a later unit/resource/owner failure.
                let original = checker.outcome.development_unit_diagnostics().to_vec();
                checker.outcome.diagnostics.extend(original);
            }
            checker.outcome.diagnostics.push(failure.diagnostic);
        }
    }
    checker.outcome
}
