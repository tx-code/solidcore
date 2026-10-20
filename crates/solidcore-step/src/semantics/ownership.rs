// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Strict selected-part identities and source dependency closure.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemantics.cpp`, Product/Closure/Reach.
//! Rust Ownership corresponds to those C++ phase methods. Deviations: checked
//! source indices, explicit iterative traversal and provisional interop evidence.
//! Geometry dependency qualification and native construction remain separate.

mod metadata;
mod presentation;

use super::checker::{internal, Checked, Checker, Failure};
use super::interop::{OwnershipEvidence, Proof};
use super::{GeometryScope, Part, PropertyAttachment, Role, Status};
use crate::{Span, Value};
use std::collections::BTreeSet;

struct Selected {
    definition: usize,
    body: usize,
    relationship: Option<usize>,
    wireframe: Option<usize>,
    wireframe_relationship: Option<usize>,
    construction: Option<usize>,
    construction_relationship: Option<usize>,
}
struct Ownership<'c, 's> {
    c: &'c mut Checker<'s>,
    part: Part,
    scope: GeometryScope,
    representation: Option<usize>,
    context: Option<usize>,
    wireframe: Option<usize>,
    construction: Option<usize>,
    outgoing: Vec<Vec<usize>>,
    presentation_roots: Vec<usize>,
    representation_use: Vec<bool>,
    administrative: Vec<bool>,
    style_overrides: BTreeSet<usize>,
}
impl<'s> Ownership<'_, 's> {
    fn field(&self, r: usize, name: &str) -> Checked<&'s Value> {
        Ok(self.c.field(r, name)?.1)
    }
    fn target_field(&self, r: usize, name: &str) -> Checked<usize> {
        self.c.target(self.field(r, name)?)
    }
    fn is_a(&self, r: usize, name: &str) -> bool {
        self.c.is_a(r, name)
    }
    fn population(&self, name: &str) -> Checked<Vec<usize>> {
        self.c.population(name)
    }
    fn role(&mut self, name: impl Into<String>, r: usize) {
        self.part.roles.push(Role {
            name: name.into(),
            record_index: r,
        });
    }
    fn rep(&self) -> Checked<usize> {
        self.representation.ok_or_else(internal)
    }
    fn ap214(&self) -> bool {
        self.c.catalog.is_some_and(super::catalog::Catalog::ap214)
    }
    fn planar(&self) -> bool {
        self.c.catalog.is_some_and(|c| c.kind == "Ap203e2Planar")
    }
    fn kind(&self) -> Checked<&'static str> {
        Ok(self.c.schema()?.kind)
    }
    fn record_failure(&self, key: &str, r: usize, responsibility: &str) -> Box<Failure> {
        match self.c.record(r) {
            Ok(record) => self.c.failure(
                Status::Unsupported,
                key,
                "D2",
                "Source ownership guard failed",
                record.span,
                Some(r),
                0,
                &[],
                "",
                "",
                responsibility,
            ),
            Err(e) => e,
        }
    }
    fn field_failure(
        &self,
        status: Status,
        key: &str,
        r: usize,
        name: &str,
        at: Option<Span>,
        path: bool,
        expected: &str,
        actual: &str,
        responsibility: &str,
    ) -> Box<Failure> {
        match self.c.field(r, name) {
            Ok((a, v)) => self.c.failure(
                status,
                key,
                if status == Status::Rejected {
                    "D3"
                } else {
                    "D2"
                },
                "Source ownership or prerequisite check failed",
                at.unwrap_or(v.span),
                Some(r),
                a.component,
                &if path {
                    vec![a.parameter + 1]
                } else {
                    Vec::new()
                },
                expected,
                actual,
                responsibility,
            ),
            Err(e) => e,
        }
    }
    fn one(&self, name: &str) -> Checked<usize> {
        let members = self.population(name)?;
        if members.len() == 1 {
            return members.first().copied().ok_or_else(internal);
        }
        let last = members.last().copied();
        let span = match last {
            Some(r) => self.c.record(r)?.id_span,
            None => self.c.source.header().get(2).ok_or_else(internal)?.span,
        };
        let mut actual = format!("{} ", members.len());
        for r in &members {
            actual.push('#');
            actual.push_str(&self.c.record(*r)?.id);
            actual.push(' ');
        }
        let mut failure = self.c.failure(
            Status::Unsupported,
            "UnsupportedSinglePartGraph",
            "D2",
            "The selected profile requires a unique product/representation chain",
            span,
            last,
            0,
            &[],
            &format!("{name} count=1"),
            &actual,
            "single-part profile guard; not PRODUCT.id uniqueness",
        );
        if members.len() > 1 {
            failure.diagnostic.source.related.push(
                self.c
                    .record(*members.first().ok_or_else(internal)?)?
                    .id_span,
            );
        }
        Err(failure)
    }
    fn link(&self, owner: usize, name: &str, target: usize, member: Option<usize>) -> Checked<()> {
        let value = self.field(owner, name)?;
        let field = if let Some(i) = member {
            value.children.get(i).ok_or_else(internal)?
        } else {
            value
        };
        if self.c.target(field)? != target {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedSinglePartGraph",
                owner,
                name,
                Some(field.span),
                false,
                &self.c.record(target)?.id,
                &field.text,
                "single-part identity guard",
            ));
        }
        Ok(())
    }
    fn plain_representation(&self, r: usize) -> Checked<bool> {
        let record = self.c.record(r)?;
        Ok(record.components.len() == 1
            && record
                .components
                .first()
                .is_some_and(|c| c.name == "REPRESENTATION"))
    }
    fn body(&self, r: usize) -> bool {
        self.is_a(r, "ADVANCED_BREP_SHAPE_REPRESENTATION")
            || self.is_a(r, "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
    }
    fn construction(&mut self, selected: &mut Selected) -> Checked<()> {
        if !self.ap214() {
            return Ok(());
        }
        let reps = self.population("CONSTRUCTIVE_GEOMETRY_REPRESENTATION")?;
        let links = self.population("CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP")?;
        if reps.is_empty() && links.is_empty() {
            return Ok(());
        }
        if self.scope != GeometryScope::GeneralBrep || reps.len() != 1 || links.len() != 1 {
            return Err(self.record_failure(
                "UnsupportedConstructionGeometryGraph",
                selected.definition,
                "project single selected supporting representation guard",
            ));
        }
        let rep = *reps.first().ok_or_else(internal)?;
        let link = *links.first().ok_or_else(internal)?;
        let first = self.target_field(link, "rep_1")?;
        let second = self.target_field(link, "rep_2")?;
        let first_context = self.target_field(first, "context_of_items")?;
        let second_context = self.target_field(second, "context_of_items")?;
        self.c.visit(self.c.record(link)?.span)?;
        for (bad, field, clause, expected) in [
            (
                first_context != second_context
                    || !self.is_a(first_context, "GEOMETRIC_REPRESENTATION_CONTEXT"),
                "rep_2",
                "WR1",
                "same GRC identity",
            ),
            (
                !self.is_a(second, "CONSTRUCTIVE_GEOMETRY_REPRESENTATION"),
                "rep_2",
                "WR2",
                "CGR",
            ),
            (
                usize::from(self.is_a(first, "SHAPE_REPRESENTATION"))
                    + usize::from(self.is_a(first, "CONSTRUCTIVE_GEOMETRY_REPRESENTATION"))
                    != 1,
                "rep_1",
                "WR3",
                "one SHAPE_REPRESENTATION/CGR category",
            ),
        ] {
            if bad {
                let clause = format!("CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP.{clause}");
                return Err(self.field_failure(
                    Status::Rejected,
                    &format!("SourceRuleViolation:{clause}"),
                    link,
                    field,
                    None,
                    true,
                    expected,
                    &self.field(link, field)?.text,
                    &format!("AP214E3_2010.exp {clause}"),
                ));
            }
        }
        if second != rep
            || (first != selected.definition
                && first != selected.body
                && selected.wireframe != Some(first))
        {
            return Err(self.record_failure(
                "UnsupportedConstructionGeometryGraph",
                link,
                "project single selected supporting representation guard",
            ));
        }
        let context = self.target_field(rep, "context_of_items")?;
        let dimension = self.field(context, "coordinate_space_dimension")?;
        self.c.visit(dimension.span)?;
        if super::number::compare(&dimension.text, "2") < 0
            || super::number::compare(&dimension.text, "3") > 0
        {
            return Err(self.field_failure(
                Status::Rejected,
                "SourceRuleViolation:CONSTRUCTIVE_GEOMETRY_REPRESENTATION.WR1",
                context,
                "coordinate_space_dimension",
                None,
                true,
                "2 <= dimension <= 3",
                &dimension.text,
                "AP214E3_2010.exp CONSTRUCTIVE_GEOMETRY_REPRESENTATION.WR1",
            ));
        }
        for item in &self.field(rep, "items")?.children {
            self.c.visit(item.span)?;
            let target = self.c.target(item)?;
            let count = [
                "PLACEMENT",
                "CURVE",
                "EDGE",
                "FACE",
                "POINT",
                "SURFACE",
                "FACE_SURFACE",
                "VERTEX_POINT",
            ]
            .iter()
            .filter(|t| self.is_a(target, t))
            .count();
            if count != 1 {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:CONSTRUCTIVE_GEOMETRY_REPRESENTATION.WR2",
                    rep,
                    "items",
                    Some(item.span),
                    true,
                    "one source category",
                    &item.text,
                    "AP214E3_2010.exp CONSTRUCTIVE_GEOMETRY_REPRESENTATION.WR2",
                ));
            }
        }
        if context != self.target_field(selected.body, "context_of_items")? {
            return Err(self.record_failure(
                "UnsupportedConstructionGeometryGraph",
                rep,
                "project single selected supporting representation guard",
            ));
        }
        selected.construction = Some(rep);
        selected.construction_relationship = Some(link);
        Ok(())
    }
    fn select_representations(&mut self, sdr: usize) -> Checked<Selected> {
        let root = self.target_field(sdr, "used_representation")?;
        let mut selected = Selected {
            definition: root,
            body: root,
            relationship: None,
            wireframe: None,
            wireframe_relationship: None,
            construction: None,
            construction_relationship: None,
        };
        let relationships = self.population("SHAPE_REPRESENTATION_RELATIONSHIP")?;
        let wireframes = self.population("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION")?;
        let error = |s: &Self, r| {
            s.record_failure(
                "UnsupportedRepresentationGraph",
                r,
                "profile shape/presentation graph guard",
            )
        };
        if !wireframes.is_empty() {
            if self.scope != GeometryScope::GeneralBrep
                || wireframes.len() != 1
                || wireframes.first() == Some(&root)
                || !self.is_a(root, "SHAPE_REPRESENTATION")
                || relationships.len() != if self.body(root) { 1 } else { 2 }
            {
                return Err(error(self, root));
            }
            selected.wireframe = wireframes.first().copied();
            for link in &relationships {
                self.c.visit(self.c.record(*link)?.span)?;
                let a = self.target_field(*link, "rep_1")?;
                let b = self.target_field(*link, "rep_2")?;
                if (a == root) == (b == root) {
                    return Err(error(self, *link));
                }
                let other = if a == root { b } else { a };
                if Some(other) == selected.wireframe && selected.wireframe_relationship.is_none() {
                    selected.wireframe_relationship = Some(*link);
                } else if self.body(other) && !self.body(root) && selected.relationship.is_none() {
                    selected.body = other;
                    selected.relationship = Some(*link);
                } else {
                    return Err(error(self, *link));
                }
            }
            if selected.wireframe_relationship.is_none() || !self.body(selected.body) {
                return Err(error(self, root));
            }
            let context = self.target_field(root, "context_of_items")?;
            if self.target_field(selected.body, "context_of_items")? != context
                || self
                    .target_field(selected.wireframe.ok_or_else(internal)?, "context_of_items")?
                    != context
            {
                return Err(error(self, root));
            }
            if root != selected.body {
                for item in &self.field(root, "items")?.children {
                    self.c.visit(item.span)?;
                    if !self.is_a(self.c.target(item)?, "AXIS2_PLACEMENT_3D") {
                        return Err(error(self, root));
                    }
                }
            }
        } else if self.body(root) {
            if let Some(link) = relationships.first() {
                return Err(error(self, *link));
            }
        } else {
            if !self.is_a(root, "SHAPE_REPRESENTATION") || relationships.len() != 1 {
                return Err(error(self, root));
            }
            let link = *relationships.first().ok_or_else(internal)?;
            self.c.visit(self.c.record(link)?.span)?;
            let a = self.target_field(link, "rep_1")?;
            let b = self.target_field(link, "rep_2")?;
            selected.body = if a == root && b != root {
                b
            } else if b == root && a != root {
                a
            } else {
                return Err(error(self, link));
            };
            if !self.body(selected.body)
                || self.target_field(root, "context_of_items")?
                    != self.target_field(selected.body, "context_of_items")?
            {
                return Err(error(self, link));
            }
            for item in &self.field(root, "items")?.children {
                self.c.visit(item.span)?;
                if !self.is_a(self.c.target(item)?, "AXIS2_PLACEMENT_3D") {
                    return Err(error(self, root));
                }
            }
            selected.relationship = Some(link);
        }
        self.construction(&mut selected)?;
        for rep in self.population("REPRESENTATION")? {
            self.c.visit(self.c.record(rep)?.span)?;
            if rep != selected.definition
                && rep != selected.body
                && selected.wireframe != Some(rep)
                && selected.construction != Some(rep)
                && !self.is_a(
                    rep,
                    "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",
                )
                && !self.plain_representation(rep)?
            {
                return Err(error(self, rep));
            }
        }
        Ok(selected)
    }
    fn product(&mut self) -> Checked<()> {
        // The strict chain is selected by source identity, never by an APD year,
        // duplicate textual product ID, representation ordering or display name.
        let (single_application, mut distinct_issue) = match self.one("APPLICATION_CONTEXT") {
            Ok(r) => (Some(r), None),
            Err(failure) => {
                if !self.c.general_interoperability()
                    || self.population("APPLICATION_CONTEXT")?.len() < 2
                {
                    return Err(failure);
                }
                (None, Some(failure))
            }
        };
        let mut apd = None;
        let mut definition_protocol = None;
        match self.one("APPLICATION_PROTOCOL_DEFINITION") {
            Ok(r) => apd = Some(r),
            Err(failure) => {
                let protocols = self.population("APPLICATION_PROTOCOL_DEFINITION")?;
                if self.c.general_interoperability() && protocols.len() > 1 {
                    let pc = self.one("PRODUCT_CONTEXT")?;
                    let pdc = self.one("PRODUCT_DEFINITION_CONTEXT")?;
                    let product_application = self.target_field(pc, "frame_of_reference")?;
                    let definition_application = self.target_field(pdc, "frame_of_reference")?;
                    for protocol in protocols {
                        let application = self.target_field(protocol, "application")?;
                        if application == product_application {
                            if apd.is_some() {
                                return Err(failure);
                            }
                            apd = Some(protocol);
                        }
                        if application == definition_application {
                            if definition_protocol.is_some() {
                                return Err(failure);
                            }
                            definition_protocol = Some(protocol);
                        }
                        if application != product_application
                            && application != definition_application
                        {
                            return Err(failure);
                        }
                    }
                    if apd.is_none() || definition_protocol.is_none() {
                        return Err(failure);
                    }
                    self.c
                        .preserve(failure, Proof::ContextApplicationProtocols)?;
                } else {
                    if !self.c.can_preserve_missing_protocol()
                        || !self.c.general_interoperability()
                        || !protocols.is_empty()
                    {
                        return Err(failure);
                    }
                    self.part.application_protocol_present = false;
                    self.c
                        .preserve(failure, Proof::MissingApplicationProtocol)?;
                }
            }
        }
        let pc = self.one("PRODUCT_CONTEXT")?;
        let pdc = self.one("PRODUCT_DEFINITION_CONTEXT")?;
        let product = self.one("PRODUCT")?;
        let formation = self.one("PRODUCT_DEFINITION_FORMATION")?;
        let pd = self.one("PRODUCT_DEFINITION")?;
        let pds = self.one("PRODUCT_DEFINITION_SHAPE")?;
        let sdr = self.one("SHAPE_DEFINITION_REPRESENTATION")?;
        let distinct = distinct_issue.is_some();
        let product_application = if distinct {
            self.target_field(pc, "frame_of_reference")?
        } else {
            single_application.ok_or_else(internal)?
        };
        let definition_application = if distinct {
            self.target_field(pdc, "frame_of_reference")?
        } else {
            single_application.ok_or_else(internal)?
        };
        let protocol_application = if distinct {
            match apd {
                Some(apd) => self.target_field(apd, "application")?,
                None => product_application,
            }
        } else {
            product_application
        };
        let ac = single_application.unwrap_or(protocol_application);
        if let Some(failure) = distinct_issue.take() {
            let selected = [
                product_application,
                definition_application,
                protocol_application,
            ];
            if self
                .population("APPLICATION_CONTEXT")?
                .iter()
                .any(|r| !selected.contains(r))
            {
                return Err(failure);
            }
            self.c
                .preserve(failure, Proof::DistinctApplicationContexts)?;
        }
        let selected = if self.planar() {
            let rep = self.one("REPRESENTATION")?;
            self.context = Some(self.one("REPRESENTATION_CONTEXT")?);
            Selected {
                definition: rep,
                body: rep,
                relationship: None,
                wireframe: None,
                wireframe_relationship: None,
                construction: None,
                construction_relationship: None,
            }
        } else {
            let selected = self.select_representations(sdr)?;
            self.context = Some(self.target_field(selected.body, "context_of_items")?);
            selected
        };
        self.representation = Some(selected.body);
        self.wireframe = selected.wireframe;
        self.construction = selected.construction;
        for (name, r) in [
            ("application_context", ac),
            ("product_context", pc),
            ("definition_context", pdc),
            ("product", product),
            ("formation", formation),
            ("definition", pd),
            ("product_shape", pds),
            ("shape_definition_representation", sdr),
            ("representation", selected.body),
            ("unit_context", self.context.ok_or_else(internal)?),
        ] {
            self.role(name, r);
        }
        if distinct {
            self.role("product_application_context", product_application);
            self.role("definition_application_context", definition_application);
        }
        if let Some(apd) = apd {
            self.part.roles.insert(
                1,
                Role {
                    name: "application_protocol".into(),
                    record_index: apd,
                },
            );
        }
        if let Some(protocol) = definition_protocol.filter(|r| Some(*r) != apd) {
            self.role("definition_application_protocol", protocol);
        }
        if let Some(link) = selected.relationship {
            self.role("definition_representation", selected.definition);
            self.role("shape_representation_relationship", link);
        }
        if let Some(rep) = selected.wireframe {
            self.role("wireframe_representation", rep);
            self.role(
                "wireframe_shape_relationship",
                selected.wireframe_relationship.ok_or_else(internal)?,
            );
        }
        if let Some(rep) = selected.construction {
            self.role("construction_representation", rep);
            self.role(
                "construction_relationship",
                selected.construction_relationship.ok_or_else(internal)?,
            );
        }
        if let Some(apd) = apd {
            self.link(
                apd,
                "application",
                if distinct { protocol_application } else { ac },
                None,
            )?;
        }
        self.link(
            pc,
            "frame_of_reference",
            if distinct { product_application } else { ac },
            None,
        )?;
        self.link(
            pdc,
            "frame_of_reference",
            if distinct { definition_application } else { ac },
            None,
        )?;
        if self.field(product, "frame_of_reference")?.children.len() != 1 {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedSinglePartGraph",
                product,
                "frame_of_reference",
                None,
                false,
                "",
                "",
                "",
            ));
        }
        self.link(product, "frame_of_reference", pc, Some(0))?;
        for (owner, field, target) in [
            (formation, "of_product", product),
            (pd, "formation", formation),
            (pd, "frame_of_reference", pdc),
            (pds, "definition", pd),
            (sdr, "definition", pds),
            (sdr, "used_representation", selected.definition),
            (
                selected.body,
                "context_of_items",
                self.context.ok_or_else(internal)?,
            ),
        ] {
            self.link(owner, field, target, None)?;
        }
        if let Some(apd) = apd {
            self.part.apd_status = self.field(apd, "status")?.text.clone();
            self.part.apd_schema_claim = self
                .field(apd, "application_interpreted_model_schema_name")?
                .text
                .clone();
            self.part.apd_year = self.field(apd, "application_protocol_year")?.text.clone();
        }
        for (r, typ, field, expected, clause) in [
            (
                pc,
                "MECHANICAL_CONTEXT",
                "discipline_type",
                "mechanical",
                "MECHANICAL_CONTEXT.WR1",
            ),
            (
                pdc,
                "DESIGN_CONTEXT",
                "life_cycle_stage",
                "design",
                "DESIGN_CONTEXT.WR1",
            ),
        ] {
            if self.is_a(r, typ) && self.field(r, field)?.text != expected {
                return Err(self.field_failure(
                    Status::Rejected,
                    &format!("SourceRuleViolation:{clause}"),
                    r,
                    field,
                    None,
                    true,
                    expected,
                    &self.field(r, field)?.text,
                    clause,
                ));
            }
        }
        for (r, field, expected, key) in [
            (
                pc,
                "discipline_type",
                "mechanical",
                "UnsupportedProductContextDiscipline",
            ),
            (
                pdc,
                "name",
                "part definition",
                "UnsupportedDefinitionContextRole",
            ),
            (
                pdc,
                "life_cycle_stage",
                "design",
                "UnsupportedDefinitionLifecycleStage",
            ),
        ] {
            if self.field(r, field)?.text != expected {
                let failure = self.field_failure(
                    Status::Unsupported,
                    key,
                    r,
                    field,
                    None,
                    true,
                    expected,
                    &self.field(r, field)?.text,
                    "profile role guard",
                );
                if key != "UnsupportedDefinitionContextRole" || !self.c.interoperability() {
                    return Err(failure);
                }
                self.c.preserve(failure, Proof::ExistingMetadata)?;
            }
        }
        if [
            "visual appearance",
            "tactile appearance",
            "contact ratio",
            "hardness",
            "treatment result",
            "surface texture",
        ]
        .contains(&self.field(pds, "name")?.text.as_str())
        {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedShapePurpose",
                pds,
                "name",
                None,
                true,
                "",
                "",
                "",
            ));
        }
        if ["centre of mass", "treatment result"]
            .contains(&self.field(selected.body, "name")?.text.as_str())
        {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedRepresentationPurpose",
                selected.body,
                "name",
                None,
                true,
                "",
                "",
                "",
            ));
        }
        let categories = self.population("PRODUCT_RELATED_PRODUCT_CATEGORY")?;
        if categories.len() > 1 {
            return Err(self.record_failure(
                "UnsupportedProductCategory",
                *categories.get(1).ok_or_else(internal)?,
                "",
            ));
        }
        if let Some(category) = categories.first().copied() {
            let products = &self.field(category, "products")?.children;
            if products.len() != 1
                || self.c.target(products.first().ok_or_else(internal)?)? != product
                || self.field(category, "name")?.text != "part"
            {
                let failure = self.field_failure(
                    Status::Unsupported,
                    "UnsupportedProductCategory",
                    category,
                    "name",
                    None,
                    true,
                    "part for this product",
                    &self.field(category, "name")?.text,
                    "profile category guard",
                );
                if !self.c.interoperability()
                    || products.len() != 1
                    || self.c.target(products.first().ok_or_else(internal)?)? != product
                {
                    return Err(failure);
                }
                self.c.preserve(failure, Proof::ExistingMetadata)?;
            }
            self.role("category", category);
        }
        let mut shapes = Vec::new();
        for item in &self.field(selected.body, "items")?.children {
            let target = self.c.target(item)?;
            if self.is_a(target, "AXIS2_PLACEMENT_3D") {
                continue;
            }
            let solid = self.is_a(selected.body, "ADVANCED_BREP_SHAPE_REPRESENTATION");
            if !self.is_a(
                target,
                if solid {
                    "MANIFOLD_SOLID_BREP"
                } else {
                    "SHELL_BASED_SURFACE_MODEL"
                },
            ) {
                return Err(self.field_failure(
                    Status::Unsupported,
                    "UnsupportedRepresentationItem",
                    selected.body,
                    "items",
                    Some(item.span),
                    true,
                    "one MSB/SBSM",
                    &self
                        .c
                        .record(target)?
                        .components
                        .first()
                        .ok_or_else(internal)?
                        .name,
                    "profile shape guard",
                ));
            }
            shapes.push(target);
        }
        if shapes.len() != 1 {
            return Err(self.field_failure(
                Status::Unsupported,
                "UnsupportedShapeMultiplicity",
                selected.body,
                "items",
                None,
                false,
                "",
                "",
                "",
            ));
        }
        let shape = *shapes.first().ok_or_else(internal)?;
        self.role("shape", shape);
        let shell = if self.is_a(shape, "MANIFOLD_SOLID_BREP") {
            self.part.shape_role = "MaterialSolid".into();
            self.target_field(shape, "outer")?
        } else {
            let boundaries = &self.field(shape, "sbsm_boundary")?.children;
            if boundaries.len() != 1 {
                return Err(self.field_failure(
                    Status::Unsupported,
                    "UnsupportedShapeMultiplicity",
                    shape,
                    "sbsm_boundary",
                    None,
                    false,
                    "",
                    "",
                    "",
                ));
            }
            let shell = self.c.target(boundaries.first().ok_or_else(internal)?)?;
            self.part.shape_role = if self.is_a(shell, "CLOSED_SHELL") {
                "ClosedSurfaceModel"
            } else {
                "OpenSurfaceModel"
            }
            .into();
            shell
        };
        self.role("shell", shell);
        for face in self.population("FACE")? {
            let mut outer = 0;
            for bound in &self.field(face, "bounds")?.children {
                outer += usize::from(self.is_a(self.c.target(bound)?, "FACE_OUTER_BOUND"));
            }
            if outer != 1 {
                if outer == 0 && !self.planar() && self.scope == GeometryScope::GeneralBrep {
                    self.part.deferred_region_faces.push(face);
                    continue;
                }
                return Err(self.field_failure(
                    Status::Unsupported,
                    "UnsupportedExplicitOuterBoundRequired",
                    face,
                    "bounds",
                    None,
                    true,
                    "one explicit outer bound",
                    &outer.to_string(),
                    "profile face-region guard",
                ));
            }
        }
        Ok(())
    }
    fn reach(&mut self, pending: Vec<usize>, geometry_only: bool) -> Checked<Vec<bool>> {
        reach(self.c, &self.outgoing, pending, geometry_only)
    }
    fn roots(&self) -> Vec<usize> {
        self.part.roles.iter().map(|r| r.record_index).collect()
    }
    fn nondefining_residue(&mut self, r: usize, body_use: &[bool]) -> Checked<bool> {
        self.c.visit(self.c.record(r)?.span)?;
        if self.is_a(r, "STYLED_ITEM") {
            let target = self.field(r, "item")?;
            self.c.visit(target.span)?;
            return Ok(body_use
                .get(self.c.target(target)?)
                .copied()
                .unwrap_or(false));
        }
        let record = self.c.record(r)?;
        let single = record.components.len() == 1;
        let name = record.components.first().map(|c| c.name.as_str());
        if (self.kind()? == "Ap203e2GeneralSource" || self.ap214())
            && single
            && name == Some("PRODUCT_CATEGORY")
        {
            return Ok(self.outgoing.get(r).ok_or_else(internal)?.is_empty());
        }
        let category_link = self.ap214() && single && name == Some("PRODUCT_CATEGORY_RELATIONSHIP");
        if !category_link && !self.c.non_geometric_style(r) {
            return Ok(false);
        }
        for index in self.outgoing.get(r).ok_or_else(internal)? {
            let reference = self
                .c
                .source
                .references()
                .get(*index)
                .ok_or_else(internal)?;
            self.c.visit(reference.span)?;
            if !(if category_link {
                self.is_a(reference.target_record_index, "PRODUCT_CATEGORY")
            } else {
                self.c.non_geometric_style(reference.target_record_index)
            }) {
                return Ok(false);
            }
        }
        Ok(true)
    }
    fn closure(&mut self) -> Checked<()> {
        self.outgoing = self.c.source.records().iter().map(|_| Vec::new()).collect();
        for (i, reference) in self.c.source.references().iter().enumerate() {
            self.outgoing
                .get_mut(reference.record_index)
                .ok_or_else(internal)?
                .push(i);
        }
        if !self.planar() {
            self.value_item_contexts()?;
            self.properties()?;
            self.general_properties()?;
            self.override_cycles()?;
            self.presentations()?;
            self.administrative()?;
        }
        if self.ap214() {
            self.category_hierarchy()?;
            self.invisibility()?;
            self.curve_styles()?;
            self.point_styles()?;
            self.predefined_colours()?;
            self.rendering()?;
        }
        let reached = self.reach(self.roots(), false)?;
        let orphans: Vec<usize> = reached
            .iter()
            .enumerate()
            .filter_map(|(r, used)| (!used).then_some(r))
            .collect();
        if let Some(first) = orphans.first().copied() {
            let mut ids = String::new();
            for r in &orphans {
                ids.push('#');
                ids.push_str(&self.c.record(*r)?.id);
                ids.push(' ');
            }
            let mut failure = self.c.failure(
                Status::Unsupported,
                "UnownedOrOrphanProfileContent",
                "D2",
                "Records are not owned by the closed single-part input",
                self.c.record(first)?.span,
                Some(first),
                0,
                &[],
                "reachable from selected roles",
                &ids,
                "profile ownership, not RI.WR1",
            );
            for r in orphans.iter().skip(1) {
                failure
                    .diagnostic
                    .source
                    .related
                    .push(self.c.record(*r)?.id_span);
            }
            if !self.c.interoperability() {
                return Err(failure);
            }
            let mut body_use = Vec::new();
            if orphans.iter().any(|r| self.is_a(*r, "STYLED_ITEM")) {
                let mut items = Vec::new();
                for value in &self.field(self.rep()?, "items")?.children {
                    items.push(self.c.target(value)?);
                }
                body_use = self.reach(items, true)?;
            }
            for r in &orphans {
                if !self.nondefining_residue(*r, &body_use)? {
                    return Err(failure);
                }
            }
            self.c.outcome.qualification.unattached = orphans;
            self.c.preserve(failure, Proof::ExistingMetadata)?;
        }
        let mut items = Vec::new();
        for item in &self.field(self.rep()?, "items")?.children {
            items.push(self.c.target(item)?);
        }
        if !self.planar() {
            for rep in self.population("REPRESENTATION")? {
                if rep == self.rep()? {
                    continue;
                }
                for item in &self.field(rep, "items")?.children {
                    items.push(self.c.target(item)?);
                }
            }
        }
        self.representation_use = self.reach(items, true)?;
        Ok(())
    }
}
pub(super) fn check(c: &mut Checker<'_>, scope: GeometryScope) -> Checked<String> {
    let mut ownership = Ownership {
        c,
        part: Part {
            application_protocol_present: true,
            ..Part::default()
        },
        scope,
        representation: None,
        context: None,
        wireframe: None,
        construction: None,
        outgoing: Vec::new(),
        presentation_roots: Vec::new(),
        representation_use: Vec::new(),
        administrative: Vec::new(),
        style_overrides: BTreeSet::new(),
    };
    ownership.product()?;
    ownership.closure()?;
    let context = ownership
        .c
        .record(ownership.context.ok_or_else(internal)?)?
        .id
        .clone();
    if ownership.c.interoperability() {
        ownership.c.ownership_evidence = Some(OwnershipEvidence {
            outgoing: ownership.outgoing,
            presentation_roots: ownership.presentation_roots,
            administrative: ownership.administrative,
            style_overrides: ownership.style_overrides,
        });
    }
    ownership.c.part = Some(ownership.part);
    ownership.c.representation_use = Some(ownership.representation_use);
    Ok(context)
}

// Same traversal/edge order and charged visits for broad ownership and approval.
// This shared helper avoids constructing a second interpreted reference graph.
pub(super) fn reach(
    c: &mut Checker<'_>,
    outgoing: &[Vec<usize>],
    mut pending: Vec<usize>,
    geometry_only: bool,
) -> Checked<Vec<bool>> {
    let mut visited = vec![false; c.source.records().len()];
    while let Some(r) = pending.pop() {
        if *visited.get(r).ok_or_else(internal)? {
            continue;
        }
        *visited.get_mut(r).ok_or_else(internal)? = true;
        c.visit(c.record(r)?.span)?;
        for index in outgoing.get(r).ok_or_else(internal)? {
            let reference = c.source.references().get(*index).ok_or_else(internal)?;
            c.visit(reference.span)?;
            let bridge = geometry_only
                && c.is_a(r, "COMPOSITE_CURVE")
                && c.is_a(reference.target_record_index, "COMPOSITE_CURVE_SEGMENT");
            if !geometry_only
                || c.is_a(reference.target_record_index, "REPRESENTATION_ITEM")
                || bridge
            {
                pending.push(reference.target_record_index);
            }
        }
    }
    Ok(visited)
}
