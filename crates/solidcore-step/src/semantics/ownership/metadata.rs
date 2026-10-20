// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Source property, administrative and dictionary attachment prerequisites.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemantics.cpp`, corresponding
//! Attach*/CheckAp214ValueItemContexts/CheckAp214CategoryHierarchy methods.
//! Deviations: strict-only policy, checked indices and iterative traversals.
//! Inverse metadata can acquire a root only after its actual targets are owned.
use super::{internal, Checked, Failure, Ownership, PropertyAttachment, Status, Value};
use std::collections::{BTreeMap, BTreeSet};

impl Ownership<'_, '_> {
    pub(super) fn value_item_contexts(&mut self) -> Checked<()> {
        if !self.ap214() {
            return Ok(());
        }
        let values = self.population("VALUE_REPRESENTATION_ITEM")?;
        if values.is_empty() {
            return Ok(());
        }
        let mut invalid = vec![None; self.c.source.records().len()];
        let mut pending = Vec::new();
        for rep in self.population("REPRESENTATION")? {
            let context = self.field(rep, "context_of_items")?;
            self.c.visit(context.span)?;
            if self.is_a(self.c.target(context)?, "GLOBAL_UNIT_ASSIGNED_CONTEXT") {
                continue;
            }
            for item in &self.field(rep, "items")?.children {
                self.c.visit(item.span)?;
                let r = self.c.target(item)?;
                let slot = invalid.get_mut(r).ok_or_else(internal)?;
                if slot.is_none() {
                    *slot = Some(rep);
                    pending.push(r);
                }
            }
        }
        let mut at = 0;
        while let Some(r) = pending.get(at).copied() {
            at += 1;
            self.c.visit(self.c.record(r)?.span)?;
            for index in self.outgoing.get(r).ok_or_else(internal)? {
                let reference = self
                    .c
                    .source
                    .references()
                    .get(*index)
                    .ok_or_else(internal)?;
                self.c.visit(reference.span)?;
                let child = reference.target_record_index;
                if self.is_a(child, "REPRESENTATION_ITEM") || self.is_a(child, "FOUNDED_ITEM") {
                    let owner = invalid.get(r).copied().flatten().ok_or_else(internal)?;
                    let slot = invalid.get_mut(child).ok_or_else(internal)?;
                    if slot.is_none() {
                        *slot = Some(owner);
                        pending.push(child);
                    }
                }
            }
        }
        for value in values {
            self.c.visit(self.c.record(value)?.span)?;
            if let Some(rep) = invalid.get(value).copied().flatten() {
                let mut failure = self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:VALUE_REPRESENTATION_ITEM.WR1",
                    rep,
                    "context_of_items",
                    None,
                    true,
                    "GLOBAL_UNIT_ASSIGNED_CONTEXT",
                    "context without unit assignment",
                    "AP214E3_2010.exp VALUE_REPRESENTATION_ITEM.WR1",
                );
                failure
                    .diagnostic
                    .source
                    .related
                    .push(self.c.record(value)?.id_span);
                return Err(failure);
            }
        }
        Ok(())
    }
    fn property_reject(
        &self,
        owner: usize,
        name: &str,
        at: &Value,
        key: &str,
        expected: &str,
        member: Option<usize>,
    ) -> Box<Failure> {
        match self.c.field(owner, name) {
            Ok((a, _)) => {
                let mut path = vec![a.parameter + 1];
                if let Some(i) = member {
                    path.push(i);
                }
                self.c.failure(
                    Status::Unsupported,
                    key,
                    "D2",
                    "Property metadata is outside the selected-part attachment subset",
                    at.span,
                    Some(owner),
                    a.component,
                    &path,
                    expected,
                    &at.text,
                    "profile property attachment guard; not complete EXPRESS WHERE",
                )
            }
            Err(e) => e,
        }
    }
    pub(super) fn properties(&mut self) -> Checked<()> {
        let pd = self.one("PRODUCT_DEFINITION")?;
        let pds = self.one("PRODUCT_DEFINITION_SHAPE")?;
        let mut attached = Vec::new();
        for link in self.population("PROPERTY_DEFINITION_REPRESENTATION")? {
            if self.is_a(link, "SHAPE_DEFINITION_REPRESENTATION") {
                continue;
            }
            self.c.outcome.property_attachment_checks += 1;
            self.c.visit(self.c.record(link)?.span)?;
            let definition = self.field(link, "definition")?;
            self.c.visit(definition.span)?;
            let property = self.c.target(definition)?;
            let characterized = self.field(property, "definition")?;
            self.c.visit(characterized.span)?;
            let owner = self.c.target(characterized)?;
            if owner != pd && owner != pds {
                return Err(self.property_reject(
                    property,
                    "definition",
                    characterized,
                    "UnsupportedPropertyAttachment",
                    "selected PD or PDS identity",
                    None,
                ));
            }
            let used = self.field(link, "used_representation")?;
            self.c.visit(used.span)?;
            let rep = self.c.target(used)?;
            if !self.plain_representation(rep)? {
                return Err(self.property_reject(
                    link,
                    "used_representation",
                    used,
                    "UnsupportedPropertyRepresentation",
                    "plain property REPRESENTATION",
                    None,
                ));
            }
            for (i, item) in self.field(rep, "items")?.children.iter().enumerate() {
                self.c.visit(item.span)?;
                let target = self.c.target(item)?;
                let value_item = self.is_a(target, "VALUE_REPRESENTATION_ITEM");
                if !self.is_a(target, "MEASURE_REPRESENTATION_ITEM")
                    && !self.is_a(target, "DESCRIPTIVE_REPRESENTATION_ITEM")
                    && !value_item
                {
                    return Err(self.property_reject(
                        rep,
                        "items",
                        item,
                        "UnsupportedPropertyItem",
                        "measure or descriptive item",
                        Some(i + 1),
                    ));
                }
                if value_item
                    && self.kind()? == "Ap203e2GeneralSource"
                    && !self.is_a(
                        self.target_field(rep, "context_of_items")?,
                        "GLOBAL_UNIT_ASSIGNED_CONTEXT",
                    )
                {
                    return Err(self.field_failure(
                        Status::Rejected,
                        "SourceRuleViolation:VALUE_REPRESENTATION_ITEM.WR1",
                        rep,
                        "context_of_items",
                        None,
                        true,
                        "GLOBAL_UNIT_ASSIGNED_CONTEXT",
                        "context without unit assignment",
                        "N2635 VALUE_REPRESENTATION_ITEM.WR1",
                    ));
                }
            }
            let context = self.field(rep, "context_of_items")?;
            self.c.visit(context.span)?;
            attached.push(PropertyAttachment {
                link,
                property_definition: property,
                characterized_definition: owner,
                product_definition: pd,
                representation: rep,
                context: self.c.target(context)?,
            });
        }
        for (i, property) in attached.iter().enumerate() {
            self.role(format!("property_link_{i}"), property.link);
        }
        self.part.property_attachments = attached;
        Ok(())
    }
    pub(super) fn general_properties(&mut self) -> Checked<()> {
        if self.kind()? != "Ap214MbxSource" {
            return Ok(());
        }
        let associations = self.population("GENERAL_PROPERTY_ASSOCIATION")?;
        if associations.is_empty() {
            return Ok(());
        }
        let owned = self.reach(self.roots(), false)?;
        let mut attached = BTreeSet::new();
        for property in &self.part.property_attachments {
            self.c
                .visit(self.c.record(property.property_definition)?.span)?;
            attached.insert(property.property_definition);
        }
        let mut counts = BTreeMap::new();
        for r in &associations {
            self.c.visit(self.c.record(*r)?.span)?;
            *counts
                .entry(self.target_field(*r, "derived_definition")?)
                .or_insert(0_usize) += 1;
        }
        for r in &associations {
            let derived = self.field(*r, "derived_definition")?;
            let target = self.c.target(derived)?;
            let base = self.target_field(*r, "base_definition")?;
            self.c.visit(derived.span)?;
            let count = *counts.get(&target).ok_or_else(internal)?;
            if count != 1 {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:GENERAL_PROPERTY_ASSOCIATION.WR1",
                    *r,
                    "derived_definition",
                    None,
                    true,
                    "one association",
                    &count.to_string(),
                    "AP214E3_2010.exp GENERAL_PROPERTY_ASSOCIATION.WR1",
                ));
            }
            if self.field(base, "name")?.text != self.field(target, "name")?.text {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:GENERAL_PROPERTY_ASSOCIATION.WR2",
                    *r,
                    "derived_definition",
                    None,
                    true,
                    &self.field(base, "name")?.text,
                    &self.field(target, "name")?.text,
                    "AP214E3_2010.exp GENERAL_PROPERTY_ASSOCIATION.WR2",
                ));
            }
            if !owned.get(target).copied().ok_or_else(internal)? || !attached.contains(&target) {
                return Err(self.field_failure(
                    Status::Unsupported,
                    "UnsupportedGeneralPropertyAttachment",
                    *r,
                    "derived_definition",
                    None,
                    true,
                    "",
                    "",
                    "",
                ));
            }
        }
        for r in associations {
            self.role(format!("general_property_association_{r}"), r);
        }
        Ok(())
    }
    pub(super) fn administrative(&mut self) -> Checked<()> {
        let mut candidates = Vec::new();
        for name in [
            "CC_DESIGN_APPROVAL",
            "CC_DESIGN_SECURITY_CLASSIFICATION",
            "CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT",
            "CC_DESIGN_DATE_AND_TIME_ASSIGNMENT",
            "APPROVAL_PERSON_ORGANIZATION",
            "APPROVAL_DATE_TIME",
            "PRESENTATION_LAYER_ASSIGNMENT",
            "APPLIED_DATE_AND_TIME_ASSIGNMENT",
        ] {
            if self.c.schema()?.find(name).is_some() {
                candidates.extend(self.population(name)?);
            }
        }
        if self.ap214() && candidates.is_empty() {
            return Ok(());
        }
        let mut attached = vec![false; self.c.source.records().len()];
        let mut changed = true;
        // Preserve fixed-point root snapshots and candidate order. Newly admitted
        // administrative records cannot bootstrap a detached cycle's ownership.
        while changed {
            changed = false;
            let owned = self.reach(self.roots(), false)?;
            for r in &candidates {
                if attached.get(*r) == Some(&true) {
                    continue;
                }
                self.c.visit(self.c.record(*r)?.span)?;
                let name = if self.is_a(*r, "APPROVAL_PERSON_ORGANIZATION") {
                    "authorized_approval"
                } else if self.is_a(*r, "APPROVAL_DATE_TIME") {
                    "dated_approval"
                } else if self.is_a(*r, "PRESENTATION_LAYER_ASSIGNMENT") {
                    "assigned_items"
                } else {
                    "items"
                };
                let value = self.field(*r, name)?;
                let mut valid = true;
                if value.kind == crate::ValueKind::Aggregate {
                    for item in &value.children {
                        self.c.visit(item.span)?;
                        valid = valid
                            && owned
                                .get(self.c.target(item)?)
                                .copied()
                                .ok_or_else(internal)?;
                    }
                } else {
                    self.c.visit(value.span)?;
                    valid = owned
                        .get(self.c.target(value)?)
                        .copied()
                        .ok_or_else(internal)?;
                }
                if valid {
                    *attached.get_mut(*r).ok_or_else(internal)? = true;
                    self.role(format!("administrative_attachment_{r}"), *r);
                    changed = true;
                }
            }
        }
        for r in candidates {
            if attached.get(r) != Some(&true) {
                return Err(self.record_failure("UnsupportedAdministrativeAttachment", r, ""));
            }
        }
        Ok(())
    }
    pub(super) fn category_hierarchy(&mut self) -> Checked<()> {
        let relationships = self.population("PRODUCT_CATEGORY_RELATIONSHIP")?;
        if relationships.is_empty() {
            return Ok(());
        }
        let mut outgoing: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for r in relationships {
            self.c.visit(self.c.record(r)?.span)?;
            outgoing
                .entry(self.target_field(r, "category")?)
                .or_default()
                .push(r);
        }
        let mut colour = vec![0_u8; self.c.source.records().len()];
        let mut stack: Vec<(usize, usize)> = Vec::new();
        for root in self.population("PRODUCT_CATEGORY")? {
            self.c.visit(self.c.record(root)?.span)?;
            if colour.get(root) != Some(&0) {
                continue;
            }
            *colour.get_mut(root).ok_or_else(internal)? = 1;
            stack.push((root, 0));
            while let Some((node, next)) = stack.last().copied() {
                let Some(relation) = outgoing
                    .get(&node)
                    .and_then(|edges| edges.get(next))
                    .copied()
                else {
                    *colour.get_mut(node).ok_or_else(internal)? = 2;
                    stack.pop();
                    continue;
                };
                stack.last_mut().ok_or_else(internal)?.1 += 1;
                let value = self.field(relation, "sub_category")?;
                self.c.visit(value.span)?;
                let child = self.c.target(value)?;
                if colour.get(child) == Some(&1) {
                    return Err(self.field_failure(
                        Status::Rejected,
                        "SourceRuleViolation:PRODUCT_CATEGORY_RELATIONSHIP.WR1",
                        relation,
                        "sub_category",
                        None,
                        true,
                        "acyclic category references",
                        "cycle",
                        "AP214E3_2010.exp PRODUCT_CATEGORY_RELATIONSHIP.WR1",
                    ));
                }
                if colour.get(child) == Some(&0) {
                    *colour.get_mut(child).ok_or_else(internal)? = 1;
                    stack.push((child, 0));
                }
            }
        }
        Ok(())
    }
    pub(super) fn invisibility(&mut self) -> Checked<()> {
        let instances = self.population("INVISIBILITY")?;
        if instances.is_empty() {
            return Ok(());
        }
        let owned = self.reach(self.roots(), false)?;
        for r in instances {
            for item in &self.field(r, "invisible_items")?.children {
                self.c.visit(item.span)?;
                if !owned
                    .get(self.c.target(item)?)
                    .copied()
                    .ok_or_else(internal)?
                {
                    return Err(self.field_failure(
                        Status::Unsupported,
                        "UnsupportedInvisibilityAttachment",
                        r,
                        "invisible_items",
                        Some(item.span),
                        true,
                        "previously owned presentation target",
                        &item.text,
                        "project invisibility attachment guard",
                    ));
                }
            }
            self.role(format!("invisibility_{r}"), r);
        }
        Ok(())
    }
}
