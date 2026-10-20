// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Presentation attachment and its necessary strict source predicates.
//! Upstream: `source/STEP_SEMANTICS/src/StepSemantics.cpp`, presentation/style methods.
//! Deviations: checked source indices and strict-only policy. Appearance never
//! creates physical ownership, suppresses required records or changes geometry.
use super::{internal, Checked, Ownership, Status, Value};
use std::collections::BTreeSet;

impl Ownership<'_, '_> {
    pub(super) fn override_cycles(&mut self) -> Checked<()> {
        if self.kind()? != "Ap214MbxSource" {
            return Ok(());
        }
        let overrides = self.population("OVER_RIDING_STYLED_ITEM")?;
        if overrides.is_empty() {
            return Ok(());
        }
        let mut colour = vec![0_u8; self.c.source.records().len()];
        for start in overrides {
            let mut path = Vec::new();
            let mut current = start;
            while self.is_a(current, "OVER_RIDING_STYLED_ITEM") && colour.get(current) != Some(&2) {
                *colour.get_mut(current).ok_or_else(internal)? = 1;
                path.push(current);
                let link = self.field(current, "over_ridden_style")?;
                self.c.visit(link.span)?;
                let next = self.c.target(link)?;
                if colour.get(next) == Some(&1) {
                    return Err(self.field_failure(
                        Status::Unsupported,
                        "UnsupportedPresentationOverrideCycle",
                        current,
                        "over_ridden_style",
                        None,
                        false,
                        "",
                        "",
                        "",
                    ));
                }
                current = next;
            }
            for r in path {
                *colour.get_mut(r).ok_or_else(internal)? = 2;
            }
        }
        Ok(())
    }
    pub(super) fn presentations(&mut self) -> Checked<()> {
        let mut items = Vec::new();
        let mut reps = vec![self.rep()?];
        reps.extend(self.wireframe);
        reps.extend(self.construction);
        for rep in reps {
            for value in &self.field(rep, "items")?.children {
                items.push(self.c.target(value)?);
            }
        }
        let used = self.reach(items, true)?;
        for (i, presentation) in self
            .population("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")?
            .into_iter()
            .enumerate()
        {
            for value in &self.field(presentation, "items")?.children {
                self.c.visit(value.span)?;
                let styled = self.c.target(value)?;
                if !self.is_a(styled, "STYLED_ITEM") {
                    return Err(self.field_failure(
                        Status::Unsupported,
                        "UnsupportedPresentationAttachment",
                        presentation,
                        "items",
                        Some(value.span),
                        true,
                        "",
                        "",
                        "",
                    ));
                }
                let target = self.field(styled, "item")?;
                if !used
                    .get(self.c.target(target)?)
                    .copied()
                    .ok_or_else(internal)?
                {
                    return Err(self.field_failure(
                        Status::Unsupported,
                        "UnsupportedPresentationAttachment",
                        styled,
                        "item",
                        None,
                        true,
                        "RI used by selected body",
                        &target.text,
                        "profile presentation attachment guard",
                    ));
                }
                let mut current = styled;
                while self.is_a(current, "OVER_RIDING_STYLED_ITEM") {
                    let owner = current;
                    let ancestor = self.field(current, "over_ridden_style")?;
                    self.c.visit(ancestor.span)?;
                    current = self.c.target(ancestor)?;
                    if !self.is_a(current, "STYLED_ITEM") {
                        return Err(self.field_failure(
                            Status::Unsupported,
                            "UnsupportedPresentationAttachment",
                            owner,
                            "over_ridden_style",
                            None,
                            true,
                            "",
                            "",
                            "",
                        ));
                    }
                    let item = self.field(current, "item")?;
                    self.c.visit(item.span)?;
                    if !used
                        .get(self.c.target(item)?)
                        .copied()
                        .ok_or_else(internal)?
                    {
                        return Err(self.field_failure(
                            Status::Unsupported,
                            "UnsupportedPresentationAttachment",
                            current,
                            "item",
                            None,
                            true,
                            "",
                            "",
                            "",
                        ));
                    }
                }
            }
            self.role(format!("presentation_{i}"), presentation);
            self.presentation_roots.push(presentation);
        }
        Ok(())
    }
    fn palette(
        &mut self,
        type_name: &str,
        names: &[&str],
        expected: &str,
        clause: &str,
    ) -> Checked<()> {
        for r in self.population(type_name)? {
            let value = self.field(r, "name")?;
            self.c.visit(value.span)?;
            if !names.contains(&value.text.as_str()) {
                return Err(self.field_failure(
                    Status::Rejected,
                    &format!("SourceRuleViolation:{clause}"),
                    r,
                    "name",
                    None,
                    true,
                    expected,
                    &value.text,
                    &format!("AP214E3_2010.exp {clause}"),
                ));
            }
        }
        Ok(())
    }
    pub(super) fn predefined_colours(&mut self) -> Checked<()> {
        self.palette(
            "DRAUGHTING_PRE_DEFINED_COLOUR",
            &[
                "red", "green", "blue", "yellow", "magenta", "cyan", "black", "white",
            ],
            "eight exact draughting colour names",
            "DRAUGHTING_PRE_DEFINED_COLOUR.WR1",
        )
    }
    fn style_size(&mut self, style: usize, field: &str) -> Checked<()> {
        let value = self.field(style, field)?;
        self.c.visit(value.span)?;
        if value.kind == crate::ValueKind::Typed && value.text == "POSITIVE_LENGTH_MEASURE" {
            let scalar = value.children.first().ok_or_else(internal)?;
            if super::super::number::compare(&scalar.text, "0.") <= 0 {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:POSITIVE_LENGTH_MEASURE.WR1",
                    style,
                    field,
                    None,
                    true,
                    "value > 0",
                    &scalar.text,
                    "AP214E3_2010.exp POSITIVE_LENGTH_MEASURE.WR1",
                ));
            }
        }
        Ok(())
    }
    fn style_colour(&mut self, style: usize, field: &str) -> Checked<()> {
        let colour = self.target_field(style, field)?;
        if self.is_a(colour, "COLOUR_RGB") {
            for (field, clause) in [
                ("red", "COLOUR_RGB.WR1"),
                ("green", "COLOUR_RGB.WR2"),
                ("blue", "COLOUR_RGB.WR3"),
            ] {
                let value = self.field(colour, field)?;
                self.c.visit(value.span)?;
                if super::super::number::compare(&value.text, "0.") < 0
                    || super::super::number::compare(&value.text, "1.") > 0
                {
                    return Err(self.field_failure(
                        Status::Rejected,
                        &format!("SourceRuleViolation:{clause}"),
                        colour,
                        field,
                        None,
                        true,
                        "0 <= value <= 1",
                        &value.text,
                        &format!("AP214E3_2010.exp {clause}"),
                    ));
                }
            }
        }
        Ok(())
    }
    fn count_style(&mut self, values: &Value, name: &str) -> Checked<usize> {
        let mut count = 0;
        for value in &values.children {
            self.c.visit(value.span)?;
            count += usize::from(self.is_a(self.c.target(value)?, name));
        }
        Ok(count)
    }
    pub(super) fn curve_styles(&mut self) -> Checked<()> {
        self.palette(
            "DRAUGHTING_PRE_DEFINED_CURVE_FONT",
            &[
                "continuous",
                "chain",
                "chain double dash",
                "dashed",
                "dotted",
            ],
            "five exact draughting font names",
            "DRAUGHTING_PRE_DEFINED_CURVE_FONT.WR1",
        )?;
        let styles = self.population("CURVE_STYLE")?;
        for r in &styles {
            self.style_size(*r, "curve_width")?;
            self.style_colour(*r, "curve_colour")?;
        }
        let mut assignments = BTreeSet::new();
        if !styles.is_empty() {
            for r in self.population("PRESENTATION_STYLE_ASSIGNMENT")? {
                let values = self.field(r, "styles")?;
                let count = self.count_style(values, "CURVE_STYLE")?;
                if count > 1 {
                    return Err(self.field_failure(
                        Status::Rejected,
                        "SourceRuleViolation:PRESENTATION_STYLE_ASSIGNMENT.WR1",
                        r,
                        "styles",
                        None,
                        true,
                        "at most one CURVE_STYLE",
                        &count.to_string(),
                        "AP214E3_2010.exp PRESENTATION_STYLE_ASSIGNMENT.WR1",
                    ));
                }
                if count != 0 {
                    assignments.insert(r);
                }
            }
        }
        for r in self.population("STYLED_ITEM")? {
            if !self.is_a(self.target_field(r, "item")?, "CURVE") {
                continue;
            }
            let mut count = 0;
            for value in &self.field(r, "styles")?.children {
                self.c.visit(value.span)?;
                count += usize::from(assignments.contains(&self.c.target(value)?));
            }
            if count != 1 {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:styled_curve.WR1",
                    r,
                    "styles",
                    None,
                    true,
                    "one curve-style assignment",
                    &count.to_string(),
                    "AP214E3_2010.exp styled_curve.WR1",
                ));
            }
        }
        Ok(())
    }
    pub(super) fn point_styles(&mut self) -> Checked<()> {
        let styles = self.population("POINT_STYLE")?;
        let markers = self.population("PRE_DEFINED_POINT_MARKER_SYMBOL")?;
        if styles.is_empty() && markers.is_empty() {
            return Ok(());
        }
        self.palette(
            "PRE_DEFINED_POINT_MARKER_SYMBOL",
            &[
                "asterisk", "circle", "dot", "plus", "square", "triangle", "x",
            ],
            "seven exact predefined point marker names",
            "PRE_DEFINED_POINT_MARKER_SYMBOL.WR1",
        )?;
        for r in styles {
            self.style_size(r, "marker_size")?;
            self.style_colour(r, "marker_colour")?;
        }
        for r in self.population("PRESENTATION_STYLE_ASSIGNMENT")? {
            let count = self.count_style(self.field(r, "styles")?, "POINT_STYLE")?;
            if count > 1 {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:PRESENTATION_STYLE_ASSIGNMENT.WR1",
                    r,
                    "styles",
                    None,
                    true,
                    "at most one POINT_STYLE",
                    &count.to_string(),
                    "AP214E3_2010.exp PRESENTATION_STYLE_ASSIGNMENT.WR1",
                ));
            }
        }
        for rep in self.population("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION")? {
            for item in &self.field(rep, "items")?.children {
                self.c.visit(item.span)?;
                let styled = self.c.target(item)?;
                if !self.is_a(styled, "STYLED_ITEM") {
                    continue;
                }
                for assignment in &self.field(styled, "styles")?.children {
                    self.c.visit(assignment.span)?;
                    for entry in &self.field(self.c.target(assignment)?, "styles")?.children {
                        self.c.visit(entry.span)?;
                        let style = self.c.target(entry)?;
                        if !self.is_a(style, "POINT_STYLE") {
                            continue;
                        }
                        let size = self.field(style, "marker_size")?;
                        self.c.visit(size.span)?;
                        let colour = self.target_field(style, "marker_colour")?;
                        if size.kind != crate::ValueKind::Typed
                            || size.text != "POSITIVE_LENGTH_MEASURE"
                            || (!self.is_a(colour, "COLOUR_RGB")
                                && !self.is_a(colour, "DRAUGHTING_PRE_DEFINED_COLOUR"))
                        {
                            return Err(self.field_failure(Status::Rejected,"SourceRuleViolation:MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION.WR6",style,"marker_size",None,true,"POSITIVE_LENGTH_MEASURE and RGB/draughting colour",&size.text,"AP214E3_2010.exp MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION.WR6"));
                        }
                    }
                }
            }
        }
        Ok(())
    }
    pub(super) fn rendering(&mut self) -> Checked<()> {
        for r in self.population("SURFACE_STYLE_TRANSPARENT")? {
            let value = self.field(r, "transparency")?;
            self.c.visit(value.span)?;
            if super::super::number::compare(&value.text, "0.") < 0
                || super::super::number::compare(&value.text, "1.") > 0
            {
                return Err(self.field_failure(
                    Status::Rejected,
                    "SourceRuleViolation:SURFACE_STYLE_TRANSPARENT.WR1",
                    r,
                    "transparency",
                    None,
                    true,
                    "0 <= value <= 1",
                    &value.text,
                    "AP214E3_2010.exp SURFACE_STYLE_TRANSPARENT.WR1",
                ));
            }
        }
        for r in self.population("SURFACE_STYLE_RENDERING_WITH_PROPERTIES")? {
            let props = self.field(r, "properties")?;
            self.c.visit(props.span)?;
            if props.children.len() == 2 {
                let a = self
                    .c
                    .target(props.children.first().ok_or_else(internal)?)?;
                let b = self.c.target(props.children.get(1).ok_or_else(internal)?)?;
                if self.c.types.get(a).ok_or_else(internal)?.0
                    == self.c.types.get(b).ok_or_else(internal)?.0
                {
                    return Err(self.field_failure(
                        Status::Rejected,
                        "SourceRuleViolation:SURFACE_STYLE_RENDERING_WITH_PROPERTIES.WR1",
                        r,
                        "properties",
                        None,
                        true,
                        "distinct TYPEOF sets",
                        "same TYPEOF set",
                        "AP214E3_2010.exp SURFACE_STYLE_RENDERING_WITH_PROPERTIES.WR1",
                    ));
                }
            }
        }
        Ok(())
    }
}
