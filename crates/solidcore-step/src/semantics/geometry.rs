// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Exact fixed-source geometry parameter and definition prerequisites.
//! Upstream: `source/STEP_SEMANTICS/src/StepGeometryDependencies.cpp`.
//! Rust Geometry/Count/failure correspond to C++ GeometryDependencies/Count/Fail.
//! Deviations: checked indices and explicit graph stacks. No fitting, unit
//! application, geometry evaluation or native construction occurs in this pass.

mod curves;
mod splines;
use super::checker::{internal, Checked, Checker, Failure};
use super::number::compare;
use crate::{Span, Value};

#[derive(Clone, Copy)]
enum Predicate {
    Dependency,
    Entity,
    Type,
}
struct Geometry<'c, 's> {
    c: &'c mut Checker<'s>,
    postorder: Vec<usize>,
    wire_prepared: bool,
}
impl<'s> Geometry<'_, 's> {
    fn field(&self, r: usize, name: &str) -> Checked<&'s Value> {
        Ok(self.c.field(r, name)?.1)
    }
    fn population(&self, name: &str) -> Checked<Vec<usize>> {
        self.c.population(name)
    }
    fn count(&mut self, span: Span, kind: Predicate) -> Checked<()> {
        // The request visit budget is charged before counting or decimal work.
        self.c.visit(span)?;
        self.c.outcome.geometry_dependency_checks.total += 1;
        match kind {
            Predicate::Type => self.c.outcome.geometry_dependency_checks.type_predicates += 1,
            Predicate::Entity => self.c.outcome.geometry_dependency_checks.entity_predicates += 1,
            Predicate::Dependency => {}
        }
        Ok(())
    }
    fn failure(
        &self,
        r: usize,
        field: &str,
        value: &Value,
        clause: &str,
        expected: &str,
        actual: &str,
        suffix: &[usize],
        dependency: bool,
        authority: Option<&str>,
        related: Vec<Span>,
    ) -> Box<Failure> {
        match self.c.field(r, field) {
            Ok((a, _)) => {
                let mut path = vec![a.parameter + 1];
                path.extend_from_slice(suffix);
                let key = if dependency {
                    "SourceGeometryDependencyViolation".into()
                } else {
                    format!("SourceRuleViolation:{clause}")
                };
                let responsibility = authority.map_or_else(
                    || {
                        format!(
                            "{} {clause}",
                            self.c
                                .catalog
                                .map_or("", super::catalog::Catalog::authority)
                        )
                    },
                    str::to_owned,
                );
                let mut failure = self.c.failure(
                    super::Status::Rejected,
                    &key,
                    "D3",
                    "Source geometry parameter violates the selected prerequisite",
                    value.span,
                    Some(r),
                    a.component,
                    &path,
                    expected,
                    actual,
                    &responsibility,
                );
                failure.diagnostic.source.related = related;
                failure
            }
            Err(e) => e,
        }
    }
    fn run(&mut self) -> Checked<()> {
        let mut trimmed = Vec::new();
        for (r, record) in self.c.source.records().iter().enumerate() {
            self.c.visit(record.span)?;
            let count = self.c.attributes.get(r).ok_or_else(internal)?.len();
            for index in 0..count {
                // Borrow the source value and copy only static slot metadata.
                // Do not clone the bound attribute graph for each source record.
                let a = self
                    .c
                    .attributes
                    .get(r)
                    .and_then(|fields| fields.get(index))
                    .ok_or_else(internal)?;
                let name = a.name;
                let positive = a.declared_type == "POSITIVE_LENGTH_MEASURE";
                let cone = record
                    .components
                    .get(a.component)
                    .is_some_and(|c| c.name == "CONICAL_SURFACE")
                    && a.name == "radius";
                if !positive && !cone {
                    continue;
                }
                let value = self.c.value(r, a)?;
                self.count(
                    value.span,
                    if positive {
                        Predicate::Type
                    } else {
                        Predicate::Entity
                    },
                )?;
                let sign = compare(&value.text, "0.");
                if if positive { sign <= 0 } else { sign < 0 } {
                    return Err(self.failure(
                        r,
                        name,
                        value,
                        if positive {
                            "POSITIVE_LENGTH_MEASURE.WR1"
                        } else {
                            "CONICAL_SURFACE.WR1"
                        },
                        if positive { ">0" } else { ">=0" },
                        &value.text,
                        &[],
                        false,
                        None,
                        Vec::new(),
                    ));
                }
            }
            if self.c.is_a(r, "B_SPLINE_CURVE_WITH_KNOTS") {
                self.direction(
                    r,
                    self.field(r, "control_points_list")?.children.len(),
                    splines::CURVE,
                )?;
            }
            if self.c.is_a(r, "B_SPLINE_SURFACE_WITH_KNOTS") {
                let grid = self.field(r, "control_points_list")?;
                let width = grid.children.first().ok_or_else(internal)?.children.len();
                self.count(grid.span, Predicate::Dependency)?;
                for (i, row) in grid.children.iter().enumerate() {
                    self.c.visit(row.span)?;
                    if row.children.len() != width {
                        return Err(self.failure(
                            r,
                            "control_points_list",
                            row,
                            "B_SPLINE_SURFACE.control_points DERIVE",
                            &width.to_string(),
                            &row.children.len().to_string(),
                            &[i + 1],
                            true,
                            None,
                            Vec::new(),
                        ));
                    }
                }
                self.direction(r, grid.children.len(), splines::U)?;
                self.direction(r, width, splines::V)?;
            }
            if self.c.is_a(r, "RATIONAL_B_SPLINE_CURVE") {
                self.weights(r, false)?;
            }
            if self.c.is_a(r, "RATIONAL_B_SPLINE_SURFACE") {
                self.weights(r, true)?;
            }
            if self.c.is_a(r, "TRIMMED_CURVE") {
                trimmed.push(r);
                for name in ["trim_1", "trim_2"] {
                    let value = self.field(r, name)?;
                    self.count(value.span, Predicate::Entity)?;
                    if value.children.len() == 2
                        && value.children.first().ok_or_else(internal)?.kind
                            == value.children.get(1).ok_or_else(internal)?.kind
                    {
                        return Err(self.failure(
                            r,
                            name,
                            value,
                            if name == "trim_1" {
                                "TRIMMED_CURVE.WR1"
                            } else {
                                "TRIMMED_CURVE.WR2"
                            },
                            "different SELECT member types",
                            "same SELECT member type",
                            &[],
                            false,
                            None,
                            Vec::new(),
                        ));
                    }
                }
            }
        }
        let catalog = self.c.schema()?;
        let composites = catalog
            .find("COMPOSITE_CURVE")
            .map(|_| self.population("COMPOSITE_CURVE"))
            .transpose()?
            .unwrap_or_default();
        let segments = catalog
            .find("COMPOSITE_CURVE_SEGMENT")
            .map(|_| self.population("COMPOSITE_CURVE_SEGMENT"))
            .transpose()?
            .unwrap_or_default();
        if !composites.is_empty() || !segments.is_empty() {
            self.composite_checks(&composites, &segments)?;
            self.composite_closure(&trimmed, &composites)?;
        } else {
            self.trimmed_closure(&trimmed)?;
        }
        self.wireframes()?;
        self.bounded_surface_role()?;
        Ok(())
    }
    fn bounded_surface_role(&mut self) -> Checked<()> {
        if self
            .c
            .schema()?
            .find("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")
            .is_none()
        {
            return Ok(());
        }
        let reps = self.population("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")?;
        if reps.is_empty() {
            return Ok(());
        }
        let mut memo = vec![None; self.c.source.records().len()];
        for r in reps {
            let items = self.field(r, "items")?;
            self.count(items.span, Predicate::Entity)?;
            let mut found = false;
            for item in &items.children {
                self.c.visit(item.span)?;
                let set = self.c.target(item)?;
                if !self.c.is_a(set, "GEOMETRIC_SET") {
                    continue;
                }
                if memo.get(set).ok_or_else(internal)?.is_none() {
                    let mut contains = false;
                    let elements = self.field(set, "elements")?;
                    self.c.visit(elements.span)?;
                    for member in &elements.children {
                        self.c.visit(member.span)?;
                        contains = self.c.is_a(self.c.target(member)?, "SURFACE") || contains;
                    }
                    *memo.get_mut(set).ok_or_else(internal)? = Some(contains);
                }
                found = found || memo.get(set).copied().flatten().ok_or_else(internal)?;
            }
            if !found {
                return Err(self.failure(
                    r,
                    "items",
                    items,
                    "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION.WR7",
                    "at least one direct GEOMETRIC_SET containing a SURFACE element",
                    "no direct surface-bearing set",
                    &[],
                    false,
                    None,
                    Vec::new(),
                ));
            }
        }
        Ok(())
    }
}
pub(super) fn check(c: &mut Checker<'_>) -> Checked<()> {
    Geometry {
        c,
        postorder: Vec::new(),
        wire_prepared: false,
    }
    .run()
}
