// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Source B-spline direction and rational-weight prerequisites, preserving U/V.
//! Upstream: `source/STEP_SEMANTICS/src/StepGeometryDependencies.cpp`, Direction/Weights.
//! Deviations: checked indices; original knots, multiplicities and weight lexemes
//! are never expanded, rounded, normalized, padded, truncated or transposed.
use super::{compare, internal, Checked, Geometry, Predicate};
#[derive(Clone, Copy)]
pub(super) struct Direction {
    degree: &'static str,
    multiplicities: &'static str,
    knots: &'static str,
    count_clause: &'static str,
    parameter_clause: &'static str,
}
pub(super) const CURVE: Direction = Direction {
    degree: "degree",
    multiplicities: "knot_multiplicities",
    knots: "knots",
    count_clause: "B_SPLINE_CURVE_WITH_KNOTS.WR2",
    parameter_clause: "B_SPLINE_CURVE_WITH_KNOTS.WR1",
};
pub(super) const U: Direction = Direction {
    degree: "u_degree",
    multiplicities: "u_multiplicities",
    knots: "u_knots",
    count_clause: "B_SPLINE_SURFACE_WITH_KNOTS.WR3",
    parameter_clause: "B_SPLINE_SURFACE_WITH_KNOTS.WR1",
};
pub(super) const V: Direction = Direction {
    degree: "v_degree",
    multiplicities: "v_multiplicities",
    knots: "v_knots",
    count_clause: "B_SPLINE_SURFACE_WITH_KNOTS.WR4",
    parameter_clause: "B_SPLINE_SURFACE_WITH_KNOTS.WR2",
};
impl Geometry<'_, '_> {
    pub(super) fn direction(&mut self, r: usize, controls: usize, d: Direction) -> Checked<()> {
        let mult = self.field(r, d.multiplicities)?;
        let knots = self.field(r, d.knots)?;
        self.count(mult.span, Predicate::Entity)?;
        if mult.children.len() != knots.children.len() {
            return Err(self.failure(
                r,
                d.multiplicities,
                mult,
                d.count_clause,
                &knots.children.len().to_string(),
                &mult.children.len().to_string(),
                &[],
                false,
                None,
                Vec::new(),
            ));
        }
        let degree = self.field(r, d.degree)?;
        self.count(degree.span, Predicate::Entity)?;
        let degree_count = super::super::number::index(
            &degree.text,
            controls.checked_sub(1).ok_or_else(internal)? as u64,
        );
        let Some(n) = degree_count.filter(|n| *n >= 1) else {
            return Err(self.failure(
                r,
                d.degree,
                degree,
                d.parameter_clause,
                "1 <= degree <= control count - 1",
                &degree.text,
                &[],
                false,
                None,
                Vec::new(),
            ));
        };
        let n = n as usize;
        let required = controls + n + 1;
        let mut sum = 0;
        for (i, value) in mult.children.iter().enumerate() {
            self.c.visit(value.span)?;
            let maximum = if i == 0 || i + 1 == mult.children.len() {
                n + 1
            } else {
                n
            };
            let value_count = super::super::number::index(&value.text, maximum as u64);
            let Some(count) =
                value_count.filter(|count| *count >= 1 && *count <= (required - sum) as u64)
            else {
                return Err(self.failure(
                    r,
                    d.multiplicities,
                    value,
                    d.parameter_clause,
                    "positive bounded knot multiplicity with required total",
                    &value.text,
                    &[i + 1],
                    false,
                    None,
                    Vec::new(),
                ));
            };
            sum += count as usize;
        }
        if sum != required {
            return Err(self.failure(
                r,
                d.multiplicities,
                mult,
                d.parameter_clause,
                &required.to_string(),
                &sum.to_string(),
                &[],
                false,
                None,
                Vec::new(),
            ));
        }
        for (i, value) in knots.children.iter().enumerate() {
            self.c.visit(value.span)?;
            if let Some(previous) = i.checked_sub(1).and_then(|p| knots.children.get(p)) {
                if compare(&previous.text, &value.text) >= 0 {
                    return Err(self.failure(
                        r,
                        d.knots,
                        value,
                        d.parameter_clause,
                        "strictly increasing distinct source knots",
                        &value.text,
                        &[i + 1],
                        false,
                        None,
                        Vec::new(),
                    ));
                }
            }
        }
        Ok(())
    }
    pub(super) fn weights(&mut self, r: usize, surface: bool) -> Checked<()> {
        let data = self.field(r, "weights_data")?;
        let controls = self.field(r, "control_points_list")?;
        let prefix = if surface {
            "RATIONAL_B_SPLINE_SURFACE"
        } else {
            "RATIONAL_B_SPLINE_CURVE"
        };
        self.count(data.span, Predicate::Entity)?;
        if data.children.len() != controls.children.len() {
            return Err(self.failure(
                r,
                "weights_data",
                data,
                &format!("{prefix}.WR1"),
                &controls.children.len().to_string(),
                &data.children.len().to_string(),
                &[],
                false,
                None,
                Vec::new(),
            ));
        }
        if surface {
            let width = controls
                .children
                .first()
                .ok_or_else(internal)?
                .children
                .len();
            for (i, row) in data.children.iter().enumerate() {
                self.c.visit(row.span)?;
                if row.children.len() != width {
                    return Err(self.failure(
                        r,
                        "weights_data",
                        row,
                        &format!("{prefix}.{}", if i == 0 { "WR1" } else { "weights DERIVE" }),
                        &width.to_string(),
                        &row.children.len().to_string(),
                        &[i + 1],
                        i != 0,
                        None,
                        Vec::new(),
                    ));
                }
            }
        }
        self.count(data.span, Predicate::Entity)?;
        for (i, value) in data.children.iter().enumerate() {
            if surface {
                self.c.visit(value.span)?;
                for (j, weight) in value.children.iter().enumerate() {
                    self.positive_weight(r, weight, &format!("{prefix}.WR2"), &[i + 1, j + 1])?;
                }
            } else {
                self.positive_weight(r, value, &format!("{prefix}.WR2"), &[i + 1])?;
            }
        }
        Ok(())
    }
    fn positive_weight(
        &mut self,
        r: usize,
        value: &crate::Value,
        clause: &str,
        path: &[usize],
    ) -> Checked<()> {
        self.c.visit(value.span)?;
        if compare(&value.text, "0.") <= 0 {
            return Err(self.failure(
                r,
                "weights_data",
                value,
                clause,
                ">0",
                &value.text,
                path,
                false,
                None,
                Vec::new(),
            ));
        }
        Ok(())
    }
}
