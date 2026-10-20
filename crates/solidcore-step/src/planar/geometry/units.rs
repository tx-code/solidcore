// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded SI-rooted conversion interpretation with exact memoized scales.
//! Upstream: `StepPlanarGeometry.cpp::UnitScale` at `d8ed45a`.
//! Deviations: checked iterative path, no input-driven recursion. Original
//! dimensions/factors remain source-owned; names never establish a unit scale.
use super::{metric, Checked, Failure, Status, Validator, R};
use crate::planar::UnitConversion;
use std::collections::BTreeMap;
impl Validator<'_, '_> {
    pub(super) fn unit_scale(&mut self, root: usize, memo: &mut BTreeMap<usize, R>) -> Checked<R> {
        let mut path = Vec::new();
        let mut active = BTreeMap::new();
        let mut unit = root;
        while !memo.contains_key(&unit) {
            self.tick(unit)?;
            if let Some(repeated) = active.get(&unit).copied() {
                let mut witnesses = Vec::new();
                let mut cycle = String::new();
                for source_unit in path.get(repeated..).ok_or(Failure::Infrastructure)? {
                    let factor = self.reference(*source_unit, "conversion_factor")?;
                    witnesses.extend([*source_unit, factor]);
                    cycle.push_str(&self.record(*source_unit)?.id);
                    cycle.push_str("->");
                    cycle.push_str(&self.record(factor)?.id);
                    cycle.push_str("->");
                }
                witnesses.push(unit);
                cycle.push_str(&self.record(unit)?.id);
                let closing = self.reference(
                    *path.last().ok_or(Failure::Infrastructure)?,
                    "conversion_factor",
                )?;
                self.out
                    .evidence
                    .metrics
                    .push(metric("unit_cycle", unit, witnesses.clone(), []));
                return Err(self.fail("CyclicUnitDefinition","D4",closing,"unit_component","Project unit interpretation requires an acyclic chain to a positive SI root; dimensions alone do not establish a scale",Status::Rejected,&witnesses,"acyclic positive SI-rooted unit chain",&cycle));
            }
            let mut si = false;
            for definition in &self.sem.units().si_definitions {
                if definition.record_index == unit {
                    if definition.quantity != "Length" {
                        return Err(Failure::Numeric("UnsupportedConversionQuantity"));
                    }
                    memo.insert(
                        unit,
                        R::decimal(&format!("1E{}", definition.canonical_power10))?,
                    );
                    si = true;
                    break;
                }
            }
            if si {
                break;
            }
            if !self.is(unit, "CONVERSION_BASED_UNIT") || !self.is(unit, "LENGTH_UNIT") {
                return Err(Failure::Numeric("UnsupportedConversionUnitDefinition"));
            }
            active.insert(unit, path.len());
            path.push(unit);
            let factor = self.reference(unit, "conversion_factor")?;
            self.tick(factor)?;
            unit = self.reference(factor, "unit_component")?;
        }
        while let Some(current) = path.pop() {
            let factor = self.reference(current, "conversion_factor")?;
            let base = self.reference(factor, "unit_component")?;
            let value = self.field(factor, "value_component")?;
            let numeric = R::decimal(&value.children.first().ok_or(Failure::Infrastructure)?.text)?;
            let base_scale = *memo.get(&base).ok_or(Failure::Infrastructure)?;
            let scale = numeric.multiply(base_scale)?;
            let mut witnesses = vec![current, factor, base];
            let dimensions = self.unit_dimensions(current, &mut witnesses)?;
            self.out.evidence.metrics.push(metric(
                "unit_scale",
                current,
                witnesses.clone(),
                [
                    ("factor_value", numeric),
                    ("base_scale_mm", base_scale),
                    ("scale_mm", scale),
                ],
            ));
            if scale <= R::ZERO {
                return Err(self.fail("UnitScaleNotPositive","D4",factor,"value_component","Project canonical unit scale must be positive and reversible; base LENGTH_MEASURE has no positivity WHERE",Status::Rejected,&witnesses,">0 mm/source-length-unit",&format!("{}/{}",scale.numerator(),scale.denominator())));
            }
            memo.insert(current, scale);
            self.out.evidence.conversions.push(UnitConversion {
                unit: current,
                dimensions: self.reference(current, "dimensions")?,
                factor,
                base_unit: base,
                value_type: value.text.clone(),
                factor_value: numeric,
                scale_mm: scale,
                exponents: dimensions,
            });
        }
        memo.get(&root).copied().ok_or(Failure::Infrastructure)
    }
}
