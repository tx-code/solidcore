// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Compile-time identity/subject/execution checks, independent of storage order.
//! Upstream: `StepRuleCatalog.h::LocalRegistryIsValid/GlobalRegistryIsValid`.
//! Deviations: const slice descent; no ordinal dispatch or serialized private IDs.
use super::catalog::{self, Subject};

const fn same(a: &str, b: &str) -> bool {
    let mut a = a.as_bytes();
    let mut b = b.as_bytes();
    loop {
        match (a.split_first(), b.split_first()) {
            (Some((x, tail_a)), Some((y, tail_b))) => {
                if *x != *y {
                    return false;
                }
                a = tail_a;
                b = tail_b;
            }
            (None, None) => return true,
            _ => return false,
        }
    }
}
pub(super) const fn valid() -> bool {
    if catalog::ALL_LOCAL.len() != 80
        || catalog::ENTITY_ORDER.len() != 77
        || catalog::ALL_GLOBAL.len() != 47
        || catalog::GLOBAL_ORDER.len() != 47
    {
        return false;
    }
    if !matches!(
        catalog::local(catalog::DIMENSION).subject,
        Subject::ContextAttribute
    ) || catalog::LENGTH_PLANS.len() != 3
    {
        return false;
    }
    let mut plans = catalog::LENGTH_PLANS;
    while let Some((plan, tail)) = plans.split_first() {
        let mut others = tail;
        while let Some((other, rest)) = others.split_first() {
            if same(plan.0, other.0) {
                return false;
            }
            others = rest;
        }
        let mut predicates = plan.1;
        while let Some((id, rest)) = predicates.split_first() {
            if !matches!(catalog::local(*id).subject, Subject::MeasureValue) {
                return false;
            }
            let mut remaining = rest;
            while let Some((other, tail)) = remaining.split_first() {
                if *id as u16 == *other as u16 {
                    return false;
                }
                remaining = tail;
            }
            predicates = rest;
        }
        // Preserve inherited predicate order, not just the set of declarations.
        if same(plan.0, "LENGTH_MEASURE") {
            if !plan.1.is_empty() {
                return false;
            }
        } else if same(plan.0, "NON_NEGATIVE_LENGTH_MEASURE") {
            let Some((id, rest)) = plan.1.split_first() else {
                return false;
            };
            if *id as u16 != catalog::LocalId::NonNegativeLengthMeasureWr1 as u16
                || !rest.is_empty()
            {
                return false;
            }
        } else if same(plan.0, "POSITIVE_LENGTH_MEASURE") {
            let Some((id, rest)) = plan.1.split_first() else {
                return false;
            };
            let Some((next, rest)) = rest.split_first() else {
                return false;
            };
            if *id as u16 != catalog::LocalId::NonNegativeLengthMeasureWr1 as u16
                || *next as u16 != catalog::LocalId::PositiveLengthMeasureWr1 as u16
                || !rest.is_empty()
            {
                return false;
            }
        } else {
            return false;
        }
        plans = tail;
    }
    let mut local = catalog::ALL_LOCAL;
    while let Some((id, rest)) = local.split_first() {
        let c = catalog::local(*id);
        let mut others = rest;
        while let Some((other, tail)) = others.split_first() {
            let d = catalog::local(*other);
            if *id as u16 == *other as u16
                || (same(c.type_name, d.type_name)
                    && same(c.kind, d.kind)
                    && same(c.label, d.label))
            {
                return false;
            }
            others = tail;
        }
        let mut order = catalog::ENTITY_ORDER;
        let mut count = 0;
        while let Some((executed, tail)) = order.split_first() {
            if *executed as u16 == *id as u16 {
                count += 1;
            }
            order = tail;
        }
        match c.subject {
            Subject::EntityPopulation => {
                if count != 1 {
                    return false;
                }
            }
            Subject::MeasureValue => {
                if count != 0 || !same(c.kind, "TYPE_WHERE") {
                    return false;
                }
            }
            Subject::ContextAttribute => {
                if count != 0 || *id as u16 != catalog::DIMENSION as u16 {
                    return false;
                }
            }
        }
        local = rest;
    }
    let mut global = catalog::ALL_GLOBAL;
    while let Some((id, rest)) = global.split_first() {
        let c = catalog::global(*id);
        if c.types.is_empty() {
            return false;
        }
        let mut others = rest;
        while let Some((other, tail)) = others.split_first() {
            if *id as u16 == *other as u16 || same(c.name, catalog::global(*other).name) {
                return false;
            }
            others = tail;
        }
        let mut order = catalog::GLOBAL_ORDER;
        let mut count = 0;
        while let Some((executed, tail)) = order.split_first() {
            if *executed as u16 == *id as u16 {
                count += 1;
            }
            order = tail;
        }
        if count != 1 {
            return false;
        }
        global = rest;
    }
    true
}
