// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Immutable profile-local type and attribute descriptors.
//! Upstream: `source/STEP_SEMANTICS/src/StepCatalog.h`, `StepCatalog.cpp`.
//! Rust Catalog/Field/Membership map to their identically named C++ descriptors.
//! Deviations: checked access, compile-time generated Rust arrays; no allocation
//! or mutable singleton on first lookup. Registration never admits foreign types.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Kind {
    String,
    Real,
    Integer,
    Boolean,
    Logical,
    Enumeration,
    Reference,
    Aggregate,
    Measure,
    SizeSelect,
    TrimmingSelect,
    Number,
    MarkerSelect,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Unique {
    None,
    Set,
    List,
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Dimension {
    pub minimum: usize,
    pub maximum: Option<usize>,
    pub unique: bool,
    pub set: bool,
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Field {
    pub name: &'static str,
    pub declared_type: &'static str,
    pub kind: Kind,
    pub optional: bool,
    pub element: Kind,
    pub minimum: usize,
    pub maximum: Option<usize>,
    pub unique: Unique,
    pub nested: &'static [Dimension],
}
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Membership(pub [u64; 4]);
impl Membership {
    pub fn contains(self, id: usize) -> bool {
        self.0
            .get(id / 64)
            .is_some_and(|word| word & (1_u64 << (id % 64)) != 0)
    }
    pub fn union(&mut self, other: Self) {
        for (a, b) in self.0.iter_mut().zip(other.0) {
            *a |= b;
        }
    }
}
pub(super) struct Declaration {
    pub name: &'static str,
    pub simple: bool,
    pub fields: &'static [Field],
    pub ancestors: Membership,
    pub flattened: &'static [(usize, usize)],
}
pub(super) struct Slot {
    pub owner: usize,
    pub origin: usize,
    pub field: usize,
}
pub(super) struct Redeclaration {
    pub slot: Slot,
    pub declared_type: &'static str,
    pub kind: Kind,
}
pub(super) struct Catalog {
    pub kind: &'static str,
    pub profile: &'static str,
    pub schema_sha256: &'static str,
    pub declarations: &'static [Declaration],
    pub name_index: &'static [(&'static str, usize)],
    pub selects: &'static [(&'static str, &'static [&'static str])],
    pub enumerations: &'static [(&'static str, &'static [&'static str])],
    pub complex: &'static [&'static [&'static str]],
    pub measures: &'static [&'static str],
    pub derived: &'static [Slot],
    pub explicit: &'static [Redeclaration],
    pub exclusive: &'static [&'static [usize]],
    pub prefixes: &'static [(&'static str, i32)],
}
impl Catalog {
    pub fn find(&self, name: &str) -> Option<usize> {
        self.name_index
            .binary_search_by(|entry| entry.0.cmp(name))
            .ok()
            .and_then(|i| self.name_index.get(i))
            .map(|v| v.1)
    }
    pub fn is_a(&self, membership: Membership, name: &str) -> bool {
        self.find(name).is_some_and(|id| membership.contains(id))
    }
    pub fn matches(&self, membership: Membership, expected: &str) -> bool {
        if let Some((_, branches)) = self.selects.iter().find(|v| v.0 == expected) {
            return branches.iter().any(|name| self.is_a(membership, name));
        }
        self.is_a(membership, expected)
    }
    pub fn enumeration(&self, name: &str, value: &str) -> bool {
        if name == "SI_PREFIX" {
            return self.prefixes.iter().any(|p| p.0 == value);
        }
        self.enumerations
            .iter()
            .find(|v| v.0 == name)
            .is_some_and(|v| v.1.contains(&value))
    }
    pub fn field(
        &self,
        membership: Membership,
        origin: usize,
        local: usize,
    ) -> Option<(Field, &'static str, bool)> {
        let mut field = *self.declarations.get(origin)?.fields.get(local)?;
        // C++ ResolveField preserves registry priority: DERIVE redeclarations
        // win over explicit narrowing, and require a transmitted '*' marker.
        if let Some(slot) = self
            .derived
            .iter()
            .find(|s| s.origin == origin && s.field == local && membership.contains(s.owner))
        {
            return Some((field, self.declarations.get(slot.owner)?.name, true));
        }
        if let Some(v) = self.explicit.iter().find(|v| {
            v.slot.origin == origin && v.slot.field == local && membership.contains(v.slot.owner)
        }) {
            field.declared_type = v.declared_type;
            field.kind = v.kind;
            return Some((field, self.declarations.get(v.slot.owner)?.name, false));
        }
        Some((field, "", false))
    }
    pub fn authority(&self) -> &'static str {
        match self.kind {
            "Ap203e2Planar" | "Ap203e2GeneralSource" => "N2635",
            "Ap203N916Source" => "STEPcode modified N916",
            _ => "AP214E3_2010.exp",
        }
    }
    pub fn ap214(&self) -> bool {
        matches!(self.kind, "Ap214e3Source" | "Ap214MbxSource")
    }
}
