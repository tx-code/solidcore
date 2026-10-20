// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Source curve definition graphs and bounded-wireframe prerequisites.
//! Upstream: `source/STEP_SEMANTICS/src/StepGeometryDependencies.cpp`, curve methods.
//! Deviations: checked indices; explicit DFS/postorder and memoization retain
//! original source occurrence visits without recursive DAG/path expansion.
use super::{internal, Checked, Geometry, Predicate};

impl Geometry<'_, '_> {
    pub(super) fn composite_checks(
        &mut self,
        composites: &[usize],
        segments: &[usize],
    ) -> Checked<()> {
        let mut uses = vec![0_usize; self.c.source.records().len()];
        for r in composites {
            let segments = self.field(*r, "segments")?;
            self.count(segments.span, Predicate::Entity)?;
            let mut discontinuities = 0;
            let mut last_discontinuous = false;
            for occurrence in &segments.children {
                self.c.visit(occurrence.span)?;
                let segment = self.c.target(occurrence)?;
                *uses.get_mut(segment).ok_or_else(internal)? += 1;
                let transition = self.field(segment, "transition")?;
                self.c.visit(transition.span)?;
                last_discontinuous = transition.text == "DISCONTINUOUS";
                discontinuities += usize::from(last_discontinuous);
            }
            if discontinuities != usize::from(last_discontinuous) {
                return Err(self.failure(
                    *r,
                    "segments",
                    segments,
                    "COMPOSITE_CURVE.WR1",
                    "only an optional final discontinuous transition",
                    &discontinuities.to_string(),
                    &[],
                    false,
                    None,
                    Vec::new(),
                ));
            }
        }
        for r in segments {
            let parent = self.field(*r, "parent_curve")?;
            self.count(parent.span, Predicate::Entity)?;
            if !self.c.is_a(self.c.target(parent)?, "BOUNDED_CURVE") {
                return Err(self.failure(
                    *r,
                    "parent_curve",
                    parent,
                    "COMPOSITE_CURVE_SEGMENT.WR1",
                    "BOUNDED_CURVE",
                    "other CURVE",
                    &[],
                    false,
                    None,
                    Vec::new(),
                ));
            }
            self.count(parent.span, Predicate::Dependency)?;
            if uses.get(*r) == Some(&0) {
                return Err(self.failure(
                    *r,
                    "parent_curve",
                    parent,
                    "COMPOSITE_CURVE_SEGMENT.using_curves",
                    "BAG[1:?] inverse use",
                    "0",
                    &[],
                    false,
                    None,
                    Vec::new(),
                ));
            }
        }
        Ok(())
    }
    pub(super) fn composite_closure(
        &mut self,
        trimmed: &[usize],
        composites: &[usize],
    ) -> Checked<()> {
        let mut state = vec![0_u8; self.c.source.records().len()];
        let mut stack: Vec<(usize, usize)> = Vec::new();
        for root in trimmed.iter().chain(composites).copied() {
            if state.get(root) == Some(&2) {
                continue;
            }
            *state.get_mut(root).ok_or_else(internal)? = 1;
            stack.push((root, 0));
            while let Some((r, next)) = stack.last().copied() {
                let edge = if self.c.is_a(r, "TRIMMED_CURVE") && next == 0 {
                    stack.last_mut().ok_or_else(internal)?.1 += 1;
                    Some((r, "basis_curve", None))
                } else if self.c.is_a(r, "COMPOSITE_CURVE") {
                    let occurrences = self.field(r, "segments")?;
                    if let Some(occurrence) = occurrences.children.get(next) {
                        stack.last_mut().ok_or_else(internal)?.1 += 1;
                        self.c.visit(occurrence.span)?;
                        Some((
                            self.c.target(occurrence)?,
                            "parent_curve",
                            Some(occurrence.span),
                        ))
                    } else {
                        None
                    }
                } else {
                    None
                };
                let Some((owner, field, through)) = edge else {
                    *state.get_mut(r).ok_or_else(internal)? = 2;
                    self.postorder.push(r);
                    stack.pop();
                    continue;
                };
                let value = self.field(owner, field)?;
                self.count(value.span, Predicate::Dependency)?;
                let target = self.c.target(value)?;
                if state.get(target) == Some(&1) {
                    return Err(self.failure(
                        owner,
                        field,
                        value,
                        "curve definition",
                        "finite curve definition graph",
                        "cycle",
                        &[],
                        true,
                        Some("project necessary geometry dependency closure"),
                        through.into_iter().collect(),
                    ));
                }
                if state.get(target) == Some(&0) {
                    *state.get_mut(target).ok_or_else(internal)? = 1;
                    stack.push((target, 0));
                }
            }
        }
        Ok(())
    }
    pub(super) fn trimmed_closure(&mut self, roots: &[usize]) -> Checked<()> {
        if roots.is_empty() {
            return Ok(());
        }
        let mut state = vec![0_u8; self.c.source.records().len()];
        let mut path = Vec::new();
        for root in roots {
            if state.get(*root) == Some(&2) {
                continue;
            }
            path.clear();
            let mut current = *root;
            while self.c.is_a(current, "TRIMMED_CURVE") && state.get(current) != Some(&2) {
                let value = self.field(current, "basis_curve")?;
                self.count(value.span, Predicate::Entity)?;
                if state.get(current) == Some(&1) {
                    return Err(self.failure(
                        current,
                        "basis_curve",
                        value,
                        "TRIMMED_CURVE.basis_curve",
                        "finite basis dependency chain",
                        "cycle",
                        &[],
                        true,
                        Some("project necessary geometry dependency closure"),
                        Vec::new(),
                    ));
                }
                *state.get_mut(current).ok_or_else(internal)? = 1;
                path.push(current);
                current = self.c.target(value)?;
            }
            for r in &path {
                *state.get_mut(*r).ok_or_else(internal)? = 2;
            }
        }
        Ok(())
    }
    fn direct_wire_curve(&self, r: usize) -> bool {
        self.c.is_a(r, "B_SPLINE_CURVE") || self.c.is_a(r, "CIRCLE") || self.c.is_a(r, "ELLIPSE")
    }
    fn prepare_wire_validity(&mut self, memo: &mut [Option<bool>]) -> Checked<()> {
        for i in 0..self.postorder.len() {
            let r = *self.postorder.get(i).ok_or_else(internal)?;
            self.c.visit(self.c.record(r)?.span)?;
            let mut valid = self.direct_wire_curve(r);
            if self.c.is_a(r, "TRIMMED_CURVE") {
                let basis = self.field(r, "basis_curve")?;
                self.c.visit(basis.span)?;
                let target = self.c.target(basis)?;
                valid = self.c.is_a(target, "LINE") || memo.get(target) == Some(&Some(true));
            }
            if self.c.is_a(r, "COMPOSITE_CURVE") {
                valid = true;
                for occurrence in &self.field(r, "segments")?.children {
                    self.c.visit(occurrence.span)?;
                    let parent = self.field(self.c.target(occurrence)?, "parent_curve")?;
                    self.c.visit(parent.span)?;
                    valid = (memo.get(self.c.target(parent)?) == Some(&Some(true))) && valid;
                }
            }
            *memo.get_mut(r).ok_or_else(internal)? = Some(valid);
        }
        self.wire_prepared = true;
        Ok(())
    }
    fn valid_wire_curve(&mut self, r: usize, memo: &mut [Option<bool>]) -> Checked<bool> {
        if !self.postorder.is_empty() && !self.wire_prepared {
            self.prepare_wire_validity(memo)?;
        }
        let mut path = Vec::new();
        let mut current = r;
        while memo.get(current).ok_or_else(internal)?.is_none() {
            self.c.visit(self.c.record(current)?.span)?;
            if self.direct_wire_curve(current) {
                *memo.get_mut(current).ok_or_else(internal)? = Some(true);
                break;
            }
            if !self.c.is_a(current, "TRIMMED_CURVE") {
                *memo.get_mut(current).ok_or_else(internal)? = Some(false);
                break;
            }
            let basis = self.field(current, "basis_curve")?;
            self.c.visit(basis.span)?;
            let next = self.c.target(basis)?;
            if self.c.is_a(next, "LINE") {
                *memo.get_mut(current).ok_or_else(internal)? = Some(true);
                break;
            }
            path.push(current);
            current = next;
        }
        let value = memo.get(current).copied().flatten().ok_or_else(internal)?;
        for parent in path {
            *memo.get_mut(parent).ok_or_else(internal)? = Some(value);
        }
        memo.get(r).copied().flatten().ok_or_else(internal)
    }
    pub(super) fn wireframes(&mut self) -> Checked<()> {
        let catalog = self.c.schema()?;
        if catalog.find("GEOMETRIC_CURVE_SET").is_none()
            || catalog
                .find("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION")
                .is_none()
        {
            return Ok(());
        }
        for r in self.population("GEOMETRIC_CURVE_SET")? {
            let elements = self.field(r, "elements")?;
            self.count(elements.span, Predicate::Entity)?;
            for (i, member) in elements.children.iter().enumerate() {
                self.c.visit(member.span)?;
                if self.c.is_a(self.c.target(member)?, "SURFACE") {
                    return Err(self.failure(
                        r,
                        "elements",
                        member,
                        "GEOMETRIC_CURVE_SET.WR1",
                        "POINT or CURVE",
                        "SURFACE",
                        &[i + 1],
                        false,
                        None,
                        Vec::new(),
                    ));
                }
            }
        }
        let reps = self.population("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION")?;
        if reps.is_empty() {
            return Ok(());
        }
        let mut memo = vec![None; self.c.source.records().len()];
        let prefix = "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION";
        for r in reps {
            let items = self.field(r, "items")?;
            let mut sets = 0;
            self.count(items.span, Predicate::Entity)?;
            for (i, item) in items.children.iter().enumerate() {
                self.c.visit(item.span)?;
                let target = self.c.target(item)?;
                let categories = usize::from(self.c.is_a(target, "GEOMETRIC_CURVE_SET"))
                    + usize::from(self.c.is_a(target, "AXIS2_PLACEMENT_3D"));
                if categories != 1 {
                    return Err(self.failure(
                        r,
                        "items",
                        item,
                        &format!("{prefix}.WR1"),
                        "one admitted GCS/AXIS2_PLACEMENT_3D category",
                        "other RI",
                        &[i + 1],
                        false,
                        None,
                        Vec::new(),
                    ));
                }
                sets += usize::from(self.c.is_a(target, "GEOMETRIC_CURVE_SET"));
            }
            self.count(items.span, Predicate::Entity)?;
            if sets == 0 {
                return Err(self.failure(
                    r,
                    "items",
                    items,
                    &format!("{prefix}.WR2"),
                    "at least one GEOMETRIC_CURVE_SET",
                    "placements only",
                    &[],
                    false,
                    None,
                    Vec::new(),
                ));
            }
            for clause in ["WR3", "WR4", "WR5"] {
                self.count(items.span, Predicate::Entity)?;
                for item in &items.children {
                    let set = self.c.target(item)?;
                    if !self.c.is_a(set, "GEOMETRIC_CURVE_SET") {
                        continue;
                    }
                    for (i, member) in self.field(set, "elements")?.children.iter().enumerate() {
                        self.c.visit(member.span)?;
                        let geometry = self.c.target(member)?;
                        let mut valid = true;
                        if clause == "WR3" && self.c.is_a(geometry, "CURVE") {
                            valid = self.valid_wire_curve(geometry, &mut memo)?;
                        }
                        if clause == "WR4" && self.c.is_a(geometry, "POINT") {
                            valid = self.c.is_a(geometry, "CARTESIAN_POINT");
                        }
                        if clause == "WR5" && self.c.is_a(geometry, "CONIC") {
                            let position = self.field(geometry, "position")?;
                            self.c.visit(position.span)?;
                            valid = self.c.is_a(self.c.target(position)?, "AXIS2_PLACEMENT_3D");
                        }
                        if !valid {
                            return Err(self.failure(
                                set,
                                "elements",
                                member,
                                &format!("{prefix}.{clause}"),
                                "valid fixed-source wireframe predicate",
                                "FALSE",
                                &[i + 1],
                                false,
                                None,
                                vec![item.span],
                            ));
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
