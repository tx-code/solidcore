// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Exact finite LINE/PLANE region observations; no source or target rewrite.
//! Upstream: `StepPlanarGeometry.cpp` Edge/Axes/Face at `d8ed45a`.
//! Deviations: checked indexing and fallible exact operations in original order.
//! Loop comparisons are budgeted; radicals remain algebraic, no floating fallback.
mod topology;
mod units;
use super::validator::{add, cross, dot, squared, subtract, times, Checked, Failure, Validator};
use super::{EdgeEvidence, FaceEvidence, LoopEvidence, Metric, Rational as R, Status, Vector};
use crate::ValueKind;
use std::collections::{BTreeMap, BTreeSet};

type Point2 = [R; 2];
fn point(p: &[Point2], i: usize) -> Checked<Point2> {
    p.get(i).copied().ok_or(Failure::Infrastructure)
}
fn next(p: &[Point2], i: usize) -> Checked<Point2> {
    if p.is_empty() {
        return Err(Failure::Infrastructure);
    }
    point(p, (i + 1) % p.len())
}
fn orient([ax, ay]: Point2, [bx, by]: Point2, [cx, cy]: Point2) -> Checked<R> {
    Ok(bx
        .subtract(ax)?
        .multiply(cy.subtract(ay)?)?
        .subtract(by.subtract(ay)?.multiply(cx.subtract(ax)?)?)?)
}
fn on_segment(p: Point2, a: Point2, b: Point2) -> Checked<bool> {
    let [px, py] = p;
    let [ax, ay] = a;
    let [bx, by] = b;
    Ok(orient(a, b, p)? == R::ZERO
        && ax.min(bx) <= px
        && px <= ax.max(bx)
        && ay.min(by) <= py
        && py <= ay.max(by))
}
fn sign(a: R) -> i32 {
    match a.cmp(&R::ZERO) {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Equal => 0,
        std::cmp::Ordering::Greater => 1,
    }
}

fn intersects(a: Point2, b: Point2, c: Point2, d: Point2) -> Checked<bool> {
    // Contact counts as intersection. Exact orientation signs and closed segment
    // bounds intentionally exclude epsilon-based stitching or vertex movement.
    let x = orient(a, b, c)?;
    let y = orient(a, b, d)?;
    let z = orient(c, d, a)?;
    let w = orient(c, d, b)?;
    Ok((sign(x) * sign(y) < 0 && sign(z) * sign(w) < 0)
        || on_segment(c, a, b)?
        || on_segment(d, a, b)?
        || on_segment(a, c, d)?
        || on_segment(b, c, d)?)
}
fn contains(polygon: &[Point2], p: Point2) -> Checked<i32> {
    // Projected-basis coordinates, not target UV. Boundary (0) is distinct from
    // outside (-1); only strict winding interior (1) may qualify a source hole.
    let mut winding = 0;
    let [_, py] = p;
    for (i, a) in polygon.iter().copied().enumerate() {
        let b = next(polygon, i)?;
        let [_, ay] = a;
        let [_, by] = b;
        if on_segment(p, a, b)? {
            return Ok(0);
        }
        if ay <= py && py < by && orient(a, b, p)? > R::ZERO {
            winding += 1;
        }
        if by <= py && py < ay && orient(a, b, p)? < R::ZERO {
            winding -= 1;
        }
    }
    Ok(if winding == 0 { -1 } else { 1 })
}
fn metric(
    kind: &'static str,
    owner: usize,
    witnesses: Vec<usize>,
    values: impl IntoIterator<Item = (&'static str, R)>,
) -> Metric {
    Metric {
        kind,
        owner,
        witnesses,
        exact: values
            .into_iter()
            .map(|(name, value)| (name.into(), value))
            .collect(),
    }
}
fn plane_residual(p: Vector, origin: Vector, normal: Vector) -> Checked<R> {
    let distance = dot(subtract(p, origin)?, normal)?;
    Ok(distance.multiply(distance)?.divide(squared(normal)?)?)
}
fn projected_curve(v: Vector, a: Vector, d: Vector) -> Checked<Vector> {
    Ok(add(
        a,
        times(d, dot(subtract(v, a)?, d)?.divide(squared(d)?)?)?,
    )?)
}
fn sense_sign(value: bool) -> R {
    if value {
        R::ONE
    } else {
        R::ONE.negate()
    }
}

impl Validator<'_, '_> {
    fn edge(&mut self, r: usize, scale: R) -> Checked<()> {
        self.tick(r)?;
        let line = self.reference(r, "edge_geometry")?;
        let vector = self.reference(line, "dir")?;
        let direction = self.reference(vector, "orientation")?;
        let magnitude = R::decimal(&self.field(vector, "magnitude")?.text)?;
        if magnitude == R::ZERO {
            return Err(self.reject(
                "NonzeroDefiningLineRequired",
                "D4",
                vector,
                "magnitude",
                "VECTOR.WR1 permits zero; this LINE cannot define a finite edge direction",
            ));
        }
        let a = self.point(self.reference(line, "pnt")?)?;
        let d = self.direction(direction)?;
        let start = self.reference(r, "edge_start")?;
        let end = self.reference(r, "edge_end")?;
        let p = self.point(self.reference(start, "vertex_geometry")?)?;
        let q = self.point(self.reference(end, "vertex_geometry")?)?;
        let d2 = squared(d)?;
        let scale2 = scale.multiply(scale)?;
        let edge = EdgeEvidence {
            edge: r,
            line,
            start,
            end,
            same_sense: self.sense(r, "same_sense")?,
            direction_norm_squared: d2,
            t_start_coefficient: dot(subtract(p, a)?, d)?.divide(magnitude)?,
            t_end_coefficient: dot(subtract(q, a)?, d)?.divide(magnitude)?,
            length_squared_mm: squared(subtract(q, p)?)?.multiply(scale2)?,
        };
        self.out.evidence.edges.push(edge.clone());
        let threshold2 = self
            .out
            .quality
            .consistency_mm
            .multiply(self.out.quality.consistency_mm)?;
        let r0 = squared(cross(subtract(p, a)?, d)?)?
            .divide(d2)?
            .multiply(scale2)?;
        let r1 = squared(cross(subtract(q, a)?, d)?)?
            .divide(d2)?
            .multiply(scale2)?;
        self.out.evidence.metrics.push(metric(
            "endpoint_curve",
            r,
            vec![line, start, end],
            [
                ("start_distance_squared_mm", r0),
                ("end_distance_squared_mm", r1),
                ("threshold_squared_mm", threshold2),
            ],
        ));
        if r0 > threshold2 || r1 > threshold2 {
            return Err(self.reject("EndpointCurveMismatch","D4",line,"pnt","Source VP endpoints do not lie on defining LINE within the explicit consistency threshold"));
        }
        if edge.length_squared_mm == R::ZERO {
            return Err(self.reject(
                "NonzeroEdgeRequired",
                "D4",
                r,
                "edge_end",
                "Distinct topological endpoints must define a nonzero geometric segment",
            ));
        }
        if edge.t_end_coefficient == edge.t_start_coefficient
            || (edge.t_end_coefficient > edge.t_start_coefficient) != edge.same_sense
        {
            return Err(self.reject("EdgeCurveSenseMismatch","D4",r,"same_sense","Source EC.same_sense disagrees with the signed LINE parameter interval; endpoints are not swapped to repair it"));
        }
        Ok(())
    }
    fn axes(&mut self, placement: usize) -> Checked<(Vector, Vector, Vector)> {
        self.tick(placement)?;
        let axis = self.field(placement, "axis")?;
        let reference = self.field(placement, "ref_direction")?;
        let normal = if axis.kind == ValueKind::Unset {
            [R::ZERO, R::ZERO, R::ONE]
        } else {
            self.direction(self.target(axis)?)?
        };
        let [_, ny, nz] = normal;
        let mut x = if reference.kind == ValueKind::Unset {
            if ny == R::ZERO && nz == R::ZERO {
                [R::ZERO, R::ONE, R::ZERO]
            } else {
                [R::ONE, R::ZERO, R::ZERO]
            }
        } else {
            self.direction(self.target(reference)?)?
        };
        // Positive normalization radicals cancel in this exact Gram-Schmidt step.
        // Source UV mapping and metric factors remain explicit in face evidence.
        x = subtract(x, times(normal, dot(x, normal)?.divide(squared(normal)?)?)?)?;
        if squared(x)? == R::ZERO {
            return Err(self.fail("DerivedPlacementAxesIndeterminate","D4",placement,"ref_direction","first_proj_axis has no defined result; optional-axis WHEREs do not invent a fallback axis",Status::Indeterminate,&[],"",""));
        }
        Ok((x, cross(normal, x)?, normal))
    }
    fn face(&mut self, r: usize, scale: R) -> Checked<FaceEvidence> {
        self.tick(r)?;
        let plane = self.reference(r, "face_geometry")?;
        let placement = self.reference(plane, "position")?;
        let same_sense = self.sense(r, "same_sense")?;
        let origin_mm = times(self.point(self.reference(placement, "location")?)?, scale)?;
        let (x, y, normal) = self.axes(placement)?;
        let x2 = squared(x)?;
        let y2 = squared(y)?;
        let metric_squared = x2.multiply(y2)?;
        let mut face = FaceEvidence {
            face: r,
            plane,
            placement,
            same_sense,
            origin_mm,
            x,
            y,
            normal,
            source_length_scale_mm: scale,
            source_uv_formula:
                "u=projected[0]*sqrt(x_norm_squared)/source_length_scale_mm; v analogously with y",
            x_norm_squared: x2,
            y_norm_squared: y2,
            metric_squared,
            area_squared_mm: R::ZERO,
            centroid_mm: [R::ZERO; 3],
            minimum_mm: [R::ZERO; 3],
            maximum_mm: [R::ZERO; 3],
            loops: Vec::new(),
        };
        let threshold2 = self
            .out
            .quality
            .consistency_mm
            .multiply(self.out.quality.consistency_mm)?;
        for bound in self.refs(r, "bounds")? {
            let loop_record = self.reference(bound, "bound")?;
            let mut loop_e = LoopEvidence {
                bound,
                loop_record,
                outer: self.is(bound, "FACE_OUTER_BOUND"),
                orientation: self.sense(bound, "orientation")?,
                oriented_edges: self.refs(loop_record, "edge_list")?,
                edges: Vec::new(),
                vertices: Vec::new(),
                edge_orientations: Vec::new(),
                projected: Vec::new(),
                signed_area_coefficient: R::ZERO,
            };
            for oe in &loop_e.oriented_edges {
                self.tick(*oe)?;
                let (start, end) = self.ends(*oe)?;
                let ec = self.reference(*oe, "edge_element")?;
                let line = self.reference(ec, "edge_geometry")?;
                loop_e.edges.push(ec);
                loop_e.vertices.push(start);
                loop_e
                    .edge_orientations
                    .push(self.sense(*oe, "orientation")?);
                let p = times(
                    self.point(self.reference(start, "vertex_geometry")?)?,
                    scale,
                )?;
                let q = times(self.point(self.reference(end, "vertex_geometry")?)?, scale)?;
                let a = times(self.point(self.reference(line, "pnt")?)?, scale)?;
                let d =
                    self.direction(self.reference(self.reference(line, "dir")?, "orientation")?)?;
                let rp = plane_residual(p, origin_mm, normal)?;
                let rq = plane_residual(q, origin_mm, normal)?;
                let c0 = plane_residual(projected_curve(p, a, d)?, origin_mm, normal)?;
                let c1 = plane_residual(projected_curve(q, a, d)?, origin_mm, normal)?;
                self.out.evidence.metrics.push(metric(
                    "edge_surface",
                    r,
                    vec![ec, *oe, plane],
                    [
                        ("vertex_start_squared_mm", rp),
                        ("vertex_end_squared_mm", rq),
                        ("line_start_squared_mm", c0),
                        ("line_end_squared_mm", c1),
                        ("threshold_squared_mm", threshold2),
                    ],
                ));
                if rp.max(rq).max(c0).max(c1) > threshold2 {
                    return Err(self.reject("EdgeSurfaceMismatch","D4",plane,"position","Both source vertices and the finite LINE segment must agree with the source PLANE; maximum endpoint residual exceeds Q"));
                }
                let delta = subtract(p, origin_mm)?;
                loop_e
                    .projected
                    .push([dot(delta, x)?.divide(x2)?, dot(delta, y)?.divide(y2)?]);
            }
            let polygon = &loop_e.projected;
            let count = polygon.len();
            if count < 3 {
                return Err(self.reject(
                    "DegeneratePlanarBoundary",
                    "D4",
                    loop_record,
                    "edge_list",
                    "A simple finite planar region requires at least three sides",
                ));
            }
            let mut identities = BTreeSet::new();
            let mut twice_area = R::ZERO;
            for i in 0..count {
                self.tick(loop_record)?;
                let a = point(polygon, i)?;
                let b = next(polygon, i)?;
                let c = point(polygon, (i + 2) % count)?;
                let [ax, ay] = a;
                let [bx, by] = b;
                let [cx, cy] = c;
                if !identities.insert(*loop_e.vertices.get(i).ok_or(Failure::Infrastructure)?)
                    || a == b
                {
                    return Err(self.reject(
                        "DegeneratePlanarBoundary",
                        "D4",
                        loop_record,
                        "edge_list",
                        "Repeated vertex identity or coincident consecutive projected vertices",
                    ));
                }
                // Preserve left-associative intermediate capacity: total+a*b-b*a.
                twice_area = twice_area
                    .checked_add(ax.multiply(by)?)?
                    .subtract(bx.multiply(ay)?)?;
                let forward = bx
                    .subtract(ax)?
                    .multiply(cx.subtract(bx)?)?
                    .checked_add(by.subtract(ay)?.multiply(cy.subtract(by)?)?)?;
                if orient(a, b, c)? == R::ZERO && forward <= R::ZERO {
                    return Err(self.reject(
                        "PlanarBoundaryIntersection",
                        "D4",
                        loop_record,
                        "edge_list",
                        "Adjacent boundary segments overlap or reverse at their common vertex",
                    ));
                }
                for j in i + 1..count {
                    self.tick(loop_record)?;
                    if j == i + 1 || (i == 0 && j + 1 == count) {
                        continue;
                    }
                    if intersects(a, b, point(polygon, j)?, next(polygon, j)?)? {
                        return Err(self.reject("PlanarBoundaryIntersection","D4",loop_record,"edge_list","Nonadjacent boundary segments intersect or touch; no automatic splitting"));
                    }
                }
            }
            loop_e.signed_area_coefficient = twice_area
                .divide(R::new(2, 1)?)?
                .multiply(sense_sign(loop_e.orientation))?;
            if loop_e.signed_area_coefficient == R::ZERO {
                return Err(self.reject(
                    "DegeneratePlanarRegion",
                    "D4",
                    loop_record,
                    "edge_list",
                    "Source projected boundary has zero area",
                ));
            }
            let relative = loop_e
                .signed_area_coefficient
                .multiply(sense_sign(same_sense))?;
            self.out.evidence.metrics.push(metric(
                "bound_orientation",
                bound,
                vec![r, loop_record],
                [
                    ("signed_area_coefficient", loop_e.signed_area_coefficient),
                    ("metric_squared", metric_squared),
                    ("face_same_sense", sense_sign(same_sense)),
                ],
            ));
            if if loop_e.outer {
                relative <= R::ZERO
            } else {
                relative >= R::ZERO
            } {
                return Err(self.reject(if loop_e.outer{"FaceBoundaryOrShellOrientationMismatch"}else{"InnerBoundaryOrientationMismatch"},"D4",if loop_e.outer{r}else{bound},if loop_e.outer{"same_sense"}else{"orientation"},"Effective bound winding (OE then FACE_BOUND) must agree with FACE_SURFACE.same_sense for outer, and oppose it for inner boundaries"));
            }
            face.loops.push(loop_e);
        }
        let outer = face
            .loops
            .iter()
            .find(|l| l.outer)
            .ok_or(Failure::Infrastructure)?;
        for (i, a) in face.loops.iter().enumerate() {
            if !a.outer {
                for _ in &outer.projected {
                    self.tick(a.loop_record)?;
                }
                if contains(
                    &outer.projected,
                    *a.projected.first().ok_or(Failure::Infrastructure)?,
                )? != 1
                {
                    return Err(self.reject(
                        "InnerBoundaryOutsideOuter",
                        "D4",
                        a.bound,
                        "bound",
                        "Every inner boundary must lie strictly inside the explicit outer boundary",
                    ));
                }
            }
            for b in face.loops.iter().skip(i + 1) {
                for (u, pa) in a.projected.iter().copied().enumerate() {
                    for (v, pb) in b.projected.iter().copied().enumerate() {
                        self.tick(a.loop_record)?;
                        if intersects(pa, next(&a.projected, u)?, pb, next(&b.projected, v)?)? {
                            return Err(self.reject("PlanarBoundaryIntersection","D4",b.bound,"bound","Different bounds intersect or touch; containment alone is insufficient"));
                        }
                    }
                }
                if !a.outer && !b.outer {
                    for _ in 0..a.projected.len() + b.projected.len() {
                        self.tick(a.loop_record)?;
                    }
                    if contains(
                        &a.projected,
                        *b.projected.first().ok_or(Failure::Infrastructure)?,
                    )? == 1
                        || contains(
                            &b.projected,
                            *a.projected.first().ok_or(Failure::Infrastructure)?,
                        )? == 1
                    {
                        return Err(self.reject("AmbiguousNestedInnerBoundary","D4",b.bound,"bound","Nested holes require an explicit supported region interpretation; no silent parity rewrite"));
                    }
                }
            }
        }
        let mut total_twice = R::ZERO;
        let mut moment = [R::ZERO; 3];
        let mut first = true;
        for loop_e in &face.loops {
            for (i, a) in loop_e.projected.iter().copied().enumerate() {
                self.tick(loop_e.loop_record)?;
                let b = next(&loop_e.projected, i)?;
                let [ax, ay] = a;
                let [bx, by] = b;
                let pa = add(origin_mm, add(times(x, ax)?, times(y, ay)?)?)?;
                let pb = add(origin_mm, add(times(x, bx)?, times(y, by)?)?)?;
                let cross_term = ax
                    .multiply(by)?
                    .subtract(bx.multiply(ay)?)?
                    .multiply(sense_sign(loop_e.orientation))?;
                total_twice = total_twice.checked_add(cross_term)?;
                moment = add(
                    moment,
                    times(
                        add(subtract(pa, origin_mm)?, subtract(pb, origin_mm)?)?,
                        cross_term,
                    )?,
                )?;
                if first {
                    face.minimum_mm = pa;
                    face.maximum_mm = pa;
                    first = false;
                }
                for ((min, max), coord) in
                    face.minimum_mm.iter_mut().zip(&mut face.maximum_mm).zip(pa)
                {
                    *min = (*min).min(coord);
                    *max = (*max).max(coord);
                }
            }
        }
        if total_twice == R::ZERO {
            return Err(self.reject(
                "DegeneratePlanarRegion",
                "D4",
                r,
                "bounds",
                "Outer area minus inner areas must be nonzero",
            ));
        }
        face.area_squared_mm = total_twice
            .multiply(total_twice)?
            .divide(R::new(4, 1)?)?
            .multiply(metric_squared)?;
        face.centroid_mm = add(
            origin_mm,
            times(moment, R::ONE.divide(R::new(3, 1)?.multiply(total_twice)?)?)?,
        )?;
        Ok(face)
    }
    pub(super) fn geometry(&mut self) -> Checked<()> {
        let unit = self
            .sem
            .units()
            .length_unit
            .ok_or(Failure::Infrastructure)?;
        self.tick(unit)?;
        let mut memo = BTreeMap::new();
        let conversions = self.population("CONVERSION_BASED_UNIT")?;
        if !conversions.is_empty() {
            self.out.conversion_units = "Running";
        }
        let scale = if let Some(power) = self.sem.units().length_power10_mm {
            R::decimal(&format!("1E{power}"))?
        } else {
            self.unit_scale(unit, &mut memo)?
        };
        for unit in conversions {
            self.unit_scale(unit, &mut memo)?;
        }
        if self.out.conversion_units == "Running" {
            self.out.conversion_units = "Passed";
        }
        for placement in self.population("AXIS2_PLACEMENT_3D")? {
            self.axes(placement)?;
        }
        for edge in self.population("EDGE_CURVE")? {
            self.edge(edge, scale)?;
        }
        for face in self.population("ADVANCED_FACE")? {
            let evidence = self.face(face, scale)?;
            self.out.evidence.faces.push(evidence);
        }
        self.shell()
    }
}
