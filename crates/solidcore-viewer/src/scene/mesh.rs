// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded viewer mesh, separate from SDK tessellation and import qualification.
//!
//! Native topology and geometry are borrowed only. A shared native-edge parameter
//! table feeds every face incidence; each incidence keeps its own UV trim. All
//! marked boundary intervals propagate to all incidences and neighbouring
//! triangles share midpoint indices. Planes need only shared-boundary splits.
//! The chord/normal screens are display samples, not continuous error proofs.
use super::{native_point, DisplayOptions, EdgeSample, Result};
use glam::{DVec2, DVec3};
use solidcore::math::Vec2;
use solidcore::nurbs::b_spline_surface::SurfParamType;
use solidcore::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
use solidcore::nurbs::knots::Side;
use solidcore::surface::surface::{Surface, SurfaceKind};
use solidcore::topology::{edgeuse::p1 as eu, EdgeId, EdgeuseId, FaceId, LoopuseStart, VertexId};
use solidcore_step_native::Candidate;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy)]
struct Node {
    uv: DVec2,
    point: DVec3,
    normal: Option<DVec3>,
    vertex: Option<VertexId>,
}
#[derive(Clone, Copy)]
struct Boundary {
    edge: EdgeId,
    use_: EdgeuseId,
    ta: f64,
    tb: f64,
}
struct FaceMesh {
    face: FaceId,
    nodes: Vec<Node>,
    triangles: Vec<[usize; 3]>,
    constraints: BTreeMap<(usize, usize), Boundary>,
    uv_scale: DVec2,
    span: DVec2,
    curved: bool,
}
struct Work<'a> {
    options: DisplayOptions,
    chord_mm: f64,
    evaluations: usize,
    nodes: usize,
    progress: &'a dyn Fn(&str) -> Result<()>,
}
impl Work<'_> {
    fn step(&mut self) -> Result<()> {
        self.evaluations = self
            .evaluations
            .checked_add(1)
            .ok_or("Display evaluation count overflow")?;
        if self.evaluations > self.options.max_evaluations {
            return Err("Display evaluation budget exceeded".into());
        }
        if self.evaluations.is_multiple_of(512) {
            (self.progress)("6 / 6  Refining native surfaces for display")?;
        }
        Ok(())
    }
    fn node(&mut self) -> Result<()> {
        self.nodes = self
            .nodes
            .checked_add(1)
            .ok_or("Display vertex count overflow")?;
        if self.nodes > self.options.max_vertices {
            return Err("Display vertex budget exceeded".into());
        }
        Ok(())
    }
    fn triangles(&self, count: usize) -> Result<()> {
        if count > self.options.max_triangles
            || count
                .checked_mul(3)
                .is_none_or(|n| n > self.options.max_vertices)
        {
            Err("Display triangle/vertex budget exceeded".into())
        } else {
            Ok(())
        }
    }
}
fn vec2(point: solidcore::math::Vec3) -> Result<DVec2> {
    let uv = DVec2::new(point.x, point.y);
    if uv.is_finite() {
        Ok(uv)
    } else {
        Err("Nonfinite native UV trim coordinate".into())
    }
}
fn native_uv(uv: DVec2) -> Vec2 {
    Vec2::new(uv.x, uv.y)
}
fn unit(vector: DVec3) -> Option<DVec3> {
    let scale = vector.abs().max_element();
    if !scale.is_finite() || scale <= 0. {
        return None;
    }
    let scaled = vector / scale;
    let length = scaled.length();
    (length.is_finite() && length > 0.).then(|| scaled / length)
}
fn surface(owner: &Candidate, face: FaceId) -> Result<&Surface> {
    owner
        .brep()
        .face(face)
        .map_err(|e| e.to_string())?
        .surface
        .as_ref()
        .ok_or_else(|| "Missing native face surface".into())
}
fn sample(owner: &Candidate, face: FaceId, uv: DVec2, work: &mut Work<'_>) -> Result<Node> {
    work.step()?;
    let values = surface(owner, face)?
        .evaluate(
            owner.kernel(),
            native_uv(uv),
            1,
            1,
            EvaluateFlags {
                u_side: Side::Right,
                v_side: Side::Right,
                only_upper_half: true,
                non_zero_tangents: false,
            },
        )
        .map_err(|e| format!("Native display surface evaluation: {e}"))?;
    let read = |u: usize, v: usize| -> Result<DVec3> {
        native_point(
            *values
                .get(u)
                .and_then(|row| row.get(v))
                .ok_or("Short native derivative matrix")?,
        )
    };
    let point = read(0, 0)?;
    let du = read(1, 0)?;
    let dv = read(0, 1)?;
    let normal = unit(du)
        .zip(unit(dv))
        .and_then(|(du, dv)| unit(du.cross(dv)));
    Ok(Node {
        uv,
        point,
        normal,
        vertex: None,
    })
}

fn boundary_vertex(owner: &Candidate, edge: EdgeId, t: f64) -> Result<Option<VertexId>> {
    let edge = owner.brep().edge(edge).map_err(|e| e.to_string())?;
    if t == edge.interval.min() {
        Ok(edge.vertices.first().copied())
    } else if t == edge.interval.max() {
        Ok(edge.vertices.get(1).copied())
    } else {
        Ok(None)
    }
}

fn same_chart_corner(a: Node, b: Node, span: DVec2) -> bool {
    if matches!((a.vertex,b.vertex),(Some(a),Some(b)) if a!=b) {
        return false;
    }
    if a.uv == b.uv {
        return true;
    }
    if a.vertex.is_none() || a.vertex != b.vertex {
        return false;
    }
    // Adjacent incidences project the SAME native vertex independently. Absorb
    // only arithmetic roundoff within one chart branch; do not weld a periodic
    // interval, a pole's distinct UV sides, or two geometrically close vertices.
    // The domain cap prevents a large absolute UV origin swallowing a full span.
    let magnitude = a.uv.abs().max(b.uv.abs()).max(span.abs()).max(DVec2::ONE);
    let machine = (magnitude * (64. * f64::EPSILON)).min(span.abs() * 1e-12);
    (a.uv - b.uv).abs().cmple(machine).all()
}
fn edge_point(
    owner: &Candidate,
    edge_id: EdgeId,
    t: f64,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<DVec3> {
    if let Some(point) = shared.get(&(edge_id, t.to_bits())) {
        return Ok(*point);
    }
    work.step()?;
    let edge = owner.brep().edge(edge_id).map_err(|e| e.to_string())?;
    let curve = edge
        .curve
        .as_ref()
        .ok_or("Missing native defining edge curve")?;
    let evaluated = native_point(
        curve
            .evaluate_point(owner.kernel(), t)
            .map_err(|e| e.to_string())?,
    )?;
    // One shared native vertex closes all incident display edges. Any pinning is
    // bounded by this separate display allowance and never changes source/BRep.
    let vertex = if t == edge.interval.min() {
        edge.vertices.first()
    } else if t == edge.interval.max() {
        edge.vertices.get(1)
    } else {
        None
    };
    let point = if let Some(id) = vertex {
        let point = native_point(owner.brep().vertex(*id).map_err(|e| e.to_string())?.point)?;
        if point.distance(evaluated) > work.chord_mm {
            return Err("Native edge endpoint exceeds display allowance".into());
        }
        point
    } else {
        evaluated
    };
    if shared.len() >= work.options.max_vertices {
        return Err("Display shared boundary sample budget exceeded".into());
    }
    shared.insert((edge_id, t.to_bits()), point);
    Ok(point)
}
fn trim_uv(owner: &Candidate, use_: EdgeuseId, t: f64, work: &mut Work<'_>) -> Result<DVec2> {
    work.step()?;
    let curve = eu::uv_trim_curve(owner.brep(), use_)
        .map_err(|e| e.to_string())?
        .0
        .ok_or("Missing actual native edge-use UV trim")?;
    vec2(
        curve
            .evaluate_point(owner.kernel(), t)
            .map_err(|e| e.to_string())?,
    )
}
fn distance_to_segment(point: DVec3, a: DVec3, b: DVec3) -> f64 {
    let direction = b - a;
    let length = direction.length_squared();
    let fraction = if length > 0. && length.is_finite() {
        ((point - a).dot(direction) / length).clamp(0., 1.)
    } else {
        0.
    };
    point.distance(a + fraction * direction)
}
fn edge_requires_split(
    owner: &Candidate,
    id: EdgeId,
    a: f64,
    b: f64,
    uses: &[(FaceId, EdgeuseId)],
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<bool> {
    let start = edge_point(owner, id, a, work, shared)?;
    let end = edge_point(owner, id, b, work, shared)?;
    for fraction in [0.25, 0.5, 0.75] {
        let t = a + (b - a) * fraction;
        if distance_to_segment(edge_point(owner, id, t, work, shared)?, start, end) > work.chord_mm
        {
            return Ok(true);
        }
        for (face, use_) in uses {
            let scale = owner
                .brep()
                .face(*face)
                .map_err(|e| e.to_string())?
                .uv_domain
                .size();
            let scale = DVec2::new(scale.x.abs(), scale.y.abs());
            if !scale.is_finite() || scale.min_element() <= 0. {
                return Err("Invalid native display UV domain".into());
            }
            let ua = trim_uv(owner, *use_, a, work)? / scale;
            let ub = trim_uv(owner, *use_, b, work)? / scale;
            let uv = trim_uv(owner, *use_, t, work)? / scale;
            if distance_to_segment(uv.extend(0.), ua.extend(0.), ub.extend(0.))
                > work.options.uv_chord_fraction
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}
fn sample_edges(
    owner: &Candidate,
    uses: &BTreeMap<EdgeId, Vec<(FaceId, EdgeuseId)>>,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<BTreeMap<EdgeId, Vec<f64>>> {
    let mut samples = BTreeMap::new();
    for id in owner.brep().edge_list() {
        let edge = owner.brep().edge(*id).map_err(|e| e.to_string())?;
        if !edge.interval.length().is_finite() || edge.interval.length() <= 0. {
            return Err("Invalid native display edge interval".into());
        }
        let curve = edge.curve.as_ref().ok_or("Missing defining edge curve")?;
        if curve.number_natural_knots().map_err(|e| e.to_string())? > work.options.max_vertices {
            return Err("Display curve knot budget exceeded".into());
        }
        let mut knots = curve
            .knots(Some(&edge.interval))
            .map_err(|e| format!("Native display curve knots: {e}"))?
            .0;
        knots.retain(|t| t.is_finite() && *t > edge.interval.min() && *t < edge.interval.max());
        knots.extend([edge.interval.min(), edge.interval.max()]);
        knots.sort_by(f64::total_cmp);
        knots.dedup();
        if knots.len() > work.options.max_vertices {
            return Err("Display boundary knot budget exceeded".into());
        }
        let mut parameters = vec![edge.interval.min()];
        for pair in knots.windows(2) {
            let [a, b] = pair else {
                return Err("Incomplete native knot interval".into());
            };
            let mut stack = vec![(*a, *b, 0u32)];
            while let Some((a, b, depth)) = stack.pop() {
                if edge_requires_split(
                    owner,
                    *id,
                    a,
                    b,
                    uses.get(id).map_or(&[], Vec::as_slice),
                    work,
                    shared,
                )? {
                    if depth >= work.options.max_depth {
                        return Err("Display boundary refinement depth exceeded".into());
                    }
                    let mid = a * 0.5 + b * 0.5;
                    if mid <= a || mid >= b {
                        return Err("Display boundary parameter subdivision exhausted".into());
                    }
                    stack.push((mid, b, depth + 1));
                    stack.push((a, mid, depth + 1));
                } else {
                    parameters.push(b);
                }
                if parameters.len().saturating_add(stack.len()) > work.options.max_vertices {
                    return Err("Display boundary vertex budget exceeded".into());
                }
            }
        }
        samples.insert(*id, parameters);
    }
    Ok(samples)
}
fn insert_constraint(
    map: &mut BTreeMap<(usize, usize), Boundary>,
    a: usize,
    b: usize,
    mut boundary: Boundary,
) {
    if a < b {
        map.insert((a, b), boundary);
    } else if a > b {
        std::mem::swap(&mut boundary.ta, &mut boundary.tb);
        map.insert((b, a), boundary);
    }
}
fn pole_bridge(
    owner: &Candidate,
    face: FaceId,
    a: Node,
    b: Node,
    work: &mut Work<'_>,
) -> Result<()> {
    if a.point.distance(b.point) > work.chord_mm {
        return Err("Native trim incidences do not meet at a shared display vertex".into());
    }
    // At a collapsed chart corner the two UV sides differ while 3D is one pole.
    // Verify the derived display segment rather than welding regular seams in UV.
    for fraction in [0.25, 0.5, 0.75] {
        if sample(owner, face, a.uv.lerp(b.uv, fraction), work)?
            .point
            .distance(a.point)
            > work.chord_mm
        {
            return Err("Native trim UV discontinuity is not a collapsed display corner".into());
        }
    }
    Ok(())
}
fn point_at(mesh: &FaceMesh, id: usize) -> Result<Node> {
    mesh.nodes
        .get(id)
        .copied()
        .ok_or_else(|| "Display mesh index out of range".into())
}
fn span_limit(surface: &Surface) -> Result<DVec2> {
    if matches!(surface.kind(), SurfaceKind::Plane(_)) {
        return Ok(DVec2::splat(f64::MAX));
    }
    let mut result = [f64::MAX; 2];
    for (slot, direction) in result.iter_mut().zip([SurfParamType::U, SurfParamType::V]) {
        let knots = surface
            .knots(direction, None)
            .map_err(|e| format!("Native display surface spans: {e}"))?
            .0;
        for pair in knots.windows(2) {
            if let [a, b] = pair {
                if b > a {
                    *slot = slot.min((b - a) * 0.5);
                }
            }
        }
        if !slot.is_finite() || *slot <= 0. {
            return Err("Invalid native surface span".into());
        }
    }
    let [u, v] = result;
    Ok(DVec2::new(u, v))
}
fn face_mesh(
    owner: &Candidate,
    face: FaceId,
    parameters: &BTreeMap<EdgeId, Vec<f64>>,
    prior_triangles: usize,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<FaceMesh> {
    let record = owner.brep().face(face).map_err(|e| e.to_string())?;
    let primary = record.faceuse.ok_or("Face has no primary face use")?;
    let loops = &owner
        .brep()
        .faceuse(primary)
        .map_err(|e| e.to_string())?
        .loopuses;
    let mut mesh = FaceMesh {
        face,
        nodes: Vec::new(),
        triangles: Vec::new(),
        constraints: BTreeMap::new(),
        uv_scale: DVec2::ONE,
        span: span_limit(surface(owner, face)?)?,
        curved: !matches!(surface(owner, face)?.kind(), SurfaceKind::Plane(_)),
    };
    let mut rings = Vec::new();
    let span = record.uv_domain.size();
    let span = DVec2::new(span.x.abs(), span.y.abs());
    for (loop_index, loopuse) in loops.iter().enumerate() {
        if matches!(
            owner
                .brep()
                .loopuse(*loopuse)
                .map_err(|e| e.to_string())?
                .start,
            LoopuseStart::Vertexuse(_)
        ) {
            if loop_index == 0 {
                return Err("Outer display loop is a vertex loop".into());
            }
            continue; // Point boundary has no removed two-dimensional area.
        }
        let mut ring = Vec::new();
        let mut segments = Vec::new();
        for use_ in eu::loopuse_edgeuses(owner.brep(), *loopuse).map_err(|e| e.to_string())? {
            let incidence = owner.brep().edgeuse(use_).map_err(|e| e.to_string())?;
            let edge = incidence.edge.ok_or("Edge use has no defining edge")?;
            let mut ts = parameters
                .get(&edge)
                .ok_or("Missing shared edge samples")?
                .clone();
            if incidence.orientation
                == solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Opposite
            {
                ts.reverse();
            }
            let mut previous = None;
            for t in ts {
                let uv = trim_uv(owner, use_, t, work)?;
                let mut node = sample(owner, face, uv, work)?;
                let point = edge_point(owner, edge, t, work, shared)?;
                if node.point.distance(point) > work.chord_mm {
                    return Err("Native trim-to-edge residual exceeds display allowance".into());
                }
                node.point = point;
                node.vertex = boundary_vertex(owner, edge, t)?;
                let id = if let Some(last) = ring.last().copied() {
                    let last_node = point_at(&mesh, last)?;
                    if same_chart_corner(last_node, node, span) {
                        last
                    } else {
                        if previous.is_none() {
                            pole_bridge(owner, face, last_node, node, work)?;
                        }
                        work.node()?;
                        let id = mesh.nodes.len();
                        mesh.nodes.push(node);
                        ring.push(id);
                        id
                    }
                } else {
                    work.node()?;
                    let id = mesh.nodes.len();
                    mesh.nodes.push(node);
                    ring.push(id);
                    id
                };
                if let Some((last, old_t)) = previous {
                    segments.push((
                        last,
                        id,
                        Boundary {
                            edge,
                            use_,
                            ta: old_t,
                            tb: t,
                        },
                    ));
                }
                previous = Some((id, t));
            }
        }
        if ring.len() < 3 {
            return Err("Native display boundary has too few UV points".into());
        }
        let first = *ring.first().ok_or("Missing UV boundary start")?;
        let last = *ring.last().ok_or("Missing UV boundary end")?;
        if same_chart_corner(point_at(&mesh, first)?, point_at(&mesh, last)?, span) {
            ring.pop();
            for (a, b, _) in &mut segments {
                if *a == last {
                    *a = first;
                }
                if *b == last {
                    *b = first;
                }
            }
        } else {
            pole_bridge(
                owner,
                face,
                point_at(&mesh, last)?,
                point_at(&mesh, first)?,
                work,
            )?;
        }
        for (a, b, boundary) in segments {
            insert_constraint(&mut mesh.constraints, a, b, boundary);
        }
        rings.push(ring);
    }
    let all = rings.iter().flatten().copied().collect::<Vec<_>>();
    let first = point_at(
        &mesh,
        *all.first().ok_or("Face has no native edge boundary")?,
    )?
    .uv;
    let (mut lo, mut hi) = (first, first);
    for id in &all {
        let uv = point_at(&mesh, *id)?.uv;
        lo = lo.min(uv);
        hi = hi.max(uv);
    }
    mesh.uv_scale = hi - lo;
    if !mesh.uv_scale.is_finite() || mesh.uv_scale.min_element() <= 0. {
        return Err("Degenerate native UV region".into());
    }
    let normalized =
        |id: usize| -> Result<DVec2> { Ok((point_at(&mesh, id)?.uv - lo) / mesh.uv_scale) };
    let mut order = Vec::new();
    let mut holes = Vec::new();
    for (index, ring) in rings.iter().enumerate() {
        if index > 0 {
            holes.push(order.len());
        }
        // Earcut's exact orientation tests otherwise turn projection roundoff
        // along a straight UV boundary into zero-domain sliver triangles. Only
        // simplify its seed polygon at machine precision. The complete native
        // boundary chain (and every original UV/parameter) remains in `rings`
        // and is restored below; no CAD topology or trim definition is changed.
        for (i, &id) in ring.iter().enumerate() {
            if i.is_multiple_of(4096) {
                (work.progress)("6 / 6  Preparing native UV display seeds")?;
            }
            let prev = *ring
                .get((i + ring.len() - 1) % ring.len())
                .ok_or("Seed ring index")?;
            let next = *ring.get((i + 1) % ring.len()).ok_or("Seed ring index")?;
            let (a, p, b) = (normalized(prev)?, normalized(id)?, normalized(next)?);
            if ring.len() > 3
                && distance_to_segment(p.extend(0.), a.extend(0.), b.extend(0.))
                    <= 64. * f64::EPSILON
            {
                continue;
            }
            order.push(id);
        }
        let begin = holes.last().copied().filter(|_| index > 0).unwrap_or(0);
        if order.len().saturating_sub(begin) < 3 {
            return Err("Degenerate native UV display seed".into());
        }
    }
    let mut coordinates = Vec::new();
    for id in &order {
        let uv = (point_at(&mesh, *id)?.uv - lo) / mesh.uv_scale;
        coordinates.extend([uv.x, uv.y]);
    }
    let indices = earcutr::earcut(&coordinates, &holes, 2)
        .map_err(|e| format!("Display UV region triangulation: {e:?}"))?;
    if indices.is_empty() || indices.len() % 3 != 0 {
        return Err("Native UV region yielded no complete display triangles".into());
    }
    work.triangles(
        prior_triangles
            .checked_add(indices.len() / 3)
            .ok_or("Display triangle count overflow")?,
    )?;
    let between = |a: usize, b: usize| -> Result<Vec<usize>> {
        for ring in &rings {
            let (Some(ia), Some(ib)) = (
                ring.iter().position(|id| *id == a),
                ring.iter().position(|id| *id == b),
            ) else {
                continue;
            };
            for forward in [true, false] {
                let mut candidates = Vec::new();
                let mut current = ia;
                for _ in 0..ring.len() {
                    current = if forward {
                        (current + 1) % ring.len()
                    } else {
                        (current + ring.len() - 1) % ring.len()
                    };
                    if current == ib {
                        break;
                    }
                    candidates.push(*ring.get(current).ok_or("Boundary index")?);
                }
                let aa = (point_at(&mesh, a)?.uv - lo) / mesh.uv_scale;
                let bb = (point_at(&mesh, b)?.uv - lo) / mesh.uv_scale;
                if candidates.iter().all(|id| {
                    mesh.nodes.get(*id).is_some_and(|node| {
                        distance_to_segment(
                            ((node.uv - lo) / mesh.uv_scale).extend(0.),
                            aa.extend(0.),
                            bb.extend(0.),
                        ) <= 1e-10
                    })
                }) {
                    return Ok(candidates);
                }
            }
        }
        Ok(Vec::new())
    };
    let mut initial = Vec::new();
    let mut planned_triangles = 0usize;
    for [a, b, c] in indices.as_chunks::<3>().0 {
        let a = *order.get(*a).ok_or("UV triangulation index")?;
        let b = *order.get(*b).ok_or("UV triangulation index")?;
        let c = *order.get(*c).ok_or("UV triangulation index")?;
        let mut polygon = vec![a];
        polygon.extend(between(a, b)?);
        polygon.push(b);
        polygon.extend(between(b, c)?);
        polygon.push(c);
        polygon.extend(between(c, a)?);
        planned_triangles = planned_triangles
            .checked_add(if polygon.len() == 3 { 1 } else { polygon.len() })
            .ok_or("Display triangle count overflow")?;
        // Check before retaining the recovery fan or appending triangles. The
        // count includes already extracted faces; it bounds display elements,
        // not process RSS or the native owning model's own allocations.
        work.triangles(
            prior_triangles
                .checked_add(planned_triangles)
                .ok_or("Display triangle count overflow")?,
        )?;
        initial.push((a, b, c, polygon));
    }
    // Earcut may remove collinear UV points, although they are distinct 3D
    // boundary samples on a cylinder. Restore every such constraint explicitly.
    for (a, b, c, polygon) in initial {
        if polygon.len() == 3 {
            mesh.triangles.push([a, b, c]);
        } else {
            let uv =
                (point_at(&mesh, a)?.uv + point_at(&mesh, b)?.uv + point_at(&mesh, c)?.uv) / 3.;
            let node = sample(owner, face, uv, work)?;
            work.node()?;
            let center = mesh.nodes.len();
            mesh.nodes.push(node);
            for (a, b) in polygon
                .iter()
                .copied()
                .zip(polygon.iter().copied().cycle().skip(1))
                .take(polygon.len())
            {
                mesh.triangles.push([a, b, center]);
            }
        }
    }
    work.triangles(mesh.triangles.len())?;
    Ok(mesh)
}
fn midpoint(
    owner: &Candidate,
    mesh: &FaceMesh,
    a: usize,
    b: usize,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<Node> {
    let key = (a.min(b), a.max(b));
    if let Some(boundary) = mesh.constraints.get(&key) {
        let t = boundary.ta * 0.5 + boundary.tb * 0.5;
        let mut node = sample(
            owner,
            mesh.face,
            trim_uv(owner, boundary.use_, t, work)?,
            work,
        )?;
        let point = edge_point(owner, boundary.edge, t, work, shared)?;
        if point.distance(node.point) > work.chord_mm {
            return Err("Refined native trim exceeds display allowance".into());
        }
        node.point = point;
        Ok(node)
    } else {
        sample(
            owner,
            mesh.face,
            (point_at(mesh, a)?.uv + point_at(mesh, b)?.uv) * 0.5,
            work,
        )
    }
}
type MeshEdge = (usize, usize);

fn key(a: usize, b: usize) -> MeshEdge {
    (a.min(b), a.max(b))
}

fn marked_edges(
    owner: &Candidate,
    mesh: &FaceMesh,
    force_interior: bool,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<BTreeSet<MeshEdge>> {
    let mut marked = BTreeSet::new();
    let mut evaluated = BTreeSet::new();
    let cosine = work.options.normal_angle_radians.cos();
    for &[a, b, c] in &mesh.triangles {
        let nodes = [point_at(mesh, a)?, point_at(mesh, b)?, point_at(mesh, c)?];
        let [na, nb, nc] = nodes;
        if force_interior && mesh.curved {
            marked.extend([key(a, b), key(b, c), key(c, a)]);
            continue;
        }
        for (a, b) in [(a, b), (b, c), (c, a)] {
            let pair = key(a, b);
            if !evaluated.insert(pair) {
                continue;
            }
            let mid = midpoint(owner, mesh, a, b, work, shared)?;
            let a = point_at(mesh, a)?;
            let b = point_at(mesh, b)?;
            let too_wide = mesh.curved && (b.uv - a.uv).abs().cmpgt(mesh.span).any();
            let curved_chord = mid.point.distance((a.point + b.point) * 0.5) > work.chord_mm;
            let turned = [a.normal, b.normal]
                .into_iter()
                .flatten()
                .any(|normal| mid.normal.is_some_and(|mid| normal.dot(mid) < cosine));
            if too_wide || curved_chord || turned {
                marked.insert(pair);
            }
        }
        let center = sample(owner, mesh.face, (na.uv + nb.uv + nc.uv) / 3., work)?;
        let displaced =
            center.point.distance((na.point + nb.point + nc.point) / 3.) > work.chord_mm;
        let turned = nodes
            .into_iter()
            .filter_map(|node| node.normal)
            .any(|normal| center.normal.is_some_and(|mid| normal.dot(mid) < cosine));
        if displaced || turned {
            marked.extend([key(a, b), key(b, c), key(c, a)]);
        }
    }
    Ok(marked)
}

fn boundary_key(boundary: &Boundary) -> (EdgeId, u64, u64) {
    // Parameter domains belong to the actual native edge, not source traversal
    // order or UV direction. Both seam incidences and adjacent caps use this key.
    (
        boundary.edge,
        boundary.ta.min(boundary.tb).to_bits(),
        boundary.ta.max(boundary.tb).to_bits(),
    )
}

fn refinement_count(mesh: &FaceMesh, marked: &BTreeSet<MeshEdge>) -> Result<usize> {
    mesh.triangles.iter().try_fold(0usize, |count, &[a, b, c]| {
        let inserted = [key(a, b), key(b, c), key(c, a)]
            .iter()
            .filter(|pair| marked.contains(pair))
            .count();
        count
            .checked_add(1 + inserted)
            .ok_or_else(|| "Display triangle count overflow".into())
    })
}

fn refine(
    owner: &Candidate,
    mesh: &mut FaceMesh,
    marked: &BTreeSet<MeshEdge>,
    work: &mut Work<'_>,
    shared: &mut BTreeMap<(EdgeId, u64), DVec3>,
) -> Result<()> {
    let old = std::mem::take(&mut mesh.triangles);
    let mut midpoints = BTreeMap::new();
    for &(a, b) in marked {
        let node = midpoint(owner, mesh, a, b, work, shared)?;
        work.node()?;
        let id = mesh.nodes.len();
        mesh.nodes.push(node);
        midpoints.insert((a, b), id);
    }
    let mut constraints = BTreeMap::new();
    for (&(a, b), boundary) in &mesh.constraints {
        if let Some(mid) = midpoints.get(&(a, b)) {
            let t = boundary.ta * 0.5 + boundary.tb * 0.5;
            insert_constraint(&mut constraints, a, *mid, Boundary { tb: t, ..*boundary });
            insert_constraint(&mut constraints, *mid, b, Boundary { ta: t, ..*boundary });
        } else {
            insert_constraint(&mut constraints, a, b, *boundary);
        }
    }
    mesh.constraints = constraints;
    for [a, b, c] in old {
        // Red/green refinement splits both triangles of every marked mesh edge.
        // Unmarked edges remain whole, so no neighbour acquires a T junction;
        // the same native boundary marks have already reached every face use.
        let [ab, bc, ca] =
            [key(a, b), key(b, c), key(c, a)].map(|pair| midpoints.get(&pair).copied());
        match (ab, bc, ca) {
            (None, None, None) => mesh.triangles.push([a, b, c]),
            (Some(ab), None, None) => mesh.triangles.extend([[a, ab, c], [ab, b, c]]),
            (None, Some(bc), None) => mesh.triangles.extend([[b, bc, a], [bc, c, a]]),
            (None, None, Some(ca)) => mesh.triangles.extend([[c, ca, b], [ca, a, b]]),
            (Some(ab), Some(bc), None) => {
                mesh.triangles
                    .extend([[a, ab, c], [ab, bc, c], [ab, b, bc]]);
            }
            (Some(ab), None, Some(ca)) => {
                mesh.triangles
                    .extend([[a, ab, ca], [ab, b, c], [ab, c, ca]]);
            }
            (None, Some(bc), Some(ca)) => {
                mesh.triangles
                    .extend([[c, ca, bc], [a, b, ca], [b, bc, ca]]);
            }
            (Some(ab), Some(bc), Some(ca)) => {
                mesh.triangles
                    .extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]);
            }
        }
    }
    work.triangles(mesh.triangles.len())
}

pub(super) struct FaceTriangles {
    pub face: FaceId,
    pub vertices: Vec<(DVec3, [f32; 3])>,
}

pub(super) struct Extracted {
    pub triangles: Vec<FaceTriangles>,
    pub edges: Vec<(EdgeId, Vec<EdgeSample>)>,
    pub chord_mm: f64,
}
pub(super) fn extract(
    owner: &Candidate,
    options: DisplayOptions,
    progress: &dyn Fn(&str) -> Result<()>,
) -> Result<Extracted> {
    options.validate()?;
    progress("6 / 6  Preparing shared native display boundaries")?;
    let mut scale = 0_f64;
    let mut origin = None;
    for id in owner.brep().vertex_list() {
        let point = native_point(owner.brep().vertex(*id).map_err(|e| e.to_string())?.point)?;
        let center = *origin.get_or_insert(point);
        scale = scale.max(point.distance(center));
    }
    // A full periodic face can have a single native vertex. Obtain scale
    // from actual curve evaluations as well, never from source labels or a
    // fabricated unit-sized box. This prepass has the same evaluation ceiling.
    let mut seed_evaluations = 0usize;
    for id in owner.brep().edge_list() {
        let edge = owner.brep().edge(*id).map_err(|e| e.to_string())?;
        let curve = edge.curve.as_ref().ok_or("Missing native scale curve")?;
        for fraction in [0., 0.25, 0.5, 0.75, 1.] {
            seed_evaluations = seed_evaluations
                .checked_add(1)
                .ok_or("Display scale count overflow")?;
            if seed_evaluations > options.max_evaluations {
                return Err("Display scale evaluation budget exceeded".into());
            }
            let point = native_point(
                curve
                    .evaluate_point(owner.kernel(), edge.interval.evaluate(fraction))
                    .map_err(|e| e.to_string())?,
            )?;
            let center = *origin.get_or_insert(point);
            scale = scale.max(point.distance(center));
        }
        if seed_evaluations.is_multiple_of(512) {
            progress("6 / 6  Measuring native display extent")?;
        }
    }
    if !scale.is_finite() || scale <= 0. {
        return Err("No finite display scale from native definitions".into());
    }
    let chord_mm = options.chord_mm.unwrap_or(scale * 0.001);
    if !chord_mm.is_finite() || chord_mm <= 0. {
        return Err("Invalid display chord allowance".into());
    }
    let mut work = Work {
        options,
        chord_mm,
        evaluations: seed_evaluations,
        nodes: 0,
        progress,
    };
    let mut shared = BTreeMap::new();
    let mut uses = BTreeMap::new();
    for face in owner.brep().face_list() {
        let primary = owner
            .brep()
            .face(*face)
            .map_err(|e| e.to_string())?
            .faceuse
            .ok_or("Missing primary face use")?;
        for loopuse in &owner
            .brep()
            .faceuse(primary)
            .map_err(|e| e.to_string())?
            .loopuses
        {
            for use_ in eu::loopuse_edgeuses(owner.brep(), *loopuse).map_err(|e| e.to_string())? {
                let edge = owner
                    .brep()
                    .edgeuse(use_)
                    .map_err(|e| e.to_string())?
                    .edge
                    .ok_or("Unowned edge use")?;
                uses.entry(edge)
                    .or_insert_with(Vec::new)
                    .push((*face, use_));
            }
        }
    }
    let parameters = sample_edges(owner, &uses, &mut work, &mut shared)?;
    let mut faces = Vec::new();
    let mut total_triangles = 0usize;
    for id in owner.brep().face_list() {
        let mesh = face_mesh(
            owner,
            *id,
            &parameters,
            total_triangles,
            &mut work,
            &mut shared,
        )?;
        total_triangles = total_triangles
            .checked_add(mesh.triangles.len())
            .ok_or("Display triangle count overflow")?;
        work.triangles(total_triangles)?;
        faces.push(mesh);
    }
    if faces.is_empty() {
        return Err("Native candidate contains no display faces".into());
    }
    for depth in 0..=options.max_depth {
        let mut marked = Vec::new();
        let mut boundary_marks = BTreeSet::new();
        for face in &faces {
            let edges = marked_edges(owner, face, depth == 0, &mut work, &mut shared)?;
            for edge in &edges {
                if let Some(boundary) = face.constraints.get(edge) {
                    boundary_marks.insert(boundary_key(boundary));
                }
            }
            marked.push(edges);
        }
        for (face, edges) in faces.iter().zip(&mut marked) {
            for (pair, boundary) in &face.constraints {
                if boundary_marks.contains(&boundary_key(boundary)) {
                    edges.insert(*pair);
                }
            }
        }
        if marked.iter().all(BTreeSet::is_empty) {
            break;
        }
        if depth == options.max_depth {
            return Err("Display surface refinement depth exceeded".into());
        }
        let count = faces
            .iter()
            .zip(&marked)
            .try_fold(0usize, |sum, (face, edges)| {
                sum.checked_add(refinement_count(face, edges)?)
                    .ok_or_else(|| "Display triangle count overflow".to_string())
            })?;
        work.triangles(count)?;
        progress("6 / 6  Refining conforming native surface interiors")?;
        for (face, edges) in faces.iter_mut().zip(&marked) {
            refine(owner, face, edges, &mut work, &mut shared)?;
        }
    }
    let mut triangles = Vec::new();
    for face in &faces {
        let mut vertices = Vec::new();
        for &[a, b, c] in &face.triangles {
            let [a, b, c] = [point_at(face, a)?, point_at(face, b)?, point_at(face, c)?];
            let fallback = unit((b.point - a.point).cross(c.point - a.point));
            // A pole has no unique differential normal. Only its displayed
            // adjacent triangle supplies a derived lighting normal; no query or
            // measurement receives this convention. Zero-area pole triangles draw nothing.
            let Some(fallback) = fallback else {
                continue;
            };
            for node in [a, b, c] {
                let normal = node.normal.unwrap_or(fallback);
                let light = normal.dot(DVec3::new(0.3, -0.4, 0.85).normalize()).abs() as f32;
                vertices.push((
                    node.point,
                    [
                        0.23 + 0.18 * light,
                        0.49 + 0.22 * light,
                        0.68 + 0.22 * light,
                    ],
                ));
            }
        }
        if vertices.is_empty() {
            return Err("Native face has no nondegenerate display triangles".into());
        }
        triangles.push(FaceTriangles {
            face: face.face,
            vertices,
        });
    }
    let mut edges = Vec::new();
    for id in owner.brep().edge_list() {
        // Only parameters installed on final mesh boundaries become line geometry;
        // scratch quarter-point screens do not create unmatched visible segments.
        let mut ts = parameters
            .get(id)
            .cloned()
            .ok_or("Missing display edge schedule")?;
        for face in &faces {
            for boundary in face
                .constraints
                .values()
                .filter(|boundary| boundary.edge == *id)
            {
                ts.extend([boundary.ta, boundary.tb]);
            }
        }
        ts.sort_by(f64::total_cmp);
        ts.dedup();
        let mut samples = Vec::new();
        for t in ts {
            samples.push(EdgeSample {
                parameter: t,
                point_mm: edge_point(owner, *id, t, &mut work, &mut shared)?,
            });
        }
        edges.push((*id, samples));
    }
    progress("6 / 6  Native display mesh complete")?;
    Ok(Extracted {
        triangles,
        edges,
        chord_mm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chart_corner_canonicalization_never_merges_different_real_vertices() {
        let original = Node {
            uv: DVec2::new(1., 2.),
            point: DVec3::new(3., 0., 7.),
            normal: None,
            vertex: Some(VertexId::from_index(2)),
        };
        let distinct = Node {
            vertex: Some(VertexId::from_index(3)),
            ..original
        };
        assert!(!same_chart_corner(original, distinct, DVec2::new(4., 7.)));
        let projected_again = Node {
            uv: DVec2::new(f64::from_bits(1_f64.to_bits() + 1), 2.),
            ..original
        };
        assert!(same_chart_corner(
            original,
            projected_again,
            DVec2::new(4., 7.)
        ));
        let periodic_other_side = Node {
            uv: DVec2::new(5., 2.),
            ..original
        };
        assert!(!same_chart_corner(
            original,
            periodic_other_side,
            DVec2::new(4., 7.)
        ));
        // A large absolute UV origin must not make an entire short periodic
        // span look like roundoff at one source/native vertex.
        let shifted = Node {
            uv: DVec2::new(1e12, 2.),
            ..original
        };
        let next_branch = Node {
            uv: DVec2::new(f64::from_bits(1e12_f64.to_bits() + 1), 2.),
            ..original
        };
        assert!(!same_chart_corner(
            shifted,
            next_branch,
            DVec2::new(0.0002, 7.)
        ));
    }
}
