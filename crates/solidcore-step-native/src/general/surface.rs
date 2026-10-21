// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Original source support definitions and finite native computational charts.
//!
//! Upstream: maintained `StepImportGeneral.cpp::Surface/AngularWindow/MissingSeam`
//! at `d8ed45a`. Windows cover defining curve boxes, never only topology vertices.
//! Source angles, radii, coefficients and topological boundaries remain unchanged.

use super::geometry::{norm, Frame, Geometry, ParameterMapping};
use super::{
    edge_bounds, need, scalar, unsupported, GeneralQuality, SourceEdge, SourceFace, SourceVertex,
};
use crate::{Error, Execution, Result};
use solidcore::curve::curve::CurveKind;
use solidcore::geom::extent2d::Extent2d;
use solidcore::math::{Vec2, Vec3, EFF_ZERO};
use solidcore::nurbs::b_spline_curve::KnotType;
use solidcore::nurbs::b_spline_surface::{
    BSplineSurface, BSplineSurfaceForm, CanonicalSurfaceData,
};
use solidcore::surface::{
    cone::Cone,
    plane::Plane,
    surface::Surface,
    torus::{Torus, TorusOptions},
};
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

pub(super) struct Seam {
    pub vertices: [usize; 2],
    pub endpoints: [Vec3; 2],
    angle: f64,
}
/// A cut in the original second-ring parameter chart, before any curve transfer.
/// Neither defining coefficients nor the original closed-edge anchor changes.
#[derive(Clone, Debug)]
pub(super) struct RingSplit {
    pub edge: usize,
    pub native_parameter: f64,
    pub point: Vec3,
}

/// Equivalent chart normalization for two closed circular cylinder boundaries.
/// Edge/vertex indices address the original source plan, in face-loop order.
#[derive(Clone, Debug)]
pub(super) struct ImplicitRingPlan {
    pub ring_edges: [usize; 2],
    pub source_vertices: [usize; 2],
    pub seam_endpoints: [Vec3; 2],
    pub seam_angle_rad: f64,
    pub split: RingSplit,
}

struct RingSupport {
    edge: usize,
    vertex: usize,
    frame: Frame,
    height: f64,
    radius: f64,
    point: Vec3,
}

struct ImplicitRings {
    rings: [RingSupport; 2],
    seam: Seam,
    aligned: bool,
}

pub(super) struct BuiltSurface {
    pub surface: Surface,
    pub seam: Option<Seam>,
    pub pole_bound: Option<usize>,
    pub mappings: Vec<ParameterMapping>,
}
fn curve(edges: &[SourceEdge], index: usize) -> Result<&SourceEdge> {
    edges.get(index).ok_or(Error::Invalid("GeneralEdgeIndex"))
}
fn point(vertices: &[SourceVertex], index: usize) -> Result<Vec3> {
    vertices
        .get(index)
        .map(|v| v.point)
        .ok_or(Error::Invalid("GeneralVertexIndex"))
}
fn envelope(face_loop: &super::SourceLoop, edges: &[SourceEdge]) -> Result<f64> {
    if face_loop.vertex().is_some() {
        return Ok(0.);
    }
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for u in face_loop.edge_uses() {
        let b = edge_bounds(curve(edges, u.edge)?)?;
        min = Vec3::new(
            min.x.min(b.min().x),
            min.y.min(b.min().y),
            min.z.min(b.min().z),
        );
        max = Vec3::new(
            max.x.max(b.max().x),
            max.y.max(b.max().y),
            max.z.max(b.max().z),
        );
    }
    Ok(norm(max - min))
}
pub(super) fn prepare_loop_head(face: &mut SourceFace, edges: &[SourceEdge]) -> Result<()> {
    if face.loops.iter().any(|l| l.outer) {
        return Ok(());
    }
    let mut largest = None;
    for (i, l) in face.loops.iter().enumerate() {
        let size = envelope(l, edges)?;
        if largest.is_none_or(|(_, s)| size > s) {
            largest = Some((i, size));
        }
    }
    if let Some((i, _)) = largest {
        // C++ rotates [begin,candidate,candidate+1], keeping every other loop's
        // relative order. This is a bootstrap, not an inferred region verdict.
        face.loops
            .get_mut(..=i)
            .ok_or(Error::Invalid("GeneralLoopHead"))?
            .rotate_right(1);
    }
    Ok(())
}
fn implicit_rings(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    quality: &GeneralQuality,
) -> Result<Option<ImplicitRings>> {
    if !g.is(face.geometry, "CYLINDRICAL_SURFACE") || face.loops.len() != 2 {
        return Ok(None);
    }
    let mut candidates = Vec::new();
    for l in &face.loops {
        if l.edge_uses().len() != 1 {
            return Ok(None);
        }
        let index = l
            .edge_uses()
            .first()
            .ok_or(Error::Invalid("GeneralLoopUse"))?
            .edge;
        let e = curve(edges, index)?;
        let [a, b] = e.vertices;
        if !g.is(e.geometry, "CIRCLE") || a != b {
            return Ok(None);
        }
        candidates.push((index, a));
    }
    let consistency = scalar(quality.consistency_mm);
    let radius = g.number(face.geometry, "radius")? * g.length_scale();
    need(
        radius.is_finite() && radius > 0.,
        "GeneralImplicitRingSupport",
        face.record,
    )?;
    let mut rings = Vec::new();
    for (index, vertex) in candidates {
        let edge = curve(edges, index)?;
        let cf = g.placement(g.reference(edge.geometry, "position")?)?;
        let circle_radius = g.number(edge.geometry, "radius")? * g.length_scale();
        let center = cf.origin - frame.origin;
        let height = center.dot(frame.z);
        let center_offset = norm(center - frame.z * height);
        let tilt = norm(cf.z.cross(frame.z));
        // Analytic whole-circle support bound: displaced center + possible
        // out-of-normal-plane excursion + radius mismatch. A pair of matching
        // anchors alone cannot establish that the complete rings lie on the
        // cylinder. Antiparallel ring normals remain valid source conventions.
        let support_gap = center_offset + circle_radius * tilt + (circle_radius - radius).abs();
        need(
            height.is_finite()
                && circle_radius.is_finite()
                && circle_radius > 0.
                && tilt.is_finite()
                && tilt <= 1e-12
                && support_gap.is_finite()
                && support_gap <= consistency,
            "GeneralImplicitRingSupport",
            edge.record,
        )?;
        let anchor = point(vertices, vertex)?;
        let delta = anchor - cf.origin;
        let plane_gap = delta.dot(cf.z).abs();
        let radial_gap = (norm(delta - cf.z * delta.dot(cf.z)) - circle_radius).abs();
        let built = edge
            .built
            .as_ref()
            .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
        need(
            plane_gap.is_finite()
                && radial_gap.is_finite()
                && plane_gap + radial_gap <= consistency
                && built
                    .endpoints
                    .iter()
                    .all(|p| norm(*p - anchor) <= consistency),
            "GeneralImplicitRingVertexGap",
            edge.record,
        )?;
        rings.push(RingSupport {
            edge: index,
            vertex,
            frame: cf,
            height,
            radius: circle_radius,
            point: anchor,
        });
    }
    let mut rings = rings.into_iter();
    let first = rings.next().ok_or(Error::Invalid("GeneralSeamVertex"))?;
    let second = rings.next().ok_or(Error::Invalid("GeneralSeamVertex"))?;
    need(
        first.edge != second.edge,
        "GeneralImplicitRingDistinctEdges",
        face.record,
    )?;
    // Consistency is a residual allowance, not a minimum feature size. The
    // construction caller separately checks seam length against its selected
    // native working tolerance, as on the pre-existing aligned-ring path.
    let height = first.height - second.height;
    need(
        height.is_finite() && height != 0.,
        "GeneralImplicitSeamHeight",
        face.record,
    )?;
    let delta = first.point - frame.origin;
    let radial = delta - frame.z * delta.dot(frame.z);
    let length = norm(radial);
    need(
        length.is_finite() && length > 0.,
        "GeneralDegenerateFrame",
        face.record,
    )?;
    let radial = radial * (1. / length);
    let actual_a = frame.origin + frame.z * first.height + radial * radius;
    let actual_b = frame.origin + frame.z * second.height + radial * radius;
    need(
        norm(actual_a - first.point) <= consistency,
        "GeneralImplicitRingVertexGap",
        curve(edges, first.edge)?.record,
    )?;
    let aligned = norm(actual_b - second.point) <= consistency;
    Ok(Some(ImplicitRings {
        seam: Seam {
            vertices: [first.vertex, second.vertex],
            endpoints: [actual_a, actual_b],
            angle: radial.dot(frame.y).atan2(radial.dot(frame.x)),
        },
        rings: [first, second],
        aligned,
    }))
}

fn missing_seam(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    quality: &GeneralQuality,
) -> Result<Option<Seam>> {
    let Some(rings) = implicit_rings(g, face, edges, vertices, frame, quality)? else {
        return Ok(None);
    };
    if !rings.aligned {
        return Err(unsupported(
            "GeneralImplicitSeamRequiresRingSplit",
            face.record,
        ));
    }
    Ok(Some(rings.seam))
}

/// Plan a phase cut on the complete original second ring. Call after defining
/// curves are built and before any edge curve is transferred into the `BRep`.
/// Aligned rings keep the existing direct-seam path and incur no native work.
///
/// This extends the maintained `MissingSeam` normalization; it is not a source
/// repair. Native conic parameters are rational parameters, never STEP angles.
/// Invalid support is a postcheck failure, not evidence of an unsupported phase.
pub(super) fn plan_implicit_rings(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    quality: &GeneralQuality,
    execution: &mut Execution,
) -> Result<Option<ImplicitRingPlan>> {
    if !g.is(face.geometry, "CYLINDRICAL_SURFACE") {
        return Ok(None);
    }
    let frame = g.placement(g.reference(face.geometry, "position")?)?;
    let Some(rings) = implicit_rings(g, face, edges, vertices, frame, quality)? else {
        return Ok(None);
    };
    if rings.aligned {
        return Ok(None);
    }
    let [first, second] = rings.rings;
    let edge = curve(edges, second.edge)?;
    let built = edge
        .built
        .as_ref()
        .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
    let native = built
        .native
        .as_ref()
        .ok_or(Error::Invalid("GeneralRingPlanningAfterCurveTransfer"))?;
    let CurveKind::Ellipse(circle) = native.kind() else {
        return Err(Error::Invalid("GeneralRingRequiresOriginalConic"));
    };
    let angular = circle.step_interval();
    let angle_roundoff = 64. * f64::EPSILON * 360.;
    need(
        (angular.length() - 360.).abs() <= angle_roundoff
            && native.natural_interval() == built.interval,
        "GeneralImplicitRingFullPeriod",
        edge.record,
    )?;
    let target = rings
        .seam
        .endpoints
        .get(1)
        .copied()
        .ok_or(Error::Invalid("GeneralRingEndpoint"))?;
    let delta = target - second.frame.origin;
    let angle = delta.dot(second.frame.y).atan2(delta.dot(second.frame.x)) * 180. / PI;
    // Select one representative in the ORIGINAL complete angular interval.
    // Only the existing polar converter establishes its nonangular native t.
    let angle = angle + 360. * ((angular.min() - angle) / 360.).ceil();
    need(
        angle.is_finite() && angular.min() < angle && angle < angular.max(),
        "GeneralImplicitRingCutParameter",
        edge.record,
    )?;
    execution.step()?;
    let native_parameter = circle.convert_t_from_step_to_nurbs(g.kernel(), angle)?;
    need(
        native_parameter.is_finite()
            && built.interval.min() < native_parameter
            && native_parameter < built.interval.max(),
        "GeneralImplicitRingCutParameter",
        edge.record,
    )?;
    execution.step()?;
    let cut = native.evaluate_point(g.kernel(), native_parameter)?;
    let consistency = scalar(quality.consistency_mm);
    need(
        norm(cut - target) <= consistency
            && norm(cut - second.point) > consistency
            && (norm(cut - second.frame.origin) - second.radius).abs() <= consistency,
        "GeneralImplicitRingCutGap",
        edge.record,
    )?;
    // Use the original defining-curve points for the derived seam. The separate
    // source vertex positions/identities survive unchanged in the source plan.
    let first_point = curve(edges, first.edge)?
        .built
        .as_ref()
        .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?
        .endpoints
        .first()
        .copied()
        .ok_or(Error::Invalid("GeneralRingEndpoint"))?;
    Ok(Some(ImplicitRingPlan {
        ring_edges: [first.edge, second.edge],
        source_vertices: [first.vertex, second.vertex],
        seam_endpoints: [first_point, cut],
        seam_angle_rad: rings.seam.angle,
        split: RingSplit {
            edge: second.edge,
            native_parameter,
            point: cut,
        },
    }))
}
// A full circular cone boundary and a source pole need a computational
// meridian. Keeping the source point loop as a second native loop would share
// its pole vertex with the chart outer loop and change its incidence semantics.
// Record that singular boundary explicitly; no source edge/point is discarded.
fn cone_pole_seam(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    quality: &GeneralQuality,
) -> Result<Option<(Seam, usize)>> {
    if !face.loops.iter().any(|bound| bound.vertex().is_some()) {
        return Ok(None);
    }
    if !g.is(face.geometry, "CONICAL_SURFACE") || face.loops.len() != 2 {
        return Err(unsupported(
            "GeneralVertexLoopRegionNotYetConstructed",
            face.record,
        ));
    }
    let outer = face
        .loops
        .first()
        .ok_or(Error::Invalid("GeneralPoleBoundary"))?;
    let inner = face
        .loops
        .get(1)
        .ok_or(Error::Invalid("GeneralPoleBoundary"))?;
    let Some(pole) = inner.vertex() else {
        return Err(unsupported(
            "GeneralVertexLoopRegionNotYetConstructed",
            face.record,
        ));
    };
    let [usage] = outer.edge_uses() else {
        return Err(unsupported(
            "GeneralVertexLoopRegionNotYetConstructed",
            face.record,
        ));
    };
    let circle = curve(edges, usage.edge)?;
    if !g.is(circle.geometry, "CIRCLE") || circle.vertices[0] != circle.vertices[1] {
        return Err(unsupported(
            "GeneralVertexLoopRequiresCircularRing",
            face.record,
        ));
    }
    let circle_frame = g.placement(g.reference(circle.geometry, "position")?)?;
    let delta = circle_frame.origin - frame.origin;
    let height = delta.dot(frame.z);
    let radius = g.number(circle.geometry, "radius")? * g.length_scale();
    let slope = (g.number(face.geometry, "semi_angle")?
        * g.angle_scale(face.geometry, "GeneralConeAngleUnitUnestablished")?)
    .tan();
    let support_radius = g.number(face.geometry, "radius")? * g.length_scale() + slope * height;
    let tilt = norm(circle_frame.z.cross(frame.z));
    let support_gap =
        norm(delta - frame.z * height) + radius * tilt + (radius - support_radius).abs();
    need(
        tilt.is_finite()
            && tilt <= 1e-12
            && support_radius > 0.
            && support_gap.is_finite()
            && support_gap <= scalar(quality.consistency_mm),
        "GeneralVertexLoopRingSupport",
        face.record,
    )?;
    let ring = circle.vertices[0];
    let point_on_ring = circle
        .built
        .as_ref()
        .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?
        .endpoints[0];
    let pole_point = point(vertices, pole)?;
    let radial = point_on_ring - frame.origin;
    Ok(Some((
        Seam {
            vertices: [ring, pole],
            endpoints: [point_on_ring, pole_point],
            angle: radial.dot(frame.y).atan2(radial.dot(frame.x)),
        },
        inner.record,
    )))
}

fn full_period(start: f64) -> [f64; 2] {
    let degrees = start * 180. / PI;
    [degrees, degrees + 360.]
}
fn normalized(angle: f64) -> f64 {
    let a = angle % TAU;
    if a < 0. {
        a + TAU
    } else {
        a
    }
}
fn add_cover(cover: &mut Vec<(f64, f64)>, start: f64, span: f64) {
    if start + span <= TAU {
        cover.push((start, start + span));
    } else {
        cover.push((start, TAU));
        cover.push((0., start + span - TAU));
    }
}

fn angles(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    seam: Option<&Seam>,
    quality: &GeneralQuality,
    execution: &mut Execution,
) -> Result<[f64; 2]> {
    if let Some(seam) = seam {
        return Ok(full_period(if seam.angle > 0. {
            seam.angle - TAU
        } else {
            seam.angle
        }));
    }
    let mut counts = BTreeMap::new();
    for u in face.loops.iter().flat_map(super::SourceLoop::edge_uses) {
        *counts.entry(u.edge).or_insert(0usize) += 1;
    }
    for (i, count) in &counts {
        let e = curve(edges, *i)?;
        if *count == 2 && g.is(e.geometry, "LINE") {
            let [a, b] = e
                .built
                .as_ref()
                .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?
                .endpoints;
            let a = a - frame.origin;
            let b = b - frame.origin;
            let t0 = a.dot(frame.y).atan2(a.dot(frame.x));
            let t1 = b.dot(frame.y).atan2(b.dot(frame.x));
            // IEEE remainder, unlike fmod, selects the closest periodic branch.
            let d = t0 - t1;
            let remainder = d - TAU * (d / TAU).round_ties_even();
            if remainder.abs() <= 1e-10 {
                return Ok(full_period(if t0 > 0. { t0 - TAU } else { t0 }));
            }
        }
    }
    let mut cover = Vec::new();
    for u in face.loops.iter().flat_map(super::SourceLoop::edge_uses) {
        let e = curve(edges, u.edge)?;
        let [va, vb] = e.vertices;
        if g.is(e.geometry, "CIRCLE") {
            let cf = g.placement(g.reference(e.geometry, "position")?)?;
            let dc = cf.origin - frame.origin;
            let parallel = cf.z.dot(frame.z);
            if (parallel.abs() - 1.).abs() > 1e-12
                || norm(dc - frame.z * dc.dot(frame.z)) > scalar(quality.consistency_mm)
            {
                return Ok([0., 360.]);
            }
            if va == vb {
                return Ok([0., 360.]);
            }
            let a = point(vertices, va)? - cf.origin;
            let b = point(vertices, vb)? - cf.origin;
            let t0 = a.dot(cf.y).atan2(a.dot(cf.x));
            let mut t1 = b.dot(cf.y).atan2(b.dot(cf.x));
            while t1 <= t0 {
                t1 += TAU;
            }
            let offset = cf.x.dot(frame.y).atan2(cf.x.dot(frame.x));
            let start = normalized(offset + if parallel > 0. { t0 } else { -t1 });
            add_cover(&mut cover, start, t1 - t0);
        } else if g.is(e.geometry, "LINE") {
            let a = point(vertices, va)? - frame.origin;
            let b = point(vertices, vb)? - frame.origin;
            let t0 = normalized(a.dot(frame.y).atan2(a.dot(frame.x)));
            let t1 = normalized(b.dot(frame.y).atan2(b.dot(frame.x)));
            let lo = t0.min(t1);
            let hi = t0.max(t1);
            let span = (hi - lo).min(TAU - (hi - lo));
            if span >= PI - 1e-10 {
                return Ok([0., 360.]);
            }
            add_cover(&mut cover, if hi - lo <= PI { lo } else { hi }, span);
        } else if g.is(e.geometry, "B_SPLINE_CURVE_WITH_KNOTS") {
            // The maintained path requires complete per-span control hulls and
            // a separately verified helical chart. Never replace that evidence
            // by sampled angles while that normalization is still being moved.
            let _ = execution;
            return Err(unsupported(
                "GeneralSplineAngularChartNotYetMigrated",
                e.geometry,
            ));
        } else {
            return Ok([0., 360.]);
        }
    }
    if cover.is_empty() {
        return Ok([0., 360.]);
    }
    cover.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for arc in cover {
        if let Some(last) = merged.last_mut().filter(|last| arc.0 <= last.1 + 1e-12) {
            last.1 = last.1.max(arc.1);
        } else {
            merged.push(arc);
        }
    }
    let mut gap = -1.;
    let mut start = 0.;
    for (i, arc) in merged.iter().enumerate() {
        let next = merged
            .get((i + 1) % merged.len())
            .ok_or(Error::Invalid("GeneralAngularCover"))?
            .0
            + if i + 1 == merged.len() { TAU } else { 0. };
        if next - arc.1 > gap {
            gap = next - arc.1;
            start = normalized(next);
        }
    }
    if gap <= 1e-10 {
        return Ok([0., 360.]);
    }
    start -= 1e-12;
    let mut end = start + TAU - gap + 2e-12;
    if end > TAU {
        start -= TAU;
        end -= TAU;
    }
    Ok([start * 180. / PI, end * 180. / PI])
}

#[allow(clippy::too_many_lines)]
pub(super) fn make(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    quality: &GeneralQuality,
    ring_plan: Option<&ImplicitRingPlan>,
    execution: &mut Execution,
) -> Result<BuiltSurface> {
    let record = face.geometry;
    if g.is(record, "B_SPLINE_SURFACE_WITH_KNOTS") {
        let field = g.field(record, "control_points_list")?;
        let rows = &field
            .value()
            .ok_or(Error::Invalid("GeneralSurfaceControlsMissing"))?
            .children;
        let mut controls = Vec::new();
        for row in rows {
            for value in &row.children {
                controls.push(g.position(g.reference_value(value)?)?);
            }
        }
        let um = g.multiplicities(record, "u_multiplicities")?;
        let vm = g.multiplicities(record, "v_multiplicities")?;
        let uk = g.numbers(record, "u_knots")?;
        let vk = g.numbers(record, "v_knots")?;
        let weights = g.spline_weights(record, true, &controls)?;
        execution.step()?;
        let native = BSplineSurface::create_canonical(&CanonicalSurfaceData {
            u_degree: g.degree(record, "u_degree")?,
            v_degree: g.degree(record, "v_degree")?,
            control_points: &controls,
            form: BSplineSurfaceForm::Unspecified,
            u_multiplicities: &um,
            v_multiplicities: &vm,
            u_knots: &uk,
            v_knots: &vk,
            knot_type: KnotType::Unspecified,
            weights: if weights.is_empty() {
                None
            } else {
                Some(&weights)
            },
            uv_domain: None,
        })?;
        let mut mappings = Vec::new();
        if !weights.is_empty() {
            let (u, v, actual) = native.control_points()?;
            need(
                u == rows.len() && rows.first().is_some_and(|row| v == row.children.len()),
                "GeneralRationalNetShapeChanged",
                record,
            )?;
            if let Some(mapping) = g.verify_rational_net(
                record,
                face.record,
                &controls,
                &weights,
                &actual,
                native.is_rational(),
                scalar(quality.definition_mm),
            )? {
                mappings.push(mapping);
            }
        }
        return Ok(BuiltSurface {
            surface: native.into(),
            seam: None,
            pole_bound: None,
            mappings,
        });
    }
    let frame = g.placement(g.reference(record, "position")?)?;
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for u in face.loops.iter().flat_map(super::SourceLoop::edge_uses) {
        let b = edge_bounds(curve(edges, u.edge)?)?;
        for mask in 0..8 {
            let p = Vec3::new(
                if mask & 1 == 0 { b.min().x } else { b.max().x },
                if mask & 2 == 0 { b.min().y } else { b.max().y },
                if mask & 4 == 0 { b.min().z } else { b.max().z },
            ) - frame.origin;
            let p = Vec3::new(p.dot(frame.x), p.dot(frame.y), p.dot(frame.z));
            min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
    }
    for index in face.loops.iter().filter_map(super::SourceLoop::vertex) {
        let delta = point(vertices, index)? - frame.origin;
        let local = Vec3::new(delta.dot(frame.x), delta.dot(frame.y), delta.dot(frame.z));
        min = Vec3::new(min.x.min(local.x), min.y.min(local.y), min.z.min(local.z));
        max = Vec3::new(max.x.max(local.x), max.y.max(local.y), max.z.max(local.z));
        // This first native scope supports analytic cone poles only. The STEP
        // WR7/WR8 rules prove types; they do NOT prove this geometric condition.
        let slope = (g.number(record, "semi_angle")?
            * g.angle_scale(record, "GeneralConeAngleUnitUnestablished")?)
        .tan();
        let radius = g.number(record, "radius")? * g.length_scale();
        need(
            slope.is_finite() && slope != 0.,
            "GeneralVertexLoopPoleUnestablished",
            face.record,
        )?;
        let pole = frame.origin + frame.z * (-radius / slope);
        need(
            norm(point(vertices, index)? - pole) <= scalar(quality.consistency_mm),
            "GeneralVertexLoopOffPole",
            face.record,
        )?;
    }
    let padding =
        scalar(quality.definition_mm).max(64. * f64::EPSILON * (1. + norm(min) + norm(max)));
    let pad = Vec3::new(padding, padding, padding);
    min -= pad;
    max += pad;
    let pole_cut = cone_pole_seam(g, face, edges, vertices, frame, quality)?;
    let pole_bound = pole_cut.as_ref().map(|(_, bound)| *bound);
    let seam = if let Some((seam, _)) = pole_cut {
        Some(seam)
    } else if let Some(plan) = ring_plan {
        Some(Seam {
            vertices: plan.source_vertices,
            endpoints: plan.seam_endpoints,
            angle: plan.seam_angle_rad,
        })
    } else {
        missing_seam(g, face, edges, vertices, frame, quality)?
    };
    let surface = if g.is(record, "PLANE") {
        execution.step()?;
        Plane::new(
            frame.origin,
            frame.x,
            frame.y,
            Vec2::new(1., 1.),
            &Extent2d::from_bounds(min.x, min.y, max.x, max.y),
        )
        .into()
    } else if g.is(record, "CYLINDRICAL_SURFACE") {
        let [begin, end] = angles(
            g,
            face,
            edges,
            vertices,
            frame,
            seam.as_ref(),
            quality,
            execution,
        )?;
        execution.step()?;
        Cone::new_cylinder_bounded(
            frame.origin + frame.z * min.z,
            frame.x,
            frame.y,
            g.number(record, "radius")? * g.length_scale(),
            begin,
            end,
            max.z - min.z,
            false,
            false,
            false,
        )
        .into()
    } else if g.is(record, "CONICAL_SURFACE") {
        let angle = g.number(record, "semi_angle")?
            * g.angle_scale(record, "GeneralConeAngleUnitUnestablished")?;
        let radius = g.number(record, "radius")? * g.length_scale();
        let slope = angle.tan();
        if slope > 0. {
            min.z = min.z.max(-radius / slope);
        }
        if slope < 0. {
            max.z = max.z.min(-radius / slope);
        }
        let mut shifted = frame;
        shifted.origin += frame.z * min.z;
        let [begin, end] = angles(
            g,
            face,
            edges,
            vertices,
            frame,
            seam.as_ref(),
            quality,
            execution,
        )?;
        execution.step()?;
        let mut cone = Cone::create_canonical(
            g.kernel(),
            &shifted.native(),
            (radius + slope * min.z).max(0.),
            (radius + slope * max.z).max(0.),
            max.z - min.z,
            false,
            false,
        )?;
        let domain = cone.rev().step_uv_domain();
        execution.step()?;
        cone.adjust_step_uv_domain(
            g.kernel(),
            &Extent2d::from_bounds(begin, domain.min().y, end, domain.max().y),
        )?;
        cone.into()
    } else if g.is(record, "TOROIDAL_SURFACE") {
        let major = g.number(record, "major_radius")? * g.length_scale();
        let minor = g.number(record, "minor_radius")? * g.length_scale();
        if !(major.is_finite() && minor.is_finite() && minor > 0. && major > minor) {
            return Err(unsupported("GeneralToroidalRingRequired", record));
        }
        if major <= EFF_ZERO || minor <= EFF_ZERO {
            return Err(unsupported("GeneralToroidalKernelRadiusRange", record));
        }
        execution.step()?;
        Torus::create_canonical(
            g.kernel(),
            &frame.native(),
            major,
            minor,
            TorusOptions::default(),
            Some(&Extent2d::from_bounds(0., 0., 360., 360.)),
        )?
        .into()
    } else {
        return Err(unsupported("GeneralUnsupportedSurface", record));
    };
    Ok(BuiltSurface {
        surface,
        seam,
        pole_bound,
        mappings: Vec::new(),
    })
}

#[cfg(test)]
mod ring_tests {
    use super::*;
    use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};

    const CYLINDER: &str = include_str!("../../tests/fixtures/cylinder.step");

    fn quality() -> GeneralQuality {
        GeneralQuality {
            id: "Q-implicit-ring-public-control".into(),
            definition_mm: Rational::decimal("0.0000001").unwrap(),
            consistency_mm: Rational::decimal("0.000001").unwrap(),
            derived_uv_mm: Rational::decimal("0.000001").unwrap(),
            query_mm: Rational::decimal("0.000001").unwrap(),
            maximum_work_tolerance_mm: Rational::decimal("0.00001").unwrap(),
            maximum_tolerance_growth_mm: Rational::ZERO,
        }
    }

    fn source(text: &str) -> s::Snapshot {
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        let result = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                geometry_scope: s::GeometryScope::GeneralBrep,
                physical_context: Some("12".into()),
                check_geometry_dependencies: true,
                ..s::Options::default()
            },
        );
        result
            .snapshot
            .unwrap_or_else(|| panic!("public defining geometry: {:?}", result.diagnostics))
    }

    fn record(source: &s::Snapshot, id: &str) -> usize {
        source
            .source()
            .records()
            .iter()
            .position(|r| r.id == id)
            .unwrap()
    }

    // Independently specified two-ring control over the public R=3,z=[0,7]
    // definitions. This helper does not infer loops from the native result or
    // grant a source shell qualification; integration tests own full STEP faces.
    fn inputs(
        source: &s::Snapshot,
        g: &Geometry<'_>,
    ) -> (SourceFace, Vec<SourceEdge>, Vec<SourceVertex>) {
        let mut vertices = Vec::new();
        for (vertex, position) in [("120", "102"), ("121", "103")] {
            vertices.push(SourceVertex {
                record: record(source, vertex),
                point: g.position(record(source, position)).unwrap(),
            });
        }
        let mut edges = Vec::new();
        let mut loops = Vec::new();
        for (i, (edge, geometry, oriented, bound, loop_id)) in [
            ("130", "110", "152", "170", "160"),
            ("131", "111", "154", "171", "161"),
        ]
        .into_iter()
        .enumerate()
        {
            let point = vertices[i].point;
            let mut execution = Execution { used: 0, max: 1000 };
            let built = g
                .make_curve(
                    record(source, geometry),
                    record(source, edge),
                    point,
                    point,
                    true,
                    1e-7,
                    &mut execution,
                )
                .unwrap();
            edges.push(SourceEdge {
                record: record(source, edge),
                geometry: record(source, geometry),
                vertices: [i, i],
                same_sense: true,
                built: Some(built),
            });
            loops.push(super::super::SourceLoop {
                record: record(source, loop_id),
                bound: record(source, bound),
                outer: i == 0,
                orientation: true,
                content: super::super::LoopContent::Edges(vec![super::super::SourceUse {
                    edge: i,
                    record: record(source, oriented),
                    forward: i == 0,
                }]),
            });
        }
        (
            SourceFace {
                record: record(source, "182"),
                geometry: record(source, "142"),
                same_sense: true,
                loops,
            },
            edges,
            vertices,
        )
    }

    #[test]
    fn opposite_ring_cut_keeps_original_native_conic_and_source_anchors() {
        // Lower anchor +Y, upper anchor +X. The independent cut is (0,3,7),
        // regardless of the upper circle's frame handedness or reference axis.
        for reversed in [false, true] {
            let mut text = CYLINDER.replace("(0.,3.,7.)", "(3.,0.,7.)");
            if reversed {
                text = text.replace(
                    "#107=AXIS2_PLACEMENT_3D('',#101,#104,#105);",
                    "#108=DIRECTION('',(0.,0.,-1.));\n#109=DIRECTION('',(0.,1.,0.));\n#107=AXIS2_PLACEMENT_3D('',#101,#108,#109);",
                );
            }
            let source = source(&text);
            let kernel = solidcore::kernel::Kernel::default();
            let g = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
            let (face, edges, vertices) = inputs(&source, &g);
            let built = edges[1].built.as_ref().unwrap();
            let native = built.native.as_ref().unwrap();
            let CurveKind::Ellipse(circle) = native.kind() else {
                panic!("original circle")
            };
            let controls = circle.base().gw_nurb().unwrap().cpts().to_vec();
            let interval = built.interval;
            let mut execution = Execution { used: 0, max: 2 };
            let plan =
                plan_implicit_rings(&g, &face, &edges, &vertices, &quality(), &mut execution)
                    .unwrap()
                    .unwrap();
            assert_eq!(execution.used, 2);
            assert_eq!(plan.ring_edges, [0, 1]);
            assert_eq!(plan.source_vertices, [0, 1]);
            assert_eq!(plan.split.edge, 1);
            assert!(
                interval.min() < plan.split.native_parameter
                    && plan.split.native_parameter < interval.max()
            );
            assert!((plan.split.native_parameter - 90.).abs() > 1.);
            assert!(norm(plan.split.point - Vec3::new(0., 3., 7.)) < 1e-12);
            assert!(norm(plan.seam_endpoints[0] - Vec3::new(0., 3., 0.)) < 1e-12);
            assert_eq!(plan.seam_endpoints[1], plan.split.point);
            assert!((plan.seam_angle_rad - PI / 2.).abs() < 1e-12);
            assert!(
                norm(
                    native
                        .evaluate_point(&kernel, plan.split.native_parameter)
                        .unwrap()
                        - Vec3::new(0., 3., 7.)
                ) < 1e-12
            );
            assert_eq!(circle.base().gw_nurb().unwrap().cpts(), controls);
            assert_eq!(built.interval, interval);
            assert_eq!(vertices[0].point, Vec3::new(0., 3., 0.));
            assert_eq!(vertices[1].point, Vec3::new(3., 0., 7.));
            assert_eq!(source.source().bytes(), text.as_bytes());
            let mut execution = Execution { used: 0, max: 100 };
            let surface = make(
                &g,
                &face,
                &edges,
                &vertices,
                &quality(),
                Some(&plan),
                &mut execution,
            )
            .unwrap();
            let angular = surface.surface.step_uv_domain();
            assert!((angular.min().x + 270.).abs() < 1e-12);
            assert!((angular.max().x - 90.).abs() < 1e-12);
        }
    }

    #[test]
    fn phase_planning_precharges_native_operations_and_aligned_rings_need_no_cut() {
        for split in [false, true] {
            let text = if split {
                CYLINDER.replace("(0.,3.,7.)", "(3.,0.,7.)")
            } else {
                CYLINDER.to_owned()
            };
            let source = source(&text);
            let kernel = solidcore::kernel::Kernel::default();
            let g = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
            let (face, mut edges, vertices) = inputs(&source, &g);
            if split {
                for max in 0..2 {
                    let mut execution = Execution { used: 0, max };
                    assert!(matches!(
                        plan_implicit_rings(
                            &g,
                            &face,
                            &edges,
                            &vertices,
                            &quality(),
                            &mut execution
                        ),
                        Err(Error::ResourceLimit)
                    ));
                    assert_eq!(execution.used, max);
                }
                edges[1].built.as_mut().unwrap().native.take();
                let mut execution = Execution { used: 0, max: 2 };
                assert!(matches!(
                    plan_implicit_rings(&g, &face, &edges, &vertices, &quality(), &mut execution),
                    Err(Error::Invalid("GeneralRingPlanningAfterCurveTransfer"))
                ));
                assert_eq!(execution.used, 0);
            } else {
                let mut execution = Execution { used: 0, max: 0 };
                assert!(plan_implicit_rings(
                    &g,
                    &face,
                    &edges,
                    &vertices,
                    &quality(),
                    &mut execution
                )
                .unwrap()
                .is_none());
                assert_eq!(execution.used, 0);
                assert!(missing_seam(
                    &g,
                    &face,
                    &edges,
                    &vertices,
                    g.placement(record(&source, "106")).unwrap(),
                    &quality()
                )
                .unwrap()
                .is_some());
            }
        }
    }

    #[test]
    fn thin_distinct_rings_do_not_use_consistency_as_a_feature_size_floor() {
        // Independent thin control: h=0.5 micrometers is below the chosen
        // 1-micrometer consistency allowance but above a 0.01-micrometer native
        // working tolerance. No circle, anchor or seam may be enlarged to 1 um.
        let height = 5e-7;
        let working_tolerance = 1e-8;
        let text = CYLINDER
            .replace("(0.,0.,7.)", "(0.,0.,0.0000005)")
            .replace("(0.,3.,7.)", "(0.,3.,0.0000005)");
        let source = source(&text);
        let kernel = solidcore::kernel::Kernel::default();
        let g = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
        let (face, edges, vertices) = inputs(&source, &g);
        let mut execution = Execution { used: 0, max: 0 };
        assert!(
            plan_implicit_rings(&g, &face, &edges, &vertices, &quality(), &mut execution,)
                .unwrap()
                .is_none()
        );
        assert_eq!(execution.used, 0);
        let seam = missing_seam(
            &g,
            &face,
            &edges,
            &vertices,
            g.placement(record(&source, "106")).unwrap(),
            &quality(),
        )
        .unwrap()
        .unwrap();
        let actual = norm(seam.endpoints[1] - seam.endpoints[0]);
        assert!((actual - height).abs() < 1e-18);
        assert!(actual > working_tolerance && actual < scalar(quality().consistency_mm));
        assert_eq!(vertices[1].point, Vec3::new(0., 3., height));
        assert_eq!(source.source().bytes(), text.as_bytes());
    }

    #[test]
    fn matching_anchors_do_not_excuse_a_tilted_ring_support_or_coincident_planes() {
        for (text, key) in [
            (
                CYLINDER.replace(
                    "#107=AXIS2_PLACEMENT_3D('',#101,#104,#105);",
                    "#108=DIRECTION('',(1.,0.,1.));\n#107=AXIS2_PLACEMENT_3D('',#101,#108,#105);",
                ),
                "GeneralImplicitRingSupport",
            ),
            (
                CYLINDER
                    .replace("(0.,0.,7.)", "(0.,0.,0.)")
                    .replace("(0.,3.,7.)", "(0.,3.,0.)"),
                "GeneralImplicitSeamHeight",
            ),
        ] {
            let source = source(&text);
            let kernel = solidcore::kernel::Kernel::default();
            let g = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
            let (face, edges, vertices) = inputs(&source, &g);
            let mut execution = Execution { used: 0, max: 100 };
            assert!(
                matches!(plan_implicit_rings(&g, &face, &edges, &vertices, &quality(), &mut execution), Err(Error::General { key: actual, kind: crate::GeneralFailureKind::Postcheck, .. }) if actual == key)
            );
            assert_eq!(execution.used, 0);
        }
    }
}
