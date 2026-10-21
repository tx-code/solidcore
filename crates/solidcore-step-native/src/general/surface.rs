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
pub(super) struct BuiltSurface {
    pub surface: Surface,
    pub seam: Option<Seam>,
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
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for u in &face_loop.uses {
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
fn missing_seam(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    quality: &GeneralQuality,
) -> Result<Option<Seam>> {
    if !g.is(face.geometry, "CYLINDRICAL_SURFACE") || face.loops.len() != 2 {
        return Ok(None);
    }
    let mut indices = Vec::new();
    for l in &face.loops {
        if l.uses.len() != 1 {
            return Ok(None);
        }
        let e = curve(
            edges,
            l.uses.first().ok_or(Error::Invalid("GeneralLoopUse"))?.edge,
        )?;
        let [a, b] = e.vertices;
        if !g.is(e.geometry, "CIRCLE") || a != b {
            return Ok(None);
        }
        indices.push(a);
    }
    let a = *indices.first().ok_or(Error::Invalid("GeneralSeamVertex"))?;
    let b = *indices.get(1).ok_or(Error::Invalid("GeneralSeamVertex"))?;
    let pa = point(vertices, a)?;
    let pb = point(vertices, b)?;
    let da = pa - frame.origin;
    let db = pb - frame.origin;
    let z0 = da.dot(frame.z);
    let z1 = db.dot(frame.z);
    let radial = da - frame.z * z0;
    let length = norm(radial);
    need(
        length > 0. && length.is_finite(),
        "GeneralDegenerateFrame",
        face.record,
    )?;
    let radial = radial * (1. / length);
    let radius = g.number(face.geometry, "radius")? * g.length_scale();
    let actual_a = frame.origin + frame.z * z0 + radial * radius;
    let actual_b = frame.origin + frame.z * z1 + radial * radius;
    if norm(actual_a - pa) > scalar(quality.consistency_mm)
        || norm(actual_b - pb) > scalar(quality.consistency_mm)
    {
        return Err(unsupported(
            "GeneralImplicitSeamRequiresRingSplit",
            face.record,
        ));
    }
    // A distinct height is checked again against the caller's native tolerance
    // before constructing the derived seam, rather than welding source vertices.
    need(z0 != z1, "GeneralImplicitSeamHeight", face.record)?;
    Ok(Some(Seam {
        vertices: [a, b],
        endpoints: [actual_a, actual_b],
        angle: radial.dot(frame.y).atan2(radial.dot(frame.x)),
    }))
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
    for u in face.loops.iter().flat_map(|l| &l.uses) {
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
    for u in face.loops.iter().flat_map(|l| &l.uses) {
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
            mappings,
        });
    }
    let frame = g.placement(g.reference(record, "position")?)?;
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for u in face.loops.iter().flat_map(|l| &l.uses) {
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
    let padding =
        scalar(quality.definition_mm).max(64. * f64::EPSILON * (1. + norm(min) + norm(max)));
    let pad = Vec3::new(padding, padding, padding);
    min -= pad;
    max += pad;
    let seam = missing_seam(g, face, edges, vertices, frame, quality)?;
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
        let [begin, end] = angles(g, face, edges, vertices, frame, None, quality, execution)?;
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
        mappings: Vec::new(),
    })
}
