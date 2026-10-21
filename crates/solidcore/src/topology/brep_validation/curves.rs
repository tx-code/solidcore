// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Ordered dynamic Curve/B-spline/Line assertions on actual owned geometry.
//!
//! Upstream: maintained `d8ed45a`, `SmCurve.cpp`23879--23935,
//! `SmBSplineCurveValidation.cpp`362--828, `SmLine.cpp`2345--2406.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_edge_curve`], [`assert_uv_curve`] | concrete curve `AssertValid` |
//! Deviations: checked arena access replaces unsafe dereferences. Geometry
//! attribute ownership and warning transport remain separate migration gaps.
//! Unavailable failed evaluation outputs are explicit, never guessed derivatives.

use super::{
    local::ignore_native_status, AssertValidOptions, Brep, CheckLevel, EdgeId, EdgeuseId, Object,
    Reporter,
};
use crate::curve::{
    curve::p7::{PropertyAnalysis, PropertyAnalysisOptions},
    curve::{Curve, CurveKind, CurvePropertyType},
    regularity, validity,
};
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::{
    assert_array::{labels, AssertArray},
    solution_array::SolutionArray,
    tol::{self, SmallTopoType, ZONE_TOL_3D},
};
use crate::kernel::Kernel;
use crate::math::{Point2, Vec3, BIG_DOUBLE, EFF_ZERO, UNDEF_DOUBLE};
use crate::nurbs::{b_spline_curve::BoundingBoxRequest, geom::dist_cpt_cpt};
use crate::topology::brep_cache::{arena_identity, CacheObject};

#[derive(Clone, Copy)]
enum Slot {
    Edge(EdgeId),
    Uv(EdgeuseId),
}
fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::curve")
}
fn curve(b: &Brep, slot: Slot) -> Result<&Curve> {
    match slot {
        Slot::Edge(id) => b.edge(id)?.curve.as_ref().ok_or_else(missing),
        Slot::Uv(id) => b.edgeuse(id)?.uv_trim_curve.as_ref().ok_or_else(missing),
    }
}
fn curve_mut(b: &mut Brep, slot: Slot) -> Result<&mut Curve> {
    match slot {
        Slot::Edge(id) => b.edge_mut(id)?.curve.as_mut().ok_or_else(missing),
        Slot::Uv(id) => b
            .edgeuse_mut(id)?
            .uv_trim_curve
            .as_mut()
            .ok_or_else(missing),
    }
}
fn object(b: &Brep, slot: Slot) -> Result<Object> {
    let name = curve(b, slot)?.class_string();
    Ok(match slot {
        Slot::Edge(id) => Object::Curve(id, name),
        Slot::Uv(id) => Object::UvCurve(id, name),
    })
}

/// Assert the actual defining curve of an edge. This does not assert the edge,
/// its uses or model acceptance. Unsupported subtype bodies remain explicit.
pub fn assert_edge_curve(
    k: &mut Kernel,
    b: &mut Brep,
    id: EdgeId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    assertions(k, b, Slot::Edge(id), options, reports)
}
/// Assert a use's own stored UV curve (no mate fallback or trim generation).
pub fn assert_uv_curve(
    k: &mut Kernel,
    b: &mut Brep,
    id: EdgeuseId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    assertions(k, b, Slot::Uv(id), options, reports)
}
fn assertions(
    k: &mut Kernel,
    b: &mut Brep,
    slot: Slot,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    let mut r = Reporter { reports };
    match curve(b, slot)?.kind() {
        CurveKind::Line(_) => {
            let mut valid = true;
            if !matches!(options.level, CheckLevel::Given(_)) {
                valid &= bspline(k, b, slot, options, &mut r)?;
            }
            valid &= line(b, slot, &mut r)?;
            Ok(valid)
        }
        CurveKind::BSpline(_) => bspline(k, b, slot, options, &mut r),
        _ => Err(SolidError::unported(
            "concrete curve subclass AssertValid",
            "fork.topology_validation",
        )),
    }
}
fn base(b: &Brep, slot: Slot, r: &mut Reporter<'_>) -> Result<bool> {
    let c = curve(b, slot)?;
    let who = object(b, slot)?;
    // As for Surface: geometry has no migrated attribute container. Do not
    // validate a fabricated parallel topology/notification graph instead.
    let mut valid = r.class_predicate(labels::A_OBJECT_TYPE, 0, who, true)?;
    let allowed = c.owner().is_none_or(|owner| {
        [
            labels::EDGE_TYPE,
            labels::EDGEUSE_TYPE,
            labels::SURF_OF_EXTRUSION_TYPE,
            labels::SURF_OF_REVOLUTION_TYPE,
            labels::CRV_IN_VOLUME_TYPE,
            labels::OFFSET_CURVE_TYPE,
            labels::CRV_ON_SURF_TYPE,
        ]
        .contains(&owner.type_code)
            || (c.dim() == 2 && owner.type_code == labels::FACE_TYPE)
    });
    valid &= r.class_predicate(labels::CURVE_TYPE, 0, who, allowed)?;
    Ok(valid)
}
fn zone(k: &Kernel, b: &Brep, c: &Curve) -> Result<f64> {
    let default = || tol::zone_tol_3d(k, SmallTopoType::NormalSize);
    let Some(owner) = c.owner() else {
        return Ok(default());
    };
    let raw = u32::try_from(owner.key.0).map_err(|_| missing())?;
    if owner.type_code == labels::EDGE_TYPE {
        let id = EdgeId::from_index(raw);
        if arena_identity(CacheObject::Edge(id)) != owner {
            return Err(missing());
        }
        return Ok(b.edge(id)?.tolerance);
    }
    if owner.type_code == labels::EDGEUSE_TYPE {
        let id = EdgeuseId::from_index(raw);
        // set_uv_trim_curve stores the use's raw arena index in this namespace.
        if u64::from(id.index()) != owner.key.0 {
            return Err(missing());
        }
        return b
            .edgeuse(id)?
            .edge
            .map(|id| b.edge(id).map(|e| e.tolerance))
            .transpose()
            .map(|v| v.unwrap_or_else(default));
    }
    if owner.type_code == labels::FACE_TYPE {
        return Ok(default());
    }
    Err(SolidError::unported(
        "SmCurve::GetEdge generated-geometry owner resolution",
        "fork.topology_validation",
    ))
}
fn param_report(
    r: &mut Reporter<'_>,
    who: Object,
    test: u64,
    passed: bool,
    tol: f64,
    initial: f64,
    param: f64,
    value: Option<f64>,
) -> Result<bool> {
    let result = r.class_value(labels::B_SPLINE_CURVE_TYPE, test, who, passed, tol, initial)?;
    if let Some(last) = r
        .reports
        .as_deref_mut()
        .and_then(AssertArray::last_added_mut)
    {
        last.set_owner_param(Point2::new(param, 0.));
        if let Some(v) = value {
            last.set_value(v);
        }
    }
    Ok(result)
}
fn derivatives(c: &Curve, t: f64, left: bool) -> Result<Vec3> {
    match c.evaluate(t, 1, left, true) {
        Ok(v) => v.get(1).copied().ok_or_else(missing),
        Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => Err(SolidError::unported(
            "curve AssertValid failed Evaluate outputs",
            "fork.topology_validation",
        )),
        Err(e) => Err(e),
    }
}
fn gpa(
    k: &Kernel,
    c: &Curve,
    property: CurvePropertyType,
    sep: f64,
    out: &mut PropertyAnalysis<EntityRef>,
) -> Result<()> {
    ignore_native_status(c.global_property_analysis_into(
        k,
        &c.natural_interval(),
        property,
        sep,
        &PropertyAnalysisOptions::default(),
        out,
    ))?;
    Ok(())
}
#[allow(clippy::too_many_lines)]
fn bspline(
    k: &Kernel,
    b: &mut Brep,
    slot: Slot,
    options: AssertValidOptions<'_>,
    r: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = true;
    if !matches!(options.level, CheckLevel::Given(_)) {
        valid &= base(b, slot, r)?;
    }
    ignore_native_status(validity::materialize_bspline(k, curve_mut(b, slot)?))?;
    let c = curve(b, slot)?;
    let who = object(b, slot)?;
    let Some(nurb) = validity::bspline_base(c)?.gw_nurb() else {
        valid &= r.class_predicate(labels::B_SPLINE_CURVE_TYPE, 0, who, false)?;
        return Ok(valid);
    };
    valid &= r.class_predicate(labels::B_SPLINE_CURVE_TYPE, 0, who, true)?;
    let (knots, mults) = c.knots(None)?;
    let degree = c.degree()?;
    let ends = mults
        .first()
        .zip(mults.last())
        .is_some_and(|(a, b)| *a == degree + 1 && *b == degree + 1);
    valid &= r.class_predicate(labels::B_SPLINE_CURVE_TYPE, 1, who, ends)?;
    if !valid {
        return Ok(valid);
    }
    for mult in mults.iter().skip(1).take(mults.len().saturating_sub(2)) {
        valid &= r.class_predicate(labels::B_SPLINE_CURVE_TYPE, 2, who, *mult <= degree)?;
    }
    if !valid {
        return Ok(valid);
    }
    valid &= r.class_predicate(
        labels::B_SPLINE_CURVE_TYPE,
        16,
        who,
        nurb.m().checked_add(1) == nurb.n().checked_add(degree).and_then(|v| v.checked_add(2)),
    )?;
    if !valid {
        return Ok(valid);
    }
    let zone = zone(k, b, c)?;
    let degen = c.is_degenerate(tol::xsect_tol_3d_from_zones(k, zone, zone), None)?;
    if !degen {
        let pts = nurb.cpts();
        let first = *pts.first().ok_or_else(missing)?;
        let second = *pts.get(1).ok_or_else(missing)?;
        let last = *pts.last().ok_or_else(missing)?;
        let penultimate = *pts
            .get(pts.len().checked_sub(2).ok_or_else(missing)?)
            .ok_or_else(missing)?;
        let start_tol = EFF_ZERO * (1. + first.to_euclid().max_dimension());
        let start_dist = dist_cpt_cpt(first, second);
        let start_coincident = start_dist.abs() <= start_tol;
        if start_coincident {
            valid &= r.class_value(
                labels::B_SPLINE_CURVE_TYPE,
                3,
                who,
                !start_coincident,
                start_tol,
                start_dist,
            )?;
        }
        let end_tol = EFF_ZERO * (1. + last.to_euclid().max_dimension());
        let end_dist = dist_cpt_cpt(penultimate, last);
        if end_dist.abs() <= end_tol {
            // Source rule4 intentionally reuses the START predicate/tol/value.
            valid &= r.class_value(
                labels::B_SPLINE_CURVE_TYPE,
                4,
                who,
                !start_coincident,
                start_tol,
                start_dist,
            )?;
        }
    }
    if !degen && !c.is_analytic() {
        let dim = c.dim();
        let cosine = 0.999_847_695_156_391_3;
        for (index, mult) in mults
            .iter()
            .enumerate()
            .skip(1)
            .take(mults.len().saturating_sub(2))
        {
            if !valid {
                break;
            }
            if *mult < degree {
                continue;
            }
            let t = *knots.get(index).ok_or_else(missing)?;
            // Native lower FALSE, upper TRUE; Evaluate's nonzero tangent default
            // is TRUE. Do not swap sides based on their English names.
            let lower = derivatives(c, t, false)?;
            let Ok(a) = lower.unitize() else {
                if dim == 2 || dim == 3 {
                    valid &= param_report(
                        r,
                        who,
                        if dim == 2 { 5 } else { 6 },
                        false,
                        EFF_ZERO,
                        UNDEF_DOUBLE,
                        t,
                        Some(lower.length()),
                    )?;
                }
                continue;
            };
            let upper = derivatives(c, t, true)?;
            let Ok(d) = upper.unitize() else {
                // Both sides use indices5/6, not the obsolete comments7/8.
                if dim == 2 || dim == 3 {
                    let _ = param_report(
                        r,
                        who,
                        if dim == 2 { 5 } else { 6 },
                        false,
                        EFF_ZERO,
                        UNDEF_DOUBLE,
                        t,
                        Some(upper.length()),
                    )?;
                }
                valid = false;
                continue;
            };
            let dot = a.dot(d);
            if dot < cosine {
                if dim == 2 || dim == 3 {
                    let _ = param_report(
                        r,
                        who,
                        if dim == 2 { 7 } else { 8 },
                        dot > cosine,
                        cosine,
                        dot,
                        t,
                        Some(dot),
                    )?;
                }
                valid = false;
            }
        }
        let domain = c.natural_interval();
        let bbox = c.calculate_bounding_box(
            k,
            &domain,
            BoundingBoxRequest {
                normal: true,
                pseudo: false,
                polar: false,
            },
            false,
        )?;
        let size = bbox.normal.ok_or_else(missing)?.size().length();
        let size_t = domain.length();
        let tolerance = if c.is_bounded() {
            (size / size_t) / 10000.
        } else {
            1e-3
        };
        let separation = if c.is_bounded() {
            size / 20f64.max(size_t)
        } else {
            5.
        };
        let proof = if matches!(c.kind(), CurveKind::BSpline(_)) {
            regularity::prove(nurb, dim, tolerance)
        } else {
            regularity::Proof::default()
        };
        let mut out = PropertyAnalysis {
            solutions: SolutionArray::new(),
            comb_points: Vec::new(),
            comb_vecs: Vec::new(),
        };
        if !proof.speed {
            gpa(
                k,
                c,
                CurvePropertyType::MinimizeFirstDerivative,
                separation,
                &mut out,
            )?;
        }
        for solution in out.solutions.as_slice() {
            let t = solution.start.param(0)?;
            if domain
                .is_value_on_boundary(t, EFF_ZERO * (1. + domain.length()))
                .0
            {
                continue;
            }
            let length = derivatives(c, t, false)?.length();
            if length < tolerance {
                if dim == 2 || dim == 3 {
                    valid &= param_report(
                        r,
                        who,
                        if dim == 2 { 11 } else { 12 },
                        length >= tolerance,
                        tolerance,
                        length,
                        t,
                        Some(length),
                    )?;
                }
                break;
            }
        }
        let radius_test = if dim == 2 { 13 } else { 14 };
        let requested = match options.level {
            CheckLevel::Zero | CheckLevel::One | CheckLevel::Two => true,
            CheckLevel::Heal => false,
            CheckLevel::Given(v) => v.contains(&radius_test),
        };
        if !proof.radius && requested {
            gpa(
                k,
                c,
                CurvePropertyType::MinimizeRadiusOfCurvature,
                separation,
                &mut out,
            )?;
            let mut radius = BIG_DOUBLE;
            let mut parameter = -BIG_DOUBLE;
            for solution in out.solutions.as_slice() {
                if solution.start.solution_value < radius {
                    radius = solution.start.solution_value;
                    parameter = solution.start.param(0)?;
                }
            }
            if radius < tolerance && (dim == 2 || dim == 3) {
                valid &= param_report(
                    r,
                    who,
                    radius_test,
                    radius >= tolerance,
                    tolerance,
                    radius,
                    parameter,
                    None,
                )?;
            }
        }
    }
    let pole_tol = ZONE_TOL_3D / 10.;
    let pole = c.has_internal_pole(pole_tol)?;
    valid &= r.class_value(
        labels::B_SPLINE_CURVE_TYPE,
        15,
        who,
        !pole,
        pole_tol,
        UNDEF_DOUBLE,
    )?;
    Ok(valid)
}
fn line(b: &Brep, slot: Slot, r: &mut Reporter<'_>) -> Result<bool> {
    let c = curve(b, slot)?;
    let CurveKind::Line(line) = c.kind() else {
        return Err(missing());
    };
    let mut valid = true;
    if line.is_bounded() && line.base().gw_nurb().is_some() {
        let domain = line.natural_interval();
        let analytic = line.step_interval();
        let a = line
            .evaluate_step(analytic.min(), 0)
            .first()
            .copied()
            .ok_or_else(missing)?;
        let d = line
            .evaluate_step(analytic.max(), 0)
            .first()
            .copied()
            .ok_or_else(missing)?;
        let first = line.base().evaluate_point(domain.min())?;
        let last = line.base().evaluate_point(domain.max())?;
        let tol = EFF_ZERO * (1. + a.max_dimension().max(d.max_dimension()));
        let (i, j, start, end) = if line.inside_out() {
            (0, 1, last, first)
        } else {
            (2, 3, first, last)
        };
        let who = object(b, slot)?;
        valid &= r.class_value(
            labels::LINE_TYPE,
            i,
            who,
            a.closer_than(tol, start),
            tol,
            a.distance(start),
        )?;
        valid &= r.class_value(
            labels::LINE_TYPE,
            j,
            who,
            d.closer_than(tol, end),
            tol,
            d.distance(end),
        )?;
    }
    Ok(valid)
}
