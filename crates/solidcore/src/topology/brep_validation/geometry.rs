// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native geometric callees of pointer validation. Missing algorithms must fail
//! explicitly, never make a completed topology walk stand in for geometry checks.
//!
//! Upstream: `SmBrep.cpp::sm_CheckEdgeuseGeometry` (`SM_CHANGE_CHECK` off) and
//! `SmEdge.cpp::SmEdge::ValidateGeometry` at maintained fork `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | `check_edgeuse` | `sm_CheckEdgeuseGeometry` |
//! | `validate_edge` | `SmEdge::ValidateGeometry` |
//!
//! Deviations: checked arena access; exact owned copies represent borrowed UV
//! and B-spline surface values. Release warning-only branches do not become new
//! failures. Unavailable surface dispatch remains an explicit dependency.
use super::{Brep, EdgeId, EdgeuseId};
use crate::curve::{
    crv_on_surf::CrvOnSurf,
    curve::{Curve, CurveKind},
};
use crate::error::{ignored, Result, SolidError};
use crate::geom::tol::are_same;
use crate::kernel::Kernel;
use crate::math::Point2;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::SurfaceKind;
use crate::topology::brep_cache::{arena_identity, CacheObject};
use crate::topology::{edge, edgeuse::p1 as eu};

const OP: &str = "topology::brep_validation::geometry";
fn required<T>(value: Option<T>) -> Result<T> {
    value.ok_or(SolidError::null_pointer(OP))
}

pub(super) fn check_edgeuse(k: &Kernel, b: &mut Brep, id: EdgeuseId) -> Result<()> {
    if eu::uv_trim_curve(b, id)?.0.is_none() {
        return Ok(());
    }
    let lu = required(eu::loopuse(b, id)?)?;
    let fu = required(b.loopuse(lu)?.faceuse)?;
    let face = required(b.faceuse(fu)?.face)?;
    let vu = required(b.edgeuse(id)?.vertexuse)?;
    let vertex = required(b.vertexuse(vu)?.vertex)?;
    let cw = required(b.edgeuse(id)?.cw)?;
    // Preserve all four SER calls and their order. The two gap-distance branches
    // in the Release C++ helper only issue debug warnings, not failed checks.
    let start =
        crate::topology::edgeuse::p3::normalized_evaluate(k, b, id, 0., true, false, true)?.point;
    let _start_3d =
        crate::topology::edgeuse::p3::normalized_evaluate(k, b, id, 0., false, false, true)?.point;
    let _previous =
        crate::topology::edgeuse::p3::normalized_evaluate(k, b, cw, 1., true, false, true)?.point;
    let _previous_3d =
        crate::topology::edgeuse::p3::normalized_evaluate(k, b, cw, 1., false, false, true)?.point;
    let surface = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or(SolidError::null_pointer(OP))?;
    let projected = surface.evaluate_point(Point2::new(start.x, start.y))?;
    let _distance = projected.distance(b.vertex(vertex)?.point);
    Ok(())
}

pub(super) fn validate_edge(k: &Kernel, b: &mut Brep, id: EdgeId, fix: bool) -> Result<()> {
    let primary = *b
        .edge(id)?
        .edgeuses
        .first()
        .ok_or(SolidError::null_pointer(OP))?;
    let same = if b.edgeuse(primary)?.orientation == OrientType::Same {
        primary
    } else {
        required(eu::mate(b, primary)?)?
    };
    let start = required(b.vertexuse(required(b.edgeuse(same)?.vertexuse)?)?.vertex)?;
    let end = required(edge::other_vertex(b, id, Some(start))?)?;
    // PassesValidityCheck(REVERSE_DIRECTION) mutates even when bFixTolerances is
    // FALSE. Mutate the actual owned curve, then restore the slot before any error.
    let mut curve = b
        .edge_mut(id)?
        .curve
        .take()
        .ok_or(SolidError::null_pointer(OP))?;
    let checked = curve.check_reverse_direction(k);
    b.edge_mut(id)?.curve = Some(curve);
    checked?; // The returned failure bits warn only; checked call errors are real.
    let curve = b
        .edge(id)?
        .curve
        .as_ref()
        .ok_or(SolidError::null_pointer(OP))?;
    if curve.owner() != Some(arena_identity(CacheObject::Edge(id))) {
        return Err(SolidError::sm_err(OP));
    }
    let interval = b.edge(id)?.interval;
    let first = curve.evaluate_point(interval.min())?;
    let last = curve.evaluate_point(interval.max())?;
    for (vertex, point) in [(start, first), (end, last)] {
        let v = b.vertex(vertex)?;
        let distance = point.distance(v.point);
        if distance > v.tolerance + b.edge(id)?.tolerance {
            if fix && distance < v.tolerance * 100. {
                b.vertex_mut(vertex)?.tolerance = distance * 2.;
            } else {
                return Err(SolidError::sm_err(OP));
            }
        }
    }
    // An interval outside the natural interval is a warning in the C++ body,
    // not a returned failure. Do not turn that warning into a stronger predicate.
    for incidence in b.edge(id)?.edgeuses.clone() {
        if b.edgeuse(incidence)?.uv_trim_curve.is_none() {
            continue;
        }
        if fix {
            let mut uv = b
                .edgeuse_mut(incidence)?
                .uv_trim_curve
                .take()
                .ok_or(SolidError::null_pointer(OP))?;
            let natural = uv.natural_interval();
            let edited = (|| -> Result<()> {
                // Both tests use the captured old interval. If both ends differ,
                // C++ edits twice; preserve notifications and ignored statuses.
                if !are_same(natural.min(), interval.min()) {
                    ignored(uv.edit_parameterization(k, &interval))?;
                }
                if !are_same(natural.max(), interval.max()) {
                    ignored(uv.edit_parameterization(k, &interval))?;
                }
                Ok(())
            })();
            b.edgeuse_mut(incidence)?.uv_trim_curve = Some(uv);
            edited?;
        }
        let face = required(eu::face(b, incidence)?)?;
        let surface = b
            .face(face)?
            .surface
            .as_ref()
            .ok_or(SolidError::null_pointer(OP))?;
        let surface = match surface.kind() {
            SurfaceKind::BSpline(s) => s.clone_exact(),
            _ => {
                // UNPORTED(curve.crv_on_surf): SmCrvOnSurf over non-B-spline surface
                return Err(SolidError::unported(
                    "SmCrvOnSurf over non-B-spline surface",
                    "curve.crv_on_surf",
                ));
            }
        };
        let uv = b
            .edgeuse(incidence)?
            .uv_trim_curve
            .as_ref()
            .ok_or(SolidError::null_pointer(OP))?;
        let uv_interval = uv.natural_interval();
        // C++ borrows these definitions. Do not run copy-constructor knot fixup
        // while representing the borrowed values with owned Rust storage.
        let on_surface = Curve::from(CrvOnSurf::new(k, borrowed_uv_copy(uv)?, surface, None, 0)?);
        let defining = b
            .edge(id)?
            .curve
            .as_ref()
            .ok_or(SolidError::null_pointer(OP))?;
        // CurveMaxDistanceBetween's failure status and excessive distance warn
        // only upstream; Unported still means the algorithm was not executed.
        ignored(defining.curve_max_distance_between(
            k,
            &interval,
            &on_surface,
            uv_interval.min(),
            uv_interval.max(),
            51,
            None,
        ))?;
        let mut uv = b
            .edgeuse_mut(incidence)?
            .uv_trim_curve
            .take()
            .ok_or(SolidError::null_pointer(OP))?;
        let checked = uv.check_reverse_direction(k);
        b.edgeuse_mut(incidence)?.uv_trim_curve = Some(uv);
        checked?;
    }
    Ok(())
}

fn borrowed_uv_copy(curve: &Curve) -> Result<Curve> {
    let copied = match curve.kind() {
        CurveKind::BSpline(c) => Curve::from(c.clone_exact()),
        CurveKind::Line(c) => {
            let mut copy = c.clone();
            *copy.base_mut() = c.base().clone_exact();
            Curve::from(copy)
        }
        CurveKind::Ellipse(c) => {
            let mut copy = c.clone();
            *copy.base_mut() = c.base().clone_exact();
            Curve::from(copy)
        }
        CurveKind::Hyperbola(c) => {
            let mut copy = c.clone();
            copy.base = c.base.clone_exact();
            Curve::from(copy)
        }
        CurveKind::Parabola(c) => {
            let mut copy = c.clone();
            *copy.base_mut() = c.base().clone_exact();
            Curve::from(copy)
        }
        _ => {
            return Err(SolidError::invalid_input(
                OP,
                "UV curve is not B-spline-derived",
            ))
        }
    };
    Ok(copied)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::{
        b_spline_curve::BSplineCurve,
        crv_basic::Curve as NurbCurve,
        math::{CPoint, NOW},
    };

    #[test]
    fn borrowed_uv_storage_preserves_near_coincident_knots_exactly() {
        let knots = vec![0., 0., 1e-14, 1., 1.];
        let nurb = NurbCurve::new(
            vec![
                CPoint::new(0., 0., 0., NOW),
                CPoint::new(0.5, 0., 0., NOW),
                CPoint::new(1., 0., 0., NOW),
            ],
            1,
            knots.clone(),
        )
        .unwrap();
        let original = Curve::from(BSplineCurve::from_nurb(nurb, 2, true));
        let copy = borrowed_uv_copy(&original).unwrap();
        let CurveKind::BSpline(copy) = copy.kind() else {
            panic!()
        };
        assert_eq!(copy.gw_nurb().unwrap().knots(), knots);
        let CurveKind::BSpline(original) = original.kind() else {
            panic!()
        };
        assert_eq!(original.gw_nurb().unwrap().knots(), knots);
    }
}
