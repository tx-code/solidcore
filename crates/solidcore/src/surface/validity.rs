// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native B-spline surface validity queries over exact control-net data.
//!
//! Upstream: `SmBSplineSurface.cpp` (3635--4075), `SmNurbsSrf.cpp`
//! (1950--2210), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`materialize_bspline`] | dynamic `GetOrCreateGwNurbPointer` |
//! | [`excessive_multiplicity`] | `HasKnotMultiplicityGreaterThanDegree` |
//! | [`repeated_end_points`] | `HasRepeatedEndControlPoints` |
//! | [`internal_pole`] | `HasInternalPole` |
//! | [`degenerate_control_row`] | `HasDegenerateControlPointRow` |
//! Deviations: checked control-net indices replace undefined native access.
//! Source asymmetries, bounding-box conventions and early exits are retained.
//! Geometry is not fitted, reparameterized, healed or detached from its owner.

use crate::error::{Result, SolidError};
use crate::geom::extent3d::Extent3d;
use crate::geom::tol::ZONE_TOL_3D;
use crate::kernel::Kernel;
use crate::math::{Point2, Point3};
use crate::nurbs::b_spline_surface::{BSplineSurface, ControlPointNet, SurfParamType};
use crate::surface::surface::{Surface, SurfaceKind};

fn invalid() -> SolidError {
    SolidError::invalid_input(
        "surface::validity",
        "native control-net index outside representation",
    )
}

/// Materialize the actual owned B-spline family object, preserving its concrete
/// analytic/revolution definition. No query copy can replace this native effect.
pub fn materialize_bspline(k: &Kernel, surface: &mut Surface) -> Result<()> {
    // GetOrCreate is NOT MakeNurb unconditionally: a caller/reader may retain a
    // different existing NURB. Rebuilding it would silently hide a mismatch.
    if bspline_base(surface)?.gw_nurb().is_some() {
        return Ok(());
    }
    match surface.kind_mut() {
        SurfaceKind::BSpline(s) => s.make_nurb(),
        SurfaceKind::Plane(s) => s.make_nurb(),
        SurfaceKind::SurfOfRevolution(s) => s.make_nurb(k),
        _ => Err(SolidError::unported(
            "SmBSplineSurface validity on non-BSpline dynamic surface",
            "fork.topology_validation",
        )),
    }
}

/// Stored B-spline base after materialization (no exact-copy constructor).
pub fn bspline_base(surface: &Surface) -> Result<&BSplineSurface> {
    match surface.kind() {
        SurfaceKind::BSpline(s) => Ok(s),
        SurfaceKind::Plane(s) => Ok(s.base()),
        SurfaceKind::SurfOfRevolution(s) => Ok(s.base()),
        _ => Err(invalid()),
    }
}

fn net(surface: &Surface) -> Result<ControlPointNet> {
    bspline_base(surface)?.control_point_net()
}
fn at(net: &ControlPointNet, u: usize, v: usize) -> Result<Point3> {
    if u >= net.u_count || v >= net.v_count {
        return Err(invalid());
    }
    let index = u
        .checked_mul(net.v_count)
        .and_then(|i| i.checked_add(v))
        .ok_or_else(invalid)?;
    net.points.get(index).copied().ok_or_else(invalid)
}
fn tolerance(value: f64) -> f64 {
    if value > 0. {
        value
    } else {
        ZONE_TOL_3D
    }
}

/// Test knot multiplicities, endpoints against degree+1 and interior against
/// degree, in native U-then-V order. Requires a materialized representation.
pub fn excessive_multiplicity(surface: &Surface) -> Result<bool> {
    let base = bspline_base(surface)?;
    if base.gw_nurb().is_none() {
        return Ok(false);
    }
    for dir in [SurfParamType::U, SurfParamType::V] {
        let (_, multiplicities) = base.knots(dir, None)?;
        let degree = base.degree(dir);
        for (i, multiple) in multiplicities.iter().copied().enumerate() {
            let limit = degree + usize::from(i == 0 || i + 1 == multiplicities.len());
            if multiple > limit {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Detect repeated end-control-point pairs, except on singular boundary curves.
/// Three-point length is required even though only the first pair is measured.
pub fn repeated_end_points(surface: &Surface, tol: f64) -> Result<bool> {
    if bspline_base(surface)?.gw_nurb().is_none() {
        return Ok(false);
    }
    let net = net(surface)?;
    let n = net.u_count.checked_sub(1).ok_or_else(invalid)?;
    let m = net.v_count.checked_sub(1).ok_or_else(invalid)?;
    let tol = tolerance(tol);
    for side in 0..4 {
        let (curves, length) = if side < 2 { (n, m) } else { (m, n) };
        if length < 2 {
            continue;
        }
        for line in 0..=curves {
            if line == 0 || line == curves {
                let mut bounds = Extent3d::new();
                let mut degenerate = false;
                for i in 0..=length {
                    let point = if side < 2 {
                        at(&net, line, i)?
                    } else {
                        at(&net, i, line)?
                    };
                    bounds.add_point(point);
                    if i == 1 {
                        degenerate = bounds.is_point_sized(tol);
                        if !degenerate {
                            break;
                        }
                    }
                }
                if degenerate && bounds.is_point_sized(tol) {
                    continue;
                }
            }
            let (a, b) = match side {
                0 => (at(&net, line, 0)?, at(&net, line, 1)?),
                1 => (at(&net, line, m)?, at(&net, line, m - 1)?),
                2 => (at(&net, 0, line)?, at(&net, 1, line)?),
                _ => (at(&net, n, line)?, at(&net, n - 1, line)?),
            };
            // N_DistCptCpt converts rational points to Euclidean coordinates;
            // the original scalar SM_IS_ZERO_TO_TOL comparison includes equality.
            if a.distance(b).abs() <= tol {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Native internal-row/column predicate. Its first nondegenerate row stops BOTH
/// scans and it uses `GetMaxDimension` (absolute coordinates), not span diameter.
pub fn degenerate_control_row(surface: &Surface, tol: f64) -> Result<bool> {
    if bspline_base(surface)?.gw_nurb().is_none() {
        return Ok(false);
    }
    let net = net(surface)?;
    let n = net.u_count.checked_sub(1).ok_or_else(invalid)?;
    let m = net.v_count.checked_sub(1).ok_or_else(invalid)?;
    let tol = tolerance(tol);
    let mut degenerate = true;
    for u in 1..n {
        if !degenerate {
            break;
        }
        let mut bounds = Extent3d::from_point(at(&net, u, 0)?);
        for v in 1..=m {
            bounds.add_point(at(&net, u, v)?);
            if bounds.max_dimension() > tol {
                degenerate = false;
                break;
            }
        }
        if degenerate {
            return Ok(true);
        }
    }
    for v in 1..m {
        if !degenerate {
            break;
        }
        let mut bounds = Extent3d::from_point(at(&net, 0, v)?);
        for u in 1..=n {
            bounds.add_point(at(&net, u, v)?);
            if bounds.max_dimension() > tol {
                degenerate = false;
                break;
            }
        }
        if degenerate {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Seek coincident knot-span control subsets in source U-then-V order. Only the
/// first pass confirms its candidate with `IsSingularity`, exactly as native.
pub fn internal_pole(
    k: &Kernel,
    surface: &Surface,
    tol: f64,
    face_tolerance: Option<f64>,
) -> Result<bool> {
    let base = bspline_base(surface)?;
    if base.gw_nurb().is_none() {
        return Ok(false);
    }
    let net = net(surface)?;
    let (_, mult_u) = base.knots(SurfParamType::U, None)?;
    let (_, mult_v) = base.knots(SurfParamType::V, None)?;
    let p = base.degree(SurfParamType::U);
    let q = base.degree(SurfParamType::V);
    let tol = tolerance(tol);
    let mut count_u = *mult_u.first().ok_or_else(invalid)?;
    for i in 1..mult_u.len().saturating_sub(1) {
        let multiple = *mult_u.get(i).ok_or_else(invalid)?;
        count_u = count_u.checked_add(multiple).ok_or_else(invalid)?;
        let rows = p
            .checked_add(1)
            .and_then(|d| d.checked_sub(multiple))
            .ok_or_else(invalid)?;
        let u = count_u
            .checked_sub(rows)
            .and_then(|n| n.checked_sub(1))
            .ok_or_else(invalid)?;
        let mut count_v = 0usize;
        for multiple in mult_v.iter().take(mult_v.len().saturating_sub(1)).copied() {
            count_v = count_v.checked_add(multiple).ok_or_else(invalid)?;
            let v = count_v
                .checked_sub(1)
                .and_then(|n| n.checked_sub(q))
                .ok_or_else(invalid)?;
            let first = at(&net, u, v)?;
            let mut coincident = true;
            for du in (0..rows).rev() {
                for dv in (1..=q).rev() {
                    coincident &= first.closer_than(tol, at(&net, u + du, v + dv)?);
                    if !coincident {
                        break;
                    }
                }
                if !coincident {
                    break;
                }
            }
            if coincident {
                let u_index = u
                    .checked_add(rows)
                    .and_then(|n| n.checked_sub(1))
                    .ok_or_else(invalid)?;
                let uv = Point2::new(
                    base.greville_abscissa(SurfParamType::U, u_index)?,
                    base.greville_abscissa(SurfParamType::V, v + q)?,
                );
                if surface
                    .is_singularity_with_owner_tolerance(
                        k,
                        uv,
                        crate::surface::surface::p5::SS_UNKNOWN,
                        None,
                        tol,
                        false,
                        face_tolerance,
                    )?
                    .singular
                {
                    return Ok(true);
                }
            }
        }
    }
    let mut count_v = *mult_v.first().ok_or_else(invalid)?;
    for i in 1..mult_v.len().saturating_sub(1) {
        let multiple = *mult_v.get(i).ok_or_else(invalid)?;
        count_v = count_v.checked_add(multiple).ok_or_else(invalid)?;
        let rows = q
            .checked_add(1)
            .and_then(|d| d.checked_sub(multiple))
            .ok_or_else(invalid)?;
        let v = count_v
            .checked_sub(rows)
            .and_then(|n| n.checked_sub(1))
            .ok_or_else(invalid)?;
        let mut count_u = 0usize;
        for multiple in mult_u.iter().take(mult_u.len().saturating_sub(1)).copied() {
            count_u = count_u.checked_add(multiple).ok_or_else(invalid)?;
            let u = count_u
                .checked_sub(1)
                .and_then(|n| n.checked_sub(p))
                .ok_or_else(invalid)?;
            let first = at(&net, u, v)?;
            let mut coincident = true;
            for dv in (0..rows).rev() {
                for du in (1..=p).rev() {
                    coincident &= first.closer_than(tol, at(&net, u + du, v + dv)?);
                    if !coincident {
                        break;
                    }
                }
                if !coincident {
                    break;
                }
            }
            if coincident {
                return Ok(true);
            }
        }
    }
    Ok(false)
}
