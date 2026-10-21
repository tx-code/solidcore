// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual Surface/B-spline surface assertion sequence and native diagnostics.
//!
//! Upstream: `SmSurface.cpp::AssertValid` (35175--35235),
//! `SmBSplineSurface.cpp::AssertValid` (5814--6041), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_surface`] | dynamic Surface/BSplineSurface `AssertValid` |
//! Deviations: owning face supplies missing geometry owner context. Checked
//! errors replace undefined native access. Other dynamic surface subclass
//! assertion bodies outside the primitive/revolution family remain explicit
//! dependencies; no geometry repair is added.

use super::{
    local::ignore_native_status, AssertValidOptions, Brep, CheckLevel, FaceId, Object, Reporter,
};
use crate::curve::curve::Curve;
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray};
use crate::geom::geom_utility::p2::ContinuityType;
use crate::geom::point_grid::PointGrid;
use crate::geom::solution_array::SolutionType;
use crate::geom::tol::{self, SmallTopoType, ANG_TOL_DEG, ZONE_TOL_3D};
use crate::kernel::Kernel;
use crate::math::{EFF_ZERO, EFF_ZERO_DEG, UNDEF_DOUBLE};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::surface::{
    surface::{Surface, SurfaceKind},
    validity,
};

#[path = "surface_primitives.rs"]
mod primitives;

fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::assert_surface")
}
fn surface(b: &Brep, f: FaceId) -> Result<&Surface> {
    b.face(f)?.surface.as_ref().ok_or_else(missing)
}
fn object(b: &Brep, f: FaceId) -> Result<Object> {
    Ok(Object::Surface(f, surface(b, f)?.class_string()))
}

/// Native dynamic assertion, including Plane's own Given/base-chain convention.
/// Concrete primitives execute their own assertions and actual parent chain.
pub fn assert_surface(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    if matches!(
        surface(b, f)?.kind(),
        SurfaceKind::Cone(_) | SurfaceKind::Torus(_) | SurfaceKind::SurfOfRevolution(_)
    ) {
        return primitives::assert_primitive(k, b, f, options, reports);
    }
    if matches!(surface(b, f)?.kind(), SurfaceKind::Plane(_)) {
        let mut report = Reporter { reports };
        let mut valid = true;
        if !matches!(options.level, CheckLevel::Given(_)) {
            valid &= bspline_assertions(k, b, f, options, report.reports.as_deref_mut())?;
        }
        valid &= plane_assertions(k, b, f, &mut report)?;
        return Ok(valid);
    }
    if matches!(surface(b, f)?.kind(), SurfaceKind::BSpline(_)) {
        return bspline_assertions(k, b, f, options, reports);
    }
    // UNPORTED(fork.topology_validation): concrete subclasses add their own rules.
    Err(SolidError::unported(
        "concrete surface subclass AssertValid",
        "fork.topology_validation",
    ))
}

/// Execute the B-spline family dynamic assertion using the actual owned face
/// surface. Its walk argument is ignored by native. Working tolerances/cache
/// materialization are native behavior, not import acceptance or repair policy.
#[allow(clippy::too_many_lines)]
fn bspline_assertions(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    if !matches!(
        surface(b, f)?.kind(),
        SurfaceKind::BSpline(_)
            | SurfaceKind::Plane(_)
            | SurfaceKind::SurfOfRevolution(_)
            | SurfaceKind::Cone(_)
            | SurfaceKind::Torus(_)
    ) {
        // UNPORTED(fork.topology_validation): remaining concrete subclass bodies.
        return Err(SolidError::unported(
            "dynamic surface subclass AssertValid",
            "fork.topology_validation",
        ));
    }
    let mut report = Reporter { reports };
    let mut valid = true;
    if !matches!(options.level, CheckLevel::Given(_)) {
        valid &= base_assertions(k, b, f, &mut report)?;
    }
    // GetOrCreateGwNurbPointer must mutate this exact object. Restore its owning
    // slot before either an ordinary status or a missing callee is propagated.
    let mut owned = b.face_mut(f)?.surface.take().ok_or_else(missing)?;
    let materialized = validity::materialize_bspline(k, &mut owned);
    b.face_mut(f)?.surface = Some(owned);
    ignore_native_status(materialized)?;
    let s = surface(b, f)?;
    let who = object(b, f)?;
    let present = validity::bspline_base(s)?.gw_nurb().is_some();
    valid &= report.class_predicate(labels::B_SPLINE_SURFACE_TYPE, 0, who, present)?;
    if !present {
        // UNPORTED(fork.topology_validation): native later dereferences null or
        // constructs an invalid empty point grid; never fabricate a completion.
        return Err(SolidError::unported(
            "SmBSplineSurface::AssertValid null NURB continuation",
            "fork.topology_validation",
        ));
    }
    valid &= report.class_value(
        labels::B_SPLINE_SURFACE_TYPE,
        1,
        who,
        !validity::excessive_multiplicity(s)?,
        EFF_ZERO,
        UNDEF_DOUBLE,
    )?;
    let tol = ZONE_TOL_3D / 10.;
    // Defaults are from SmSurface.h1476--1484, NOT zero or face tolerance.
    let point = s.is_degenerate_point(k, ZONE_TOL_3D / 10., None)?;
    let mut curve = false;
    let mut internal = false;
    let mut row = false;
    if !point {
        curve = s.is_degenerate_curve(k, EFF_ZERO_DEG, None, ZONE_TOL_3D / 10.)?;
        if !curve {
            let face_tol = s
                .face()?
                .map(|owner| crate::topology::surface_cache::owning_face_tolerance(b, owner))
                .transpose()?;
            internal = validity::internal_pole(k, s, tol, face_tol)?;
            valid &= report.class_value(
                labels::B_SPLINE_SURFACE_TYPE,
                2,
                who,
                !internal,
                tol,
                UNDEF_DOUBLE,
            )?;
            if !internal {
                row = validity::degenerate_control_row(s, tol)?;
                valid &= report.class_value(
                    labels::B_SPLINE_SURFACE_TYPE,
                    3,
                    who,
                    !row,
                    tol,
                    UNDEF_DOUBLE,
                )?;
            }
            // Source index4 is warning-only, not an assertion. Still execute
            // the actual query; do not convert its result into a failed rule.
            let _repeated_end_warning = validity::repeated_end_points(s, ZONE_TOL_3D)?;
        }
    }
    if !point && !curve && !internal && !row {
        let domain = s.natural_uv_domain();
        // SmSurface.h1498: third argument is INPUT tolerance. It does not
        // overwrite dTol. U is passed as closed-column, V as closed-row.
        let closed_u = s
            .is_closed(k, &domain, SurfParamType::U, Some(tol), false)?
            .0;
        let closed_v = s
            .is_closed(k, &domain, SurfParamType::V, Some(tol), false)?
            .0;
        let net = validity::bspline_base(s)?.control_point_net()?;
        let mut grid = PointGrid::new(
            net.points,
            net.v_count,
            tol * 1000.,
            closed_u,
            closed_v,
            None,
        )?;
        let monotonic = grid.monotonic()?;
        valid &= report.class_predicate(
            labels::B_SPLINE_SURFACE_TYPE,
            5,
            who,
            monotonic.column && monotonic.row,
        )?;
    }
    Ok(valid)
}

fn base_assertions(k: &Kernel, b: &Brep, f: FaceId, report: &mut Reporter<'_>) -> Result<bool> {
    let s = surface(b, f)?;
    let who = object(b, f)?;
    // Geometry enums currently have no attribute container, so AObject's typed
    // empty-container predicate is intrinsic; geometry attributes remain an SDK
    // ownership capability to migrate, not a fabricated notification-key store.
    let mut valid = report.class_predicate(labels::A_OBJECT_TYPE, 0, who, true)?;
    let owner_ok = s.owner().is_none_or(|owner| {
        [
            labels::FACE_TYPE,
            labels::OFFSET_SURFACE_TYPE,
            labels::CRV_ON_SURF_TYPE,
            labels::SRF_IN_VOLUME_TYPE,
        ]
        .contains(&owner.type_code)
    });
    valid &= report.class_predicate(labels::SURFACE_TYPE, 0, who, owner_ok)?;
    valid &= report.class_value(
        labels::SURFACE_TYPE,
        1,
        who,
        s.flat_corners(k, ANG_TOL_DEG, false)?.flags == 0,
        ANG_TOL_DEG,
        UNDEF_DOUBLE,
    )?;
    valid &= report.class_value(
        labels::SURFACE_TYPE,
        2,
        who,
        !s.is_degenerate_point(k, EFF_ZERO_DEG, None)?,
        EFF_ZERO_DEG,
        UNDEF_DOUBLE,
    )?;
    valid &= report.class_value(
        labels::SURFACE_TYPE,
        3,
        who,
        !s.is_degenerate_curve(k, EFF_ZERO_DEG, None, ZONE_TOL_3D / 10.)?,
        EFF_ZERO_DEG,
        UNDEF_DOUBLE,
    )?;
    let mut u = None;
    let mut u_values = Vec::new();
    ignore_native_status(s.calculate_continuities_into(
        k,
        SurfParamType::U,
        &mut u,
        &mut u_values,
    ))?;
    let mut v = None;
    let mut v_values = Vec::new();
    ignore_native_status(s.calculate_continuities_into(
        k,
        SurfParamType::V,
        &mut v,
        &mut v_values,
    ))?;
    let (Some(u), Some(v)) = (u, v) else {
        // UNPORTED(fork.topology_validation): uninitialized native failure output.
        return Err(SolidError::unported(
            "Surface continuity failed before output initialization",
            "fork.topology_validation",
        ));
    };
    valid &= report.class_predicate(
        labels::SURFACE_TYPE,
        4,
        who,
        (u as i32) >= (ContinuityType::G1 as i32) && (v as i32) >= (ContinuityType::G1 as i32),
    )?;
    let face_tol = s
        .face()?
        .map(|owner| crate::topology::surface_cache::owning_face_tolerance(b, owner))
        .transpose()?;
    let poles = if let Some(tol) = face_tol {
        s.get_singularities_of_face(k, tol, false, false)?
    } else {
        s.get_singularities(k, false, false)?
    };
    valid &= report.class_predicate(
        labels::SURFACE_TYPE,
        5,
        who,
        poles.approx_singular.wrapping_sub(poles.singular) == 0,
    )?;
    valid &= report.class_predicate(
        labels::SURFACE_TYPE,
        6,
        who,
        opposite_boundaries(k, s, face_tol)?,
    )?;
    Ok(valid)
}

fn opposite_boundaries(k: &Kernel, s: &Surface, face_tol: Option<f64>) -> Result<bool> {
    let approx = face_tol.map_or_else(
        || tol::approx_tol_3d(k, SmallTopoType::NormalSize),
        |tol| tol / 2.,
    );
    for direction in [SurfParamType::U, SurfParamType::V] {
        if s.is_closed(k, &s.natural_uv_domain(), direction, None, false)?
            .0
        {
            continue;
        }
        let (first, second) = s
            .create_iso_boundaries(k, direction, approx)?
            .ok_or_else(missing)?;
        let first = Curve::from(first);
        let second = Curve::from(second);
        if first.is_degenerate(k, EFF_ZERO, None)? || second.is_degenerate(k, EFF_ZERO, None)? {
            continue;
        }
        let zone = tol::zone_tol_3d(k, SmallTopoType::NormalSize);
        let checked = first.global_coincidence_checker::<EntityRef>(
            k,
            &first.natural_interval(),
            &second,
            &second.natural_interval(),
            tol::xsect_tol_3d_from_zones(k, zone, zone),
            true,
            [None, None],
        );
        let checked = match checked {
            Ok(value) => value,
            Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => {
                // UNPORTED(fork.topology_validation): inaccessible failed output state.
                return Err(SolidError::unported(
                    "Surface AssertValid boundary coincidence failed outputs",
                    "fork.topology_validation",
                ));
            }
            Err(error) => return Err(error),
        };
        if checked
            .solutions
            .iter()
            .any(|s| s.solution_type == SolutionType::RangeOfValues)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn plane_assertions(k: &Kernel, b: &Brep, f: FaceId, report: &mut Reporter<'_>) -> Result<bool> {
    let SurfaceKind::Plane(plane) = surface(b, f)?.kind() else {
        return Err(missing());
    };
    let who = object(b, f)?;
    let position = plane.position();
    let x = position.x_axis();
    let y = position.y_axis();
    let axis = Object::Placement(f);
    let mut valid = report.class_value(
        labels::AXIS2_PLACEMENT_TYPE,
        0,
        axis,
        x.dot(y).abs() < EFF_ZERO,
        EFF_ZERO,
        x.dot(y).abs(),
    )?;
    valid &= report.class_value(
        labels::AXIS2_PLACEMENT_TYPE,
        1,
        axis,
        (x.dot(x) - 1.).abs() < EFF_ZERO,
        EFF_ZERO,
        (x.dot(x) - 1.).abs(),
    )?;
    valid &= report.class_value(
        labels::AXIS2_PLACEMENT_TYPE,
        2,
        axis,
        (y.dot(y) - 1.).abs() < EFF_ZERO,
        EFF_ZERO,
        y.dot(y) - 1.,
    )?;
    let analytic = plane.step_uv_domain();
    valid &= report.class_value(
        labels::PLANE_TYPE,
        0,
        who,
        analytic.x_length() > EFF_ZERO,
        EFF_ZERO,
        analytic.x_length(),
    )?;
    valid &= report.class_value(
        labels::PLANE_TYPE,
        1,
        who,
        analytic.y_length() > EFF_ZERO,
        EFF_ZERO,
        analytic.y_length(),
    )?;
    if plane.base().gw_nurb().is_some() {
        let natural = plane.base().natural_uv_domain();
        valid &= report.class_value(
            labels::PLANE_TYPE,
            2,
            who,
            natural.are_equal(&analytic, EFF_ZERO),
            EFF_ZERO,
            UNDEF_DOUBLE,
        )?;
        // Exact evaluation clone preserves knots/weights without Copy's fixup.
        let bspline = Surface::from(plane.base().clone_exact());
        for uv in [natural.min(), natural.max()] {
            let a = surface(b, f)?.evaluate_point(k, uv)?;
            let n = bspline.evaluate_point(k, uv)?;
            let tol = EFF_ZERO * (1. + a.max_dimension());
            valid &= report.class_value(
                labels::PLANE_TYPE,
                5,
                who,
                a.closer_than(tol, n),
                tol,
                a.distance(n),
            )?;
        }
    }
    let scale = plane.uv_scale();
    valid &= report.class_value(
        labels::PLANE_TYPE,
        3,
        who,
        scale.x > EFF_ZERO,
        EFF_ZERO,
        scale.x,
    )?;
    valid &= report.class_value(
        labels::PLANE_TYPE,
        4,
        who,
        scale.y > EFF_ZERO,
        EFF_ZERO,
        scale.y,
    )?;
    Ok(valid)
}
