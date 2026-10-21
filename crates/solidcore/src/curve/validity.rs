// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Shared stored-representation queries and validity operations for curve families.
//! Upstream: `SmBSplineCurve.cpp::PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` at
//! `d8ed45a`, including the virtual lazy NURB creation through analytic kinds.
//! Returns the native front/back failure bits; they remain set after correction.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`Curve::check_reverse_direction`] | `SmBSplineCurve::PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` |
//! | [`Curve::has_internal_pole`] | `SmBSplineCurve.cpp`3701--3779; `SmCurve.h`744 |
//! | [`Curve::assert_no_periodic_jump`] | `SmBSplineCurve.cpp`7623--7691 |
//!
//! Deviations: the native warning result is a bit mask; checked failures are Rust
//! errors. The owned original curve is mutated, not a temporary approximation.
use super::curve::{Curve, CurveKind};
use crate::error::{Result, SolidError};
use crate::kernel::Kernel;
use crate::nurbs::b_spline_curve::BSplineCurve;

/// Stored B-spline base, without making a new representation or running Copy.
pub(crate) fn bspline_base(curve: &Curve) -> Result<&BSplineCurve> {
    match curve.kind() {
        CurveKind::BSpline(c) => Ok(c),
        CurveKind::Line(c) => Ok(c.base()),
        CurveKind::Ellipse(c) => Ok(c.base()),
        CurveKind::Hyperbola(c) => Ok(c.base()),
        CurveKind::Parabola(c) => Ok(c.base()),
        _ => Err(SolidError::unported(
            "non-B-spline curve validity base",
            "fork.topology_validation",
        )),
    }
}

/// Materialize the actual stored representation once. Existing NURBs (including
/// conflicts with analytic data) are retained for diagnostics, never regenerated.
pub(crate) fn materialize_bspline(k: &Kernel, curve: &mut Curve) -> Result<()> {
    if bspline_base(curve)?.gw_nurb().is_some() {
        return Ok(());
    }
    let owner = curve.owner();
    match curve.kind_mut() {
        CurveKind::BSpline(c) => c.make_nurb(),
        CurveKind::Line(c) => c.make_nurb_owned(owner),
        CurveKind::Ellipse(c) => c.make_nurb_owned(k, owner),
        CurveKind::Hyperbola(c) => c.make_nurb(owner),
        CurveKind::Parabola(c) => c.make_nurb(owner),
        _ => Err(SolidError::unported(
            "concrete curve validity materialization",
            "fork.topology_validation",
        )),
    }
}

impl Curve {
    /// Materialize this actual object's B-spline representation, preserving its
    /// analytic kind, owner, parameters and any already stored NURB. This is the
    /// explicit Rust owning counterpart of native lazy `GetOrCreate` queries.
    ///
    /// # Errors
    /// The concrete materializer's errors, or an unsupported non-B-spline family.
    pub fn materialize_stored_bspline(&mut self, k: &Kernel) -> Result<()> {
        materialize_bspline(k, self)
    }

    /// `SmBSplineCurve::AssertNoPeriodicJump` on a stored B-spline-derived UV
    /// curve. Two-control-point curves retain the native exemption. This only
    /// diagnoses jumps; it never unwraps UV coordinates or changes parameters.
    pub fn assert_no_periodic_jump(
        &mut self,
        k: &Kernel,
        closed_u: bool,
        closed_v: bool,
        domain: &crate::geom::extent2d::Extent2d,
    ) -> Result<bool> {
        bspline_base(self)?;
        if self.dim() != 2 || (!closed_u && !closed_v) {
            return Ok(true);
        }
        materialize_bspline(k, self)?;
        let polygon = bspline_base(self)?.polygon(&self.natural_interval())?;
        if polygon.len() == 2 {
            return Ok(true);
        }
        let u = 0.9 * domain.x_length();
        let v = 0.9 * domain.y_length();
        let mut valid = true;
        for pair in polygon.windows(2) {
            let a = pair
                .first()
                .ok_or_else(|| SolidError::null_pointer("periodic jump polygon"))?;
            let b = pair
                .get(1)
                .ok_or_else(|| SolidError::null_pointer("periodic jump polygon"))?;
            if closed_u && (b.x - a.x).abs() >= u {
                valid = false;
            }
            if closed_v && (b.y - a.y).abs() >= v {
                valid = false;
            }
        }
        Ok(valid)
    }
    /// Native `SmBSplineCurve::HasInternalPole`: any active degree-plus-one
    /// coincident control-point sequence, except a wholly degenerate curve.
    /// Missing stored NURB returns false; the query never materializes or repairs.
    pub fn has_internal_pole(&self, tol: f64) -> Result<bool> {
        if !matches!(
            self.kind(),
            CurveKind::BSpline(_)
                | CurveKind::Line(_)
                | CurveKind::Ellipse(_)
                | CurveKind::Hyperbola(_)
                | CurveKind::Parabola(_)
        ) {
            // SmCurve.h744 has a literal FALSE default for these receivers.
            return Ok(false);
        }
        let base = bspline_base(self)?;
        let Some(nurb) = base.gw_nurb() else {
            return Ok(false);
        };
        if self.is_degenerate(tol, None)? {
            return Ok(false);
        }
        let (knots, mults) = self.knots(None)?;
        let tolerance = if tol > 0. {
            tol
        } else {
            crate::geom::tol::ZONE_TOL_3D
        };
        let degree = nurb.degree();
        let mut knot_count = 0usize;
        for mult in mults.iter().take(knots.len().saturating_sub(1)) {
            knot_count = knot_count.checked_add(*mult).ok_or_else(|| {
                SolidError::invalid_input(
                    "curve::validity::has_internal_pole",
                    "knot count overflow",
                )
            })?;
            let first = knot_count
                .checked_sub(1)
                .and_then(|v| v.checked_sub(degree))
                .ok_or_else(|| {
                    SolidError::invalid_input(
                        "curve::validity::has_internal_pole",
                        "invalid control point index",
                    )
                })?;
            let points = nurb
                .cpts()
                .get(
                    first..=first.checked_add(degree).ok_or_else(|| {
                        SolidError::invalid_input(
                            "curve::validity::has_internal_pole",
                            "index overflow",
                        )
                    })?,
                )
                .ok_or_else(|| {
                    SolidError::invalid_input(
                        "curve::validity::has_internal_pole",
                        "short control polygon",
                    )
                })?;
            let p = points
                .first()
                .ok_or_else(|| SolidError::null_pointer("curve::validity::has_internal_pole"))?
                .to_euclid();
            if points
                .iter()
                .skip(1)
                .all(|v| p.closer_than(tolerance, v.to_euclid()))
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
    /// Check and correct control-polygon reversal as native
    /// `PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` does. A nonzero result means
    /// the check failed, even though the control points have been changed.
    /// Curves not derived from a B-spline use the native base's successful check.
    pub fn check_reverse_direction(&mut self, k: &Kernel) -> Result<u8> {
        if !matches!(
            self.kind(),
            CurveKind::BSpline(_)
                | CurveKind::Line(_)
                | CurveKind::Ellipse(_)
                | CurveKind::Hyperbola(_)
                | CurveKind::Parabola(_)
        ) {
            return Ok(0);
        }
        materialize_bspline(k, self)?;
        let base = match self.kind_mut() {
            CurveKind::BSpline(b) => b,
            CurveKind::Line(c) => c.base_mut(),
            CurveKind::Ellipse(c) => c.base_mut(),
            CurveKind::Hyperbola(c) => &mut c.base,
            CurveKind::Parabola(c) => c.base_mut(),
            _ => return Ok(0),
        };
        base.get_or_create_gw_nurb()?.end_reversal(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::extent2d::Extent2d;
    use crate::math::Vec3;
    use crate::nurbs::{
        crv_basic::Curve as Nurb,
        math::{CPoint, NOW, NOZ},
    };

    #[test]
    fn periodic_jump_retains_inclusive_boundary_and_two_point_exemption() {
        let k = Kernel::default();
        let domain = Extent2d::from_bounds(0., 0., 1., 1.);
        for (middle, expected) in [(0.9, false), (f64::from_bits(0.9_f64.to_bits() - 1), true)] {
            let nurb = Nurb::new(
                vec![
                    CPoint::new(0., 0., NOZ, NOW),
                    CPoint::new(middle, 0., NOZ, NOW),
                    CPoint::new(1., 0., NOZ, NOW),
                ],
                1,
                vec![0., 0., 0.5, 1., 1.],
            )
            .unwrap();
            let mut c = Curve::from(BSplineCurve::from_nurb(nurb, 2, true));
            assert_eq!(
                c.assert_no_periodic_jump(&k, true, false, &domain).unwrap(),
                expected
            );
        }
        let mut line = Curve::from(crate::curve::line::Line::from_points(
            Vec3::ZERO,
            Vec3::new(100., 100., 0.),
            2,
        ));
        assert!(line
            .assert_no_periodic_jump(&k, false, false, &domain)
            .unwrap());
        assert!(bspline_base(&line).unwrap().gw_nurb().is_none());
        assert!(line
            .assert_no_periodic_jump(&k, true, true, &domain)
            .unwrap());
        assert!(bspline_base(&line).unwrap().gw_nurb().is_some());
    }
}
