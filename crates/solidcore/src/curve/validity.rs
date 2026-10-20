// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Mutating reverse-direction validity check of B-spline-derived curves.
//! Upstream: `SmBSplineCurve.cpp::PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` at
//! `d8ed45a`, including the virtual lazy NURB creation through analytic kinds.
//! Returns the native front/back failure bits; they remain set after correction.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`Curve::check_reverse_direction`] | `SmBSplineCurve::PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` |
//!
//! Deviations: the native warning result is a bit mask; checked failures are Rust
//! errors. The owned original curve is mutated, not a temporary approximation.
use super::curve::{Curve, CurveKind};
use crate::error::Result;
use crate::kernel::Kernel;

impl Curve {
    /// Check and correct control-polygon reversal as native
    /// `PassesValidityCheck(SM_VC_REVERSE_DIRECTION)` does. A nonzero result means
    /// the check failed, even though the control points have been changed.
    /// Curves not derived from a B-spline use the native base's successful check.
    pub fn check_reverse_direction(&mut self, k: &Kernel) -> Result<u8> {
        let owner = self.owner();
        let base = match self.kind_mut() {
            CurveKind::BSpline(b) => b,
            CurveKind::Line(c) => {
                if c.base().gw_nurb().is_none() {
                    c.make_nurb_owned(owner)?;
                }
                c.base_mut()
            }
            CurveKind::Ellipse(c) => {
                if c.base().gw_nurb().is_none() {
                    c.make_nurb_owned(k, owner)?;
                }
                c.base_mut()
            }
            CurveKind::Hyperbola(c) => {
                if c.base.gw_nurb().is_none() {
                    c.make_nurb(owner)?;
                }
                &mut c.base
            }
            CurveKind::Parabola(c) => {
                if c.base().gw_nurb().is_none() {
                    c.make_nurb(owner)?;
                }
                c.base_mut()
            }
            _ => return Ok(0),
        };
        base.get_or_create_gw_nurb()?.end_reversal(true)
    }
}
