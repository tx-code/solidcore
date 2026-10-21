// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Portions of finite_lerp(): Copyright (c) Microsoft Corporation.
// SPDX-License-Identifier: Apache-2.0 AND (Apache-2.0 WITH LLVM-exception)
//! Exact native-knot partitions for the existing nested face integrator.
//!
//! Upstream: maintained `source/SMLib/src/SmFaceIntegralSpans.h`, SHA256
//! `9e98a31aa452494f62ccb2aa26da75de4e37ac4cb9281a72dc09b2499454fc5d`.
//! | Rust | Upstream |
//! |------|----------|
//! | `Span`, `Partition::make` | `Span`, `MakePartition` |
//! | `span_accuracy`, `INITIAL_STEP_DIVISORS` | `SpanAccuracy`, `initialStepDivisors` |
//! Deviations: safe vector growth and checked access replace the native in-place
//! knot array. Input definitions and positive spans are never tolerance-merged.
//! Numerical stopping controls are measurements, not continuous error bounds.

use crate::curve::curve::{Curve, CurveKind};
use crate::geom::extent1d::Extent1d;
use crate::{Result, SolidError};

const OP: &str = "mass::integrator::properties::spans";

pub(crate) const INITIAL_STEP_DIVISORS: [f64; 5] = [2., 8., 64., 512., 4096.];

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Span {
    pub min: f64,
    pub max: f64,
}

impl Span {
    pub fn parameter(self, unit_parameter: f64) -> f64 {
        finite_lerp(self.min, self.max, unit_parameter)
    }
    pub fn length(self) -> f64 {
        self.max - self.min
    }
    pub fn evaluate_from_left(self, native_parameter: f64) -> bool {
        // Native TRUE selects the piece beginning at an interior knot. At the
        // current span's upper end select the piece ending there, also when an
        // interior RK evaluation rounds to that end.
        native_parameter < self.max
    }
}

// MSVC 14.50.35717 include/cmath::_Common_lerp/_Linear_for_lerp finite
// runtime path (1562--1627). Split-span endpoints are finite and the evaluator
// checks unit_parameter in [0,1]. The source explicitly uses std::fma in the
// same-sign branch only. License: LICENSES/Microsoft-STL.txt and ATTRIBUTION.md.
fn finite_lerp(a: f64, b: f64, t: f64) -> f64 {
    if (a <= 0. && b >= 0.) || (a >= 0. && b <= 0.) {
        return t * b + (1. - t) * a;
    }
    if t == 1. {
        return b;
    }
    let candidate = t.mul_add(b - a, a);
    if (t > 1.) == (b > a) {
        if b > candidate {
            return b;
        }
    } else if candidate > b {
        return b;
    }
    candidate
}

#[derive(Debug, Default)]
pub(crate) struct Partition {
    boundaries: Vec<f64>,
}

impl Partition {
    pub fn make(curve: &Curve, interval: &Extent1d) -> Result<Self> {
        let min = interval.min();
        let max = interval.max();
        // C++ GetType, not IsKindOf: an analytic/registered derived evaluator
        // retains its original path and does not even request its knot vector.
        let CurveKind::BSpline(curve) = curve.kind() else {
            return Ok(Self::default());
        };
        if !min.is_finite() || !max.is_finite() || max <= min || !(max - min).is_finite() {
            return Ok(Self::default());
        }
        // GetKnotsAll and GetDegree of a concrete BSpline both require its
        // actual NURB; no virtual analytic representation is synthesized here.
        let native = curve.gw_nurb().ok_or_else(|| SolidError::sm_err(OP))?;
        Self::from_knots(native.knots(), native.degree(), min, max)
    }

    fn from_knots(knots: &[f64], degree: usize, min: f64, max: f64) -> Result<Self> {
        if degree == 0 || degree > 32 || knots.len() < 2 * (degree + 1) {
            return Ok(Self::default());
        }
        let mut previous = None;
        for &knot in knots {
            if !knot.is_finite() || previous.is_some_and(|p| knot < p) {
                return Err(SolidError::invalid_input(
                    OP,
                    "nonfinite or decreasing knots",
                ));
            }
            previous = Some(knot);
        }
        let first = *knots.first().ok_or_else(|| SolidError::sm_err(OP))?;
        let last = *knots.last().ok_or_else(|| SolidError::sm_err(OP))?;
        let clamped = min >= first
            && max <= last
            && knots.iter().take(degree + 1).all(|&knot| knot == first)
            && knots
                .iter()
                .rev()
                .take(degree + 1)
                .all(|&knot| knot == last);
        if !clamped {
            return Ok(Self::default());
        }
        let mut boundaries = vec![min];
        let mut previous = min;
        for &knot in knots {
            if knot > previous && knot < max {
                boundaries.push(knot);
                previous = knot;
            }
        }
        boundaries.push(max);
        Ok(Self { boundaries })
    }

    pub fn is_split(&self) -> bool {
        self.boundaries.len() > 2
    }
    pub fn size(&self) -> usize {
        if self.is_split() {
            self.boundaries.len() - 1
        } else {
            1
        }
    }
    pub fn span(&self, index: usize, interval: &Extent1d) -> Result<Span> {
        if !self.is_split() {
            return Ok(Span {
                min: interval.min(),
                max: interval.max(),
            });
        }
        let min = self
            .boundaries
            .get(index)
            .copied()
            .ok_or_else(|| SolidError::sm_err(OP))?;
        let max = index
            .checked_add(1)
            .and_then(|i| self.boundaries.get(i))
            .copied()
            .ok_or_else(|| SolidError::sm_err(OP))?;
        Ok(Span { min, max })
    }
}

pub(crate) fn span_accuracy(curve_accuracy: f64, spans: usize) -> f64 {
    if spans <= 1 {
        return curve_accuracy;
    }
    let fraction = curve_accuracy / spans as f64;
    // std::nextafter(fraction, 0), including zero/NaN/infinity, without using
    // f64::next_down (which postdates the crate's Rust 1.85 minimum).
    if fraction == 0. || fraction.is_nan() {
        fraction
    } else {
        f64::from_bits(fraction.to_bits() - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::line::Line;
    use crate::math::Vec3;
    use crate::nurbs::b_spline_curve::{BSplineCurve, BSplineCurveForm, CanonicalData, KnotType};

    fn corner(start: f64, knot: f64, end: f64) -> Curve {
        BSplineCurve::create_canonical(&CanonicalData {
            dim: 2,
            degree: 1,
            control_points: &[Vec3::ZERO, Vec3::X, Vec3::new(1., 1., 0.)],
            form: BSplineCurveForm::Unspecified,
            knot_multiplicities: &[2, 1, 2],
            knots: &[start, knot, end],
            knot_type: KnotType::Unspecified,
            weights: None,
            trim: None,
        })
        .unwrap()
        .into()
    }

    #[test]
    fn positive_actual_spans_keep_native_side_and_jacobian_including_tiny_width() {
        // Independent piecewise-linear derivative: each whole piece moves by
        // X then Y, even when its native width is far below SM_EFF_ZERO.
        for (start, knot, end) in [(0., 1., 2.), (1e6, 1e6 + 0.25, 1e6 + 2.), (0., 1e-10, 2.)] {
            let curve = corner(start, knot, end);
            let interval = curve.natural_interval();
            let plan = Partition::make(&curve, &interval).unwrap();
            assert!(plan.is_split());
            assert_eq!(plan.size(), 2);
            for i in 0..2 {
                let span = plan.span(i, &interval).unwrap();
                assert_eq!(span.parameter(0.), span.min);
                assert_eq!(span.parameter(1.), span.max);
                let parameter = if i == 0 { span.max } else { span.min };
                let values = curve
                    .evaluate(parameter, 1, span.evaluate_from_left(parameter), false)
                    .unwrap();
                let expected = if i == 0 { Vec3::X } else { Vec3::Y };
                assert!((values[1] * span.length() - expected).length() < 1e-12);
            }
            let min = Span {
                min: start,
                max: knot,
            }
            .parameter(0.5);
            let clipped = Partition::make(&curve, &Extent1d::from_min_max(min, end)).unwrap();
            assert_eq!(clipped.size(), 2);
            assert_eq!(clipped.span(0, &interval).unwrap().min, min);
        }
    }

    #[test]
    fn analytic_kinds_retain_the_unsplit_path_without_requesting_absent_nurb() {
        let curve: Curve = Line::from_points(Vec3::ZERO, Vec3::X, 2).into();
        assert!(!Partition::make(&curve, &curve.natural_interval())
            .unwrap()
            .is_split());
        assert!(!Partition::from_knots(&[0., 0., 1., 2., 3., 3.], 2, 0., 3.)
            .unwrap()
            .is_split());
        assert!(Partition::from_knots(&[0., 0., f64::NAN, 1., 1.], 1, 0., 1.).is_err());
        assert!(Partition::from_knots(&[0., 0., 0.75, 0.5, 1., 1.], 1, 0., 1.).is_err());
    }

    #[test]
    fn accuracy_is_allocated_per_span_with_a_step_towards_zero() {
        for count in [1, 2, 3, 17, 4096] {
            let budget = 1e-5;
            let actual = span_accuracy(budget, count);
            assert!(actual > 0. && actual * count as f64 <= budget);
            let quotient = budget / count as f64;
            assert_eq!(
                actual.to_bits(),
                if count == 1 {
                    quotient.to_bits()
                } else {
                    quotient.to_bits() - 1
                }
            );
        }
        assert_eq!(span_accuracy(f64::from_bits(1), 2), 0.);
        assert_eq!(INITIAL_STEP_DIVISORS, [2., 8., 64., 512., 4096.]);
    }

    #[test]
    fn rational_c0_current_piece_derivatives_follow_native_parameter_reversal() {
        use crate::kernel::Kernel;
        let k = Kernel::default();
        let controls = [
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(1., 1., 0.),
            Vec3::new(2., 1., 0.),
            Vec3::new(2., 2., 0.),
        ];
        let mut curve: Curve = BSplineCurve::create_canonical(&CanonicalData {
            dim: 2,
            degree: 2,
            control_points: &controls,
            form: BSplineCurveForm::Unspecified,
            knot_multiplicities: &[3, 2, 3],
            knots: &[2., 3., 5.],
            knot_type: KnotType::Unspecified,
            weights: Some(&[1., 0.5, 1., 0.75, 1.]),
            trim: None,
        })
        .unwrap()
        .into();
        for reverse in [false, true] {
            let interval = curve.natural_interval();
            if reverse {
                curve.reverse_parameterization(&k, &interval).unwrap();
            }
            let interval = curve.natural_interval();
            let plan = Partition::make(&curve, &interval).unwrap();
            assert_eq!(plan.size(), 2);
            assert_eq!(
                plan.span(0, &interval).unwrap().max,
                if reverse { 4. } else { 3. }
            );
            for i in 0..2 {
                let span = plan.span(i, &interval).unwrap();
                let parameter = if i == 0 { span.max } else { span.min };
                let values = curve
                    .evaluate(parameter, 1, span.evaluate_from_left(parameter), false)
                    .unwrap();
                // Rational endpoint derivative on each whole quadratic piece:
                // 2*w_adjacent/w_endpoint*(P_endpoint-P_adjacent).
                let expected = match (reverse, i) {
                    (false, 0) => Vec3::Y,
                    (false, _) => 1.5 * Vec3::X,
                    (true, 0) => -1.5 * Vec3::X,
                    (true, _) => -Vec3::Y,
                };
                assert!((values[1] * span.length() - expected).length() < 1e-12);
            }
        }
    }
}
