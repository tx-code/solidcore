// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded Bezier regularity certificate used only for early successful checks.
//!
//! Upstream: maintained `d8ed45a`, `SmBSplineCurveValidation.cpp` (30--333).
//! | Rust | Upstream |
//! |------|----------|
//! | [`prove`] | `proveBezierRegularity` |
//! Deviations: software IEEE binary64 arithmetic replaces the native SSE/MXCSR
//! guard, so interval bounds do not depend on the caller's rounding/FTZ/DAZ state.
//! The operation order and outward one-ULP expansion are unchanged. This proves
//! mathematical bounds; it does not emulate native floating-environment/cache
//! behavior in nondefault modes. Inconclusive bounds always retain the GPA path.
//! Subdivision depth is exactly three (at most eight leaves), never input-driven.

use crate::nurbs::{
    crv_basic::Curve,
    math::{NOW, NOZ},
};
use softfloat::F64 as S;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Proof {
    pub speed: bool,
    pub radius: bool,
}
#[derive(Clone, Copy, Debug, Default)]
struct I {
    lo: S,
    hi: S,
}
fn s(x: f64) -> S {
    S::from_bits(x.to_bits())
}
fn finite(x: S) -> bool {
    x.to_bits() & 0x7ff0_0000_0000_0000 != 0x7ff0_0000_0000_0000
}
fn lower(x: S) -> S {
    let bits = x.to_bits();
    if bits == 0xfff0_0000_0000_0000 || !finite(x) && bits & 0x000f_ffff_ffff_ffff != 0 {
        return x;
    }
    if bits.trailing_zeros() >= 63 {
        return S::from_bits(0x8000_0000_0000_0001);
    }
    S::from_bits(if bits >> 63 != 0 {
        bits.wrapping_add(1)
    } else {
        bits.wrapping_sub(1)
    })
}
fn upper(x: S) -> S {
    -lower(-x)
}
fn min(a: S, b: S) -> S {
    if b < a {
        b
    } else {
        a
    }
}
fn max(a: S, b: S) -> S {
    if a < b {
        b
    } else {
        a
    }
}
fn abs(x: S) -> S {
    S::from_bits(x.to_bits() & 0x7fff_ffff_ffff_ffff)
}
impl I {
    fn point(x: f64) -> Self {
        Self { lo: s(x), hi: s(x) }
    }
    fn bad() -> Self {
        Self {
            lo: s(1.),
            hi: s(-1.),
        }
    }
    fn finite(self) -> bool {
        finite(self.lo) && finite(self.hi) && self.lo <= self.hi
    }
    fn hull(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::bad();
        }
        Self {
            lo: min(self.lo, b.lo),
            hi: max(self.hi, b.hi),
        }
    }
}
impl core::ops::Add for I {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::bad();
        }
        Self {
            lo: lower(self.lo + b.lo),
            hi: upper(self.hi + b.hi),
        }
    }
}
impl core::ops::Sub for I {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::bad();
        }
        Self {
            lo: lower(self.lo - b.hi),
            hi: upper(self.hi - b.lo),
        }
    }
}
impl core::ops::Mul for I {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::bad();
        }
        let values = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        let mut low = values.first().copied().unwrap_or(s(0.));
        let mut high = low;
        for value in values.iter().skip(1) {
            low = min(low, *value);
            high = max(high, *value);
        }
        Self {
            lo: lower(low),
            hi: upper(high),
        }
    }
}
impl core::ops::Div for I {
    type Output = Self;
    fn div(self, b: Self) -> Self {
        if !b.finite() || b.lo <= s(0.) {
            return Self::bad();
        }
        self * Self {
            lo: lower(s(1.) / b.hi),
            hi: upper(s(1.) / b.lo),
        }
    }
}
type Pole = [I; 4];
type Bezier = [Pole; 4];
fn at<T: Copy>(a: &[T], i: usize) -> Option<T> {
    a.get(i).copied()
}
fn coordinate(a: &Bezier, i: usize, axis: usize) -> Option<I> {
    at(a.get(i)?, axis)
}
fn put(a: &mut Bezier, i: usize, axis: usize, value: I) -> Option<()> {
    *a.get_mut(i)?.get_mut(axis)? = value;
    Some(())
}

fn bound(poles: &Bezier, degree: usize, dim: usize, length: I, tol: S) -> Option<Proof> {
    let mut translated = *poles;
    for axis in 0..dim {
        let origin = coordinate(poles, 0, axis)?.lo / coordinate(poles, 0, 3)?.lo;
        if !finite(origin) {
            return None;
        }
        for i in 0..=degree {
            put(
                &mut translated,
                i,
                axis,
                coordinate(poles, i, axis)?
                    - I {
                        lo: origin,
                        hi: origin,
                    } * coordinate(poles, i, 3)?,
            )?;
        }
    }
    let mut pos = at(&translated, 0)?;
    let mut first = [I::default(); 4];
    let mut second = first;
    let mut differences = [[I::default(); 4]; 4];
    for axis in 0..4 {
        for i in 1..=degree {
            *pos.get_mut(axis)? = at(&pos, axis)?.hull(coordinate(&translated, i, axis)?);
        }
        for i in 0..degree {
            let value = (coordinate(&translated, i + 1, axis)? - coordinate(&translated, i, axis)?)
                * I::point(degree as f64)
                / length;
            put(&mut differences, i, axis, value)?;
            *first.get_mut(axis)? = if i == 0 {
                value
            } else {
                at(&first, axis)?.hull(value)
            };
        }
        for i in 0..degree - 1 {
            let value = (coordinate(&differences, i + 1, axis)?
                - coordinate(&differences, i, axis)?)
                * I::point((degree - 1) as f64)
                / length;
            *second.get_mut(axis)? = if i == 0 {
                value
            } else {
                at(&second, axis)?.hull(value)
            };
        }
        if !at(&pos, axis)?.finite() || !at(&first, axis)?.finite() || !at(&second, axis)?.finite()
        {
            return None;
        }
    }
    if at(&pos, 3)?.lo <= s(0.) {
        return None;
    }
    let mut speed = s(0.);
    let mut acceleration = s(0.);
    for axis in 0..dim {
        let position = at(&pos, axis)? / at(&pos, 3)?;
        let velocity = (at(&first, axis)? - at(&first, 3)? * position) / at(&pos, 3)?;
        let a = (at(&second, axis)?
            - I::point(2.) * at(&first, 3)? * velocity
            - at(&second, 3)? * position)
            / at(&pos, 3)?;
        if !position.finite() || !velocity.finite() || !a.finite() {
            return None;
        }
        speed = max(speed, max(velocity.lo, -velocity.hi));
        acceleration = upper(acceleration + max(abs(a.lo), abs(a.hi)));
    }
    let proven_speed = speed >= tol;
    Some(Proof {
        speed: proven_speed,
        radius: proven_speed
            && finite(acceleration)
            && lower(speed * speed) >= upper(tol * acceleration),
    })
}
fn subdivide(
    poles: &Bezier,
    degree: usize,
    dim: usize,
    length: I,
    tol: S,
    depth: usize,
) -> Option<Proof> {
    let whole = bound(poles, degree, dim, length, tol).unwrap_or_default();
    if (whole.speed && whole.radius) || depth == 0 {
        return Some(whole);
    }
    let mut work = *poles;
    let mut left = [[I::default(); 4]; 4];
    let mut right = left;
    *left.get_mut(0)? = at(&work, 0)?;
    *right.get_mut(degree)? = at(&work, degree)?;
    for level in 1..=degree {
        for i in 0..=degree - level {
            for axis in 0..4 {
                let value =
                    (coordinate(&work, i, axis)? + coordinate(&work, i + 1, axis)?) * I::point(0.5);
                put(&mut work, i, axis, value)?;
            }
        }
        *left.get_mut(level)? = at(&work, 0)?;
        *right.get_mut(degree - level)? = at(&work, degree - level)?;
    }
    let half = length * I::point(0.5);
    let a = subdivide(&left, degree, dim, half, tol, depth - 1)?;
    let b = subdivide(&right, degree, dim, half, tol, depth - 1)?;
    Some(Proof {
        speed: whole.speed || (a.speed && b.speed),
        radius: whole.radius || (a.radius && b.radius),
    })
}

/// Certify already-separated Bezier spans, never extracting or altering knots.
pub(crate) fn prove(curve: &Curve, dimension: u64, tolerance: f64) -> Proof {
    prove_inner(curve, dimension, tolerance).unwrap_or_default()
}
fn prove_inner(c: &Curve, dimension: u64, tolerance: f64) -> Option<Proof> {
    let dim = usize::try_from(dimension).ok()?;
    let degree = c.degree();
    let last = c.cpts().len().checked_sub(1)?;
    if !(2..=3).contains(&dim)
        || !finite(s(tolerance))
        || s(tolerance) <= s(0.)
        || !(1..=3).contains(&degree)
        || last < degree
        || last > 16 * degree
        || last % degree != 0
        || c.knots().len() != last + degree + 2
    {
        return None;
    }
    let knots = c.knots();
    let end = knots.len().checked_sub(1)?;
    for i in 0..=degree {
        if at(knots, i)? != at(knots, 0)? || at(knots, end - i)? != at(knots, end)? {
            return None;
        }
    }
    let polynomial = c.cpts().first()?.w == NOW;
    let mut result = Proof {
        speed: true,
        radius: true,
    };
    for start in (0..last).step_by(degree) {
        let finish = at(knots, start + degree + 1)?;
        let length = I::point(finish) - I::point(at(knots, start + degree)?);
        if !length.finite() || length.lo <= s(0.) {
            return None;
        }
        for repeat in 1..=degree {
            if at(knots, start + degree + repeat)? != finish {
                return None;
            }
        }
        let mut poles = [[I::default(); 4]; 4];
        for i in 0..=degree {
            let p = c.cpts().get(start + i)?;
            if (p.w == NOW) != polynomial
                || (!polynomial && s(p.w) <= s(0.))
                || (dim == 2 && p.z != 0. && p.z != NOZ)
                || (dim == 3 && p.z == NOZ)
            {
                return None;
            }
            let pole = [
                I::point(p.x),
                I::point(p.y),
                I::point(if dim == 2 { 0. } else { p.z }),
                I::point(if polynomial { 1. } else { p.w }),
            ];
            if pole.iter().any(|i| !i.finite()) {
                return None;
            }
            *poles.get_mut(i)? = pole;
        }
        let span = subdivide(&poles, degree, dim, length, s(tolerance), 3)?;
        result.speed &= span.speed;
        result.radius &= span.radius;
        if !result.speed && !result.radius {
            return None;
        }
    }
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::{math::CPoint, nurbs_crv::allocate_nurb_curve};
    fn curve(points: &[(f64, f64)], degree: usize) -> Curve {
        let mut c = allocate_nurb_curve(points.len() - 1, degree, points.len() + degree).unwrap();
        for (p, (x, y)) in c.cpts_mut().iter_mut().zip(points) {
            *p = CPoint::new(*x, *y, 0., NOW);
        }
        let count = c.knots().len();
        for (i, k) in c.knots_mut().iter_mut().enumerate() {
            *k = if i <= degree {
                0.
            } else if i + degree >= count - 1 {
                1.
            } else {
                0.5
            };
        }
        c
    }
    #[test]
    fn software_arithmetic_matches_ieee_nearest_on_boundary_and_normal_inputs() {
        let values = [
            0.,
            -0.,
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            -f64::MIN_POSITIVE,
            1.,
            -1.,
            0.1,
            1e100,
            f64::MAX,
        ];
        for a in values {
            for b in values {
                for (computed, expected) in [
                    (s(a) + s(b), a + b),
                    (s(a) - s(b), a - b),
                    (s(a) * s(b), a * b),
                    (s(a) / s(b), a / b),
                ] {
                    let got = f64::from_bits(computed.to_bits());
                    if expected.is_nan() {
                        assert!(got.is_nan());
                    } else {
                        assert_eq!(got.to_bits(), expected.to_bits(), "{a} {b}");
                    }
                }
            }
        }
        for v in [
            0.,
            -0.,
            f64::from_bits(1),
            -f64::from_bits(1),
            f64::MIN_POSITIVE,
            f64::MAX,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ] {
            assert_eq!(lower(s(v)).to_bits(), v.next_down().to_bits());
            assert_eq!(upper(s(v)).to_bits(), v.next_up().to_bits());
        }
    }
    #[test]
    fn analytic_line_and_parabola_certificates_use_original_parameter() {
        let line = curve(&[(0., 0.), (1., 0.)], 1);
        assert_eq!(
            prove(&line, 3, 0.5),
            Proof {
                speed: true,
                radius: true
            }
        );
        let parabola = curve(&[(0., 0.), (0.5, 0.), (1., 1.)], 2);
        // C=(t,t²): speed>=1, minimum radius=1/2 exactly.
        assert_eq!(
            prove(&parabola, 3, 0.25),
            Proof {
                speed: true,
                radius: true
            }
        );
        assert!(!prove(&parabola, 3, 0.6).radius);
        assert!(!prove(&parabola, 3, 3.).speed);
    }
    #[test]
    fn proof_never_normalizes_weights_z_or_unsupported_spans() {
        let mut c = curve(&[(0., 0.), (1., 0.)], 1);
        c.cpts_mut()[0].w = -1.;
        let before = c.clone();
        assert_eq!(prove(&c, 3, 0.1), Proof::default());
        assert_eq!(c.cpts(), before.cpts());
        c.cpts_mut()[0].w = NOW;
        c.cpts_mut()[0].z = 3.;
        assert_eq!(prove(&c, 2, 0.1), Proof::default());
        assert_eq!(prove(&c, 3, 0.), Proof::default());
    }
}
