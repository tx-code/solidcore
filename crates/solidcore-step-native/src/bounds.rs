// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Outward software-binary64 enclosures for the affine import certificate.
//! Upstream: `source/STEP_IMPORT/src/StepImportInterval.h` and affine verification
//! in `StepImportNative.cpp` at `d8ed45a`. No sampled point set establishes this
//! certificate. Deviations: software nearest-even arithmetic replaces the native
//! floating environment; unavailable finite evidence propagates to an infinite
//! norm bound instead of throwing. Integer conversion and budget comparison use
//! integer bits, so neither depends on host rounding, FTZ or DAZ settings.
use softfloat::F64 as S;
use solidcore_step::planar::Rational;
use std::cmp::Ordering;

const EXPONENT: u64 = 0x7ff0_0000_0000_0000;
const FRACTION: u64 = 0x000f_ffff_ffff_ffff;
const SIGN: u64 = 0x8000_0000_0000_0000;

#[derive(Clone, Copy)]
pub(super) struct I {
    lo: S,
    hi: S,
}
fn s(x: f64) -> S {
    S::from_bits(x.to_bits())
}
fn finite(x: S) -> bool {
    x.to_bits() & EXPONENT != EXPONENT
}
fn down(x: S) -> S {
    let b = x.to_bits();
    if b == 0xfff0_0000_0000_0000
        || (b & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000 && b & 0x000f_ffff_ffff_ffff != 0)
    {
        return x;
    }
    if b.trailing_zeros() >= 63 {
        return S::from_bits(0x8000_0000_0000_0001);
    }
    S::from_bits(if b >> 63 != 0 {
        b.wrapping_add(1)
    } else {
        b.wrapping_sub(1)
    })
}
fn up(x: S) -> S {
    -down(-x)
}
fn min(a: S, b: S) -> S {
    if a < b {
        a
    } else {
        b
    }
}
fn max(a: S, b: S) -> S {
    if a > b {
        a
    } else {
        b
    }
}
fn abs(a: S) -> S {
    S::from_bits(a.to_bits() & 0x7fff_ffff_ffff_ffff)
}
impl I {
    // This sentinel cannot regain finite evidence through later arithmetic.
    // NaN alone is unsuitable: certificate accumulation uses f64::max, which
    // ignores NaN. norm_upper turns every unavailable interval into +infinity.
    fn unestablished() -> Self {
        Self {
            lo: s(f64::NEG_INFINITY),
            hi: s(f64::INFINITY),
        }
    }
    fn finite(self) -> bool {
        finite(self.lo) && finite(self.hi) && self.lo <= self.hi
    }
    fn enclose(lo: S, hi: S) -> Self {
        let result = Self { lo, hi };
        if result.finite() {
            result
        } else {
            Self::unestablished()
        }
    }
    pub fn point(x: f64) -> Self {
        Self::enclose(s(x), s(x))
    }
    /// Establish containment in an exact native binary64 parameter range.
    pub fn within_range(self, lo: f64, hi: f64) -> bool {
        let range = Self::enclose(s(lo), s(hi));
        self.finite() && range.finite() && self.lo >= range.lo && self.hi <= range.hi
    }
    fn integer(value: i64) -> Self {
        let magnitude = value.unsigned_abs();
        if magnitude == 0 {
            return Self::point(0.);
        }
        // Every i64 magnitude is finite binary64 and is at least one. Retain
        // its leading 53 bits and use discarded integer bits to choose adjacent
        // endpoints. No i64 as f64 host conversion is needed. The largest
        // right shift is 11, and a nonzero integer cannot underflow.
        let exponent = magnitude.ilog2();
        let (significand, inexact) = if exponent <= 52 {
            (magnitude << (52 - exponent), false)
        } else {
            let shift = exponent - 52;
            (magnitude >> shift, magnitude & ((1_u64 << shift) - 1) != 0)
        };
        let low = (u64::from(exponent + 1023) << 52) | (significand & FRACTION);
        let high = low + u64::from(inexact);
        if value < 0 {
            Self::enclose(S::from_bits(SIGN | high), S::from_bits(SIGN | low))
        } else {
            Self::enclose(S::from_bits(low), S::from_bits(high))
        }
    }
    pub fn rational(x: Rational) -> Self {
        if x.numerator() == 0 {
            return Self::point(0.);
        }
        // Rational guarantees a positive i64 denominator and excludes i64::MIN
        // numerators. Subsequent arithmetic uses software nearest-even binary64
        // and expands by one ULP, including gradual-underflow results.
        Self::integer(x.numerator()) / Self::integer(x.denominator())
    }
    pub fn sqrt(self) -> Self {
        if !self.finite() || self.hi < s(0.) {
            return Self::unestablished();
        }
        // SqrtNonnegative clips the enclosure's negative tail; a wholly
        // negative interval cannot establish a real square root.
        Self::enclose(
            max(s(0.), down(max(self.lo, s(0.)).sqrt())),
            up(self.hi.sqrt()),
        )
    }
    fn magnitude(self) -> S {
        max(abs(self.lo), abs(self.hi))
    }
}
impl std::ops::Add for I {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unestablished();
        }
        Self::enclose(down(self.lo + b.lo), up(self.hi + b.hi))
    }
}
impl std::ops::Sub for I {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unestablished();
        }
        Self::enclose(down(self.lo - b.hi), up(self.hi - b.lo))
    }
}
impl std::ops::Mul for I {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        if !self.finite() || !b.finite() {
            return Self::unestablished();
        }
        let v = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        let (lo, hi) = v
            .into_iter()
            .fold((s(f64::INFINITY), s(f64::NEG_INFINITY)), |(lo, hi), v| {
                (min(lo, v), max(hi, v))
            });
        Self::enclose(down(lo), up(hi))
    }
}
impl std::ops::Div for I {
    type Output = Self;
    fn div(self, b: Self) -> Self {
        if !self.finite() || !b.finite() || b.lo <= s(0.) && b.hi >= s(0.) {
            return Self::unestablished();
        }
        self * Self::enclose(down(s(1.) / b.hi), up(s(1.) / b.lo))
    }
}
pub(super) type V = [I; 3];
pub(super) fn exact(v: [Rational; 3]) -> V {
    v.map(I::rational)
}
pub(super) fn native(v: solidcore::math::Vec3) -> V {
    [I::point(v.x), I::point(v.y), I::point(v.z)]
}
pub(super) fn add(a: V, b: V) -> V {
    let [ax, ay, az] = a;
    let [bx, by, bz] = b;
    [ax + bx, ay + by, az + bz]
}
pub(super) fn sub(a: V, b: V) -> V {
    let [ax, ay, az] = a;
    let [bx, by, bz] = b;
    [ax - bx, ay - by, az - bz]
}
pub(super) fn times(a: V, b: I) -> V {
    a.map(|v| v * b)
}
pub(super) fn norm_upper(a: V) -> f64 {
    let mut total = s(0.);
    for component in a {
        if !component.finite() {
            return f64::INFINITY;
        }
        let m = component.magnitude();
        // Expanding a rounded zero product to the least subnormal encloses
        // nonzero underflow. Overflowed squares/sums remain infinite bounds.
        total = up(total + up(m * m));
    }
    let bound = up(total.sqrt());
    if finite(bound) {
        f64::from_bits(bound.to_bits())
    } else {
        f64::INFINITY
    }
}

/// Exact comparison of a nonnegative binary64 upper bound with a nonnegative
/// rational budget. No rounded decimal threshold or absolute epsilon is used.
pub(super) fn within(bound: f64, budget: Rational) -> bool {
    let bits = bound.to_bits();
    if bits & EXPONENT == EXPONENT
        || bits & SIGN != 0 && bits & !SIGN != 0
        || budget.numerator() < 0
    {
        return false;
    }
    if bits & !SIGN == 0 {
        return true;
    }
    let exponent = ((bits >> 52) & 0x7ff) as i32;
    let mantissa = if exponent == 0 {
        bits & 0x000f_ffff_ffff_ffff
    } else {
        (bits & 0x000f_ffff_ffff_ffff) | (1u64 << 52)
    };
    let power = if exponent == 0 {
        -1074
    } else {
        exponent - 1023 - 52
    };
    // At most 53 significand bits times a positive 63-bit denominator fit u128.
    // No floating conversion of the exact budget or absolute error floor occurs.
    let left = u128::from(mantissa) * budget.denominator() as u128;
    let right = budget.numerator() as u128;
    if right == 0 {
        return false;
    }
    let (a, b, shift) = if power >= 0 {
        (left, right, power as u32)
    } else {
        (right, left, (-power) as u32)
    };
    let a_bits = 128 - a.leading_zeros();
    let b_bits = 128 - b.leading_zeros();
    // Compare bit lengths before shifting, including subnormal exponents.
    // Equal lengths imply a_bits + shift <= 128, so that shift loses no bits.
    let ordering = if a_bits + shift == b_bits {
        a.checked_shl(shift)
            .map_or(Ordering::Greater, |v| v.cmp(&b))
    } else {
        (a_bits + shift).cmp(&b_bits)
    };
    if power >= 0 {
        ordering != Ordering::Greater
    } else {
        ordering != Ordering::Less
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcore::math::Vec3;

    fn rational(numerator: i64, denominator: i64) -> Rational {
        Rational::new(numerator, denominator).unwrap()
    }

    fn bound(value: I) -> f64 {
        norm_upper([value, I::point(0.), I::point(0.)])
    }

    #[test]
    fn invalid_evidence_cannot_disappear_during_certificate_accumulation() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let invalid = I::point(value);
            let one = I::point(1.);
            for result in [
                invalid,
                invalid + one,
                one + invalid,
                invalid - one,
                one - invalid,
                invalid * one,
                one * invalid,
                invalid / one,
                one / invalid,
                invalid.sqrt(),
            ] {
                let upper = bound(result);
                assert_eq!(upper, f64::INFINITY);
                assert_eq!(0.0_f64.max(upper), f64::INFINITY);
                assert!(!within(upper, rational(i64::MAX, 1)));
            }
            let vector = native(Vec3::new(0., value, 0.));
            assert_eq!(norm_upper(vector), f64::INFINITY);
        }
    }

    #[test]
    fn undefined_division_square_root_and_overflow_are_unestablished() {
        let one = I::point(1.);
        let zero = I::point(0.);
        let crosses_zero = zero + zero;
        let huge = I::point(f64::MAX);
        let tiny = I::point(f64::from_bits(1));
        for value in [
            one / zero,
            one / crosses_zero,
            zero / zero,
            I::point(-1.).sqrt(),
            huge + huge,
            huge - I::point(-f64::MAX),
            huge * I::point(2.),
            one / tiny,
        ] {
            assert_eq!(bound(value), f64::INFINITY);
            assert_eq!(bound(value * zero + one), f64::INFINITY);
        }
        assert_eq!(bound(huge), f64::INFINITY);
        assert!(within(bound(one / I::point(-2.)), Rational::ONE));
    }

    #[test]
    fn gradual_underflow_and_sqrt_zero_tail_remain_conservative() {
        let tiny = I::point(f64::from_bits(1));
        for value in [tiny * I::point(0.5), tiny / I::point(2.), tiny * tiny] {
            let upper = bound(value);
            assert!(upper.is_finite());
            assert!(upper > 0.);
            assert!(!within(upper, Rational::ZERO));
        }
        let root = (I::point(0.) + I::point(0.)).sqrt();
        assert!(root.lo >= s(0.));
        assert!(root.finite());
        assert!(within(bound(root), rational(1, i64::MAX)));
    }

    #[test]
    fn integer_enclosures_retain_values_beyond_binary64_precision() {
        for value in [0, 1, -1, 1_i64 << 53, -(1_i64 << 53), i64::MIN] {
            let interval = I::integer(value);
            assert!(interval.finite());
            assert_eq!(interval.lo.to_bits(), (value as f64).to_bits());
            assert_eq!(interval.hi.to_bits(), (value as f64).to_bits());
        }
        for value in [(1_i64 << 53) + 1, i64::MAX] {
            let interval = I::integer(value);
            let exact = rational(value, 1);
            assert!(within(f64::from_bits(interval.lo.to_bits()), exact));
            assert!(!within(f64::from_bits(interval.hi.to_bits()), exact));
            let negative = I::integer(-value);
            assert_eq!(negative.lo.to_bits(), (-interval.hi).to_bits());
            assert_eq!(negative.hi.to_bits(), (-interval.lo).to_bits());
        }
        let zero = I::rational(Rational::ZERO);
        assert_eq!(zero.lo.to_bits(), 0);
        assert_eq!(zero.hi.to_bits(), 0);
    }

    #[test]
    fn rational_and_affine_vector_interfaces_enclose_independent_truth() {
        let source = exact([rational(1, 3), rational(-2, 5), rational(7, 4)]);
        let stored = native(Vec3::new(1. / 3., -0.4, 1.75));
        let error = norm_upper(sub(source, stored));
        assert!(within(error, rational(1, 1_000_000_000_000)));
        assert!(!within(error, Rational::ZERO));

        // The independent exact affine result is (3, 0, -2), whose norm is
        // sqrt(13), strictly between 3 and 4. Exercise the importer interface,
        // including a rational normalized-direction factor.
        let vector = add(
            exact([Rational::ONE, Rational::ZERO, rational(-1, 1)]),
            times(
                native(Vec3::new(4., 0., -2.)),
                I::point(1.) / I::rational(rational(4, 1)).sqrt(),
            ),
        );
        let upper = norm_upper(vector);
        assert!(!within(upper, rational(3, 1)));
        assert!(within(upper, rational(4, 1)));
    }

    #[test]
    fn exact_budget_comparison_has_no_decimal_rounding_or_absolute_floor() {
        let tenth = rational(1, 10);
        assert!(!within(0.1, tenth));
        assert!(within(f64::from_bits(0.1_f64.to_bits() - 1), tenth));
        let third = rational(1, 3);
        assert!(within(1. / 3., third));
        assert!(!within(f64::from_bits((1. / 3.0_f64).to_bits() + 1), third));

        let minimum_budget = rational(1, i64::MAX);
        let power = f64::from_bits((1023 - 63) << 52);
        assert!(within(power, minimum_budget));
        assert!(!within(f64::from_bits(power.to_bits() + 1), minimum_budget));
        let maximum_budget = rational(i64::MAX, 1);
        let power = f64::from_bits((1023 + 63) << 52);
        assert!(!within(power, maximum_budget));
        assert!(within(f64::from_bits(power.to_bits() - 1), maximum_budget));

        let subnormal = f64::from_bits(1);
        assert!(within(subnormal, minimum_budget));
        assert!(!within(subnormal, Rational::ZERO));
        assert!(within(0., Rational::ZERO));
        assert!(within(-0., Rational::ZERO));
        assert!(!within(f64::MAX, maximum_budget));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -subnormal, -1.] {
            assert!(!within(value, maximum_budget));
        }
        assert!(!within(0., rational(-1, 1)));
    }

    #[test]
    fn parameter_range_containment_checks_the_entire_finite_interval() {
        assert!(I::point(1.).within_range(1., 2.));
        assert!(I::point(2.).within_range(1., 2.));
        assert!(I::rational(rational(3, 2)).within_range(1., 2.));
        assert!(!I::point(1.).within_range(2., 1.));
        assert!(!I::point(0.).within_range(1., 2.));
        assert!(!I::point(3.).within_range(1., 2.));
        // A rounded sum straddling a range endpoint is not containment proof.
        assert!(!(I::point(1.) + I::point(1.)).within_range(1., 2.));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!I::point(value).within_range(1., 2.));
            assert!(!I::point(1.).within_range(value, 2.));
            assert!(!I::point(1.).within_range(1., value));
        }
        let tiny = f64::from_bits(1);
        assert!(I::point(tiny).within_range(0., tiny));
        assert!(!I::point(tiny).within_range(0., 0.));
    }
}
