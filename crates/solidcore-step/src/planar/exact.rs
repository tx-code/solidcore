// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bounded exact arithmetic for planar source evidence, never a rounding fallback.
//! Upstream: `source/STEP_PLANAR/src/StepExact.cpp` at `d8ed45a`.
//! Deviations: fallible methods replace throwing operators. This decimal contract
//! intentionally differs from dimensional rational parsing (exponent/zero timing).

use serde::{ser::SerializeStruct, Serialize, Serializer};
use std::cmp::Ordering;

/// An exact operation could not establish a result in the bounded representation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactError {
    /// Intermediate or decimal representation exceeded signed 63-bit magnitude.
    Capacity,
    /// A required division has a zero denominator.
    UndefinedDivision,
    /// The caller provided a malformed decimal, rather than a source number.
    InvalidDecimal,
}

/// Reduced rational with a positive denominator and no `i64::MIN` numerator.
/// All geometry decisions use these exact values; capacity is not source rejection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Rational {
    n: i64,
    d: i64,
}
impl Default for Rational {
    fn default() -> Self {
        Self::ZERO
    }
}
fn gcd(mut a: i64, mut b: i64) -> i64 {
    while b != 0 {
        let remainder = a % b;
        a = b;
        b = remainder;
    }
    a
}
fn product(a: i64, b: i64) -> Result<i64, ExactError> {
    a.checked_mul(b)
        .filter(|v| *v != i64::MIN)
        .ok_or(ExactError::Capacity)
}
fn sum(a: i64, b: i64) -> Result<i64, ExactError> {
    a.checked_add(b)
        .filter(|v| *v != i64::MIN)
        .ok_or(ExactError::Capacity)
}
impl Rational {
    /// Exact zero.
    pub const ZERO: Self = Self { n: 0, d: 1 };
    /// Exact one.
    pub const ONE: Self = Self { n: 1, d: 1 };
    pub(super) const FIXTURE_MM: Self = Self {
        n: 1,
        d: 10_000_000,
    };
    /// Validate and reduce a numerator/positive denominator pair.
    pub fn new(n: i64, d: i64) -> Result<Self, ExactError> {
        if n == i64::MIN || d <= 0 {
            return Err(ExactError::Capacity);
        }
        let divisor = gcd(n.abs(), d);
        Ok(Self {
            n: n / divisor,
            d: d / divisor,
        })
    }
    /// Read-only reduced numerator.
    pub const fn numerator(self) -> i64 {
        self.n
    }
    /// Read-only positive reduced denominator.
    pub const fn denominator(self) -> i64 {
        self.d
    }
    /// Interpret a decimal using the original planar capacity/error precedence.
    pub fn decimal(text: &str) -> Result<Self, ExactError> {
        let mut bytes = text.as_bytes().iter().copied().peekable();
        if bytes.peek().is_none() {
            return Err(ExactError::InvalidDecimal);
        }
        let negative = bytes.peek() == Some(&b'-');
        if matches!(bytes.peek(), Some(b'+' | b'-')) {
            bytes.next();
        }
        let mut digits = String::new();
        let mut fractional: i64 = 0;
        let mut dot = false;
        while let Some(&byte) = bytes.peek() {
            if matches!(byte, b'E' | b'e') {
                break;
            }
            bytes.next();
            if byte == b'.' {
                if dot {
                    return Err(ExactError::InvalidDecimal);
                }
                dot = true;
            } else {
                if !byte.is_ascii_digit() {
                    return Err(ExactError::InvalidDecimal);
                }
                digits.push(char::from(byte));
                if dot {
                    fractional += 1;
                }
            }
        }
        if digits.is_empty() {
            return Err(ExactError::InvalidDecimal);
        }
        let mut exponent: i64 = 0;
        if bytes.next().is_some() {
            let minus = bytes.peek() == Some(&b'-');
            if matches!(bytes.peek(), Some(b'+' | b'-')) {
                bytes.next();
            }
            if bytes.peek().is_none() {
                return Err(ExactError::InvalidDecimal);
            }
            for byte in bytes {
                if !byte.is_ascii_digit() {
                    return Err(ExactError::InvalidDecimal);
                }
                // Original StepExact checks BEFORE the next digit, including zero
                // mantissas. Do not reuse the unit parser's zero short circuit.
                if exponent > 100_000 {
                    return Err(ExactError::Capacity);
                }
                exponent = exponent * 10 + i64::from(byte - b'0');
            }
            if minus {
                exponent = -exponent;
            }
        }
        let reduced = digits.trim_start_matches('0');
        if reduced.is_empty() {
            return Ok(Self::ZERO);
        }
        exponent -= fractional;
        let coefficient = reduced.trim_end_matches('0');
        exponent += (reduced.len() - coefficient.len()) as i64;
        if coefficient.len() > 19 || !(-18..=18).contains(&exponent) {
            return Err(ExactError::Capacity);
        }
        let mut n = 0;
        let mut d = 1;
        for byte in coefficient.bytes() {
            n = sum(product(n, 10)?, i64::from(byte - b'0'))?;
        }
        for _ in 0..exponent.max(0) {
            n = product(n, 10)?;
        }
        for _ in 0..(-exponent).max(0) {
            d = product(d, 10)?;
        }
        Self::new(if negative { -n } else { n }, d)
    }
    /// Checked addition; preserve the original intermediate products and reductions.
    pub fn checked_add(self, b: Self) -> Result<Self, ExactError> {
        let g = gcd(self.d, b.d);
        Self::new(
            sum(product(self.n, b.d / g)?, product(b.n, self.d / g)?)?,
            product(self.d, b.d / g)?,
        )
    }
    /// Checked subtraction.
    pub fn subtract(self, b: Self) -> Result<Self, ExactError> {
        self.checked_add(b.negate())
    }
    /// Exact negation; the excluded `i64::MIN` makes it total.
    #[must_use]
    pub const fn negate(self) -> Self {
        Self {
            n: -self.n,
            d: self.d,
        }
    }
    /// Checked multiplication with the original cross cancellations.
    pub fn multiply(self, b: Self) -> Result<Self, ExactError> {
        let x = gcd(self.n.abs(), b.d);
        let y = gcd(b.n.abs(), self.d);
        Self::new(product(self.n / x, b.n / y)?, product(self.d / y, b.d / x)?)
    }
    /// Checked division; zero remains explicitly undefined.
    pub fn divide(self, b: Self) -> Result<Self, ExactError> {
        if b.n == 0 {
            return Err(ExactError::UndefinedDivision);
        }
        self.multiply(Self::new(if b.n < 0 { -b.d } else { b.d }, b.n.abs())?)
    }
}
impl Ord for Rational {
    fn cmp(&self, b: &Self) -> Ordering {
        if (self.n < 0) != (b.n < 0) {
            return self.n.cmp(&b.n);
        }
        // Continued fractions avoid overflowing cross products. The reversal
        // after each reciprocal and the final negative sign are independent.
        let (mut a, mut ad, mut c, mut cd) = (self.n.abs(), self.d, b.n.abs(), b.d);
        let mut reversed = false;
        let order = loop {
            let (x, y) = (a / ad, c / cd);
            if x != y {
                break if reversed { y.cmp(&x) } else { x.cmp(&y) };
            }
            let (r, s) = (a % ad, c % cd);
            if r == 0 || s == 0 {
                break if reversed { s.cmp(&r) } else { r.cmp(&s) };
            }
            (a, ad, c, cd) = (ad, r, cd, s);
            reversed = !reversed;
        };
        if self.n < 0 {
            order.reverse()
        } else {
            order
        }
    }
}
impl PartialOrd for Rational {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Serialize for Rational {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut value = serializer.serialize_struct("Rational", 3)?;
        value.serialize_field("kind", "rational")?;
        value.serialize_field("num", &self.n.to_string())?;
        value.serialize_field("den", &self.d.to_string())?;
        value.end()
    }
}
