// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Exact source-decimal order and bounded rational dimension arithmetic.
//! Upstream: StepSourceNumber.cpp and StepUnitRational.cpp under `STEP_SEMANTICS/src`.
//! compare/Rational correspond to `CompareSourceReals/unit_dimensions::Rational`.
//! Deviations: checked Rust arithmetic, identical signed-i64 capacity on native
//! and wasm; no binary floats, exponent expansion, or arithmetic-capacity widening.
use std::cmp::Ordering;

fn order(value: Ordering) -> i32 {
    match value {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}
fn magnitude(a: &str, b: &str) -> i32 {
    order(a.len().cmp(&b.len()).then_with(|| a.cmp(b)))
}
fn exponent(text: &str) -> (bool, &str) {
    let minus = text.starts_with('-');
    let digits = text.trim_start_matches(['+', '-']).trim_start_matches('0');
    if digits.is_empty() {
        (false, "0")
    } else {
        (minus, digits)
    }
}
fn capped(digits: &str, cap: i64) -> i64 {
    let mut n = 0;
    for c in digits.bytes() {
        let d = i64::from(c - b'0');
        if d > cap || n > (cap - d) / 10 {
            return cap;
        }
        n = n * 10 + d;
    }
    n.min(cap)
}
fn difference(high: &str, low: &str, cap: i64) -> i64 {
    let mut result = 0;
    let mut place = 1;
    let mut borrow = 0;
    let mut low = low.bytes().rev();
    for h in high.bytes().rev() {
        let mut d = i32::from(h - b'0') - borrow - low.next().map_or(0, |c| i32::from(c - b'0'));
        borrow = i32::from(d < 0);
        if borrow != 0 {
            d += 10;
        }
        let d = i64::from(d);
        if d != 0 && place > (cap - result) / d {
            result = cap;
        } else {
            result += d * place;
        }
        place = if place > cap / 10 { cap } else { place * 10 };
    }
    result
}
fn exponent_difference(a: (bool, &str), b: (bool, &str), cap: i64) -> i64 {
    if a.0 != b.0 {
        let sum = (capped(a.1, cap) + capped(b.1, cap)).min(cap);
        return if a.0 { -sum } else { sum };
    }
    let cmp = magnitude(a.1, b.1);
    if cmp == 0 {
        return 0;
    }
    let n = if cmp > 0 {
        difference(a.1, b.1, cap)
    } else {
        difference(b.1, a.1, cap)
    };
    i64::from(if a.0 { -cmp } else { cmp }) * n
}
struct Decimal<'a> {
    sign: i32,
    digits: String,
    offset: i64,
    exponent: (bool, &'a str),
}
fn decimal(text: &str) -> Decimal<'_> {
    let sign = if text.starts_with('-') { -1 } else { 1 };
    let text = text.trim_start_matches(['+', '-']);
    let (mantissa, exp) = text.split_once(['E', 'e']).unwrap_or((text, "0"));
    let digits = mantissa.replace('.', "");
    let first = digits.bytes().position(|c| c != b'0');
    let Some(first) = first else {
        return Decimal {
            sign: 0,
            digits,
            offset: 0,
            exponent: exponent(exp),
        };
    };
    let point = mantissa.find('.').unwrap_or(mantissa.len());
    let trimmed = digits
        .get(first..)
        .unwrap_or("")
        .trim_end_matches('0')
        .to_owned();
    Decimal {
        sign,
        digits: trimmed,
        offset: point as i64 - first as i64,
        exponent: exponent(exp),
    }
}
pub(super) fn compare(left: &str, right: &str) -> i32 {
    let a = decimal(left);
    let b = decimal(right);
    if a.sign != b.sign {
        return order(a.sign.cmp(&b.sign));
    }
    if a.sign == 0 {
        return 0;
    }
    // Saturate only the *difference*: nearly equal huge exponents must retain
    // borrows, and a capped result dominates all possible mantissa offsets.
    let cap = a.offset.abs() + b.offset.abs() + 3;
    let position = exponent_difference(a.exponent, b.exponent, cap) + a.offset - b.offset;
    if position != 0 {
        return a.sign * order(position.cmp(&0));
    }
    for (x, y) in a
        .digits
        .bytes()
        .chain(std::iter::repeat(b'0'))
        .zip(b.digits.bytes().chain(std::iter::repeat(b'0')))
        .take(a.digits.len().max(b.digits.len()))
    {
        if x != y {
            return a.sign * order(x.cmp(&y));
        }
    }
    0
}
pub(super) fn index(text: &str, maximum: u64) -> Option<u64> {
    let negative = text.starts_with('-');
    let digits = text.trim_start_matches(['+', '-']).trim_start_matches('0');
    if digits.is_empty() {
        return Some(0);
    }
    if negative || magnitude(digits, &maximum.to_string()) > 0 {
        return None;
    }
    digits.parse().ok()
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct Rational {
    pub n: i64,
    pub d: i64,
}
impl Default for Rational {
    fn default() -> Self {
        Self { n: 0, d: 1 }
    }
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Capacity;
fn gcd(mut a: i64, mut b: i64) -> i64 {
    while b != 0 {
        let r = a % b;
        a = b;
        b = r;
    }
    a
}
fn product(a: i64, b: i64) -> Result<i64, Capacity> {
    a.checked_mul(b).filter(|n| *n != i64::MIN).ok_or(Capacity)
}
fn sum(a: i64, b: i64) -> Result<i64, Capacity> {
    a.checked_add(b).filter(|n| *n != i64::MIN).ok_or(Capacity)
}
impl Rational {
    pub fn new(n: i64, d: i64) -> Self {
        let g = gcd(n.abs(), d);
        Self { n: n / g, d: d / g }
    }
    pub fn add(self, b: Self) -> Result<Self, Capacity> {
        let g = gcd(self.d, b.d);
        Ok(Self::new(
            sum(product(self.n, b.d / g)?, product(b.n, self.d / g)?)?,
            product(self.d, b.d / g)?,
        ))
    }
    pub fn multiply(self, b: Self) -> Result<Self, Capacity> {
        let x = gcd(self.n.abs(), b.d);
        let y = gcd(b.n.abs(), self.d);
        Ok(Self::new(
            product(self.n / x, b.n / y)?,
            product(self.d / y, b.d / x)?,
        ))
    }
    pub fn decimal(text: &str) -> Result<Self, Capacity> {
        let negative = text.starts_with('-');
        let text = text.trim_start_matches(['+', '-']);
        let (mantissa, exp) = text.split_once(['E', 'e']).unwrap_or((text, "0"));
        let digits = mantissa.replace('.', "");
        let first = digits.bytes().position(|c| c != b'0');
        let Some(first) = first else {
            return Ok(Self::default());
        }; // zero stays exact even at huge exponent
        let reduced = digits.get(first..).ok_or(Capacity)?.trim_end_matches('0');
        let trailing = digits.len() - first - reduced.len();
        let frac = mantissa.find('.').map_or(0, |p| mantissa.len() - p - 1);
        let (minus, exponent_digits) = exponent(exp);
        let magnitude = index(exponent_digits, text.len() as u64 + 20).ok_or(Capacity)? as i64;
        let mut exponent = if minus { -magnitude } else { magnitude };
        exponent += trailing as i64 - frac as i64;
        let mut reduced = reduced.as_bytes().to_vec();
        let mut denominator = 1;
        if exponent < 0 {
            let places = (-exponent) as usize;
            if places >= 63 || reduced.len() > 19 + places {
                return Err(Capacity);
            }
            while exponent < 0 {
                let last = *reduced.last().ok_or(Capacity)? - b'0';
                let divisor: u8 = if last % 2 == 0 {
                    2
                } else if last == 5 {
                    5
                } else {
                    1
                };
                denominator = product(denominator, 10 / i64::from(divisor))?;
                if divisor != 1 {
                    let mut remainder = 0;
                    for digit in &mut reduced {
                        let n = remainder * 10 + u32::from(*digit - b'0');
                        *digit = b'0' + (n / u32::from(divisor)) as u8;
                        remainder = n % u32::from(divisor);
                    }
                    if reduced.first() == Some(&b'0') {
                        reduced.remove(0);
                    }
                }
                exponent += 1;
            }
        }
        let reduced = std::str::from_utf8(&reduced).map_err(|_| Capacity)?;
        let coefficient = index(reduced, i64::MAX as u64).ok_or(Capacity)? as i64;
        let mut value = Self::new(
            if negative { -coefficient } else { coefficient },
            denominator,
        );
        while exponent > 0 {
            value.n = product(value.n, 10)?;
            exponent -= 1;
        }
        Ok(value)
    }
}
