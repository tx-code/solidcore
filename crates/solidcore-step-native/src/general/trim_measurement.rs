// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Portions of finite_lerp(): Copyright (c) Microsoft Corporation.
// SPDX-License-Identifier: Apache-2.0 AND (Apache-2.0 WITH LLVM-exception)
//! Read-only paired-parameter observations of the final installed trim.
//!
//! Upstream: `STEP_IMPORT/src/StepImportTrimMeasurement.h/.cpp`, fork `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | `check_parameters` | `CheckFinalTrimParameters` |
//! | `measure` | `MeasureFinalTrim` |
//! Deviations: checked counters replace `size_t` overflow; Rust missing dependencies
//! retain their explicit `Unported` error. Samples never establish a continuous bound.

use super::{failure, GeneralFailureKind};
use crate::{Error, Result};
use solidcore::curve::curve::Curve;
use solidcore::geom::tol::ScaledZero;
use solidcore::kernel::Kernel;
use solidcore::math::{is_infinite_param, Vec2};
use solidcore::surface::surface::Surface;

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct TrimMeasurement {
    pub maximum_mm: f64,
    pub samples: usize,
}

fn rejected(key: &'static str, record: usize) -> Error {
    failure(key, record, GeneralFailureKind::Indeterminate)
}

fn native<T>(value: solidcore::Result<T>, key: &'static str, record: usize) -> Result<T> {
    match value {
        Ok(value) => Ok(value),
        Err(error) if error.is_unported() => Err(Error::Kernel(error)),
        Err(_) => Err(rejected(key, record)),
    }
}

fn check_parameters(edge: &Curve, uv: &Curve, begin: f64, end: f64, record: usize) -> Result<()> {
    let domain = uv.natural_interval();
    let edge_domain = edge.natural_interval();
    // The source's parameter policy is SmEdgeuse::AssertValid tests 25/26,
    // 100*GetScaledZero at the DEFINING curve endpoints. A mm gap allowance
    // cannot excuse another UV domain or authorize an affine parameter map.
    if !begin.is_finite()
        || !end.is_finite()
        || end <= begin
        || !domain.min().is_finite()
        || !domain.max().is_finite()
        || !edge_domain.min().is_finite()
        || !edge_domain.max().is_finite()
        || domain.max() <= domain.min()
        || edge_domain.max() <= edge_domain.min()
        || is_infinite_param(edge_domain.min())
        || is_infinite_param(edge_domain.max())
    {
        return Err(rejected("FinalTrimParameterDomain", record));
    }
    let min_roundoff = 100. * edge_domain.min().scaled_zero();
    let max_roundoff = 100. * edge_domain.max().scaled_zero();
    if !min_roundoff.is_finite()
        || !max_roundoff.is_finite()
        || (domain.min() - begin).abs() > min_roundoff
        || (domain.max() - end).abs() > max_roundoff
        || domain.min() < edge_domain.min() - min_roundoff
        || domain.max() > edge_domain.max() + max_roundoff
    {
        return Err(rejected("FinalTrimParameterDomain", record));
    }
    Ok(())
}

// MSVC 14.50.35717 include/cmath::_Common_lerp/_Linear_for_lerp finite
// runtime path, lines 1562--1627. CheckFinalTrimParameters and the strict
// interior-knot filter establish finite endpoints; i/8 is finite in [0,1].
// The same-sign path explicitly calls std::fma upstream, so mul_add here
// PRESERVES its single rounding. Do not replace the opposite-sign expression
// with FMA or use a+(b-a)*t for that branch. Microsoft STL's license is kept
// in LICENSES/Microsoft-STL.txt; see ATTRIBUTION.md.
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

pub(super) fn measure(
    kernel: &Kernel,
    edge: &Curve,
    uv: &Curve,
    surface: &Surface,
    begin: f64,
    end: f64,
    record: usize,
) -> Result<TrimMeasurement> {
    check_parameters(edge, uv, begin, end, record)?;
    let mut breaks = vec![begin, end];
    for curve in [edge, uv] {
        let (knots, _) = native(curve.knots(None), "FinalTrimKnots", record)?;
        for knot in knots {
            if knot > begin && knot < end {
                breaks.push(knot);
            }
        }
    }
    // The strict interior filter excludes NaNs and infinities. Keep ordinary
    // double comparison/equality, including the source's -0 == +0 behavior.
    breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    breaks.dedup();
    let mut result = TrimMeasurement::default();
    for span in breaks.windows(2) {
        let (Some(a), Some(b)) = (span.first(), span.get(1)) else {
            return Err(Error::Invalid("FinalTrimSpanInvariant"));
        };
        for i in 0..=8_u32 {
            let t = finite_lerp(*a, *b, f64::from(i) / 8.);
            let point = native(
                edge.evaluate_point(kernel, t),
                "FinalTrimEvaluation",
                record,
            )?;
            let param = native(uv.evaluate_point(kernel, t), "FinalTrimEvaluation", record)?;
            let lifted = native(
                surface.evaluate_point(kernel, Vec2::new(param.x, param.y)),
                "FinalTrimEvaluation",
                record,
            )?;
            // SmVector3d.h:958 uses left-associated squared differences and
            // smos_Sqrt. Vec3::distance is that body; geometry::norm instead
            // ports three-argument hypot and is not equivalent here.
            let gap = point.distance(lifted);
            if !gap.is_finite() {
                return Err(rejected("FinalTrimNonfinite", record));
            }
            if gap > result.maximum_mm {
                result.maximum_mm = gap;
            }
            result.samples = result.samples.checked_add(1).ok_or(Error::ResourceLimit)?;
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcore::curve::line::Line;
    use solidcore::geom::extent1d::Extent1d;
    use solidcore::geom::extent2d::Extent2d;
    use solidcore::math::{Vec3, INFINITE_PARAMETER};
    use solidcore::nurbs::b_spline_curve::{
        BSplineCurve, BSplineCurveForm, CanonicalData, KnotType,
    };
    use solidcore::surface::plane::Plane;

    fn plane() -> Surface {
        Plane::new(
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec2::new(1., 1.),
            &Extent2d::from_bounds(-1., -1., 2., 2.),
        )
        .into()
    }
    fn line(dim: u64, y: f64, interval: Option<&Extent1d>) -> Curve {
        Line::create_line_segment(dim, Vec3::new(0., y, 0.), Vec3::new(1., y, 0.), interval)
            .unwrap()
            .into()
    }
    fn spline(
        dim: u64,
        degree: usize,
        controls: &[Vec3],
        knots: &[f64],
        multiplicities: &[usize],
    ) -> Curve {
        BSplineCurve::create_canonical(&CanonicalData {
            dim,
            degree,
            control_points: controls,
            form: BSplineCurveForm::Unspecified,
            knot_multiplicities: multiplicities,
            knots,
            knot_type: KnotType::Unspecified,
            weights: None,
            trim: None,
        })
        .unwrap()
        .into()
    }
    fn domain_failure(result: &Result<TrimMeasurement>) {
        assert!(matches!(
            result,
            Err(Error::General {
                key: "FinalTrimParameterDomain",
                ..
            })
        ));
    }

    #[test]
    fn same_parameter_straight_control_and_displaced_uv_follow_cpp_measurement_contract() {
        let kernel = Kernel::default();
        let edge = line(3, 0., None);
        let exact = measure(&kernel, &edge, &line(2, 0., None), &plane(), 0., 1., 0).unwrap();
        assert_eq!(exact.samples, 9);
        assert_eq!(exact.maximum_mm, 0.);
        let displaced = measure(&kernel, &edge, &line(2, 0.02, None), &plane(), 0., 1., 0).unwrap();
        assert_eq!(displaced.samples, 9);
        assert!((displaced.maximum_mm - 0.02).abs() < 1e-12);
        assert!(displaced.maximum_mm > 0.001);
    }

    #[test]
    fn covering_affinely_remapped_and_outside_domains_are_not_paired_parameters() {
        let kernel = Kernel::default();
        let edge = line(3, 0., None);
        let wider = Extent1d::from_min_max(-0.25, 1.25);
        let uv = line(2, 0., Some(&wider));
        domain_failure(&measure(&kernel, &edge, &uv, &plane(), 0., 1., 0));
        domain_failure(&measure(&kernel, &edge, &uv, &plane(), -0.25, 1.25, 0));
        let uv = line(2, 0., None);
        domain_failure(&measure(&kernel, &edge, &uv, &plane(), 0.25, 0.75, 0));
        domain_failure(&measure(&kernel, &edge, &uv, &plane(), 1., 0., 0));
        domain_failure(&measure(
            &kernel,
            &edge,
            &uv,
            &plane(),
            0.,
            f64::INFINITY,
            0,
        ));
        domain_failure(&measure(&kernel, &edge, &uv, &plane(), f64::NAN, 1., 0));
        let marker = Extent1d::from_min_max(0., INFINITE_PARAMETER);
        domain_failure(&measure(
            &kernel,
            &line(3, 0., Some(&marker)),
            &line(2, 0., Some(&marker)),
            &plane(),
            marker.min(),
            marker.max(),
            0,
        ));
    }

    #[test]
    fn source_parameter_roundoff_is_allowed_but_actual_physical_gap_is_still_measured() {
        let kernel = Kernel::default();
        let roundoff = 4. * f64::EPSILON;
        let near = Extent1d::from_min_max(-roundoff, 1. + roundoff);
        // StepImportGeneralTests.cpp::FinalTrimContract uses explicit knots:
        // Line's optional reparameterization can accept these endpoints as
        // already equal without changing its actual domain. Retain the real
        // roundoff here instead of assuming it must cause nonzero distance.
        let uv = spline(
            2,
            1,
            &[Vec3::ZERO, Vec3::X],
            &[near.min(), near.max()],
            &[2, 2],
        );
        assert_eq!(uv.natural_interval(), near);
        let edge = line(3, 0., None);
        let measured = measure(&kernel, &edge, &uv, &plane(), 0., 1., 0).unwrap();
        assert_eq!(measured.samples, 9);
        assert!(measured.maximum_mm < 1e-12);

        // The plane lift has y=0.02 for every t, independently of its x map.
        // Thus distance=sqrt((t-x_uv(t))^2+0.02^2), bounded below by 0.02;
        // roundoff in x cannot excuse that physical displacement.
        let shifted_uv = spline(
            2,
            1,
            &[Vec3::new(0., 0.02, 0.), Vec3::new(1., 0.02, 0.)],
            &[near.min(), near.max()],
            &[2, 2],
        );
        assert_eq!(shifted_uv.natural_interval(), near);
        let shifted = measure(&kernel, &edge, &shifted_uv, &plane(), 0., 1., 0).unwrap();
        assert_eq!(shifted.samples, 9);
        assert!((shifted.maximum_mm - 0.02).abs() < 1e-12);
        assert!(shifted.maximum_mm > 0.001);
    }

    #[test]
    fn identical_curve_images_with_a_different_parameter_map_do_not_hide_the_lift_gap() {
        let kernel = Kernel::default();
        // Both curves trace the same x-axis segment with the same domain and
        // endpoints, but x_uv(t)=t^2. Closest-distance geometry would miss this;
        // the independent same-t maximum is t-t^2=1/4 at t=1/2.
        let uv = spline(2, 2, &[Vec3::ZERO, Vec3::ZERO, Vec3::X], &[0., 1.], &[3, 3]);
        let measured = measure(&kernel, &line(3, 0., None), &uv, &plane(), 0., 1., 0).unwrap();
        assert_eq!(measured.samples, 9);
        assert_eq!(measured.maximum_mm, 0.25);
    }

    #[test]
    fn both_knot_sets_partition_the_actual_used_interval_with_nine_samples_per_span() {
        let kernel = Kernel::default();
        let edge = spline(
            3,
            1,
            &[Vec3::ZERO, Vec3::new(0.25, 0., 0.), Vec3::X],
            &[0., 0.25, 1.],
            &[2, 1, 2],
        );
        let uv = spline(
            2,
            1,
            &[Vec3::ZERO, Vec3::new(0.5, 0., 0.), Vec3::X],
            &[0., 0.5, 1.],
            &[2, 1, 2],
        );
        let measured = measure(&kernel, &edge, &uv, &plane(), 0., 1., 0).unwrap();
        assert_eq!(measured.samples, 27);
        assert!(measured.maximum_mm <= f64::EPSILON);
        let interval = Extent1d::from_min_max(0.25, 0.75);
        let child_uv = spline(
            2,
            1,
            &[Vec3::new(0.25, 0., 0.), Vec3::new(0.75, 0., 0.)],
            &[0.25, 0.75],
            &[2, 2],
        );
        let child = measure(
            &kernel,
            &edge,
            &child_uv,
            &plane(),
            interval.min(),
            interval.max(),
            0,
        )
        .unwrap();
        assert_eq!(child.samples, 9);
        assert!(child.maximum_mm <= f64::EPSILON);
    }

    #[test]
    fn native_distance_overflow_is_not_replaced_by_a_finite_hypot_observation() {
        let kernel = Kernel::default();
        let point = Vec3::new(1e200, 0., 0.);
        let edge = spline(3, 1, &[point, point], &[0., 1.], &[2, 2]);
        assert!(matches!(
            measure(&kernel, &edge, &line(2, 0., None), &plane(), 0., 1., 0),
            Err(Error::General {
                key: "FinalTrimNonfinite",
                ..
            })
        ));
    }

    #[test]
    fn finite_msvc_lerp_keeps_endpoints_cross_zero_and_nondecreasing_span_samples() {
        for (a, b) in [(-3., 5.), (2., 9.), (-9., -2.), (1e300, 1.5e300)] {
            let values = (0..=8)
                .map(|i| finite_lerp(a, b, f64::from(i) / 8.))
                .collect::<Vec<_>>();
            assert_eq!(values[0], a);
            assert_eq!(values[8], b);
            assert!(values.windows(2).all(|pair| pair[0] <= pair[1]));
        }
    }
}
