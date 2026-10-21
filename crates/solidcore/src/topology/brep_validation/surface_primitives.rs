// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Concrete primitive assertions and their actual revolution parent sequence.
//!
//! Upstream: maintained `d8ed45a`, SmSurfOfRevolution.cpp:6153--6404,
//! SmCone.cpp:5793--5919, SmCylinder.cpp:444--506, SmTorus.cpp:1661--1758;
//! SmBSplineSurface.cpp:6073--6156 (`HasSameParameterization`).
//! Deviations: the nested generator and polar curve are uniquely owned Rust
//! fields. Their context/containment is intrinsic; an external generator owner
//! conflicts with that containment and is rejected. The C++ independently
//! corruptible owner/context backlinks are not represented or adversarially
//! qualified by this port. No missing numerical query is waived by containment.
//! Source report-index mistakes, early-exit guards and tolerance order remain.

use super::{bspline_assertions, missing, object, surface};
use crate::curve::curve::{Curve, CurveKind};
use crate::error::{ignored, Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray};
use crate::geom::extent1d::Extent1d;
use crate::geom::geom_utility::p1::line_point_distance;
use crate::geom::tol::{self, are_same, are_same_to_tol, SmallTopoType};
use crate::kernel::Kernel;
use crate::math::{max2, min2, Point2, Point3, EFF_ZERO, EFF_ZERO_DEG, PI, UNDEF_DOUBLE};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
use crate::surface::surf_of_revolution::p1::{gen_base, gen_step_interval};
use crate::surface::surf_of_revolution::SurfOfRevolution;
use crate::surface::surface::{Surface, SurfaceKind};
use crate::topology::brep_validation::{AssertValidOptions, CheckLevel, Reporter};
use crate::topology::{Brep, FaceId};

fn revolution(s: &Surface) -> Result<&SurfOfRevolution> {
    match s.kind() {
        SurfaceKind::Cone(c) => Ok(c.rev()),
        SurfaceKind::Torus(t) => Ok(t.base()),
        SurfaceKind::SurfOfRevolution(r) => Ok(r),
        _ => Err(missing()),
    }
}

fn same_parameterization(
    s: &Surface,
    analytic: SurfParamType,
    nurb: SurfParamType,
    curve: &Curve,
) -> Result<bool> {
    let rev = revolution(s)?;
    let domain = rev.step_uv_domain();
    let domain = if analytic == SurfParamType::U {
        domain.u_interval()
    } else {
        domain.v_interval()
    };
    let curve_domain = gen_step_interval(curve)?;
    // Both knot queries occur before the domain comparison in the source.
    let (knots, mults) = ignored(rev.base().knots(nurb, None))?.unwrap_or_default();
    let (curve_knots, curve_mults) = ignored(curve.knots(None))?.unwrap_or_default();
    let tolerance = EFF_ZERO * (1. + max2(domain.max_dimension(), curve_domain.max_dimension()));
    let mut valid = domain.is_contained_by(&curve_domain, tolerance)
        && curve_domain.is_contained_by(&domain, tolerance);
    if knots.len() != curve_knots.len() {
        valid = false;
    }
    for (i, knot) in knots.iter().copied().enumerate() {
        if !valid {
            break;
        }
        let other = curve_knots.get(i).copied().ok_or_else(missing)?;
        let mut same = are_same(knot, other);
        valid &= same;
        same &= mults.get(i).ok_or_else(missing)? == curve_mults.get(i).ok_or_else(missing)?;
        valid &= same;
    }
    Ok(valid)
}

fn generator_direction(rev: &SurfOfRevolution) -> SurfParamType {
    if rev.swap_uv() {
        SurfParamType::U
    } else {
        SurfParamType::V
    }
}

pub(super) fn assert_primitive(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.face_mut(f)?
        .surface
        .as_mut()
        .ok_or_else(missing)?
        .bind_evaluation_context(k);
    let mut report = Reporter { reports };
    match surface(b, f)?.kind() {
        SurfaceKind::Cone(c) if c.is_cylinder_class() => {
            let mut valid = true;
            if !matches!(options.level, CheckLevel::Given(_)) {
                valid &= cone_assertions(k, b, f, options, &mut report)?;
            }
            valid &= cylinder_assertions(b, f, &mut report)?;
            Ok(valid)
        }
        SurfaceKind::Cone(_) => cone_assertions(k, b, f, options, &mut report),
        SurfaceKind::Torus(_) => torus_assertions(k, b, f, options, &mut report),
        SurfaceKind::SurfOfRevolution(_) => revolution_assertions(k, b, f, options, &mut report),
        _ => Err(missing()),
    }
}

fn evaluated_point(value: Result<Point3>) -> Result<Point3> {
    Ok(ignored(value)?.unwrap_or(Point3::UNDEF))
}

fn parameterization_nurb(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<()> {
    // HasSameParameterization's virtual GetKnots can create this exact object's
    // missing representation. Restore its owning slot before propagating errors.
    let mut owned = b.face_mut(f)?.surface.take().ok_or_else(missing)?;
    let result = crate::surface::validity::materialize_bspline(k, &mut owned);
    b.face_mut(f)?.surface = Some(owned);
    super::ignore_native_status(result).map(|_| ())
}

fn revolution_assertions(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = true;
    // Source tests m_pNurb before calling its parent: never materialize merely
    // to manufacture a parent validation result for an unmaterialized object.
    if revolution(surface(b, f)?)?.base().gw_nurb().is_some()
        && !matches!(options.level, CheckLevel::Given(_))
    {
        valid &= bspline_assertions(k, b, f, options, report.reports.as_deref_mut())?;
    }
    let s = surface(b, f)?;
    let rev = revolution(s)?;
    let who = object(b, f)?;
    let class = labels::SURF_OF_REVOLUTION_TYPE;
    // These value-owned objects cannot retain a distinct native context pointer.
    valid &= report.class_predicate(class, 0, who, true)?;
    valid &= report.class_predicate(class, 1, who, true)?;
    if let Some(generator) = rev.gen_curve() {
        let inside_out = match s.kind() {
            SurfaceKind::Cone(c) => c.inside_out(),
            SurfaceKind::Torus(t) => t.inside_out(),
            _ => false,
        };
        let gen_inside_out = match generator.kind() {
            CurveKind::Line(l) => l.inside_out(),
            CurveKind::Ellipse(e) => e.inside_out(),
            _ => gen_base(generator)?.inside_out(),
        };
        valid &= report.class_predicate(class, 11, who, gen_inside_out == inside_out)?;
        if rev.base().gw_nurb().is_some() {
            valid &= report.class_value(
                class,
                2,
                who,
                same_parameterization(s, SurfParamType::V, generator_direction(rev), generator)?,
                EFF_ZERO,
                UNDEF_DOUBLE,
            )?;
            if valid && rev.have_polar_conversion() {
                let step = rev.step_uv_domain().evaluate(0., 0.4567);
                let uv =
                    ignored(s.convert_uv_from_step_to_nurbs(k, step))?.unwrap_or(Point2::UNDEF);
                let a = evaluated_point(s.evaluate_step_point(step))?;
                let n = evaluated_point(s.evaluate_point(uv))?;
                let tolerance = EFF_ZERO * (1. + a.max_dimension());
                valid &= report.class_value(
                    class,
                    3,
                    who,
                    a.closer_than(tolerance, n),
                    tolerance,
                    a.distance(n),
                )?;
            }
        }
    }
    let sweep = rev.step_uv_domain().u_interval();
    let tolerance = EFF_ZERO * 100. * 360.;
    let angles = rev.polar_converter().angles();
    if valid && rev.have_polar_conversion() {
        let first = angles.first().copied().ok_or_else(missing)?;
        let last = angles.last().copied().ok_or_else(missing)?;
        valid &= report.class_predicate(class, 4, who, angles.len() > 1)?;
        valid &= report.class_value(
            class,
            5,
            who,
            are_same_to_tol(first, sweep.min(), tolerance)
                || are_same_to_tol(first, sweep.max(), tolerance),
            tolerance,
            max2((first - sweep.min()).abs(), (first - sweep.max()).abs()),
        )?;
        valid &= report.class_value(
            class,
            6,
            who,
            are_same_to_tol(last, sweep.min(), tolerance)
                || are_same_to_tol(last, sweep.max(), tolerance),
            tolerance,
            max2((last - sweep.min()).abs(), (last - sweep.max()).abs()),
        )?;
    }
    if let Some(polar) = rev.polar_curve() {
        valid &= report.class_predicate(class, 12, who, !polar.inside_out())?;
        if rev.base().gw_nurb().is_some() {
            let mut polar_curve = polar.clone();
            // Exact query view: Clone's NURB copy-constructor fixup must not
            // alter the stored knots being compared by this assertion.
            polar_curve.base = polar.base().clone_exact();
            let dir = if rev.swap_uv() {
                SurfParamType::V
            } else {
                SurfParamType::U
            };
            valid &= report.class_value(
                class,
                7,
                who,
                same_parameterization(s, SurfParamType::U, dir, &Curve::from(polar_curve))?,
                EFF_ZERO,
                UNDEF_DOUBLE,
            )?;
            if valid && rev.have_polar_conversion() {
                let step = rev.step_uv_domain().evaluate(0.4567, 0.7654);
                let uv =
                    ignored(s.convert_uv_from_step_to_nurbs(k, step))?.unwrap_or(Point2::UNDEF);
                let a = evaluated_point(s.evaluate_step_point(step))?;
                let n = evaluated_point(s.evaluate_point(uv))?;
                let tolerance = EFF_ZERO * 10. * (1. + a.max_dimension());
                let distance = a.distance(n);
                valid &=
                    report.class_value(class, 8, who, distance < tolerance, tolerance, distance)?;
            }
        }
    }
    if let Some(generator) = rev.gen_curve().filter(|_| rev.base().gw_nurb().is_some()) {
        let domain = rev.base().natural_uv_domain();
        let interval = generator.natural_interval();
        let angle = rev.step_uv_domain().min().x * PI / 180.;
        let cosine = angle.cos();
        let sine = angle.sin();
        let position = rev.position();
        let x = position.x_axis();
        let y = position.y_axis();
        let origin = position.origin();
        let z = position.z_axis();
        for i in 0..4 {
            let parameter = if i == 0 {
                0.
            } else if i == 3 {
                1.
            } else {
                f64::from(i) / 3.
            };
            let u0 = evaluated_point(rev.base().evaluate_point(domain.evaluate(0., parameter)))?;
            let v0 = evaluated_point(rev.base().evaluate_point(domain.evaluate(parameter, 0.)))?;
            let point =
                evaluated_point(gen_base(generator)?.evaluate_point(interval.evaluate(parameter)))?;
            let radius = line_point_distance(origin, z, point).0;
            let point = point - radius * x + radius * (cosine * x + sine * y);
            let tolerance = EFF_ZERO * 100. * (1. + point.max_dimension());
            let (index, distance) = if rev.swap_uv() {
                (9, (point - v0).length())
            } else {
                (10, (point - u0).length())
            };
            valid &=
                report.class_value(class, index, who, distance < tolerance, tolerance, distance)?;
        }
    }
    // The source comments say index13 but execute index2. Keep the executable key.
    valid &= report.class_value(
        class,
        2,
        who,
        sweep.length() <= 360. + EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        sweep.length() - 360.,
    )?;
    let generator = rev.gen_curve().ok_or_else(|| {
        SolidError::unported(
            "SmSurfOfRevolution::AssertValid null generator continuation",
            "fork.topology_validation",
        )
    })?;
    // Unique field ownership is the Rust invariant. A conflicting externally
    // attached owner is observable and fails; no owner metadata is rewritten.
    valid &= report.class_predicate(class, 14, who, generator.owner().is_none())?;
    // A generator owned by a surface has no GetEdge; old-tolerance SmTol.cpp:788
    // uses the supplied Kernel's system tolerance, not the face's tolerance.
    let zone = tol::zone_tol_3d(k, SmallTopoType::NormalSize);
    valid &= report.class_predicate(
        class,
        15,
        who,
        !generator.is_degenerate(
            tol::xsect_tol_3d_from_zones(k, zone, zone),
            Some(&generator.natural_interval()),
        )?,
    )?;
    if let Some(reports) = report.reports.as_deref_mut() {
        reports.remove_cascading(who.key(), class, 15, labels::SURFACE_TYPE, 5);
    }
    Ok(valid)
}

fn cone_assertions(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = true;
    if !matches!(options.level, CheckLevel::Given(_)) {
        valid &= revolution_assertions(k, b, f, options, report)?;
    }
    let SurfaceKind::Cone(cone) = surface(b, f)?.kind() else {
        return Err(missing());
    };
    let who = object(b, f)?;
    let class = labels::CONE_TYPE;
    let top = cone.top_radius();
    let bottom = cone.bot_radius();
    valid &= report.class_value(
        class,
        0,
        who,
        top >= -EFF_ZERO && bottom >= -EFF_ZERO,
        EFF_ZERO,
        min2(top, bottom),
    )?;
    let generator = ignored(cone.rev().create_generator_from_angle(k, 0.))?.ok_or_else(missing)?;
    parameterization_nurb(k, b, f)?;
    let s = surface(b, f)?;
    let SurfaceKind::Cone(cone) = s.kind() else {
        return Err(missing());
    };
    let rev = cone.rev();
    valid &= report.class_value(
        class,
        1,
        who,
        same_parameterization(s, SurfParamType::V, generator_direction(rev), &generator)?,
        EFF_ZERO,
        UNDEF_DOUBLE,
    )?;
    let line = rev.gen_curve().and_then(|g| {
        if let CurveKind::Line(l) = g.kind() {
            Some(l)
        } else {
            None
        }
    });
    valid &= report.class_predicate(class, 4, who, line.is_some())?;
    if let Some(line) = line {
        let scale = line.line_scale();
        let cone_scale = 1. / (cone.semi_angle_deg() * (PI / 180.)).cos();
        let tolerance = (1. + scale) * EFF_ZERO;
        valid &= report.class_value(
            class,
            5,
            who,
            are_same_to_tol(scale, cone_scale, tolerance),
            tolerance,
            scale - cone_scale,
        )?;
    }
    let apex = cone.apex(k)?;
    let interval = rev.step_uv_domain().v_interval();
    valid &= report.class_value(
        class,
        6,
        who,
        apex.is_cylinder
            || !interval.contains_value(apex.param, EFF_ZERO)
            || interval.is_value_on_boundary(apex.param, EFF_ZERO).0,
        EFF_ZERO,
        UNDEF_DOUBLE,
    )?;
    let origin = rev.position().origin();
    let z = rev.position().z_axis();
    let uv = s.natural_uv_domain().evaluate(0.45678, 0.45678);
    let derivatives = ignored(s.evaluate(
        uv,
        1,
        1,
        EvaluateFlags {
            only_upper_half: true,
            ..EvaluateFlags::default()
        },
    ))?;
    let derivative = |i, j| {
        derivatives
            .as_ref()
            .and_then(|m| m.get(i))
            .and_then(|row: &Vec<Point3>| row.get(j))
            .copied()
            .unwrap_or(Point3::UNDEF)
    };
    let point = derivative(0, 0);
    let sweep = if rev.swap_uv() {
        derivative(0, 1)
    } else {
        derivative(1, 0)
    };
    let on_axis = origin + (point - origin).dot(z) * z;
    let value = (point - on_axis).cross(sweep).dot(z);
    valid &= report.class_value(class, 3, who, value > 0., 0., value)?;
    Ok(valid)
}

fn cylinder_assertions(b: &Brep, f: FaceId, report: &mut Reporter<'_>) -> Result<bool> {
    let SurfaceKind::Cone(c) = surface(b, f)?.kind() else {
        return Err(missing());
    };
    let who = object(b, f)?;
    let class = labels::CYLINDER_TYPE;
    let interval = c.rev().step_uv_domain().u_interval();
    let mut valid = report.class_value(
        class,
        0,
        who,
        tol::is_zero(c.semi_angle_deg()),
        EFF_ZERO,
        c.semi_angle_deg(),
    )?;
    valid &= report.class_value(
        class,
        1,
        who,
        interval.length() <= 360. + EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        interval.length() - 360.,
    )?;
    valid &= report.class_value(
        class,
        2,
        who,
        interval.max() <= 360. + EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        interval.max() - 360.,
    )?;
    // Source's minimum-domain check also reports index2, not its label's index3.
    valid &= report.class_value(
        class,
        2,
        who,
        interval.min() >= -360. - EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        interval.min() + 360.,
    )?;
    Ok(valid)
}

fn torus_assertions(
    k: &mut Kernel,
    b: &mut Brep,
    f: FaceId,
    options: AssertValidOptions<'_>,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    let mut valid = true;
    if !matches!(options.level, CheckLevel::Given(_)) {
        valid &= revolution_assertions(k, b, f, options, report)?;
    }
    let SurfaceKind::Torus(torus) = surface(b, f)?.kind() else {
        return Err(missing());
    };
    let who = object(b, f)?;
    let class = labels::TORUS_TYPE;
    valid &= report.class_value(
        class,
        0,
        who,
        torus.major_radius() > 0.,
        0.,
        torus.major_radius(),
    )?;
    valid &= report.class_value(
        class,
        1,
        who,
        torus.minor_radius() > 0.,
        0.,
        torus.minor_radius(),
    )?;
    let rev = torus.base();
    if torus.major_radius() < torus.minor_radius() {
        let angle = 90. + (torus.major_radius() / torus.minor_radius()).acos() * 180. / PI;
        let overlap = Extent1d::from_min_max(angle, 360. - angle);
        if !overlap.are_disjoint_periodic(&rev.step_uv_domain().u_interval(), 360., -1.) {
            valid = false;
        }
    }
    let generator = ignored(rev.create_generator_from_angle(k, 0.))?.ok_or_else(missing)?;
    parameterization_nurb(k, b, f)?;
    let s = surface(b, f)?;
    let rev = revolution(s)?;
    valid &= report.class_value(
        class,
        2,
        who,
        same_parameterization(s, SurfParamType::V, generator_direction(rev), &generator)?,
        EFF_ZERO,
        UNDEF_DOUBLE,
    )?;
    let u = rev.step_uv_domain().u_interval();
    let v = rev.step_uv_domain().v_interval();
    // Preserve the source's actual indices 2/3, including its mislabelled U test.
    valid &= report.class_value(
        class,
        2,
        who,
        u.length() <= 360. + EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        u.length() - 360.,
    )?;
    valid &= report.class_value(
        class,
        3,
        who,
        v.length() <= 360. + EFF_ZERO_DEG,
        EFF_ZERO_DEG,
        v.length() - 360.,
    )?;
    Ok(valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::line::Line;
    use crate::geom::assert_array::{AssertReport, ObjectKey, ReportSpec};
    use crate::geom::attribute::ObjectKey as OwnerKey;
    use crate::geom::axis2_placement::Axis2Placement;
    use crate::geom::extent2d::Extent2d;
    use crate::geom::object::ObjectRef;
    use crate::math::Vec3;
    use crate::surface::cone::Cone;
    use crate::surface::torus::{Torus, TorusOptions};

    fn primitives(kernel: &Kernel) -> Vec<Surface> {
        let mut cone = Cone::new_bounded(
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            2.,
            3.,
            0.,
            90.,
            4.,
            false,
            false,
            false,
        );
        cone.make_nurb(kernel).unwrap();
        let mut cylinder = Cone::new_cylinder_bounded(
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            2.,
            0.,
            90.,
            4.,
            false,
            false,
            false,
        );
        cylinder.make_nurb(kernel).unwrap();
        let domain = Extent2d::from_bounds(0., 0., 90., 90.);
        let torus = Torus::create_canonical(
            kernel,
            &Axis2Placement::IDENTITY,
            4.,
            1.,
            TorusOptions::default(),
            Some(&domain),
        )
        .unwrap();
        let generator = Curve::from(Line::from_points(
            Vec3::new(2., 0., 0.),
            Vec3::new(2., 0., 3.),
            3,
        ));
        let mut rev = SurfOfRevolution::new(
            kernel,
            generator,
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            Extent2d::from_bounds(0., 0., 90., 1.),
            false,
        )
        .unwrap();
        rev.make_nurb(kernel).unwrap();
        vec![cone.into(), cylinder.into(), torus.into(), rev.into()]
    }

    fn owning_face(mut surface: Surface) -> (Brep, FaceId) {
        let mut brep = Brep::new_with_infinite_region().unwrap();
        let shell = brep.add_shell().unwrap();
        let face = brep.add_face(shell).unwrap();
        surface.set_owner(Some(ObjectRef {
            type_code: labels::FACE_TYPE,
            key: OwnerKey(u64::from(face.index())),
        }));
        brep.face_mut(face).unwrap().uv_domain = surface.natural_uv_domain();
        brep.face_mut(face).unwrap().surface = Some(surface);
        (brep, face)
    }

    fn given() -> AssertValidOptions<'static> {
        AssertValidOptions {
            level: CheckLevel::Given(&[]),
            walk: false,
        }
    }

    #[test]
    fn native_primitive_surface_assertions_run_the_actual_full_parent_chain() {
        let mut kernel = Kernel::default();
        for surface in primitives(&kernel) {
            let expected_type = surface.type_code();
            let (mut brep, face) = owning_face(surface);
            let mut reports = AssertArray::new();
            let valid = super::super::assert_surface(
                &mut kernel,
                &mut brep,
                face,
                AssertValidOptions {
                    level: CheckLevel::Two,
                    walk: true,
                },
                Some(&mut reports),
            )
            .unwrap();
            assert!(valid, "type {expected_type}: {:?}", reports.reports());
            assert!(reports.is_empty());
            assert_eq!(
                brep.face(face)
                    .unwrap()
                    .surface
                    .as_ref()
                    .unwrap()
                    .type_code(),
                expected_type
            );
        }
    }

    #[test]
    fn cylinder_given_reports_its_own_semi_angle_without_running_cone_parent() {
        let mut kernel = Kernel::default();
        let mut surface = primitives(&kernel).remove(1);
        let SurfaceKind::Cone(cylinder) = surface.kind_mut() else {
            panic!("cylinder");
        };
        cylinder.semi_angle_deg = 1.;
        let (mut brep, face) = owning_face(surface);
        let mut reports = AssertArray::new();
        assert!(!super::super::assert_surface(
            &mut kernel,
            &mut brep,
            face,
            given(),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(reports.len(), 1);
        assert_eq!(reports.reports()[0].reporting_type, labels::CYLINDER_TYPE);
        assert_eq!(reports.reports()[0].test_index, 0);
    }

    #[test]
    fn cone_and_torus_given_keep_concrete_radius_checks() {
        let mut kernel = Kernel::default();
        let mut shapes = primitives(&kernel);
        let mut torus = shapes.remove(2);
        let mut cone = shapes.remove(0);
        if let SurfaceKind::Cone(c) = cone.kind_mut() {
            c.base_radius = -10.;
        }
        if let SurfaceKind::Torus(t) = torus.kind_mut() {
            t.minor_radius = 0.;
        }
        for (surface, class, index) in
            [(cone, labels::CONE_TYPE, 0), (torus, labels::TORUS_TYPE, 1)]
        {
            let (mut brep, face) = owning_face(surface);
            let mut reports = AssertArray::new();
            assert!(!super::super::assert_surface(
                &mut kernel,
                &mut brep,
                face,
                given(),
                Some(&mut reports)
            )
            .unwrap());
            assert!(reports
                .reports()
                .iter()
                .any(|r| r.reporting_type == class && r.test_index == index));
        }
    }

    #[test]
    fn revolution_value_owned_generator_rejects_conflicting_external_owner() {
        let mut kernel = Kernel::default();
        let mut surface = primitives(&kernel).remove(3);
        let SurfaceKind::SurfOfRevolution(rev) = surface.kind_mut() else {
            panic!("revolution");
        };
        // A different surface owner is a represented metadata conflict. A C++
        // raw backlink damaged independently of containment is not represented.
        rev.gen_curve.as_mut().unwrap().set_owner(
            Some(ObjectRef {
                type_code: labels::SURFACE_TYPE,
                key: OwnerKey(123456),
            }),
            None,
        );
        let (mut brep, face) = owning_face(surface);
        let mut reports = AssertArray::new();
        assert!(!super::super::assert_surface(
            &mut kernel,
            &mut brep,
            face,
            given(),
            Some(&mut reports)
        )
        .unwrap());
        assert!(reports
            .reports()
            .iter()
            .any(|r| r.reporting_type == labels::SURF_OF_REVOLUTION_TYPE && r.test_index == 14));
    }

    #[test]
    fn revolution_cascading_report_removal_is_scoped_and_preserves_last_add() {
        let mut reports = AssertArray::new();
        let object = ObjectKey(101);
        for (owner, class, index) in [
            (object, labels::SURFACE_TYPE, 5),
            (ObjectKey(102), labels::SURFACE_TYPE, 5),
            (object, labels::SURF_OF_REVOLUTION_TYPE, 15),
        ] {
            reports.add_unique_delete_duplicate(
                AssertReport::new(ReportSpec {
                    reporter: Some(owner),
                    owner: Some(owner),
                    test_class: class,
                    test_index: index,
                    ..ReportSpec::default()
                })
                .unwrap(),
            );
        }
        reports.remove_cascading(
            object,
            labels::SURF_OF_REVOLUTION_TYPE,
            15,
            labels::SURFACE_TYPE,
            5,
        );
        assert_eq!(reports.len(), 2);
        assert!(reports.last_add());
        assert_eq!(reports.reports()[0].owner, Some(ObjectKey(102)));
    }
}
