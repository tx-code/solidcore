// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::curve::{
    curve::p7::PropertyAnalysisOptions,
    curve::{Curve, CurveKind, CurvePropertyType},
    line::Line,
};
use crate::geom::solution_array::{ObjectKind, SolutionObject};
use crate::math::{Point3, Vec3};
use crate::nurbs::{
    b_spline_curve::BSplineCurve,
    crv_basic::Curve as Nurb,
    math::{CPoint, NOW},
};
use crate::topology::{
    brep::p2,
    brep_cache::{arena_identity, CacheObject},
    brep_constructor::BrepConstructor,
};
fn fixture(mut c: Curve) -> (Brep, EdgeId) {
    let mut b = BrepConstructor::new().unwrap().finish().unwrap();
    let (_, v) = p2::make_shell_vertex_topology(&mut b, None).unwrap();
    let (edge, _) = p2::make_edge_vertex_topology(&mut b, None, v).unwrap();
    c.set_owner(Some(arena_identity(CacheObject::Edge(edge))), None);
    b.edge_mut(edge).unwrap().curve = Some(c);
    (b, edge)
}
fn bspline(points: &[(f64, f64)], degree: usize, knots: Vec<f64>) -> Curve {
    BSplineCurve::from_nurb(
        Nurb::new(
            points
                .iter()
                .map(|(x, y)| CPoint::new(*x, *y, 0., NOW))
                .collect(),
            degree,
            knots,
        )
        .unwrap(),
        3,
        true,
    )
    .into()
}
#[derive(Clone, Copy, PartialEq)]
struct Opaque;
impl SolutionObject for Opaque {
    fn kind(&self) -> ObjectKind {
        ObjectKind::Other
    }
}

#[test]
fn property_solver_uses_actual_curve_domain_with_absent_or_opaque_output_handles() {
    // Keep the SDK fixture's scaling operation: replacing this by a decimal
    // 1e-11 literal changes the ordinate's binary64 bits.
    let c = bspline(
        &[(0., 0.), (0.001, 0.), (0., 1e-8 * 0.001)],
        2,
        vec![0., 0., 0., 1., 1., 1.],
    );
    for handle in [None, Some(Opaque)] {
        let out = c
            .global_property_analysis(
                &Kernel::default(),
                &c.natural_interval(),
                CurvePropertyType::MinimizeFirstDerivative,
                0.000025,
                &PropertyAnalysisOptions {
                    handle,
                    ..Default::default()
                },
            )
            .unwrap();
        // SDK's five candidates: approximate minima coalesce under the actual
        // natural-domain rule, leaving the tighter exact midpoint and four
        // other candidates. No acceptance/report epsilon is enlarged.
        assert_eq!(out.solutions.len(), 5);
        assert_eq!(out.solutions.as_slice()[0].start.param(0).unwrap(), 0.5);
        assert_eq!(
            out.solutions.as_slice()[0].start.solution_value,
            1.0000000000000002e-22
        );
        assert!(out
            .solutions
            .as_slice()
            .iter()
            .all(|s| s.objects[0] == handle));
    }
}

#[test]
fn failed_base_materialization_keeps_missing_nurb_and_the_prefix_diagnostic() {
    let (mut b, id) = fixture(BSplineCurve::new(3).into());
    let mut reports = AssertArray::new();
    assert!(!assert_edge_curve(
        &mut Kernel::default(),
        &mut b,
        id,
        AssertValidOptions::default(),
        Some(&mut reports)
    )
    .unwrap());
    assert_eq!(reports.len(), 1);
    assert_eq!(reports.reports()[0].test_index, 0);
    assert_eq!(
        reports.reports()[0].reporting_type,
        labels::B_SPLINE_CURVE_TYPE
    );
    let CurveKind::BSpline(c) = b.edge(id).unwrap().curve.as_ref().unwrap().kind() else {
        panic!()
    };
    assert!(c.gw_nurb().is_none());
}

#[test]
fn given_line_keeps_missing_nurb_but_nongiven_materializes_actual_owned_curve() {
    let (mut b, id) = fixture(Line::from_points(Point3::ZERO, Vec3::X, 3).into());
    let mut k = Kernel::default();
    assert!(assert_edge_curve(
        &mut k,
        &mut b,
        id,
        AssertValidOptions {
            level: CheckLevel::Given(&[]),
            walk: true
        },
        None
    )
    .unwrap());
    let CurveKind::Line(line) = b.edge(id).unwrap().curve.as_ref().unwrap().kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_none());
    assert!(assert_edge_curve(&mut k, &mut b, id, AssertValidOptions::default(), None).unwrap());
    let c = b.edge(id).unwrap().curve.as_ref().unwrap();
    let CurveKind::Line(line) = c.kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_some());
    assert_eq!(c.owner(), Some(arena_identity(CacheObject::Edge(id))));
}

#[test]
fn repeated_kink_report_keeps_original_parameter_and_does_not_edit_knots() {
    let c = bspline(
        &[(0., 0.), (0.5, 0.5), (1., 0.)],
        1,
        vec![0., 0., 0.5, 1., 1.],
    );
    let (mut b, id) = fixture(c);
    let mut k = Kernel::default();
    let mut reports = AssertArray::new();
    assert!(!assert_edge_curve(
        &mut k,
        &mut b,
        id,
        AssertValidOptions::default(),
        Some(&mut reports)
    )
    .unwrap());
    assert_eq!(reports.reports()[0].owner_param.x, 0.5);
    let CurveKind::BSpline(c) = b.edge_mut(id).unwrap().curve.as_mut().unwrap().kind_mut() else {
        panic!()
    };
    c.get_or_create_gw_nurb().unwrap().knots_mut()[2] = 0.625;
    assert!(!assert_edge_curve(
        &mut k,
        &mut b,
        id,
        AssertValidOptions::default(),
        Some(&mut reports)
    )
    .unwrap());
    assert_eq!(reports.len(), 1);
    assert_eq!(reports.reports()[0].owner_param.x, 0.5);
    let CurveKind::BSpline(c) = b.edge(id).unwrap().curve.as_ref().unwrap().kind() else {
        panic!()
    };
    assert_eq!(c.gw_nurb().unwrap().knots()[2], 0.625);
}
