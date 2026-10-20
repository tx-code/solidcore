// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::curve::{curve::Curve, line::Line};
use crate::geom::extent2d::Extent2d;
use crate::math::{Point3, Vec2, Vec3};
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::{plane::Plane, surface::Surface};
use crate::topology::brep_constructor::{BrepConstructor, VertexNode};
use crate::topology::{LoopuseStart, VertexuseOwner};

fn fixture() -> (Brep, FaceId, LoopId, EdgeuseId) {
    let mut k = Kernel::default();
    let mut builder = BrepConstructor::new().unwrap();
    builder.start_shell().unwrap();
    let f = builder.start_face(OrientType::Same).unwrap();
    let dom = Extent2d::from_bounds(0., 0., 1., 1.);
    builder
        .set_face_surface(
            Surface::from(Plane::new(
                Point3::ZERO,
                Vec3::X,
                Vec3::Y,
                Vec2::new(1., 1.),
                &dom,
            )),
            dom,
        )
        .unwrap();
    let l = builder.start_loop(OrientType::Same).unwrap();
    let p = [Point3::ZERO, Vec3::X, Vec3::Y];
    for i in 0..3 {
        builder
            .start_edge(&k, OrientType::Same, None, None, None)
            .unwrap();
        builder
            .set_edge_curve(
                &k,
                Curve::from(Line::from_points(p[i], p[(i + 1) % 3], 3)),
                false,
            )
            .unwrap();
        builder
            .set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))
            .unwrap();
        builder.end_edge().unwrap();
    }
    builder.end_loop().unwrap();
    builder.end_face().unwrap();
    builder.end_shell(&mut k, false).unwrap();
    let b = builder.finish().unwrap();
    let lu = b.loop_(l).unwrap().loopuse.unwrap();
    let LoopuseStart::Edgeuse(Some(start)) = b.loopuse(lu).unwrap().start else {
        panic!()
    };
    (b, f, l, start)
}

#[test]
fn completed_pointer_walk_keeps_success_status_even_when_reports_exist() {
    let (mut b, f, _, _) = fixture();
    required_links(&b, None).unwrap();
    let fu = b.face(f).unwrap().faceuse.unwrap();
    b.faceuse_mut(fu).unwrap().orientation = OrientType::Opposite;
    let mut reports = AssertArray::new();
    validate_pointers(
        &mut Kernel::default(),
        &mut b,
        ValidationOptions::default(),
        Some(&mut reports),
    )
    .unwrap();
    assert!(reports.reports().iter().any(|r| r.test_index == 14));
    assert!(reports.reports().iter().any(|r| r.test_index == 17));
}

#[test]
fn release_given_requests_do_not_skip_boolean_geometry_oriented_checks() {
    let (mut b, f, _, _) = fixture();
    let fu = b.face(f).unwrap().faceuse.unwrap();
    b.faceuse_mut(fu).unwrap().orientation = OrientType::Opposite;
    let mut reports = AssertArray::new();
    validate_pointers(
        &mut Kernel::default(),
        &mut b,
        ValidationOptions {
            level: CheckLevel::Given(&[0]),
            walk: false,
        },
        Some(&mut reports),
    )
    .unwrap();
    assert!(reports.reports().iter().any(|r| r.test_index == 14));
    let first = b.edge(b.edge_list()[0]).unwrap().curve.as_ref().unwrap();
    let crate::curve::curve::CurveKind::Line(line) = first.kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_some());
}

#[test]
fn loop_face_checks_follow_primary_use_incidence_not_the_convenience_cache() {
    let (mut b, _, l, _) = fixture();
    b.loop_mut(l).unwrap().face = FaceId::from_index(u32::MAX);
    let mut reports = AssertArray::new();
    validate_pointers(
        &mut Kernel::default(),
        &mut b,
        ValidationOptions::default(),
        Some(&mut reports),
    )
    .unwrap();
    assert!(reports.is_empty());
}
#[test]
fn absent_infinite_region_stops_before_list_getters() {
    let mut reports = AssertArray::new();
    let error = required_links(&Brep::new(), Some(&mut reports)).unwrap_err();
    assert!(matches!(error, SolidError::Upstream { code: 1001, .. }));
    assert_eq!(
        reports
            .reports()
            .iter()
            .map(|r| r.test_index)
            .collect::<Vec<_>>(),
        vec![113]
    );
}
#[test]
fn shell_faceuse_prefix_precedes_global_face_surface_check() {
    let (mut b, f, _, _) = fixture();
    let fu = b.face(f).unwrap().faceuse.unwrap();
    b.face_mut(f).unwrap().surface = None;
    b.faceuse_mut(fu).unwrap().mate = None;
    let mut reports = AssertArray::new();
    assert!(required_links(&b, Some(&mut reports)).is_err());
    assert_eq!(
        reports
            .reports()
            .iter()
            .map(|r| r.test_index)
            .collect::<Vec<_>>(),
        vec![124]
    );
    assert_eq!(reports.reports()[0].owner, Some(Object::Faceuse(fu).key()));
}
#[test]
fn missing_vertex_owner_preserves_original_report_identity() {
    let (mut b, _, _, _) = fixture();
    let vertex = b.vertex_list()[0];
    let vu = b.vertex(vertex).unwrap().vertexuses[0];
    b.vertexuse_mut(vu).unwrap().owner = VertexuseOwner::Edgeuse(None);
    let mut reports = AssertArray::new();
    assert!(required_links(&b, Some(&mut reports)).is_err());
    let r = &reports.reports()[0];
    assert_eq!(
        (r.reporting_type, r.list_index, r.test_index),
        (labels::BREP_TYPE, 1, 145)
    );
    assert_eq!(r.owner, Some(Object::Vertexuse(vu).key()));
    assert_eq!(r.name, "Null");
}
#[test]
fn nonclosing_ccw_ring_is_bounded_and_attributed_to_the_loopuse() {
    let (mut b, _, l, start) = fixture();
    let next = b.edgeuse(start).unwrap().ccw.unwrap();
    b.edgeuse_mut(next).unwrap().ccw = Some(next);
    let mut reports = AssertArray::new();
    assert!(required_links(&b, Some(&mut reports)).is_err());
    assert_eq!(
        reports
            .reports()
            .iter()
            .map(|r| r.test_index)
            .collect::<Vec<_>>(),
        vec![146]
    );
    assert_eq!(
        reports.reports()[0].owner,
        Some(Object::Loopuse(b.loop_(l).unwrap().loopuse.unwrap()).key())
    );
}
#[test]
fn repeated_failure_uses_existing_assert_array_deduplication() {
    let (mut b, f, _, _) = fixture();
    b.face_mut(f).unwrap().surface = None;
    let mut reports = AssertArray::new();
    assert!(required_links(&b, Some(&mut reports)).is_err());
    assert!(reports.last_add());
    assert!(required_links(&b, Some(&mut reports)).is_err());
    assert_eq!(reports.len(), 1);
    assert!(!reports.last_add());
}
#[test]
fn selected_checks_cannot_skip_the_forced_level_zero_prefix() {
    let (mut b, f, _, _) = fixture();
    b.face_mut(f).unwrap().surface = None;
    let mut reports = AssertArray::new();
    let error = validate_pointers(
        &mut Kernel::default(),
        &mut b,
        ValidationOptions {
            level: CheckLevel::Given(&[]),
            walk: true,
        },
        Some(&mut reports),
    )
    .unwrap_err();
    assert!(matches!(error, SolidError::Upstream { code: 1001, .. }));
    assert_eq!(reports.reports()[0].test_index, 121);
}
#[test]
fn dangling_arena_handle_is_an_access_error_not_an_invented_source_assertion() {
    let (mut b, f, _, _) = fixture();
    let fu = b.face(f).unwrap().faceuse.unwrap();
    b.faceuse_mut(fu).unwrap().mate = Some(FaceuseId::from_index(u32::MAX));
    let mut reports = AssertArray::new();
    assert!(matches!(
        required_links(&b, Some(&mut reports)),
        Err(SolidError::NotFound { .. })
    ));
    assert!(reports.is_empty());
}
