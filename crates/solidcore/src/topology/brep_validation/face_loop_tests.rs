// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::topology::{
    edgeuse::{p1 as eu, p3 as eu3, side},
    LoopuseStart,
};

fn options() -> AssertValidOptions<'static> {
    AssertValidOptions {
        level: CheckLevel::Given(&[4, 12, 13]),
        walk: false,
    }
}

#[test]
fn use_walk_flag_is_ignored_but_root_required_content_still_has_its_own_gate() {
    let (mut b, face, _, _) = tests::fixture();
    let fu = b.face(face).unwrap().faceuse.unwrap();
    assert!(assert_faceuse(
        &mut Kernel::default(),
        &mut b,
        fu,
        AssertValidOptions::default(),
        None
    )
    .unwrap());
    b.faceuse_mut(fu).unwrap().loopuses.clear();
    // Empty typed list means head=NULL AND count=0. Native Faceuse has no
    // separate required-content rule; the root presence preflight owns it.
    assert!(assert_faceuse(
        &mut Kernel::default(),
        &mut b,
        fu,
        AssertValidOptions::default(),
        None
    )
    .unwrap());
    let mut reports = AssertArray::new();
    assert!(required_links(&b, Some(&mut reports)).is_err());
    assert_eq!(reports.reports()[0].test_index, 126);
}

#[test]
fn paired_loopuse_start_rotation_is_legal_but_reversing_its_order_is_not() {
    let (mut b, _, loop_, _) = tests::fixture();
    let lu = b.loop_(loop_).unwrap().loopuse.unwrap();
    let mate = b.loopuse(lu).unwrap().mate.unwrap();
    let LoopuseStart::Edgeuse(Some(start)) = b.loopuse(mate).unwrap().start else {
        panic!()
    };
    b.loopuse_mut(mate).unwrap().start = LoopuseStart::Edgeuse(b.edgeuse(start).unwrap().ccw);
    let mut k = Kernel::default();
    assert!(assert_loop(&mut k, &mut b, loop_, options(), None).unwrap());
    for id in eu::loopuse_edgeuses(&b, mate).unwrap() {
        let record = b.edgeuse_mut(id).unwrap();
        std::mem::swap(&mut record.ccw, &mut record.cw);
    }
    let mut reports = AssertArray::new();
    assert!(!assert_loop(&mut k, &mut b, loop_, options(), Some(&mut reports)).unwrap());
    assert!(reports
        .reports()
        .iter()
        .any(|r| r.reporting_type == labels::LOOP_TYPE && r.test_index == 6));
}

#[test]
fn stored_uv_changes_the_native_missing_uv_periodic_side_allowance() {
    let (mut b, _, _, _) = tests::fixture();
    let first = b.edge(b.edge_list()[0]).unwrap().edgeuses[0];
    assert!(eu::uv_trim_curve(&b, first).unwrap().0.is_none());
    let mate = eu::mate(&b, first).unwrap().unwrap();
    assert_eq!(b.edgeuse(first).unwrap().orientation, OrientType::Same);
    let k = Kernel::default();
    let ordinary = side::edgeuse_side(&k, &b, mate, side::SideOptions::default()).unwrap();
    assert!(!ordinary.inside);
    assert!(ordinary.on_boundary);
    let known_periodic = side::SideOptions {
        periodic_v: Some(true),
        ..side::SideOptions::default()
    };
    assert!(
        side::edgeuse_side(&k, &b, mate, known_periodic)
            .unwrap()
            .inside
    );
    eu3::get_or_create_uv_trim_curve(&k, &mut b, first, false, false, true)
        .unwrap()
        .status
        .unwrap();
    assert!(
        !side::edgeuse_side(&k, &b, mate, known_periodic)
            .unwrap()
            .inside
    );
}

#[test]
fn face_request_guards_do_not_replace_missing_dynamic_surface_assertions() {
    let (mut b, face, _, _) = tests::fixture();
    assert!(assert_face(&mut Kernel::default(), &mut b, face, options(), None).unwrap());
    let (mut b, face, _, _) = tests::fixture();
    assert!(matches!(
        assert_face(
            &mut Kernel::default(),
            &mut b,
            face,
            AssertValidOptions {
                walk: false,
                ..AssertValidOptions::default()
            },
            None
        ),
        Err(SolidError::Unported {
            upstream: "SmSurface::AssertValid dynamic dispatch",
            ..
        })
    ));
}

#[test]
fn face_domain_error_keeps_the_original_rule_identity_and_does_not_repair_data() {
    let (mut b, face, _, _) = tests::fixture();
    let domain = crate::geom::extent2d::Extent2d::from_bounds(-0.1, -0.1, 1.1, 1.1);
    b.face_mut(face).unwrap().uv_domain = domain;
    let mut reports = AssertArray::new();
    assert!(!assert_face(
        &mut Kernel::default(),
        &mut b,
        face,
        options(),
        Some(&mut reports)
    )
    .unwrap());
    assert!(reports
        .reports()
        .iter()
        .any(|r| r.reporting_type == labels::FACE_TYPE && r.test_index == 2));
    assert!(reports
        .reports()
        .iter()
        .any(|r| r.reporting_type == labels::FACE_TYPE && r.test_index == 23));
    assert_eq!(b.face(face).unwrap().uv_domain, domain);
}

#[test]
fn owner_tolerance_uses_the_geometry_namespace_and_never_defaults_a_foreign_key() {
    let (mut b, face, _, _) = tests::fixture();
    b.face_mut(face).unwrap().tolerance = 0.002;
    let owner = crate::topology::brep_cache::arena_identity(
        crate::topology::brep_cache::CacheObject::Face(face),
    );
    assert_eq!(
        crate::topology::surface_cache::owning_face_tolerance(&b, owner).unwrap(),
        0.002
    );
    let metadata =
        crate::topology::brep::p2::object_ref(&b, crate::error::EntityRef::Face(face)).unwrap();
    assert!(matches!(
        crate::topology::surface_cache::owning_face_tolerance(&b, metadata),
        Err(SolidError::Unported { .. })
    ));
}

#[test]
fn zero_width_face_domain_never_yields_an_accepted_vertex_classification_result() {
    for level in [
        CheckLevel::Given(&[12, 13][..]),
        CheckLevel::Given(&[4, 12, 13][..]),
    ] {
        let (mut b, face, _, _) = tests::fixture();
        let domain = crate::geom::extent2d::Extent2d::from_bounds(0., 0., 0., 1.);
        b.face_mut(face).unwrap().uv_domain = domain;
        let result = assert_face(
            &mut Kernel::default(),
            &mut b,
            face,
            AssertValidOptions { level, walk: false },
            None,
        );
        assert!(!matches!(result, Ok(true)));
        assert_eq!(b.face(face).unwrap().uv_domain, domain);
    }
}
