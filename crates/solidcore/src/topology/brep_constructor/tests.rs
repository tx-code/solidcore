// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::curve::{
    curve::{Curve, CurveKind},
    line::Line,
};
use crate::geom::extent2d::Extent2d;
use crate::math::Vec3;
use crate::surface::{plane::Plane, surface::Surface};
use crate::topology::brep_cache::{arena_identity, CacheObject};

fn one_edge() -> (Kernel, BrepConstructor, FaceId, EdgeId) {
    let k = Kernel::default();
    let mut builder = BrepConstructor::new().unwrap();
    builder.start_shell().unwrap();
    let face = builder.start_face(OrientType::Same).unwrap();
    let domain = Extent2d::from_bounds(0., 0., 2., 3.);
    let plane = Surface::from(Plane::from_normal(Point3::ZERO, Vec3::Z));
    builder.set_face_surface(plane, domain).unwrap();
    builder.start_loop(OrientType::Same).unwrap();
    let edge = builder
        .start_edge(&k, OrientType::Same, None, None, None)
        .unwrap();
    (k, builder, face, edge)
}

#[test]
fn finish_requires_closed_scopes_and_does_not_return_a_partial_brep() {
    let mut builder = BrepConstructor::new().unwrap();
    builder.start_shell().unwrap();
    assert!(builder.finish().is_err());
    let fresh = BrepConstructor::new().unwrap().finish().unwrap();
    assert_eq!(fresh.regions().len(), 1);
    assert_eq!(fresh.shells().len(), 0);
}

#[test]
fn explicit_parameters_are_not_projected_clamped_or_snapped() {
    let (k, mut b, _, edge) = one_edge();
    b.set_edge_curve(
        &k,
        Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3)),
        false,
    )
    .unwrap();
    b.set_limits(&k, VertexNode::Parameter(-2.), VertexNode::Parameter(3.))
        .unwrap();
    let before = b.brep().edge(edge).unwrap().interval;
    assert_eq!((before.min(), before.max()), (-2., 3.));
    assert!(b
        .set_limits(&k, VertexNode::Parameter(4.), VertexNode::Parameter(1.))
        .is_err());
    assert_eq!(b.brep().edge(edge).unwrap().interval, before);
}

#[test]
fn duplicate_definition_does_not_replace_geometry_or_its_owner() {
    let (k, mut b, _, edge) = one_edge();
    b.set_edge_curve(
        &k,
        Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3)),
        false,
    )
    .unwrap();
    assert!(b
        .set_edge_curve(
            &k,
            Curve::from(Line::from_points(Vec3::Y, Vec3::Z, 3)),
            false
        )
        .is_err());
    let c = b.brep().edge(edge).unwrap().curve.as_ref().unwrap();
    assert_eq!(c.evaluate_point(&k, 0.).unwrap(), Point3::ZERO);
    assert_eq!(c.evaluate_point(&k, 1.).unwrap(), Vec3::X);
    assert_eq!(c.owner(), Some(arena_identity(CacheObject::Edge(edge))));
}

#[test]
fn uv_dimension_error_retains_the_face_surface_and_unpublished_candidate() {
    let (k, mut b, face, edge) = one_edge();
    let bad = Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3));
    assert!(b.set_edge_curve(&k, bad, true).is_err());
    let surface = b.brep().face(face).unwrap().surface.as_ref().unwrap();
    assert_eq!(
        surface.owner(),
        Some(arena_identity(CacheObject::Face(face)))
    );
    assert!(b.brep().edge(edge).unwrap().curve.is_none());
    assert!(b.finish().is_err());
}

#[test]
fn failed_loop_without_geometry_stays_unpublished() {
    let (k, mut b, _, _) = one_edge();
    assert!(b.end_loop(&k).is_err());
    assert!(b.finish().is_err());
}

#[test]
fn vertex_loop_has_two_distinct_uses_of_one_vertex_and_two_loop_sides() {
    let mut b = BrepConstructor::new().unwrap();
    let mut k = Kernel::default();
    b.start_shell().unwrap();
    b.start_face(OrientType::Same).unwrap();
    let loop_id = b.single_vertex_loop(Vec3::new(1., 2., 3.)).unwrap();
    let a = b.brep().loop_(loop_id).unwrap().loopuse.unwrap();
    let mate = b.brep().loopuse(a).unwrap().mate.unwrap();
    assert_ne!(a, mate);
    let LoopuseStart::Vertexuse(Some(va)) = b.brep().loopuse(a).unwrap().start else {
        panic!()
    };
    let LoopuseStart::Vertexuse(Some(vb)) = b.brep().loopuse(mate).unwrap().start else {
        panic!()
    };
    assert_ne!(va, vb);
    assert_eq!(
        b.brep().vertexuse(va).unwrap().vertex,
        b.brep().vertexuse(vb).unwrap().vertex
    );
    b.end_face().unwrap();
    b.end_shell(&mut k, false).unwrap();
    assert_eq!(b.finish().unwrap().vertices().len(), 1);
}

#[test]
fn coincident_coordinates_do_not_weld_source_vertex_identities() {
    let mut b = BrepConstructor::new().unwrap();
    let a = b.start_vertex(Point3::ZERO).unwrap();
    b.end_vertex();
    let second = b.start_vertex_of_loop(Point3::ZERO).unwrap();
    assert_ne!(a, second);
    assert_eq!(b.brep().vertices().len(), 2);
}

#[test]
fn region_hint_is_not_material_classification_and_swap_only_routes_future_shells() {
    let mut b = BrepConstructor::new().unwrap();
    let region = b.start_region(false, None).unwrap();
    assert_eq!(b.brep().region(region).unwrap().is_void, 0);
    b.swap_regions();
    let (outer, inner) = b.start_shell().unwrap();
    assert_eq!(b.brep().shell(outer).unwrap().region, Some(region));
    assert!(inner.is_none());
}

#[test]
fn pre_stitch_delegates_to_actual_pointer_and_geometry_validation() {
    let (mut k, mut b, _, first) = one_edge();
    b.set_edge_curve(
        &k,
        Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3)),
        false,
    )
    .unwrap();
    b.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))
        .unwrap();
    b.end_edge().unwrap();
    for (a, end) in [(Vec3::X, Vec3::Y), (Vec3::Y, Point3::ZERO)] {
        b.start_edge(&k, OrientType::Same, None, None, None)
            .unwrap();
        b.set_edge_curve(&k, Curve::from(Line::from_points(a, end, 3)), false)
            .unwrap();
        b.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))
            .unwrap();
        b.end_edge().unwrap();
    }
    b.end_loop(&k).unwrap();
    b.end_face().unwrap();
    let CurveKind::Line(line) = b.brep().edge(first).unwrap().curve.as_ref().unwrap().kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_none());
    b.validate_before_stitch(&mut k).unwrap();
    let CurveKind::Line(line) = b.brep().edge(first).unwrap().curve.as_ref().unwrap().kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_some());
}

#[test]
fn radial_dependency_error_removes_the_temporary_edge_before_propagating() {
    use std::collections::BTreeMap;
    let k = Kernel::default();
    let mut b = BrepConstructor::new().unwrap();
    b.start_shell().unwrap();
    let points = [
        Point3::ZERO,
        Vec3::new(2., 0., 0.),
        Vec3::new(2., 3., 0.),
        Vec3::new(0., 3., 0.),
        Vec3::new(0., 0., 4.),
        Vec3::new(2., 0., 4.),
    ];
    let vertices: Vec<_> = points
        .iter()
        .map(|p| b.start_vertex_of_loop(*p).unwrap())
        .collect();
    let mut defined = BTreeMap::new();
    for polygon in [[0, 1, 2, 3], [1, 0, 4, 5]] {
        b.start_face(OrientType::Same).unwrap();
        let domain = Extent2d::from_bounds(0., 0., 1., 1.);
        let p = points[polygon[0]];
        let u = points[polygon[1]] - p;
        let v = points[polygon[3]] - p;
        b.set_face_surface(
            Surface::from(Plane::new(
                p,
                u,
                v,
                crate::math::Vec2::new(u.length(), v.length()),
                &domain,
            )),
            domain,
        )
        .unwrap();
        b.start_loop(OrientType::Same).unwrap();
        for i in 0..4 {
            let a = polygon[i];
            let end = polygon[(i + 1) % 4];
            let key = (a.min(end), a.max(end));
            let old = defined.get(&key).copied();
            let sense = if old.is_none_or(|(_, start)| a == start) {
                OrientType::Same
            } else {
                OrientType::Opposite
            };
            let edge = b
                .start_edge(
                    &k,
                    sense,
                    old.map(|(e, _)| e),
                    Some(vertices[a]),
                    Some(vertices[end]),
                )
                .unwrap();
            if old.is_none() {
                b.set_edge_curve(
                    &k,
                    Curve::from(Line::from_points(points[a], points[end], 3)),
                    false,
                )
                .unwrap();
                b.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))
                    .unwrap();
                defined.insert(key, (edge, a));
            }
            b.end_edge().unwrap();
        }
        b.end_loop(&k).unwrap();
        b.end_face().unwrap();
    }
    let shared = defined[&(0, 1)].0;
    assert_eq!(b.brep().edge(shared).unwrap().edgeuses.len(), 4);
    let before = b.brep().edges().len();
    b.start_face(OrientType::Same).unwrap();
    let domain = Extent2d::from_bounds(0., 0., 2., 3.);
    b.set_face_surface(
        Surface::from(Plane::from_normal(Point3::ZERO, Vec3::Z)),
        domain,
    )
    .unwrap();
    let loop_id = b.start_loop(OrientType::Same).unwrap();
    let error = b
        .start_edge(
            &k,
            OrientType::Opposite,
            Some(shared),
            Some(vertices[1]),
            Some(vertices[0]),
        )
        .unwrap_err();
    // Real DoTrace and analytic iso curves now run. A deeper owned point solve
    // still requires its original face-tolerance resolver; cleanup must hold.
    assert!(
        matches!(
            error,
            SolidError::Unported {
                upstream: "SmFace::GetTolerance",
                unit: "topology.face_p2"
            }
        ),
        "{error:?}"
    );
    assert_eq!(b.brep().edges().len(), before);
    assert_eq!(b.brep.edge_list.len(), before);
    assert_eq!(b.brep().edge(shared).unwrap().edgeuses.len(), 4);
    let lu = b.brep().loop_(loop_id).unwrap().loopuse.unwrap();
    let LoopuseStart::Edgeuse(Some(eu)) = b.brep().loopuse(lu).unwrap().start else {
        panic!()
    };
    assert!(b.brep().edgeuse(eu).unwrap().edge.is_none());
    assert!(b.finish().is_err());
}

#[test]
fn uv_only_iso_line_lifts_exactly_without_mutating_the_source_curve() {
    let k = Kernel::default();
    let mut b = BrepConstructor::new().unwrap();
    b.start_shell().unwrap();
    let face = b.start_face(OrientType::Same).unwrap();
    let domain = Extent2d::from_bounds(0., 0., 1., 1.);
    let plane = Plane::new(
        Vec3::new(10., 20., 30.),
        Vec3::X,
        Vec3::Y,
        crate::math::Vec2::new(2., 3.),
        &domain,
    );
    b.set_face_surface(Surface::from(plane), domain).unwrap();
    b.start_loop(OrientType::Same).unwrap();
    let edge = b
        .start_edge(&k, OrientType::Same, None, None, None)
        .unwrap();
    let uv = Curve::from(Line::from_points(
        Vec3::new(0.25, 0., 0.),
        Vec3::new(0.25, 1., 0.),
        2,
    ));
    b.set_edge_curve(&k, uv, true).unwrap();
    let curve = b.brep().edge(edge).unwrap().curve.as_ref().unwrap();
    assert_eq!(
        curve.evaluate_point(&k, 0.).unwrap(),
        Vec3::new(10.5, 20., 30.)
    );
    assert_eq!(
        curve.evaluate_point(&k, 0.5).unwrap(),
        Vec3::new(10.5, 21.5, 30.)
    );
    assert_eq!(
        curve.evaluate_point(&k, 1.).unwrap(),
        Vec3::new(10.5, 23., 30.)
    );
    let incidence = b.current_edgeuse().unwrap();
    let source = b
        .brep()
        .edgeuse(incidence)
        .unwrap()
        .uv_trim_curve
        .as_ref()
        .unwrap();
    let CurveKind::Line(line) = source.kind() else {
        panic!()
    };
    assert!(line.base().gw_nurb().is_none());
    assert_eq!(
        b.brep()
            .face(face)
            .unwrap()
            .surface
            .as_ref()
            .unwrap()
            .owner(),
        Some(arena_identity(CacheObject::Face(face)))
    );
}

#[test]
fn failed_non_iso_lift_restores_surface_ownership_before_returning() {
    let (k, mut b, face, edge) = one_edge();
    let uv = Curve::from(Line::from_points(Point3::ZERO, Vec3::new(1., 1., 0.), 2));
    let error = b.set_edge_curve(&k, uv, true).unwrap_err();
    assert!(matches!(
        error,
        SolidError::Unported {
            unit: "curve.crv_on_surf",
            ..
        }
    ));
    let surface = b.brep().face(face).unwrap().surface.as_ref().unwrap();
    assert_eq!(
        surface.owner(),
        Some(arena_identity(CacheObject::Face(face)))
    );
    assert!(b.brep().edge(edge).unwrap().curve.is_none());
    assert!(b.finish().is_err());
}

#[test]
fn shared_edge_does_not_resolve_the_unused_endpoint_ids() {
    let (k, mut b, _, edge) = one_edge();
    let unused = VertexId::from_index(u32::MAX);
    assert_eq!(
        b.start_edge(
            &k,
            OrientType::Opposite,
            Some(edge),
            Some(unused),
            Some(unused)
        )
        .unwrap(),
        edge
    );
    assert_eq!(b.brep().edge(edge).unwrap().edgeuses.len(), 4);
}
