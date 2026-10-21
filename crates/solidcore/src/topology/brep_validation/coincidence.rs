// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Root's LEVEL2 first-coincidence check, in native traversal/pair order.
//!
//! Upstream: maintained SmBrep.cpp30178--30625, sm_FindCoinFace14885--14991,
//! SmTools.cpp122--255. This is the root's default bFindAll=FALSE call, not the
//! separate public all-witness/independent output-array interface.
//! Reports never determine coincidence truth. Missing implementations propagate.
//! Deviations: native output parameters become a private first witness; only
//! `first` translates that witness to the native report pair. A failed peer
//! surface evaluation currently terminates its receiver search instead of
//! continuing with native's retained output point; malformed-surface failure
//! parity remains unqualified. Edge/face classification is still explicit Unported.

use super::{bp, Brep, Object, Reporter};
use crate::error::{ignored, EntityRef, Result, SolidError};
use crate::geom::extent2d::Extent2d;
use crate::geom::extent3d::Extent3d;
use crate::geom::global_solver::SolverOperation;
use crate::geom::solution_array::SolutionType;
use crate::intersect::point_class::{PointClassType, PointClassification};
use crate::kernel::Kernel;
use crate::nurbs::b_spline_curve::p2::BoundingBoxRequest;
use crate::query::{
    edge as qe,
    face::{p1 as qf, p2 as qf2},
};
use crate::surface::surface::p5::DropPointOptions;
use crate::topology::{edge, loopuse, EdgeId, FaceId, VertexId};

// Pair kinds carry the topology types used by the native SM_LIST_3 reports.
// Geometry traversal does not consult the optional report sink, and an
// unimplemented edge/face classifier cannot construct a successful witness.
enum Witness {
    Vertices(VertexId, VertexId, f64),
    Edges(EdgeId, EdgeId, f64),
    Faces(FaceId, FaceId, f64),
    VertexEdge(VertexId, EdgeId, f64),
    VertexFace(VertexId, FaceId, f64),
}

pub(super) fn first(k: &Kernel, b: &mut Brep, report: &mut Reporter<'_>) -> Result<bool> {
    let tolerance = b.tolerance();
    let Some(witness) = find_first(k, b, tolerance)? else {
        return Ok(false);
    };
    let (test, owner, other, deviation) = match witness {
        Witness::Vertices(a, b, deviation) => (0, Object::Vertex(a), Object::Vertex(b), deviation),
        Witness::Edges(a, b, deviation) => (1, Object::Edge(a), Object::Edge(b), deviation),
        Witness::Faces(a, b, deviation) => (2, Object::Face(a), Object::Face(b), deviation),
        Witness::VertexEdge(a, b, deviation) => (3, Object::Vertex(a), Object::Edge(b), deviation),
        Witness::VertexFace(a, b, deviation) => (4, Object::Vertex(a), Object::Face(b), deviation),
    };
    report.coincidence_pair(test, owner, other, tolerance, deviation)?;
    Ok(true)
}

fn invalid() -> SolidError {
    SolidError::invalid_input(
        "topology::brep_validation::coincidence",
        "missing required topology/geometry",
    )
}
fn edge_vertices(b: &Brep, e: EdgeId) -> Result<(Option<VertexId>, Option<VertexId>)> {
    let a = edge::vertex(b, e)?;
    Ok((a, edge::other_vertex(b, e, a)?))
}
fn outer_edges(b: &Brep, f: FaceId) -> Result<Vec<EdgeId>> {
    let l = *b.face(f)?.loops.first().ok_or_else(invalid)?;
    let lu = b.loop_(l)?.loopuse.ok_or_else(invalid)?;
    loopuse::edges(b, lu, None)?
        .into_iter()
        .map(|e| e.ok_or_else(invalid))
        .collect()
}
fn trim_domain(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<Extent2d> {
    Ok(ignored(
        crate::topology::face::p1::calculate_uv_domain_from_uv_trim_curves(
            k, b, f, true, false, false,
        ),
    )?
    .map_or(b.face(f)?.uv_domain, |v| v.domain))
}

fn coincident_edge(
    k: &Kernel,
    b: &Brep,
    edges: &[EdgeId],
    index: usize,
    tol: f64,
) -> Result<Option<(EdgeId, f64)>> {
    let id = *edges.get(index).ok_or_else(invalid)?;
    let vertices = edge_vertices(b, id)?;
    let target = b.edge(id)?.curve.as_ref().ok_or_else(invalid)?;
    let interval = b.edge(id)?.interval;
    for other in edges.iter().skip(index + 1) {
        let ends = edge_vertices(b, *other)?;
        if ends != vertices && ends != (vertices.1, vertices.0) {
            continue;
        }
        let edge = b.edge(*other)?;
        let curve = edge.curve.as_ref().ok_or_else(invalid)?;
        let tolerance = tol + edge.tolerance;
        let middle = curve.evaluate_point(edge.interval.evaluate(0.5))?;
        let drop = target.drop_point(
            k,
            &interval,
            middle,
            None,
            tolerance,
            Some(interval.mid()),
            SolverOperation::Minimize,
        )?;
        if !drop.success || drop.distance > tolerance {
            continue;
        }
        let c = curve.evaluate(edge.interval.evaluate(0.5), 1, true, true)?;
        let a = target.evaluate(drop.parameter, 1, true, true)?;
        let same = a
            .get(1)
            .ok_or_else(invalid)?
            .dot(*c.get(1).ok_or_else(invalid)?)
            > 0.;
        let (start, end) = if same {
            (edge.interval.min(), edge.interval.max())
        } else {
            (edge.interval.max(), edge.interval.min())
        };
        let Some(gap) = ignored(target.curve_max_distance_between(
            k,
            &interval,
            curve,
            start,
            end,
            30,
            Some(tolerance),
        ))?
        else {
            continue;
        };
        if gap.distance <= tolerance {
            return Ok(Some((*other, gap.distance)));
        }
    }
    Ok(None)
}

fn coincident_face(
    k: &Kernel,
    b: &mut Brep,
    faces: &[FaceId],
    index: usize,
    boxes: &[Option<Extent3d>],
    tol: f64,
) -> Result<Option<(FaceId, f64)>> {
    let face = *faces.get(index).ok_or_else(invalid)?;
    let edges = outer_edges(b, face)?;
    // Native recomputes the receiver box even though it precomputed all peers.
    let this_box = ignored(qf::calculate_bounding_box(k, b, face, false))?;
    for (j, other) in faces.iter().enumerate().skip(index + 1) {
        let tolerance = tol + b.face(*other)?.tolerance;
        if let (Some(a), Some(c)) = (this_box, boxes.get(j).copied().flatten()) {
            if a.are_disjoint(&c, tolerance) {
                continue;
            }
        }
        if b.face(*other)?.loops.is_empty() {
            continue;
        }
        let candidates = outer_edges(b, *other)?;
        if candidates.len() != edges.len()
            || edges.iter().filter(|e| candidates.contains(e)).count() != edges.len()
        {
            continue;
        }
        let domain = trim_domain(k, b, *other)?;
        let point = b
            .face(*other)?
            .surface
            .as_ref()
            .ok_or_else(invalid)?
            .evaluate_point(domain.evaluate(0.5, 0.5))?;
        let domain = trim_domain(k, b, face)?;
        let Some(drop) = ignored(crate::topology::surface_cache::drop_face_point(
            k,
            b,
            face,
            point,
            &domain,
            &DropPointOptions {
                guess_uv: Some(domain.evaluate(0.5, 0.5)),
                ..Default::default()
            },
        ))?
        else {
            continue;
        };
        if drop.success && drop.gap < tolerance && domain.contains_point(drop.result_uv, 0.) {
            return Ok(Some((*other, drop.gap)));
        }
    }
    Ok(None)
}

fn find_first(k: &Kernel, b: &mut Brep, tolerance: f64) -> Result<Option<Witness>> {
    let vertices = bp::vertices(b, None)?;
    for (i, a) in vertices.iter().enumerate() {
        let point = b.vertex(*a)?.point;
        for c in vertices.iter().skip(i + 1) {
            let other = b.vertex(*c)?.point;
            // CloserThan's source comment says strict; its executable predicate
            // accepts equality (SmVector3d.h999--1018). Preserve that boundary.
            if point.closer_than(tolerance, other) {
                return Ok(Some(Witness::Vertices(*a, *c, (point - other).length())));
            }
        }
    }
    let edges = bp::edges(b, None)?;
    for (i, id) in edges.iter().enumerate() {
        if let Some((other, dev)) = ignored(coincident_edge(k, b, &edges, i, tolerance))?.flatten()
        {
            if dev < tolerance {
                return Ok(Some(Witness::Edges(*id, other, dev)));
            }
        }
    }
    let faces = bp::faces(b, None)?;
    let mut boxes = Vec::new();
    for f in &faces {
        boxes.push(ignored(qf::calculate_bounding_box(k, b, *f, false))?);
    }
    for (i, id) in faces.iter().enumerate() {
        if let Some((other, dev)) =
            ignored(coincident_face(k, b, &faces, i, &boxes, tolerance))?.flatten()
        {
            if dev < tolerance {
                return Ok(Some(Witness::Faces(*id, other, dev)));
            }
        }
    }
    let mut edge_boxes = Vec::new();
    for e in &edges {
        let bbox = ignored(qe::calculate_bounding_box(
            k,
            b,
            *e,
            BoundingBoxRequest {
                normal: true,
                pseudo: false,
                polar: false,
            },
            false,
        ))?
        .and_then(|v| v.normal)
        .unwrap_or_default();
        edge_boxes.push(bbox);
        let rec = b.edge(*e)?;
        let Some(curve) = rec.curve.as_ref() else {
            continue;
        };
        let ends = edge_vertices(b, *e)?;
        for v in &vertices {
            if ends.0 == Some(*v) || ends.1 == Some(*v) {
                continue;
            }
            let point = b.vertex(*v)?.point;
            if !bbox.contains_point(point, tolerance) {
                continue;
            }
            let Some(drop) = ignored(curve.drop_point(
                k,
                &rec.interval,
                point,
                None,
                tolerance,
                None,
                SolverOperation::Intersect,
            ))?
            else {
                continue;
            };
            if !drop.success || drop.distance > tolerance {
                continue;
            }
            return Ok(Some(Witness::VertexEdge(*v, *e, drop.distance)));
        }
    }
    for f in &faces {
        let bbox = ignored(qf::calculate_bounding_box(k, b, *f, false))?.unwrap_or_default();
        let domain = b.face(*f)?.uv_domain;
        let face_vertices = qf2::face_vertices(b, *f, false, None)?;
        let face_edges = crate::topology::topology_traverser::face_edges(b, *f)?;
        let mut pc = PointClassification::<EntityRef>::new(b.face(*f)?.tolerance);
        for v in &vertices {
            if face_vertices.contains(v) {
                continue;
            }
            let vertex = b.vertex(*v)?;
            let (point, src_tol) = (vertex.point, vertex.tolerance);
            if !bbox.contains_point(point, tolerance) {
                continue;
            }
            let Some(drop) = ignored(crate::topology::surface_cache::drop_face_point(
                k,
                b,
                *f,
                point,
                &domain,
                &DropPointOptions::default(),
            ))?
            else {
                continue;
            };
            if !drop.success || drop.gap > tolerance {
                continue;
            }
            if ignored(qf2::point_classify(
                k,
                b,
                *f,
                drop.result_uv,
                &mut pc,
                qf2::PointClassifyOptions {
                    src_zone_tol_3d: src_tol,
                    do_boundary_intersections: false,
                    use_uv_classification: true,
                    ..Default::default()
                },
            ))?
            .is_none()
            {
                continue;
            }
            if matches!(
                pc.point_class(),
                PointClassType::Face | PointClassType::Edge | PointClassType::Vertex
            ) {
                return Ok(Some(Witness::VertexFace(*v, *f, drop.gap)));
            }
        }
        for (j, e) in edges.iter().enumerate() {
            if face_edges.contains(e) {
                continue;
            }
            let edge_box = edge_boxes.get(j).ok_or_else(invalid)?;
            if bbox.are_disjoint(edge_box, tolerance) {
                continue;
            }
            let rec = b.edge(*e)?;
            rec.curve.as_ref().ok_or_else(invalid)?;
            b.face(*f)?.surface.as_ref().ok_or_else(invalid)?;
            let interval = rec.interval;
            let Some(solution) = ignored(
                crate::topology::surface_cache::simple_face_edge_coincidence(
                    k,
                    b,
                    *f,
                    *e,
                    &domain,
                    &interval,
                    tolerance,
                    true,
                    [None, None],
                ),
            )?
            .flatten() else {
                continue;
            };
            if solution.solution_type != SolutionType::RangeOfValues
                || solution.start.solution_value > tolerance
                || solution.end.solution_value > tolerance
            {
                continue;
            }
            // The actual next dependency is CurveOnClassify with this edge's
            // owning geometry/face handles. Do not manufacture interval classes.
            return Err(SolidError::unported(
                "SmFace::CurveOnClassify for coincidence edge/face",
                "query.face_p2",
            ));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::assert_array::AssertArray;
    use crate::math::Point3;

    #[test]
    fn first_pair_order_and_truth_do_not_depend_on_the_report_sink() {
        let (mut b, _, _, _) = super::super::tests::fixture();
        let vertices = bp::vertices(&b, None).unwrap();
        for vertex in &vertices {
            b.vertex_mut(*vertex).unwrap().point = Point3::ZERO;
        }
        // Native returns on the first vertex pair before visiting any curves.
        // This also guards against collecting every pair before reporting one.
        let edge = b.edge_list()[0];
        b.edge_mut(edge).unwrap().curve = None;
        let k = Kernel::default();
        assert!(first(&k, &mut b, &mut Reporter { reports: None }).unwrap());
        let mut reports = AssertArray::new();
        assert!(first(
            &k,
            &mut b,
            &mut Reporter {
                reports: Some(&mut reports),
            },
        )
        .unwrap());
        let pair = reports.reports();
        assert_eq!(pair.len(), 1);
        assert_eq!((pair[0].list_index, pair[0].test_index), (3, 0));
        assert_eq!(pair[0].owner, Some(Object::Vertex(vertices[0]).key()));
        assert_eq!(pair[0].other, Some(Object::Vertex(vertices[1]).key()));
        assert_eq!(pair[0].value, 0.);
        assert_eq!(pair[0].tol3d, b.tolerance());
    }

    #[test]
    fn vertex_coincidence_accepts_the_native_equal_tolerance_boundary() {
        let (mut b, _, _, _) = super::super::tests::fixture();
        let vertices = bp::vertices(&b, None).unwrap();
        let tolerance = 0.125;
        b.set_tolerance(tolerance);
        b.vertex_mut(vertices[0]).unwrap().point = Point3::ZERO;
        b.vertex_mut(vertices[1]).unwrap().point = Point3::new(tolerance, 0., 0.);
        let mut reports = AssertArray::new();
        assert!(first(
            &Kernel::default(),
            &mut b,
            &mut Reporter {
                reports: Some(&mut reports),
            },
        )
        .unwrap());
        let pair = reports.reports();
        assert_eq!(pair.len(), 1);
        assert_eq!(pair[0].value, tolerance);
        assert_eq!(pair[0].tol3d, tolerance);
    }
}
