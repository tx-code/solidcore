// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Synthetic required-link observer; successful output is prefix evidence only.
use serde_json::{json, Value};
use solidcore::curve::{curve::Curve, line::Line};
use solidcore::error::SolidError;
use solidcore::geom::assert_array::AssertArray;
use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec2, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::brep_cache::{arena_identity, CacheObject};
use solidcore::topology::brep_constructor::{BrepConstructor, VertexNode};
use solidcore::topology::brep_validation::required_links;
use solidcore::topology::shell::ShellMembers;
use solidcore::topology::{Edge, EdgeuseOwner, LoopuseStart, VertexuseOwner};
use std::error::Error;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn require<T>(value: Option<T>) -> Result<T> {
    value.ok_or_else(|| "missing fixture identity".into())
}
fn at<T>(values: &[T], index: usize) -> Result<&T> {
    values.get(index).ok_or_else(|| "synthetic index".into())
}
fn run() -> Result<Value> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let defect = args.first().ok_or("defect [selection] [second-defect]")?;
    let selection = args.get(1).map_or("zero", String::as_str);
    let second = args.get(2).map_or("none", String::as_str);
    let mut k = Kernel::default();
    let mut builder = BrepConstructor::new()?;
    let (outer, _) = builder.start_shell()?;
    let face = builder.start_face(OrientType::Same)?;
    let domain = Extent2d::from_bounds(0., 0., 1., 1.);
    builder.set_face_surface(
        Surface::from(Plane::new(
            Point3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec2::new(1., 1.),
            &domain,
        )),
        domain,
    )?;
    let loop_id = builder.start_loop(OrientType::Same)?;
    let points = [Point3::ZERO, Vec3::X, Vec3::Y];
    let vertices = points
        .iter()
        .map(|p| builder.start_vertex_of_loop(*p))
        .collect::<solidcore::Result<Vec<_>>>()?;
    let mut edges = Vec::new();
    for i in 0..3 {
        let next = (i + 1) % 3;
        let e = builder.start_edge(
            &k,
            OrientType::Same,
            None,
            Some(*at(&vertices, i)?),
            Some(*at(&vertices, next)?),
        )?;
        builder.set_edge_curve(
            &k,
            Curve::from(Line::from_points(*at(&points, i)?, *at(&points, next)?, 3)),
            false,
        )?;
        builder.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
        builder.end_edge()?;
        edges.push(e);
    }
    builder.end_loop()?;
    builder.end_face()?;
    builder.end_shell(&mut k, false)?;
    let mut b = builder.finish()?;
    if defect == "edge_no_uses" || second == "edge_no_uses" {
        let mut e = Edge::new([*at(&vertices, 0)?, *at(&vertices, 1)?]);
        let id = b.add_edge_record(e.clone())?;
        let mut c = Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3));
        c.set_owner(
            Some(arena_identity(CacheObject::Edge(id))),
            Some(arena_identity(CacheObject::Brep)),
        );
        e.curve = Some(c);
        *b.edge_mut(id)? = e;
    }
    let fu = require(b.face(face)?.faceuse)?;
    let lu = require(b.loop_(loop_id)?.loopuse)?;
    let LoopuseStart::Edgeuse(Some(first)) = b.loopuse(lu)?.start else {
        return Err("missing loop start".into());
    };
    let v0 = *at(&vertices, 0)?;
    let vu = *b
        .vertex(v0)?
        .vertexuses
        .first()
        .ok_or("missing fixture vertex use")?;
    for name in [defect.as_str(), second] {
        match name {
            "none" | "edge_no_uses" => {}
            "no_infinite_region" => b.set_infinite_region(None),
            "region_no_owner" => {
                let r = require(b.infinite_region())?;
                b.region_mut(r)?.in_brep = false;
            }
            "shell_no_owner" => b.shell_mut(outer)?.region = None,
            "shell_empty" => b.shell_mut(outer)?.members = ShellMembers::Empty,
            "face_no_surface" => b.face_mut(face)?.surface = None,
            "face_no_primary" => b.face_mut(face)?.faceuse = None,
            "fu_no_face" => b.faceuse_mut(fu)?.face = None,
            "fu_no_mate" => b.faceuse_mut(fu)?.mate = None,
            "fu_no_shell" => b.faceuse_mut(fu)?.shell = None,
            "fu_no_loops" => b.faceuse_mut(fu)?.loopuses.clear(),
            "loop_no_primary" => b.loop_mut(loop_id)?.loopuse = None,
            "lu_no_loop" => b.loopuse_mut(lu)?.loop_ = None,
            "lu_no_mate" => b.loopuse_mut(lu)?.mate = None,
            "lu_no_faceuse" => b.loopuse_mut(lu)?.faceuse = None,
            "lu_no_start" => b.loopuse_mut(lu)?.start = LoopuseStart::Edgeuse(None),
            "edge_no_curve" => b.edge_mut(*at(&edges, 0)?)?.curve = None,
            "eu_no_edge" => b.edgeuse_mut(first)?.edge = None,
            "eu_no_vertexuse" => b.edgeuse_mut(first)?.vertexuse = None,
            "eu_no_owner" => b.edgeuse_mut(first)?.owner = EdgeuseOwner::Loopuse(None),
            "eu_no_ccw" => b.edgeuse_mut(first)?.ccw = None,
            "eu_no_cw" => b.edgeuse_mut(first)?.cw = None,
            "vu_no_vertex" => b.vertexuse_mut(vu)?.vertex = None,
            "vu_no_owner" => b.vertexuse_mut(vu)?.owner = VertexuseOwner::Edgeuse(None),
            "vertex_no_uses" => b.vertex_mut(v0)?.vertexuses.clear(),
            "loop_bad_ring" => {
                let other = require(b.edgeuse(first)?.ccw)?;
                b.edgeuse_mut(other)?.ccw = Some(other);
            }
            _ => return Err("unknown defect".into()),
        }
    }
    let mut reports = AssertArray::new();
    let result = if selection == "no_reports" {
        required_links(&b, None)
    } else {
        required_links(&b, Some(&mut reports))
    };
    let result = if selection == "repeat" {
        required_links(&b, Some(&mut reports))
    } else {
        result
    };
    let status = match result {
        Ok(()) => 1000,
        Err(SolidError::Upstream { code, .. }) => code,
        Err(error) => return Err(error.into()),
    };
    let rows: Vec<_> = reports
        .reports()
        .iter()
        .map(|r| {
            json!([
                r.reporting_type,
                r.list_index,
                r.test_index,
                r.assert_type.code(),
                r.owner.map(|k| k.0),
                r.owner_type,
                r.name,
                r.message
            ])
        })
        .collect();
    Ok(json!({"status":status,"last_add":reports.last_add(),"reports":rows}))
}
fn main() -> Result<()> {
    println!("{}", run()?);
    Ok(())
}
