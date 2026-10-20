// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Synthetic required-link observer; successful output is prefix evidence only.
use serde_json::{json, Value};
use solidcore::curve::{
    curve::{Curve, CurveKind},
    line::Line,
};
use solidcore::error::SolidError;
use solidcore::geom::assert_array::AssertArray;
use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec2, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::brep_cache::{arena_identity, CacheObject};
use solidcore::topology::brep_constructor::{BrepConstructor, VertexNode};
use solidcore::topology::brep_validation::{
    required_links, validate_pointers, CheckLevel, ValidationOptions,
};
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
    let mode = args.get(3).map_or("prefix", String::as_str);
    let full = mode != "prefix";
    let raw = mode == "raw_fix" || mode == "raw_check";
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
        let curve = if i == 0 && defect.starts_with("reverse_") {
            use solidcore::nurbs::{
                b_spline_curve::BSplineCurve,
                crv_basic::Curve as NurbCurve,
                math::{CPoint, NOW, NOZ},
            };
            let xs = [
                0.,
                if defect == "reverse_back" { -0.5 } else { 1. },
                -1.,
                if defect == "reverse_front" { 0.5 } else { 2. },
                1.,
            ];
            let scale = match defect.as_str() {
                "reverse_tiny" => 1e-8,
                "reverse_limit" => 5e-8,
                "reverse_above" => 1e-7,
                _ => 1.,
            };
            let cp = xs
                .into_iter()
                .enumerate()
                .map(|(i, x)| {
                    let x = if defect == "reverse_flat" && i == 1 {
                        0.
                    } else {
                        x
                    };
                    let weight = if defect == "reverse_rational" {
                        f64::from(u32::try_from(i).unwrap_or(0)) + 2.
                    } else {
                        NOW
                    };
                    CPoint::new(
                        x * scale * if weight == NOW { 1. } else { weight },
                        0.,
                        if defect == "reverse_2d" { NOZ } else { 0. },
                        weight,
                    )
                })
                .collect();
            Curve::from(BSplineCurve::from_gw_nurb(
                3,
                &NurbCurve::new(cp, 1, vec![0., 0., 0.25, 0.5, 0.75, 1., 1.])?,
            ))
        } else {
            Curve::from(Line::from_points(*at(&points, i)?, *at(&points, next)?, 3))
        };
        builder.set_edge_curve(&k, curve, false)?;
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
            name if name.starts_with("reverse_") => {}
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
            "fu_wrong_orientation" => b.faceuse_mut(fu)?.orientation = OrientType::Opposite,
            "mate_wrong_orientation" => {
                let mate = require(b.faceuse(fu)?.mate)?;
                b.faceuse_mut(mate)?.orientation = OrientType::Same;
            }
            "lu_wrong_orientation" => b.loopuse_mut(lu)?.orientation = OrientType::Opposite,
            "mate_lu_wrong_orientation" => {
                let mate = require(b.loopuse(lu)?.mate)?;
                b.loopuse_mut(mate)?.orientation = OrientType::Opposite;
            }
            "face_low_tolerance" => b.face_mut(face)?.tolerance = 0.000001,
            "face_high_tolerance" => b.face_mut(face)?.tolerance = 0.1,
            "edge_low_tolerance" => b.edge_mut(*at(&edges, 0)?)?.tolerance = 0.,
            "edge_high_tolerance" => b.edge_mut(*at(&edges, 0)?)?.tolerance = 0.1,
            "vertex_low_tolerance" => b.vertex_mut(v0)?.tolerance = 0.,
            "vertex_high_tolerance" => b.vertex_mut(v0)?.tolerance = 0.1,
            "root_zero_tolerance" => b.set_tolerance(0.),
            "surface_no_owner" => {
                b.face_mut(face)?
                    .surface
                    .as_mut()
                    .ok_or("missing surface")?
                    .set_owner(None);
            }
            _ => return Err("unknown defect".into()),
        }
    }
    let mut reports = AssertArray::new();
    let reversal = if raw {
        let edge = *at(&edges, 0)?;
        let curve = b.edge_mut(edge)?.curve.as_mut().ok_or("missing curve")?;
        let CurveKind::BSpline(base) = curve.kind_mut() else {
            return Err("raw reversal needs a spline fixture".into());
        };
        Some(
            base.get_or_create_gw_nurb()?
                .end_reversal(mode == "raw_fix")?,
        )
    } else {
        None
    };
    let level = match selection {
        "given" => CheckLevel::Given(&[0]),
        "two" => CheckLevel::Two,
        _ => CheckLevel::Zero,
    };
    let mut invoke = |reports: Option<&mut AssertArray>| {
        if raw {
            Ok(())
        } else if full {
            validate_pointers(
                &mut k,
                &mut b,
                ValidationOptions { level, walk: false },
                reports,
            )
        } else {
            required_links(&b, reports)
        }
    };
    let result = invoke(if selection == "no_reports" {
        None
    } else {
        Some(&mut reports)
    });
    let result = if selection == "repeat" {
        invoke(Some(&mut reports))
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
                r.message,
                r.tol3d,
                r.value
            ])
        })
        .collect();
    let mut result = json!({"status":status,"last_add":reports.last_add(),"reports":rows});
    if let Some(mask) = reversal {
        result
            .as_object_mut()
            .ok_or("observer result is not an object")?
            .insert("reversal_mask".into(), json!(mask));
    }
    if full {
        let cached = match b.edge(*at(&edges, 0)?)?.curve.as_ref().map(Curve::kind) {
            Some(CurveKind::BSpline(base)) => base.gw_nurb(),
            Some(CurveKind::Line(c)) => c.base().gw_nurb(),
            _ => None,
        };
        let points = cached.map_or(Value::Null, |c| {
            json!(c
                .cpts()
                .iter()
                .map(|p| [p.x, p.y, p.z, p.w])
                .collect::<Vec<_>>())
        });
        result
            .as_object_mut()
            .ok_or("observer result is not an object")?
            .insert("control_points".into(), points);
    }
    Ok(result)
}
fn main() -> Result<()> {
    println!("{}", run()?);
    Ok(())
}
