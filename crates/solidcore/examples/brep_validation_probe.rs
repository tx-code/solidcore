// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Synthetic required-link observer; successful output is prefix evidence only.
#[path = "support/brep_subtopology_probe.rs"]
mod subprobe;
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
use solidcore::query::brep::{
    calculate_bounding_box, calculate_tight_bounding_box, lock_bounding_box, BoundingBoxOptions,
};
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::brep_cache::{arena_identity, CacheObject};
use solidcore::topology::brep_constructor::{BrepConstructor, VertexNode};
use solidcore::topology::brep_validation::{
    required_links, validate_and_update_tolerances, validate_pointers, CheckLevel,
    ValidationOptions,
};
use solidcore::topology::shell::ShellMembers;
use solidcore::topology::{Edge, EdgeuseOwner, LoopuseStart, VertexuseOwner};
use std::error::Error;

// A host-only fault at the trim boundary-build phase. The real cache factories,
// ordinary-failure fallback, BRep context and plain cache algorithms still run.
struct RejectTrim<'a>(solidcore::topology::surface_cache::BrepSurfaceContext<'a>);
macro_rules! delegate_cache {
    ($(fn $name:ident($($arg:ident : $ty:ty),*) -> $ret:ty;)*) => {
        $(fn $name(&mut self,$($arg:$ty),*) -> $ret {
            solidcore::surface::cache_topology::CacheTopology::$name(&mut self.0,$($arg),*)
        })*
    };
}
impl solidcore::surface::cache_topology::CacheTopology<solidcore::error::EntityRef>
    for RejectTrim<'_>
{
    fn create_uv_trim_curves(
        &mut self,
        _k: &Kernel,
        _f: solidcore::error::EntityRef,
    ) -> solidcore::Result<()> {
        Err(SolidError::sm_err("synthetic trim factory failure"))
    }
    delegate_cache! {
        fn face(s:&Surface) -> solidcore::Result<Option<solidcore::error::EntityRef>>;
        fn face_tolerance(f:solidcore::error::EntityRef) -> solidcore::Result<f64>;
        fn editing_enabled(f:solidcore::error::EntityRef) -> solidcore::Result<bool>;
        fn set_editing_enabled(f:solidcore::error::EntityRef,e:bool) -> solidcore::Result<()>;
        fn upward_loopuses(f:solidcore::error::EntityRef) -> solidcore::Result<Vec<solidcore::error::EntityRef>>;
        fn edgeuses(l:solidcore::error::EntityRef) -> solidcore::Result<Vec<solidcore::error::EntityRef>>;
        fn uv_trim_curve(e:solidcore::error::EntityRef) -> solidcore::Result<Option<solidcore::nurbs::b_spline_curve::BSplineCurve>>;
        fn vertexuses(l:solidcore::error::EntityRef) -> solidcore::Result<Vec<solidcore::error::EntityRef>>;
        fn uv_point(k:&Kernel,v:solidcore::error::EntityRef) -> solidcore::Result<Vec2>;
        fn vertex_tolerance(v:solidcore::error::EntityRef) -> solidcore::Result<f64>;
        fn point_classify(k:&Kernel,f:solidcore::error::EntityRef,uv:Vec2,t:f64,flags:(bool,bool)) -> solidcore::Result<(i32,Option<solidcore::error::EntityRef>)>;
    }
}
fn probe_cache(
    k: &Kernel,
    b: &mut solidcore::topology::Brep,
    face: solidcore::topology::FaceId,
    old: Option<solidcore::surface::cache_topology::OwnedSurfaceCache<solidcore::error::EntityRef>>,
    fail: bool,
) -> solidcore::Result<
    solidcore::surface::cache_topology::OwnedSurfaceCache<solidcore::error::EntityRef>,
> {
    if !fail {
        return solidcore::topology::surface_cache::get_face_surface_cache(k, b, face, old);
    }
    // Fault fixtures use only the fixed analytic unit plane. Host-only Copy
    // retains that exact definition; production factories use clone_for_evaluation.
    let source = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer("cache probe"))?;
    let mut surface = source.clone();
    surface.set_owner(source.owner());
    solidcore::surface::surface::p1::get_owned_surface_cache_with_topology(
        k,
        &surface,
        None,
        old,
        &mut RejectTrim(solidcore::topology::surface_cache::BrepSurfaceContext { brep: b, face }),
    )
}

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
    let tolerance = mode == "tolerance_check" || mode == "tolerance_update";
    let box_mode = mode.starts_with("bbox_");
    let cache_mode = mode.starts_with("cache_");
    let local_mode = mode == "local_bsp";
    let root_mode = mode == "assert_root" || mode == "assert_root_walk";
    let vertex_mode = (mode.starts_with("assert_vertex") && !mode.starts_with("assert_vertexuse"))
        || mode.starts_with("vertex_");
    let sub_mode = mode.starts_with("sub_");
    let topology_mode = sub_mode
        || vertex_mode
        || mode.starts_with("assert_region")
        || mode.starts_with("assert_shell")
        || mode.starts_with("assert_face")
        || mode.starts_with("assert_loop")
        || mode.starts_with("assert_vertexuse")
        || mode.starts_with("assert_edge");
    let side_mode = mode.starts_with("edge_side") || mode.starts_with("loop_side");
    let mut k = Kernel::default();
    let mut builder = BrepConstructor::new()?;
    let (outer, _) = builder.start_shell()?;
    let face = builder.start_face(OrientType::Same)?;
    let domain = Extent2d::from_bounds(0., 0., 1., 1.);
    builder.set_face_surface(
        Surface::from(Plane::new(
            Vec3::new(0., 0., if defect == "surface_offset" { 1. } else { 0. }),
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
    builder.end_loop(&k)?;
    if mode.contains("_hole") {
        let mut holes = if mode.contains("_outside") {
            vec![vec![(0.8, 0.8), (0.8, 0.9), (0.9, 0.9), (0.9, 0.8)]]
        } else if mode.contains("_cross") {
            vec![vec![(0.2, 0.2), (0.2, 0.8), (0.8, 0.8), (0.8, 0.2)]]
        } else if mode.contains("_shared") {
            vec![vec![(0., 0.), (0.2, 0.3), (0.3, 0.2)]]
        } else {
            vec![vec![(0.2, 0.2), (0.2, 0.4), (0.4, 0.4), (0.4, 0.2)]]
        };
        if mode.contains("_nested") {
            holes.push(vec![(0.25, 0.25), (0.25, 0.35), (0.35, 0.35), (0.35, 0.25)]);
        }
        for hp in holes {
            builder.start_loop(OrientType::Opposite)?;
            let mut hv = Vec::new();
            for (i, (x, y)) in hp.iter().copied().enumerate() {
                hv.push(if mode.contains("_shared") && i == 0 {
                    *at(&vertices, 0)?
                } else {
                    builder.start_vertex_of_loop(Point3::new(x, y, 0.))?
                });
            }
            for i in 0..hp.len() {
                let j = (i + 1) % hp.len();
                builder.start_edge(
                    &k,
                    OrientType::Same,
                    None,
                    Some(*at(&hv, i)?),
                    Some(*at(&hv, j)?),
                )?;
                let (x0, y0) = *at(&hp, i)?;
                let (x1, y1) = *at(&hp, j)?;
                builder.set_edge_curve(
                    &k,
                    Curve::from(Line::from_points(
                        Point3::new(x0, y0, 0.),
                        Point3::new(x1, y1, 0.),
                        3,
                    )),
                    false,
                )?;
                builder.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
                builder.end_edge()?;
            }
            builder.end_loop(&k)?;
        }
    }
    let vertex_loop = if mode.contains("vertexloop") {
        Some(builder.single_vertex_loop(Point3::new(0.2, 0.2, 0.))?)
    } else {
        None
    };
    builder.end_face()?;
    if mode.contains("_trihedral") {
        let mut zv = None;
        let mut ze = None;
        for n in 0..2 {
            builder.start_face(OrientType::Same)?;
            let (u, v) = if n == 0 {
                (Vec3::Z, Vec3::X)
            } else {
                (Vec3::Y, Vec3::Z)
            };
            builder.set_face_surface(
                Plane::new(Vec3::ZERO, u, v, Vec2::new(1., 1.), &domain).into(),
                domain,
            )?;
            builder.start_loop(OrientType::Same)?;
            if n == 0 {
                zv = Some(builder.start_vertex_of_loop(Vec3::Z)?);
            }
            let z = require(zv)?;
            let vv = [
                *at(&vertices, 0)?,
                if n == 0 { z } else { *at(&vertices, 2)? },
                if n == 0 { *at(&vertices, 1)? } else { z },
            ];
            let pp = [
                Vec3::ZERO,
                if n == 0 { Vec3::Z } else { Vec3::Y },
                if n == 0 { Vec3::X } else { Vec3::Z },
            ];
            let existing = [
                if n == 0 { None } else { Some(*at(&edges, 2)?) },
                None,
                if n == 0 { Some(*at(&edges, 0)?) } else { ze },
            ];
            for j in 0..3 {
                let old = *at(&existing, j)?;
                let edge = builder.start_edge(
                    &k,
                    if old.is_some() {
                        OrientType::Opposite
                    } else {
                        OrientType::Same
                    },
                    old,
                    Some(*at(&vv, j)?),
                    Some(*at(&vv, (j + 1) % 3)?),
                )?;
                if old.is_none() {
                    builder.set_edge_curve(
                        &k,
                        Line::from_points(*at(&pp, j)?, *at(&pp, (j + 1) % 3)?, 3).into(),
                        false,
                    )?;
                    builder.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
                }
                if n == 0 && j == 0 {
                    ze = Some(edge);
                }
                builder.end_edge()?;
            }
            builder.end_loop(&k)?;
            builder.end_face()?;
        }
    }
    if mode.contains("_manifold") || mode.contains("_spine") {
        for tip in [Vec3::Z, -Vec3::Y]
            .into_iter()
            .take(if mode.contains("_spine") { 2 } else { 1 })
        {
            builder.start_face(OrientType::Same)?;
            builder.set_face_surface(
                Plane::new(Vec3::X, -Vec3::X, tip, Vec2::new(1., 1.), &domain).into(),
                domain,
            )?;
            builder.start_loop(OrientType::Same)?;
            let tip_vertex = builder.start_vertex_of_loop(tip)?;
            let vv = [*at(&vertices, 1)?, *at(&vertices, 0)?, tip_vertex];
            let pp = [Vec3::X, Vec3::ZERO, tip];
            for j in 0..3 {
                builder.start_edge(
                    &k,
                    if j == 0 {
                        OrientType::Opposite
                    } else {
                        OrientType::Same
                    },
                    if j == 0 { Some(*at(&edges, 0)?) } else { None },
                    Some(*at(&vv, j)?),
                    Some(*at(&vv, (j + 1) % 3)?),
                )?;
                if j != 0 {
                    builder.set_edge_curve(
                        &k,
                        Line::from_points(*at(&pp, j)?, *at(&pp, (j + 1) % 3)?, 3).into(),
                        false,
                    )?;
                    builder.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
                }
                builder.end_edge()?;
            }
            builder.end_loop(&k)?;
            builder.end_face()?;
        }
    }
    builder.end_shell(&mut k, false)?;
    let mut b = builder.finish()?;
    if defect.starts_with("bspline_") {
        let source = b.face(face)?.surface.as_ref().ok_or("synthetic surface")?;
        let owner = source.owner();
        let solidcore::surface::surface::SurfaceKind::Plane(plane) = source.kind() else {
            return Err("synthetic plane".into());
        };
        let mut surface = Surface::from(plane.base_with_nurb()?.clone_exact());
        surface.set_owner(owner);
        b.face_mut(face)?.surface = Some(surface);
    }
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
    let region = require(b.infinite_region())?;
    let mut target_shell = outer;
    if (mode.starts_with("assert_region")
        || mode.starts_with("assert_shell")
        || ((mode.starts_with("assert_vertexuse")
            || mode.starts_with("assert_edge")
            || vertex_mode
            || sub_mode)
            && (mode.contains("_wire") || mode.contains("_shellvertex"))))
        && (mode.contains("_vertex") || mode.contains("_wire") || mode.contains("_shellvertex"))
    {
        let (shell, vertex) =
            solidcore::topology::brep::p2::make_shell_vertex_topology(&mut b, None)?;
        target_shell = shell;
        if mode.contains("_wire") {
            let (wire, end) =
                solidcore::topology::brep::p2::make_edge_vertex_topology(&mut b, None, vertex)?;
            if mode.starts_with("assert_edge") || mode.contains("_geometry") {
                b.vertex_mut(vertex)?.point = Vec3::ZERO;
                b.vertex_mut(end)?.point = Vec3::X;
                let mut c = Curve::from(Line::from_points(Vec3::ZERO, Vec3::X, 3));
                c.set_owner(
                    Some(arena_identity(CacheObject::Edge(wire))),
                    Some(arena_identity(CacheObject::Brep)),
                );
                b.edge_mut(wire)?.curve = Some(c);
                b.edge_mut(wire)?.interval =
                    solidcore::geom::extent1d::Extent1d::from_min_max(0., 1.);
            }
            if mode.contains("_chain") || mode.contains("_branch") {
                solidcore::topology::brep::p2::make_edge_vertex_topology(&mut b, None, end)?;
            }
            if mode.contains("_branch") {
                solidcore::topology::brep::p2::make_edge_vertex_topology(&mut b, None, vertex)?;
            }
        }
    }
    let observed_vertices = b.vertex_list().to_vec();
    let observed_edges = b.edge_list().to_vec();
    let fu = require(b.face(face)?.faceuse)?;
    let mate_fu = require(b.faceuse(fu)?.mate)?;
    let lu = require(b.loop_(loop_id)?.loopuse)?;
    let target_loop = vertex_loop.unwrap_or(loop_id);
    let target_lu = require(b.loop_(target_loop)?.loopuse)?;
    let target_mate_lu = require(b.loopuse(target_lu)?.mate)?;
    let LoopuseStart::Edgeuse(Some(first)) = b.loopuse(lu)?.start else {
        return Err("missing loop start".into());
    };
    let v0 = *at(&vertices, 0)?;
    let vu = *b
        .vertex(v0)?
        .vertexuses
        .first()
        .ok_or("missing fixture vertex use")?;
    let target_vu = if (mode.starts_with("assert_vertexuse") || vertex_mode || sub_mode)
        && mode.contains("_vertexloop")
    {
        match b.loopuse(target_lu)?.start {
            LoopuseStart::Vertexuse(Some(vu)) => vu,
            _ => return Err("missing vertex loop use".into()),
        }
    } else if (mode.starts_with("assert_vertexuse") || vertex_mode || sub_mode)
        && (mode.contains("_wire") || mode.contains("_shellvertex"))
    {
        match b.shell(target_shell)?.members {
            ShellMembers::Vertexuse(vu) => vu,
            ShellMembers::Edgeuse(eu) => require(b.edgeuse(eu)?.vertexuse)?,
            _ => return Err("missing shell vertex use".into()),
        }
    } else {
        vu
    };
    if (side_mode || mode.starts_with("assert_edge") || vertex_mode) && mode.contains("_uv") {
        for eu in solidcore::topology::edgeuse::p1::loopuse_edgeuses(&b, lu)? {
            solidcore::topology::edgeuse::p3::get_or_create_uv_trim_curve(
                &k, &mut b, eu, false, false, true,
            )?
            .status?;
        }
    }
    if defect == "warm_bspline" {
        let source = b.face(face)?.surface.as_ref().ok_or("surface fixture")?;
        let owner = source.owner();
        let solidcore::surface::surface::SurfaceKind::Plane(plane) = source.kind() else {
            return Err("plane receiver".into());
        };
        let mut surface = Surface::from(plane.base_with_nurb()?.clone_exact());
        surface.set_owner(owner);
        b.face_mut(face)?.surface = Some(surface);
    }
    let mark_types = [
        solidcore::geom::context::MarkType::Mark,
        solidcore::geom::context::MarkType::Mark2,
        solidcore::geom::context::MarkType::Mark3,
        solidcore::geom::context::MarkType::MarkIo,
        solidcore::geom::context::MarkType::MarkAssert,
    ];
    let before_marks = mark_types.map(|mark| k.context.current_mark(mark));
    for name in [defect.as_str(), second] {
        match name {
            "none" | "edge_no_uses" | "surface_offset" | "trim_failure" | "warm_bspline" => {}
            name if name.starts_with("reverse_") || name.starts_with("bspline_") => {}
            "face_domain_outside" => {
                b.face_mut(face)?.uv_domain = Extent2d::from_bounds(-0.1, -0.1, 1.1, 1.1);
            }
            "face_domain_line" => {
                b.face_mut(face)?.uv_domain = Extent2d::from_bounds(0., 0., 0., 1.);
            }
            "coincident_vertices" => {
                let point = b.vertex(*at(&vertices, 0)?)?.point;
                b.vertex_mut(*at(&vertices, 2)?)?.point = point;
            }
            "mate_rotate" => {
                let mate = require(b.loopuse(lu)?.mate)?;
                let LoopuseStart::Edgeuse(Some(start)) = b.loopuse(mate)?.start else {
                    return Err("mate fixture head".into());
                };
                b.loopuse_mut(mate)?.start = LoopuseStart::Edgeuse(b.edgeuse(start)?.ccw);
            }
            "mate_reversed_order" => {
                let mate = require(b.loopuse(lu)?.mate)?;
                for eu in solidcore::topology::edgeuse::p1::loopuse_edgeuses(&b, mate)? {
                    let use_ = b.edgeuse_mut(eu)?;
                    std::mem::swap(&mut use_.ccw, &mut use_.cw);
                }
            }
            "fu_empty_loops" | "fu_no_loops" => b.faceuse_mut(fu)?.loopuses.clear(),
            "empty_shell" => b.shell_mut(target_shell)?.members = ShellMembers::Empty,
            "region_mark3_ahead" | "shell_mark3_ahead" => {
                let mut contexts = [solidcore::geom::context::Context::default()];
                let mut lock = solidcore::geom::context::MarkLock::new(
                    &mut contexts,
                    0,
                    solidcore::geom::context::MarkType::Mark3.code(),
                )?;
                let entity = if name == "region_mark3_ahead" {
                    solidcore::error::EntityRef::Region(region)
                } else {
                    solidcore::error::EntityRef::Shell(target_shell)
                };
                b.notification_topology_mut(entity)?
                    .mark(contexts.first(), solidcore::geom::context::MarkType::Mark3);
                lock.release(&mut contexts)?;
            }
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
            "fu_no_shell" => {
                let links = solidcore::topology::shell::faceuse_sibling_links(&b, fu)?;
                b.faceuse_mut(fu)?.sibling_links = Some(links);
                b.faceuse_mut(fu)?.shell = None;
            }
            "loop_no_primary" => b.loop_mut(loop_id)?.loopuse = None,
            "lu_no_loop" => b.loopuse_mut(lu)?.loop_ = None,
            "lu_no_mate" => b.loopuse_mut(lu)?.mate = None,
            "lu_no_faceuse" => b.loopuse_mut(lu)?.faceuse = None,
            "lu_no_start" => b.loopuse_mut(lu)?.start = LoopuseStart::Edgeuse(None),
            "edge_no_curve" => b.edge_mut(*at(&edges, 0)?)?.curve = None,
            "curve_no_owner" => {
                b.edge_mut(*at(&edges, 0)?)?
                    .curve
                    .as_mut()
                    .ok_or("curve")?
                    .set_owner(None, None);
            }
            "edge_interval_short" | "edge_interval_outside" => {
                b.edge_mut(*at(&edges, 0)?)?.interval = if name == "edge_interval_short" {
                    solidcore::geom::extent1d::Extent1d::from_min_max(0., 0.5)
                } else {
                    solidcore::geom::extent1d::Extent1d::from_min_max(-0.1, 1.1)
                };
            }
            "edge_oversized_tolerance" => b.edge_mut(*at(&edges, 0)?)?.tolerance = 2.,
            "eu_wrong_orientation" => b.edgeuse_mut(first)?.orientation = OrientType::Opposite,
            "eu_vertex_backpointer" => {
                let mate = require(solidcore::topology::edgeuse::p1::mate(&b, first)?)?;
                let vu = require(b.edgeuse(first)?.vertexuse)?;
                b.vertexuse_mut(vu)?.owner = VertexuseOwner::Edgeuse(Some(mate));
            }
            "uv_no_owner" => {
                b.edgeuse_mut(first)?
                    .uv_trim_curve
                    .as_mut()
                    .ok_or("UV fixture")?
                    .set_owner(None, None);
            }
            "uv_zero_length" | "uv_bad_dimension" | "uv_interval" => {
                let mut c = Curve::from(Line::from_points(
                    Vec3::ZERO,
                    if name == "uv_zero_length" {
                        Vec3::ZERO
                    } else {
                        Vec3::X
                    },
                    if name == "uv_bad_dimension" { 3 } else { 2 },
                ));
                if name == "uv_interval" {
                    c.edit_parameterization(
                        &k,
                        &solidcore::geom::extent1d::Extent1d::from_min_max(0., 2.),
                    )?;
                }
                c.set_owner(
                    Some(solidcore::geom::object::ObjectRef {
                        type_code: solidcore::geom::assert_array::labels::EDGEUSE_TYPE,
                        key: solidcore::geom::attribute::ObjectKey(u64::from(first.index())),
                    }),
                    None,
                );
                b.edgeuse_mut(first)?.uv_trim_curve = Some(c);
            }
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
            "vertex_shift" => b.vertex_mut(v0)?.point = Vec3::new(0.0003, 0.0004, 0.0005),
            "root_zero_tolerance" => b.set_tolerance(0.),
            "initial_validate_only" => b.set_validate_only(true),
            name if name.starts_with("gap_") => {
                let factor = match name {
                    "gap_half" => 0.5,
                    "gap_between" => 0.75,
                    "gap_full" => 1.,
                    "gap_above" => 2.,
                    _ => 1000.,
                };
                b.vertex_mut(v0)?.point = Vec3::new(0., 0., 1e-5 * factor);
            }
            "locked_box" => {
                lock_bounding_box(&k, &mut b, false)?;
                b.vertex_mut(v0)?.point = Vec3::new(100., 100., 100.);
            }
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
        "given" if root_mode => CheckLevel::Given(&[8]),
        "given15" => CheckLevel::Given(&[15]),
        "given_face_owner" => CheckLevel::Given(&[4]),
        "given_face_vertices" => CheckLevel::Given(&[12, 13]),
        "given_face_all" => CheckLevel::Given(&[
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
            24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
        ]),
        "given" => CheckLevel::Given(&[0]),
        "two" => CheckLevel::Two,
        "heal" => CheckLevel::Heal,
        "one" => CheckLevel::One,
        "given_empty" => CheckLevel::Given(&[]),
        _ => CheckLevel::Zero,
    };
    let mut tolerance_output = None;
    let mut box_output = None;
    let mut cache_output = None;
    let mut cache_slot = None;
    let mut local_output = None;
    let mut root_output = None;
    let mut side_output = None;
    let mut sub_output = None;
    let mut vertex_query = (
        1000,
        false,
        -1.,
        None,
        Vec3::UNDEF,
        solidcore::math::BIG_DOUBLE,
        None,
        None,
        [solidcore::math::UNDEF_DOUBLE; 4],
    );
    let mut invoke = |reports: Option<&mut AssertArray>| {
        if sub_mode {
            let edge = if mode.contains("_wire") {
                let ShellMembers::Edgeuse(use_) = b.shell(target_shell)?.members else {
                    return Err(SolidError::null_pointer("wire fixture"));
                };
                b.edgeuse(use_)?
                    .edge
                    .ok_or_else(|| SolidError::null_pointer("wire edge"))?
            } else {
                *at(&edges, 0).map_err(|_| SolidError::null_pointer("fixture edge"))?
            };
            let fixture = subprobe::Fixture {
                region,
                shell: target_shell,
                face,
                loop_: target_loop,
                edge,
                vertex: b
                    .vertexuse(target_vu)?
                    .vertex
                    .ok_or_else(|| SolidError::null_pointer("fixture vertex"))?,
                fu,
                lu: target_lu,
                eu: first,
                vu: target_vu,
            };
            let sub = subprobe::observe(&mut k, &mut b, mode, selection, &fixture)?;
            let status = sub.status;
            root_output = Some(status == 1000);
            sub_output = Some(sub);
            if status == 1000 {
                Ok(())
            } else {
                Err(SolidError::Upstream {
                    operation: "subtopology observer",
                    code: status,
                    entity: None,
                })
            }
        } else if raw {
            Ok(())
        } else if root_mode {
            solidcore::topology::brep_validation::assert_valid(
                &mut k,
                &mut b,
                solidcore::topology::brep_validation::AssertValidOptions {
                    level,
                    walk: mode == "assert_root_walk",
                },
                reports,
            )
            .map(|valid| {
                root_output = Some(valid);
            })
        } else if topology_mode {
            use solidcore::topology::brep_validation as validation;
            let options = validation::AssertValidOptions {
                level,
                walk: mode.ends_with("_walk"),
            };
            let result = if mode.starts_with("assert_region") {
                validation::assert_region(&mut k, &mut b, region, options, reports)
            } else if mode.starts_with("assert_shell") {
                validation::assert_shell(&mut k, &mut b, target_shell, options, reports)
            } else if mode.starts_with("assert_faceuse") {
                validation::assert_faceuse(
                    &mut k,
                    &mut b,
                    if mode.contains("_mate") { mate_fu } else { fu },
                    options,
                    reports,
                )
            } else if mode.starts_with("assert_face") {
                validation::assert_face(&mut k, &mut b, face, options, reports)
            } else if mode.starts_with("assert_loopuse") {
                validation::assert_loopuse(
                    &mut k,
                    &mut b,
                    if mode.contains("_mate") {
                        target_mate_lu
                    } else {
                        target_lu
                    },
                    options,
                    reports,
                )
            } else if mode.starts_with("assert_vertexuse") {
                validation::assert_vertexuse(&mut k, &mut b, target_vu, options, reports)
            } else if vertex_mode {
                use solidcore::topology::vertex::validity as v;
                let vertex = b
                    .vertexuse(target_vu)?
                    .vertex
                    .ok_or_else(|| SolidError::null_pointer("target vertex"))?;
                let value = if mode.starts_with("assert_vertex") {
                    validation::assert_vertex(&mut k, &mut b, vertex, options, reports)?
                } else if mode.starts_with("vertex_pointers") {
                    v::check_pointers(&b, vertex)?
                } else if mode.starts_with("vertex_within") {
                    let r = v::within_faces(&k, &mut b, vertex, mode.contains("_force"))?;
                    vertex_query.2 = r.max_gap;
                    vertex_query.3 = r.max_face;
                    r.within
                } else if mode.starts_with("vertex_on_corner") {
                    let (valid, c) = v::is_on_face_corner(&k, &mut b, vertex)?;
                    vertex_query.1 = c.found;
                    vertex_query.2 = c.max_gap;
                    vertex_query.4 = c.point;
                    valid
                } else if mode.starts_with("vertex_corner") {
                    let c = v::calc_corner_from_faces(&k, &mut b, vertex)?;
                    vertex_query.1 = c.found;
                    vertex_query.2 = c.max_gap;
                    vertex_query.3 = c.max_face;
                    vertex_query.4 = c.point;
                    true
                } else {
                    let p = v::has_parallel_faces(&k, &mut b, vertex, None)?;
                    vertex_query.2 = p.max_gap;
                    vertex_query.5 = p.min_angle_degrees;
                    if let Some([(a, uv1), (c, uv2)]) = p.pair {
                        vertex_query.6 = Some(a);
                        vertex_query.7 = Some(c);
                        vertex_query.8 = [uv1.x, uv1.y, uv2.x, uv2.y];
                    }
                    p.found
                };
                Ok(value)
            } else if mode.starts_with("assert_edgeuse") {
                let first = if mode.contains("_wire") {
                    let ShellMembers::Edgeuse(use_) = b.shell(target_shell)?.members else {
                        return Err(SolidError::null_pointer("wire fixture"));
                    };
                    use_
                } else {
                    first
                };
                let use_ = if mode.contains("_mate") {
                    solidcore::topology::edgeuse::p1::mate(&b, first)?
                        .ok_or_else(|| SolidError::null_pointer("fixture mate"))?
                } else {
                    first
                };
                validation::assert_edgeuse(&mut k, &mut b, use_, options, reports)
            } else if mode.starts_with("assert_edge") {
                let edge = if mode.contains("_wire") {
                    let ShellMembers::Edgeuse(use_) = b.shell(target_shell)?.members else {
                        return Err(SolidError::null_pointer("wire fixture"));
                    };
                    b.edgeuse(use_)?
                        .edge
                        .ok_or_else(|| SolidError::null_pointer("wire fixture edge"))?
                } else {
                    edges
                        .first()
                        .copied()
                        .ok_or_else(|| SolidError::null_pointer("fixture edge"))?
                };
                validation::assert_edge(&mut k, &mut b, edge, options, reports)
            } else {
                validation::assert_loop(&mut k, &mut b, target_loop, options, reports)
            };
            result.map(|valid| root_output = Some(valid))
        } else if side_mode {
            use solidcore::topology::edgeuse::side;
            let periodic_u = if selection == "periodic_u" {
                Some(true)
            } else {
                None
            };
            let periodic_v = if selection == "periodic_v" {
                Some(true)
            } else {
                None
            };
            if mode.starts_with("loop_side") {
                let result = side::loopuse_side(
                    &k,
                    &b,
                    if mode.contains("_mate") {
                        target_mate_lu
                    } else {
                        target_lu
                    },
                    periodic_u,
                    periodic_v,
                )?;
                side_output = Some(json!([
                    result.inside,
                    result.on_boundary,
                    result.edgeuse_index
                ]));
            } else {
                let mut target = if mode.contains("_last") {
                    *b.edge(
                        *edges
                            .get(2)
                            .ok_or_else(|| SolidError::null_pointer("side edge fixture"))?,
                    )?
                    .edgeuses
                    .first()
                    .ok_or_else(|| SolidError::null_pointer("side fixture"))?
                } else {
                    first
                };
                if mode.contains("_mate") {
                    target = solidcore::topology::edgeuse::p1::mate(&b, target)?
                        .ok_or_else(|| SolidError::null_pointer("side fixture mate"))?;
                }
                let result = side::edgeuse_side(
                    &k,
                    &b,
                    target,
                    side::SideOptions {
                        samples: selection
                            .strip_prefix("samples_")
                            .map(str::parse)
                            .transpose()
                            .map_err(|_| {
                                SolidError::invalid_input("side samples", "invalid count")
                            })?
                            .unwrap_or(6),
                        periodic_u,
                        periodic_v,
                    },
                )?;
                side_output = Some(json!([result.inside, result.on_boundary, 0]));
            }
            Ok(())
        } else if local_mode {
            let outside = defect == "bspline_outside";
            let boundary = defect == "bspline_boundary";
            let point = Vec3::new(
                if outside {
                    1.01
                } else if boundary {
                    1.
                } else {
                    0.2
                },
                if outside {
                    -0.01
                } else if boundary {
                    0.
                } else {
                    0.3
                },
                0.01,
            );
            let guess = Vec2::new(
                if outside { 1. } else { 0.5 },
                if outside { 0. } else { 0.5 },
            );
            let source = b
                .face(face)?
                .surface
                .as_ref()
                .ok_or_else(|| SolidError::null_pointer("local probe"))?;
            let r = source.local_point_solve_1_for_face::<solidcore::error::EntityRef>(
                &k,
                &domain,
                solidcore::geom::global_solver::SolverOperation::Minimize,
                point,
                guess,
                None,
                Some(b.face(face)?.tolerance),
            )?;
            local_output = Some(json!([
                r.found,
                r.solution.start.param(0)?,
                r.solution.start.param(1)?,
                r.solution.start.solution_value
            ]));
            Ok(())
        } else if tolerance {
            validate_and_update_tolerances(
                &mut k,
                &mut b,
                mode == "tolerance_check",
                level,
                reports,
            )
            .map(|r| {
                tolerance_output = Some(r);
            })
        } else if cache_mode {
            use solidcore::surface::surface_cache::p1::FaceContainment;
            b.set_editing_enabled(mode == "cache_editing");
            k.context
                .set_doing_boolean(mode == "cache_boolean" || mode == "cache_boolean_tree");
            let mut cache = probe_cache(
                &k,
                &mut b,
                face,
                cache_slot.take(),
                defect == "trim_failure",
            )?;
            if matches!(
                mode,
                "cache_reuse" | "cache_rebuild" | "cache_reuse_editing" | "cache_rebuild_boolean"
            ) {
                cache.base_mut().trim_subdivision_factor = 37.;
                cache.base_mut().face_was_modified = mode != "cache_reuse";
                b.set_editing_enabled(mode == "cache_reuse_editing");
                k.context.set_doing_boolean(mode == "cache_rebuild_boolean");
                cache = probe_cache(&k, &mut b, face, Some(cache), defect == "trim_failure")?;
            }
            if mode == "cache_boolean_tree" || mode == "cache_base_tree" {
                cache.get_tree_with_topology(
                    &k,
                    &mut solidcore::topology::surface_cache::BrepSurfaceContext {
                        brep: &mut b,
                        face,
                    },
                )?;
            }
            let base = cache.base();
            let a = base.uv_domain().min();
            let z = base.uv_domain().max();
            let done = match base.face_containment_done {
                FaceContainment::NotDone => 0,
                FaceContainment::Done => 1,
                FaceContainment::UnderConstruction => 2,
            };
            let mut trims = Vec::new();
            for edge in &edges {
                let eu = solidcore::topology::edgeuse::p1::primary_edgeuse(&b, *edge)?
                    .ok_or_else(|| SolidError::null_pointer("cache probe"))?;
                let curve = solidcore::topology::edgeuse::p1::uv_trim_curve(&b, eu)?.0;
                trims.push(if let Some(c) = curve {
                    let ivl = c.natural_interval();
                    let mut samples = Vec::new();
                    for n in 0..3 {
                        let p = c.evaluate_point(
                            &k,
                            ivl.min() + (ivl.max() - ivl.min()) * f64::from(n) / 2.,
                        )?;
                        samples.push([p.x, p.y, p.z]);
                    }
                    json!([c.type_code(), ivl.min(), ivl.max(), samples])
                } else {
                    Value::Null
                });
            }
            let identity = |object| -> solidcore::Result<u64> {
                let r = arena_identity(object);
                let t = u64::try_from(r.type_code)
                    .map_err(|_| SolidError::invalid_input("cache probe", "negative type code"))?;
                Ok((t << 32) | r.key.0)
            };
            let cache_role = json!([
                cache.view().trimmed().is_some(),
                base.tree().is_some(),
                base.trim_subdivision_factor,
                base.face_was_modified
            ]);
            let mut point_tests = Vec::new();
            for uv in [
                Vec2::new(0.2, 0.2),
                Vec2::new(0.8, 0.8),
                Vec2::new(0., 0.),
                Vec2::new(0.5, 0.),
            ] {
                if base.tree().is_none() {
                    break;
                }
                let (good, object) = if let Some(trim) = cache.view().trimmed() {
                    trim.point_test_with_topology(
                        &k,
                        uv,
                        None,
                        &mut solidcore::topology::surface_cache::BrepSurfaceContext {
                            brep: &mut b,
                            face,
                        },
                    )?
                } else {
                    (base.point_test(uv), None)
                };
                let key = match object {
                    Some(solidcore::error::EntityRef::Face(id)) => {
                        Some(identity(CacheObject::Face(id))?)
                    }
                    Some(solidcore::error::EntityRef::Edge(id)) => {
                        Some(identity(CacheObject::Edge(id))?)
                    }
                    Some(solidcore::error::EntityRef::Vertex(id)) => {
                        Some(identity(CacheObject::Vertex(id))?)
                    }
                    None => None,
                    _ => {
                        return Err(SolidError::invalid_input(
                            "cache probe",
                            "unexpected point-test object",
                        ))
                    }
                };
                point_tests.push(json!([1000, good, key]));
            }
            cache_output = Some((
                json!([
                    a.x,
                    a.y,
                    z.x,
                    z.y,
                    base.have_t_surface_cache,
                    base.process_boundary_curves,
                    base.point_test_enabled,
                    done,
                    b.editing_enabled(),
                    b.face(face)?
                        .surface
                        .as_ref()
                        .and_then(|s| s.face().ok())
                        .flatten()
                        .is_some()
                ]),
                json!(trims),
                json!(point_tests),
                cache_role,
            ));
            cache_slot = Some(cache);
            Ok(())
        } else if box_mode {
            let result = if mode == "bbox_tight" {
                calculate_tight_bounding_box(&k, &b)
            } else {
                let flags = mode
                    .strip_prefix("bbox_")
                    .and_then(|v| v.parse::<u8>().ok())
                    .unwrap_or(0);
                calculate_bounding_box(
                    &k,
                    &b,
                    BoundingBoxOptions {
                        approx: flags & 1 != 0,
                        tight: flags & 2 != 0,
                        expand_by_zone_tol_3d: flags & 4 != 0,
                    },
                )
            };
            result.map(|r| {
                box_output = Some(r);
            })
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
            let mut row = json!([
                r.reporting_type,
                r.list_index,
                r.test_index,
                r.assert_type.code(),
                r.owner.map(|k| k.0),
                r.owner_type,
                r.name,
                r.message,
                r.tol3d,
                r.value,
                r.owner_type_string
            ]);
            if mode.starts_with("assert_edge") || vertex_mode {
                if let Some(row) = row.as_array_mut() {
                    row.push(json!([r.owner_param.x, r.owner_param.y]));
                }
            }
            row
        })
        .collect();
    let mut result = json!({"status":status,"last_add":reports.last_add(),"reports":rows});
    if let Some(mask) = reversal {
        result
            .as_object_mut()
            .ok_or("observer result is not an object")?
            .insert("reversal_mask".into(), json!(mask));
    }
    if full && !tolerance && !box_mode {
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
    let output = result
        .as_object_mut()
        .ok_or("observer result is not an object")?;
    if let Some(r) = tolerance_output {
        let vu = |id: Option<solidcore::topology::VertexuseId>| {
            id.map(|id| (16006_u64 << 32) | u64::from(id.index()))
        };
        let eu = |id: Option<solidcore::topology::EdgeuseId>| {
            id.map(|id| (16005_u64 << 32) | u64::from(id.index()))
        };
        output.insert(
            "tolerance".into(),
            json!([
                r.violated_or_updated,
                r.max_edge_face_trim_curve_gap,
                r.max_vertex_edge_gap,
                r.max_vertex_face_gap,
                r.max_edge_face_gap,
                r.max_vertex_face_trim_curve_gap,
                vu(r.vertex_edge_source),
                vu(r.vertex_face_source),
                eu(r.edge_face_trim_curve_source),
                eu(r.edge_face_source),
                eu(r.vertex_face_trim_curve_source)
            ]),
        );
    }
    if tolerance || box_mode || cache_mode {
        let mut working = vec![b.tolerance(), b.face(face)?.tolerance];
        for id in &edges {
            working.push(b.edge(*id)?.tolerance);
        }
        for id in &vertices {
            working.push(b.vertex(*id)?.tolerance);
        }
        output.insert("working_tolerances".into(), json!(working));
        output.insert("validate_only".into(), json!(b.validate_only()));
    }
    if let Some(valid) = root_output {
        output.insert(
            if topology_mode {
                "local_valid"
            } else {
                "root_valid"
            }
            .into(),
            json!(valid),
        );
    }
    if topology_mode {
        use solidcore::error::EntityRef;
        let mut current = Vec::new();
        let mut observed = vec![
            EntityRef::Region(region),
            EntityRef::Shell(target_shell),
            EntityRef::Faceuse(fu),
            EntityRef::Faceuse(mate_fu),
        ];
        observed.extend(observed_vertices.iter().copied().map(EntityRef::Vertex));
        observed.extend(observed_edges.iter().copied().map(EntityRef::Edge));
        for vertex in observed_vertices {
            observed.extend(
                b.vertex(vertex)?
                    .vertexuses
                    .iter()
                    .copied()
                    .map(EntityRef::Vertexuse),
            );
        }
        for entity in observed {
            let key = solidcore::topology::brep::p2::object_ref(&b, entity)?.key;
            let record = b
                .notifications()
                .topology
                .get(&key)
                .copied()
                .unwrap_or_default();
            current
                .push(mark_types.map(|mark| record.get_mark(mark) == k.context.current_mark(mark)));
        }
        output.insert("current_marks".into(), json!(current));
        output.insert(
            "mark_advances".into(),
            json!(mark_types
                .iter()
                .zip(before_marks)
                .map(|(mark, before)| k.context.current_mark(*mark) - before)
                .collect::<Vec<_>>()),
        );
    }
    if mode.starts_with("assert_edge") {
        let mut uses = Vec::new();
        for edge in b.edge_list() {
            uses.extend(b.edge(*edge)?.edgeuses.iter().copied());
        }
        uses.sort_by_key(|use_| use_.index());
        let states = uses
            .into_iter()
            .map(|id| {
                let use_ = b.edgeuse(id)?;
                let uv = use_.uv_trim_curve.as_ref();
                let nurb = uv.is_some_and(|uv| match uv.kind() {
                    CurveKind::BSpline(c) => c.gw_nurb().is_some(),
                    CurveKind::Line(c) => c.base().gw_nurb().is_some(),
                    _ => false,
                });
                Ok(json!([
                    id.index(),
                    uv.is_some(),
                    nurb,
                    use_.max_edge_face_gap.is_init()
                ]))
            })
            .collect::<solidcore::Result<Vec<_>>>()?;
        output.insert("use_state".into(), json!(states));
    }
    if vertex_mode {
        let vertex = require(b.vertexuse(target_vu)?.vertex)?;
        let p = b.vertex(vertex)?.point;
        output.insert("vertex_point".into(), json!([p.x, p.y, p.z]));
        let states = b
            .vertex(vertex)?
            .vertexuses
            .iter()
            .map(|id| {
                let g = b.vertexuse(*id)?.vertex_face_gap;
                Ok(json!([
                    id.index(),
                    g.is_init(),
                    g.gap_3d,
                    g.face_uv.x,
                    g.face_uv.y
                ]))
            })
            .collect::<solidcore::Result<Vec<_>>>()?;
        output.insert("vertex_state".into(), json!(states));
        if mode.starts_with("vertex_") {
            let key = |face: Option<solidcore::topology::FaceId>| {
                face.map(|f| {
                    ((solidcore::geom::assert_array::labels::FACE_TYPE as u64) << 32)
                        | u64::from(f.index())
                })
            };
            output.insert(
                "vertex_query".into(),
                json!([
                    vertex_query.0,
                    vertex_query.1,
                    vertex_query.2,
                    key(vertex_query.3),
                    [vertex_query.4.x, vertex_query.4.y, vertex_query.4.z],
                    vertex_query.5,
                    key(vertex_query.6),
                    key(vertex_query.7),
                    vertex_query.8
                ]),
            );
        }
    }
    if let Some(sub) = sub_output {
        output.insert("sub_output".into(), sub.output);
        output.insert("sub_advances".into(), sub.advances);
        output.insert("sub_marks".into(), sub.marks);
    }
    if let Some(side) = side_output {
        output.insert("side".into(), side);
    }
    if let Some(local) = local_output {
        output.insert("local".into(), local);
    }
    if let Some((cache, trims, points, role)) = cache_output {
        output.insert("cache".into(), cache);
        output.insert("uv_trims".into(), trims);
        output.insert("point_tests".into(), points);
        output.insert("cache_role".into(), role);
    }
    if let Some(bounds) = box_output {
        let a = bounds.min();
        let z = bounds.max();
        output.insert("box".into(), json!([a.x, a.y, a.z, z.x, z.y, z.z]));
    }
    Ok(result)
}
fn main() -> Result<()> {
    match run() {
        Ok(value) => println!("{value}"),
        Err(error) => {
            if let Some(SolidError::Unported { upstream, unit }) = error.downcast_ref() {
                println!("{}", json!({"deferred":{"upstream":upstream,"unit":unit}}));
            } else {
                return Err(error);
            }
        }
    }
    Ok(())
}
