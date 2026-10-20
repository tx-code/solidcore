// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host-only observer matching `scripts/brep_constructor_reference.cpp`.
//! Synthetic construction evidence is distinct from native acceptance evidence.
use serde_json::{json, Value};
use solidcore::curve::{curve::Curve, line::Line};
use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point2, Point3, Vec2};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::brep_cache::{arena_identity, CacheObject};
use solidcore::topology::brep_constructor::{BrepConstructor, VertexNode};
use solidcore::topology::edgeuse::p1 as eu;
use solidcore::topology::shell::ShellMembers;
use solidcore::topology::{Brep, EdgeId, EdgeuseOwner, LoopuseStart, VertexuseOwner};
use std::collections::BTreeMap;
use std::error::Error;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn required<T>(id: Option<T>) -> Result<T> {
    id.ok_or_else(|| "observer: missing required identity".into())
}
fn index<T: Copy + PartialEq>(values: &[T], id: T) -> Result<usize> {
    values
        .iter()
        .position(|x| *x == id)
        .ok_or_else(|| "observer: missing identity".into())
}
fn at<T>(values: &[T], i: usize) -> Result<&T> {
    values
        .get(i)
        .ok_or_else(|| "invalid synthetic index".into())
}
fn orient(value: OrientType) -> i32 {
    match value {
        OrientType::Same => 1,
        OrientType::Opposite => -1,
        OrientType::Unknown => 0,
    }
}
fn point(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn observe(b: &Brep, manifold: bool) -> Result<Value> {
    let regions: Vec<_> = b.regions().ids().collect();
    let shells: Vec<_> = b.shells().ids().collect();
    let faces: Vec<_> = b.faces().ids().collect();
    let edges: Vec<_> = b.edges().ids().collect();
    let vertices: Vec<_> = b.vertices().ids().collect();
    let mut lus = Vec::new();
    for f in &faces {
        for l in &b.face(*f)?.loops {
            let lu = required(b.loop_(*l)?.loopuse)?;
            lus.push(lu);
            lus.push(required(b.loopuse(lu)?.mate)?);
        }
    }
    let mut eus = Vec::new();
    for e in &edges {
        eus.extend_from_slice(&b.edge(*e)?.edgeuses);
    }
    let mut rs = Vec::new();
    for id in &regions {
        let r = b.region(*id)?;
        let ss = r
            .shells
            .iter()
            .map(|s| index(&shells, *s))
            .collect::<Result<Vec<_>>>()?;
        rs.push(json!([r.is_void, ss]));
    }
    let mut ss = Vec::new();
    for s in &shells {
        let mut fs = Vec::new();
        if let ShellMembers::Faceuses(fus) = &b.shell(*s)?.members {
            for fu in fus {
                let fu = b.faceuse(*fu)?;
                fs.push(json!([
                    index(&faces, required(fu.face)?)?,
                    orient(fu.orientation)
                ]));
            }
        }
        ss.push(json!(fs));
    }
    let mut fs = Vec::new();
    for f in &faces {
        let face = b.face(*f)?;
        let fu = b.faceuse(required(face.faceuse)?)?;
        let mate = b.faceuse(required(fu.mate)?)?;
        let surface = face.surface.as_ref().ok_or("missing surface")?;
        let d = face.uv_domain;
        fs.push(json!([
            face.tolerance,
            index(&shells, required(fu.shell)?)?,
            index(&shells, required(mate.shell)?)?,
            surface.owner() == Some(arena_identity(CacheObject::Face(*f))),
            [d.min().x, d.min().y, d.max().x, d.max().y],
            point(surface.evaluate_point(Point2::new(0.25, 0.75))?)
        ]));
    }
    let mut ls = Vec::new();
    for lu in &lus {
        let l = b.loopuse(*lu)?;
        let fu = b.faceuse(required(l.faceuse)?)?;
        let (vertex, uses) = match l.start {
            LoopuseStart::Vertexuse(vu) => {
                let v = required(b.vertexuse(required(vu)?)?.vertex)?;
                (i64::try_from(index(&vertices, v)?)?, Vec::new())
            }
            LoopuseStart::Edgeuse(first) => {
                let start = required(first)?;
                let mut cur = start;
                let mut list = Vec::new();
                loop {
                    list.push(index(&eus, cur)?);
                    cur = required(b.edgeuse(cur)?.ccw)?;
                    if cur == start {
                        break;
                    }
                    if list.len() > eus.len() {
                        return Err("broken synthetic loop ring".into());
                    }
                }
                (-1, list)
            }
            LoopuseStart::Unknown => return Err("unknown loop form".into()),
        };
        ls.push(json!([
            index(&faces, required(fu.face)?)?,
            orient(fu.orientation),
            orient(l.orientation),
            index(&lus, required(l.mate)?)?,
            vertex,
            uses
        ]));
    }
    let mut es = Vec::new();
    for e in &edges {
        let edge = b.edge(*e)?;
        let c = edge.curve.as_ref().ok_or("missing curve")?;
        let d = edge.interval;
        let mut ps = Vec::new();
        for i in 0..3 {
            ps.push(point(c.evaluate_point(
                d.min() + (d.max() - d.min()) * f64::from(i) / 2.,
            )?));
        }
        es.push(json!([
            edge.tolerance,
            d.min(),
            d.max(),
            c.dim(),
            c.owner() == Some(arena_identity(CacheObject::Edge(*e))),
            ps
        ]));
    }
    let mut uses = Vec::new();
    for u in &eus {
        let incidence = b.edgeuse(*u)?;
        let EdgeuseOwner::Loopuse(lu) = incidence.owner else {
            return Err("not a loop incidence".into());
        };
        let vertex = required(b.vertexuse(required(incidence.vertexuse)?)?.vertex)?;
        let uv = if let Some(c) = &incidence.uv_trim_curve {
            let d = c.natural_interval();
            let mut ps = Vec::new();
            for i in 0..3 {
                ps.push(point(c.evaluate_point(
                    d.min() + (d.max() - d.min()) * f64::from(i) / 2.,
                )?));
            }
            let owner = c.owner().is_some_and(|owner| {
                owner.type_code == eu::TYPE && owner.key.0 == u64::from(u.index())
            });
            json!([c.dim(), owner, d.min(), d.max(), ps])
        } else {
            Value::Null
        };
        uses.push(json!([
            index(&edges, required(incidence.edge)?)?,
            index(&lus, required(lu)?)?,
            orient(incidence.orientation),
            index(&vertices, vertex)?,
            index(&eus, required(eu::mate(b, *u)?)?)?,
            index(&eus, required(eu::radial(b, *u)?)?)?,
            index(&eus, required(incidence.cw)?)?,
            index(&eus, required(incidence.ccw)?)?,
            uv
        ]));
    }
    let mut vs = Vec::new();
    for v in &vertices {
        let vertex = b.vertex(*v)?;
        let mut owners = Vec::new();
        for vu in &vertex.vertexuses {
            owners.push(match b.vertexuse(*vu)?.owner {
                VertexuseOwner::Edgeuse(u) => i64::try_from(index(&eus, required(u)?)?)?,
                VertexuseOwner::Loopuse(lu) => -1 - i64::try_from(index(&lus, required(lu)?)?)?,
                _ => return Err("unexpected vertex incidence form".into()),
            });
        }
        vs.push(json!([point(vertex.point), vertex.tolerance, owners]));
    }
    Ok(
        json!({"manifold":manifold,"regions":rs,"shells":ss,"faces":fs,"loops":ls,"edges":es,"uses":uses,"vertices":vs}),
    )
}

fn run() -> Result<Value> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 8 {
        return Err("shape walk outside vertices uv point-limits scale offset".into());
    }
    let shape = at(&args, 0)?.as_str();
    let walk = if at(&args, 1)? == "1" {
        OrientType::Same
    } else {
        OrientType::Opposite
    };
    let outside = if at(&args, 2)? == "1" {
        OrientType::Same
    } else {
        OrientType::Opposite
    };
    let explicit_vertices = at(&args, 3)? != "0";
    let uv = at(&args, 4)? != "0";
    let point_limits = at(&args, 5)? != "0";
    let scale = at(&args, 6)?.parse::<f64>()?;
    let offset = at(&args, 7)?.parse::<f64>()?;
    let points: Vec<_> = [
        [0., 0., 0.],
        [2., 0., 0.],
        [2., 3., 0.],
        [0., 3., 0.],
        [0., 0., 4.],
        [2., 0., 4.],
        [2., 3., 4.],
        [0., 3., 4.],
        [0.5, 0.5, 0.],
        [1.5, 0.5, 0.],
        [1.5, 2.5, 0.],
        [0.5, 2.5, 0.],
        [2., -3., 0.],
        [0., -3., 0.],
    ]
    .into_iter()
    .map(|[x, y, z]| Point3::new(x * scale + offset, y * scale + offset, z * scale + offset))
    .collect();
    let faces = match shape {
        "plane" | "vertex" | "lift" => vec![vec![vec![0, 1, 2, 3]]],
        "hole" => vec![vec![vec![0, 1, 2, 3], vec![8, 9, 10, 11]]],
        "two" => vec![vec![vec![0, 1, 2, 3]], vec![vec![2, 1, 5, 6]]],
        "radial" => vec![
            vec![vec![0, 1, 2, 3]],
            vec![vec![1, 0, 4, 5]],
            vec![vec![1, 0, 13, 12]],
        ],
        "solid" | "surface" => vec![
            vec![vec![0, 3, 2, 1]],
            vec![vec![4, 5, 6, 7]],
            vec![vec![0, 1, 5, 4]],
            vec![vec![1, 2, 6, 5]],
            vec![vec![2, 3, 7, 6]],
            vec![vec![3, 0, 4, 7]],
        ],
        _ => return Err("unknown shape".into()),
    };
    let mut k = Kernel::default();
    let mut builder = BrepConstructor::new()?;
    builder.brep_mut().set_tolerance(1.0e-5);
    let region = if shape == "solid" {
        let r = builder.start_region(true, None)?;
        builder.brep_mut().region_mut(r)?.is_void = 0;
        Some(r)
    } else {
        None
    };
    builder.start_shell()?;
    let mut vertices = BTreeMap::new();
    if explicit_vertices && shape != "vertex" {
        for face in &faces {
            for l in face {
                for i in l {
                    if !vertices.contains_key(i) {
                        vertices.insert(*i, builder.start_vertex_of_loop(*at(&points, *i)?)?);
                    }
                }
            }
        }
    }
    let mut defined: BTreeMap<(usize, usize), (EdgeId, usize, usize)> = BTreeMap::new();
    for face in faces {
        builder.start_face(outside)?;
        let polygon = at(&face, 0)?;
        let p = *at(&points, *at(polygon, 0)?)?;
        let u = *at(&points, *at(polygon, 1)?)? - p;
        let v = *at(&points, *at(polygon, 3)?)? - p;
        let domain = Extent2d::from_bounds(0., 0., 1., 1.);
        builder.set_face_surface(
            Surface::from(Plane::new(
                p,
                u,
                v,
                Vec2::new(u.length(), v.length()),
                &domain,
            )),
            domain,
        )?;
        if shape == "vertex" {
            builder.single_vertex_loop(p)?;
        } else {
            for (l, polygon) in face.into_iter().enumerate() {
                let loop_walk = if l == 0 {
                    walk
                } else if walk == OrientType::Same {
                    OrientType::Opposite
                } else {
                    OrientType::Same
                };
                builder.start_loop(loop_walk)?;
                for (i, a) in polygon.iter().enumerate() {
                    let b = *at(&polygon, (i + 1) % polygon.len())?;
                    let key = ((*a).min(b), (*a).max(b));
                    let existing = defined.get(&key).copied();
                    let sense = if existing.is_none_or(|(_, start, _)| start == *a) {
                        OrientType::Same
                    } else {
                        OrientType::Opposite
                    };
                    let e = builder.start_edge(
                        &k,
                        sense,
                        existing.map(|(e, _, _)| e),
                        vertices.get(a).copied(),
                        vertices.get(&b).copied(),
                    )?;
                    if existing.is_none() {
                        let start = *at(&points, *a)?;
                        let end = *at(&points, b)?;
                        if shape == "lift" {
                            let project = |q: Point3| {
                                let d = q - p;
                                Point3::new(d.dot(u) / u.dot(u), d.dot(v) / v.dot(v), 0.)
                            };
                            builder.set_edge_curve(
                                &k,
                                Curve::from(Line::from_points(project(start), project(end), 2)),
                                true,
                            )?;
                        } else {
                            builder.set_edge_curve(
                                &k,
                                Curve::from(Line::from_points(start, end, 3)),
                                false,
                            )?;
                        }
                        let (start, end) = if point_limits {
                            (VertexNode::Point(start), VertexNode::Point(end))
                        } else {
                            (VertexNode::Parameter(0.), VertexNode::Parameter(1.))
                        };
                        builder.set_limits(&k, start, end)?;
                        defined.insert(key, (e, *a, b));
                    }
                    if uv && shape != "lift" {
                        let (_, start, end) = defined.get(&key).ok_or("missing defining edge")?;
                        let project = |q: Point3| {
                            let d = q - p;
                            Point3::new(d.dot(u) / u.dot(u), d.dot(v) / v.dot(v), 0.)
                        };
                        let start = project(*at(&points, *start)?);
                        let end = project(*at(&points, *end)?);
                        builder.set_edge_curve(
                            &k,
                            Curve::from(Line::from_points(start, end, 2)),
                            true,
                        )?;
                    }
                    builder.end_edge()?;
                }
                builder.end_loop()?;
            }
        }
        builder.end_face()?;
    }
    builder.end_shell(&mut k, false)?;
    if region.is_some() {
        builder.end_region()?;
    }
    let manifold = builder.is_manifold_construction();
    let b = builder.finish()?;
    observe(&b, manifold)
}
fn main() -> Result<()> {
    match run() {
        Ok(value) => println!("{value}"),
        Err(error) => {
            if let Some(solidcore::error::SolidError::Unported { upstream, unit }) =
                error.downcast_ref()
            {
                println!("{}", json!({"deferred":{"upstream":upstream,"unit":unit}}));
            } else {
                return Err(error);
            }
        }
    }
    Ok(())
}
