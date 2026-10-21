// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host observer of periodic-jump and pole-turn queries on public synthetic data.
use serde_json::json;
use solidcore::geom::{
    assert_array::labels, attribute::ObjectKey, extent2d::Extent2d, object::ObjectRef,
};
use solidcore::nurbs::{
    b_spline_curve::BSplineCurve,
    b_spline_surface::BSplineSurface,
    b_spline_surface_n_lib::p2::OrientType,
    crv_basic::Curve as Nurb,
    math::{CPoint, NOW, NOZ},
};
use solidcore::surface::surface::Surface;
use solidcore::topology::{
    brep_constructor::{BrepConstructor, VertexNode},
    edgeuse::{p1 as eu, validity},
    LoopuseStart,
};
use solidcore::{
    curve::{
        curve::{Curve, CurveKind},
        line::Line,
    },
    error::SolidError,
    kernel::Kernel,
    math::{Vec2, Vec3},
};
fn curve(name: &str, dim: u64) -> solidcore::Result<Curve> {
    if name == "line" {
        return Ok(Line::from_points(Vec3::ZERO, Vec3::new(1., 1., 0.), dim).into());
    }
    let pts = match name {
        "jump" => vec![(0., 0.), (0.9, 0.), (1., 0.)],
        "below" => vec![(0., 0.), (0.9 - 1e-13, 0.), (1., 0.)],
        "jump_v" => vec![(0., 0.), (0., 0.9), (0., 1.)],
        "dog" => vec![(0., 0.), (0., 0.2), (0.5, 0.2), (0.5, 0.4)],
        "turn" => vec![(0., 0.), (0., 0.2), (0.5, 0.2)],
        "v_bug" => vec![(0.5, 0.71), (0.5, 0.01), (0.8, 0.16)],
        "turn_before" => vec![(0.02 - 1e-13, 0.), (0.02 - 1e-13, 0.2), (0.5, 0.2)],
        "turn_at" => vec![(0.02, 0.), (0.02, 0.2), (0.5, 0.2)],
        "turn_right" => vec![(1., 0.), (1., 0.2), (0.5, 0.2)],
        "turn_top" => vec![(0.2, 1.), (0.4, 1.), (0.4, 0.5)],
        "turn_bottom" => vec![(0., 0.), (0.2, 0.), (0.2, 0.5)],
        "dog_end" => vec![
            (0.6, 0.8),
            (0.7, 0.8),
            (0.8, 0.8),
            (0.5, 0.4),
            (0.5, 0.2),
            (0., 0.2),
            (0., 0.),
        ],
        "short" => vec![(0., 0.), (1., 1.)],
        _ => return Err(SolidError::invalid_input("query fixture", "unknown curve")),
    };
    let degree = if name == "dog" || name == "dog_end" {
        3
    } else {
        1
    };
    let mut knots = vec![0.; degree + 1];
    for i in 1..pts.len() - degree {
        knots.push(i as f64 / (pts.len() - degree) as f64);
    }
    knots.extend(std::iter::repeat_n(1., degree + 1));
    if name == "dog_end" {
        knots = vec![0., 0., 0., 0., 0.5, 0.5, 0.5, 1., 1., 1., 1.];
    }
    let points = pts
        .into_iter()
        .map(|(x, y)| CPoint::new(x, y, if dim == 2 { NOZ } else { 0. }, NOW))
        .collect();
    Ok(BSplineCurve::from_nurb(Nurb::new(points, degree, knots)?, dim, true).into())
}
fn surface(name: &str) -> solidcore::Result<Surface> {
    let (mut a, mut b, mut c, mut d) = (Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1., 1., 0.));
    match name {
        "u_min" => c = a,
        "u_max" => {
            b = Vec3::ZERO;
            d = Vec3::ZERO;
            a = Vec3::X;
            c = Vec3::new(1., 1., 0.);
        }
        "v_min" => b = a,
        "v_max" => {
            c = Vec3::ZERO;
            d = Vec3::ZERO;
            a = Vec3::Y;
            b = Vec3::new(1., 1., 0.);
        }
        _ => {}
    }
    Ok(BSplineSurface::create_bilinear_surface(a, b, c, d)?.into())
}
fn stored(c: &Curve) -> bool {
    match c.kind() {
        CurveKind::BSpline(c) => c.gw_nurb().is_some(),
        CurveKind::Line(l) => l.base().gw_nurb().is_some(),
        _ => false,
    }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let family = args.get(1).ok_or("family")?;
    let name = args.get(2).ok_or("shape")?;
    let kind = args.get(3).ok_or("surface/flags")?;
    let dim = args.get(4).ok_or("dimension")?.parse()?;
    let mate = args.get(5).ok_or("mate")?.parse::<i32>()? != 0;
    let mut k = Kernel::default();
    let mut c = curve(name, dim)?;
    let before = stored(&c);
    if family == "periodic" {
        let flags = kind.parse::<i32>()?;
        let value = c.assert_no_periodic_jump(
            &k,
            flags & 1 != 0,
            flags & 2 != 0,
            &Extent2d::from_bounds(0., 0., 1., 1.),
        )?;
        println!(
            "{}",
            json!({"value":value,"index":null,"before_nurb":before,"after_nurb":stored(&c)})
        );
        return Ok(());
    }
    let s = surface(kind)?;
    let uv = [
        Vec2::new(0.2, 0.2),
        Vec2::new(0.8, 0.2),
        Vec2::new(0.2, 0.8),
    ];
    let points = uv
        .into_iter()
        .map(|uv| s.evaluate_point(uv))
        .collect::<solidcore::Result<Vec<_>>>()?;
    let mut builder = BrepConstructor::new()?;
    builder.start_shell()?;
    builder.start_face(OrientType::Same)?;
    builder.set_face_surface(s, Extent2d::from_bounds(0., 0., 1., 1.))?;
    let loop_ = builder.start_loop(OrientType::Same)?;
    let vertices = points
        .iter()
        .map(|p| builder.start_vertex_of_loop(*p))
        .collect::<solidcore::Result<Vec<_>>>()?;
    for i in 0..3 {
        let j = (i + 1) % 3;
        builder.start_edge(
            &k,
            OrientType::Same,
            None,
            vertices.get(i).copied(),
            vertices.get(j).copied(),
        )?;
        builder.set_edge_curve(
            &k,
            Line::from_points(
                *points.get(i).ok_or("point")?,
                *points.get(j).ok_or("point")?,
                3,
            )
            .into(),
            false,
        )?;
        builder.set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
        builder.end_edge()?;
    }
    builder.end_loop()?;
    builder.end_face()?;
    builder.end_shell(&mut k, false)?;
    let mut b = builder.finish()?;
    let lu = b.loop_(loop_)?.loopuse.ok_or("loopuse")?;
    let LoopuseStart::Edgeuse(Some(use_)) = b.loopuse(lu)?.start else {
        return Err("use".into());
    };
    c.set_owner(
        Some(ObjectRef {
            type_code: labels::EDGEUSE_TYPE,
            key: ObjectKey(u64::from(use_.index())),
        }),
        None,
    );
    b.edgeuse_mut(use_)?.uv_trim_curve = Some(c);
    let target = if mate {
        eu::mate(&b, use_)?.ok_or("mate")?
    } else {
        use_
    };
    let index = validity::dog_leg_near_pole(&k, &mut b, target)?;
    let after = stored(b.edgeuse(use_)?.uv_trim_curve.as_ref().ok_or("curve")?);
    println!(
        "{}",
        json!({"value":index.is_some(),"index":index,"before_nurb":before,"after_nurb":after})
    );
    Ok(())
}
