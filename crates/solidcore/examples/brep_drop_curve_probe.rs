// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host-only observer of complete-curve projections on synthetic raw NURBS.
use serde_json::{json, Value};
use solidcore::curve::curve::{Curve, CurveKind};
use solidcore::error::{Result, SolidError};
use solidcore::geom::{extent1d::Extent1d, extent2d::Extent2d};
use solidcore::intersect::surface_drop_lib::p1::DropFailRecord;
use solidcore::kernel::Kernel;
use solidcore::math::{Vec2, Vec3};
use solidcore::nurbs::{
    b_spline_curve::BSplineCurve,
    b_spline_surface::BSplineSurface,
    math::{CPoint, NOW},
};
use solidcore::surface::{
    plane::Plane,
    surface::{
        p2::{DropCurveFail, SurfaceKind},
        Surface,
    },
};
use solidcore::topology::ObjectHandle;

fn fixture(shape: &str, scale: f64, offset: f64) -> Result<Surface> {
    if shape == "plane" {
        return Ok(Plane::new(
            Vec3::new(offset, offset, offset),
            Vec3::X,
            Vec3::Y,
            Vec2::new(scale, scale),
            &Extent2d::from_bounds(0., 0., 1., 1.),
        )
        .into());
    }
    let cylinder = shape == "cylinder" || shape == "cone";
    let circle = [
        [1., 0.],
        [1., 1.],
        [0., 1.],
        [-1., 1.],
        [-1., 0.],
        [-1., -1.],
        [0., -1.],
        [1., -1.],
        [1., 0.],
    ];
    let mut points = Vec::new();
    for (i, &[circle_x, circle_y]) in circle.iter().enumerate().take(if cylinder { 9 } else { 2 }) {
        let mut row = Vec::new();
        for j in 0..2 {
            let (mut x, mut y, mut z) = if cylinder {
                (circle_x, circle_y, j as f64)
            } else {
                (
                    i as f64,
                    j as f64,
                    if shape == "saddle" {
                        (i * j) as f64
                    } else {
                        0.
                    },
                )
            };
            if shape == "cone" {
                x *= j as f64;
                y *= j as f64;
            }
            if shape == "point" {
                x = 0.;
                y = 0.;
                z = 0.;
            }
            x = x * scale + offset;
            y = y * scale + offset;
            z = z * scale + offset;
            let w = if cylinder {
                if i % 2 == 1 {
                    2f64.sqrt() / 2.
                } else {
                    1.
                }
            } else {
                NOW
            };
            row.push(CPoint::new(
                if w > 0. { x * w } else { x },
                if w > 0. { y * w } else { y },
                if w > 0. { z * w } else { z },
                w,
            ));
        }
        points.push(row);
    }
    let u = if cylinder {
        vec![0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.]
    } else {
        vec![0., 0., 1., 1.]
    };
    Ok(BSplineSurface::from_data(
        points.len() - 1,
        1,
        if cylinder { 2 } else { 1 },
        1,
        u.len() - 1,
        3,
        &points,
        &u,
        &[0., 0., 1., 1.],
    )?
    .into())
}
fn run() -> std::result::Result<Value, Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let shape = args.first().ok_or("shape")?;
    let path = args.get(1).ok_or("path")?;
    let mode = args.get(2).ok_or("mode")?;
    let keep = args.get(3).ok_or("keep")? != "0";
    let scale = args.get(4).map_or(Ok(1.), |s| s.parse::<f64>())?;
    let offset = args.get(5).map_or(Ok(0.), |s| s.parse::<f64>())?;
    let surface = fixture(shape, scale, offset)?;
    let points = match path.as_str() {
        "iso" => vec![[0.2, 0.2, 0.], [0.8, 0.2, 0.]],
        "seam" => vec![[1., 0., 0.2], [1., 0., 0.8]],
        "radial" => vec![[1.01, 0., 0.2], [1.01, 0., 0.8]],
        "pole" => vec![[0., 0., 0.], [0., 0., 0.]],
        "outside" => vec![[-0.3, 0.2, 0.], [-0.1, 0.8, 0.]],
        "cross" => vec![[0.1, 0.2, 0.], [1.2, 0.8, 0.]],
        "curved" => vec![[0.1, 0.2, 0.02], [0.5, 0.45, 0.125], [0.9, 0.7, 0.63]],
        "quadratic" => vec![[0.1, 0.2, 0.], [0.3, 0.6, 0.], [0.9, 0.7, 0.]],
        "bulge" => vec![[0.1, 0.2, 0.], [1.5, 0.6, 0.], [0.9, 0.7, 0.]],
        _ => vec![[0.1, 0.2, 0.], [0.9, 0.7, 0.]],
    };
    let cp: Vec<_> = points
        .iter()
        .map(|p| {
            CPoint::new(
                p[0] * scale + offset,
                p[1] * scale + offset,
                p[2] * scale + offset,
                NOW,
            )
        })
        .collect();
    let mut knots = vec![2.; cp.len()];
    knots.extend(vec![5.; cp.len()]);
    let curve = Curve::from(BSplineCurve::from_data(
        3,
        cp.len() - 1,
        knots.len() - 1,
        cp.len() - 1,
        &cp,
        &knots,
    )?);
    let interval = if mode == "trim" {
        Extent1d::from_min_max(2.5, 4.5)
    } else {
        Extent1d::from_min_max(2., 5.)
    };
    let domain = match mode.as_str() {
        "subset" => Extent2d::from_bounds(0.05, 0.05, 0.95, 0.95),
        "outside" => Extent2d::from_bounds(-0.1, 0., 1., 1.),
        "fallback" => Extent2d::new(),
        _ => surface.natural_uv_domain(),
    };
    let flag = || match surface.kind() {
        SurfaceKind::Plane(p) => p.is_out_of_bounds_enabled(),
        SurfaceKind::BSpline(s) => s.is_out_of_bounds_enabled(),
        _ => false,
    };
    let before = flag();
    let mut fail = DropCurveFail::<ObjectHandle>::default();
    let result = if mode == "trace" {
        surface
            .drop_and_trim_curve(
                &Kernel::default(),
                &domain,
                &curve,
                &interval,
                1e-5 * scale,
                keep,
                false,
                Some(DropFailRecord {
                    fail: &mut fail,
                    surface: None,
                    curve: None,
                }),
            )
            .map(|r| solidcore::intersect::surface_drop_lib::p3::CurveDrop {
                max_drop_to_surf: r.max_drop_to_surf,
                max_approx_dev: r.max_approx_dev,
                uv_curves: r.uv_curves.into_iter().map(Curve::from).collect(),
            })
    } else {
        surface.drop_curve(
            &Kernel::default(),
            &domain,
            &curve,
            &interval,
            1e-5 * scale,
            keep,
            Some(DropFailRecord {
                fail: &mut fail,
                surface: None,
                curve: None,
            }),
            true,
        )
    };
    let (status, distance, deviation, curves) = match result {
        Ok(r) => (1000, r.max_drop_to_surf, r.max_approx_dev, r.uv_curves),
        Err(SolidError::Unported { upstream, unit }) => {
            return Ok(json!({"deferred":{"upstream":upstream,"unit":unit}}))
        }
        Err(SolidError::Upstream { code, .. }) => (code, 0., 0., Vec::new()),
        Err(_) => (1001, 0., 0., Vec::new()),
    };
    let mut outputs = Vec::new();
    for uv in curves {
        let b = match uv.kind() {
            CurveKind::BSpline(c) => c,
            CurveKind::Line(c) => c.base(),
            _ => return Err("unexpected UV kind".into()),
        };
        let (knots, multiplicities) = b.knots(None)?;
        let ivl = uv.natural_interval();
        let mut samples = Vec::new();
        for i in 0..9 {
            let p = uv.evaluate_point(ivl.evaluate(f64::from(i) / 8.))?;
            samples.push([p.x, p.y, p.z]);
        }
        outputs.push(json!({"degree":uv.degree()?,"dimension":uv.dim(),"knots":knots,"multiplicities":multiplicities,"samples":samples}));
    }
    Ok(
        json!({"status":status,"distance":distance,"deviation":deviation,"flags":[before,flag()],"failure":[fail.fail_status,fail.from_call as i32,fail.create_path_type as i32],"curves":outputs}),
    )
}
fn main() {
    match run() {
        Ok(v) => println!("{v}"),
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    }
}
