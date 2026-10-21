// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host-only observer for actual point-grid/surface validity operations.
use serde_json::{json, Value};
use solidcore::error::SolidError;
use solidcore::geom::{extent2d::Extent2d, point_grid::PointGrid};
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec2, Vec3};
use solidcore::nurbs::{
    b_spline_surface::BSplineSurface,
    math::{CPoint, NOW},
};
use solidcore::surface::{plane::Plane, surface::Surface, validity};
use solidcore::topology::{
    brep_constructor::BrepConstructor,
    brep_validation::{self, AssertValidOptions, CheckLevel},
};

fn knots(count: usize, degree: usize) -> Vec<f64> {
    let mut result = vec![0.; degree + 1];
    for i in 1..count - degree {
        result.push(i as f64 / (count - degree) as f64);
    }
    result.extend(std::iter::repeat_n(1., degree + 1));
    result
}
fn fixture(name: &str, scale: f64, offset: f64) -> solidcore::Result<Surface> {
    if name == "plane" {
        return Ok(Surface::from(Plane::new(
            Point3::new(offset, offset, offset),
            Vec3::X,
            Vec3::Y,
            Vec2::new(scale, scale),
            &Extent2d::from_bounds(0., 0., 1., 1.),
        )));
    }
    let (nu, nv, p, q) = match name {
        "bilinear" => (2, 2, 1, 1),
        "pole_u" => (3, 2, 1, 1),
        "pole_v" => (3, 3, 2, 1),
        "row_zero" | "late_row_zero" => (5, 3, 2, 2),
        "false_pole_u" | "false_pole_v" => (4, 4, 2, 2),
        "mult_u" => (6, 3, 2, 2),
        "mult_v" => (6, 6, 2, 2),
        _ => (3, 3, 2, 2),
    };
    let mut points = Vec::new();
    for i in 0..nu {
        let mut row = Vec::new();
        for j in 0..nv {
            let mut point = Vec3::new(i as f64, j as f64, 0.);
            match name {
                "point" => point = Vec3::ZERO,
                "curve" => point = Vec3::new((i + j) as f64, 0., 0.),
                "fold_u" => point.x = if i == 1 { 1. } else { 0. },
                "fold_v" => point.y = if j == 1 { 1. } else { 0. },
                "row_zero" | "pole_u" if i == 1 => point = Vec3::ZERO,
                "late_row_zero" if i == 2 => point = Vec3::ZERO,
                "pole_v" if j == 1 => point = Vec3::ZERO,
                "repeat" if j == 1 => point.y = 0.,
                _ => {}
            }
            if name == "false_pole_u" || name == "false_pole_v" {
                let (a, c) = if name == "false_pole_v" {
                    (j, i)
                } else {
                    (i, j)
                };
                if (a == 1 && c <= 2) || (a == 2 && (c == 1 || c == 2)) {
                    point = Vec3::ZERO;
                }
                if a == 2 && c == 0 {
                    point = if name == "false_pole_u" {
                        Vec3::new(10., 0., 0.)
                    } else {
                        Vec3::new(0., 10., 0.)
                    };
                }
            }
            point = point * scale + Vec3::new(offset, offset, offset);
            let weight = if name.contains("rational") {
                2. + i as f64 + j as f64
            } else {
                NOW
            };
            let xyz = if weight == NOW { point } else { point * weight };
            row.push(CPoint::new(xyz.x, xyz.y, xyz.z, weight));
        }
        points.push(row);
    }
    let u = if name == "mult_u" {
        vec![0., 0., 0., 0.5, 0.5, 0.5, 1., 1., 1.]
    } else {
        knots(nu, p)
    };
    let v = if name == "mult_v" {
        vec![0., 0., 0., 0.5, 0.5, 0.5, 1., 1., 1.]
    } else {
        knots(nv, q)
    };
    Ok(Surface::from(BSplineSurface::from_data(
        nu - 1,
        nv - 1,
        p,
        q,
        u.len() - 1,
        v.len() - 1,
        &points,
        &u,
        &v,
    )?))
}
fn run() -> Result<Value, Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let family = args
        .first()
        .ok_or("family shape selection [action] [scale] [offset]")?;
    let name = args.get(1).map_or("quad", String::as_str);
    let selection = args.get(2).map_or("zero", String::as_str);
    let action = args.get(3).map_or("none", String::as_str);
    let scale: f64 = args.get(4).map_or("1", String::as_str).parse()?;
    let offset: f64 = args.get(5).map_or("0", String::as_str).parse()?;
    if family == "grid" {
        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                let x = if name == "row_fold" && j == 2 {
                    0.
                } else {
                    f64::from(j)
                };
                let y = if name == "col_fold" && i == 2 {
                    0.
                } else {
                    f64::from(i)
                };
                let point = if name == "point" {
                    Vec3::ZERO
                } else {
                    Vec3::new(x, y, 0.)
                };
                points.push(point * scale + Vec3::new(offset, offset, offset));
            }
        }
        if name == "short_row" {
            points.pop();
        }
        if name == "repeat" {
            for row in points.chunks_mut(3) {
                let last = row.get(1).copied().ok_or("grid fixture")?;
                *row.get_mut(2).ok_or("grid fixture")? = last;
            }
        }
        let flags: usize = selection.parse()?;
        let mut grid = PointGrid::new(points, 3, 1e-6, flags & 1 != 0, flags & 2 != 0, None)?;
        let c = grid.coincident()?;
        let m = grid.monotonic()?;
        grid.set_closed_column(flags & 1 == 0);
        grid.set_closed_row(flags & 2 == 0);
        if action == "clear" {
            grid.clear_cache();
        }
        let c2 = grid.coincident()?;
        let m2 = grid.monotonic()?;
        return Ok(
            json!({"grid":[c.column,c.row,m.column,m.row,c2.column,c2.row,m2.column,m2.row],"good_row_size":grid.is_good_row_size()}),
        );
    }
    let mut k = Kernel::default();
    let mut builder = BrepConstructor::new()?;
    builder.start_shell()?;
    let face =
        builder.start_face(solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same)?;
    let shape = fixture(name, scale, offset)?;
    let domain = Extent2d::from_bounds(0., 0., 1., 1.);
    builder.set_face_surface(shape, domain)?;
    builder.end_face()?;
    builder.end_shell(&mut k, false)?;
    let mut b = builder.finish()?;
    let before = validity::bspline_base(b.face(face)?.surface.as_ref().ok_or("surface")?)?
        .gw_nurb()
        .is_some();
    if family == "query" {
        let face_tol = b.face(face)?.tolerance;
        let s = b.face_mut(face)?.surface.as_mut().ok_or("surface")?;
        validity::materialize_bspline(&k, s)?;
        let result = [
            validity::excessive_multiplicity(s)?,
            validity::repeated_end_points(s, 1e-6)?,
            validity::internal_pole(&k, s, 1e-6, Some(face_tol))?,
            validity::degenerate_control_row(s, 1e-6)?,
        ];
        return Ok(
            json!({"queries":result,"before_nurb":before,"after_nurb":validity::bspline_base(s)?.gw_nurb().is_some()}),
        );
    }
    let level = match selection {
        "one" => CheckLevel::One,
        "two" => CheckLevel::Two,
        "heal" => CheckLevel::Heal,
        "given" => CheckLevel::Given(&[0]),
        _ => CheckLevel::Zero,
    };
    let mut reports = solidcore::geom::assert_array::AssertArray::new();
    let options = AssertValidOptions { level, walk: false };
    let mut valid = brep_validation::assert_surface(
        &mut k,
        &mut b,
        face,
        options,
        if selection == "no_reports" {
            None
        } else {
            Some(&mut reports)
        },
    )?;
    if selection == "repeat" {
        valid = brep_validation::assert_surface(&mut k, &mut b, face, options, Some(&mut reports))?;
    }
    let rows: Vec<_> = reports
        .reports()
        .iter()
        .map(|r| {
            json!([
                r.reporting_type,
                r.list_index,
                r.test_index,
                r.assert_type.code(),
                r.owner.map(|v| v.0),
                r.owner_type,
                r.name,
                r.message,
                r.tol3d,
                r.value,
                r.owner_type_string
            ])
        })
        .collect();
    Ok(
        json!({"valid":valid,"reports":rows,"last_add":reports.last_add(),"before_nurb":before,
        "after_nurb":validity::bspline_base(b.face(face)?.surface.as_ref().ok_or("surface")?)?.gw_nurb().is_some()}),
    )
}
fn main() {
    match run() {
        Ok(value) => println!("{value}"),
        Err(error) => {
            if let Some(SolidError::Unported { upstream, unit }) =
                error.downcast_ref::<SolidError>()
            {
                println!("{}", json!({"deferred":{"unit":unit,"upstream":upstream}}));
            } else {
                eprintln!("{error}");
                std::process::exit(1);
            }
        }
    }
}
