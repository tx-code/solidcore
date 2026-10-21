// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host observer for concrete `CrvOnSurf` evaluation/copy/transform and SDK archives.
use serde_json::{json, Value};
use solidcore::curve::{crv_on_surf::CrvOnSurf, curve::Curve, line::Line};
use solidcore::geom::{axis2_placement::Axis2Placement, extent2d::Extent2d};
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec2, Vec3};
use solidcore::nurbs::b_spline_surface::BSplineSurface;
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::database_io::{DatabaseReader, FileType, InputStream};

fn vec(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}
fn observe(k: &Kernel, c: &CrvOnSurf, t: f64) -> solidcore::Result<Value> {
    let d = c.evaluate(k, t, 3, true, false)?;
    let (u, v) = c.evaluate_surface_1st_deriv_derivs(k, t, true)?;
    let n = c.evaluate_surface_normal(k, t, true)?;
    let interval = c.natural_interval();
    Ok(
        json!({"surface_type":c.surface().type_code(),"interval":[interval.min(),interval.max()],
        "derivatives":d.into_iter().map(vec).collect::<Vec<_>>(),"normal":vec(n.normal),
        "uv":[[n.uv.x,n.uv.y],[n.uv_tangent.x,n.uv_tangent.y]],
        "surface_derivatives":[vec(u[0]),vec(u[1]),vec(v[0]),vec(v[1])]}),
    )
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let shape = args.get(1).ok_or("shape")?;
    let owner = args.get(2).ok_or("owner")?.parse()?;
    let action = args.get(3).ok_or("action")?;
    let t = args.get(4).ok_or("parameter")?.parse()?;
    let s = args.get(5).ok_or("scale")?.parse::<f64>()?;
    let o = args.get(6).ok_or("offset")?.parse()?;
    let k = Kernel::default();
    let domain = Extent2d::from_bounds(0., 0., 1., 1.);
    let surface = if shape == "bilinear" {
        Surface::from(BSplineSurface::create_bilinear_surface(
            Point3::new(o, o, o),
            Point3::new(o + s, o, o),
            Point3::new(o, o + s, o),
            Point3::new(o + s, o + s, o + s),
        )?)
    } else {
        let (x, y) = if shape == "rotated_plane" {
            (Vec3::Y, Vec3::Z)
        } else {
            (Vec3::X, Vec3::Y)
        };
        Surface::from(Plane::new(
            Point3::new(o, o, o),
            x,
            y,
            Vec2::new(s, 2. * s),
            &domain,
        ))
    };
    let uv = Curve::from(Line::create_line_segment(
        2,
        Point3::new(0.125, 0.25, 0.),
        Point3::new(0.875, 0.75, 0.),
        None,
    )?);
    let mut c = CrvOnSurf::new(&k, uv, surface, Some(domain), owner)?;
    let transform = Axis2Placement::new(Point3::new(3., -2., 1.), Vec3::Y, -Vec3::X);
    let scale = Some(if action == "nonuniform" {
        Vec3::new(2., 3., 4.)
    } else {
        Vec3::new(2., 2., 2.)
    });
    let mut status = None;
    let transform_status = |r: solidcore::Result<()>| -> solidcore::Result<Option<i32>> {
        match r {
            Ok(()) => Ok(None),
            Err(solidcore::error::SolidError::Upstream { code, .. }) => Ok(Some(code)),
            Err(e) => Err(e),
        }
    };
    if action == "transform" || action == "nonuniform" {
        status = transform_status(c.transform_with_kernel(&k, &transform, scale))?;
    }
    if action == "identity" {
        c.transform_with_kernel(&k, &Axis2Placement::default(), None)?;
    }
    let result = if action == "copy" || action == "copy_transform" {
        let mut copied = c.copy(&k)?;
        if action == "copy_transform" {
            status = transform_status(copied.transform_with_kernel(&k, &transform, scale))?;
        }
        observe(&k, &copied, t)?
    } else if action == "binary" || action == "ascii" {
        let hex = args.get(7).ok_or("SDK archive hex")?;
        let bytes = (0..hex.len())
            .step_by(2)
            .map(|i| {
                hex.get(i..i + 2)
                    .ok_or("odd or non-ASCII hex")
                    .and_then(|pair| u8::from_str_radix(pair, 16).map_err(|_| "invalid hex"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let kind = if action == "binary" {
            FileType::Binary
        } else {
            FileType::Ascii
        };
        let mut db = DatabaseReader::with_stream(kind, InputStream::new(bytes));
        db.set_version(40);
        observe(&k, &CrvOnSurf::read_from_db(&k, &mut db, 40)?, t)?
    } else {
        observe(&k, &c, t)?
    };
    println!(
        "{}",
        json!({"status":status,"result":result,"original":observe(&k,&c,t)?})
    );
    Ok(())
}
