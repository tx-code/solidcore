// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host observer of actual owned curve assertions, not complete edge validity.
use serde_json::{json, Value};
use solidcore::geom::{
    assert_array::{labels, AssertArray},
    attribute::ObjectKey,
    object::ObjectRef,
};
use solidcore::nurbs::{
    b_spline_curve::BSplineCurve,
    math::{NOW, NOZ},
    nurbs_crv::allocate_nurb_curve,
};
use solidcore::topology::{
    brep::p2,
    brep_cache::{arena_identity, CacheObject},
    brep_constructor::BrepConstructor,
    brep_validation::{self, AssertValidOptions, CheckLevel},
};
use solidcore::{
    curve::{
        curve::{Curve, CurveKind},
        line::Line,
    },
    kernel::Kernel,
    math::Point3,
};

fn shape(name: &str, dim: u64, scale: f64, offset: f64) -> solidcore::Result<Curve> {
    if name == "missing_nurb" {
        return Ok(BSplineCurve::new(dim).into());
    }
    if name == "line" || name == "line_bad" {
        let mut c = Line::from_points(
            Point3::new(offset, offset, if dim == 2 { 0. } else { offset }),
            Point3::new(
                offset + scale,
                offset + scale,
                if dim == 2 { 0. } else { offset },
            ),
            dim,
        );
        if name == "line_bad" {
            c.make_nurb()?;
            let mut n = c
                .base()
                .gw_nurb()
                .ok_or_else(|| solidcore::error::SolidError::null_pointer("line fixture"))?
                .clone();
            if let Some(p) = n.cpts_mut().first_mut() {
                p.x += scale / 8.;
            }
            c = Line::from_parts(
                BSplineCurve::from_nurb(n, dim, true),
                c.line_point(),
                c.line_vector(),
                c.step_interval(),
                c.line_scale(),
                c.inside_out(),
            );
        }
        return Ok(c.into());
    }
    let (degree, points, knots) = match name {
        "linear" => (1, vec![(0., 0.), (1., 1.)], vec![0., 0., 1., 1.]),
        "cubic" | "rational" => (
            3,
            vec![(0., 0.), (0.25, 1.), (0.75, 0.), (1., 1.)],
            vec![0., 0., 0., 0., 1., 1., 1., 1.],
        ),
        "kink" => (
            1,
            vec![(0., 0.), (0.5, 0.5), (1., 0.)],
            vec![0., 0., 0.5, 1., 1.],
        ),
        "start_repeat" => (
            2,
            vec![(0., 0.), (0., 0.), (1., 0.)],
            vec![0., 0., 0., 1., 1., 1.],
        ),
        "end_repeat" => (
            2,
            vec![(0., 0.), (1., 0.), (1., 0.)],
            vec![0., 0., 0., 1., 1., 1.],
        ),
        "point" => (2, vec![(0., 0.); 3], vec![0., 0., 0., 1., 1., 1.]),
        "pole" => (
            1,
            vec![(0., 0.), (1., 0.), (1., 0.), (2., 0.)],
            vec![0., 0., 0.25, 0.75, 1., 1.],
        ),
        "unclamped" => (
            2,
            vec![(0., 0.), (0.5, 0.5), (1., 0.)],
            vec![0., 0., 0.125, 0.875, 1., 1.],
        ),
        "mult" => (
            2,
            vec![
                (0., 0.),
                (0.25, 0.),
                (0.5, 0.),
                (0.5, 0.5),
                (0.75, 1.),
                (1., 1.),
            ],
            vec![0., 0., 0., 0.5, 0.5, 0.5, 1., 1., 1.],
        ),
        "slow" => (
            2,
            vec![(0., 0.), (1., 0.), (0., 0.00000001)],
            vec![0., 0., 0., 1., 1., 1.],
        ),
        _ => (
            2,
            vec![(0., 0.), (0.5, 0.5), (1., 0.)],
            vec![0., 0., 0., 1., 1., 1.],
        ),
    };
    let mut n = allocate_nurb_curve(points.len() - 1, degree, knots.len() - 1)?;
    n.knots_mut().copy_from_slice(&knots);
    for (i, (p, (x, y))) in n.cpts_mut().iter_mut().zip(points).enumerate() {
        let w = if name == "rational" {
            (i + 1) as f64
        } else {
            NOW
        };
        let weight = if w == NOW { 1. } else { w };
        p.x = (x * scale + offset) * weight;
        p.y = (y * scale + offset) * weight;
        p.z = if dim == 2 {
            if name == "tag2_z" {
                i as f64 * scale
            } else {
                NOZ
            }
        } else {
            offset * weight
        };
        p.w = w;
    }
    Ok(BSplineCurve::from_nurb(n, dim, true).into())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let name = args.get(1).ok_or("shape")?;
    let slot = args.get(2).ok_or("slot")?;
    let selection = args.get(3).ok_or("level")?;
    let dim = args.get(4).ok_or("dim")?.parse()?;
    let c = shape(
        name,
        dim,
        args.get(5).ok_or("scale")?.parse()?,
        args.get(6).ok_or("offset")?.parse()?,
    )?;
    let mut k = Kernel::default();
    let mut b = BrepConstructor::new()?.finish()?;
    let (_, v) = p2::make_shell_vertex_topology(&mut b, None)?;
    let (edge, _) = p2::make_edge_vertex_topology(&mut b, None, v)?;
    let eu = *b.edge(edge)?.edgeuses.first().ok_or("use")?;
    let mut c = c;
    c.set_owner(
        Some(if selection == "bad_owner" {
            arena_identity(CacheObject::Brep)
        } else if slot == "uv" {
            ObjectRef {
                type_code: labels::EDGEUSE_TYPE,
                key: ObjectKey(u64::from(eu.index())),
            }
        } else {
            arena_identity(CacheObject::Edge(edge))
        }),
        None,
    );
    if slot == "uv" {
        b.edgeuse_mut(eu)?.uv_trim_curve = Some(c);
    } else {
        b.edge_mut(edge)?.curve = Some(c);
    }
    let stored = |b: &solidcore::topology::Brep| -> solidcore::Result<bool> {
        let c = if slot == "uv" {
            b.edgeuse(eu)?.uv_trim_curve.as_ref()
        } else {
            b.edge(edge)?.curve.as_ref()
        }
        .ok_or_else(|| solidcore::error::SolidError::null_pointer("curve observer"))?;
        Ok(match c.kind() {
            CurveKind::Line(l) => l.base().gw_nurb().is_some(),
            CurveKind::BSpline(c) => c.gw_nurb().is_some(),
            _ => false,
        })
    };
    let before = stored(&b)?;
    if selection == "debug_lpa" || selection == "debug_gpa" {
        use solidcore::curve::curve::{
            p5::local_property_analysis, p7::PropertyAnalysisOptions, CurvePropertyType as P,
        };
        use solidcore::geom::solution_array::Solution;
        let c = if slot == "uv" {
            b.edgeuse(eu)?.uv_trim_curve.as_ref()
        } else {
            b.edge(edge)?.curve.as_ref()
        }
        .ok_or("debug curve")?;
        let mut rows = Vec::<Value>::new();
        for prop in [P::MinimizeFirstDerivative, P::MinimizeRadiusOfCurvature] {
            if selection == "debug_lpa" {
                for guess in [0., 0.3, 0.5, 0.8, 1.] {
                    let mut sol = Solution::<solidcore::error::EntityRef>::new();
                    let result = local_property_analysis(
                        &k,
                        c,
                        &c.natural_interval(),
                        prop,
                        guess,
                        None,
                        &[],
                        &mut sol,
                    );
                    match result {
                        Ok(found) => rows.push(json!([
                            prop as i32,
                            guess,
                            1000,
                            found,
                            sol.start.param(0)?,
                            sol.start.solution_value
                        ])),
                        Err(solidcore::error::SolidError::Upstream { code, .. }) => {
                            rows.push(json!([prop as i32, guess, code, false, null, null]));
                        }
                        Err(e) => return Err(e.into()),
                    }
                }
            } else {
                let bounds = c.calculate_bounding_box(
                    &k,
                    &c.natural_interval(),
                    solidcore::nurbs::b_spline_curve::BoundingBoxRequest {
                        normal: true,
                        pseudo: false,
                        polar: false,
                    },
                    false,
                )?;
                let sep = bounds.normal.ok_or("bounds")?.size().length() / 20.;
                let out = c.global_property_analysis::<solidcore::error::EntityRef>(
                    &k,
                    &c.natural_interval(),
                    prop,
                    sep,
                    &PropertyAnalysisOptions::default(),
                )?;
                for sol in out.solutions.as_slice() {
                    rows.push(json!([
                        prop as i32,
                        1000,
                        sol.start.param(0)?,
                        sol.start.solution_value
                    ]));
                }
            }
        }
        println!("{}", json!({"solutions":rows}));
        return Ok(());
    }
    let mut reports = AssertArray::new();
    let radius = if dim == 2 { 13 } else { 14 };
    let requested = [radius];
    let level = match selection.as_str() {
        "one" => CheckLevel::One,
        "two" => CheckLevel::Two,
        "heal" => CheckLevel::Heal,
        "given" => CheckLevel::Given(&[0]),
        "given_radius" => CheckLevel::Given(&requested),
        "given_empty" => CheckLevel::Given(&[]),
        _ => CheckLevel::Zero,
    };
    let mut call = || {
        let r = if selection == "no_reports" {
            None
        } else {
            Some(&mut reports)
        };
        let options = AssertValidOptions { level, walk: false };
        if slot == "uv" {
            brep_validation::assert_uv_curve(&mut k, &mut b, eu, options, r)
        } else {
            brep_validation::assert_edge_curve(&mut k, &mut b, edge, options, r)
        }
    };
    let result = (|| {
        let first = call()?;
        let valid = if selection == "repeat" {
            call()?
        } else {
            first
        };
        Ok::<_, solidcore::error::SolidError>((first, valid))
    })();
    match result {
        Err(solidcore::error::SolidError::Unported { unit, upstream }) => {
            println!("{}", json!({"deferred":{"unit":unit,"upstream":upstream}}));
        }
        Err(e) => return Err(e.into()),
        Ok((first, valid)) => {
            let rows: Vec<Value> = reports
                .reports()
                .iter()
                .map(|r| {
                    json!([
                        r.reporting_type,
                        r.list_index,
                        r.test_index,
                        r.assert_type as i32,
                        r.owner.map_or(0, |o| o.0),
                        r.owner_type,
                        r.name,
                        r.message,
                        r.tol3d,
                        r.value,
                        r.owner_type_string,
                        [r.owner_param.x, r.owner_param.y]
                    ])
                })
                .collect();
            let c = if slot == "uv" {
                b.edgeuse(eu)?.uv_trim_curve.as_ref()
            } else {
                b.edge(edge)?.curve.as_ref()
            }
            .ok_or("missing observed curve")?;
            println!(
                "{}",
                json!({"curve_type":c.type_code(),"valid":valid,"first":first,"before_nurb":before,"after_nurb":stored(&b)?,"last_add":reports.last_add(),"reports":rows})
            );
        }
    }
    Ok(())
}
