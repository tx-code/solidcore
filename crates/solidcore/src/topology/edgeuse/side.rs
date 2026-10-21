// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native natural-boundary inside-side queries of edgeuses and loopuses.
//!
//! Upstream: `SmEdgeuse.cpp::IsEdgeuseSideInSurface` (871--1064),
//! `SmLoopuse.cpp::IsLoopuseSideInSurface` (1034--1099), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`edgeuse_side`] | `SmEdgeuse::IsEdgeuseSideInSurface` |
//! | [`loopuse_side`] | `SmLoopuse::IsLoopuseSideInSurface` |
//! Deviations: only the previous sample is retained (the source arrays have no
//! other non-debug consumer). Missing failed-drop output state stays explicit;
//! checked ids replace unsafe pointers. Sampling is the original predicate,
//! not a continuous region-orientation or approximation-error bound.

use super::p1 as eu;
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::extent2d::{SS_UMAX, SS_UMIN, SS_VMAX, SS_VMIN};
use crate::geom::extent3d::Extent3d;
use crate::kernel::Kernel;
use crate::math::{Point2, Point3, Vec2};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::p1::map_to_2d_dir;
use crate::surface::surface::p5::DropPointOptions;
use crate::topology::{Brep, EdgeuseId, LoopuseId};

/// Native sample count and optional known periodicity (both absent by default).
#[derive(Debug, Clone, Copy)]
pub struct SideOptions {
    /// Number of edge samples; native default is six. Zero/one retain native
    /// endpoint/unsigned-threshold behavior rather than adding a new minimum.
    pub samples: usize,
    /// Known U periodicity; `None` calls the actual surface query.
    pub periodic_u: Option<bool>,
    /// Known V periodicity; `None` calls the actual surface query.
    pub periodic_v: Option<bool>,
}
impl Default for SideOptions {
    fn default() -> Self {
        Self {
            samples: 6,
            periodic_u: None,
            periodic_v: None,
        }
    }
}

/// Native boolean and natural-boundary output of an edgeuse-side check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeuseSide {
    /// Its sampled inside side lies in the natural surface domain.
    pub inside: bool,
    /// All samples retained at least one common natural boundary. Missing
    /// face/surface/edge returns true here, as the native early return does.
    pub on_boundary: bool,
}

fn missing() -> SolidError {
    SolidError::null_pointer("topology::edgeuse::side")
}

/// Check the native sampled side and preserve the source boundary priority and
/// seam allowance. The actual surface owner supplies its native working tolerance.
pub fn edgeuse_side(
    k: &Kernel,
    b: &Brep,
    id: EdgeuseId,
    options: SideOptions,
) -> Result<EdgeuseSide> {
    let okay = EdgeuseSide {
        inside: true,
        on_boundary: true,
    };
    let Some(face) = eu::face(b, id)? else {
        return Ok(okay);
    };
    let Some(surface) = b.face(face)?.surface.as_ref() else {
        return Ok(okay);
    };
    let Some(edge) = b.edgeuse(id)?.edge else {
        return Ok(okay);
    };
    let edge_record = b.edge(edge)?;
    let curve = edge_record.curve.as_ref().ok_or_else(missing)?;
    let domain = surface.natural_uv_domain();
    let periodic_u = options
        .periodic_u
        .map_or_else(|| surface.is_periodic(k, &domain, SurfParamType::U), Ok)?;
    let periodic_v = options
        .periodic_v
        .map_or_else(|| surface.is_periodic(k, &domain, SurfParamType::V), Ok)?;
    let owner_lookup = |owner| crate::topology::surface_cache::owning_face_tolerance(b, owner);
    let surface_tol = crate::topology::surface_cache::working_surface_tolerance(k, b, surface)?;
    let uv_curve = eu::uv_trim_curve(b, id)?.0;
    let same = b.edgeuse(id)?.orientation == OrientType::Same;
    let mut edge_point = Point3::UNDEF;
    let mut uv_guess_point = Point3::UNDEF;
    let mut previous = Point2::UNDEF;
    let mut up_u = 0;
    let mut down_u = 0;
    let mut up_v = 0;
    let mut down_v = 0;
    let mut common = SS_UMIN | SS_VMIN | SS_UMAX | SS_VMAX;
    let mut bounds = Extent3d::new();
    for i in 0..options.samples {
        let t = if same {
            if i == 0 {
                0.
            } else if i + 1 == options.samples {
                1.
            } else {
                i as f64 / (options.samples - 1) as f64
            }
        } else if i == 0 {
            1.
        } else if i + 1 == options.samples {
            0.
        } else {
            1. - i as f64 / (options.samples - 1) as f64
        };
        let param = edge_record.interval.evaluate(t);
        match curve.evaluate_point(k, param) {
            Ok(point) => edge_point = point,
            Err(error) if error.is_unported() => return Err(error),
            Err(_) => {}
        }
        // Source evaluates an existing UV curve at the 3D EDGE parameter, not
        // its natural-interval fraction. Do not normalize this convention.
        let guess = if let Some(uv) = uv_curve {
            match uv.evaluate_point(k, param) {
                Ok(point) => uv_guess_point = point,
                Err(error) if error.is_unported() => return Err(error),
                Err(_) => {}
            }
            Some(Point2::new(uv_guess_point.x, uv_guess_point.y))
        } else if i > 0 {
            Some(previous)
        } else {
            None
        };
        let dropped = match surface.drop_point_with_face_lookup::<EntityRef>(
            k,
            edge_point,
            &domain,
            &DropPointOptions {
                guess_uv: guess,
                ..DropPointOptions::default()
            },
            Some(&owner_lookup),
        ) {
            Ok(result) => result,
            Err(error) if error.is_unported() => return Err(error),
            Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => {
                // UNPORTED(fork.topology_validation): the existing fallible drop
                // interface cannot expose its native partial outputs on error.
                return Err(SolidError::unported(
                    "SmSurface::DropPoint failed out-parameter state",
                    "fork.topology_validation",
                ));
            }
            Err(error) => return Err(error),
        };
        let uv = dropped.result_uv; // Source deliberately does not test success.
        let tol_u = map_to_2d_dir(k, surface_tol, uv, Vec2::new(1., 0.), surface)?;
        let tol_v = map_to_2d_dir(k, surface_tol, uv, Vec2::new(0., 1.), surface)?;
        let location = (domain.point_boundaries(uv, 2. * tol_u) & (SS_UMIN | SS_UMAX))
            | (domain.point_boundaries(uv, 2. * tol_v) & (SS_VMIN | SS_VMAX));
        bounds.add_point(edge_point);
        common &= location;
        if i > 0 {
            if previous.x < uv.x {
                up_u += 1;
            }
            if previous.x > uv.x {
                down_u += 1;
            }
            if previous.y < uv.y {
                up_v += 1;
            }
            if previous.y > uv.y {
                down_v += 1;
            }
        }
        previous = uv;
        if common == 0 {
            return Ok(EdgeuseSide {
                inside: true,
                on_boundary: false,
            });
        }
    }
    if bounds.max_dimension() < 2. * surface_tol {
        return Ok(okay);
    }
    let threshold = options.samples.wrapping_sub(2);
    let no_uv = uv_curve.is_none();
    let inside = if common & SS_UMIN != 0 {
        down_v >= threshold || (no_uv && periodic_u && up_v >= threshold)
    } else if common & SS_UMAX != 0 {
        up_v >= threshold || (no_uv && periodic_u && down_v >= threshold)
    } else if common & SS_VMIN != 0 {
        up_u >= threshold || (no_uv && periodic_v && down_u >= threshold)
    } else if common & SS_VMAX != 0 {
        down_u >= threshold || (no_uv && periodic_v && up_u >= threshold)
    } else {
        true
    };
    Ok(EdgeuseSide {
        inside,
        on_boundary: true,
    })
}

/// Native aggregate loopuse-side result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopuseSide {
    /// All tested edgeuses had their inside side in the natural surface domain.
    pub inside: bool,
    /// At least one tested edgeuse was on a natural boundary.
    pub on_boundary: bool,
    /// The first failing edgeuse index; native initializes this to zero even if
    /// all tests pass or the loop has no edgeuses.
    pub edgeuse_index: usize,
}

/// Aggregate six-sample edgeuse checks in CCW order; break at first failure.
pub fn loopuse_side(
    k: &Kernel,
    b: &Brep,
    id: LoopuseId,
    periodic_u: Option<bool>,
    periodic_v: Option<bool>,
) -> Result<LoopuseSide> {
    let mut result = LoopuseSide {
        inside: true,
        on_boundary: false,
        edgeuse_index: 0,
    };
    for (i, eu) in eu::loopuse_edgeuses(b, id)?.iter().copied().enumerate() {
        let checked = edgeuse_side(
            k,
            b,
            eu,
            SideOptions {
                periodic_u,
                periodic_v,
                ..SideOptions::default()
            },
        )?;
        result.on_boundary |= checked.on_boundary;
        result.inside &= checked.inside;
        if !checked.inside {
            result.edgeuse_index = i;
            break;
        }
    }
    Ok(result)
}
