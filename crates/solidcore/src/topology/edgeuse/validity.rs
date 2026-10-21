// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native near-pole control-polygon turn query over actual stored UV data.
//!
//! Upstream: maintained `d8ed45a`, `SmEdgeuse.cpp`3663--3895.
//! | Rust | Upstream |
//! |------|----------|
//! | [`dog_leg_near_pole`] | `SmEdgeuse::HasDogLegNearPole` |
//! Deviations: a native Boolean out-index becomes Option<usize>; native writes
//! the integer directly even though its pointer type is SmBoolean*. Undefined
//! short intersector polygons return checked errors. Original turn/circle false
//! positives and the ordinary branch's overwritten X direction are retained.

use super::p1 as eu;
use crate::curve::validity;
use crate::geom::{
    extent2d::{SS_UMAX, SS_UMIN, SS_VMAX, SS_VMIN},
    geom_utility::p1::is_dog_leg,
};
use crate::topology::{surface_cache::working_surface_tolerance, Brep, EdgeuseId};
use crate::{
    error::{Result, SolidError},
    kernel::Kernel,
    math::{rad_to_deg, Point2, Vec3, PI},
};

fn invalid() -> SolidError {
    SolidError::invalid_input(
        "edgeuse::dog_leg_near_pole",
        "native polygon index outside stored data",
    )
}
/// Find the first diagnosed near-pole turn, including a mate's UV curve when
/// this use owns none. None means the native query returned false; this is not
/// a proof of correct trimming or a request to move the defining geometry.
pub fn dog_leg_near_pole(k: &Kernel, b: &mut Brep, id: EdgeuseId) -> Result<Option<usize>> {
    let (present, holder) = {
        let (c, holder) = eu::uv_trim_curve(b, id)?;
        (c.is_some(), holder)
    };
    if !present {
        return Ok(None);
    }
    let Some(face) = eu::face(b, id)? else {
        return Ok(None);
    };
    let Some(surface) = b.face(face)?.surface.as_ref() else {
        return Ok(None);
    };
    let singular = if surface.owner().is_some() {
        surface.get_singularities_of_face(
            k,
            working_surface_tolerance(k, b, surface)?,
            false,
            false,
        )?
    } else {
        surface.get_singularities(k, false, false)?
    }
    .singular;
    if singular == 0 {
        return Ok(None);
    }
    let domain = crate::topology::surface_cache::natural_domain_owned(k, b, face)?;
    let holder = holder.ok_or_else(invalid)?;
    let c = b
        .edgeuse_mut(holder)?
        .uv_trim_curve
        .as_mut()
        .ok_or_else(invalid)?;
    validity::materialize_bspline(k, c)?;
    let base = validity::bspline_base(c)?;
    let polygon = base.polygon(&c.natural_interval())?;
    let special = base.is_intersector_approx()?;
    let near = |p: Vec3| {
        let u = (singular & SS_UMIN != 0 && (p.x - domain.u_min()).abs() < domain.x_length() / 50.)
            || (singular & SS_UMAX != 0 && (p.x - domain.u_max()).abs() < domain.x_length() / 50.);
        let v = (singular & SS_VMIN != 0 && (p.y - domain.v_min()).abs() < domain.y_length() / 50.)
            || (singular & SS_VMAX != 0 && (p.y - domain.v_max()).abs() < domain.y_length() / 50.);
        (u, v)
    };
    if special {
        if polygon.len() < 4 {
            return Err(invalid());
        }
        for end in 0..2 {
            let first = if end == 0 { 0 } else { polygon.len() - 4 };
            let endpoint = *polygon.get(first + end * 3).ok_or_else(invalid)?;
            let (u, v) = near(endpoint);
            let p = polygon.get(first..first + 4).ok_or_else(invalid)?;
            let uv = |i: usize| -> Result<Point2> {
                let p = p.get(i).ok_or_else(invalid)?;
                Ok(Point2::new(p.x, p.y))
            };
            if (u || v) && is_dog_leg(uv(0)?, uv(1)?, uv(2)?, uv(3)?, 35., 45.) {
                return Ok(Some(first + 1));
            }
        }
        return Ok(None);
    }
    for (i, p) in polygon.iter().copied().enumerate() {
        let (u, v) = near(p);
        if !u && !v {
            continue;
        }
        for direction in 0..2 {
            if (direction == 0 && !u) || (direction == 1 && !v) {
                continue;
            }
            if i == 0 || i + 1 >= polygon.len() {
                continue;
            }
            let last = p - *polygon.get(i - 1).ok_or_else(invalid)?;
            let next = *polygon.get(i + 1).ok_or_else(invalid)? - p;
            // Native AngleBetween writes zero before returning an error for a
            // zero vector (SmVector3d.cpp442). No unspecified angle is invented.
            let angle = last.angle_between(next).unwrap_or(0.);
            // Source resets sSingVec to (1,0,0) for BOTH U and V passes.
            let a = rad_to_deg(last.angle_between(Vec3::X).unwrap_or(0.));
            let d = rad_to_deg(next.angle_between(Vec3::X).unwrap_or(0.));
            let contained = |x: f64, a: f64, b: f64| {
                let e = crate::geom::tol::scaled_zero2(&a, &b);
                a <= x + e && x - e <= b
            };
            let parallel = |angle: f64| contained(angle, 0., 10.) || contained(angle, 170., 180.);
            if angle > 50. * (PI / 180.)
                && ((parallel(a) && contained(d, 35., 145.))
                    || (parallel(d) && contained(a, 35., 145.)))
            {
                return Ok(Some(i));
            }
        }
    }
    Ok(None)
}
