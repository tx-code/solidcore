// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Directed latitude bands and exact minor-circle computational cuts.
//!
//! Project extension of the preserved STEP chart contract, not source repair.
//! The fixed kernel `surface/torus.rs` STEP equation is
//! S(u,v)=O+(R+r*cos(v))*(cos(u)*X+sin(u)*Y)+r*sin(v)*Z.
//! Native angular domains use degrees; source physical units remain owned by
//! `Geometry`. Only aligned, complete circular latitude boundaries are covered.

use super::{curve, full_period, need, norm, point, scalar, unsupported, Seam};
use crate::general::geometry::{Frame, Geometry};
use crate::general::{GeneralQuality, SourceEdge, SourceFace, SourceVertex};
use crate::{Error, Result};
use solidcore::geom::{extent1d::Extent1d, extent2d::Extent2d};
use solidcore::math::{Vec3, EFF_ZERO};
use std::f64::consts::{PI, TAU};

pub(super) struct MinorArc {
    pub frame: Frame,
    pub radius_mm: f64,
    pub step_interval_deg: Extent1d,
    pub reversed: bool,
    pub consistency_mm: f64,
    pub source_face: usize,
}

pub(super) struct Band {
    pub seam: Seam,
    pub domain_deg: Extent2d,
}

struct Latitude {
    vertex: usize,
    edge: usize,
    point: Vec3,
    radial: Vec3,
    radius_mm: f64,
    angle_rad: f64,
    positive_u: bool,
}

/// Preserve both source loops and their defining circles, adding only a paired
/// cut on their selected toroidal region. `SourceUse.forward` already includes
/// EDGE/OE/BOUND/FACE senses and addresses the normal-primary face use: +u is
/// the lower v boundary and -u is the upper v boundary. This determines the
/// directed v interval, including a long arc; choosing the shorter arc would
/// change the source region. Opposite material face uses stay opposite.
pub(super) fn latitude_band(
    g: &Geometry<'_>,
    face: &SourceFace,
    edges: &[SourceEdge],
    vertices: &[SourceVertex],
    frame: Frame,
    quality: &GeneralQuality,
) -> Result<Option<Band>> {
    if face.loops.len() != 2 {
        return Ok(None);
    }
    let mut candidates = Vec::new();
    for bound in &face.loops {
        let [usage] = bound.edge_uses() else {
            return Ok(None);
        };
        let edge = curve(edges, usage.edge)?;
        if !g.is(edge.geometry, "CIRCLE") || edge.vertices[0] != edge.vertices[1] {
            return Ok(None);
        }
        candidates.push(*usage);
    }
    let major = g.number(face.geometry, "major_radius")? * g.length_scale();
    let minor = g.number(face.geometry, "minor_radius")? * g.length_scale();
    if !(major.is_finite() && minor.is_finite() && major > minor && minor > EFF_ZERO) {
        return Err(unsupported("GeneralToroidalRingRequired", face.geometry));
    }
    let consistency = scalar(quality.consistency_mm);
    let mut rings = Vec::new();
    for usage in candidates {
        let edge = curve(edges, usage.edge)?;
        let cf = g.placement(g.reference(edge.geometry, "position")?)?;
        let radius = g.number(edge.geometry, "radius")? * g.length_scale();
        let center = cf.origin - frame.origin;
        let height = center.dot(frame.z);
        let eccentricity = norm(center - frame.z * height);
        let tilt = norm(cf.z.cross(frame.z));
        if tilt > 1e-12 {
            return Err(unsupported(
                "GeneralToroidalNonLatitudeRingNotYetConstructed",
                edge.record,
            ));
        }
        // For a coaxial parallel circle the whole support residual is the
        // meridian distance |sqrt((rho-R)^2+h^2)-r|. Translation contributes at
        // most eccentricity. With the enforced tiny tilt, aligning the circle
        // plane moves any point by <=2*rho*sin(theta), also for reversed normals.
        // Distance to the torus is 1-Lipschitz, so their sum screens the complete
        // defining circle, not sampled anchors. It neither enlarges a working
        // tolerance nor establishes a continuous native equivalence certificate.
        let support = eccentricity
            + 2. * radius * tilt
            + (norm(Vec3::new(radius - major, height, 0.)) - minor).abs();
        need(
            radius.is_finite()
                && radius > 0.
                && tilt.is_finite()
                && support.is_finite()
                && support <= consistency,
            "GeneralToroidalRingSupport",
            edge.record,
        )?;
        let vertex = edge.vertices[0];
        let anchor = point(vertices, vertex)?;
        let delta = anchor - cf.origin;
        let plane_gap = delta.dot(cf.z).abs();
        let radius_gap = (norm(delta - cf.z * delta.dot(cf.z)) - radius).abs();
        let built = edge
            .built
            .as_ref()
            .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
        need(
            plane_gap.is_finite()
                && radius_gap.is_finite()
                && plane_gap + radius_gap <= consistency
                && built
                    .endpoints
                    .iter()
                    .all(|p| norm(*p - anchor) <= consistency),
            "GeneralToroidalRingVertexGap",
            edge.record,
        )?;
        let local = anchor - frame.origin;
        let radial = local - frame.z * local.dot(frame.z);
        let radial_length = norm(radial);
        need(
            radial_length.is_finite() && radial_length > 0.,
            "GeneralDegenerateFrame",
            face.record,
        )?;
        rings.push(Latitude {
            edge: usage.edge,
            vertex,
            point: built.endpoints[0],
            radial: radial * (1. / radial_length),
            radius_mm: radius,
            angle_rad: height.atan2(radius - major),
            positive_u: usage.forward == (cf.z.dot(frame.z) > 0.),
        });
    }
    let mut rings = rings.into_iter();
    let first = rings.next().ok_or(Error::Invalid("GeneralSeamVertex"))?;
    let second = rings.next().ok_or(Error::Invalid("GeneralSeamVertex"))?;
    need(
        first.edge != second.edge && first.positive_u != second.positive_u,
        "GeneralToroidalRingOrientation",
        face.record,
    )?;
    if norm(first.radial - second.radial) * first.radius_mm.max(second.radius_mm) > consistency {
        // A later chart extension can use the existing complete shared-edge
        // partition mechanism. Never replace the second source anchor now.
        return Err(unsupported(
            "GeneralToroidalRingPhaseNotYetConstructed",
            face.record,
        ));
    }
    let (lower, upper) = if first.positive_u {
        (first.angle_rad, second.angle_rad)
    } else {
        (second.angle_rad, first.angle_rad)
    };
    let span = (upper - lower).rem_euclid(TAU);
    need(
        span.is_finite() && span > 0. && span < TAU,
        "GeneralToroidalRingRegion",
        face.record,
    )?;
    let lower_deg = lower * 180. / PI;
    let upper_deg = (lower + span) * 180. / PI;
    let angle = first.radial.dot(frame.y).atan2(first.radial.dot(frame.x));
    let [u0, u1] = full_period(if angle > 0. { angle - TAU } else { angle });
    // This frame is the actual minor circle in the source torus's meridian.
    // Reversing its NURBS traversal aligns increasing native parameters with
    // the first->second source vertices, without reversing either source ring.
    let arc = MinorArc {
        frame: Frame {
            origin: frame.origin + first.radial * major,
            x: first.radial,
            y: frame.z,
            z: first.radial.cross(frame.z),
        },
        radius_mm: minor,
        step_interval_deg: Extent1d::from_min_max(lower_deg, upper_deg),
        reversed: !first.positive_u,
        consistency_mm: consistency,
        source_face: face.record,
    };
    Ok(Some(Band {
        seam: Seam {
            vertices: [first.vertex, second.vertex],
            endpoints: [first.point, second.point],
            angle,
            minor_arc: Some(arc),
        },
        domain_deg: Extent2d::from_bounds(u0, lower_deg, u1, upper_deg),
    }))
}
