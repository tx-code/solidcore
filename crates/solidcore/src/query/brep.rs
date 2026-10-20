// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native `BRep` box queries used by validation and spatial-cache construction.
//!
//! Upstream: `SmBrep.cpp::CalculateBoundingBox/CalculateTightBoundingBox`,
//! `LockBoundingBox/UnLockBoundingBox`, maintained fork `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`calculate_bounding_box`] | `CalculateBoundingBox` |
//! | [`calculate_tight_bounding_box`] | `CalculateTightBoundingBox` |
//! | [`lock_bounding_box`] / [`unlock_bounding_box`] | `LockBoundingBox` / `UnLockBoundingBox` |
//!
//! Deviations: returned owned values replace output pointers; normal boxes are
//! published only after successful traversal. Ignored ordinary tight-box failures
//! retain the previous local box; unavailable callees remain explicit errors.
use crate::error::{ignored, Result};
use crate::geom::extent3d::Extent3d;
use crate::kernel::Kernel;
use crate::nurbs::b_spline_curve::BoundingBoxRequest;
use crate::topology::Brep;

/// Native loose-box options, including its exact approximation cutoffs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBoxOptions {
    /// Skip faces; with more than 500 vertices AND 100 edges return vertices only.
    pub approximate: bool,
    /// Request tight member boxes from the owning edge/face query.
    pub tight: bool,
    /// Expand each member's box by that member's working tolerance.
    pub expand_by_tolerance: bool,
}
impl Default for BoundingBoxOptions {
    fn default() -> Self {
        Self {
            approximate: false,
            tight: false,
            expand_by_tolerance: true,
        }
    }
}

/// `CalculateBoundingBox`: ordered union of actual vertex, edge and face boxes.
/// A locked box takes precedence over every option; tolerance is in model units.
pub fn calculate_bounding_box(
    k: &Kernel,
    b: &Brep,
    options: BoundingBoxOptions,
) -> Result<Extent3d> {
    if let Some(locked) = b.locked_bounding_box() {
        return Ok(*locked);
    }
    let mut result = Extent3d::new();
    for id in b.vertex_list() {
        let vertex = b.vertex(*id)?;
        let mut part = Extent3d::from_points(vertex.point, vertex.point);
        if options.expand_by_tolerance {
            part.expand_absolute(vertex.tolerance);
        }
        result = result.union(&part);
    }
    if options.approximate && b.vertex_list().len() > 500 && b.edge_list().len() > 100 {
        return Ok(result);
    }
    for id in b.edge_list() {
        let edge = b.edge(*id)?;
        if edge.curve.is_none() {
            continue;
        }
        let mut part = super::edge::calculate_bounding_box(
            k,
            b,
            *id,
            BoundingBoxRequest {
                normal: true,
                ..BoundingBoxRequest::default()
            },
            options.tight,
        )?
        .normal
        .unwrap_or_else(Extent3d::new);
        if options.expand_by_tolerance {
            part.expand_absolute(edge.tolerance);
        }
        result = result.union(&part);
    }
    if !options.approximate {
        for id in b.face_list() {
            let mut part = super::face::p1::calculate_bounding_box(k, b, *id, options.tight)?;
            if options.expand_by_tolerance {
                part.expand_absolute(b.face(*id)?.tolerance);
            }
            result = result.union(&part);
        }
    }
    Ok(result)
}

/// `CalculateTightBoundingBox`: no tolerance expansion and no locked-box shortcut.
/// Retains the native contained-member shortcut and ignored status behavior.
pub fn calculate_tight_bounding_box(k: &Kernel, b: &Brep) -> Result<Extent3d> {
    let mut result = Extent3d::new();
    for id in b.vertex_list() {
        result.add_point(b.vertex(*id)?.point);
    }
    let mut edge_box = Extent3d::new();
    for id in b.edge_list() {
        if let Some(boxes) = ignored(super::edge::calculate_bounding_box(
            k,
            b,
            *id,
            BoundingBoxRequest {
                normal: true,
                ..BoundingBoxRequest::default()
            },
            false,
        ))? {
            if let Some(normal) = boxes.normal {
                edge_box = normal;
            }
        }
        if edge_box.is_contained_by(&result, 0.) {
            continue;
        }
        if let Some(Some(tight)) = ignored(super::edge::calculate_tight_bounding_box(k, b, *id))? {
            edge_box = tight;
        }
        result = result.union(&edge_box);
    }
    let mut face_box = Extent3d::new();
    for id in b.face_list() {
        if let Some(part) = ignored(super::face::p1::calculate_bounding_box(k, b, *id, false))? {
            face_box = part;
        }
        if face_box.is_contained_by(&result, 0.) {
            continue;
        }
        if let Some(part) = ignored(super::face::p3::calculate_tight_bounding_box(k, b, *id))? {
            face_box = part;
        }
        result = result.union(&face_box);
    }
    Ok(result)
}

/// Clear an old lock first, then compute/store a replacement. Failure leaves the
/// model unlocked, matching native failure cleanup.
pub fn lock_bounding_box(k: &Kernel, b: &mut Brep, tight: bool) -> Result<()> {
    unlock_bounding_box(b);
    let bounds = if tight {
        calculate_tight_bounding_box(k, b)?
    } else {
        calculate_bounding_box(k, b, BoundingBoxOptions::default())?
    };
    b.set_locked_bounding_box_record(Some(bounds));
    Ok(())
}
/// Release the explicit box lock, leaving geometry and working tolerances alone.
pub fn unlock_bounding_box(b: &mut Brep) {
    b.set_locked_bounding_box_record(None);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Point3, Vec3};
    #[test]
    fn lock_overrides_query_options_until_released() {
        let k = Kernel::default();
        let mut b = Brep::new();
        let v = b.add_vertex(Point3::ZERO).unwrap();
        b.vertex_mut(v).unwrap().tolerance = 0.1;
        lock_bounding_box(&k, &mut b, false).unwrap();
        let locked = *b.locked_bounding_box().unwrap();
        b.vertex_mut(v).unwrap().point = Vec3::new(10., 20., 30.);
        assert_eq!(
            calculate_bounding_box(
                &k,
                &b,
                BoundingBoxOptions {
                    approximate: true,
                    tight: true,
                    expand_by_tolerance: false
                }
            )
            .unwrap(),
            locked
        );
        unlock_bounding_box(&mut b);
        let current = calculate_bounding_box(
            &k,
            &b,
            BoundingBoxOptions {
                expand_by_tolerance: false,
                ..BoundingBoxOptions::default()
            },
        )
        .unwrap();
        assert_eq!(current.min(), Vec3::new(10., 20., 30.));
        assert_eq!(current.max(), current.min());
    }
    #[test]
    fn failed_relock_releases_the_old_override() {
        let k = Kernel::default();
        let mut b = Brep::new();
        b.add_vertex(Point3::ZERO).unwrap();
        lock_bounding_box(&k, &mut b, false).unwrap();
        let s = b.add_shell().unwrap();
        b.add_face(s).unwrap();
        assert!(lock_bounding_box(&k, &mut b, false).is_err());
        assert!(b.locked_bounding_box().is_none());
    }
}
