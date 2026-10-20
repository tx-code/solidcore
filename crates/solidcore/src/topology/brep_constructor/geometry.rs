// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Geometry transfer and original constructor endpoint interpretation.
//! Upstream: `SmBrepData.cpp::SetFaceSurface/SetEdgeCurve/SetLimits` at `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`BrepConstructor::set_face_surface`] | `SetFaceSurface` |
//! | [`BrepConstructor::set_edge_curve`] | `SetEdgeCurve` |
//! | [`BrepConstructor::set_limits`] | `SetLimits` |
//!
//! Deviations: owned Rust geometry prevents shared-pointer aliases. The UV setter
//! retains its original current-use assignment; canonical SAME installation is a
//! separate edge-use operation. Source import using explicit parameters does not
//! acquire the legacy point-drop retry/snapping policy accidentally.
use super::{eu, invalid, BrepConstructor, Kernel, Point3, Result, VertexNode};
use crate::curve::curve::Curve;
use crate::geom::attribute::ObjectKey;
use crate::geom::extent2d::Extent2d;
use crate::geom::global_solver::SolverOperation;
use crate::geom::object::ObjectRef;
use crate::surface::surface::Surface;
use crate::topology::brep_cache::{arena_identity, CacheObject};

impl BrepConstructor {
    /// Transfer one surface to the active face and retain its explicit UV domain.
    /// To retain another face's geometry, copy it before transfer; Rust values
    /// cannot alias the C++ borrowed-pointer/copy branch or slice its concrete kind.
    pub fn set_face_surface(&mut self, mut surface: Surface, domain: Extent2d) -> Result<()> {
        let face = self
            .face
            .ok_or_else(|| invalid("surface requires active face"))?;
        surface.set_owner(Some(arena_identity(CacheObject::Face(face))));
        let record = self.brep.face_mut(face)?;
        record.surface = Some(surface);
        record.uv_domain = domain;
        Ok(())
    }
    /// Transfer defining 3D geometry, or assign the current incidence's UV curve
    /// and lift missing 3D geometry through the actual kernel surface operation.
    /// Existing 3D geometry is not replaced; lifting failures remain failures.
    pub fn set_edge_curve(&mut self, k: &Kernel, mut curve: Curve, uv: bool) -> Result<()> {
        let edge = self
            .edge
            .ok_or_else(|| invalid("curve requires active edge"))?;
        let incidence = Self::required_use(self.edgeuse)?;
        if !uv {
            if self.brep.edge(edge)?.curve.is_some() {
                return Err(invalid("defining curve already assigned"));
            }
            curve.set_owner(
                Some(arena_identity(CacheObject::Edge(edge))),
                Some(arena_identity(CacheObject::Brep)),
            );
            self.brep.edge_mut(edge)?.curve = Some(curve);
            return Ok(());
        }
        if curve.dim() != 2 {
            return Err(invalid("UV trim curve must have dimension two"));
        }
        let face =
            eu::face(&self.brep, incidence)?.ok_or_else(|| invalid("UV edge use has no face"))?;
        curve.set_owner(
            Some(ObjectRef {
                type_code: eu::TYPE,
                key: ObjectKey(u64::from(incidence.index())),
            }),
            None,
        );
        self.brep.edgeuse_mut(incidence)?.uv_trim_curve = Some(curve);
        if self.brep.edge(edge)?.curve.is_none() {
            let domain = self.brep.face(face)?.uv_domain;
            // Suppress the trimmed-surface owner cache only for the lift, then
            // restore it on both success and error before returning control.
            let mut surface = self
                .brep
                .face_mut(face)?
                .surface
                .take()
                .ok_or_else(|| invalid("UV lift requires a surface"))?;
            let uv_curve = self
                .brep
                .edgeuse(incidence)?
                .uv_trim_curve
                .as_ref()
                .ok_or_else(|| invalid("missing UV curve"))?;
            let interval = uv_curve.natural_interval();
            let tolerance = self.brep.edge(edge)?.tolerance;
            let owner = surface.set_owner(None);
            let lifted = surface.lift_curve(k, &domain, uv_curve, &interval, tolerance, true);
            surface.set_owner(owner);
            self.brep.face_mut(face)?.surface = Some(surface);
            let mut lifted = lifted?.curve;
            lifted.set_owner(
                Some(arena_identity(CacheObject::Edge(edge))),
                Some(arena_identity(CacheObject::Brep)),
            );
            self.brep.edge_mut(edge)?.curve = Some(lifted);
        }
        Ok(())
    }
    /// Set the underlying curve's increasing bounds. Explicit parameters bypass
    /// point dropping. Point/UV nodes use the original 1x/100x drop retry, followed
    /// by strict-distance endpoint snapping; this is not importer repair policy.
    pub fn set_limits(&mut self, k: &Kernel, start: VertexNode, end: VertexNode) -> Result<()> {
        let edge = self
            .edge
            .ok_or_else(|| invalid("limits require active edge"))?;
        let record = self.brep.edge(edge)?;
        let curve = record
            .curve
            .as_ref()
            .ok_or_else(|| invalid("limits require defining curve"))?;
        let min = Self::parameter(k, curve, record.tolerance, start, true)?;
        let max = Self::parameter(k, curve, record.tolerance, end, false)?;
        self.brep.edge_mut(edge)?.interval.set_min_max(min, max)
    }
    fn parameter(
        k: &Kernel,
        curve: &Curve,
        tolerance: f64,
        node: VertexNode,
        start: bool,
    ) -> Result<f64> {
        let point = match node {
            VertexNode::Parameter(value) => return Ok(value),
            VertexNode::Point(point) => point,
            VertexNode::Uv(point) => Point3::new(point.x, point.y, 0.),
        };
        let interval = curve.natural_interval();
        let mut found = curve.drop_point(
            k,
            &interval,
            point,
            None,
            tolerance,
            None,
            SolverOperation::Minimize,
        )?;
        if !found.success {
            found = curve.drop_point(
                k,
                &interval,
                point,
                None,
                tolerance * 100.,
                None,
                SolverOperation::Minimize,
            )?;
            if !found.success {
                return Err(invalid("curve point drop did not find a parameter"));
            }
        }
        let endpoint = if start {
            interval.min()
        } else {
            interval.max()
        };
        // Upstream ignores these two evaluation statuses. Unported remains an
        // explicit gap; other errors retain an undefined point and do not snap.
        let a = crate::error::ignored(curve.evaluate_point(found.parameter))?;
        let b = crate::error::ignored(curve.evaluate_point(endpoint))?;
        if let (Some(a), Some(b)) = (a, b) {
            if a.distance_squared(b) < tolerance * tolerance {
                return Ok(endpoint);
            }
        }
        Ok(found.parameter)
    }
}
