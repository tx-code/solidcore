// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual surface-cache ownership for native B-rep ray intersections.
//!
//! Upstream: maintained `SmSurface.cpp::GlobalLineIntersect`,
//! `SmTrimSrfCache.cpp`, `SmSurfaceCache.cpp`, `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | `line_intersections` | native owning surface cache before the line/tree solve |
//! | `RayCacheTopology` | live owner calls in the ray solver's handle namespace |
//!
//! Deviations: cache-only loop/use handles use the model's checked topology
//! namespace. Geometry owners keep their separate original raw-index namespace.
//! Every callback delegates to the existing live B-rep surface context; no model
//! copies, parallel topology interpretations or default tolerances are introduced.

use crate::error::{EntityRef, Result, SolidError};
use crate::geom::assert_array::labels::{EDGEUSE_TYPE, LOOPUSE_TYPE, VERTEXUSE_TYPE};
use crate::geom::solution_array::SolutionArray;
use crate::intersect::topology_solver::p1::PointFaceRayRequest;
use crate::kernel::Kernel;
use crate::math::{Point2, EFF_ZERO_SQ};
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::surface::cache_topology::{CacheTopology, OwnedSurfaceCache};
use crate::surface::surface::p1::get_owned_surface_cache_with_topology;
use crate::surface::surface::{Surface, SurfaceKind};
use crate::topology::brep_cache::CacheObject;
use crate::topology::surface_cache::BrepSurfaceContext;
use crate::topology::{Brep, EdgeuseId, LoopuseId, VertexuseId};

const OP: &str = "topology::brep::p2::surface_owner";

fn wrong_handle() -> SolidError {
    SolidError::invalid_input(
        OP,
        "surface cache handle is outside the scoped topology namespace",
    )
}

pub(super) fn line_intersections(
    k: &Kernel,
    b: &mut Brep,
    request: &PointFaceRayRequest<CacheObject>,
) -> Result<SolutionArray<CacheObject>> {
    let CacheObject::Face(face) = request.face else {
        return Err(wrong_handle());
    };
    if request.face_surface != CacheObject::Surface(face) {
        return Err(wrong_handle());
    }
    let surface = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer(OP))?
        .clone_for_evaluation()?;
    // Preserve the existing virtual plane path, which does not request a cache
    // for its ordinary transverse line solve. All other current Surface kinds
    // use the shared base line/tree body in Surface::global_line_intersect_in.
    if matches!(surface.kind(), SurfaceKind::Plane(_)) {
        return surface.global_line_intersect(
            k,
            request.face_surface,
            &request.domain,
            request.point,
            request.direction,
            None,
            request.fire_ray,
            request.tolerance,
        );
    }
    // Match the base entry's guard before allocating or building any cache.
    if request.direction.length_squared() < EFF_ZERO_SQ {
        return Err(SolidError::invalid_input(
            "surface::surface::p8::Surface::base_global_line_intersect",
            "zero line vector",
        ));
    }
    let mut topology = RayCacheTopology {
        inner: BrepSurfaceContext { brep: b, face },
        surface_handle: request.face_surface,
    };
    let mut cache = get_owned_surface_cache_with_topology(
        k,
        &surface,
        Some(request.face_surface),
        None,
        &mut topology,
    )?;
    // Keep the actual plain/trimmed allocation, reuse flags and native GetTree
    // predicate. The ready entry below does not redo this through UnresolvedTopology.
    cache.get_tree_with_topology(k, &mut topology)?;
    let face_tolerance = topology.face_tolerance(request.face)?;
    surface.base_global_line_intersect_ready_cache(
        k,
        Some(request.face_surface),
        cache.base(),
        Some(face_tolerance),
        &request.domain,
        request.point,
        request.direction,
        None,
        request.fire_ray,
        request.tolerance,
    )
}

struct RayCacheTopology<'a> {
    inner: BrepSurfaceContext<'a>,
    surface_handle: CacheObject,
}

impl RayCacheTopology<'_> {
    fn encode(&self, entity: EntityRef) -> Result<CacheObject> {
        // Face/edge/vertex already have native cache handles. Uses are cache
        // metadata only, encoded with the model's actual notification identity.
        let object = super::object_ref(self.inner.brep, entity)?;
        match entity {
            EntityRef::Face(face) => Ok(CacheObject::Face(face)),
            EntityRef::Edge(edge) => Ok(CacheObject::Edge(edge)),
            EntityRef::Vertex(vertex) => Ok(CacheObject::Vertex(vertex)),
            EntityRef::Loopuse(_) | EntityRef::Edgeuse(_) | EntityRef::Vertexuse(_) => {
                Ok(CacheObject::Other(object))
            }
            _ => Err(wrong_handle()),
        }
    }

    fn decode(&self, object: CacheObject) -> Result<EntityRef> {
        let entity = match object {
            CacheObject::Face(face) => EntityRef::Face(face),
            CacheObject::Edge(edge) => EntityRef::Edge(edge),
            CacheObject::Vertex(vertex) => EntityRef::Vertex(vertex),
            CacheObject::Other(reference) => {
                let kind = u64::try_from(reference.type_code).map_err(|_| wrong_handle())?;
                if reference.key.0 >> 32 != kind {
                    return Err(wrong_handle());
                }
                let index = u32::try_from(reference.key.0 & u64::from(u32::MAX))
                    .map_err(|_| wrong_handle())?;
                let entity = match reference.type_code {
                    LOOPUSE_TYPE => EntityRef::Loopuse(LoopuseId::from_index(index)),
                    EDGEUSE_TYPE => EntityRef::Edgeuse(EdgeuseId::from_index(index)),
                    VERTEXUSE_TYPE => EntityRef::Vertexuse(VertexuseId::from_index(index)),
                    _ => return Err(wrong_handle()),
                };
                if super::object_ref(self.inner.brep, entity)? != reference {
                    return Err(wrong_handle());
                }
                entity
            }
            _ => return Err(wrong_handle()),
        };
        // Validate liveness for both typed and metadata handles. These handles
        // are generated and consumed within this one model-bound cache query;
        // raw geometry-owner keys are not interchangeable with use metadata.
        super::object_ref(self.inner.brep, entity)?;
        Ok(entity)
    }

    fn encode_list(&self, entities: Vec<EntityRef>) -> Result<Vec<CacheObject>> {
        entities
            .into_iter()
            .map(|entity| self.encode(entity))
            .collect()
    }
}

impl CacheTopology<CacheObject> for RayCacheTopology<'_> {
    fn face(&mut self, surface: &Surface) -> Result<Option<CacheObject>> {
        let face = CacheTopology::<EntityRef>::face(&mut self.inner, surface)?;
        face.map(|face| self.encode(face)).transpose()
    }
    fn face_tolerance(&mut self, face: CacheObject) -> Result<f64> {
        let face = self.decode(face)?;
        CacheTopology::<EntityRef>::face_tolerance(&mut self.inner, face)
    }
    fn point_solver_face_tolerance(&mut self, surface: &Surface) -> Result<Option<f64>> {
        CacheTopology::<EntityRef>::point_solver_face_tolerance(&mut self.inner, surface)
    }
    fn point_solver_cache(
        &mut self,
        k: &Kernel,
        surface: &Surface,
    ) -> Result<OwnedSurfaceCache<CacheObject>> {
        get_owned_surface_cache_with_topology(k, surface, Some(self.surface_handle), None, self)
    }
    fn editing_enabled(&mut self, face: CacheObject) -> Result<bool> {
        let face = self.decode(face)?;
        CacheTopology::<EntityRef>::editing_enabled(&mut self.inner, face)
    }
    fn set_editing_enabled(&mut self, face: CacheObject, enabled: bool) -> Result<()> {
        let face = self.decode(face)?;
        CacheTopology::<EntityRef>::set_editing_enabled(&mut self.inner, face, enabled)
    }
    fn create_uv_trim_curves(&mut self, k: &Kernel, face: CacheObject) -> Result<()> {
        let face = self.decode(face)?;
        CacheTopology::<EntityRef>::create_uv_trim_curves(&mut self.inner, k, face)
    }
    fn upward_loopuses(&mut self, face: CacheObject) -> Result<Vec<CacheObject>> {
        let face = self.decode(face)?;
        let uses = CacheTopology::<EntityRef>::upward_loopuses(&mut self.inner, face)?;
        self.encode_list(uses)
    }
    fn edgeuses(&mut self, loopuse: CacheObject) -> Result<Vec<CacheObject>> {
        let loopuse = self.decode(loopuse)?;
        let uses = CacheTopology::<EntityRef>::edgeuses(&mut self.inner, loopuse)?;
        self.encode_list(uses)
    }
    fn uv_trim_curve(&mut self, edgeuse: CacheObject) -> Result<Option<BSplineCurve>> {
        let edgeuse = self.decode(edgeuse)?;
        CacheTopology::<EntityRef>::uv_trim_curve(&mut self.inner, edgeuse)
    }
    fn vertexuses(&mut self, loopuse: CacheObject) -> Result<Vec<CacheObject>> {
        let loopuse = self.decode(loopuse)?;
        let uses = CacheTopology::<EntityRef>::vertexuses(&mut self.inner, loopuse)?;
        self.encode_list(uses)
    }
    fn uv_point(&mut self, k: &Kernel, vertexuse: CacheObject) -> Result<Point2> {
        let vertexuse = self.decode(vertexuse)?;
        CacheTopology::<EntityRef>::uv_point(&mut self.inner, k, vertexuse)
    }
    fn vertex_tolerance(&mut self, vertexuse: CacheObject) -> Result<f64> {
        let vertexuse = self.decode(vertexuse)?;
        CacheTopology::<EntityRef>::vertex_tolerance(&mut self.inner, vertexuse)
    }
    fn point_classify(
        &mut self,
        k: &Kernel,
        face: CacheObject,
        uv: Point2,
        tolerance: f64,
        flags: (bool, bool),
    ) -> Result<(i32, Option<CacheObject>)> {
        let face = self.decode(face)?;
        let (class, object) = CacheTopology::<EntityRef>::point_classify(
            &mut self.inner,
            k,
            face,
            uv,
            tolerance,
            flags,
        )?;
        Ok((class, object.map(|object| self.encode(object)).transpose()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::attribute::ObjectKey;
    use crate::geom::object::ObjectRef;
    use crate::topology::Loopuse;

    #[test]
    fn cache_use_handles_require_the_scoped_topology_namespace_and_live_record() {
        let mut brep = Brep::new_with_infinite_region().unwrap();
        let shell = brep.add_shell().unwrap();
        let face = brep.add_face(shell).unwrap();
        let loopuse = brep.add_loopuse_record(Loopuse::default()).unwrap();
        let context = RayCacheTopology {
            inner: BrepSurfaceContext {
                brep: &mut brep,
                face,
            },
            surface_handle: CacheObject::Surface(face),
        };
        let entity = EntityRef::Loopuse(loopuse);
        let handle = context.encode(entity).unwrap();
        assert_eq!(context.decode(handle).unwrap(), entity);
        // A raw geometry-owner index is not a topology use's registered key.
        assert!(context
            .decode(CacheObject::Other(ObjectRef {
                type_code: LOOPUSE_TYPE,
                key: ObjectKey(u64::from(loopuse.index())),
            }))
            .is_err());
        assert!(context
            .decode(CacheObject::Other(ObjectRef {
                type_code: LOOPUSE_TYPE,
                key: ObjectKey((LOOPUSE_TYPE as u64) << 32 | u64::from(u32::MAX)),
            }))
            .is_err());
    }
}
