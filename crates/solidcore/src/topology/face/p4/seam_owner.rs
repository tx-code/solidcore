// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Actual mutable owner execution of the native three-dimensional seam query.
//!
//! Upstream: maintained d8ed45a SmFace.cpp15629--16449,
//! SmCurveClass.cpp14715--14751; SmVertexuse.cpp Get/CalcVertexFaceGap.
//! Deviations: short typed arena borrows replace const native lazy writes.
//! No B-rep copy, global registry, default gap or eager gap preparation is used.
//! The borrowed public APIs retain their existing separate adapter behavior.
//! Mutable generic 2D boundary execution is deliberately not claimed here.

use super::{
    has_seam_problem_into, new_seam_check, seam_handles, HasSeamProblemOptions, SeamCheck,
    SeamCurves, SeamExecution, SeamProblem,
};
use crate::curve::curve::Curve;
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::assert_array::labels;
use crate::geom::attribute::ObjectKey;
use crate::geom::object::ObjectRef;
use crate::heal::face_props::LoopObject;
use crate::intersect::curve_class::p1::{src_zone_tol_3d_of, CurveClassification};
use crate::intersect::curve_class::p4::{
    evaluate_crossing_binormal, map_crossing_sector, LoopCrossingsExecution, UvSectorIo,
};
use crate::intersect::curve_class::p5::{prepare_vertex_intersection, VertexIntersectionExecution};
use crate::intersect::curve_class::p7::{
    EndInside, FaceIntervalEdgeQuery, FaceIntervalExecution, FaceIntervalVertexQuery,
};
use crate::intersect::curve_class::p9::{BoundaryEdge, EdgeIntersectionExecution};
use crate::intersect::point_class::{
    ClassGeometry, ClassObject, ClassObjectKind, PointClassification,
};
use crate::kernel::Kernel;
use crate::math::{Point2, Point3};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::query::face::p2::{curve_on_classify_using, CurveOnClassifyObjects, CurveOnExecution};
use crate::surface::surface::Surface;
use crate::surface::volume::Volume;
use crate::topology::{
    edge, edgeuse::p1 as eu, vertex, vertexuse, Brep, EdgeId, EdgeuseId, FaceId, LoopId, VertexId,
    VertexuseId,
};

const OP: &str = "topology::face::p4::owning_seam_problem";

fn missing() -> SolidError {
    SolidError::invalid_input(OP, "unresolved owning seam object")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum NativeObject {
    Topology(EntityRef),
    Surface(FaceId),
    EdgeCurve(EdgeId),
    Trim(EdgeuseId),
    Temporary(usize),
}

impl ClassObject for NativeObject {
    fn class_kind(&self) -> ClassObjectKind {
        match self {
            Self::Surface(_) => ClassObjectKind::Surface,
            Self::EdgeCurve(_) | Self::Trim(_) | Self::Temporary(_) => ClassObjectKind::Curve,
            Self::Topology(entity) => match entity {
                EntityRef::Face(_) => ClassObjectKind::Face,
                EntityRef::Vertex(_) => ClassObjectKind::Vertex,
                EntityRef::Edge(_) => ClassObjectKind::Edge,
                EntityRef::Loop(_) => ClassObjectKind::Loop,
                EntityRef::Region(_) => ClassObjectKind::Region,
                EntityRef::Faceuse(_) => ClassObjectKind::Faceuse,
                EntityRef::Loopuse(_) => ClassObjectKind::Loopuse,
                EntityRef::Vertexuse(_) => ClassObjectKind::Vertexuse,
                EntityRef::Edgeuse(_) => ClassObjectKind::Edgeuse,
                _ => ClassObjectKind::Other,
            },
        }
    }
    fn temporary(index: usize, kind: ClassObjectKind) -> Option<Self> {
        (kind == ClassObjectKind::Curve).then_some(Self::Temporary(index))
    }
}
impl LoopObject for NativeObject {
    fn from_loop(l: LoopId) -> Self {
        Self::Topology(EntityRef::Loop(l))
    }
    fn as_loop(&self) -> Option<LoopId> {
        if let Self::Topology(EntityRef::Loop(l)) = self {
            Some(*l)
        } else {
            None
        }
    }
    fn as_vertex(&self) -> Option<VertexId> {
        if let Self::Topology(EntityRef::Vertex(v)) = self {
            Some(*v)
        } else {
            None
        }
    }
    fn from_face(f: FaceId) -> Option<Self> {
        Some(Self::Topology(EntityRef::Face(f)))
    }
    fn from_face_surface(f: FaceId) -> Option<Self> {
        Some(Self::Surface(f))
    }
    fn from_topology(entity: EntityRef) -> Option<Self> {
        Some(Self::Topology(entity))
    }
    fn as_topology(&self) -> Option<EntityRef> {
        if let Self::Topology(entity) = self {
            Some(*entity)
        } else {
            None
        }
    }
    fn from_edge_curve(e: EdgeId) -> Option<Self> {
        Some(Self::EdgeCurve(e))
    }
    fn as_edge_curve(&self) -> Option<EdgeId> {
        if let Self::EdgeCurve(e) = self {
            Some(*e)
        } else {
            None
        }
    }
    fn from_trim_curve(eu: EdgeuseId) -> Option<Self> {
        Some(Self::Trim(eu))
    }
    fn as_trim_curve(&self) -> Option<EdgeuseId> {
        if let Self::Trim(eu) = self {
            Some(*eu)
        } else {
            None
        }
    }
}

struct NativeGeometry<'a> {
    brep: &'a Brep,
    curves: Option<&'a SeamCurves>,
    direction: SurfParamType,
}

impl<'a> NativeGeometry<'a> {
    fn resolve_curve(&self, object: NativeObject) -> Option<&'a Curve> {
        match object {
            NativeObject::EdgeCurve(e) => self.brep.edge(e).ok()?.curve.as_ref(),
            NativeObject::Trim(eu_id) => eu::uv_trim_curve(self.brep, eu_id).ok()?.0,
            NativeObject::Temporary(_) => {
                let curves = self.curves?;
                let (curve, uv) = seam_handles::<NativeObject>(self.direction).ok()?;
                if object == curve {
                    Some(&curves.curve)
                } else if object == uv {
                    Some(&curves.uv_curve)
                } else {
                    None
                }
            }
            _ => None,
        }
    }
    fn resolve_surface(&self, object: NativeObject) -> Option<&'a Surface> {
        let NativeObject::Surface(face) = object else {
            return None;
        };
        self.brep.face(face).ok()?.surface.as_ref()
    }
}

impl ClassGeometry<NativeObject> for NativeGeometry<'_> {
    fn curve(&self, object: NativeObject) -> Option<&Curve> {
        self.resolve_curve(object)
    }
    fn surface(&self, object: NativeObject) -> Option<&Surface> {
        self.resolve_surface(object)
    }
    fn volume(&self, _: NativeObject) -> Option<&Volume> {
        None
    }
    fn point(&self, object: NativeObject) -> Option<Point3> {
        let NativeObject::Topology(EntityRef::Vertex(v)) = object else {
            return None;
        };
        Some(self.brep.vertex(v).ok()?.point)
    }
    fn classification_vertex_point(&self, object: NativeObject) -> Option<Point3> {
        self.point(object)
    }
    fn topology_zone_tol_3d(&self, object: NativeObject) -> Option<f64> {
        match object {
            NativeObject::Topology(EntityRef::Vertex(v)) => {
                Some(self.brep.vertex(v).ok()?.tolerance)
            }
            NativeObject::Topology(EntityRef::Edge(e)) => Some(self.brep.edge(e).ok()?.tolerance),
            NativeObject::Topology(EntityRef::Face(f)) => Some(self.brep.face(f).ok()?.tolerance),
            _ => None,
        }
    }
    fn vertex_face_gap(
        &self,
        object: NativeObject,
        _surface: &Surface,
        face: Option<NativeObject>,
    ) -> Option<f64> {
        let NativeObject::Topology(EntityRef::Vertex(v)) = object else {
            return None;
        };
        let NativeObject::Topology(EntityRef::Face(f)) = face? else {
            return None;
        };
        let vu = vertex::vertexuse_of_face(self.brep, v, Some(f)).ok()??;
        if !vertexuse::has_vertex_face_gap(self.brep, vu).ok()? {
            return Some(0.);
        }
        let gap = self.brep.vertexuse(vu).ok()?.vertex_face_gap;
        gap.is_init().then_some(gap.gap_3d)
    }
    fn topology_entity(&self, object: NativeObject) -> Option<(&Brep, EntityRef)> {
        let NativeObject::Topology(entity) = object else {
            return None;
        };
        Some((self.brep, entity))
    }
    fn topology_handle(&self, entity: EntityRef) -> Option<NativeObject> {
        Some(NativeObject::Topology(entity))
    }
    fn owner_of(&self, owner: ObjectRef) -> Option<ObjectRef> {
        if owner.type_code != labels::EDGEUSE_TYPE {
            return None;
        }
        let index = u32::try_from(owner.key.0).ok()?;
        let edge = self.brep.edgeuse(EdgeuseId::from_index(index)).ok()?.edge?;
        Some(ObjectRef {
            type_code: labels::EDGE_TYPE,
            key: ObjectKey(u64::from(edge.index())),
        })
    }
}
impl CurveOnClassifyObjects<NativeObject> for NativeGeometry<'_> {
    fn face(&self, f: FaceId) -> NativeObject {
        NativeObject::Topology(EntityRef::Face(f))
    }
    fn vertex(&self, v: VertexId) -> NativeObject {
        NativeObject::Topology(EntityRef::Vertex(v))
    }
    fn edge(&self, e: EdgeId) -> NativeObject {
        NativeObject::Topology(EntityRef::Edge(e))
    }
    fn vertexuse(&self, vu: VertexuseId) -> NativeObject {
        NativeObject::Topology(EntityRef::Vertexuse(vu))
    }
    fn edgeuse(&self, eu_id: EdgeuseId) -> NativeObject {
        NativeObject::Topology(EntityRef::Edgeuse(eu_id))
    }
    fn edge_curve(&self, e: EdgeId) -> NativeObject {
        NativeObject::EdgeCurve(e)
    }
    fn trim_curve(&self, eu_id: EdgeuseId) -> NativeObject {
        NativeObject::Trim(eu_id)
    }
    fn face_surface(&self, f: FaceId) -> Option<NativeObject> {
        self.brep.face(f).ok()?.surface.as_ref()?;
        Some(NativeObject::Surface(f))
    }
    fn loop_id(&self, object: NativeObject) -> Option<LoopId> {
        if let NativeObject::Topology(EntityRef::Loop(l)) = object {
            Some(l)
        } else {
            None
        }
    }
}

struct OwningSeamExecution<'a> {
    brep: &'a mut Brep,
}
impl SeamExecution<NativeObject> for OwningSeamExecution<'_> {
    type Geometry<'a>
        = NativeGeometry<'a>
    where
        Self: 'a;
    fn brep(&self) -> &Brep {
        self.brep
    }
    fn surface_for_evaluation(&self, face: FaceId) -> Result<Option<Surface>> {
        self.brep
            .face(face)?
            .surface
            .as_ref()
            .map(Surface::clone_for_evaluation)
            .transpose()
    }
    fn geometry<'a>(
        &'a self,
        _face: FaceId,
        direction: SurfParamType,
        curves: Option<&'a SeamCurves>,
    ) -> Result<NativeGeometry<'a>> {
        Ok(NativeGeometry {
            brep: self.brep,
            curves,
            direction,
        })
    }
    fn classify_curve(
        &mut self,
        k: &Kernel,
        face: FaceId,
        direction: SurfParamType,
        curves: &SeamCurves,
        classification: &mut CurveClassification<NativeObject>,
    ) -> Result<()> {
        curve_on_classify_using(
            k,
            &mut OwningCurveOn {
                brep: self.brep,
                face,
                curves,
                direction,
            },
            false,
            classification,
            true,
            false,
            false,
        )
        .map(|_| ())
    }
    fn loop_crossings(
        &mut self,
        k: &Kernel,
        face: FaceId,
        direction: SurfParamType,
        curves: &SeamCurves,
        classification: &CurveClassification<NativeObject>,
        loops: &[LoopId],
        props: &mut crate::heal::face_props::FaceProps<NativeObject>,
    ) -> Result<()> {
        let handles = loops
            .iter()
            .map(|&id| NativeObject::from_loop(id))
            .collect::<Vec<_>>();
        let result = classification.get_loop_crossings_using(
            k,
            &mut OwningCurveOn {
                brep: self.brep,
                face,
                curves,
                direction,
            },
            &handles,
            Some(props),
        );
        let (raw, done) = match result {
            Ok(value) => (value.raw_touches, value.done_touches),
            Err(error) if super::must_propagate(&error) => return Err(error),
            Err(_) => (Vec::new(), Vec::new()),
        };
        if direction == SurfParamType::U {
            props.raw_touch_list_u = raw;
            props.done_touch_list_u = done;
        } else {
            props.raw_touch_list_v = raw;
            props.done_touch_list_v = done;
        }
        Ok(())
    }
}

struct OwningCurveOn<'b, 's> {
    brep: &'b mut Brep,
    face: FaceId,
    curves: &'s SeamCurves,
    direction: SurfParamType,
}
impl<'s> OwningCurveOn<'_, 's> {
    fn objects(&self) -> NativeGeometry<'_> {
        NativeGeometry {
            brep: self.brep,
            curves: Some(self.curves),
            direction: self.direction,
        }
    }
    fn interval_curve(&self, object: NativeObject) -> Result<&'s Curve> {
        // This actual seam pass classifies caller-owned temporary curves. Borrow
        // them independently of the mutable model instead of cloning the model
        // or retaining an immutable arena borrow across native lazy writes.
        let (curve, uv) = seam_handles::<NativeObject>(self.direction)?;
        if object == curve {
            Ok(&self.curves.curve)
        } else if object == uv {
            Ok(&self.curves.uv_curve)
        } else {
            Err(missing())
        }
    }
    fn interval_face(&self, object: NativeObject) -> Result<FaceId> {
        if object == NativeObject::Topology(EntityRef::Face(self.face)) {
            self.brep.face(self.face)?;
            Ok(self.face)
        } else {
            Err(missing())
        }
    }
}

impl FaceIntervalExecution<NativeObject> for OwningCurveOn<'_, '_> {
    fn classification_curve(&self, object: NativeObject) -> Option<&Curve> {
        self.objects().resolve_curve(object)
    }
    fn vertex_sector(
        &mut self,
        k: &Kernel,
        query: &FaceIntervalVertexQuery<NativeObject>,
    ) -> Result<EndInside> {
        let face = self.interval_face(query.face)?;
        let NativeObject::Topology(EntityRef::Vertex(vertex)) = query.vertex else {
            return Err(missing());
        };
        let curve = self.interval_curve(query.curve)?;
        let uv_curve = query
            .uv_curve
            .map(|object| self.interval_curve(object))
            .transpose()?;
        // SmCurveClass.cpp::ClassifyFaceInterval always requests validation of
        // the selected vertex sector, forwards then backwards. Keep the actual
        // arena/context and carry its doubtful-answer flag into the same conflict
        // resolution algorithm; a successful call alone is not an inside answer.
        let sector = super::find_vertex_sector(
            k,
            self.brep,
            face,
            super::FindVertexSectorArgs {
                vertex,
                curve,
                uv_curve,
                param_at_vertex: query.from,
                param_ivl_end: query.to,
                orientation: query.direction,
                zone_tol_3d: query.tolerance,
                validate_sector: true,
                uv_space_okay: query.uv_space_okay,
            },
        )?;
        Ok(EndInside {
            inside: sector.vertexuse.is_some(),
            possible_problem: sector.possible_problem,
        })
    }
    fn edge_side(
        &mut self,
        k: &Kernel,
        query: &FaceIntervalEdgeQuery<NativeObject>,
    ) -> Result<EndInside> {
        let face = self.interval_face(query.face)?;
        let NativeObject::Topology(EntityRef::Edge(edge)) = query.edge else {
            return Err(missing());
        };
        let curve = self.interval_curve(query.curve)?;
        let result = crate::query::face::p1::edge_curve_on_classify(
            k,
            self.brep,
            face,
            Some(edge),
            curve,
            &crate::query::face::p1::EdgeCurveOnOptions {
                edge_param: query.edge_parameter,
                curve_interval: query.interval,
                curve_orientation: query.direction,
                curve_tolerance: query.tolerance,
                uv_space_okay: query.uv_space_okay,
                face_index: query.face_index,
            },
        )?;
        Ok(EndInside {
            inside: result.inside,
            possible_problem: result.possible_problem,
        })
    }
    fn classify_uv(
        &mut self,
        k: &Kernel,
        face: NativeObject,
        uv: Point2,
        tolerance: f64,
        do_xsects: bool,
        use_uv: bool,
        point: &mut PointClassification<NativeObject>,
    ) -> Result<()> {
        let face = self.interval_face(face)?;
        let mut native = point.map_objects(&mut native_entity)?;
        let result = crate::query::face::p2::point_classify(
            k,
            self.brep,
            face,
            uv,
            &mut native,
            crate::query::face::p2::PointClassifyOptions {
                src_zone_tol_3d: tolerance,
                do_boundary_intersections: do_xsects,
                use_uv_classification: use_uv,
                ..Default::default()
            },
        );
        // Native classifications retain writes made before ordinary or unported
        // failure. Copy the complete state back before returning that same status.
        *point = native.map_objects(&mut |entity| Ok(NativeObject::Topology(entity)))?;
        result
    }
    fn classify_point(
        &mut self,
        k: &Kernel,
        face: NativeObject,
        point: Point3,
        tolerance: f64,
        do_xsects: bool,
        classification: &mut PointClassification<NativeObject>,
    ) -> Result<()> {
        let face = self.interval_face(face)?;
        let mut native = classification.map_objects(&mut native_entity)?;
        let result = crate::query::face::p1::point_3d_classify(
            k,
            self.brep,
            face,
            point,
            tolerance,
            do_xsects,
            &mut native,
            crate::query::face::p1::Point3dClassifyOptions::default(),
        );
        *classification = native.map_objects(&mut |entity| Ok(NativeObject::Topology(entity)))?;
        result
    }
}

fn native_entity(object: NativeObject) -> Result<EntityRef> {
    let NativeObject::Topology(entity) = object else {
        return Err(missing());
    };
    Ok(entity)
}
impl LoopCrossingsExecution<NativeObject> for OwningCurveOn<'_, '_> {
    type Geometry<'a>
        = NativeGeometry<'a>
    where
        Self: 'a;
    fn geometry(&self) -> Self::Geometry<'_> {
        self.objects()
    }
    fn vertexuse_sector(
        &mut self,
        k: &Kernel,
        object: NativeObject,
    ) -> Result<UvSectorIo<NativeObject>> {
        let NativeObject::Topology(EntityRef::Vertexuse(id)) = object else {
            return Err(missing());
        };
        // Compute on the same arena at the native call position. Mapping its
        // completed outputs needs only a subsequent short immutable borrow.
        let sector = vertexuse::compute_uv_sector(k, self.brep, id, false)?.io;
        map_crossing_sector(&self.objects(), &sector)
    }
    fn edgeuse_binormal(
        &mut self,
        k: &Kernel,
        object: NativeObject,
        parameter: f64,
        point: &mut Point3,
        binormal: &mut Point3,
    ) -> Result<()> {
        let NativeObject::Topology(EntityRef::Edgeuse(id)) = object else {
            return Err(missing());
        };
        evaluate_crossing_binormal(k, self.brep, id, parameter, point, binormal)
    }
}
impl CurveOnExecution<NativeObject> for OwningCurveOn<'_, '_> {
    fn face_handle(&self) -> NativeObject {
        NativeObject::Topology(EntityRef::Face(self.face))
    }
    fn surface_handle(&self) -> Option<NativeObject> {
        self.objects().face_surface(self.face)
    }
    fn curve(&self, cc: &CurveClassification<NativeObject>) -> Result<&Curve> {
        self.objects()
            .resolve_curve(cc.curve().ok_or_else(missing)?)
            .ok_or_else(missing)
    }
    fn intersect_boundary(
        &mut self,
        k: &Kernel,
        cc: &mut CurveClassification<NativeObject>,
        uv: bool,
        has_big_gaps: bool,
        _do_check_3d: bool,
    ) -> Result<()> {
        if uv {
            return Err(SolidError::unported(
                "SmFace::CurveOnClassify owning 2D boundary execution",
                "query.face_p2",
            ));
        }
        let targets = cc
            .tgt_loops()
            .iter()
            .map(|object| self.objects().loop_id(*object).ok_or_else(missing))
            .collect::<Result<Vec<_>>>()?;
        let vertices = super::vertices(
            self.brep,
            self.face,
            None,
            cc.tgt_loops_only(),
            Some(&targets),
        )?
        .into_iter()
        .map(|v| NativeObject::Topology(EntityRef::Vertex(v)))
        .collect::<Vec<_>>();
        // The public preamble lifts only 2D curves; this actual seam is 3D.
        if self.curve(cc)?.dim() != 3 {
            return Err(missing());
        }
        let source_zone = src_zone_tol_3d_of(k, Some(cc));
        cc.intersect_vertices_using(
            k,
            &vertices,
            &mut OwningVertexPass {
                brep: self.brep,
                face: self.face,
                curves: self.curves,
                direction: self.direction,
                source_zone,
                has_big_gaps,
            },
        )?;
        let edges = super::edges(
            self.brep,
            self.face,
            None,
            cc.tgt_loops_only(),
            Some(&targets),
        )?;
        let source_handle = cc.curve().ok_or_else(missing)?;
        let source_zone = src_zone_tol_3d_of(k, Some(cc));
        cc.intersect_edges_using(
            k,
            &edges,
            &mut OwningEdgePass {
                brep: self.brep,
                face: self.face,
                curves: self.curves,
                direction: self.direction,
                source_handle,
                source_zone,
                has_big_gaps,
            },
        )
    }
    fn classify_interval(
        &mut self,
        k: &Kernel,
        cc: &mut CurveClassification<NativeObject>,
        index: usize,
        do_point_classify: bool,
        uv_space_okay: bool,
    ) -> Result<()> {
        let face = NativeObject::Topology(EntityRef::Face(self.face));
        cc.classify_face_interval_using(
            k,
            self,
            index,
            do_point_classify,
            face,
            None,
            uv_space_okay,
            None,
        )
    }
}

struct OwningVertexPass<'b, 's> {
    brep: &'b mut Brep,
    face: FaceId,
    curves: &'s SeamCurves,
    direction: SurfParamType,
    source_zone: f64,
    has_big_gaps: bool,
}
impl VertexIntersectionExecution<NativeObject> for OwningVertexPass<'_, '_> {
    fn intersect_vertex(
        &mut self,
        k: &Kernel,
        classification: &mut CurveClassification<NativeObject>,
        vertex: NativeObject,
    ) -> Result<()> {
        let input = {
            let objects = NativeGeometry {
                brep: self.brep,
                curves: Some(self.curves),
                direction: self.direction,
            };
            prepare_vertex_intersection(k, &objects, vertex, self.source_zone)?
        };
        let big = if self.has_big_gaps {
            let NativeObject::Topology(EntityRef::Vertex(v)) = vertex else {
                return Err(missing());
            };
            // Do not substitute a nearest UV, a loop occurrence or a ready-cache
            // assumption. Reuse the actual source GetVertexuseOfFace traversal.
            let vu =
                vertex::vertexuse_of_face(self.brep, v, Some(self.face))?.ok_or_else(missing)?;
            // Zero is only the source's known-absent GetVertexFaceGap result.
            // A missing vertexuse relationship already failed above; unavailable
            // numerical calculations propagate through the real getter.
            let gap =
                vertexuse::vertex_face_gap(k, self.brep, vu, false)?.map_or(0., |gap| gap.gap_3d);
            if gap > input.tolerance {
                gap
            } else {
                input.tolerance
            }
        } else {
            input.tolerance
        };
        let objects = NativeGeometry {
            brep: self.brep,
            curves: Some(self.curves),
            direction: self.direction,
        };
        let surface = objects
            .resolve_surface(NativeObject::Surface(self.face))
            .ok_or_else(missing)?;
        let curve = objects
            .resolve_curve(classification.curve().ok_or_else(missing)?)
            .ok_or_else(missing)?;
        classification.intersect_prepared_vertex(
            k,
            &objects,
            surface,
            curve,
            input,
            big,
            self.has_big_gaps,
        )
    }
}

struct OwningEdgePass<'b, 's> {
    brep: &'b mut Brep,
    face: FaceId,
    curves: &'s SeamCurves,
    direction: SurfParamType,
    source_handle: NativeObject,
    source_zone: f64,
    has_big_gaps: bool,
}
impl EdgeIntersectionExecution<NativeObject, EdgeId> for OwningEdgePass<'_, '_> {
    fn intersect_edge(
        &mut self,
        k: &Kernel,
        classification: &mut CurveClassification<NativeObject>,
        edge: EdgeId,
    ) -> Result<()> {
        let mut input = {
            let record = self.brep.edge(edge)?;
            // The source resolves the target curve before fetching the gap.
            record.curve.as_ref().ok_or_else(missing)?;
            BoundaryEdge {
                edge: NativeObject::Topology(EntityRef::Edge(edge)),
                curve: NativeObject::EdgeCurve(edge),
                interval: record.interval,
                zone_tol_3d: record.tolerance,
                face_gap: None,
            }
        };
        if self.has_big_gaps {
            input.face_gap = match edge::edgeuse_of_face(self.brep, edge, Some(self.face))? {
                Some(eu_id) => {
                    eu::max_edge_face_gap(k, self.brep, eu_id, false)?.map(|gap| gap.gap_3d)
                }
                None => None,
            };
        }
        let objects = NativeGeometry {
            brep: self.brep,
            curves: Some(self.curves),
            direction: self.direction,
        };
        let source = objects
            .resolve_curve(self.source_handle)
            .ok_or_else(missing)?;
        classification.intersect_prepared_edge(
            k,
            &objects,
            source,
            input,
            self.source_handle,
            self.source_zone,
            self.has_big_gaps,
        )
    }
}

fn owning_seam_check(k: &Kernel, brep: &mut Brep, face: FaceId) -> Result<SeamCheck<NativeObject>> {
    let mut out = new_seam_check(k);
    has_seam_problem_into(
        k,
        &mut OwningSeamExecution { brep },
        face,
        HasSeamProblemOptions::default(),
        &mut out,
    )?;
    Ok(out)
}

pub(crate) fn owning_seam_problem(
    k: &Kernel,
    brep: &mut Brep,
    face: FaceId,
) -> Result<SeamProblem> {
    Ok(owning_seam_check(k, brep, face)?.problem)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::ellipse::Ellipse;
    use crate::curve::line::Line;
    use crate::geom::axis2_placement::Axis2Placement;
    use crate::geom::extent1d::Extent1d;
    use crate::heal::face_props::{FaceProps, HealerOpType, PropStage};
    use crate::intersect::curve_class::p2::ResetOptions;
    use crate::intersect::point_class::PointClassType;
    use crate::math::{Point2, Vec3};
    use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
    use crate::surface::cone::Cone;
    use crate::topology::brep_constructor::{BrepConstructor, VertexNode};

    // Independently designed full cylindrical side. One seam edge is reused
    // in opposite directions; each end circle is a closed edge on one vertex.
    fn cylinder() -> (Kernel, Brep, FaceId, [VertexId; 2]) {
        let mut k = Kernel::default();
        let mut builder = BrepConstructor::new().unwrap();
        let bottom = builder.start_vertex(Point3::new(2., 0., 0.)).unwrap();
        builder.end_vertex();
        let top = builder.start_vertex(Point3::new(2., 0., 3.)).unwrap();
        builder.end_vertex();
        builder.start_shell().unwrap();
        let face = builder.start_face(OrientType::Same).unwrap();
        let mut s = Cone::new_cylinder_bounded(
            Point3::ZERO,
            Vec3::X,
            Vec3::Y,
            2.,
            0.,
            360.,
            3.,
            false,
            false,
            false,
        );
        s.make_nurb(&k).unwrap();
        let s = Surface::from(s);
        let domain = s.natural_uv_domain();
        builder.set_face_surface(s, domain).unwrap();
        builder.start_loop(OrientType::Same).unwrap();
        let circle = |z: f64| {
            Curve::from(
                Ellipse::create_canonical_circle(
                    &k,
                    &Axis2Placement::new(Point3::new(0., 0., z), Vec3::X, Vec3::Y),
                    2.,
                    Some(&Extent1d::from_min_max(0., 360.)),
                    None,
                )
                .unwrap(),
            )
        };
        let mut seam = None;
        for (index, orientation, first, last) in [
            (0, OrientType::Same, bottom, bottom),
            (1, OrientType::Same, bottom, top),
            (2, OrientType::Opposite, top, top),
            (3, OrientType::Opposite, bottom, top),
        ] {
            let id = builder
                .start_edge(
                    &k,
                    orientation,
                    if index == 3 { seam } else { None },
                    Some(first),
                    Some(last),
                )
                .unwrap();
            if index != 3 {
                let c = if index == 0 {
                    circle(0.)
                } else if index == 2 {
                    circle(3.)
                } else {
                    Curve::from(Line::from_points(
                        Point3::new(2., 0., 0.),
                        Point3::new(2., 0., 3.),
                        3,
                    ))
                };
                let interval = c.natural_interval();
                builder.set_edge_curve(&k, c, false).unwrap();
                builder
                    .set_limits(
                        &k,
                        VertexNode::Parameter(interval.min()),
                        VertexNode::Parameter(interval.max()),
                    )
                    .unwrap();
                if index == 1 {
                    seam = Some(id);
                }
            }
            builder.end_edge().unwrap();
        }
        builder.end_loop(&k).unwrap();
        builder.end_face().unwrap();
        builder.end_shell(&mut k, false).unwrap();
        let mut b = builder.finish().unwrap();
        crate::topology::face::p3::create_uv_trim_curves(&k, &mut b, face, false, None).unwrap();
        (k, b, face, [bottom, top])
    }

    fn seam_class(
        k: &Kernel,
        b: &Brep,
        face: FaceId,
    ) -> (SeamCurves, CurveClassification<NativeObject>) {
        let surface = b.face(face).unwrap().surface.as_ref().unwrap();
        let seam = surface
            .create_seam_curve(k, SurfParamType::U, true)
            .unwrap()
            .unwrap();
        let uv = seam.uv_curve().clone();
        let interval = seam.natural_interval();
        let (curve_handle, uv_handle) = seam_handles::<NativeObject>(SurfParamType::U).unwrap();
        let mut class = CurveClassification::with_curve_interval(
            k,
            curve_handle,
            interval,
            b.face(face).unwrap().tolerance,
        );
        class.set_uv_curve1(Some(uv_handle));
        class.set_surface1(Some(NativeObject::Surface(face)));
        class.set_is_curve_owned(true);
        class.set_ok_to_set_tolerance(false);
        class.reset(k, &ResetOptions::default()).unwrap();
        class.set_face(Some(NativeObject::Topology(EntityRef::Face(face))));
        (
            SeamCurves {
                curve: Curve::from(seam),
                uv_curve: uv,
            },
            class,
        )
    }

    fn gap_state(b: &Brep, vu: VertexuseId) -> (Option<VertexuseId>, Point2, f64) {
        let gap = b.vertexuse(vu).unwrap().vertex_face_gap;
        (gap.vertexuse, gap.face_uv, gap.gap_3d)
    }

    #[test]
    fn owning_cylinder_seam_executes_native_stage2_and_keeps_success_boolean_convention() {
        let (k, mut b, face, _) = cylinder();
        // The independent SDK observer clears these after UV construction, so
        // cache readiness proves that the actual Stage2 call did the lazy work.
        let vertices = b.vertexuses().iter().map(|(id, _)| id).collect::<Vec<_>>();
        let edges = b.edgeuses().iter().map(|(id, _)| id).collect::<Vec<_>>();
        assert_eq!((vertices.len(), edges.len()), (8, 8));
        for vu in vertices {
            vertexuse::init_vertex_face_gap(&mut b, vu).unwrap();
        }
        for eu_id in edges {
            b.edgeuse_mut(eu_id).unwrap().max_edge_face_gap.reset();
        }
        let check = owning_seam_check(&k, &mut b, face).unwrap();
        assert_eq!(check.problem.crossed, SurfParamType::Neither);
        assert_eq!(check.problem.missing, SurfParamType::Neither);
        assert_eq!(check.problem.near_miss, SurfParamType::Neither);
        // Native Stage2 returns SM_SUCCESS (1000) as SmBoolean here.
        assert!(check.has_problem);
        assert!(check.seam_u.is_some());
        assert!(check.seam_v.is_none());
        assert_eq!(
            b.vertexuses()
                .iter()
                .filter(|(_, vu)| vu.vertex_face_gap.is_init())
                .count(),
            2
        );
        assert_eq!(
            b.edgeuses()
                .iter()
                .filter(|(_, eu)| eu.max_edge_face_gap.is_init())
                .count(),
            6
        );
    }

    #[test]
    fn shared_seam_controller_keeps_ready_public_cylinder_verdicts_for_both_actual_adapters() {
        let (k, mut borrowed_model, face, vertices) = cylinder();
        for v in vertices {
            let vu = vertex::vertexuse_of_face(&borrowed_model, v, Some(face))
                .unwrap()
                .unwrap();
            vertexuse::vertex_face_gap(&k, &mut borrowed_model, vu, false)
                .unwrap()
                .unwrap();
        }
        let objects = NativeGeometry {
            brep: &borrowed_model,
            curves: None,
            direction: SurfParamType::U,
        };
        let borrowed = super::super::has_seam_problem(
            &k,
            &borrowed_model,
            face,
            &objects,
            HasSeamProblemOptions::default(),
        )
        .unwrap();
        // Independently construct the second model; never clone a topology graph.
        let (owning_kernel, mut owning_model, owning_face, _) = cylinder();
        let owned = owning_seam_check(&owning_kernel, &mut owning_model, owning_face).unwrap();
        assert_eq!(borrowed.problem.crossed, owned.problem.crossed);
        assert_eq!(borrowed.problem.missing, owned.problem.missing);
        assert_eq!(borrowed.problem.near_miss, owned.problem.near_miss);
        assert_eq!(borrowed.class_u.size(), owned.class_u.size());
        assert_eq!(borrowed.has_problem, owned.has_problem);
        // This numerical/controller comparison deliberately does not qualify
        // the legacy borrowed adapter's discarded lazy cache side effects.
    }

    #[test]
    fn owning_seam_vertex_reuses_ready_gap_without_recomputing_siblings() {
        let (k, mut b, face, vertices) = cylinder();
        let vertex = vertices[0];
        let vu = vertex::vertexuse_of_face(&b, vertex, Some(face))
            .unwrap()
            .unwrap();
        vertexuse::vertex_face_gap(&k, &mut b, vu, false)
            .unwrap()
            .unwrap();
        let siblings = vertex::vertexuses_of_face(&b, vertex, Some(face), None).unwrap();
        let before = siblings
            .iter()
            .map(|&id| (id, gap_state(&b, id)))
            .collect::<Vec<_>>();
        let (curves, mut class) = seam_class(&k, &b, face);
        let source_zone = src_zone_tol_3d_of(&k, Some(&class));
        class
            .intersect_vertices_using(
                &k,
                &[NativeObject::Topology(EntityRef::Vertex(vertex))],
                &mut OwningVertexPass {
                    brep: &mut b,
                    face,
                    curves: &curves,
                    direction: SurfParamType::U,
                    source_zone,
                    has_big_gaps: true,
                },
            )
            .unwrap();
        for (id, expected) in before {
            assert_eq!(gap_state(&b, id), expected);
        }
    }

    #[test]
    fn owning_seam_lazy_vertex_gap_initializes_only_selected_use_and_clears_siblings() {
        let (k, mut b, face, vertices) = cylinder();
        let vertex = vertices[0];
        let selected = vertex::vertexuse_of_face(&b, vertex, Some(face))
            .unwrap()
            .unwrap();
        let sibling = vertex::vertexuses_of_face(&b, vertex, Some(face), None)
            .unwrap()
            .into_iter()
            .find(|&id| id != selected)
            .unwrap();
        vertexuse::vertex_face_gap(&k, &mut b, sibling, false)
            .unwrap()
            .unwrap();
        assert!(!b.vertexuse(selected).unwrap().vertex_face_gap.is_init());
        assert!(b.vertexuse(sibling).unwrap().vertex_face_gap.is_init());
        let (curves, mut class) = seam_class(&k, &b, face);
        let source_zone = src_zone_tol_3d_of(&k, Some(&class));
        class
            .intersect_vertices_using(
                &k,
                &[NativeObject::Topology(EntityRef::Vertex(vertex))],
                &mut OwningVertexPass {
                    brep: &mut b,
                    face,
                    curves: &curves,
                    direction: SurfParamType::U,
                    source_zone,
                    has_big_gaps: true,
                },
            )
            .unwrap();
        assert!(b.vertexuse(selected).unwrap().vertex_face_gap.is_init());
        assert!(!b.vertexuse(sibling).unwrap().vertex_face_gap.is_init());
    }

    #[test]
    fn owning_seam_vertex_failure_retains_earlier_classification_and_never_prepares_later_gap() {
        let (k, mut b, face, vertices) = cylinder();
        let first = vertex::vertexuse_of_face(&b, vertices[0], Some(face))
            .unwrap()
            .unwrap();
        let later = vertex::vertexuse_of_face(&b, vertices[1], Some(face))
            .unwrap()
            .unwrap();
        let before_later = gap_state(&b, later);
        let (curves, mut class) = seam_class(&k, &b, face);
        let source_zone = src_zone_tol_3d_of(&k, Some(&class));
        let inputs = [
            NativeObject::Topology(EntityRef::Vertex(vertices[0])),
            NativeObject::Topology(EntityRef::Vertex(VertexId::from_index(u32::MAX))),
            NativeObject::Topology(EntityRef::Vertex(vertices[1])),
        ];
        assert!(class
            .intersect_vertices_using(
                &k,
                &inputs,
                &mut OwningVertexPass {
                    brep: &mut b,
                    face,
                    curves: &curves,
                    direction: SurfParamType::U,
                    source_zone,
                    has_big_gaps: true
                }
            )
            .is_err());
        assert!(b.vertexuse(first).unwrap().vertex_face_gap.is_init());
        assert_eq!(gap_state(&b, later), before_later);
        assert!(class
            .intervals()
            .iter()
            .any(
                |interval| interval.start.point_class() == PointClassType::Vertex
                    || interval.end.point_class() == PointClassType::Vertex
            ));
    }

    #[test]
    fn owning_seam_stage2_failure_commits_callback_classifications_and_keeps_stage2a() {
        let (k, b, face, _) = cylinder();
        let (_, class) = seam_class(&k, &b, face);
        let expected_size = class.size();
        let mut props = FaceProps::new();
        let mut classes = props
            .prepare_props_stage2(&k, &b, face, 0, HealerOpType::All)
            .unwrap()
            .unwrap();
        assert_eq!(props.prop_stage, PropStage::Stage2A);
        classes.class_u = class;
        assert!(props
            .finish_props_stage2(
                classes,
                Err(SolidError::unported(
                    "synthetic seam callback dependency",
                    "topology.face_p4"
                ))
            )
            .is_err());
        assert_eq!(props.prop_stage, PropStage::Stage2A);
        assert_eq!(props.crv_class_u.as_ref().unwrap().size(), expected_size);
    }
}
