//! Scoped topology access for a surface's subdivision and trim caches.
//!
//! Upstream: `source/SMLib/src/SmSurfaceCache.cpp`, `SmTrimSrfCache.cpp`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`CacheTopology`] | owner calls reached by `SmTrimSrfCache` / `SmSurfaceCache` |
//!
//! Deviations: native owner pointers become a caller-borrowed resolver. The cache
//! never owns the `BRep`, detaches geometry, or retains a tolerance snapshot.

use crate::error::{Result, SolidError};
use crate::kernel::Kernel;
use crate::math::Point2;
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::surface::surface::Surface;

/// The live topology operations used by `SmTrimSrfCache`. Each call must resolve
/// handles in one model; working tolerances are not import acceptance budgets.
/// Missing owners remain errors rather than falling back to system tolerances.
pub trait CacheTopology<O: Copy + PartialEq> {
    /// `GetSurface()->GetFace()` in the cache's handle space.
    fn face(&mut self, surface: &Surface) -> Result<Option<O>>;
    /// `SmTol::GetZoneTol3d(pFace)`.
    fn face_tolerance(&mut self, face: O) -> Result<f64>;
    /// Native pole calculation using the actual face tolerance where owned.
    fn singularities(&mut self, k: &Kernel, surface: &Surface) -> Result<u32> {
        let value = if let Some(face) = self.face(surface)? {
            surface.get_singularities_of_face(k, self.face_tolerance(face)?, false, false)?
        } else {
            surface.get_singularities(k, false, false)?
        };
        Ok(value.singular)
    }
    /// `face->GetBrep()->m_bEditingEnabled`.
    fn editing_enabled(&mut self, face: O) -> Result<bool>;
    /// Temporary assignment of the owning model's editing flag.
    fn set_editing_enabled(&mut self, face: O, enabled: bool) -> Result<()>;
    /// `CreateUVTrimCurves(TRUE, NULL, NULL, ...)`, including native side effects.
    fn create_uv_trim_curves(&mut self, k: &Kernel, face: O) -> Result<()>;
    /// Upward loop uses in native traversal order.
    fn upward_loopuses(&mut self, face: O) -> Result<Vec<O>>;
    /// Edge uses in the loop's native order.
    fn edgeuses(&mut self, loopuse: O) -> Result<Vec<O>>;
    /// Exact B-spline view of the stored UV trim, without copy-constructor fixup.
    fn uv_trim_curve(&mut self, edgeuse: O) -> Result<Option<BSplineCurve>>;
    /// Vertex uses in native traversal order.
    fn vertexuses(&mut self, loopuse: O) -> Result<Vec<O>>;
    /// `SmVertexuse::ComputeUVPoint`.
    fn uv_point(&mut self, k: &Kernel, vertexuse: O) -> Result<Point2>;
    /// `vertexuse->GetVertex()->GetTolerance()`.
    fn vertex_tolerance(&mut self, vertexuse: O) -> Result<f64>;
    /// `SmFace::PointClassify`; flags are `(do_boundary_intersections, use_uv_classification)`.
    fn point_classify(
        &mut self,
        k: &Kernel,
        face: O,
        uv: Point2,
        tol: f64,
        flags: (bool, bool),
    ) -> Result<(i32, Option<O>)>;
}

/// Legacy standalone callers have no model lookup. Owned paths keep their
/// original explicit missing-dependency errors until a live resolver is supplied.
pub struct UnresolvedTopology;

impl<O: Copy + PartialEq> CacheTopology<O> for UnresolvedTopology {
    fn face(&mut self, surface: &Surface) -> Result<Option<O>> {
        match surface.face()? {
            None => Ok(None),
            // UNPORTED(topology.face_p2): cache handle without model lookup.
            Some(_) => Err(SolidError::unported(
                "SmFace handle of a surface's face",
                "topology.face_p2",
            )),
        }
    }
    fn face_tolerance(&mut self, _: O) -> Result<f64> {
        // UNPORTED(geom.tol): owned tolerance without model lookup.
        Err(SolidError::unported(
            "SmTol::GetZoneTol3d(const SmObject*)",
            "geom.tol",
        ))
    }
    fn editing_enabled(&mut self, _: O) -> Result<bool> {
        // UNPORTED(topology.face_p4): owning model without lookup.
        Err(SolidError::unported("SmFace::GetBrep", "topology.face_p4"))
    }
    fn set_editing_enabled(&mut self, _: O, _: bool) -> Result<()> {
        // UNPORTED(topology.face_p4): owning model without lookup.
        Err(SolidError::unported("SmFace::GetBrep", "topology.face_p4"))
    }
    fn create_uv_trim_curves(&mut self, _: &Kernel, _: O) -> Result<()> {
        // UNPORTED(topology.face_p3): live face required by the existing operation.
        Err(SolidError::unported(
            "SmFace::CreateUVTrimCurves",
            "topology.face_p3",
        ))
    }
    fn upward_loopuses(&mut self, _: O) -> Result<Vec<O>> {
        // UNPORTED(topology.face_p4): live face required for traversal.
        Err(SolidError::unported(
            "SmFace::GetUpwardLoopuses",
            "topology.face_p4",
        ))
    }
    fn edgeuses(&mut self, _: O) -> Result<Vec<O>> {
        // UNPORTED(topology.loopuse): live loop use required for traversal.
        Err(SolidError::unported(
            "SmLoopuse::GetEdgeuses",
            "topology.loopuse",
        ))
    }
    fn uv_trim_curve(&mut self, _: O) -> Result<Option<BSplineCurve>> {
        // UNPORTED(topology.edgeuse_p1): live edge use required for stored trim lookup.
        Err(SolidError::unported(
            "SmEdgeuse::GetUVTrimCurve",
            "topology.edgeuse_p1",
        ))
    }
    fn vertexuses(&mut self, _: O) -> Result<Vec<O>> {
        // UNPORTED(topology.loopuse): live loop use required for traversal.
        Err(SolidError::unported(
            "SmLoopuse::GetVertexuses",
            "topology.loopuse",
        ))
    }
    fn uv_point(&mut self, _: &Kernel, _: O) -> Result<Point2> {
        // UNPORTED(topology.vertexuse): live vertex use required by the existing operation.
        Err(SolidError::unported(
            "SmVertexuse::ComputeUVPoint",
            "topology.vertexuse",
        ))
    }
    fn vertex_tolerance(&mut self, _: O) -> Result<f64> {
        // UNPORTED(topology.vertex): live vertex required for tolerance lookup.
        Err(SolidError::unported(
            "SmVertex::GetTolerance",
            "topology.vertex",
        ))
    }
    fn point_classify(
        &mut self,
        _: &Kernel,
        _: O,
        _: Point2,
        _: f64,
        _: (bool, bool),
    ) -> Result<(i32, Option<O>)> {
        // UNPORTED(query.face_p2): live face required by the existing classifier.
        Err(SolidError::unported(
            "SmFace::PointClassify",
            "query.face_p2",
        ))
    }
}
