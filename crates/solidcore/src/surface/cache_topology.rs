//! Scoped topology access for a surface's subdivision and trim caches.
//!
//! Upstream: `source/SMLib/src/SmSurfaceCache.cpp`, `SmTrimSrfCache.cpp`,
//! `SmSurface.cpp::CacheMakeOrValidate`, `SmCacheMgr.cpp` (default single slot).
//! | Rust | Upstream |
//! |------|----------|
//! | [`CacheTopology`] | owner calls reached by `SmTrimSrfCache` / `SmSurfaceCache` |
//! | [`OwnedSurfaceCache`], [`SurfaceCacheView`] | owning/borrowed dynamic surface-cache role |
//! | [`OwnedSurfaceCache::get_tree_with_topology`] | virtual `SmSurfaceCache::GetTree` |
//!
//! Deviations: native owner pointers become a caller-borrowed resolver. The cache
//! never owns the `BRep`, detaches geometry, or retains a tolerance snapshot.

use crate::error::{Result, SolidError};
use crate::kernel::Kernel;
use crate::math::Point2;
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::surface::surface::Surface;
use crate::surface::surface_cache::p1::{FaceContainment, SurfaceCache};
use crate::surface::trim_srf_cache::TrimSrfCache;

/// The actual dynamic role of an owning native surface cache. Both allocations
/// occupy the one object cache slot used by `SmCacheMgr` without
/// `SM_USE_GLOBAL_CACHE`; a plain fallback remains a plain cache on reuse.
#[derive(Debug)]
pub enum OwnedSurfaceCache<O> {
    /// `SmSurfaceCache`, including a successful fallback after trim build failure.
    Surface(Box<SurfaceCache<O>>),
    /// `SmTrimSrfCache`, including a cache whose Boolean build was delayed.
    Trim(Box<TrimSrfCache<O>>),
}

/// A borrowed cache preserving its actual dynamic role for virtual dispatch.
pub enum SurfaceCacheView<'a, O> {
    /// The plain cache base class.
    Surface(&'a SurfaceCache<O>),
    /// The trimmed cache override.
    Trim(&'a TrimSrfCache<O>),
}

impl<O: Copy + PartialEq> SurfaceCacheView<'_, O> {
    /// The common base state; the returned data alone does not imply trim support.
    pub const fn base(&self) -> &SurfaceCache<O> {
        match self {
            Self::Surface(cache) => cache,
            Self::Trim(cache) => cache.base(),
        }
    }

    /// The trimmed override when the actual object is an `SmTrimSrfCache`.
    pub const fn trimmed(&self) -> Option<&TrimSrfCache<O>> {
        match self {
            Self::Surface(_) => None,
            Self::Trim(cache) => Some(cache),
        }
    }
}

impl<O: Copy + PartialEq> OwnedSurfaceCache<O> {
    /// Common native invalidation predicate, shared by the generic and `BRep`
    /// getters so a reusable slot needs no geometry copy or evaluation.
    pub(crate) fn requires_replacement(&self, editing_disabled: bool) -> bool {
        self.base().face_was_modified && editing_disabled
    }

    /// Borrow the common state while retaining the allocation's dynamic type.
    pub fn view(&self) -> SurfaceCacheView<'_, O> {
        match self {
            Self::Surface(cache) => SurfaceCacheView::Surface(cache),
            Self::Trim(cache) => SurfaceCacheView::Trim(cache),
        }
    }

    /// The base fields shared by both native cache roles.
    pub fn base(&self) -> &SurfaceCache<O> {
        match self {
            Self::Surface(cache) => cache,
            Self::Trim(cache) => cache.base(),
        }
    }

    /// Mutable native base fields, including temporary solver flags.
    pub fn base_mut(&mut self) -> &mut SurfaceCache<O> {
        match self {
            Self::Surface(cache) => cache,
            Self::Trim(cache) => cache.base_mut(),
        }
    }

    /// Native `GetTree` predicate followed by the actual virtual `BuildTree`.
    /// Delayed Boolean caches and owner-attached plain fallback caches retain
    /// the same rebuild predicate. No plain object is promoted to a trimmed one.
    pub fn get_tree_with_topology(
        &mut self,
        k: &Kernel,
        topology: &mut impl CacheTopology<O>,
    ) -> Result<&mut crate::geom::tree::Tree<O>> {
        let base = self.base();
        let needs_build = base.tree().is_none()
            || (!base.have_t_surface_cache
                && base.surface().face()?.is_some()
                && base.face_containment_done != FaceContainment::UnderConstruction);
        if needs_build {
            match self {
                Self::Trim(cache) => cache.build_tree_with_topology(k, topology)?,
                Self::Surface(cache) => cache.build_tree_with(k, false, &mut |c, k, n| {
                    c.test_against_tolerances_with_topology(k, n, topology)
                })?,
            }
        }
        self.base_mut().tree_mut().ok_or_else(|| {
            SolidError::invalid_input(
                "surface::cache_topology::get_tree",
                "cache build produced no tree",
            )
        })
    }
}

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
