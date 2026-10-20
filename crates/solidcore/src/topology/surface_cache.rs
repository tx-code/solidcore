//! Live `BRep` adapter for the native surface-cache owner queries.
//!
//! Upstream: `SmTrimSrfCache.cpp`, `SmSurface.cpp::CacheMakeOrValidate`,
//! `SmFace.cpp::GetUpwardLoopuses`. See `docs/BREP_TOLERANCES.md`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`BrepSurfaceContext`] | face/model owner resolution for native cache calls |
//! | [`make_face_cache`] | `CacheMakeOrValidate(SM_OC_TRIMSRF)` plus `GetTree` |
//!
//! Deviations: scoped model borrow and typed handles replace native pointers;
//! caches keep exact evaluation copies, never a second interpretation of topology.

use crate::curve::curve::CurveKind;
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::solution_array::{ObjectKind, SolutionObject};
use crate::intersect::point_class::PointClassification;
use crate::kernel::Kernel;
use crate::math::Point2;
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::query::face::p2::{point_classify, PointClassifyOptions};
use crate::surface::cache_topology::CacheTopology;
use crate::surface::surface::p1::{cache_tolerances, CacheTolerances};
use crate::surface::surface::Surface;
use crate::surface::surface_cache::p1::SurfaceCacheOptions;
use crate::surface::trim_srf_cache::TrimSrfCache;
use crate::topology::{edgeuse::p1 as eu, face, loopuse, vertex, vertexuse, Brep, FaceId};

const OP: &str = "topology::surface_cache";

impl SolutionObject for EntityRef {
    fn kind(&self) -> ObjectKind {
        match self {
            Self::Face(_) => ObjectKind::Face,
            Self::Edge(_) => ObjectKind::Edge,
            Self::Vertex(_) => ObjectKind::Vertex,
            _ => ObjectKind::Other,
        }
    }
}

/// An actual face and its owning model. No callback, tolerance or model pointer
/// survives this borrow. Use a fresh context after editing the model.
pub struct BrepSurfaceContext<'a> {
    /// The model whose typed handles are resolved by this context.
    pub brep: &'a mut Brep,
    /// The actual face of the surface being cached.
    pub face: FaceId,
}

fn wrong_handle() -> SolidError {
    SolidError::invalid_input(OP, "cache topology handle has the wrong type")
}

impl CacheTopology<EntityRef> for BrepSurfaceContext<'_> {
    fn face(&mut self, surface: &Surface) -> Result<Option<EntityRef>> {
        let Some(owner) = surface.face()? else {
            return Ok(None);
        };
        let source = self
            .brep
            .face(self.face)?
            .surface
            .as_ref()
            .ok_or_else(|| SolidError::null_pointer(OP))?;
        if source.face()? != Some(owner) {
            return Err(SolidError::invalid_input(
                OP,
                "surface does not belong to the scoped face",
            ));
        }
        Ok(Some(EntityRef::Face(self.face)))
    }
    fn face_tolerance(&mut self, face: EntityRef) -> Result<f64> {
        let EntityRef::Face(f) = face else {
            return Err(wrong_handle());
        };
        face::p2::tolerance(self.brep, f)
    }
    fn editing_enabled(&mut self, face: EntityRef) -> Result<bool> {
        let EntityRef::Face(f) = face else {
            return Err(wrong_handle());
        };
        self.brep.face(f)?;
        Ok(self.brep.editing_enabled())
    }
    fn set_editing_enabled(&mut self, face: EntityRef, enabled: bool) -> Result<()> {
        let EntityRef::Face(f) = face else {
            return Err(wrong_handle());
        };
        self.brep.face(f)?;
        self.brep.set_editing_enabled(enabled);
        Ok(())
    }
    fn create_uv_trim_curves(&mut self, k: &Kernel, face: EntityRef) -> Result<()> {
        let EntityRef::Face(f) = face else {
            return Err(wrong_handle());
        };
        face::p3::create_uv_trim_curves(k, self.brep, f, true, None).map(|_| ())
    }
    fn upward_loopuses(&mut self, face: EntityRef) -> Result<Vec<EntityRef>> {
        let EntityRef::Face(f) = face else {
            return Err(wrong_handle());
        };
        let Some(fu) = self.brep.face(f)?.faceuse else {
            return Ok(Vec::new());
        };
        Ok(eu::faceuse_loopuses(self.brep, fu, false, None)?
            .into_iter()
            .map(EntityRef::Loopuse)
            .collect())
    }
    fn edgeuses(&mut self, lu: EntityRef) -> Result<Vec<EntityRef>> {
        let EntityRef::Loopuse(lu) = lu else {
            return Err(wrong_handle());
        };
        Ok(loopuse::edgeuses(self.brep, lu, None, None)?
            .into_iter()
            .map(EntityRef::Edgeuse)
            .collect())
    }
    fn uv_trim_curve(&mut self, eu_id: EntityRef) -> Result<Option<BSplineCurve>> {
        let EntityRef::Edgeuse(id) = eu_id else {
            return Err(wrong_handle());
        };
        eu::uv_trim_curve(self.brep, id)?
            .0
            .map(uv_bspline_copy)
            .transpose()
    }
    fn vertexuses(&mut self, lu: EntityRef) -> Result<Vec<EntityRef>> {
        let EntityRef::Loopuse(lu) = lu else {
            return Err(wrong_handle());
        };
        loopuse::vertexuses(self.brep, lu, None)?
            .into_iter()
            .map(|vu| {
                vu.map(EntityRef::Vertexuse)
                    .ok_or_else(|| SolidError::null_pointer(OP))
            })
            .collect()
    }
    fn uv_point(&mut self, k: &Kernel, vu: EntityRef) -> Result<Point2> {
        let EntityRef::Vertexuse(vu) = vu else {
            return Err(wrong_handle());
        };
        vertexuse::compute_uv_point(k, self.brep, vu, true)
    }
    fn vertex_tolerance(&mut self, vu: EntityRef) -> Result<f64> {
        let EntityRef::Vertexuse(vu) = vu else {
            return Err(wrong_handle());
        };
        let v = vertexuse::vertex(self.brep, vu)?.ok_or_else(|| SolidError::null_pointer(OP))?;
        vertex::tolerance(self.brep, v)
    }
    fn point_classify(
        &mut self,
        k: &Kernel,
        f: EntityRef,
        uv: Point2,
        tol: f64,
        flags: (bool, bool),
    ) -> Result<(i32, Option<EntityRef>)> {
        let EntityRef::Face(f) = f else {
            return Err(wrong_handle());
        };
        let mut pc = PointClassification::new(tol);
        point_classify(
            k,
            self.brep,
            f,
            uv,
            &mut pc,
            PointClassifyOptions {
                src_zone_tol_3d: tol,
                do_boundary_intersections: flags.0,
                use_uv_classification: flags.1,
                ..PointClassifyOptions::default()
            },
        )?;
        Ok((pc.point_class().code(), pc.object()))
    }
}

/// Exact native B-spline base needed by the cache and `CreateByJoining`. Analytic
/// line storage is materialized through its real lazy NURB accessor, not fitted.
pub(crate) fn uv_bspline_copy(curve: &crate::curve::curve::Curve) -> Result<BSplineCurve> {
    let base = match curve.kind() {
        CurveKind::BSpline(c) => c,
        CurveKind::Line(c) => return Ok(c.base_with_nurb()?.clone_exact()),
        CurveKind::Ellipse(c) => c.base(),
        CurveKind::Hyperbola(c) => &c.base,
        CurveKind::Parabola(c) => c.base(),
        _ => {
            return Err(SolidError::invalid_input(
                OP,
                "UV trim has no native B-spline base",
            ))
        }
    };
    Ok(base.clone_exact())
}

/// Builds the real trimmed cache for an owned face. Its UV-generation and gap
/// caches may change the model just as native `CacheMakeOrValidate` does; failure
/// drops this unpublished cache and restores temporary model flags.
pub fn make_face_cache(k: &Kernel, b: &mut Brep, face: FaceId) -> Result<TrimSrfCache<EntityRef>> {
    let surface = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer(OP))?
        .clone_for_evaluation()?;
    let CacheTolerances {
        chord_height_tol,
        angle_tol_rad,
    } = cache_tolerances(&surface)?;
    let mut cache = TrimSrfCache::new(
        surface,
        SurfaceCacheOptions {
            chord_height_tolerance: chord_height_tol,
            ang_tol_rad: angle_tol_rad,
            ..SurfaceCacheOptions::default()
        },
    );
    // Native caches delay construction during Booleans. This operation explicitly
    // asks for the built cache needed by ComputeUVPoint, so GetTree builds it here.
    cache.build_tree_with_topology(k, &mut BrepSurfaceContext { brep: b, face })?;
    Ok(cache)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::{curve::Curve, line::Line};
    use crate::geom::extent2d::Extent2d;
    use crate::math::{Point3, Vec2, Vec3};
    use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
    use crate::surface::plane::Plane;
    use crate::topology::brep_constructor::{BrepConstructor, VertexNode};

    fn triangle() -> (Kernel, Brep, FaceId) {
        let mut k = Kernel::default();
        let mut builder = BrepConstructor::new().unwrap();
        builder.start_shell().unwrap();
        let face = builder.start_face(OrientType::Same).unwrap();
        let domain = Extent2d::from_bounds(0., 0., 1., 1.);
        builder
            .set_face_surface(
                Surface::from(Plane::new(
                    Point3::ZERO,
                    Vec3::X,
                    Vec3::Y,
                    Vec2::new(1., 1.),
                    &domain,
                )),
                domain,
            )
            .unwrap();
        builder.start_loop(OrientType::Same).unwrap();
        let points = [Point3::ZERO, Vec3::X, Vec3::Y];
        let vertices = points
            .iter()
            .map(|p| builder.start_vertex_of_loop(*p).unwrap())
            .collect::<Vec<_>>();
        for i in 0..3 {
            let next = (i + 1) % 3;
            builder
                .start_edge(
                    &k,
                    OrientType::Same,
                    None,
                    Some(vertices[i]),
                    Some(vertices[next]),
                )
                .unwrap();
            builder
                .set_edge_curve(
                    &k,
                    Curve::from(Line::from_points(points[i], points[next], 3)),
                    false,
                )
                .unwrap();
            builder
                .set_limits(&k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))
                .unwrap();
            builder.end_edge().unwrap();
        }
        builder.end_loop().unwrap();
        builder.end_face().unwrap();
        builder.end_shell(&mut k, false).unwrap();
        (k, builder.finish().unwrap(), face)
    }

    #[test]
    fn owned_plane_cache_retains_geometry_owner_and_real_trim_handles() {
        let (k, mut b, face) = triangle();
        let owner = b.face(face).unwrap().surface.as_ref().unwrap().owner();
        let points = b
            .vertex_list()
            .iter()
            .map(|v| b.vertex(*v).unwrap().point)
            .collect::<Vec<_>>();
        let cache = make_face_cache(&k, &mut b, face).unwrap();
        assert!(!b.editing_enabled());
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
        assert!(cache.base().have_t_surface_cache);
        assert!(cache.base().point_test_enabled);
        assert!(!cache.base().process_boundary_curves);
        assert_eq!(
            cache.base().uv_domain(),
            Extent2d::from_bounds(0., 0., 1., 1.)
        );
        assert_eq!(
            b.vertex_list()
                .iter()
                .map(|v| b.vertex(*v).unwrap().point)
                .collect::<Vec<_>>(),
            points
        );
        for e in b.edge_list() {
            let eu = b.edge(*e).unwrap().edgeuses[0];
            assert!(matches!(
                eu::uv_trim_curve(&b, eu).unwrap().0.unwrap().kind(),
                CurveKind::Line(_)
            ));
        }
        let mut context = BrepSurfaceContext { brep: &mut b, face };
        assert_eq!(
            cache
                .point_test_with_topology(&k, Point2::new(0.2, 0.2), None, &mut context)
                .unwrap(),
            (true, Some(EntityRef::Face(face)))
        );
        assert_eq!(
            cache
                .point_test_with_topology(&k, Point2::new(0.8, 0.8), None, &mut context)
                .unwrap(),
            (false, None)
        );
    }

    #[test]
    fn editing_model_builds_base_cache_without_generating_trims() {
        let (k, mut b, face) = triangle();
        b.set_editing_enabled(true);
        let cache = make_face_cache(&k, &mut b, face).unwrap();
        assert!(b.editing_enabled());
        assert!(!cache.base().have_t_surface_cache);
        assert!(!cache.base().point_test_enabled);
        assert!(cache.base().process_boundary_curves);
        for e in b.edge_list() {
            let eu = b.edge(*e).unwrap().edgeuses[0];
            assert!(eu::uv_trim_curve(&b, eu).unwrap().0.is_none());
        }
    }

    #[test]
    fn failed_trim_generation_restores_editing_and_surface_owner() {
        let (k, mut b, face) = triangle();
        let owner = b.face(face).unwrap().surface.as_ref().unwrap().owner();
        let edge = b.edge_list()[0];
        b.edge_mut(edge).unwrap().curve = None;
        assert!(make_face_cache(&k, &mut b, face).is_err());
        assert!(!b.editing_enabled());
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
    }
}
