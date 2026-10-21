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
use crate::surface::surface::Surface;
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

impl crate::surface::surface::p5::PointDropContext<EntityRef> for BrepSurfaceContext<'_> {
    fn face_tolerance(&mut self, owner: crate::geom::object::ObjectRef) -> Result<f64> {
        owning_face_tolerance(self.brep, owner)
    }
    fn global_point_solve(
        &mut self,
        k: &Kernel,
        surface: &Surface,
        domain: &crate::geom::extent2d::Extent2d,
        op: crate::geom::global_solver::SolverOperation,
        point: crate::math::Point3,
        tolerance: f64,
    ) -> Result<crate::geom::solution_array::SolutionArray<EntityRef>> {
        // SmSurface.cpp20065 gets a cache only for a fast-result trim check.
        // Let the shared global pipeline perform that lookup lazily; generic
        // numerical solves retain their original absence of cache construction.
        let mut solutions = crate::geom::solution_array::SolutionArray::new();
        surface.global_point_solve_into_with_surface_cache(
            k,
            None,
            domain,
            op,
            point,
            tolerance,
            None,
            crate::geom::global_solver::SolutionRequested::All,
            &mut solutions,
            None,
            self,
        )?;
        Ok(solutions)
    }
}

/// Native point drop on an actual face, retaining its concrete surface and
/// classification/cache context. All entrypoints share the Surface pipeline;
/// this bridge supplies ownership, without fitting geometry or clearing owners.
pub fn drop_face_point(
    k: &Kernel,
    b: &mut Brep,
    face: FaceId,
    point: crate::math::Point3,
    domain: &crate::geom::extent2d::Extent2d,
    options: &crate::surface::surface::p5::DropPointOptions,
) -> Result<crate::surface::surface::p5::DropPoint<EntityRef>> {
    let surface = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer(OP))?
        .clone_for_evaluation()?;
    surface.drop_point_with_context(
        k,
        point,
        domain,
        options,
        &mut BrepSurfaceContext { brep: b, face },
    )
}

/// The native simple surface/curve coincidence walk with the actual owning
/// face and edge. Query callbacks end each geometry borrow before the global
/// surface phase may materialize the model's trim/cache data.
pub(crate) fn simple_face_edge_coincidence(
    k: &Kernel,
    b: &mut Brep,
    face: FaceId,
    edge: crate::topology::EdgeId,
    domain: &crate::geom::extent2d::Extent2d,
    interval: &crate::geom::extent1d::Extent1d,
    tolerance: f64,
    partial: bool,
    objects: [Option<EntityRef>; 2],
) -> Result<Option<crate::geom::solution_array::Solution<EntityRef>>> {
    let mut query = FaceEdgeCoincidence {
        brep: b,
        face,
        edge,
    };
    // Preserve the caller's required defining geometry before entering the walk.
    query.curve()?;
    query.surface()?;
    Surface::simple_coincidence_checker_with_query(
        k, &mut query, domain, interval, tolerance, partial, objects,
    )
}

struct FaceEdgeCoincidence<'a> {
    brep: &'a mut Brep,
    face: FaceId,
    edge: crate::topology::EdgeId,
}

impl FaceEdgeCoincidence<'_> {
    fn surface(&self) -> Result<&Surface> {
        self.brep
            .face(self.face)?
            .surface
            .as_ref()
            .ok_or_else(|| SolidError::null_pointer(OP))
    }
    fn curve(&self) -> Result<&crate::curve::curve::Curve> {
        self.brep
            .edge(self.edge)?
            .curve
            .as_ref()
            .ok_or_else(|| SolidError::null_pointer(OP))
    }
}

impl crate::surface::surface::p6::SimpleCoincidenceQuery<EntityRef> for FaceEdgeCoincidence<'_> {
    fn surface_box(
        &mut self,
        k: &Kernel,
        domain: &crate::geom::extent2d::Extent2d,
    ) -> Result<Option<crate::geom::extent3d::Extent3d>> {
        let surface = self.surface()?;
        let boxes = if let Some(owner) = surface.face()? {
            surface.calculate_bounding_box_for_face(
                k,
                domain,
                [true, false, false],
                true,
                owning_face_tolerance(self.brep, owner)?,
            )?
        } else {
            surface.calculate_bounding_box(k, domain, [true, false, false], true)?
        };
        Ok(boxes.normal_box)
    }
    fn curve_box(
        &mut self,
        k: &Kernel,
        interval: &crate::geom::extent1d::Extent1d,
    ) -> Result<Option<crate::geom::extent3d::Extent3d>> {
        Ok(self
            .curve()?
            .calculate_bounding_box(
                k,
                interval,
                crate::nurbs::b_spline_curve::BoundingBoxRequest {
                    normal: true,
                    pseudo: false,
                    polar: false,
                },
                true,
            )?
            .normal)
    }
    fn evaluate_curve(
        &self,
        k: &Kernel,
        parameter: f64,
        from_left: bool,
    ) -> Result<Vec<crate::math::Vec3>> {
        self.curve()?.evaluate(k, parameter, 1, from_left, true)
    }
    fn curve_degree(&self) -> Result<usize> {
        self.curve()?.degree()
    }
    fn curve_interval(&self) -> Result<crate::geom::extent1d::Extent1d> {
        Ok(self.curve()?.natural_interval())
    }
    fn normal(&self, k: &Kernel, uv: Point2) -> Result<crate::math::Vec3> {
        self.surface()?.evaluate_normal(
            k,
            uv,
            crate::nurbs::knots::Side::Right,
            crate::nurbs::knots::Side::Right,
        )
    }
    fn global_point(
        &mut self,
        k: &Kernel,
        domain: &crate::geom::extent2d::Extent2d,
        point: crate::math::Point3,
        tolerance: f64,
    ) -> Result<Vec<crate::geom::solution_array::SolutionEnd>> {
        let surface = self.surface()?.clone_for_evaluation()?;
        let mut solutions = crate::geom::solution_array::SolutionArray::<EntityRef>::new();
        // SmSurface::SimpleCoincidenceChecker asks MINIMIZE/SINGLE here. The
        // existing global pipeline chooses fast projection and actual cache role
        // lazily; this adapter supplies its real topology, never an eager cache.
        surface.global_point_solve_into_with_surface_cache(
            k,
            None,
            domain,
            crate::geom::global_solver::SolverOperation::Minimize,
            point,
            tolerance,
            None,
            crate::geom::global_solver::SolutionRequested::Single,
            &mut solutions,
            None,
            &mut BrepSurfaceContext {
                brep: self.brep,
                face: self.face,
            },
        )?;
        Ok(solutions
            .as_slice()
            .iter()
            .map(|solution| solution.start)
            .collect())
    }
    fn local_point(
        &mut self,
        k: &Kernel,
        domain: &crate::geom::extent2d::Extent2d,
        point: crate::math::Point3,
        guess: Point2,
    ) -> Result<(bool, crate::geom::solution_array::Solution<EntityRef>)> {
        let surface = self.surface()?;
        let tolerance = working_surface_tolerance(k, self.brep, surface)?;
        let result = surface.local_point_solve_for_face(
            k,
            domain,
            crate::geom::global_solver::SolverOperation::Minimize,
            point,
            guess,
            Some(tolerance),
        )?;
        Ok((result.found, result.solution))
    }
}

fn wrong_handle() -> SolidError {
    SolidError::invalid_input(OP, "cache topology handle has the wrong type")
}

/// Resolve the native geometry-owner namespace in this exact model. Working
/// tolerances are not import accuracy or repair allowances. Foreign owners need
/// the future shared resolver; a default value would hide missing ownership.
pub(crate) fn owning_face_tolerance(
    b: &Brep,
    owner: crate::geom::object::ObjectRef,
) -> Result<f64> {
    // arena_identity uses a raw u32 index in the geometry-owner namespace.
    // Round-trip through that canonical encoder to reject metadata/custom keys;
    // do not scan all faces for each point drop in a large model.
    if let Ok(index) = u32::try_from(owner.key.0) {
        let id = FaceId::from_index(index);
        if crate::topology::brep_cache::arena_identity(
            crate::topology::brep_cache::CacheObject::Face(id),
        ) == owner
        {
            if let Some(face) = b.faces().get(id) {
                return Ok(face.tolerance);
            }
        }
    }
    // UNPORTED(topology.face_p2): owner outside the supplied model.
    Err(SolidError::unported(
        "SmFace::GetTolerance outside owning model",
        "topology.face_p2",
    ))
}

/// Actual surface working tolerance, including the unowned base convention.
pub(crate) fn working_surface_tolerance(k: &Kernel, b: &Brep, surface: &Surface) -> Result<f64> {
    match surface.face()? {
        Some(owner) => owning_face_tolerance(b, owner),
        None => crate::surface::surface::p1::zone_tol_3d(k, surface),
    }
}

/// Native `GetNaturalUVDomain` on the actual stored surface. B-spline families
/// lazily create a missing NURB; an evaluation copy would lose that effect.
/// Existing definitions and UV parameterizations are retained.
pub(crate) fn natural_domain_owned(
    k: &Kernel,
    b: &mut Brep,
    id: FaceId,
) -> Result<crate::geom::extent2d::Extent2d> {
    use crate::surface::{surface::SurfaceKind, validity};
    let s = b
        .face_mut(id)?
        .surface
        .as_mut()
        .ok_or_else(|| SolidError::null_pointer(OP))?;
    if matches!(
        s.kind(),
        SurfaceKind::BSpline(_) | SurfaceKind::Plane(_) | SurfaceKind::SurfOfRevolution(_)
    ) {
        validity::materialize_bspline(k, s)?;
    }
    Ok(s.natural_uv_domain())
}

impl CacheTopology<EntityRef> for BrepSurfaceContext<'_> {
    fn point_solver_face_tolerance(&mut self, surface: &Surface) -> Result<Option<f64>> {
        surface
            .face()?
            .map(|owner| owning_face_tolerance(self.brep, owner))
            .transpose()
    }
    fn point_solver_cache(
        &mut self,
        k: &Kernel,
        _surface: &Surface,
    ) -> Result<crate::surface::cache_topology::OwnedSurfaceCache<EntityRef>> {
        get_face_surface_cache(k, self.brep, self.face, None)
    }
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
    let mut context = BrepSurfaceContext { brep: b, face };
    let mut cache = crate::surface::surface::p1::make_trim_srf_cache_with_topology(
        k,
        &surface,
        None,
        &mut context,
    )?;
    // This API explicitly asks for a built trim cache; the role-preserving native
    // getter below leaves Boolean construction delayed until an actual GetTree.
    if cache.base().tree().is_none() {
        cache.build_tree_with_topology(k, &mut context)?;
    }
    Ok(cache)
}

/// `smsurf_GetSurfaceCache` for a real face, optionally consuming its old single
/// cache slot. Dynamic role, reuse flags and Boolean delays remain observable.
/// The caller owns this model-bound slot and invalidates it after geometric edits.
pub fn get_face_surface_cache(
    k: &Kernel,
    b: &mut Brep,
    face: FaceId,
    old: Option<crate::surface::cache_topology::OwnedSurfaceCache<EntityRef>>,
) -> Result<crate::surface::cache_topology::OwnedSurfaceCache<EntityRef>> {
    // Native reuse reads owner/base flags only. Avoid copying a potentially large
    // NURBS/owned-geometry graph (and its copy-depth checks) for a retained slot.
    let source = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer(OP))?;
    if let Some(old) = old {
        let editing_disabled =
            old.base().face_was_modified && !b.editing_enabled() && source.face()?.is_some();
        if !old.requires_replacement(editing_disabled) {
            return Ok(old);
        }
        // Match DeleteObjectsCache before constructing/evaluating a replacement.
        drop(old);
    }
    let surface = b
        .face(face)?
        .surface
        .as_ref()
        .ok_or_else(|| SolidError::null_pointer(OP))?
        .clone_for_evaluation()?;
    crate::surface::surface::p1::get_owned_surface_cache_with_topology(
        k,
        &surface,
        None,
        None,
        &mut BrepSurfaceContext { brep: b, face },
    )
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
        builder.end_loop(&k).unwrap();
        builder.end_face().unwrap();
        builder.end_shell(&mut k, false).unwrap();
        (k, builder.finish().unwrap(), face)
    }

    #[test]
    fn owned_simple_coincidence_matches_the_borrowed_algorithm_without_losing_owners() {
        let (k, mut b, face) = triangle();
        let edge = b.edge_list()[0];
        let domain = b.face(face).unwrap().uv_domain;
        let interval = b.edge(edge).unwrap().interval;
        b.face_mut(face).unwrap().tolerance = 0.002;
        let surface_owner = b.face(face).unwrap().surface.as_ref().unwrap().owner();
        let curve_owner = b.edge(edge).unwrap().curve.as_ref().unwrap().owner();
        let result = simple_face_edge_coincidence(
            &k,
            &mut b,
            face,
            edge,
            &domain,
            &interval,
            1.0e-5,
            false,
            [Some(EntityRef::Edge(edge)), Some(EntityRef::Face(face))],
        )
        .unwrap()
        .unwrap();
        let plane = Surface::from(Plane::new(
            Point3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec2::new(1., 1.),
            &domain,
        ));
        let line = Curve::from(Line::from_points(Point3::ZERO, Vec3::X, 3));
        let borrowed = plane
            .simple_coincidence_checker::<EntityRef>(
                &k,
                &domain,
                &line,
                &interval,
                1.0e-5,
                false,
                [None, None],
            )
            .unwrap()
            .unwrap();
        assert_eq!(result.solution_type, borrowed.solution_type);
        assert_eq!(result.start, borrowed.start);
        assert_eq!(result.end, borrowed.end);
        assert_eq!(result.objects[0], Some(EntityRef::Edge(edge)));
        assert_eq!(result.objects[1], Some(EntityRef::Face(face)));
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            surface_owner
        );
        assert_eq!(
            b.edge(edge).unwrap().curve.as_ref().unwrap().owner(),
            curve_owner
        );
    }

    #[test]
    fn owned_surface_boxes_expand_normal_and_pseudo_by_the_actual_face_tolerance() {
        let (k, mut b, face) = triangle();
        b.face_mut(face).unwrap().tolerance = 0.125;
        let record = b.face(face).unwrap();
        let surface = record.surface.as_ref().unwrap();
        let base = surface
            .calculate_bounding_box(&k, &record.uv_domain, [true, true, false], false)
            .unwrap();
        let expanded = surface
            .calculate_bounding_box_for_face(
                &k,
                &record.uv_domain,
                [true, true, false],
                true,
                owning_face_tolerance(&b, surface.face().unwrap().unwrap()).unwrap(),
            )
            .unwrap();
        let mut normal = base.normal_box.unwrap();
        normal.expand_absolute(0.125);
        let mut pseudo = base.pseudo_box.unwrap();
        pseudo.expand_absolute(0.125);
        assert_eq!(expanded.normal_box, Some(normal));
        assert_eq!(expanded.pseudo_box, Some(pseudo));
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

    #[test]
    fn boolean_lookup_delays_the_actual_trimmed_build_until_get_tree() {
        let (mut k, mut b, face) = triangle();
        k.context.set_doing_boolean(true);
        let mut cache = get_face_surface_cache(&k, &mut b, face, None).unwrap();
        assert!(cache.view().trimmed().is_some());
        assert!(cache.base().tree().is_none());
        assert!(!cache.base().have_t_surface_cache);
        cache
            .get_tree_with_topology(&k, &mut BrepSurfaceContext { brep: &mut b, face })
            .unwrap();
        assert!(cache.base().tree().is_some());
        assert!(cache.base().have_t_surface_cache);
        assert!(k.context.doing_boolean());
        assert!(!b.editing_enabled());
    }

    #[test]
    fn cache_reuse_keeps_native_state_until_modified_and_editing_disabled() {
        let (k, mut b, face) = triangle();
        let mut old = get_face_surface_cache(&k, &mut b, face, None).unwrap();
        // Use retained state rather than allocator addresses to witness reuse.
        old.base_mut().trim_subdivision_factor = 37.;
        let mut reused = get_face_surface_cache(&k, &mut b, face, Some(old)).unwrap();
        assert_eq!(reused.base().trim_subdivision_factor, 37.);
        reused.base_mut().face_was_modified = true;
        b.set_editing_enabled(true);
        let retained = get_face_surface_cache(&k, &mut b, face, Some(reused)).unwrap();
        assert!(retained.base().face_was_modified);
        assert_eq!(retained.base().trim_subdivision_factor, 37.);
        b.set_editing_enabled(false);
        let rebuilt = get_face_surface_cache(&k, &mut b, face, Some(retained)).unwrap();
        assert!(!rebuilt.base().face_was_modified);
        assert_eq!(rebuilt.base().trim_subdivision_factor, 20.);
    }

    #[test]
    fn a_plain_cache_stays_plain_on_single_slot_reuse_and_fast_projection() {
        use crate::geom::{
            global_solver::{SolutionRequested, SolverOperation},
            solution_array::SolutionArray,
        };
        use crate::surface::cache_topology::OwnedSurfaceCache;
        let (k, mut b, face) = triangle();
        let surface = b
            .face(face)
            .unwrap()
            .surface
            .as_ref()
            .unwrap()
            .clone_for_evaluation()
            .unwrap();
        let base = crate::surface::surface::p1::make_surface_cache_with_topology(
            &k,
            &surface,
            None,
            &mut BrepSurfaceContext { brep: &mut b, face },
        )
        .unwrap();
        let mut cache = get_face_surface_cache(
            &k,
            &mut b,
            face,
            Some(OwnedSurfaceCache::Surface(Box::new(base))),
        )
        .unwrap();
        assert!(cache.view().trimmed().is_none());
        // Native GlobalPointSolve tests the dynamic TrimSrfCache role as well as
        // flags; this point is outside the triangular trim but in the surface.
        cache.base_mut().point_test_enabled = true;
        let mut solutions = SolutionArray::<EntityRef>::new();
        surface
            .global_point_solve_into_with_surface_cache(
                &k,
                None,
                &surface.natural_uv_domain(),
                SolverOperation::Minimize,
                Vec3::new(0.8, 0.8, 1.),
                1e-5,
                None,
                SolutionRequested::All,
                &mut solutions,
                Some(cache.view()),
                &mut BrepSurfaceContext { brep: &mut b, face },
            )
            .unwrap();
        assert_eq!(solutions.len(), 1);
        assert!(solutions.get_at(0).unwrap().start.param(0).unwrap() > 0.7);
        assert!(solutions.get_at(0).unwrap().start.param(1).unwrap() > 0.7);
    }

    #[test]
    fn owned_bspline_local_projection_keeps_face_tolerance_in_boundary_seam_test() {
        use crate::geom::global_solver::SolverOperation;
        let (k, b, face) = triangle();
        let original = b.face(face).unwrap().surface.as_ref().unwrap();
        let crate::surface::surface::SurfaceKind::Plane(plane) = original.kind() else {
            panic!("plane fixture")
        };
        let mut surface = Surface::from(plane.base_with_nurb().unwrap().clone_exact());
        surface.set_owner(original.owner());
        let tol = b.face(face).unwrap().tolerance;
        let domain = surface.natural_uv_domain();
        let point = Vec3::new(1.01, -0.01, 0.01);
        let guess = Point2::new(1., 0.);
        let r = surface
            .local_point_solve_1_for_face::<EntityRef>(
                &k,
                &domain,
                SolverOperation::Minimize,
                point,
                guess,
                None,
                Some(tol),
            )
            .unwrap();
        // Analytic independent truth for this degree-one identity patch: the
        // nearest point in the rectangular domain is its (1,0) corner.
        assert!(r.found);
        assert_eq!(r.solution.start.param(0).unwrap(), 1.);
        assert_eq!(r.solution.start.param(1).unwrap(), 0.);
        let expected = point.distance(Vec3::X);
        assert!((r.solution.start.solution_value - expected).abs() <= expected.abs() * 1e-9);
        assert_eq!(surface.owner(), original.owner());
        assert_eq!(b.face(face).unwrap().tolerance, tol);
    }
}
