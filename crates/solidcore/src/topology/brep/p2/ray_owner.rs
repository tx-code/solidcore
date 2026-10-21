// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Model-owned local execution for the native B-rep ray query.
//!
//! Upstream: maintained `SmBrep.cpp::RayFire` and
//! `SmTopologySolver.cpp::SmTLSPointFace::SolveIt`, `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | `point_solve` | `SmBrep::RayFire`'s `BrepPointSolve` call |
//! | `RayOwner` | actual topology/geometry owners used by local point solvers |
//!
//! Deviations: the cache trees borrow independently of the mutable owning B-rep.
//! A local solve borrows geometry only until its owned solutions are available;
//! face classification then updates the actual model. No model copies, global
//! registries, reference-counted graphs or alternate kernels are introduced.

use crate::curve::curve::Curve;
use crate::error::{ignored, EntityRef, Result, SolidError};
use crate::geom::extent1d::Extent1d;
use crate::geom::extent2d::Extent2d;
use crate::geom::global_solver::{GlobalSolver, SolverOperation};
use crate::geom::object::{ObjectRef, ShapeMember};
use crate::geom::solution_array::SolutionArray;
use crate::intersect::point_class::{PointClassType, PointClassification};
use crate::intersect::topology_solver::p1::{
    accept_point_face_ray, point_face_ray_request, point_vertex_zone_tol_3d,
    solve_borrowed_point_pair, LocalSolver, LocalSolverKind, PointLocalExecutor,
    RayFaceClassification, TopologyObjects, TopologySolver,
};
use crate::intersect::topology_solver::p2::TopologySolveOptions;
use crate::kernel::Kernel;
use crate::math::{Point2, Point3};
use crate::query::face::{p1 as face_point, p2 as face_uv};
use crate::surface::surface::Surface;
use crate::topology::brep_cache::{BrepCache, BrepObjects, CacheObject};
use crate::topology::{Brep, BrepId, FaceId};

const OP: &str = "topology::brep::p2::ray_owner";

pub(super) fn point_solve(
    kernel: &Kernel,
    brep: &mut Brep,
    brep_id: BrepId,
    cache: &BrepCache,
    keys: &dyn Fn(CacheObject) -> ObjectRef,
    owner_of: &dyn Fn(ObjectRef) -> Option<ObjectRef>,
    point: Point3,
    options: TopologySolveOptions<'_>,
) -> Result<SolutionArray<CacheObject>> {
    let trees = CacheTrees { cache };
    let mut owner = RayOwner {
        brep,
        brep_id,
        cache,
        keys,
        owner_of,
        classification: None,
    };
    TopologySolver::brep_point_solve_with_executor(
        kernel,
        &trees,
        &mut owner,
        Some(CacheObject::Brep),
        point,
        options,
    )
}

struct RayOwner<'a> {
    brep: &'a mut Brep,
    brep_id: BrepId,
    cache: &'a BrepCache,
    keys: &'a dyn Fn(CacheObject) -> ObjectRef,
    owner_of: &'a dyn Fn(ObjectRef) -> Option<ObjectRef>,
    // Native Point3DClassify retries reuse the current candidate's actual point
    // classification. Ordinary failure may leave observable classification state.
    classification: Option<PointClassification<EntityRef>>,
}

impl RayOwner<'_> {
    fn objects(&self) -> BrepObjects<'_> {
        BrepObjects {
            brep: self.brep,
            brep_id: self.brep_id,
            cache: self.cache,
            keys: self.keys,
            owner_of: self.owner_of,
        }
    }

    fn face(&self, object: CacheObject) -> Result<FaceId> {
        let CacheObject::Face(face) = object else {
            return Err(SolidError::invalid_input(OP, "expected actual face handle"));
        };
        self.brep.face(face)?;
        Ok(face)
    }
}

impl PointLocalExecutor<CacheObject> for RayOwner<'_> {
    fn solve_pair(
        &mut self,
        kernel: &Kernel,
        state: &mut GlobalSolver<'_, CacheObject>,
        local: &mut LocalSolver<'_, CacheObject>,
        first: Option<CacheObject>,
        second: Option<CacheObject>,
    ) -> Result<()> {
        if state.operation != SolverOperation::Rayfire {
            return Err(SolidError::invalid_input(
                OP,
                "owner executor requires ray operation",
            ));
        }
        match local.kind {
            LocalSolverKind::PointVertex | LocalSolverKind::PointEdge => {
                // The existing numerical bodies run while this short immutable
                // borrow is live. They retain no geometry borrow in a point ray.
                let objects = self.objects();
                solve_borrowed_point_pair(kernel, &objects, state, local, first, second)
            }
            LocalSolverKind::PointFace => {
                let request = {
                    let objects = self.objects();
                    point_face_ray_request(&objects, state, local, first, second)?
                };
                // Geometry borrows have ended. Both native cache construction
                // and PointClassify may update this same owning model.
                if let Some(request) = request {
                    let solutions =
                        super::surface_owner::line_intersections(kernel, self.brep, &request)?;
                    let candidates = request.candidates(solutions);
                    accept_point_face_ray(kernel, state, local, self, &candidates)?;
                }
                Ok(())
            }
            _ => Err(SolidError::invalid_input(
                OP,
                "unexpected native point-ray local solver",
            )),
        }
    }
}

impl RayFaceClassification<CacheObject> for RayOwner<'_> {
    fn source_zone_tolerance(&self, kernel: &Kernel, vertex: Option<CacheObject>) -> Result<f64> {
        point_vertex_zone_tol_3d(kernel, &self.objects(), vertex)
    }

    fn object_zone_tolerance(&self, kernel: &Kernel, object: CacheObject) -> Result<f64> {
        self.objects().object_zone_tolerance(kernel, object)
    }

    fn point_classify(
        &mut self,
        kernel: &Kernel,
        face: CacheObject,
        uv: Point2,
        source_tolerance: f64,
    ) -> Result<i32> {
        let face = self.face(face)?;
        let mut classification = PointClassification::new(source_tolerance);
        // SmTopologySolver.cpp:780--790 ignores ordinary PointClassify status
        // and reads the resulting class. A missing implementation still stops.
        let status = face_uv::point_classify(
            kernel,
            self.brep,
            face,
            uv,
            &mut classification,
            face_uv::PointClassifyOptions {
                src_zone_tol_3d: source_tolerance,
                do_boundary_intersections: true,
                use_uv_classification: true,
                ..Default::default()
            },
        );
        let class = classification.point_class().code();
        self.classification = Some(classification);
        ignored(status)?;
        Ok(class)
    }

    fn surface_owner(&self, surface: CacheObject) -> Result<Option<CacheObject>> {
        self.objects().surface_owner(surface)
    }

    fn classifies_on_face(
        &mut self,
        kernel: &Kernel,
        face: CacheObject,
        point: Point3,
        tolerance: f64,
    ) -> Result<bool> {
        let face = self.face(face)?;
        let mut classification = self.classification.take().ok_or_else(|| {
            SolidError::invalid_input(
                OP,
                "3D retry requires the current ray candidate classification",
            )
        })?;
        let result = (|| {
            // SmTopologySolver.cpp:797--827 retries only a failed status, using
            // tolerance and then tolerance * 100, and retains that same state.
            if ignored(face_point::point_3d_classify(
                kernel,
                self.brep,
                face,
                point,
                tolerance,
                true,
                &mut classification,
                face_point::Point3dClassifyOptions::default(),
            ))?
            .is_none()
                && ignored(face_point::point_3d_classify(
                    kernel,
                    self.brep,
                    face,
                    point,
                    tolerance * 100.0,
                    true,
                    &mut classification,
                    face_point::Point3dClassifyOptions::default(),
                ))?
                .is_none()
            {
                return Ok(false);
            }
            Ok(classification.point_class() == PointClassType::Face)
        })();
        self.classification = Some(classification);
        result
    }
}

// This resolver owns only the actual spatial-cache handles. Every native leaf
// executes through RayOwner; geometry must never be read through this resolver
// while that owner holds the mutable model. Unexpected access fails explicitly.
struct CacheTrees<'a> {
    cache: &'a BrepCache,
}

fn cache_only() -> SolidError {
    SolidError::invalid_input(OP, "native ray geometry requires the owning local executor")
}

impl TopologyObjects<CacheObject> for CacheTrees<'_> {
    fn brep_cache(&self, object: CacheObject) -> Result<&BrepCache> {
        if object != CacheObject::Brep {
            return Err(cache_only());
        }
        Ok(self.cache)
    }

    fn curve(&self, _: CacheObject) -> Result<&Curve> {
        Err(cache_only())
    }
    fn analytic_curve_handle(&self, _: CacheObject) -> Option<CacheObject> {
        None
    }
    fn vertex_point(&self, _: CacheObject) -> Result<Point3> {
        Err(cache_only())
    }
    fn vertex_tolerance(&self, _: CacheObject) -> Result<f64> {
        Err(cache_only())
    }
    fn curve_edge(&self, _: CacheObject) -> Result<Option<CacheObject>> {
        Err(cache_only())
    }
    fn edge_curve(&self, _: CacheObject) -> Result<Option<CacheObject>> {
        Err(cache_only())
    }
    fn edge_interval(&self, _: CacheObject) -> Result<Extent1d> {
        Err(cache_only())
    }
    fn surface_face(&self, _: CacheObject) -> Result<Option<CacheObject>> {
        Err(cache_only())
    }
    fn face_surface(&self, _: CacheObject) -> Result<CacheObject> {
        Err(cache_only())
    }
    fn surface(&self, _: CacheObject) -> Result<&Surface> {
        Err(cache_only())
    }
    fn face_uv_domain(&self, _: CacheObject) -> Result<Extent2d> {
        Err(cache_only())
    }
    fn face_tolerance(&self, _: CacheObject) -> Result<f64> {
        Err(cache_only())
    }
    fn surface_owner(&self, _: CacheObject) -> Result<Option<CacheObject>> {
        Err(cache_only())
    }
    fn shape_member(&self, _: CacheObject) -> ShapeMember {
        ShapeMember::Other
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::line::Line;
    use crate::geom::global_solver::SolutionRequested;
    use crate::math::{Vec2, Vec3, BIG_DOUBLE};
    use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
    use crate::surface::plane::Plane;
    use crate::topology::brep_cache::{arena_identity, make_brep_cache, CacheSource};
    use crate::topology::brep_constructor::{BrepConstructor, VertexNode};

    fn triangle() -> (Kernel, Brep, FaceId) {
        let mut kernel = Kernel::default();
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
        for (start, end) in [(0, 1), (1, 2), (2, 0)] {
            builder
                .start_edge(&kernel, OrientType::Same, None, None, None)
                .unwrap();
            builder
                .set_edge_curve(
                    &kernel,
                    Curve::from(Line::from_points(points[start], points[end], 3)),
                    false,
                )
                .unwrap();
            builder
                .set_limits(
                    &kernel,
                    VertexNode::Parameter(0.),
                    VertexNode::Parameter(1.),
                )
                .unwrap();
            builder.end_edge().unwrap();
        }
        builder.end_loop(&kernel).unwrap();
        builder.end_face().unwrap();
        builder.end_shell(&mut kernel, false).unwrap();
        (kernel, builder.finish().unwrap(), face)
    }

    #[test]
    fn owning_classification_keeps_trim_mutations_and_native_boundary_classes() {
        let (kernel, mut brep, face) = triangle();
        assert!(brep
            .edgeuses()
            .iter()
            .all(|(_, eu)| eu.uv_trim_curve.is_none()));
        let cache = make_brep_cache(&kernel, CacheSource::Brep(&brep), &arena_identity).unwrap();
        let tolerance = brep.tolerance();
        let mut owner = RayOwner {
            brep: &mut brep,
            brep_id: BrepId::from_index(0),
            cache: &cache,
            keys: &arena_identity,
            owner_of: &|_| None,
            classification: None,
        };
        for (uv, expected) in [
            (Point2::new(0.25, 0.25), PointClassType::Face),
            (Point2::new(0.5, 0.), PointClassType::Edge),
            (Point2::ZERO, PointClassType::Vertex),
            (Point2::new(0.875, 0.875), PointClassType::Unknown),
        ] {
            assert_eq!(
                owner
                    .point_classify(&kernel, CacheObject::Face(face), uv, tolerance)
                    .unwrap(),
                expected.code()
            );
        }
        assert!(owner
            .brep
            .edgeuses()
            .iter()
            .any(|(_, eu)| eu.uv_trim_curve.is_some()));
    }

    #[test]
    fn native_ray_uses_trim_classification_inside_the_surface_domain() {
        let (kernel, mut brep, face) = triangle();
        let cache = make_brep_cache(&kernel, CacheSource::Brep(&brep), &arena_identity).unwrap();
        let direction = [-Vec3::Z];
        for (point, inside) in [
            (Point3::new(0.25, 0.25, 1.), true),
            (Point3::new(0.875, 0.875, 1.), false),
        ] {
            let tolerance = brep.tolerance();
            let solutions = point_solve(
                &kernel,
                &mut brep,
                BrepId::from_index(0),
                &cache,
                &arena_identity,
                &|_| None,
                point,
                TopologySolveOptions {
                    operation: SolverOperation::Rayfire,
                    requested: SolutionRequested::All,
                    tolerance_3d: tolerance,
                    best_answer_so_far_sq: BIG_DOUBLE,
                    opt_vectors: &direction,
                },
            )
            .unwrap();
            assert_eq!(
                solutions
                    .as_slice()
                    .iter()
                    .any(|s| s.objects.contains(&Some(CacheObject::Face(face)))),
                inside
            );
        }
    }

    #[test]
    fn edge_target_ray_resolves_tolerance_inside_the_cached_curve_solver() {
        let (kernel, mut brep, _) = triangle();
        let cache = make_brep_cache(&kernel, CacheSource::Brep(&brep), &arena_identity).unwrap();
        let start = Point3::new(2., 2., 2.);
        let target = Point3::new(0.12345, 0., 0.);
        let direction = [(target - start).unitize().unwrap()];
        let tolerance = brep.tolerance();
        // The p9 search deliberately aims at an edge sample. Unlike an interior
        // face ray, this reaches CpGlobalSolver::AddSortedSolution before the
        // outer topology solver can replace the curve handle by its edge.
        let solutions = point_solve(
            &kernel,
            &mut brep,
            BrepId::from_index(0),
            &cache,
            &arena_identity,
            &|_| None,
            start,
            TopologySolveOptions {
                operation: SolverOperation::Rayfire,
                requested: SolutionRequested::All,
                tolerance_3d: tolerance,
                best_answer_so_far_sq: BIG_DOUBLE,
                opt_vectors: &direction,
            },
        )
        .unwrap();
        assert!(solutions.as_slice().iter().any(|solution| {
            solution
                .objects
                .iter()
                .any(|object| matches!(object, Some(CacheObject::Edge(_))))
        }));
    }

    #[test]
    fn owned_bspline_triangle_ray_uses_live_cache_topology_and_keeps_its_owner() {
        check_owned_bspline_triangle_ray(1e-5, 1e-5, false);
    }

    #[test]
    fn owned_bspline_triangle_ray_keeps_nondefault_context_and_face_tolerance() {
        // Distinct system/face tolerances and delayed Boolean cache construction
        // exercise the caller's context, rather than a replacement default Kernel.
        check_owned_bspline_triangle_ray(3e-3, 7e-5, true);
    }

    fn check_owned_bspline_triangle_ray(system_zone: f64, face_tolerance: f64, boolean: bool) {
        let (mut kernel, mut brep, face) = triangle();
        kernel.tolerances.zone_tol_3d = system_zone;
        kernel.context.set_doing_boolean(boolean);
        brep.face_mut(face).unwrap().tolerance = face_tolerance;
        let original = brep.face(face).unwrap().surface.as_ref().unwrap();
        let crate::surface::surface::SurfaceKind::Plane(plane) = original.kind() else {
            panic!("plane fixture");
        };
        // The identical degree-one definition as an explicit NURBS surface,
        // so the actual generic line/cache path runs instead of Plane's override.
        let mut surface = Surface::from(plane.base_with_nurb().unwrap().clone_exact());
        surface.set_owner(original.owner());
        let owner = surface.owner();
        brep.face_mut(face).unwrap().surface = Some(surface);
        let editing = brep.editing_enabled();
        let tolerance = brep.tolerance();
        let cache = make_brep_cache(&kernel, CacheSource::Brep(&brep), &arena_identity).unwrap();
        let direction = [-Vec3::Z];
        let solutions = point_solve(
            &kernel,
            &mut brep,
            BrepId::from_index(0),
            &cache,
            &arena_identity,
            &|_| None,
            Point3::new(0.25, 0.25, 1.),
            TopologySolveOptions {
                operation: SolverOperation::Rayfire,
                requested: SolutionRequested::All,
                tolerance_3d: tolerance,
                best_answer_so_far_sq: BIG_DOUBLE,
                opt_vectors: &direction,
            },
        )
        .unwrap();
        assert!(solutions
            .as_slice()
            .iter()
            .any(|solution| { solution.objects.contains(&Some(CacheObject::Face(face))) }));
        assert_eq!(
            brep.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
        assert_eq!(brep.editing_enabled(), editing);
        assert_eq!(brep.tolerance(), tolerance);
        assert_eq!(brep.face(face).unwrap().tolerance, face_tolerance);
        assert_eq!(kernel.context.doing_boolean(), boolean);
    }
}
