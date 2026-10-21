//! Face, shell and region bounding boxes, face point and edge-crossing classification,
//! and the checked curve evaluation API entry point.
//!
//! Upstream: `source/SMLib/src/SmFace.cpp`, `source/SMLib/src/SmShell.cpp`,
//! `source/SMLib/src/SmRegion.cpp`, `source/SM_API/src/SmApiCurves.cpp`,
//! `source/SMLib/inc/SmFace.h`, `source/SMLib/inc/SmCurveClass.h`.
//!
//! | Rust                                  | Upstream                         |
//! |---------------------------------------|----------------------------------|
//! | [`evaluate_curve`]                    | `SmApiEvaluateCurve`             |
//! | [`calculate_bounding_box`]            | `SmFace::CalculateBoundingBox`   |
//! | [`calculate_shell_bounding_box_into`], [`calculate_shell_bounding_box`] | `SmShell::CalculateBoundingBox` |
//! | [`calculate_region_bounding_box_into`], [`calculate_region_bounding_box`] | `SmRegion::CalculateBoundingBox` |
//! | [`point_3d_classify`], [`Point3dClassifyOptions`] | `SmFace::Point3DClassify` |
//! | [`edge_curve_on_classify`], [`EdgeCurveOnOptions`], [`CurveOnClassification`] | `SmFace::EdgeCurveOnClassify` |
//! | [`ClassifyProblem`]                   | `SmClassifyProblem` (CO-PORTED; re-exported by `query::face::p3`) |
//!
//! Deviations: a nullable curve is `Option<&Curve>`; `SmApiEvaluateCurve`'s three
//! optional output pointers become one returned `[position, d1, d2]` (all three are
//! evaluated upstream too). The face box returns its box (upstream writes its output
//! only on success). The shell and region boxes initialise `output` first and keep
//! the union of the children that succeeded before a failing child, as upstream's
//! in/out box does; [`calculate_shell_bounding_box`] / [`calculate_region_bounding_box`]
//! return the box instead. Shell and region boxes take `&mut Kernel` because the wire
//! edge traversal claims a context mark. The surface box of a face expands by the
//! face's own tolerance (what `SmBSplineSurface::CalculateBoundingBox` reads through
//! `GetFace()`): arena surfaces hold no owning face, so the B-spline/plane/revolution
//! box is computed unexpanded and expanded here, as the override's last step does.
//! `Point3DClassify` borrows its actual mutable model for the surface cache and
//! point solver. Its temporary point-test flag is restored on success, early
//! return and failure; the same actual cache is used for the final UV classification.
//! `EdgeCurveOnClassify` lifts a 2D curve to a curve on the
//! face's surface only when an upward edge use of the face is found (upstream lifts it
//! first and ignores `SetAll`'s status); an `SmCrvOnSurf` over a surface other than a
//! B-spline is a dispatch gap (`curve.crv_on_surf`). No recursion.

use crate::curve::crv_on_surf::CrvOnSurf;
use crate::curve::curve::{Curve, CurveKind};
use crate::error::ignored;
use crate::geom::extent1d::Extent1d;
use crate::geom::extent3d::Extent3d;
use crate::geom::global_solver::{SolutionRequested, SolverOperation};
use crate::geom::tol::xsect_tol_3d_from_zones;
use crate::geom::vector2d::Point2;
use crate::geom::vector3d::{Point3, Vec3};
use crate::intersect::point_class::{ClassGeometry, PointClassType, PointClassification};
use crate::kernel::Kernel;
use crate::nurbs::b_spline_curve::BoundingBoxRequest;
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::p4::NoObject;
use crate::surface::surface::Surface;
use crate::surface::volume::Volume;
use crate::topology::face::p2 as fp2;
use crate::topology::shell::{self, ShellType};
use crate::topology::{
    edge, edgeuse::p1 as eu1, topology_traverser, vertex, Brep, EdgeId, EdgeuseId, FaceId, LoopId,
    RegionId, ShellId,
};
use crate::{EntityRef, Result, SolidError};

const OP_CURVE: &str = "query::face::p1::evaluate_curve";
const OP_FACE_BOX: &str = "query::face::p1::calculate_bounding_box";
const OP_SHELL_BOX: &str = "query::face::p1::calculate_shell_bounding_box";
const OP_REGION_BOX: &str = "query::face::p1::calculate_region_bounding_box";
const OP_POINT: &str = "query::face::p1::point_3d_classify";
const OP_EDGE_CURVE: &str = "query::face::p1::edge_curve_on_classify";

// ---- SmApiEvaluateCurve ------------------------------------------------------------------

/// `SmApiEvaluateCurve`: position, first and second derivative of `curve` at
/// `parameter`, which must lie in the curve's natural interval (inclusive). Evaluates
/// from the left with nonzero tangents (upstream's defaults).
///
/// # Errors
///
/// [`SolidError::InvalidInput`] (`SM_ERR_INVALID_INPUT`) for a null curve or a parameter
/// outside the natural interval (NaN included); those of the curve's evaluator.
pub fn evaluate_curve(k: &Kernel, curve: Option<&Curve>, parameter: f64) -> Result<[Vec3; 3]> {
    let curve = curve.ok_or_else(|| SolidError::invalid_input(OP_CURVE, "null curve"))?;

    // Negated in-range test so a NaN parameter is rejected too.
    let domain = curve.natural_interval();
    if !(parameter >= domain.min() && parameter <= domain.max()) {
        return Err(SolidError::invalid_input(
            OP_CURVE,
            "parameter outside natural domain",
        ));
    }

    let values = curve.evaluate(k, parameter, 2, true, true)?;
    let mut iter = values.into_iter();
    let missing = || SolidError::invalid_input(OP_CURVE, "curve returned too few derivatives");
    Ok([
        iter.next().ok_or_else(missing)?,
        iter.next().ok_or_else(missing)?,
        iter.next().ok_or_else(missing)?,
    ])
}

// ---- bounding boxes ------------------------------------------------------------------------

fn tight_face_box(k: &Kernel, b: &Brep, f: FaceId) -> Result<Extent3d> {
    crate::query::face::p3::calculate_tight_bounding_box(k, b, f)
}

const NORMAL_BOX: BoundingBoxRequest = BoundingBoxRequest {
    normal: true,
    pseudo: false,
    polar: false,
};

/// `SmFace::CalculateBoundingBox(rFaceBBox, bTight)` (`tight` defaults to `false`
/// upstream): with `tight`, `CalculateTightBoundingBox`. Otherwise, when the face's
/// surface is linear in U or V, the union of its vertex points (each expanded by the
/// vertex tolerance) and its edge curve boxes over the edge intervals (each expanded by
/// the edge tolerance); else the surface's box over the face's UV domain, expanded by
/// the face tolerance.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a face without surface (upstream dereferences null)
/// or a vertex use without vertex; those of the curve and surface boxes and of the
/// face's vertex and edge lists; with `tight`, those of
/// [`crate::query::face::p3::calculate_tight_bounding_box`].
pub fn calculate_bounding_box(k: &Kernel, b: &Brep, f: FaceId, tight: bool) -> Result<Extent3d> {
    // when asked - call expensive TightBoundingBox
    if tight {
        return tight_face_box(k, b, f);
    }

    let mut face_box = Extent3d::new();

    let surface = fp2::surface(b, f)?
        .ok_or_else(|| SolidError::invalid_input(OP_FACE_BOX, "face without surface"))?;
    let u_deg = surface.degree(SurfParamType::U);
    let v_deg = surface.degree(SurfParamType::V);

    // If one of the directions of the surface is linear we can compute the bounding box
    // using the edges and vertices.
    if u_deg == 1 || v_deg == 1 {
        for v in super::p2::face_vertices(b, f, false, None)? {
            let mut v_box = Extent3d::from_point(vertex::point(b, v)?);
            v_box.expand_absolute(vertex::tolerance(b, v)?);
            face_box = face_box.union(&v_box);
        }

        // Now load the Edge Tree
        for e in topology_traverser::face_edges(b, f)? {
            if let Some(curve) = edge::curve(b, e)? {
                let mut e_box = curve
                    .calculate_bounding_box(k, &edge::interval(b, e)?, NORMAL_BOX, true)?
                    .normal
                    .ok_or_else(|| SolidError::invalid_input(OP_FACE_BOX, "missing edge box"))?;
                e_box.expand_absolute(edge::tolerance(b, e)?);
                face_box = face_box.union(&e_box);
            }
        }
    } else {
        // Get the bounding box of the surface
        face_box = surface_box(k, surface, &fp2::uv_domain(b, f)?, fp2::tolerance(b, f)?)?;
    }

    // Set output and return
    Ok(face_box)
}

/// `pSurface->CalculateBoundingBox(GetUVDomain(), &box)` of a face's surface (expanded by
/// the zone tolerance, the default): the B-spline override (also that of planes and
/// surfaces of revolution) expands its normal box by the face tolerance as its last
/// step; the other kinds dispatch as is.
fn surface_box(
    k: &Kernel,
    surface: &Surface,
    uv_domain: &crate::geom::extent2d::Extent2d,
    face_tolerance: f64,
) -> Result<Extent3d> {
    let missing = || SolidError::invalid_input(OP_FACE_BOX, "missing surface box");
    surface
        .calculate_bounding_box_for_face(k, uv_domain, [true, false, false], true, face_tolerance)?
        .normal_box
        .ok_or_else(missing)
}

/// `SmShell::CalculateBoundingBox(rBBox, bTight)` (`tight` defaults to `false`): `output`
/// is initialised, then holds a vertex shell's vertex point, the union of a wire
/// shell's edge boxes ([`crate::query::edge::calculate_bounding_box`] over
/// [`shell::wire_edges`]) or the union of a face shell's face boxes
/// ([`calculate_bounding_box`] of each face use's face, in face-use list order); it
/// stays empty for any other shell type.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a vertex shell without vertex or a face use without
/// face (upstream dereferences null); those of the child boxes and lists. `output` then
/// holds the union of the children before the failing one.
pub fn calculate_shell_bounding_box_into(
    k: &mut Kernel,
    b: &Brep,
    id: ShellId,
    tight: bool,
    output: &mut Extent3d,
) -> Result<()> {
    output.init();

    let s = b.shell(id)?;
    match s.shell_type {
        ShellType::Vertex => {
            let v = s.vertex(b)?.ok_or_else(|| {
                SolidError::invalid_input(OP_SHELL_BOX, "vertex shell without vertex")
            })?;
            output.add_point(vertex::point(b, v)?);
        }
        ShellType::Wire => {
            for e in shell::wire_edges(k, b, id)? {
                let this_box =
                    crate::query::edge::calculate_bounding_box(k, b, e, NORMAL_BOX, tight)?
                        .normal
                        .ok_or_else(|| {
                            SolidError::invalid_input(OP_SHELL_BOX, "missing edge box")
                        })?;
                *output = output.union(&this_box);
            }
        }
        ShellType::Faceuse => {
            for fu in s.get_faceuses()? {
                let f = b.faceuse(fu)?.face.ok_or_else(|| {
                    SolidError::invalid_input(OP_SHELL_BOX, "face use without face")
                })?;
                let this_box = calculate_bounding_box(k, b, f, tight)?;
                *output = output.union(&this_box);
            }
        }
        ShellType::Unknown | ShellType::Other(_) => {}
    }

    Ok(())
}

/// [`calculate_shell_bounding_box_into`], returning the box.
///
/// # Errors
///
/// Those of [`calculate_shell_bounding_box_into`].
pub fn calculate_shell_bounding_box(
    k: &mut Kernel,
    b: &Brep,
    id: ShellId,
    tight: bool,
) -> Result<Extent3d> {
    let mut output = Extent3d::new();
    calculate_shell_bounding_box_into(k, b, id, tight, &mut output)?;
    Ok(output)
}

/// `SmRegion::CalculateBoundingBox(rBBox, bTight)` (`tight` defaults to `false`): `output`
/// is initialised, then holds the union of the region's shell boxes
/// ([`calculate_shell_bounding_box_into`]) in shell list order.
///
/// # Errors
///
/// `SM_ERR` ([`SolidError::Upstream`]) for a region without shells; those of the shell
/// boxes. `output` then holds the union of the shells before the failing one.
pub fn calculate_region_bounding_box_into(
    k: &mut Kernel,
    b: &Brep,
    id: RegionId,
    tight: bool,
    output: &mut Extent3d,
) -> Result<()> {
    output.init();

    let shells = b.region(id)?.get_shells(None, |_, _| Ok(false))?;
    if shells.is_empty() {
        return Err(SolidError::sm_err(OP_REGION_BOX));
    }
    let mut this_box = Extent3d::new();
    for s in shells {
        calculate_shell_bounding_box_into(k, b, s, tight, &mut this_box)?;
        *output = output.union(&this_box);
    }

    Ok(())
}

/// [`calculate_region_bounding_box_into`], returning the box.
///
/// # Errors
///
/// Those of [`calculate_region_bounding_box_into`].
pub fn calculate_region_bounding_box(
    k: &mut Kernel,
    b: &Brep,
    id: RegionId,
    tight: bool,
) -> Result<Extent3d> {
    let mut output = Extent3d::new();
    calculate_region_bounding_box_into(k, b, id, tight, &mut output)?;
    Ok(output)
}

// ---- SmFace::Point3DClassify ---------------------------------------------------------------

/// The optional arguments of [`point_3d_classify`]; [`Default`] gives upstream's
/// defaults (`pOptGuessUV = NULL`, `bTgtLoopsOnly = FALSE`, `pOptTgtLoops = NULL`,
/// `bUVSpaceOkay = TRUE`).
#[derive(Debug, Clone, Copy)]
pub struct Point3dClassifyOptions<'a> {
    /// `pOptGuessUV`: a guess for a local point drop before the global one.
    pub guess_uv: Option<Point2>,
    /// `bTgtLoopsOnly`: only classify against [`Point3dClassifyOptions::tgt_loops`].
    pub tgt_loops_only: bool,
    /// `pOptTgtLoops`: with `tgt_loops_only`, the loops to classify against (`None` or
    /// empty: the outer loop).
    pub tgt_loops: Option<&'a [LoopId]>,
    /// `bUVSpaceOkay`: UV-space classification may be tried when the 3D one is doubtful.
    pub uv_space_okay: bool,
}

impl Default for Point3dClassifyOptions<'_> {
    fn default() -> Self {
        Self {
            guess_uv: None,
            tgt_loops_only: false,
            tgt_loops: None,
            uv_space_okay: true,
        }
    }
}

/// The geometry and stored topology tolerances of one B-rep, for a classification of
/// its entities.
struct BrepGeometry<'a> {
    b: &'a Brep,
}

impl ClassGeometry<EntityRef> for BrepGeometry<'_> {
    fn curve(&self, o: EntityRef) -> Option<&Curve> {
        match o {
            EntityRef::Edge(e) => edge::curve(self.b, e).ok().flatten(),
            _ => None,
        }
    }

    fn surface(&self, o: EntityRef) -> Option<&Surface> {
        match o {
            EntityRef::Face(f) => fp2::surface(self.b, f).ok().flatten(),
            _ => None,
        }
    }

    fn volume(&self, _o: EntityRef) -> Option<&Volume> {
        None
    }

    fn point(&self, o: EntityRef) -> Option<Point3> {
        match o {
            EntityRef::Vertex(v) => vertex::point(self.b, v).ok(),
            _ => None,
        }
    }

    fn classification_vertex_point(&self, v: EntityRef) -> Option<Point3> {
        self.point(v)
    }

    fn topology_zone_tol_3d(&self, o: EntityRef) -> Option<f64> {
        match o {
            EntityRef::Vertex(v) => vertex::tolerance(self.b, v).ok(),
            EntityRef::Edge(e) => edge::tolerance(self.b, e).ok(),
            EntityRef::Face(f) => fp2::tolerance(self.b, f).ok(),
            _ => None,
        }
    }
}

/// The UV start of the first solution of a point solve.
fn solution_uv(parameters: &[f64]) -> Result<Point2> {
    let missing = || SolidError::invalid_input(OP_POINT, "point solve without parameters");
    Ok(Point2::new(
        *parameters.first().ok_or_else(missing)?,
        *parameters.get(1).ok_or_else(missing)?,
    ))
}

/// `SmTemporaryChangeValue` for `Point3DClassify`'s actual cache. The native scope
/// also encloses UV classification and gap evaluation, not only the point solve.
fn with_point_test_disabled<T>(
    cache: Option<&mut crate::surface::cache_topology::OwnedSurfaceCache<EntityRef>>,
    body: impl FnOnce(
        Option<&mut crate::surface::cache_topology::OwnedSurfaceCache<EntityRef>>,
    ) -> Result<T>,
) -> Result<T> {
    let Some(cache) = cache else {
        return body(None);
    };
    let saved = cache.base().point_test_enabled;
    cache.base_mut().point_test_enabled = false;
    let result = body(Some(&mut *cache));
    cache.base_mut().point_test_enabled = saved;
    result
}

/// `SmFace::Point3DClassify`: classifies the 3D point `point` against face `f` into `pc`:
/// the point is dropped onto the face's surface within `src_zone_tol_3d` (a local drop
/// from [`Point3dClassifyOptions::guess_uv`] first, else the global drop over the face's
/// UV domain); a drop outside the UV domain (by more than a thousandth of its diagonal)
/// leaves `pc` unknown; else the UV point is classified by
/// [`super::p2::point_classify`], `pc`'s gap becomes the distance from `point` to the
/// surface point, and a classification to the face, an edge or a vertex whose gap
/// exceeds the intersection tolerance of `src_zone_tol_3d` and the classified object's
/// zone tolerance is reset to unknown. `pc` starts unknown on this face with source
/// tolerance `src_zone_tol_3d`; its other members are kept unless written.
///
/// `b` is mutable because [`super::p2::point_classify`] is.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a face without surface (upstream dereferences null);
/// those of the point solves and of [`super::p2::point_classify`]. `pc` keeps what was
/// written before the failure.
pub fn point_3d_classify(
    k: &Kernel,
    b: &mut Brep,
    f: FaceId,
    point: Point3,
    src_zone_tol_3d: f64,
    do_boundary_intersections: bool,
    pc: &mut PointClassification<EntityRef>,
    opts: Point3dClassifyOptions<'_>,
) -> Result<()> {
    let uv_domain = fp2::uv_domain(b, f)?;

    // init output - SM_PC_UNKNOWN is used for OUTSIDE the face's trim boundary
    pc.set_class_object(PointClassType::Unknown, None);
    pc.set_face(Some(EntityRef::Face(f)));
    pc.set_src_zone_tol_3d(src_zone_tol_3d);
    fp2::surface(b, f)?
        .ok_or_else(|| SolidError::invalid_input(OP_POINT, "face without surface"))?;

    // SmFace.cpp::Point3DClassify (maintained d8ed45a) checks out the actual
    // surface cache before either solve. Ordinary cache failure leaves the
    // native null-cache branch; an unimplemented dependency remains explicit.
    // The model borrow below supplies the real owner; no B-rep clone or detached
    // surface can substitute for it. Only exact evaluation geometry is copied.
    let mut cache = ignored(crate::topology::surface_cache::get_face_surface_cache(
        k, b, f, None,
    ))?;
    let reached_gap_check = with_point_test_disabled(cache.as_mut(), |cache| {
        let surface = fp2::surface(b, f)?
            .ok_or_else(|| SolidError::invalid_input(OP_POINT, "face without surface"))?
            .clone_for_evaluation()?;

        // when given a hint - use it in LocalSolve
        let mut found = None;
        if let Some(guess) = opts.guess_uv {
            // intersect 3dPoint with surface
            let r = surface.local_point_solve_for_face::<NoObject>(
                k,
                &uv_domain,
                SolverOperation::Intersect,
                point,
                guess,
                Some(fp2::tolerance(b, f)?),
            )?;
            if r.found {
                found = Some(solution_uv(&r.solution.start.parameters)?);
            }
        }

        // when Local Solve did not get an answer - try global solves
        let uv = if let Some(uv) = found {
            uv
        } else {
            // intersect 3DPoint with surface; no retries with larger tolerances
            let mut solutions = crate::geom::solution_array::SolutionArray::new();
            surface.global_point_solve_into_with_surface_cache(
                k,
                None,
                &uv_domain,
                SolverOperation::Intersect,
                point,
                src_zone_tol_3d,
                None,
                SolutionRequested::Single,
                &mut solutions,
                cache
                    .as_deref()
                    .map(crate::surface::cache_topology::OwnedSurfaceCache::view),
                &mut crate::topology::surface_cache::BrepSurfaceContext { brep: b, face: f },
            )?;
            let Some(first) = solutions.as_slice().first() else {
                // Point may not be inside of the boundary of the face.
                return Ok(false);
            };
            solution_uv(&first.start.parameters)?
        };

        // low work - UVPoint not contained by Face's domain
        let size = uv_domain.size().length();
        if !uv_domain.contains_point(uv, size / 1000.0) {
            return Ok(false);
        }

        // Classify against the same cached boundary after the untrimmed point
        // solve; native PointClassify itself does not use point_test_enabled.
        let trim_cache = match cache {
            Some(crate::surface::cache_topology::OwnedSurfaceCache::Trim(cache)) => {
                Some(cache.as_mut())
            }
            _ => None,
        };
        super::p2::point_classify(
            k,
            b,
            f,
            uv,
            pc,
            super::p2::PointClassifyOptions {
                src_zone_tol_3d,
                do_boundary_intersections,
                use_uv_classification: false,
                tgt_loops_only: opts.tgt_loops_only,
                tgt_loops: opts.tgt_loops,
                uv_space_okay: opts.uv_space_okay,
                trim_cache,
            },
        )?;

        // PointClassify set the gap to that of the surface point; use the distance
        // between the 3D point and its surface point instead.
        if let Some(surface_point) = ignored(surface.evaluate_point(k, uv))? {
            pc.set_gap_3d(point.distance(surface_point));
        }
        Ok(true)
    })?;
    if !reached_gap_check {
        return Ok(());
    }

    // If we classified to something but not within the specified tolerance, return no
    // classification.
    if pc.point_class() != PointClassType::Unknown && pc.point_class() != PointClassType::Region {
        let geometry = BrepGeometry { b };
        let xsect_tol_3d =
            xsect_tol_3d_from_zones(k, src_zone_tol_3d, pc.obj_zone_tol_3d(k, &geometry)?);
        if pc.gap_3d(k, &geometry, None)? > xsect_tol_3d {
            pc.set_class_object(PointClassType::Unknown, None);
        }
    }

    Ok(())
}

// ---- SmFace::EdgeCurveOnClassify -----------------------------------------------------------

/// Why a sector classification may be wrong (`SmClassifyProblem`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClassifyProblem {
    /// `SM_CP_NONE`: the classification can be trusted.
    #[default]
    None = 0,
    /// `SM_CP_VERTEX_ON_SINGULARITY`.
    VertexOnSingularity = 1,
    /// `SM_CP_TOGGLED_CLASSIFICATION`: growing step-offs changed it once.
    ToggledClassification = 2,
    /// `SM_CP_SHORT_SECTOR_EDGEUSE1`: the first edge use is shorter than the step-off.
    ShortSectorEdgeuse1 = 3,
    /// `SM_CP_SHORT_SECTOR_EDGEUSE2`: the second edge use is shorter than the step-off.
    ShortSectorEdgeuse2 = 4,
    /// `SM_CP_SHORT_INTERVAL`: the curve interval is shorter than the step-off.
    ShortInterval = 5,
    /// `SM_CP_SEESAW_CLASSIFICATION`: growing step-offs changed it more than once.
    SeesawClassification = 6,
    /// `SM_CP_UNRESOLVED_CLASSIFICATION`: the iteration limit was reached.
    UnresolvedClassification = 7,
}

/// The answers of [`edge_curve_on_classify`] (`rbIsInsideFace`, `rbPossibleProblem`,
/// `reClassifyProblem`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CurveOnClassification {
    /// Whether the classified curve segment is inside the face.
    pub inside: bool,
    /// Whether a classification reported a possible problem.
    pub possible_problem: bool,
    /// The problem of the last classification that reported one.
    pub problem: ClassifyProblem,
}

/// The arguments of [`edge_curve_on_classify`] after the edge and curve.
#[derive(Debug, Clone, Copy)]
pub struct EdgeCurveOnOptions {
    /// `dEdgeParam`: the edge parameter of the edge/curve intersection.
    pub edge_param: f64,
    /// `crCurveInterval`: the range of interest of the curve.
    pub curve_interval: Extent1d,
    /// `eCurveOrientation`: [`OrientType::Same`] for the curve segment after the
    /// intersection, [`OrientType::Opposite`] for the one before.
    pub curve_orientation: OrientType,
    /// `d3dCurveTolerance`: the zone tolerance of the curve's object.
    pub curve_tolerance: f64,
    /// `bUVSpaceOkay` (upstream default `TRUE`).
    pub uv_space_okay: bool,
    /// `lFaceIndx`, for debugging a target face (upstream default `SM_UNDEF_ULONG`).
    pub face_index: Option<u64>,
}

impl EdgeCurveOnOptions {
    /// The required arguments with upstream's defaults for the others.
    #[must_use]
    pub const fn new(
        edge_param: f64,
        curve_interval: Extent1d,
        curve_orientation: OrientType,
        curve_tolerance: f64,
    ) -> Self {
        Self {
            edge_param,
            curve_interval,
            curve_orientation,
            curve_tolerance,
            uv_space_okay: true,
            face_index: None,
        }
    }
}

/// `SmFace::SectorClassifyCurveOn` with the same edge use as both sector bounds (in
/// opposite directions), the edge's tolerance for both and no singularity.
fn sector_classify_curve_on(
    k: &Kernel,
    b: &mut Brep,
    f: FaceId,
    edgeuse: EdgeuseId,
    edge_tolerance: f64,
    curve: &Curve,
    opts: &EdgeCurveOnOptions,
) -> Result<CurveOnClassification> {
    use crate::query::face::p3::{SectorClassifyInput, SectorEdgeuse};
    let eu = |orientation| SectorEdgeuse {
        edgeuse,
        param: opts.edge_param,
        zone_tol_3d: edge_tolerance,
        orientation,
    };
    // `lFaceIndx` only selects debug output upstream
    let class = crate::query::face::p3::sector_classify_curve_on(
        k,
        b,
        f,
        &SectorClassifyInput {
            eu1: eu(OrientType::Same),
            eu2: eu(OrientType::Opposite),
            curve,
            curve_interval: opts.curve_interval,
            curve_orientation: opts.curve_orientation,
            curve_zone_tol_3d: opts.curve_tolerance,
            on_singularity: false,
            singularity_uv: Point2::new(0.0, 0.0),
            uv_space_okay: opts.uv_space_okay,
        },
    )?;
    Ok(CurveOnClassification {
        inside: class.inside,
        possible_problem: class.possible_problem,
        problem: class.problem,
    })
}

/// The 3D curve of a 2D `curve` on face `f`'s surface (`SmCrvOnSurf::SetAll`).
fn lift_to_face(k: &Kernel, b: &Brep, f: FaceId, curve: &Curve) -> Result<Curve> {
    let surface = fp2::surface(b, f)?
        .ok_or_else(|| SolidError::invalid_input(OP_EDGE_CURVE, "face without surface"))?;
    Ok(Curve::new(CurveKind::CrvOnSurf(Box::new(CrvOnSurf::new(
        k,
        curve.clone(),
        surface.clone_for_evaluation()?,
        None,
        0,
    )?))))
}

/// `SmFace::EdgeCurveOnClassify`: classifies the segment of `curve` that crosses the
/// face's boundary at `edge` (at [`EdgeCurveOnOptions::edge_param`]) as inside or outside
/// the face, against every upward edge use of the edge on this face (a seam edge has
/// two; an outside answer does not stop the search). A possible problem is passed on;
/// the first inside answer ends the search. A 2D `curve` is taken as a curve on the
/// face's surface.
///
/// `b` is mutable because the sector classification is.
///
/// # Errors
///
/// `SM_ERR_NULL_POINTER` for a null `edge`; those of the edge use list, the curve lift
/// and the sector classification
/// ([`crate::query::face::p3::sector_classify_curve_on`]).
pub fn edge_curve_on_classify(
    k: &Kernel,
    b: &mut Brep,
    f: FaceId,
    edge: Option<EdgeId>,
    curve: &Curve,
    opts: &EdgeCurveOnOptions,
) -> Result<CurveOnClassification> {
    let edge = edge.ok_or_else(|| SolidError::null_pointer(OP_EDGE_CURVE))?;

    // init outputs
    let mut out = CurveOnClassification::default();

    let edgeuses = edge::edgeuses(b, edge, None, None)?;
    let upward = fp2::upward_faceuse(b, f)?;

    // work in 3d: a 2D curve is taken as an SmCrvOnSurf on this face's surface
    let mut lifted = None;

    // for every edgeuse
    for eu in edgeuses {
        // skip all but the upward edgeuses (closed curves get 2) connected to this face
        if eu1::face(b, eu)? != Some(f) || eu1::faceuse(b, eu)? != upward {
            continue;
        }

        // Have found upward edgeuse connected to this face - classify curve relative to it
        let eu_tol = edge::tolerance(b, edge)?;
        let this_curve = if curve.dim() == 2 {
            if lifted.is_none() {
                lifted = Some(lift_to_face(k, b, f, curve)?);
            }
            lifted
                .as_ref()
                .ok_or_else(|| SolidError::invalid_input(OP_EDGE_CURVE, "missing lifted curve"))?
        } else {
            curve
        };
        let mine = sector_classify_curve_on(k, b, f, eu, eu_tol, this_curve, opts)?;

        // pass on any detected problems
        if mine.possible_problem {
            out.possible_problem = true;
            out.problem = mine.problem;
        }

        // pass on the answer
        if mine.inside {
            out.inside = true;
            return Ok(out);
        }

        // not all done: a seam edge has two upward edge uses connected to this face
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::line::Line;

    fn owned_triangle() -> (Kernel, Brep, FaceId) {
        use crate::geom::extent2d::Extent2d;
        use crate::math::Vec2;
        use crate::surface::plane::Plane;
        use crate::topology::brep_constructor::{BrepConstructor, VertexNode};

        let mut k = Kernel::default();
        let mut builder = BrepConstructor::new().unwrap();
        builder.start_shell().unwrap();
        let face = builder.start_face(OrientType::Same).unwrap();
        let domain = Extent2d::from_bounds(0., 0., 1., 1.);
        builder
            .set_face_surface(
                Plane::new(Point3::ZERO, Vec3::X, Vec3::Y, Vec2::new(1., 1.), &domain).into(),
                domain,
            )
            .unwrap();
        let points = [Point3::ZERO, Vec3::X, Vec3::Y];
        let vertices = points
            .iter()
            .map(|point| builder.start_vertex_of_loop(*point).unwrap())
            .collect::<Vec<_>>();
        builder.start_loop(OrientType::Same).unwrap();
        for index in 0..3 {
            let next = (index + 1) % 3;
            builder
                .start_edge(
                    &k,
                    OrientType::Same,
                    None,
                    Some(vertices[index]),
                    Some(vertices[next]),
                )
                .unwrap();
            builder
                .set_edge_curve(
                    &k,
                    Line::from_points(points[index], points[next], 3).into(),
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
    fn owned_point_3d_classify_matches_independent_triangle_and_plane_truth() {
        let (k, mut b, face) = owned_triangle();
        let owner = b.face(face).unwrap().surface.as_ref().unwrap().owner();
        let domain = b.face(face).unwrap().uv_domain;
        let intervals = b
            .edges()
            .iter()
            .map(|(_, edge)| edge.interval)
            .collect::<Vec<_>>();
        let vertices = b
            .vertices()
            .iter()
            .map(|(_, vertex)| vertex.point)
            .collect::<Vec<_>>();
        // Independent analytic oracle: z=0 and x>=0, y>=0, x+y<=1.
        // Boundary points and a point inside the plane's rectangular UV domain
        // but outside the triangle distinguish trim classification from inversion.
        for (point, expected) in [
            (Point3::new(0.2, 0.2, 0.), PointClassType::Face),
            (Point3::new(0.4, 0., 0.), PointClassType::Edge),
            (Point3::ZERO, PointClassType::Vertex),
            (Point3::new(0.8, 0.8, 0.), PointClassType::Unknown),
            (Point3::new(0.2, 0.2, 0.1), PointClassType::Unknown),
        ] {
            let mut classified = PointClassification::new(1e-6);
            point_3d_classify(
                &k,
                &mut b,
                face,
                point,
                1e-6,
                true,
                &mut classified,
                Point3dClassifyOptions::default(),
            )
            .unwrap();
            assert_eq!(classified.point_class(), expected, "point {point:?}");
            assert!(!b.editing_enabled());
        }
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
        assert_eq!(b.face(face).unwrap().uv_domain, domain);
        assert_eq!(
            b.edges()
                .iter()
                .map(|(_, edge)| edge.interval)
                .collect::<Vec<_>>(),
            intervals
        );
        assert_eq!(
            b.vertices()
                .iter()
                .map(|(_, vertex)| vertex.point)
                .collect::<Vec<_>>(),
            vertices
        );
    }

    #[test]
    fn point_3d_classify_cache_scope_restores_flag_after_early_return_and_errors() {
        let (k, mut b, face) = owned_triangle();
        let mut cache =
            crate::topology::surface_cache::get_face_surface_cache(&k, &mut b, face, None).unwrap();
        for saved in [false, true] {
            cache.base_mut().point_test_enabled = saved;
            let early = with_point_test_disabled(Some(&mut cache), |cache| {
                assert!(!cache.unwrap().base().point_test_enabled);
                Ok(false)
            })
            .unwrap();
            assert!(!early);
            assert_eq!(cache.base().point_test_enabled, saved);
            for error in [
                SolidError::invalid_input("public::point_test", "injected ordinary failure"),
                SolidError::unported("public cleanup injection", "query.face_p1"),
            ] {
                let result: Result<()> = with_point_test_disabled(Some(&mut cache), |cache| {
                    assert!(!cache.unwrap().base().point_test_enabled);
                    Err(error.clone())
                });
                assert_eq!(result, Err(error));
                assert_eq!(cache.base().point_test_enabled, saved);
            }
        }
        assert!(!b.editing_enabled());
    }

    #[test]
    fn borrowed_surface_solve_without_owning_model_keeps_explicit_owner_error() {
        let (k, b, face) = owned_triangle();
        let surface = b.face(face).unwrap().surface.as_ref().unwrap();
        let owner = surface.owner();
        let result = surface.global_point_solve::<NoObject>(
            &k,
            Some(NoObject),
            &b.face(face).unwrap().uv_domain,
            SolverOperation::Intersect,
            Point3::new(0.2, 0.2, 0.),
            1e-6,
            None,
            SolutionRequested::Single,
        );
        assert!(matches!(
            result,
            Err(SolidError::Unported {
                upstream: "SmFace handle of a surface's face",
                unit: "topology.face_p2",
            })
        ));
        assert_eq!(surface.owner(), owner);
    }

    #[test]
    fn evaluate_curve_rejects_null_nan_and_outside() {
        let k = &Kernel::default();
        assert!(evaluate_curve(k, None, 0.0).is_err());
        let c: Curve = Line::from_points(Vec3::new(0., 0., 0.), Vec3::new(1., 2., 3.), 3).into();
        assert!(evaluate_curve(k, Some(&c), f64::NAN).is_err());
        assert!(evaluate_curve(k, Some(&c), 2.0).is_err());
        let [p, d1, _] = evaluate_curve(k, Some(&c), 0.5).unwrap();
        assert_eq!(p, Vec3::new(0.5, 1.0, 1.5));
        assert_eq!(d1, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn region_without_shells_is_sm_err() {
        let mut k = Kernel::default();
        let mut b = Brep::default();
        let r = b
            .add_region_record(crate::topology::Region::default())
            .unwrap();
        let mut out = Extent3d::from_point(Vec3::new(1., 2., 3.));
        let e = calculate_region_bounding_box_into(&mut k, &b, r, false, &mut out).unwrap_err();
        assert!(matches!(e, SolidError::Upstream { .. }));
        assert_eq!(out, Extent3d::new());
    }

    #[test]
    fn null_edge_is_null_pointer() {
        let k = Kernel::default();
        let mut b = Brep::default();
        let c: Curve = Line::from_points(Vec3::new(0., 0., 0.), Vec3::new(1., 0., 0.), 3).into();
        let opts = EdgeCurveOnOptions::new(
            0.0,
            Extent1d::from_min_max(0.0, 1.0),
            OrientType::Same,
            1e-6,
        );
        let e = edge_curve_on_classify(&k, &mut b, FaceId::from_index(0), None, &c, &opts);
        assert!(e.is_err());
    }
}
