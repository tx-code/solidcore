//! Vertex incidence and face-corner geometry over the actual owning model.
//!
//! Upstream: maintained `d8ed45a`, `SmVertex.cpp`1223--1364,1446--1707,
//! 1720--1801,1904--2302. `CalcCornerFromFaces` does not move the vertex.
//! | Rust | Upstream |
//! |------|----------|
//! | [`check_pointers`] | `CheckPointers` |
//! | [`within_faces`] | `IsWithinXSectTol3dOfFaces` |
//! | [`has_parallel_faces`] | `HasParallelFaces` |
//! | [`calc_corner_from_faces`] | `CalcCornerFromFaces` |
//! | [`is_on_face_corner`] | `IsOnFaceCorner` |
//! Deviations: checked ids, all requested outputs in values, no recursion;
//! unavailable native partial outputs are errors. Empty direct parallel scans
//! are checked errors instead of the source's unsigned count underflow.

mod links;
pub use links::check_pointers;

use super as vertex;
use crate::error::{ignored, Result, SolidError};
use crate::geom::tol::{self, SmallTopoType};
use crate::intersect::surface_intersector::p1::{IntersectionPointType, SurfaceIntersector};
use crate::kernel::Kernel;
use crate::math::{rad_to_deg, Point2, Point3, Vec3, BIG_DOUBLE};
use crate::nurbs::knots::Side;
use crate::surface::surface::{p1 as surface1, p5::DropPointOptions, Surface};
use crate::topology::{surface_cache, vertexuse as vu, Brep, FaceId, VertexId, VertexuseId};

const OP: &str = "topology::vertex::validity";
fn required<T>(v: Option<T>) -> Result<T> {
    v.ok_or_else(|| SolidError::null_pointer(OP))
}
fn surface(b: &Brep, f: FaceId) -> Result<&Surface> {
    required(b.face(f)?.surface.as_ref())
}
fn use_face(b: &Brep, v: VertexuseId) -> Result<Option<FaceId>> {
    vu::faceuse(b, v)?
        .map(|f| b.faceuse(f).map(|f| f.face))
        .transpose()
        .map(Option::flatten)
}

/// Native BRep-based intersection threshold; independent of import quality or
/// repair budgets. The null owning-model branch uses the system convention.
pub(crate) fn xsect(k: &Kernel, b: &Brep, v: VertexId) -> Result<f64> {
    Ok(if vertex::brep(b, v)?.is_some() {
        tol::xsect_tol3d(k, 2. * b.tolerance())
    } else {
        tol::context_xsect_tol_3d(k, None, SmallTopoType::NormalSize)
    })
}

/// Result of a cached/forced vertex-to-face intersection-tolerance check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceGapCheck {
    /// Every considered gap is strictly smaller than the `BRep` threshold.
    pub within: bool,
    /// Largest observed gap, initialized to zero as native.
    pub max_gap: f64,
    /// Face referenced by the actual winning gap record.
    pub max_face: Option<FaceId>,
}
/// Read/refresh actual vertex-face caches. Forced evaluation clears every use's
/// cache first; later accesses can share previously computed sibling results.
pub fn within_faces(k: &Kernel, b: &mut Brep, v: VertexId, force: bool) -> Result<FaceGapCheck> {
    let threshold = xsect(k, b, v)?;
    let uses = vertex::vertexuses(b, v)?;
    if force {
        for use_ in &uses {
            vu::init_vertex_face_gap(b, *use_)?;
        }
    }
    let mut out = FaceGapCheck {
        within: true,
        max_gap: 0.,
        max_face: None,
    };
    for use_ in uses {
        if vu::has_vertex_face_gap(b, use_)? {
            let gap = required(vu::vertex_face_gap(k, b, use_, false)?)?;
            out.within &= gap.gap_3d < threshold;
            if out.max_gap < gap.gap_3d {
                out.max_gap = gap.gap_3d;
                out.max_face = gap.vertexuse.map(|v| use_face(b, v)).transpose()?.flatten();
            }
        }
    }
    Ok(out)
}

/// Actual parallel-face scan output (including unset witnesses).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParallelFaces {
    /// A tested oriented-normal angle was below the requested angle.
    pub found: bool,
    /// Smallest measured angle in degrees; native BIG sentinel if untested.
    pub min_angle_degrees: f64,
    /// Largest separation of a parallel pair's evaluated points.
    pub max_gap: f64,
    /// Winning pair, including its UV witnesses; later equal gaps win.
    pub pair: Option<[(FaceId, Point2); 2]>,
}
/// Scan the native vertexuse choices and actual caches. The source helper
/// `GetVertexuseOfFace` ignores its face argument; that quirk remains in the
/// shared accessor. Normals are oriented angles, not absolute dot products.
pub fn has_parallel_faces(
    k: &Kernel,
    b: &mut Brep,
    v: VertexId,
    angle: Option<f64>,
) -> Result<ParallelFaces> {
    let faces = vertex::faces(b, v)?;
    if faces.is_empty() {
        return Err(SolidError::invalid_input(
            OP,
            "native empty parallel-face scan underflows",
        ));
    }
    let mut uses = Vec::new();
    for face in faces {
        uses.push(required(vertex::vertexuse_of_face(b, v, Some(face))?)?);
    }
    let mut out = ParallelFaces {
        found: false,
        min_angle_degrees: BIG_DOUBLE,
        max_gap: 0.,
        pair: None,
    };
    for (i, first) in uses.iter().copied().take(uses.len() - 1).enumerate() {
        let f1 = required(use_face(b, first)?)?;
        // Source reads the surface before asking for the cached gap.
        surface(b, f1)?;
        let Some(g1) = vu::vertex_face_gap(k, b, first, false)? else {
            continue;
        };
        let s1 = surface(b, f1)?;
        let p1 = s1.evaluate_point(k, g1.face_uv)?;
        let n1 = s1.evaluate_normal(k, g1.face_uv, Side::Right, Side::Right)?;
        for second in uses.iter().copied().skip(i + 1) {
            let f2 = required(use_face(b, second)?)?;
            surface(b, f2)?;
            let Some(g2) = vu::vertex_face_gap(k, b, second, false)? else {
                continue;
            };
            let s2 = surface(b, f2)?;
            let p2 = s2.evaluate_point(k, g2.face_uv)?;
            let n2 = s2.evaluate_normal(k, g2.face_uv, Side::Right, Side::Right)?;
            // SmVector3d.cpp442 initializes its angle output to zero before
            // failing on a zero vector; the caller ignores that status.
            let a = rad_to_deg(n1.angle_between(n2).unwrap_or(0.));
            if out.min_angle_degrees > a {
                out.min_angle_degrees = a;
            }
            if a < angle.unwrap_or(tol::ANG_TOL_DEG) {
                out.found = true;
                let gap = p1.distance(p2);
                if out.max_gap <= gap {
                    out.max_gap = gap;
                    out.pair = Some([(f1, g1.face_uv), (f2, g2.face_uv)]);
                }
            }
        }
    }
    Ok(out)
}

/// Result of native corner calculation. `max_gap` usually measures movement to
/// the first target surface's solution, not the maximum residual over all faces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Corner {
    /// A target point was established by the native algorithm.
    pub found: bool,
    /// Native output, initially -1; see the method's branch-specific semantics.
    pub max_gap: f64,
    /// Native maximum-gap face witness (usually unset outside parallel branch).
    pub max_face: Option<FaceId>,
    /// Suggested point. Never assigned to the defining vertex by this query.
    pub point: Point3,
}
impl Default for Corner {
    fn default() -> Self {
        Self {
            found: false,
            max_gap: -1.,
            max_face: None,
            point: Vec3::UNDEF,
        }
    }
}

/// Execute all native target counts: no-face, parallel-offset midpoint, one
/// surface drop, two-surface refinement, and three-surface intersection. Tolerances
/// and UV domains are those of their original owners; no fitting is introduced.
#[allow(clippy::too_many_lines)]
pub fn calc_corner_from_faces(k: &Kernel, b: &mut Brep, v: VertexId) -> Result<Corner> {
    let mut out = Corner::default();
    let uses = vertex::vertexuses(b, v)?;
    let faces = vertex::faces(b, v)?;
    if faces.is_empty() {
        return Ok(out);
    }
    let parallel = has_parallel_faces(k, b, v, Some(tol::ANG_TOL_DEG))?;
    let approx = tol::context_approx_tol_3d(k, Some(&k.context), SmallTopoType::NormalSize);
    if parallel.found && parallel.max_gap > approx {
        let [(f1, uv1), (f2, uv2)] = required(parallel.pair)?;
        out.found = true;
        out.point = (surface(b, f1)?.evaluate_point(k, uv1)?
            + surface(b, f2)?.evaluate_point(k, uv2)?)
            / 2.;
        let uses = vertex::vertexuses(b, v)?;
        for use_ in &uses {
            vu::init_vertex_face_gap(b, *use_)?;
        }
        // SmTemporaryChangeValue bypasses SetPoint/Notify and restores even on
        // failure. Cache invalidation on each side of the query is intentional.
        let original = b.vertex(v)?.point;
        b.vertex_mut(v)?.point = out.point;
        let checked = within_faces(k, b, v, true);
        b.vertex_mut(v)?.point = original;
        for use_ in &uses {
            vu::init_vertex_face_gap(b, *use_)?;
        }
        let checked = checked?;
        out.max_gap = checked.max_gap;
        out.max_face = checked.max_face;
        return Ok(out);
    }
    let mut targets = Vec::<(FaceId, Point2, Vec3)>::new();
    for use_ in uses {
        let Some(fu) = vu::faceuse(b, use_)? else {
            continue;
        };
        let face = required(b.faceuse(fu)?.face)?;
        let uv = vu::compute_uv_point(k, b, use_, false)?;
        let Some(surface) = b.face(face)?.surface.as_ref() else {
            continue;
        };
        // EvaluateNormal's explicit status guard skips failed normal evaluations.
        let Some(normal) = ignored(surface.evaluate_normal(k, uv, Side::Right, Side::Right))?
        else {
            continue;
        };
        if let Some((_, old, _)) = targets.iter().find(|(f, _, _)| *f == face) {
            let tol = surface1::map_to_2d(k, b.face(face)?.tolerance, uv, surface)?;
            if old.closer_than(tol, uv) {
                continue;
            }
        }
        // Preserve the fall-through switch: after three targets, remaining uses
        // still perform the UV/normal/duplicate calls but add no more targets.
        if targets.len() == 2 && normal.is_parallel_to(required(targets.get(1))?.2, 1.) {
            continue;
        }
        if (targets.len() == 1 || targets.len() == 2)
            && normal.is_parallel_to(required(targets.first())?.2, 1.)
        {
            continue;
        }
        if targets.len() < 3 {
            targets.push((face, uv, normal));
        }
    }
    let Some((first, guess, _)) = targets.first().copied() else {
        return Ok(out);
    };
    let point = b.vertex(v)?.point;
    let uv = match targets.as_slice() {
        [_] => {
            let domain = surface_cache::natural_domain_owned(k, b, first)?;
            let drop = surface_cache::drop_face_point(
                k,
                b,
                first,
                point,
                &domain,
                &DropPointOptions {
                    guess_uv: Some(guess),
                    ..DropPointOptions::default()
                },
            )?;
            out.max_gap = drop.gap;
            if !drop.success {
                return Ok(out);
            }
            drop.result_uv
        }
        [_, (second, guess2, _)] => {
            let d1 = surface_cache::natural_domain_owned(k, b, first)?;
            let d2 = surface_cache::natural_domain_owned(k, b, *second)?;
            let intersector = SurfaceIntersector::new(
                k,
                surface(b, first)?,
                &d1,
                surface(b, *second)?,
                &d2,
                false,
            )?;
            let Some(uvs) = intersector.refine_point(
                k,
                guess,
                *guess2,
                IntersectionPointType::Crossing,
                None,
                None,
                false,
            )?
            else {
                return Ok(out);
            };
            *required(uvs.first())?
        }
        [_, (second, guess2, _), (third, guess3, _)] => {
            let solution = surface(b, first)?.local_surface_surface_intersect(
                k,
                &b.face(first)?.uv_domain,
                surface(b, *second)?,
                &b.face(*second)?.uv_domain,
                surface(b, *third)?,
                &b.face(*third)?.uv_domain,
                guess,
                *guess2,
                *guess3,
            )?;
            let Some(solution) = solution else {
                return Ok(out);
            };
            solution.uv1
        }
        _ => return Err(SolidError::sm_err(OP)),
    };
    out.found = true;
    out.point = surface(b, first)?.evaluate_point(k, uv)?;
    out.max_gap = out.point.distance(b.vertex(v)?.point);
    Ok(out)
}

/// Native corner-location predicate and its branch-specific calculation result.
/// A no-answer calculation retains native true; it is not import acceptance.
pub fn is_on_face_corner(k: &Kernel, b: &mut Brep, v: VertexId) -> Result<(bool, Corner)> {
    let corner = calc_corner_from_faces(k, b, v)?;
    let zero = tol::ScaledZero::scaled_zero(&b.vertex(v)?.point);
    Ok((!corner.found || tol::in_tol(corner.max_gap, zero), corner))
}
