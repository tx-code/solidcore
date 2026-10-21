// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Normal projection of a complete curve, retaining its original parameters.
//!
//! Upstream: `source/SMLib/src/SmSurfaceDropLib.cpp:4194-6255`,
//! `source/SMLib/inc/SmSurface.h:219-249` (failure-record scopes).
//! | Rust | Upstream |
//! |------|----------|
//! | [`Surface::drop_curve`] | virtual `SmSurface::DropCurve` |
//! | [`CurveDrop`] | distance, deviation and UV output arguments |
//!
//! Deviations: values own unpublished curves (RAII deletes them on failure).
//! The temporary out-of-bounds flag is applied to an exact evaluation snapshot,
//! not to the caller's geometry. Cache acquisition uses the existing scoped
//! cache bridge; shared cache-manager checkout/alias effects remain unqualified.
//! Borrowed curve queries cannot persist lazy NURBs. The native owned-edge bridge
//! commits those effects at the original knot-query sites through a scoped hook.
//! Failed value-only gap queries propagate: inaccessible native partial outputs
//! are not replaced with zero. These limits do not qualify all native owners.
//! Iterative subdivision has a 1,000,000-step resource bound; exceeding it is an
//! error, never an accepted approximation. No defining geometry is fitted.

use super::p1::{
    pop_stack, push_point, remove_duplicate_drop_point_solutions, stack_look, DropFailRecord,
    DropPt, SM_SUCCESS,
};
use super::p2::{add_to_curve, flush_curve, DropOffset};
use crate::curve::crv_on_surf::CrvOnSurf;
use crate::curve::curve::p5::make_curve_cache;
use crate::curve::curve::Curve;
use crate::error::{ignored, Result, SolidError};
use crate::geom::extent1d::{Extent1d, ExtentPointType};
use crate::geom::extent2d::Extent2d;
use crate::geom::global_solver::{SolutionRequested, SolverOperation};
use crate::geom::point_set::PointContainment;
use crate::geom::solution_array::{SolutionArray, SolutionObject};
use crate::geom::tol::scaled_zero2;
use crate::geom::vector2d::Point2;
use crate::geom::vector3d::Vec3;
use crate::kernel::Kernel;
use crate::math::{max2, EFF_ZERO, EFF_ZERO_PARAM};
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::surface::cache_topology::{SurfaceCacheView, UnresolvedTopology};
use crate::surface::surface::p2::{
    surface_cache, CreateUvTrimCurvePathType as Path, DropCurveFail, DropCurveFailArgs,
    SurfaceKind, UNSURE,
};
use crate::surface::surface::p3::{set_out_of_bounds_enabled, Singularities};
use crate::surface::surface::p7::approx_derivative_lengths;
use crate::surface::surface::Surface;
use crate::surface::surface_cache::SurfaceCache;

const OP: &str = "intersect::surface_drop_lib::Surface::drop_curve";

/// Outputs of a complete-curve normal projection (`DropCurve`). Distances are
/// sampled native diagnostics, not continuous error bounds or import budgets.
#[derive(Debug, Default)]
pub struct CurveDrop {
    /// Largest distance along a surface normal, in model length units.
    pub max_drop_to_surf: f64,
    /// Largest lateral deviation from the normal line, in model length units.
    pub max_approx_dev: f64,
    /// UV images in native discovery order, with original curve parameters.
    pub uv_curves: Vec<Curve>,
}

#[allow(clippy::struct_excessive_bools)] // independent native classification/option flags
struct State<O> {
    input_domain: Extent2d,
    interval: Extent1d,
    tol: f64,
    keep_all: bool,
    degen_curve: bool,
    degen_u: bool,
    degen_v: bool,
    contained: bool,
    max_drop: f64,
    max_dev: f64,
    closed_u: bool,
    closed_v: bool,
    closure: SurfParamType,
    poles: Option<Singularities>,
    solutions: SolutionArray<O>,
    curves: Vec<BSplineCurve>,
    published_drop: f64,
    published_dev: f64,
}

impl<O: SolutionObject> State<O> {
    // Scopes 1..4 are the four native SM_SETn_DROP_CURVE_FAIL macros. In
    // particular scope 4 clears old intermediate polygon/knots on success.
    fn record(
        &self,
        record: &mut DropFailRecord<'_, O>,
        status: i32,
        path: Path,
        scope: u8,
        polygon: &[Vec3],
        knots: &[f64],
    ) {
        let poles = self.poles.as_ref().filter(|_| scope >= 2);
        record.fail.set(&DropCurveFailArgs {
            fail_status: status,
            from_call: 1,
            create_path_type: path,
            has_context: true,
            uv_domain: self.input_domain,
            surface: record.surface,
            curve: record.curve,
            interval: self.interval,
            approx_tol_3d: self.tol,
            max_drop_to_surf_arg: self.published_drop,
            max_approx_dev: self.published_dev,
            keep_all_drop_curves: i32::from(self.keep_all),
            degen_curve: i32::from(self.degen_curve),
            degen_surf_u: i32::from(self.degen_u),
            degen_surf_v: i32::from(self.degen_v),
            drop_domain_is_contained: i32::from(self.contained),
            max_drop_to_surf: self.max_drop,
            closed_u: if scope >= 2 {
                i32::from(self.closed_u)
            } else {
                UNSURE
            },
            closed_v: if scope >= 2 {
                i32::from(self.closed_v)
            } else {
                UNSURE
            },
            srf_closure: if scope >= 2 {
                self.closure
            } else {
                SurfParamType::Unknown
            },
            srf_pole_points: poles.and_then(|p| p.pole_points.as_ref().map(<[Vec3; 4]>::as_slice)),
            srf_pole_normals: poles
                .and_then(|p| p.pole_normals.as_ref().map(<[Vec3; 4]>::as_slice)),
            singularities: poles.map_or(0, |p| u64::from(p.singular)),
            approx_singularities: poles.map_or(0, |p| u64::from(p.approx_singular)),
            solutions: (scope >= 3).then_some(&self.solutions),
            ctrl_poly: (scope == 3).then_some(polygon),
            knots: (scope == 3).then_some(knots),
            uv_curves: (scope >= 3).then_some(self.curves.as_slice()),
            ..DropCurveFailArgs::default()
        });
    }

    fn output(self) -> CurveDrop {
        CurveDrop {
            max_drop_to_surf: self.published_drop,
            max_approx_dev: self.published_dev,
            uv_curves: self.curves.into_iter().map(Curve::from).collect(),
        }
    }
}

fn evaluate(curve: &Curve, point: &mut DropPt, nonzero: bool) -> Result<()> {
    let values = curve.evaluate(point.t, 2, true, nonzero)?;
    for (out, value) in point.crv_pd.iter_mut().zip(values) {
        *out = value;
    }
    Ok(())
}

fn uv<O: SolutionObject>(solutions: &SolutionArray<O>, index: usize) -> Result<Point2> {
    let s = solutions.get(index).ok_or_else(|| SolidError::sm_err(OP))?;
    Ok(Point2::new(s.start.parameters[0], s.start.parameters[1]))
}

fn set_uv<O: SolutionObject>(
    solutions: &mut SolutionArray<O>,
    index: usize,
    point: Point2,
) -> Result<()> {
    let s = solutions
        .get_mut(index)
        .ok_or_else(|| SolidError::sm_err(OP))?;
    s.start.parameters[0] = point.x;
    s.start.parameters[1] = point.y;
    Ok(())
}

impl Surface {
    /// Project a complete 3D curve onto this surface along its normals. A seam
    /// may yield two UV curves; a crossing/pole/off-domain walk may yield none.
    /// This is distinct from `drop_and_trim_curve`, which can retain partial pieces.
    /// `creating_uv_trim_curve` is used by the Plane override; the base ignores it.
    ///
    /// # Errors
    /// Propagated geometry/cache/solver errors; missing owning-model dependencies
    /// remain explicit. Failure releases unpublished UV candidates. The optional
    /// record retains only data available at the corresponding native stage.
    #[allow(clippy::too_many_arguments)]
    pub fn drop_curve<O: SolutionObject>(
        &self,
        k: &Kernel,
        domain: &Extent2d,
        curve: &Curve,
        interval: &Extent1d,
        approx_tol_3d: f64,
        keep_all: bool,
        record: Option<DropFailRecord<'_, O>>,
        creating_uv_trim_curve: bool,
    ) -> Result<CurveDrop> {
        self.drop_curve_with_knots(
            k,
            domain,
            curve,
            interval,
            approx_tol_3d,
            keep_all,
            record,
            creating_uv_trim_curve,
            &mut || Ok(()),
        )
    }

    /// Scoped bridge for native lazy `GetKnots` effects on the actual owning
    /// curve. The callback runs only at native knot-query sites, including
    /// failed projections; it must not edit the defining curve or parameters.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn drop_curve_with_knots<O: SolutionObject>(
        &self,
        k: &Kernel,
        domain: &Extent2d,
        curve: &Curve,
        interval: &Extent1d,
        approx_tol_3d: f64,
        keep_all: bool,
        record: Option<DropFailRecord<'_, O>>,
        creating_uv_trim_curve: bool,
        curve_knots: &mut impl FnMut() -> Result<()>,
    ) -> Result<CurveDrop> {
        if let SurfaceKind::Plane(p) = self.kind() {
            // Native virtual dispatch: Plane does not touch the failure record.
            let out = p.drop_curve(
                k,
                domain,
                curve,
                interval,
                approx_tol_3d,
                keep_all,
                creating_uv_trim_curve,
            )?;
            return Ok(CurveDrop {
                max_drop_to_surf: out.max_drop_to_surf,
                max_approx_dev: out.max_approx_dev,
                uv_curves: out.uv_curves,
            });
        }
        self.drop_curve_base_with_knots(
            k,
            domain,
            curve,
            interval,
            approx_tol_3d,
            keep_all,
            record,
            curve_knots,
        )
    }

    /// Qualified `SmSurface::DropCurve` base call. Plane's fallback must bypass
    /// virtual dispatch, otherwise its failed shortcut would call itself forever.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn drop_curve_base_with_knots<O: SolutionObject>(
        &self,
        k: &Kernel,
        domain: &Extent2d,
        curve: &Curve,
        interval: &Extent1d,
        approx_tol_3d: f64,
        keep_all: bool,
        record: Option<DropFailRecord<'_, O>>,
        curve_knots: &mut impl FnMut() -> Result<()>,
    ) -> Result<CurveDrop> {
        let mut local = DropCurveFail::default();
        let mut record = record.unwrap_or(DropFailRecord {
            fail: &mut local,
            surface: None,
            curve: None,
        });
        record.fail.un_init();
        let natural = self.natural_uv_domain();
        let curve_natural = curve.natural_interval();
        let length = curve.approximate_length(&curve_natural, 3, false)?.0;
        let mid = curve.evaluate_point(curve_natural.evaluate(0.5))?;
        let degen = curve.is_degenerate(scaled_zero2(&mid, &length), Some(interval))?;
        let input_domain = *domain;
        let domain = if domain.x_length() >= EFF_ZERO && domain.y_length() >= EFF_ZERO {
            *domain
        } else {
            natural
        };
        let mut state = State {
            input_domain,
            interval: *interval,
            tol: approx_tol_3d,
            keep_all,
            degen_curve: degen,
            degen_u: domain.x_length() < EFF_ZERO,
            // Native tests X twice (SmSurfaceDropLib.cpp:4244-4245).
            degen_v: domain.x_length() < EFF_ZERO,
            contained: domain.is_contained_by(&natural, EFF_ZERO),
            max_drop: 0.0,
            max_dev: 0.0,
            closed_u: false,
            closed_v: false,
            closure: SurfParamType::Neither,
            poles: None,
            solutions: SolutionArray::new(),
            curves: Vec::new(),
            published_drop: 0.0,
            published_dev: 0.0,
        };
        if state.degen_u || state.degen_v {
            state.record(&mut record, 1034, Path::DegenSurfaceFail, 1, &[], &[]);
            return Err(SolidError::sm_err(OP));
        }
        if !state.contained {
            state.record(
                &mut record,
                1020,
                Path::DropDomainUncontainedFail,
                1,
                &[],
                &[],
            );
            return Err(SolidError::sm_err(OP));
        }
        let size = approx_derivative_lengths(self, &natural)?;
        let uv_tol = Point2::new(
            approx_tol_3d / (size.x + 1.0),
            approx_tol_3d / (size.y + 1.0),
        );
        let mut working = self.clone_for_evaluation()?;
        set_out_of_bounds_enabled(&mut working, true);
        let mut cache = surface_cache::<O>(k, &working, None)?;
        // The scoped cache is not published. Restore even on a propagated error,
        // exactly as native SmTemporaryChangeValue does for checked-out caches.
        let flags = (cache.point_test_enabled, cache.process_boundary_curves);
        cache.point_test_enabled = false;
        cache.process_boundary_curves = true;
        let result = project(
            k,
            &working,
            &domain,
            curve,
            &mut cache,
            uv_tol,
            &mut state,
            &mut record,
            curve_knots,
        );
        cache.point_test_enabled = flags.0;
        cache.process_boundary_curves = flags.1;
        result?;
        Ok(state.output())
    }
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn project<O: SolutionObject>(
    k: &Kernel,
    surface: &Surface,
    domain: &Extent2d,
    curve: &Curve,
    cache: &mut SurfaceCache<O>,
    uv_tol: Point2,
    state: &mut State<O>,
    record: &mut DropFailRecord<'_, O>,
    curve_knots: &mut impl FnMut() -> Result<()>,
) -> Result<()> {
    let tol = state.tol;
    let natural = surface.natural_uv_domain();
    let poles = surface.get_singularities(k, true, true)?;
    state.poles = Some(poles);
    let mut offset = DropOffset::new(1.0, poles.singular);
    state.closed_u = surface
        .is_closed(&natural, SurfParamType::U, Some(tol), false)?
        .0;
    state.closed_v = surface
        .is_closed(&natural, SurfParamType::V, Some(tol), false)?
        .0;
    state.closure = match (state.closed_u, state.closed_v) {
        (true, true) => SurfParamType::Both,
        (true, false) => SurfParamType::U,
        (false, true) => SurfParamType::V,
        (false, false) => SurfParamType::Neither,
    };
    let mut start = DropPt::new(state.interval.min(), Some(state.degen_curve));
    let mut end = DropPt::new(state.interval.max(), Some(state.degen_curve));
    evaluate(curve, &mut start, true)?;
    surface.global_point_solve_into_with_surface_cache(
        k,
        None,
        domain,
        SolverOperation::Minimize,
        start.crv_pd[0],
        max2(tol / 100.0, EFF_ZERO),
        None,
        SolutionRequested::All,
        &mut state.solutions,
        Some(SurfaceCacheView::Surface(cache)),
        &mut UnresolvedTopology,
    )?;
    start.uv_pd[0] = uv(&state.solutions, 0)?;
    let _ =
        ignored(start.set_properties(k, surface, &mut offset, poles.singular, tol, domain, true))?;
    if start.good_drop == Some(false) {
        state.record(record, 1020, Path::FirstDropUncontainedFail, 2, &[], &[]);
        return Ok(());
    }
    remove_duplicate_drop_point_solutions(&mut state.solutions, uv_tol.x, uv_tol.y);
    start.uv_pd[0] = uv(&state.solutions, 0)?;
    let pole = if poles.singular != 0 {
        surface.is_singularity_with_poles(
            k,
            start.uv_pd[0],
            u64::from(poles.singular),
            poles.pole_points.as_ref().map(<[Vec3; 4]>::as_slice),
            tol,
            false,
        )?
    } else {
        crate::surface::surface::p5::Singularity {
            singular: false,
            direction: SurfParamType::Neither,
        }
    };
    if poles.singular != 0 && pole.singular {
        if state.degen_curve {
            let (min, mid, max) = (natural.min(), natural.mid(), natural.max());
            let (a, b) = if matches!(pole.direction, SurfParamType::U | SurfParamType::Both) {
                if start.uv_pd[0].y < mid.y {
                    (min, Point2::new(max.x, min.y))
                } else {
                    (max, Point2::new(min.x, max.y))
                }
            } else if start.uv_pd[0].x > mid.x {
                (Point2::new(max.x, min.y), max)
            } else {
                (Point2::new(min.x, max.y), min)
            };
            let mut line = BSplineCurve::create_line_segment(
                2,
                Vec3::new(a.x, a.y, 0.0),
                Vec3::new(b.x, b.y, 0.0),
            )?;
            line.edit_parameterization(&state.interval)?;
            state.curves.push(line);
            state.published_drop = start.drop_to_surf;
            state.published_dev = start.approx_dev;
            return Ok(());
        }
        for i in 0..state.solutions.len() {
            start.uv_pd[0] = uv(&state.solutions, i)?;
            let result = ignored(surface.find_degen_param_for_direction(
                start.uv_pd[0],
                start.crv_pd[1],
                true,
                pole.direction,
                0.0001,
            ))?;
            if let Some(p) = result.filter(|p| p.converged) {
                let mut value = uv(&state.solutions, i)?;
                if pole.direction == SurfParamType::U {
                    value.x = p.uv.x;
                }
                if pole.direction == SurfParamType::V {
                    value.y = p.uv.y;
                }
                set_uv(&mut state.solutions, i, value)?;
            } else {
                state.record(record, 1036, Path::BadFindDegenParamWarning, 4, &[], &[]);
            }
        }
        remove_duplicate_drop_point_solutions(&mut state.solutions, uv_tol.x, uv_tol.y);
        start.uv_pd[0] = uv(&state.solutions, 0)?;
    }
    seam_starts(surface, domain, uv_tol, &mut start, state)?;
    let mut curve_cache = None;
    for i in 0..state.solutions.len() {
        start.t = state.interval.min();
        start.uv_pd[0] = uv(&state.solutions, i)?;
        let uv_epsilon = EFF_ZERO_PARAM
            * (1.0
                + domain
                    .u_interval()
                    .length()
                    .min(domain.v_interval().length()));
        if !domain.contains_point(start.uv_pd[0], uv_epsilon) {
            continue;
        }
        let iso = surface.drop_iso_curve_with_knots(
            domain,
            curve,
            &state.interval,
            start.uv_pd[0],
            tol,
            curve_knots,
        )?;
        start.drop_to_surf = iso.max_drop_to_surf;
        start.approx_dev = iso.max_approx_dev;
        if iso.successful {
            state.curves.push(
                iso.iso_uv_curve
                    .ok_or_else(|| SolidError::null_pointer(OP))?,
            );
            if state.max_drop < start.drop_to_surf {
                state.max_drop = start.drop_to_surf;
            }
            if state.max_dev < start.approx_dev {
                state.max_dev = start.approx_dev;
            }
            continue;
        }
        if curve_cache.is_none() {
            // SmCurveCache tessellation queries the original curve's knots, not
            // its cache-owned copy (SmSurfaceDropLib.cpp:5024).
            curve_knots()?;
            curve_cache = Some(make_curve_cache::<O>(k, curve, None).map_err(|e| match e {
                e @ SolidError::Unported { .. } => e,
                _ => SolidError::null_pointer(OP),
            })?);
        }
        let cc = curve_cache
            .as_ref()
            .ok_or_else(|| SolidError::null_pointer(OP))?;
        let mut polygon = Vec::new();
        let mut knots = Vec::new();
        let mut have_tangent = false;
        let mut fix_boundary = false;
        'spans: for span in cc.spans() {
            let mut ivl = *span.interval();
            if ivl.max() <= start.t {
                continue;
            }
            if ivl.min() > start.t {
                let _ = ignored(ivl.set_min_max(start.t, ivl.max()))?;
            }
            if state.interval.are_disjoint(&ivl, 0.0) {
                continue;
            }
            if !ivl.is_contained_by(&state.interval, EFF_ZERO) {
                let Some(clipped) = ignored(ivl.intersect(&state.interval))? else {
                    continue;
                };
                if curve.approximate_length(&clipped, 5, false)?.0 < tol {
                    continue;
                }
                ivl = clipped;
            }
            start.t = ivl.min();
            end.t = ivl.max();
            evaluate(cc.curve(), &mut start, true)?;
            if !have_tangent {
                let _ = ignored(start.set_properties(
                    k,
                    surface,
                    &mut offset,
                    poles.singular,
                    tol,
                    domain,
                    true,
                ))?;
                if start.drop_to_surf.abs() > state.max_drop {
                    state.max_drop = start.drop_to_surf.abs();
                }
                if start.approx_dev > state.max_dev {
                    state.max_dev = start.approx_dev;
                }
            }
            let mut status = start.get_next_point(
                k,
                surface,
                &mut offset,
                poles.singular,
                poles.pole_points.as_ref().map(<[Vec3; 4]>::as_slice),
                domain,
                state.closure,
                cc.curve(),
                tol,
                end.t,
                &mut end,
            )?;
            if status == SM_SUCCESS {
                if end.drop_to_surf.abs() > state.max_drop {
                    state.max_drop = end.drop_to_surf.abs();
                }
                if end.approx_dev > state.max_dev {
                    state.max_dev = end.approx_dev;
                }
            }
            if (end.on_pole != Some(false) || end.leaving_domain != Some(false))
                && !state.interval.is_value_on_boundary(end.t, EFF_ZERO).0
            {
                status = 1020;
            }
            if status != SM_SUCCESS {
                if status == 1020
                    && start.on_boundary == Some(true)
                    && start.leaving_domain == Some(true)
                    && knots.len() > 1
                {
                    let tail = Extent1d::from_min_max(start.t, state.interval.max());
                    if curve.approximate_length(&tail, 5, false)?.0 < tol {
                        let p = curve.evaluate_point(state.interval.max())?;
                        if (start.pd_3d[0] - p).length() < tol {
                            if let Some(last) = knots.last_mut() {
                                *last = state.interval.max();
                            }
                        }
                        break;
                    }
                }
                if status == 1022 || status == 1021 {
                    if state.keep_all && end.approx_dev < tol {
                        status = SM_SUCCESS;
                        end.good_drop = Some(true);
                    } else {
                        state.record(record, 1021, Path::KeptBeyondTolDrop, 3, &polygon, &knots);
                    }
                }
                if status != SM_SUCCESS {
                    let path = record.fail.create_path_type;
                    state.record(record, status, path, 3, &polygon, &knots);
                    if record.fail.create_path_type == Path::Unknown {
                        record.fail.create_path_type = Path::DropStartPointFail;
                    }
                    polygon.clear();
                    knots.clear();
                    break;
                }
            }
            let mut stack = Vec::new();
            push_point(end.t, end.uv_pd, &mut stack);
            let mut steps = 0usize;
            while !stack.is_empty() {
                steps += 1;
                if steps > 1_000_000 {
                    return Err(SolidError::invalid_input(
                        OP,
                        "drop subdivision exceeds resource bound",
                    ));
                }
                (end.t, end.uv_pd) = stack_look(&stack)?;
                start.fix_boundary_tangents(domain, &mut end, uv_tol.x, uv_tol.y, tol);
                let accuracy = start.test_span_accuracy(
                    surface,
                    domain,
                    cc.curve(),
                    &end,
                    tol,
                    uv_tol.x,
                    uv_tol.y,
                    true,
                )?;
                if accuracy.passes || (end.t - start.t).abs() < state.interval.length() / 8000.0 {
                    if state.max_drop < accuracy.max_drop_to_surf {
                        state.max_drop = accuracy.max_drop_to_surf;
                    }
                    if state.max_dev < accuracy.max_approx_dev {
                        state.max_dev = accuracy.max_approx_dev;
                    }
                    // Source uses SE, not SER: ordinary add errors retain the added
                    // polygon and continue. Unported dependencies are never hidden.
                    let _ = ignored(add_to_curve(
                        k,
                        surface,
                        domain,
                        &start,
                        &end,
                        &mut polygon,
                        &mut knots,
                        tol,
                        uv_tol.x,
                        uv_tol.y,
                        &mut state.curves,
                        &mut fix_boundary,
                    ))?;
                    start = end;
                    start.from_left_u = Some(end.from_left_u == Some(false));
                    start.from_left_v = Some(end.from_left_v == Some(false));
                    have_tangent = true;
                    let _ = pop_stack(&mut stack)?;
                    continue;
                }
                let mut found = false;
                for _ in 0..6 {
                    end.t = (start.t + end.t) / 2.0;
                    status = start.get_next_point(
                        k,
                        surface,
                        &mut offset,
                        poles.singular,
                        poles.pole_points.as_ref().map(<[Vec3; 4]>::as_slice),
                        domain,
                        state.closure,
                        cc.curve(),
                        tol,
                        end.t,
                        &mut end,
                    )?;
                    if status == SM_SUCCESS {
                        found = true;
                        break;
                    }
                }
                if !found {
                    state.record(record, status, Path::SplitDropStepFail, 3, &polygon, &knots);
                    polygon.clear();
                    knots.clear();
                    break 'spans;
                }
                // Native split branch compares signed distance, unlike span endpoints.
                if state.max_drop < end.drop_to_surf {
                    state.max_drop = end.drop_to_surf;
                }
                if state.max_dev < end.approx_dev {
                    state.max_dev = end.approx_dev;
                }
                push_point(end.t, end.uv_pd, &mut stack);
            }
        }
        flush_curve(
            domain,
            &mut polygon,
            &mut knots,
            uv_tol.x,
            uv_tol.y,
            &mut state.curves,
            &mut fix_boundary,
        )?;
    }
    if state.curves.is_empty() {
        return Ok(());
    }
    state.published_drop = state.max_drop;
    state.published_dev = state.max_dev;
    let mut distances = Vec::new();
    for uv in &state.curves {
        let ivl = uv.natural_interval();
        let cos = CrvOnSurf::new(
            k,
            Curve::from(uv.clone()),
            surface.clone_for_evaluation()?,
            None,
            0,
        )?;
        // Native ignores the status but consumes its output variables. Our value
        // API cannot recover inaccessible failed outputs, so propagate that error
        // instead of manufacturing a zero-distance success.
        let gap = cos.surface_curve_max_distance_between(
            k,
            &ivl,
            curve,
            ivl.min(),
            ivl.max(),
            20,
            tol,
            None,
        )?;
        if gap.max_dist > state.published_drop {
            state.published_drop = gap.max_dist;
        }
        if gap.max_dev > state.published_dev {
            state.published_dev = gap.max_dev;
        }
        distances.push((gap.max_dist, gap.max_dev));
    }
    if !state.keep_all
        && (state.published_drop > tol || state.published_dev > tol)
        && state.curves.len() > 1
    {
        state.published_drop = 0.0;
        state.published_dev = 0.0;
        // Deliberately strict, signed, distance-only filtering. A single curve
        // is never culled by this native postpass, even when beyond tolerance.
        state.curves = std::mem::take(&mut state.curves)
            .into_iter()
            .zip(distances)
            .filter_map(|(curve, (dist, dev))| {
                if dist < tol {
                    if state.published_drop < dist {
                        state.published_drop = dist;
                    }
                    if state.published_dev < dev {
                        state.published_dev = dev;
                    }
                    Some(curve)
                } else {
                    None
                }
            })
            .collect();
    }
    let path = record.fail.create_path_type;
    state.record(record, SM_SUCCESS, path, 4, &[], &[]);
    if record.fail.create_path_type == Path::Unknown {
        record.fail.create_path_type = Path::DropCurveOkay;
    }
    Ok(())
}

fn seam_starts<O: SolutionObject>(
    surface: &Surface,
    domain: &Extent2d,
    tol: Point2,
    start: &mut DropPt,
    state: &mut State<O>,
) -> Result<()> {
    let natural = surface.natural_uv_domain();
    let (u, v) = natural.classify_point(start.uv_pd[0], tol.x, Some(tol.y));
    let boundary = |a| matches!(a, ExtentPointType::Start | ExtentPointType::End);
    let seam_u = boundary(u) && state.closed_u;
    let seam_v = boundary(v) && state.closed_v;
    if !seam_u && !seam_v {
        return Ok(());
    }
    let count = state.solutions.len();
    for i in 0..count {
        start.uv_pd[0] = uv(&state.solutions, i)?;
        let (u, v) = domain.classify_point(start.uv_pd[0], tol.x, Some(tol.y));
        let mut cu = if seam_u {
            start.is_moving_along_boundary(
                surface,
                &natural,
                tol.x,
                tol.y,
                5.0,
                SurfParamType::U,
            )?
        } else {
            PointContainment::Unknown
        };
        let mut cv = if seam_v {
            start.is_moving_along_boundary(
                surface,
                &natural,
                tol.x,
                tol.y,
                5.0,
                SurfParamType::V,
            )?
        } else {
            PointContainment::Unknown
        };
        let mut value = uv(&state.solutions, i)?;
        if cu == PointContainment::Outside {
            value.x = if u == ExtentPointType::Start {
                natural.max().x
            } else {
                natural.min().x
            };
        }
        if cv == PointContainment::Outside {
            value.y = if v == ExtentPointType::Start {
                natural.max().y
            } else {
                natural.min().y
            };
        }
        set_uv(&mut state.solutions, i, value)?;
        if cu == PointContainment::Unknown && seam_u {
            cu = PointContainment::OnBoundary;
        }
        if cv == PointContainment::Unknown && seam_v {
            cv = PointContainment::OnBoundary;
        }
        for (along, is_u) in [(cu, true), (cv, false)] {
            if along == PointContainment::OnBoundary {
                let mut original = *state
                    .solutions
                    .get(i)
                    .ok_or_else(|| SolidError::sm_err(OP))?;
                let mut other = original;
                if is_u {
                    original.start.parameters[0] = natural.min().x;
                    other.start.parameters[0] = natural.max().x;
                } else {
                    original.start.parameters[1] = natural.min().y;
                    other.start.parameters[1] = natural.max().y;
                }
                *state
                    .solutions
                    .get_mut(i)
                    .ok_or_else(|| SolidError::sm_err(OP))? = original;
                state.solutions.add(&other);
            }
        }
    }
    remove_duplicate_drop_point_solutions(&mut state.solutions, tol.x, tol.y);
    start.uv_pd[0] = uv(&state.solutions, 0)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::b_spline_surface::BSplineSurface;
    use crate::nurbs::math::{CPoint, NOW};
    use crate::topology::ObjectHandle;

    fn flat() -> Surface {
        let rows = [
            vec![CPoint::new(0., 0., 0., NOW), CPoint::new(0., 1., 0., NOW)],
            vec![CPoint::new(1., 0., 0., NOW), CPoint::new(1., 1., 0., NOW)],
        ];
        BSplineSurface::from_data(
            1,
            1,
            1,
            1,
            3,
            3,
            &rows,
            &[0., 0., 1., 1.],
            &[0., 0., 1., 1.],
        )
        .unwrap()
        .into()
    }

    fn line(a: Vec3, b: Vec3) -> Curve {
        let mut curve = BSplineCurve::create_line_segment(3, a, b).unwrap();
        curve
            .edit_parameterization(&Extent1d::from_min_max(2., 5.))
            .unwrap();
        curve.into()
    }

    #[test]
    fn complete_projection_retains_source_parameters_and_geometry() {
        let s = flat();
        let c = line(Vec3::new(0.1, 0.2, 0.), Vec3::new(0.9, 0.7, 0.));
        let r = s
            .drop_curve::<ObjectHandle>(
                &Kernel::default(),
                &s.natural_uv_domain(),
                &c,
                &c.natural_interval(),
                1e-5,
                true,
                None,
                true,
            )
            .unwrap();
        assert_eq!(r.uv_curves.len(), 1);
        assert_eq!(r.uv_curves[0].natural_interval(), c.natural_interval());
        for i in 0..=8 {
            let t = c.natural_interval().evaluate(f64::from(i) / 8.);
            let uv = r.uv_curves[0].evaluate_point(t).unwrap();
            assert!((uv - c.evaluate_point(t).unwrap()).length() < 1e-14);
        }
        assert!(!matches!(s.kind(), SurfaceKind::BSpline(b) if b.is_out_of_bounds_enabled()));
    }

    #[test]
    fn domain_rejection_precedes_cache_and_owned_knot_effects() {
        let s = flat();
        let c = line(Vec3::ZERO, Vec3::X);
        let mut fail = DropCurveFail::<ObjectHandle>::default();
        let mut knots = 0;
        let invalid = Extent2d::from_bounds(-0.1, 0., 1., 1.);
        let error = s
            .drop_curve_with_knots(
                &Kernel::default(),
                &invalid,
                &c,
                &c.natural_interval(),
                1e-5,
                true,
                Some(DropFailRecord {
                    fail: &mut fail,
                    surface: None,
                    curve: None,
                }),
                true,
                &mut || {
                    knots += 1;
                    Ok(())
                },
            )
            .unwrap_err();
        assert!(matches!(error, SolidError::Upstream { code: 1001, .. }));
        assert_eq!(knots, 0);
        assert_eq!(fail.fail_status, 1020);
        assert_eq!(fail.create_path_type, Path::DropDomainUncontainedFail);
        assert_eq!(fail.uv_domain, invalid);
        assert!(fail.solutions.is_empty());
        assert!(fail.uv_curves.is_empty());
    }

    #[test]
    fn owned_knot_effects_follow_iso_and_generic_query_sites() {
        let s = flat();
        let k = Kernel::default();
        for (a, b) in [
            (Vec3::new(0.1, 0.2, 0.), Vec3::new(0.9, 0.2, 0.)),
            (Vec3::new(0.1, 0.2, 0.), Vec3::new(0.9, 0.7, 0.)),
        ] {
            let c = line(a, b);
            let mut knots = 0;
            let r = s
                .drop_curve_with_knots::<ObjectHandle>(
                    &k,
                    &s.natural_uv_domain(),
                    &c,
                    &c.natural_interval(),
                    1e-5,
                    true,
                    None,
                    true,
                    &mut || {
                        knots += 1;
                        Ok(())
                    },
                )
                .unwrap();
            assert_eq!(knots, 1);
            assert_eq!(r.uv_curves.len(), 1);
        }
    }

    #[test]
    fn first_uncontained_drop_keeps_empty_output_and_failure_record() {
        let s = flat();
        let c = line(Vec3::new(-0.3, 0.2, 0.), Vec3::new(-0.1, 0.8, 0.));
        let mut fail = DropCurveFail::<ObjectHandle>::default();
        let mut knots = 0;
        let r = s
            .drop_curve_with_knots(
                &Kernel::default(),
                &s.natural_uv_domain(),
                &c,
                &c.natural_interval(),
                1e-5,
                true,
                Some(DropFailRecord {
                    fail: &mut fail,
                    surface: None,
                    curve: None,
                }),
                true,
                &mut || {
                    knots += 1;
                    Ok(())
                },
            )
            .unwrap();
        assert!(r.uv_curves.is_empty());
        assert_eq!(knots, 0);
        assert_eq!(fail.fail_status, 1020);
        assert_eq!(fail.create_path_type, Path::FirstDropUncontainedFail);
        assert_eq!(r.max_drop_to_surf, 0.);
    }
}
