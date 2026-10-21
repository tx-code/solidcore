// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Ordered Face-local geometry and use assertions from the maintained SDK.
//!
//! Upstream: `SmFace.cpp::AssertValid` (20789--21142), `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_face`] | `SmFace::AssertValid` |
//! Deviations: checked handles replace undefined native dereferences. Periodic
//! seam classification, surface-local assertions and unavailable geometric paths
//! remain explicit dependencies. Native boolean and numerical evidence differ.

use super::{
    face_tests as tests, local, local_base, uses, AssertValidOptions, Brep, CheckLevel, Object,
    Reporter,
};
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray};
use crate::intersect::point_class::{PointClassType, PointClassification};
use crate::kernel::Kernel;
use crate::math::{EFF_ZERO, EFF_ZERO_DEG, UNDEF_DOUBLE};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::query::face::{p1 as qf1, p2 as qf2};
use crate::topology::{
    brep_cache::{arena_identity, CacheObject},
    face, vertexuse, FaceId,
};

fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::assert_face")
}

fn needed(level: CheckLevel<'_>, test: u64, minimum: i32) -> bool {
    match level {
        CheckLevel::Given(requests) => requests.contains(&test),
        CheckLevel::Heal => false,
        _ => level.native_code() >= minimum,
    }
}

/// Execute the actual Face sequence. Source explicit request guards and Release
/// report-macro levels are intentionally different; Given is not a blanket filter.
#[allow(clippy::too_many_lines)]
pub fn assert_face(
    k: &mut Kernel,
    b: &mut Brep,
    id: FaceId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.face(id)?;
    let who = Object::Face(id);
    let mut report = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::topology_base(k, b, who, &mut report)?
    };
    let face_tol = b.face(id)?.tolerance;
    let primary = b.face(id)?.faceuse.ok_or_else(missing)?;
    b.faceuse(primary)?;
    valid &= report.class_predicate(labels::FACE_TYPE, 0, who, true)?;
    if let Some(surface) = b.face(id)?.surface.as_ref() {
        let domain = b.face(id)?.uv_domain;
        valid &= report.class_predicate(labels::FACE_TYPE, 1, who, true)?;
        valid &= report.class_value(
            labels::FACE_TYPE,
            2,
            who,
            domain.is_contained_by(&surface.natural_uv_domain(), EFF_ZERO),
            EFF_ZERO,
            UNDEF_DOUBLE,
        )?;
        valid &= report.class_value(
            labels::FACE_TYPE,
            22,
            who,
            domain.x_length() > EFF_ZERO && domain.y_length() > EFF_ZERO,
            EFF_ZERO,
            UNDEF_DOUBLE,
        )?;
    }
    // This check is present in the maintained RELEASE profile, and is not an
    // early return. A missing surface later reaches a checked native-deref error.
    valid &= report.class_predicate(labels::FACE_TYPE, 3, who, b.face(id)?.surface.is_some())?;
    let owner_guard = [4, 5, 6, 23, 24]
        .into_iter()
        .any(|test| needed(options.level, test, 0));
    if owner_guard {
        if let Some(surface) = b.face(id)?.surface.as_ref() {
            if let Some(owner) = surface.face()? {
                let owner_tol = crate::topology::surface_cache::owning_face_tolerance(b, owner)?;
                valid &= report.class_predicate(
                    labels::FACE_TYPE,
                    4,
                    who,
                    owner == arena_identity(CacheObject::Face(id)),
                )?;
                valid &= report.class_value(
                    labels::FACE_TYPE,
                    23,
                    who,
                    b.face(id)?
                        .uv_domain
                        .is_contained_by(&surface.natural_uv_domain(), EFF_ZERO),
                    0.,
                    UNDEF_DOUBLE,
                )?;
                let poles = surface.get_singularities_of_face(k, owner_tol, true, false)?;
                if poles.singular != 0 {
                    for point in poles.pole_points.ok_or_else(missing)? {
                        if point.is_undef() {
                            continue;
                        }
                        let mut classified = PointClassification::<EntityRef>::new(owner_tol);
                        local::ignore_native_status(qf1::point_3d_classify(
                            k,
                            b,
                            id,
                            point,
                            owner_tol,
                            true,
                            &mut classified,
                            qf1::Point3dClassifyOptions::default(),
                        ))?;
                        valid &= report.class_predicate(
                            labels::FACE_TYPE,
                            24,
                            who,
                            !matches!(
                                classified.point_class(),
                                PointClassType::Face | PointClassType::Edge
                            ),
                        )?;
                    }
                }
            } else {
                valid &= report.class_predicate(labels::FACE_TYPE, 35, who, false)?;
            }
        }
    }
    // Source only needs one connected VU per logical vertex. ComputeUVPoint may
    // materialize derived UV state; a failed UV computation uses the actual 3D
    // classifier. The classifiers' statuses are ignored, their written state is not.
    if needed(options.level, 12, 0) || needed(options.level, 13, 0) {
        for vertex in qf2::face_vertices(b, id, false, None)? {
            let mut connected = false;
            for vu in b.vertex(vertex)?.vertexuses.clone() {
                // SmTol.cpp764 dispatches to SmTopology::GetTolerance for
                // Vertexuse, which has NO override. Do not use Vertex.tolerance.
                let vu_tol = crate::geom::tol::ZONE_TOL_3D;
                if let Some(fu) = vertexuse::faceuse(b, vu)? {
                    if b.faceuse(fu)?.face == Some(id) {
                        connected = true;
                        let mut classified = PointClassification::<EntityRef>::new(face_tol);
                        if let Some(uv) = local::ignore_native_status(vertexuse::compute_uv_point(
                            k, b, vu, true,
                        ))? {
                            local::ignore_native_status(qf2::point_classify(
                                k,
                                b,
                                id,
                                uv,
                                &mut classified,
                                qf2::PointClassifyOptions {
                                    src_zone_tol_3d: vu_tol,
                                    do_boundary_intersections: true,
                                    use_uv_classification: true,
                                    ..qf2::PointClassifyOptions::default()
                                },
                            ))?;
                        } else {
                            local::ignore_native_status(qf1::point_3d_classify(
                                k,
                                b,
                                id,
                                b.vertex(vertex)?.point,
                                vu_tol,
                                true,
                                &mut classified,
                                qf1::Point3dClassifyOptions::default(),
                            ))?;
                        }
                        valid &= report.class_predicate(
                            labels::FACE_TYPE,
                            12,
                            who,
                            classified.point_class() != PointClassType::Unknown,
                        )?;
                        break;
                    }
                }
            }
            valid &= report.class_predicate(labels::FACE_TYPE, 13, who, connected)?;
        }
    }
    // Connected faceuses are ALWAYS visited, outside the request guard above.
    let (first, second) = face::p2::faceuses(b, id)?;
    let child_options = AssertValidOptions {
        level: if matches!(options.level, CheckLevel::Given(_)) {
            CheckLevel::Given(&[])
        } else {
            options.level
        },
        walk: false,
    };
    if let Some(fu) = first {
        valid &= uses::assert_faceuse(k, b, fu, child_options, report.reports.as_deref_mut())?;
    }
    if let Some(fu) = second {
        valid &= uses::assert_faceuse(k, b, fu, child_options, report.reports.as_deref_mut())?;
    }
    let face_tol = b.face(id)?.tolerance;
    let domain = b.face(id)?.uv_domain;
    let surface = b.face(id)?.surface.as_ref().ok_or_else(missing)?;
    valid &= report.class_value(
        labels::FACE_TYPE,
        27,
        who,
        !surface.is_degenerate_point(face_tol / 4., Some(&domain))?,
        face_tol / 4.,
        UNDEF_DOUBLE,
    )?;
    valid &= report.class_value(
        labels::FACE_TYPE,
        28,
        who,
        !surface.is_degenerate_curve(EFF_ZERO_DEG, Some(&domain), face_tol / 4.)?,
        face_tol / 4.,
        UNDEF_DOUBLE,
    )?;
    // Release's macros use only the numeric comparison; Heal and Given still
    // execute these LEVEL2 predicates. Skipped macros reset LastAdd as native.
    let check_2 = options.level.native_code() >= 2;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        32,
        who,
        !check_2 || tests::coincident_vertices(b, id, false)?.is_empty(),
    )?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        33,
        who,
        !check_2 || !tests::coincident_edges(k, b, id)?,
    )?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        34,
        who,
        !check_2 || !tests::partially_coincident_edges(k, b, id)?,
    )?;
    valid &= report.class_predicate(labels::FACE_TYPE, 37, who, tests::nested_loops(k, b, id)?)?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        38,
        who,
        !check_2 || b.face(id)?.surface.is_none() || tests::disjoint_loops(k, b, id)?,
    )?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        39,
        who,
        tests::distinct_loop_vertices(b, id)?,
    )?;
    // HasSeamProblem is an unconditional call, even when LEVEL2 reports skip.
    let seams = tests::seam_problem(k, b, id)?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        29,
        who,
        !check_2 || seams.crossed == SurfParamType::Neither,
    )?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        31,
        who,
        !check_2 || seams.missing == SurfParamType::Neither,
    )?;
    valid &= report.class_predicate(
        labels::FACE_TYPE,
        36,
        who,
        !check_2 || seams.near_miss == SurfParamType::Neither,
    )?;
    // Composite/obsolete rules remain disabled. Rule30 is SM_NMTLIB_7166-only.
    if b.face(id)?.surface.is_some() && !matches!(options.level, CheckLevel::Given(_)) {
        valid &= super::assert_surface(k, b, id, child_options, report.reports.as_deref_mut())?;
    }
    if options.walk {
        valid &= super::assert_subtopology(
            k,
            b,
            EntityRef::Face(id),
            options.level,
            report.reports.as_deref_mut(),
        )?;
    }
    Ok(valid)
}
