// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Source-ordered Edge/Edgeuse assertions over actual owned topology/geometry.
//!
//! Upstream: maintained `d8ed45a`, `SmEdge.cpp`5460--5510,6996--7031,
//! 7480--7750; `SmEdgeuse.cpp`6070--6354.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_edge`] | `SmEdge::AssertValid` |
//! | [`assert_edgeuse`] | `SmEdgeuse::AssertValid` |
//! Deviations: checked arena access replaces undefined native dereferences;
//! malformed radial cycles stop at100000 visits, above all native fixtures.
//! Inaccessible failed binormal partial outputs remain explicit missing evidence.
//! A completed local routine is not model/import qualification or repair.

use super::{
    local_base, AssertValidOptions, Brep, CheckLevel, EdgeId, EdgeuseId, Object, Reporter,
};
use crate::curve::{crv_on_surf::CrvOnSurf, curve::Curve};
use crate::geom::{
    assert_array::{labels, AssertArray},
    extent1d::Extent1d,
    extent2d::Extent2d,
    tol,
};
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::topology::{
    edge,
    edgeuse::{p1 as eu, p2 as eu2, p3 as eu3, validity},
    shell::{self, ShellType},
    surface_cache::natural_domain_owned,
    topology_traverser::Marks,
    EdgeuseOwner, VertexuseOwner,
};
use crate::{
    error::{Result, SolidError},
    kernel::Kernel,
    math::{rad_to_deg, Vec3, EFF_ZERO, UNDEF_DOUBLE},
};

fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::edge")
}
fn required<T>(value: Option<T>) -> Result<T> {
    value.ok_or_else(missing)
}
fn children(options: AssertValidOptions<'_>) -> AssertValidOptions<'_> {
    AssertValidOptions {
        level: if matches!(options.level, CheckLevel::Given(_)) {
            CheckLevel::Given(&[])
        } else {
            options.level
        },
        walk: false,
    }
}
fn binormal(k: &Kernel, b: &mut Brep, id: EdgeuseId, t: f64) -> Result<eu2::BinormalEval> {
    match eu3::evaluate_binormal(k, b, id, t, true, &eu3::BinormalOptions::default()) {
        Err(SolidError::Upstream { .. } | SolidError::Nlib { .. }) => Err(SolidError::unported(
            "edge AssertValid failed binormal partial outputs",
            "fork.topology_validation",
        )),
        r => r,
    }
}
fn on_face_edge_form(k: &Kernel, b: &mut Brep, id: EdgeId) -> Result<(bool, f64)> {
    let mut valid = true;
    let mut max = 0.;
    for f in edge::faces(b, id, None)? {
        if let Some(use_) = edge::edgeuse_of_face(b, id, Some(f))? {
            let face = required(eu::face(b, use_)?)?;
            let threshold =
                tol::xsect_tol_3d_from_zones(k, b.edge(id)?.tolerance, b.face(face)?.tolerance);
            if let Some(gap) = eu::max_edge_face_gap(k, b, use_, false)? {
                if max < gap.gap_3d {
                    max = gap.gap_3d;
                }
                if gap.gap_3d > threshold {
                    valid = false;
                }
            }
        }
    }
    Ok((valid, max))
}
fn orientations(b: &Brep, id: EdgeId) -> Result<bool> {
    let Some(primary) = edge::primary_edgeuse(b, id)? else {
        return Ok(true);
    };
    let mut current = primary;
    let mut valid = true;
    for _ in 0..super::MAX_LOOP_WALK {
        // Source obtains mate->radial before its NULL guard; a missing mate
        // is a checked access failure here, not a fabricated failed predicate.
        let mate = required(eu::mate(b, current)?)?;
        let Some(radial) = eu::radial(b, mate)? else {
            return Ok(valid);
        };
        valid &= b.edgeuse(current)?.orientation != b.edgeuse(mate)?.orientation;
        valid &= b.edgeuse(mate)?.orientation != b.edgeuse(radial)?.orientation;
        current = radial;
        if current == primary || !valid {
            return Ok(valid);
        }
    }
    Err(SolidError::invalid_input(
        "edge orientation walk",
        "radial cycle does not return to primary",
    ))
}
/// Execute the source Edge routine and its real child validators. Missing
/// callees/non-root traversal remain errors, never a partial successful result.
#[allow(clippy::too_many_lines)]
pub fn assert_edge(
    k: &mut Kernel,
    b: &mut Brep,
    id: EdgeId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.edge(id)?;
    let who = Object::Edge(id);
    let mut r = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::owning(k, b, who, &mut r)?
    };
    let c = b.edge(id)?.curve.as_ref();
    valid &= r.class_predicate(labels::EDGE_TYPE, 0, who, c.is_some())?;
    if let Some(c) = c {
        valid &= r.class_predicate(labels::EDGE_TYPE, 1, who, true)?;
        valid &= r.class_predicate(
            labels::EDGE_TYPE,
            2,
            who,
            b.edge(id)?
                .interval
                .is_contained_by(&c.natural_interval(), EFF_ZERO),
        )?;
        valid &= super::assert_edge_curve(k, b, id, children(options), r.reports.as_deref_mut())?;
    }
    let uses = b.edge(id)?.edgeuses.clone();
    valid &= r.class_predicate(labels::EDGE_TYPE, 3, who, !uses.is_empty())?;
    for use_ in uses {
        valid &= assert_edgeuse(k, b, use_, children(options), r.reports.as_deref_mut())?;
    }
    let record = b.edge(id)?;
    let tolerance = record.tolerance;
    let domain = record.interval;
    let length = record
        .curve
        .as_ref()
        .ok_or_else(missing)?
        .approximate_length(&domain, 5, false)?
        .0;
    valid &= r.class_value(
        labels::EDGE_TYPE,
        7,
        who,
        length >= tolerance - EFF_ZERO,
        tolerance,
        length - tolerance,
    )?;
    if options.level.native_code() >= 2 {
        let threshold = tol::xsect_tol_3d_from_zones(k, tolerance, tolerance);
        let (ok, gap) = on_face_edge_form(k, b, id)?;
        valid &= r.class_value(labels::EDGE_TYPE, 13, who, ok, threshold, gap)?;
    }
    valid &= r.class_predicate(labels::EDGE_TYPE, 14, who, orientations(b, id)?)?;
    let vertices = edge::vertices(b, id, None)?;
    valid &= r.class_predicate(
        labels::EDGE_TYPE,
        15,
        who,
        vertices.len() == 1 || vertices.len() == 2,
    )?;
    if vertices.len() == 1 || vertices.len() == 2 {
        if let Some(start) = edge::start_vertex(b, id)? {
            let p = b
                .edge(id)?
                .curve
                .as_ref()
                .ok_or_else(missing)?
                .evaluate_point(domain.min())?;
            valid &= r.class_predicate(
                labels::EDGE_TYPE,
                16,
                who,
                b.vertex(start)?.point.distance(p) <= tolerance + b.vertex(start)?.tolerance,
            )?;
            if let Some(end) = edge::other_vertex(b, id, Some(start))? {
                let p = b
                    .edge(id)?
                    .curve
                    .as_ref()
                    .ok_or_else(missing)?
                    .evaluate_point(domain.max())?;
                valid &= r.class_predicate(
                    labels::EDGE_TYPE,
                    17,
                    who,
                    b.vertex(end)?.point.distance(p) <= tolerance + b.vertex(end)?.tolerance,
                )?;
            }
        }
    }
    if edge::is_manifold(b, id)? || edge::is_spine(b, id)? {
        valid &= radial_order(k, b, id, &mut r)?;
    }
    if options.walk {
        valid &= super::assert_subtopology(
            k,
            b,
            crate::error::EntityRef::Edge(id),
            options.level,
            r.reports.as_deref_mut(),
        )?;
    }
    Ok(valid)
}
fn radial_order(k: &Kernel, b: &mut Brep, id: EdgeId, r: &mut Reporter<'_>) -> Result<bool> {
    let who = Object::Edge(id);
    let mut valid = true;
    let mut target = required(edge::primary_edgeuse(b, id)?)?;
    let mut radial = required(eu::radial(b, target)?)?;
    let mut primary = None;
    let mut primary_bi = Vec3::UNDEF;
    let mut primary_tan = Vec3::UNDEF;
    let mut angles = Vec::<f64>::new();
    let vertices = edge::vertices(b, id, None)?;
    let t = b.edge(id)?.interval.evaluate(0.456);
    let mut have_nonzero = false;
    for _ in 0..super::MAX_LOOP_WALK {
        if Some(target) == primary {
            return Ok(valid);
        }
        let a = matches!(b.edgeuse(target)?.owner, EdgeuseOwner::Loopuse(_));
        let d = matches!(b.edgeuse(radial)?.owner, EdgeuseOwner::Loopuse(_));
        valid &= r.class_predicate(labels::EDGE_TYPE, 8, who, a)?;
        valid &= r.class_predicate(labels::EDGE_TYPE, 8, who, d)?;
        if a && d {
            let first = binormal(k, b, target, t)?;
            let second = binormal(k, b, radial, t)?;
            if primary.is_none() {
                primary = Some(target);
                primary_bi = first.binormal;
                primary_tan = first.tangent;
            }
            let mut angle = rad_to_deg(
                primary_tan
                    .ccw_angle_between(primary_bi, second.binormal)
                    .unwrap_or(0.),
            );
            if angle < -EFF_ZERO {
                angle += 360.;
            }
            if have_nonzero && angle <= EFF_ZERO {
                angle += 360.;
            }
            // Preserve source's unusual assignment only inside this branch.
            if !have_nonzero && angle > 360. + EFF_ZERO {
                angle -= 360.;
                have_nonzero = angle.abs() > EFF_ZERO;
            }
            if let Some(previous) = angles.last() {
                if angle.abs() <= EFF_ZERO && primary == eu::mate(b, radial)? {
                    angle = 360.;
                }
                valid &= r.class_value(
                    labels::EDGE_TYPE,
                    9,
                    who,
                    angle > *previous - EFF_ZERO,
                    EFF_ZERO,
                    angle - *previous,
                )?;
                valid &= r.class_value(
                    labels::EDGE_TYPE,
                    10,
                    who,
                    Extent1d::from_min_max(0., 360.).contains_value(angle, EFF_ZERO),
                    EFF_ZERO,
                    UNDEF_DOUBLE,
                )?;
                for use_ in [target, radial] {
                    let vertex = b
                        .edgeuse(use_)?
                        .vertexuse
                        .map(|v| b.vertexuse(v).map(|r| r.vertex))
                        .transpose()?
                        .flatten();
                    valid &=
                        r.class_predicate(labels::EDGE_TYPE, 11, who, vertices.contains(&vertex))?;
                }
            }
            angles.push(angle);
        }
        target = required(eu::mate(b, radial)?)?;
        radial = required(eu::radial(b, target)?)?;
    }
    Err(SolidError::invalid_input(
        "edge radial order",
        "cycle does not return to primary",
    ))
}
fn wire_members(k: &mut Kernel, b: &mut Brep, id: crate::topology::ShellId) -> Result<Vec<EdgeId>> {
    let mut marks = Marks::from_brep(b)?;
    let result = shell::wire_edges_with_marks(k, b, id, &mut marks);
    marks.store_in_brep(b)?;
    result
}
/// Source use checks, including own-UV rules and mate-fallback pole query. The
/// native routine ignores its walk argument; it never asserts a modeling copy.
#[allow(clippy::too_many_lines)]
pub fn assert_edgeuse(
    k: &mut Kernel,
    b: &mut Brep,
    id: EdgeuseId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    let who = Object::Edgeuse(id);
    let mut r = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::topology_base(k, b, who, &mut r)?
    };
    let (owner, ccw, cw, vu, stored) = {
        let e = b.edgeuse(id)?;
        (e.owner, e.ccw, e.cw, e.vertexuse, e.uv_trim_curve.is_some())
    };
    let present = match owner {
        EdgeuseOwner::Loopuse(Some(v)) => {
            b.loopuse(v)?;
            true
        }
        EdgeuseOwner::Shell(Some(v)) => {
            b.shell(v)?;
            true
        }
        _ => false,
    };
    if present {
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 0, who, true)?;
    }
    for (index, use_) in [(1, ccw), (2, cw)] {
        if let Some(use_) = use_ {
            b.edgeuse(use_)?;
            valid &= r.class_predicate(labels::EDGEUSE_TYPE, index, who, true)?;
        }
    }
    if let Some(v) = vu {
        b.vertexuse(v)?;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 3, who, true)?;
    }
    if stored {
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 4, who, true)?;
    }
    if present {
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 5, who, true)?;
    }
    if stored {
        let face = eu::face(b, id)?;
        let edge = b.edgeuse(id)?.edge;
        let uv = b.edgeuse(id)?.uv_trim_curve.as_ref().ok_or_else(missing)?;
        let uv_domain = uv.natural_interval();
        let interval = edge
            .map(|e| b.edge(e).map(|e| e.interval))
            .transpose()?
            .unwrap_or(uv_domain);
        let curve_domain = edge
            .map(|e| b.edge(e))
            .transpose()?
            .and_then(|e| e.curve.as_ref())
            .map_or(uv_domain, Curve::natural_interval);
        let has_surface = face
            .map(|f| b.face(f).map(|f| f.surface.is_some()))
            .transpose()?
            .unwrap_or(false);
        let (domain, closed_u, closed_v) = if has_surface {
            let face = required(face)?;
            let d = natural_domain_owned(k, b, face)?;
            let s = b.face(face)?.surface.as_ref().ok_or_else(missing)?;
            (
                d,
                s.is_closed(&d, SurfParamType::U, None, false)?.0,
                s.is_closed(&d, SurfParamType::V, None, false)?.0,
            )
        } else {
            (Extent2d::new(), false, false)
        };
        let tolerance = if let Some(e) = edge {
            b.edge(e)?.tolerance
        } else if let Some(f) = face {
            b.face(f)?.tolerance
        } else {
            b.tolerance()
        };
        let uv = b.edgeuse(id)?.uv_trim_curve.as_ref().ok_or_else(missing)?;
        let owner_ok = uv.owner().is_some_and(|o| {
            o.type_code == labels::EDGEUSE_TYPE && o.key.0 == u64::from(id.index())
        });
        if owner_ok {
            let s = b
                .face(required(face)?)?
                .surface
                .as_ref()
                .ok_or_else(missing)?
                .clone_for_evaluation()?;
            let uv = super::geometry::borrowed_uv_copy(uv)?;
            let on_surface = Curve::from(CrvOnSurf::new(k, uv, s, None, 0)?);
            let length = on_surface.approximate_length(&interval, 5, false)?.0;
            valid &= r.class_value(
                labels::EDGEUSE_TYPE,
                11,
                who,
                length >= tolerance,
                tolerance,
                length,
            )?;
        }
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 6, who, owner_ok)?;
        let periodic = b
            .edgeuse_mut(id)?
            .uv_trim_curve
            .as_mut()
            .ok_or_else(missing)?
            .assert_no_periodic_jump(k, closed_u, closed_v, &domain)?;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 7, who, periodic)?;
        let min = 100. * tol::ScaledZero::scaled_zero(&curve_domain.min());
        let max = 100. * tol::ScaledZero::scaled_zero(&curve_domain.max());
        let unequal = (uv_domain.min() - interval.min()).abs() > min
            || (uv_domain.max() - interval.max()).abs() > max;
        let outside = uv_domain.min() < curve_domain.min() - min
            || uv_domain.max() > curve_domain.max() + max;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 25, who, !unequal)?;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 26, who, !outside)?;
    }
    if let EdgeuseOwner::Loopuse(v) = owner {
        let lu = required(v)?;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 17, who, true)?;
        valid &= r.class_predicate(
            labels::EDGEUSE_TYPE,
            14,
            who,
            eu::loopuse_edgeuses(b, lu)?.contains(&id),
        )?;
    } else {
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 17, who, true)?;
    }
    if let EdgeuseOwner::Shell(v) = owner {
        let s = required(v)?;
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 18, who, true)?;
        let has = match b.shell(s)?.shell_type {
            ShellType::Wire => wire_members(k, b, s)?.contains(&required(b.edgeuse(id)?.edge)?),
            ShellType::Faceuse => true,
            _ => false,
        };
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 13, who, has)?;
    } else {
        valid &= r.class_predicate(labels::EDGEUSE_TYPE, 18, who, true)?;
    }
    valid &= r.class_predicate(
        labels::EDGEUSE_TYPE,
        14,
        who,
        ccw.map(|e| b.edgeuse(e).map(|r| r.cw == Some(id)))
            .transpose()?
            .unwrap_or(true),
    )?;
    valid &= r.class_predicate(
        labels::EDGEUSE_TYPE,
        15,
        who,
        cw.map(|e| b.edgeuse(e).map(|r| r.ccw == Some(id)))
            .transpose()?
            .unwrap_or(true),
    )?;
    valid &= r.class_predicate(
        labels::EDGEUSE_TYPE,
        16,
        who,
        vu.map(|v| {
            b.vertexuse(v)
                .map(|r| matches!(r.owner,VertexuseOwner::Edgeuse(Some(e)) if e==id))
        })
        .transpose()?
        .unwrap_or(true),
    )?;
    valid &= r.class_predicate(labels::EDGEUSE_TYPE, 19, who, corner_vertex(b, id)?)?;
    if matches!(owner, EdgeuseOwner::Loopuse(_)) {
        let e = required(b.edgeuse(id)?.edge)?;
        if (edge::is_manifold(b, e)? || edge::is_spine(b, e)?)
            && b.edgeuse(id)?.orientation == OrientType::Same
        {
            valid &= use_pair(k, b, id, e, &mut r)?;
        }
    }
    if stored {
        valid &= super::assert_uv_curve(k, b, id, children(options), r.reports.as_deref_mut())?;
    }
    let dogleg = validity::dog_leg_near_pole(k, b, id)?.is_some();
    valid &= r.class_predicate(labels::EDGEUSE_TYPE, 20, who, !dogleg)?;
    Ok(valid)
}
fn corner_vertex(b: &Brep, id: EdgeuseId) -> Result<bool> {
    let use_ = b.edgeuse(id)?;
    let Some(vu) = use_.vertexuse else {
        return Ok(true);
    };
    let Some(v) = b.vertexuse(vu)?.vertex else {
        return Ok(true);
    };
    if edge::is_lamina(b, required(use_.edge)?)? {
        return Ok(true);
    }
    let Some(cw) = use_.cw else { return Ok(true) };
    let Some(mate) = eu::mate(b, cw)? else {
        return Ok(true);
    };
    let mate = b.edgeuse(mate)?;
    let Some(e) = mate.edge else { return Ok(true) };
    let Some(vu) = mate.vertexuse else {
        return Ok(true);
    };
    let Some(other) = b.vertexuse(vu)?.vertex else {
        return Ok(true);
    };
    Ok(edge::is_lamina(b, e)? || v == other)
}
fn use_pair(
    k: &Kernel,
    b: &mut Brep,
    id: EdgeuseId,
    edge: EdgeId,
    r: &mut Reporter<'_>,
) -> Result<bool> {
    let who = Object::Edgeuse(id);
    let radial = eu::radial(b, id)?;
    let mate = eu::mate(b, id)?;
    let mut valid = true;
    let same_shell = if let Some(radial) = radial {
        eu::shell(b, id)? == eu::shell(b, radial)?
    } else {
        false
    };
    valid &= r.class_predicate(labels::EDGEUSE_TYPE, 8, who, radial.is_some() && same_shell)?;
    // Preserve report 8 before accessing the mate's face: checked failures
    // must retain the same already-produced diagnostic prefix as native.
    let same_face = if let Some(mate) = mate {
        eu::face(b, id)? == eu::face(b, mate)?
    } else {
        false
    };
    valid &= r.class_predicate(labels::EDGEUSE_TYPE, 22, who, mate.is_some() && same_face)?;
    let orientation = b.edgeuse(id)?.orientation;
    for (index, use_) in [(23, radial), (24, mate)] {
        valid &= r.class_predicate(
            labels::EDGEUSE_TYPE,
            index,
            who,
            use_.map(|e| b.edgeuse(e).map(|r| r.orientation != orientation))
                .transpose()?
                .unwrap_or(false),
        )?;
    }
    if let (Some(radial), Some(_)) = (radial, mate) {
        let t = b.edge(edge)?.interval.evaluate(0.456);
        let a = binormal(k, b, id, t)?;
        let d = binormal(k, b, radial, t)?;
        let tangent = eu2::is_tangent_sector(k, b, id, 5., false, false)?;
        let dot = a.face_normal.dot(d.face_normal);
        valid &= r.class_value(
            labels::EDGEUSE_TYPE,
            9,
            who,
            !tangent || dot > -EFF_ZERO,
            EFF_ZERO,
            dot,
        )?;
        let cross = a
            .binormal
            .cross(d.binormal)
            .dot(a.face_normal.cross(d.face_normal));
        valid &= r.class_value(
            labels::EDGEUSE_TYPE,
            10,
            who,
            tangent || cross < EFF_ZERO,
            EFF_ZERO,
            cross,
        )?;
        valid &= r.class_predicate(
            labels::EDGEUSE_TYPE,
            27,
            who,
            a.point.distance(d.point) < b.edge(edge)?.tolerance,
        )?;
    }
    Ok(valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::curve::CurveKind;

    fn options(level: CheckLevel<'static>) -> AssertValidOptions<'static> {
        AssertValidOptions { level, walk: false }
    }

    #[test]
    fn given_edge_still_runs_level_two_gap_and_materializes_the_owned_curve() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let edge = b.edgeuse(use_).unwrap().edge.unwrap();
        let stored = |b: &Brep| match b.edge(edge).unwrap().curve.as_ref().unwrap().kind() {
            CurveKind::Line(c) => c.base().gw_nurb().is_some(),
            _ => panic!("fixture line"),
        };
        assert!(!stored(&b));
        assert!(assert_edge(
            &mut Kernel::default(),
            &mut b,
            edge,
            options(CheckLevel::Given(&[])),
            None
        )
        .unwrap());
        assert!(stored(&b));
        assert!(b.edgeuse(use_).unwrap().max_edge_face_gap.is_init());
        assert!(b
            .edgeuse(eu::mate(&b, use_).unwrap().unwrap())
            .unwrap()
            .max_edge_face_gap
            .is_init());
    }

    #[test]
    fn own_uv_owner_failure_does_not_apply_to_a_mate_borrow() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let mut k = Kernel::default();
        crate::topology::edgeuse::p3::get_or_create_uv_trim_curve(
            &k, &mut b, use_, false, false, true,
        )
        .unwrap()
        .status
        .unwrap();
        b.edgeuse_mut(use_)
            .unwrap()
            .uv_trim_curve
            .as_mut()
            .unwrap()
            .set_owner(None, None);
        let mate = eu::mate(&b, use_).unwrap().unwrap();
        let mut reports = AssertArray::new();
        assert!(assert_edgeuse(
            &mut k,
            &mut b,
            mate,
            options(CheckLevel::Given(&[])),
            Some(&mut reports)
        )
        .unwrap());
        assert!(reports.is_empty());
        assert!(!assert_edgeuse(
            &mut k,
            &mut b,
            use_,
            options(CheckLevel::Given(&[])),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports
                .reports()
                .iter()
                .map(|r| r.test_index)
                .collect::<Vec<_>>(),
            vec![6]
        );
        let count = reports.len();
        assert!(!assert_edgeuse(
            &mut k,
            &mut b,
            use_,
            options(CheckLevel::Given(&[])),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(reports.len(), count);
    }

    #[test]
    fn uv_parameter_mismatch_is_reported_without_reparameterizing_source() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let mut k = Kernel::default();
        crate::topology::edgeuse::p3::get_or_create_uv_trim_curve(
            &k, &mut b, use_, false, false, true,
        )
        .unwrap()
        .status
        .unwrap();
        let interval = Extent1d::from_min_max(0., 2.);
        b.edgeuse_mut(use_)
            .unwrap()
            .uv_trim_curve
            .as_mut()
            .unwrap()
            .edit_parameterization(&k, &interval)
            .unwrap();
        let mut reports = AssertArray::new();
        assert!(!assert_edgeuse(
            &mut k,
            &mut b,
            use_,
            options(CheckLevel::Given(&[])),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports
                .reports()
                .iter()
                .map(|r| r.test_index)
                .collect::<Vec<_>>(),
            vec![25, 26]
        );
        assert_eq!(
            b.edgeuse(use_)
                .unwrap()
                .uv_trim_curve
                .as_ref()
                .unwrap()
                .natural_interval(),
            interval
        );
    }

    #[test]
    fn start_gap_accepts_equality_and_rejects_the_next_binary64_value() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let mut k = Kernel::default();
        let edge = b.edgeuse(use_).unwrap().edge.unwrap();
        let vertex = edge::start_vertex(&b, edge).unwrap().unwrap();
        let limit = b.edge(edge).unwrap().tolerance + b.vertex(vertex).unwrap().tolerance;
        let original = b.vertex(vertex).unwrap().point;
        b.vertex_mut(vertex).unwrap().point = original + Vec3::new(0., 0., limit);
        assert!(assert_edge(&mut k, &mut b, edge, options(CheckLevel::Zero), None).unwrap());
        b.vertex_mut(vertex).unwrap().point =
            original + Vec3::new(0., 0., f64::from_bits(limit.to_bits() + 1));
        let mut reports = AssertArray::new();
        assert!(!assert_edge(
            &mut k,
            &mut b,
            edge,
            options(CheckLevel::Zero),
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports
                .reports()
                .iter()
                .map(|r| (r.reporting_type, r.test_index))
                .collect::<Vec<_>>(),
            vec![(labels::EDGE_TYPE, 16)]
        );
    }
}
