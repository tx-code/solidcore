// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Ordered body of `SmBrep::ValidatePointers`, tests 0--111, at `d8ed45a`.
//! Native status, report selection, and disabled predicates remain distinct.
//! No geometric predicate is replaced by a topology or sampled stand-in.
use super::{bp, geometry, Brep, Object, Reporter, ValidationOptions, MAX_LOOP_WALK, OP};
use crate::error::{ignored, Result, SolidError};
use crate::kernel::Kernel;
use crate::math::{Point2, Point3, EFF_ZERO, UNDEF_DOUBLE};
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::topology::shell::{ShellMembers, ShellType};
use crate::topology::{
    edge, edgeuse::p1 as eu, face::p2 as face, loopuse, EdgeuseOwner, FaceId, LoopuseId,
    LoopuseStart, VertexuseOwner,
};
use std::collections::BTreeSet;

struct Checks<'a, 'b, 'c> {
    report: &'a mut Reporter<'b>,
    options: ValidationOptions<'c>,
    ok: bool,
}
impl Checks<'_, '_, '_> {
    fn selected(&self, _test: u64) -> bool {
        self.options.level.native_code() >= 0
    }
    fn test(&mut self, test: u64, owner: Object, passed: bool) -> Result<()> {
        self.ok &= self
            .report
            .predicate(test, owner, !self.selected(test) || passed)?;
        Ok(())
    }
    fn value(
        &mut self,
        test: u64,
        owner: Object,
        passed: bool,
        tol: f64,
        value: f64,
    ) -> Result<()> {
        // The Release VALUE macro tests numeric level, not Given requests. All
        // predicates here are level 0, so values still run for SM_LEVEL_GIVEN.
        self.ok &= self.report.value(test, owner, passed, tol, value)?;
        Ok(())
    }
    fn required<T>(&mut self, test: u64, owner: Object, value: Option<T>) -> Result<T> {
        if let Some(v) = value {
            Ok(v)
        } else {
            self.test(test, owner, false)?;
            Err(SolidError::Upstream {
                operation: OP,
                code: i32::from(self.ok),
                entity: None,
            })
        }
    }
}

pub(super) fn check(
    k: &mut Kernel,
    b: &mut Brep,
    options: ValidationOptions<'_>,
    report: &mut Reporter<'_>,
) -> Result<()> {
    let mut c = Checks {
        report,
        options,
        ok: true,
    };
    let tol = b.tolerance();
    c.test(0, Object::Brep, tol > EFF_ZERO)?;
    // Use the actual owning query: its context mark advancement is observable.
    let shells: BTreeSet<_> = bp::shells(k, b, None)?.into_iter().collect();
    let mut lus = BTreeSet::new();
    for f in b.face_list() {
        let fu = b.face(*f)?.faceuse.ok_or(SolidError::null_pointer(OP))?;
        let mate = b.faceuse(fu)?.mate.ok_or(SolidError::null_pointer(OP))?;
        for id in [fu, mate] {
            lus.extend(b.faceuse(id)?.loopuses.iter().copied());
        }
    }
    for e in b.edge_list() {
        for id in &b.edge(*e)?.edgeuses {
            let live = match b.edgeuse(*id)?.owner {
                EdgeuseOwner::Shell(Some(s)) => shells.contains(&s),
                EdgeuseOwner::Loopuse(Some(l)) => {
                    lus.contains(&l) && loopuse::shell(b, l)?.is_some()
                }
                _ => false,
            };
            if !live {
                c.test(98, Object::Edgeuse(*id), false)?;
                return Err(SolidError::sm_err(OP));
            }
        }
    }
    for r in bp::regions(b, None)? {
        for s in b.region(r)?.get_shells(None, |_, _| Ok(true))? {
            if b.shell(s)?.shell_type == ShellType::Wire {
                match b.shell(s)?.wire_edge(b)? {
                    None => c.test(1, Object::Shell(s), true)?, // Original null-edge branch is tautological.
                    Some(e) => c.test(2, Object::Shell(s), !b.edge(e)?.edgeuses.is_empty())?,
                }
            }
        }
    }
    for f in b.face_list().to_vec() {
        check_face(k, b, f, &mut c)?;
    }
    for e in b.edge_list().to_vec() {
        let owner = Object::Edge(e);
        c.test(57, owner, true)?; // Same-context attachment is intrinsic to this arena.
        let primary = *b
            .edge(e)?
            .edgeuses
            .first()
            .ok_or(SolidError::null_pointer(OP))?;
        let orientation = b.edgeuse(primary)?.orientation;
        let first = edge::vertex(b, e)?;
        let other = edge::other_vertex(b, e, first)?;
        let start = edge::start_vertex(b, e)?;
        let good = (orientation == OrientType::Same && start == first)
            || (orientation == OrientType::Opposite && start == other);
        if !good {
            c.test(58, owner, false)?;
        }
        let etol = b.edge(e)?.tolerance;
        c.value(59, owner, etol < tol * 1000., tol * 1000., etol)?;
        c.value(60, owner, etol > tol * 0.99, tol * 0.99, etol)?;
        if c.selected(61) {
            let good = ignored(geometry::validate_edge(k, b, e, false))?.is_some();
            c.test(61, owner, good)?;
        } else {
            c.test(61, owner, true)?;
        }
        let uses = b.edge(e)?.edgeuses.clone();
        c.test(62, owner, uses.len() % 2 == 0)?;
        let last = *uses.last().ok_or(SolidError::null_pointer(OP))?;
        let mut last_orientation = b.edgeuse(last)?.orientation;
        for id in &uses {
            let u = b.edgeuse(*id)?;
            let who = Object::Edgeuse(*id);
            let orient = u.orientation;
            c.test(63, who, orient != last_orientation)?;
            last_orientation = orient;
            c.test(64, who, u.edge == Some(e))?;
            if let EdgeuseOwner::Shell(s) = u.owner {
                if let Some(s) = s {
                    c.test(
                        65,
                        Object::Shell(s),
                        b.shell(s)?.shell_type != ShellType::Vertex,
                    )?;
                } else {
                    return Err(SolidError::null_pointer(OP));
                }
            } else {
                let ccw = c.required(66, who, u.ccw)?;
                c.test(67, who, b.edgeuse(ccw)?.cw == Some(*id))?;
                let cw = c.required(68, who, b.edgeuse(*id)?.cw)?;
                c.test(69, who, b.edgeuse(cw)?.ccw == Some(*id))?;
                if let (Some(curve), Some(curve_owner)) = eu::uv_trim_curve(b, *id)? {
                    let f = eu::face(b, *id)?.ok_or(SolidError::null_pointer(OP))?;
                    let domain = b.face(f)?.uv_domain;
                    let ivl = b.edge(e)?.interval;
                    let who = Object::UvCurve(curve_owner, curve.class_string());
                    let mut previous = Point3::UNDEF;
                    for (test, param) in [
                        (70, ivl.min()),
                        (71, ivl.max()),
                        (72, (ivl.min() + ivl.max()) / 2.),
                    ] {
                        if let Some(p) = ignored(curve.evaluate_point(param))? {
                            previous = p;
                        }
                        let inside =
                            domain.contains_point(Point2::new(previous.x, previous.y), 0.0001);
                        c.value(test, who, inside, 0.0001, UNDEF_DOUBLE)?;
                    }
                }
            }
            let vu = c.required(73, who, b.edgeuse(*id)?.vertexuse)?;
            let vu_owner = b.vertexuse(vu)?.owner;
            c.test(
                74,
                Object::Vertexuse(vu),
                matches!(vu_owner, VertexuseOwner::Edgeuse(_)),
            )?;
            c.test(75, who, vu_owner == VertexuseOwner::Edgeuse(Some(*id)))?;
            let radial = eu::radial(b, *id)?;
            c.test(
                76,
                who,
                match radial {
                    Some(r) => eu::radial(b, r)? == Some(*id),
                    None => false,
                },
            )?;
            let mate = eu::mate(b, *id)?;
            c.test(
                77,
                who,
                match mate {
                    Some(m) => eu::mate(b, m)? == Some(*id),
                    None => false,
                },
            )?;
            if !edge::is_wire(b, e)? {
                let lu = eu::loopuse(b, *id)?.ok_or(SolidError::null_pointer(OP))?;
                let m = mate.ok_or(SolidError::null_pointer(OP))?;
                let ml = eu::loopuse(b, m)?.ok_or(SolidError::null_pointer(OP))?;
                c.test(
                    78,
                    Object::Loopuse(lu),
                    b.loopuse(lu)?.loop_ == b.loopuse(ml)?.loop_,
                )?;
                let fu = c.required(79, Object::Loopuse(lu), b.loopuse(lu)?.faceuse)?;
                let mf = c.required(80, Object::Loopuse(ml), b.loopuse(ml)?.faceuse)?;
                c.test(
                    81,
                    Object::Faceuse(fu),
                    b.faceuse(fu)?.face == b.faceuse(mf)?.face,
                )?;
            }
        }
    }
    for id in b.vertex_list().to_vec() {
        let v = b.vertex(id)?;
        let owner = Object::Vertex(id);
        c.value(
            82,
            owner,
            v.tolerance < tol * 1000.,
            tol * 1000.,
            v.tolerance,
        )?;
        c.value(83, owner, v.tolerance > tol * 0.99, tol * 0.99, v.tolerance)?;
        for vu in v.vertexuses.clone() {
            let u = b.vertexuse(vu)?;
            let who = Object::Vertexuse(vu);
            c.test(84, who, u.vertex == Some(id))?;
            match u.owner {
                VertexuseOwner::Shell(s) => {
                    let s = c.required(85, who, s)?;
                    c.test(
                        86,
                        Object::Shell(s),
                        b.shell(s)?.shell_type == ShellType::Vertex,
                    )?;
                    c.test(
                        87,
                        Object::Shell(s),
                        b.shell(s)?.members == ShellMembers::Vertexuse(vu),
                    )?;
                }
                VertexuseOwner::Loopuse(l) => {
                    let l = c.required(88, who, l)?;
                    c.test(
                        89,
                        Object::Loopuse(l),
                        matches!(b.loopuse(l)?.start, LoopuseStart::Vertexuse(_)),
                    )?;
                    c.test(
                        90,
                        Object::Loopuse(l),
                        b.loopuse(l)?.start == LoopuseStart::Vertexuse(Some(vu)),
                    )?;
                }
                VertexuseOwner::Edgeuse(e) => {
                    let e = c.required(91, who, e)?;
                    c.test(92, Object::Edgeuse(e), b.edgeuse(e)?.vertexuse == Some(vu))?;
                    c.test(93, Object::Edgeuse(e), b.edgeuse(e)?.edge.is_some())?;
                }
                VertexuseOwner::Unknown => {}
            }
        }
    }
    for r in bp::regions(b, None)? {
        for s in b.region(r)?.get_shells(None, |_, _| Ok(true))? {
            c.test(94, Object::Shell(s), b.shell(s)?.region == Some(r))?;
            if b.shell(s)?.shell_type == ShellType::Faceuse {
                for fu in b.shell(s)?.get_faceuses()? {
                    c.test(95, Object::Faceuse(fu), b.faceuse(fu)?.shell == Some(s))?;
                }
            }
        }
    }
    // Source test 96 is removed composite topology and remains unexecuted.
    for e in b.edge_list() {
        let uses = &b.edge(*e)?.edgeuses;
        for pair in (1..uses.len()).step_by(2) {
            let a = *uses.get(pair - 1).ok_or(SolidError::null_pointer(OP))?;
            let u = *uses.get(pair).ok_or(SolidError::null_pointer(OP))?;
            let next = *uses
                .get((pair + 1) % uses.len())
                .ok_or(SolidError::null_pointer(OP))?;
            c.test(97, Object::Edgeuse(u), eu::radial(b, u)? == Some(next))?;
            let s1 = eu::shell(b, u)?.ok_or(SolidError::null_pointer(OP))?;
            let s2 = eu::shell(b, next)?.ok_or(SolidError::null_pointer(OP))?;
            if b.shell(s1)?.region != b.shell(s2)?.region {
                c.test(98, Object::Edgeuse(u), false)?;
            }
            c.test(
                99,
                Object::Edgeuse(a),
                b.edgeuse(a)?.orientation != b.edgeuse(u)?.orientation,
            )?;
            let first = *uses.first().ok_or(SolidError::null_pointer(OP))?;
            let second = *uses.get(1).ok_or(SolidError::null_pointer(OP))?;
            c.test(
                100,
                Object::Edgeuse(a),
                b.edgeuse(a)?.orientation == b.edgeuse(first)?.orientation,
            )?;
            c.test(
                101,
                Object::Edgeuse(u),
                b.edgeuse(u)?.orientation == b.edgeuse(second)?.orientation,
            )?;
        }
    }
    for f in b.face_list() {
        let fu0 = b.face(*f)?.faceuse.ok_or(SolidError::null_pointer(OP))?;
        let fu1 = b.faceuse(fu0)?.mate.ok_or(SolidError::null_pointer(OP))?;
        for (fu, first_test, rest_test, empty_test) in [(fu0, 102, 103, 109), (fu1, 104, 105, 110)]
        {
            let loops = &b.faceuse(fu)?.loopuses;
            if let Some(first) = loops.first() {
                c.test(
                    first_test,
                    Object::Loopuse(*first),
                    b.loopuse(*first)?.orientation == OrientType::Same,
                )?;
            } else {
                c.test(empty_test, Object::Faceuse(fu), false)?;
                return Err(SolidError::Upstream {
                    operation: OP,
                    code: i32::from(c.ok),
                    entity: None,
                });
            }
            for l in loops.iter().skip(1) {
                c.test(
                    rest_test,
                    Object::Loopuse(*l),
                    b.loopuse(*l)?.orientation == OrientType::Opposite,
                )?;
            }
        }
        for l in face::loops(b, *f, false, None)? {
            let a = b.loop_(l)?.loopuse.ok_or(SolidError::null_pointer(OP))?;
            let m = b.loopuse(a)?.mate.ok_or(SolidError::null_pointer(OP))?;
            c.test(106, Object::Loopuse(a), b.loopuse(a)?.faceuse == Some(fu0))?;
            c.test(106, Object::Loopuse(m), b.loopuse(m)?.faceuse == Some(fu1))?;
        }
        for fu in [fu0, fu1] {
            let contains = match b.faceuse(fu)?.shell {
                Some(s) => b.shell(s)?.get_faceuses()?.contains(&fu),
                None => false,
            };
            c.test(
                if fu == fu0 { 107 } else { 108 },
                Object::Faceuse(fu),
                contains,
            )?;
        }
    }
    // The original ignores accumulated bRtn here and ALWAYS returns SM_SUCCESS.
    // A successful status alone is never a valid-model qualification.
    Ok(())
}

fn check_face(k: &Kernel, b: &mut Brep, f: FaceId, c: &mut Checks<'_, '_, '_>) -> Result<()> {
    let tol = b.face(f)?.tolerance;
    // In the supported Release profile both BOOL and VALUE macros ignore Given
    // request lists. Reuse the owning algorithm and its exact report side effects.
    let outcome = crate::topology::trimming_tools::check_face(
        k,
        b,
        f,
        tol,
        c.report.reports.as_deref_mut(),
        c.options.walk,
    );
    c.ok &= ignored(outcome)?.is_some();
    let who = Object::Face(f);
    c.test(4, who, true)?;
    c.test(5, who, tol < b.tolerance() * 1000.)?;
    c.test(6, who, tol > b.tolerance() * 0.99)?;
    let surface = b
        .face(f)?
        .surface
        .as_ref()
        .ok_or(SolidError::null_pointer(OP))?;
    c.test(7, who, true)?;
    let swho = Object::Surface(f, surface.class_string());
    c.test(8, swho, true)?;
    c.test(9, swho, surface.owner().is_some())?;
    // Surface validity predicate 10 is commented out in the native source.
    if !b
        .face(f)?
        .uv_domain
        .is_contained_by(&surface.natural_uv_domain(), EFF_ZERO)
    {
        c.test(11, swho, false)?;
    }
    let fu = c.required(12, who, b.face(f)?.faceuse)?;
    c.test(13, Object::Faceuse(fu), true)?;
    c.test(
        14,
        Object::Faceuse(fu),
        b.faceuse(fu)?.orientation == OrientType::Same,
    )?;
    let mate = c.required(15, Object::Faceuse(fu), b.faceuse(fu)?.mate)?;
    c.test(16, Object::Faceuse(mate), true)?;
    c.test(
        17,
        Object::Faceuse(fu),
        b.faceuse(fu)?.orientation != b.faceuse(mate)?.orientation,
    )?;
    c.test(18, who, true)?;
    c.test(19, Object::Faceuse(fu), b.faceuse(fu)?.face == Some(f))?;
    c.test(20, Object::Faceuse(mate), b.faceuse(mate)?.face == Some(f))?;
    c.test(21, Object::Faceuse(fu), b.faceuse(fu)?.mate == Some(mate))?;
    c.test(22, Object::Faceuse(mate), b.faceuse(mate)?.mate == Some(fu))?;
    let s1 = c.required(23, Object::Faceuse(fu), b.faceuse(fu)?.shell)?;
    let s2 = c.required(24, Object::Faceuse(mate), b.faceuse(mate)?.shell)?;
    for (s, ctx) in [(s1, 25), (s2, 26)] {
        c.test(ctx, Object::Shell(s), true)?;
    }
    c.test(
        27,
        Object::Shell(s1),
        b.shell(s1)?.shell_type == ShellType::Faceuse,
    )?;
    c.test(
        28,
        Object::Shell(s2),
        b.shell(s2)?.shell_type == ShellType::Faceuse,
    )?;
    for (i, l) in face::loops(b, f, false, None)?.into_iter().enumerate() {
        // SmLoop::GetFace follows the primary LU/FU chain; Loop.face is only a
        // convenience field and must not become a parallel source of truth.
        let primary = b.loop_(l)?.loopuse.ok_or(SolidError::null_pointer(OP))?;
        let owning_fu = b
            .loopuse(primary)?
            .faceuse
            .ok_or(SolidError::null_pointer(OP))?;
        c.test(29, Object::Loop(l), b.faceuse(owning_fu)?.face == Some(f))?;
        let a = c.required(30, Object::Loop(l), b.loop_(l)?.loopuse)?;
        let m = c.required(31, Object::Loop(l), b.loopuse(a)?.mate)?;
        c.test(32, Object::Loop(l), true)?;
        c.test(33, Object::Loopuse(a), true)?;
        c.test(34, Object::Loopuse(m), true)?;
        if i == 0 {
            c.test(
                35,
                Object::Loopuse(a),
                b.loopuse(a)?.orientation == OrientType::Same,
            )?;
            c.test(
                36,
                Object::Loopuse(m),
                b.loopuse(m)?.orientation == OrientType::Same,
            )?;
            c.test(
                37,
                Object::Loopuse(a),
                !matches!(b.loopuse(a)?.start, LoopuseStart::Vertexuse(_)),
            )?;
            c.test(
                38,
                Object::Loopuse(m),
                !matches!(b.loopuse(m)?.start, LoopuseStart::Vertexuse(_)),
            )?;
        } else {
            c.test(
                39,
                Object::Loopuse(a),
                b.loopuse(a)?.orientation == OrientType::Opposite,
            )?;
            c.test(
                40,
                Object::Loopuse(m),
                b.loopuse(m)?.orientation == OrientType::Opposite,
            )?;
        }
        c.test(41, Object::Loopuse(a), b.loopuse(a)?.loop_ == Some(l))?;
        c.test(42, Object::Loopuse(m), b.loopuse(m)?.loop_ == Some(l))?;
        c.test(43, Object::Loopuse(a), b.loopuse(a)?.mate == Some(m))?;
        c.test(44, Object::Loopuse(m), b.loopuse(m)?.mate == Some(a))?;
        if matches!(b.loopuse(a)?.start, LoopuseStart::Vertexuse(_)) {
            continue;
        }
        walk_loop(k, b, a, [45, 46, 47, 48, 49, 50], c)?;
        if matches!(b.loopuse(m)?.start, LoopuseStart::Vertexuse(_)) {
            continue;
        }
        walk_loop(k, b, m, [51, 52, 53, 54, 55, 56], c)?;
        let uses_a = loopuse::edgeuses(b, a, None, None)?;
        let uses_b = loopuse::edgeuses(b, m, None, None)?;
        let mut mated = uses_a.len() == uses_b.len();
        if mated {
            for u in uses_a {
                let valid = match eu::mate(b, u)? {
                    Some(u) => eu::loopuse(b, u)? == Some(m),
                    None => false,
                };
                if !valid {
                    mated = false;
                    break;
                }
            }
        }
        c.test(111, Object::Loopuse(a), mated)?;
    }
    Ok(())
}
fn walk_loop(
    k: &Kernel,
    b: &mut Brep,
    lu: LoopuseId,
    tests: [u64; 6],
    c: &mut Checks<'_, '_, '_>,
) -> Result<()> {
    let [start_test, ctx, geo, ty, back, reciprocal] = tests;
    let first = match b.loopuse(lu)?.start {
        LoopuseStart::Edgeuse(id) => id,
        _ => None,
    };
    let start = c.required(start_test, Object::Loopuse(lu), first)?;
    let mut current = start;
    let mut count = 0;
    loop {
        c.test(ctx, Object::Edgeuse(current), true)?;
        if c.selected(geo) {
            let good = ignored(geometry::check_edgeuse(k, b, current))?.is_some();
            c.test(geo, Object::Edgeuse(current), good)?;
        } else {
            c.test(geo, Object::Edgeuse(current), true)?;
        }
        c.test(
            ty,
            Object::Edgeuse(current),
            matches!(b.edgeuse(current)?.owner, EdgeuseOwner::Loopuse(_)),
        )?;
        c.test(
            back,
            Object::Edgeuse(current),
            b.edgeuse(current)?.owner == EdgeuseOwner::Loopuse(Some(lu)),
        )?;
        let previous = current;
        current = b
            .edgeuse(current)?
            .ccw
            .ok_or(SolidError::null_pointer(OP))?;
        c.test(
            reciprocal,
            Object::Edgeuse(current),
            b.edgeuse(current)?.cw == Some(previous),
        )?;
        count += 1;
        if current == start {
            break;
        }
        if count >= MAX_LOOP_WALK {
            return Err(SolidError::sm_err(OP));
        }
    }
    Ok(())
}
