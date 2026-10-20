// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Required-pointer checks 112--146 of `SmBrep::ValidatePointers` (`d8ed45a`).
//! The original source stops at the first failure before unsafe deeper getters.
//! Context/list-sentinel pointers are intrinsic to Rust's owned arena model.
use super::{
    bp, Brep, EdgeuseId, FaceuseId, LoopuseId, Object, Reporter, Result, VertexuseId, MAX_LOOP_WALK,
};
use crate::topology::edgeuse::p1 as eu;
use crate::topology::shell::{ShellMembers, ShellType};
use crate::topology::{EdgeuseOwner, LoopuseStart, VertexuseOwner};

pub(super) fn check(b: &Brep, report: &mut Reporter<'_>) -> Result<()> {
    // Kernel references and arena list containers cannot be NULL in this model.
    report.required(112, Object::Brep, true)?;
    report.required(113, Object::Brep, b.infinite_region().is_some())?;
    for test in [114, 115, 116] {
        report.required(test, Object::Brep, true)?;
    }
    if let Some(region) = b.infinite_region() {
        report.required(117, Object::Region(region), b.region(region)?.in_brep)?;
    }
    for region in bp::regions(b, None)? {
        if Some(region) != b.infinite_region() {
            report.required(117, Object::Region(region), b.region(region)?.in_brep)?;
        }
        for shell in b.region(region)?.get_shells(None, |_, _| Ok(true))? {
            let s = b.shell(shell)?;
            report.required(118, Object::Shell(shell), s.region.is_some())?;
            if s.shell_type != ShellType::Unknown {
                let has_head = match &s.members {
                    ShellMembers::Empty => false,
                    ShellMembers::Faceuses(ids) => !ids.is_empty(),
                    ShellMembers::Edgeuse(_) | ShellMembers::Vertexuse(_) => true,
                };
                report.required(119, Object::Shell(shell), has_head)?;
            }
            match &s.members {
                ShellMembers::Faceuses(fus) if s.shell_type == ShellType::Faceuse => {
                    for fu in fus {
                        faceuse(b, report, *fu)?;
                    }
                }
                ShellMembers::Edgeuse(first) if s.shell_type == ShellType::Wire => {
                    edgeuse(b, report, *first)?;
                    let edge = b.edgeuse(*first)?.edge;
                    if let Some(edge) = edge {
                        let uses = &b.edge(edge)?.edgeuses;
                        if let Some(start) = uses.iter().position(|u| u == first) {
                            for id in uses
                                .iter()
                                .cycle()
                                .skip(start + 1)
                                .take(uses.len().saturating_sub(1))
                            {
                                edgeuse(b, report, *id)?;
                            }
                        }
                    }
                }
                ShellMembers::Vertexuse(vu) if s.shell_type == ShellType::Vertex => {
                    vertexuse(b, report, *vu)?;
                }
                _ => {}
            }
        }
    }
    for face in b.face_list() {
        let f = b.face(*face)?;
        report.required(120, Object::Face(*face), true)?;
        report.required(121, Object::Face(*face), f.surface.is_some())?;
        report.required(122, Object::Face(*face), f.faceuse.is_some())?;
        if let Some(fu) = f.faceuse {
            faceuse(b, report, fu)?;
            let mate = b.faceuse(fu)?.mate;
            if let Some(mate) = mate {
                faceuse(b, report, mate)?;
            }
            loopuses(b, report, fu)?;
            if let Some(mate) = mate {
                loopuses(b, report, mate)?;
            }
        }
    }
    for edge in b.edge_list() {
        let e = b.edge(*edge)?;
        report.required(132, Object::Edge(*edge), true)?;
        report.required(133, Object::Edge(*edge), e.curve.is_some())?;
        report.required(134, Object::Edge(*edge), !e.edgeuses.is_empty())?;
        for eu in &e.edgeuses {
            edgeuse(b, report, *eu)?;
        }
    }
    for vertex in b.vertex_list() {
        let v = b.vertex(*vertex)?;
        report.required(142, Object::Vertex(*vertex), true)?;
        report.required(143, Object::Vertex(*vertex), !v.vertexuses.is_empty())?;
        for vu in &v.vertexuses {
            vertexuse(b, report, *vu)?;
        }
    }
    Ok(())
}

fn faceuse(b: &Brep, report: &mut Reporter<'_>, id: FaceuseId) -> Result<()> {
    let fu = b.faceuse(id)?;
    let owner = Object::Faceuse(id);
    report.required(123, owner, fu.face.is_some())?;
    report.required(124, owner, fu.mate.is_some())?;
    report.required(125, owner, fu.shell.is_some())?;
    report.required(126, owner, !fu.loopuses.is_empty())
}
fn edgeuse(b: &Brep, report: &mut Reporter<'_>, id: EdgeuseId) -> Result<()> {
    let u = b.edgeuse(id)?;
    let owner = Object::Edgeuse(id);
    report.required(135, owner, u.edge.is_some())?;
    report.required(136, owner, u.vertexuse.is_some())?;
    let connected = match u.owner {
        EdgeuseOwner::Loopuse(id) => id.is_some(),
        EdgeuseOwner::Shell(id) => id.is_some(),
        EdgeuseOwner::Unknown => false,
    };
    report.required(137, owner, connected)?;
    if matches!(u.owner, EdgeuseOwner::Loopuse(_)) {
        report.required(138, owner, u.ccw.is_some())?;
        report.required(139, owner, u.cw.is_some())?;
    }
    // Literal NULL radial-sibling pointers have no separate representation in
    // this arena model. Ask the owning getters: absent links retain these labels,
    // while an inconsistent membership is an access error, not a fabricated NULL.
    if let Some(edge) = u.edge {
        if !b.edge(edge)?.edgeuses.contains(&id) {
            return Err(crate::error::SolidError::invalid_input(
                super::OP,
                "edge use is absent from its radial owner list",
            ));
        }
    }
    report.required(140, owner, eu::edgeuse_next(b, id)?.is_some())?;
    report.required(141, owner, eu::edgeuse_last(b, id)?.is_some())?;
    Ok(())
}
fn vertexuse(b: &Brep, report: &mut Reporter<'_>, id: VertexuseId) -> Result<()> {
    let vu = b.vertexuse(id)?;
    let owner = Object::Vertexuse(id);
    report.required(144, owner, vu.vertex.is_some())?;
    let connected = match vu.owner {
        VertexuseOwner::Edgeuse(id) => id.is_some(),
        VertexuseOwner::Loopuse(id) => id.is_some(),
        VertexuseOwner::Shell(id) => id.is_some(),
        VertexuseOwner::Unknown => false,
    };
    report.required(145, owner, connected)
}
fn loopuses(b: &Brep, report: &mut Reporter<'_>, fu: FaceuseId) -> Result<()> {
    for id in &b.faceuse(fu)?.loopuses {
        loopuse(b, report, *id)?;
        if let Some(loop_id) = b.loopuse(*id)?.loop_ {
            report.required(
                127,
                Object::Loop(loop_id),
                b.loop_(loop_id)?.loopuse.is_some(),
            )?;
        }
    }
    Ok(())
}
fn loopuse(b: &Brep, report: &mut Reporter<'_>, id: LoopuseId) -> Result<()> {
    let lu = b.loopuse(id)?;
    let owner = Object::Loopuse(id);
    report.required(128, owner, lu.loop_.is_some())?;
    report.required(129, owner, lu.mate.is_some())?;
    report.required(130, owner, lu.faceuse.is_some())?;
    let present = match lu.start {
        LoopuseStart::Unknown => false,
        LoopuseStart::Edgeuse(id) => id.is_some(),
        LoopuseStart::Vertexuse(id) => id.is_some(),
    };
    report.required(131, owner, present)?;
    match lu.start {
        LoopuseStart::Edgeuse(Some(start)) => {
            let mut cur = start;
            let mut count = 0;
            loop {
                edgeuse(b, report, cur)?;
                cur = b
                    .edgeuse(cur)?
                    .ccw
                    .ok_or(crate::error::SolidError::sm_err(super::OP))?;
                count += 1;
                if cur == start || count >= MAX_LOOP_WALK {
                    break;
                }
            }
            report.required(146, owner, cur == start)?;
        }
        LoopuseStart::Vertexuse(Some(vu)) => vertexuse(b, report, vu)?,
        _ => {}
    }
    Ok(())
}
