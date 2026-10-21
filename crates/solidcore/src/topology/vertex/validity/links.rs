//! Literal vertex-use incidence checks, without inventing extra model invariants.
//!
//! Upstream: maintained `d8ed45a`, `SmVertex.cpp`1446--1707.
//! | Rust | Upstream |
//! |------|----------|
//! | [`check_pointers`] | `SmVertex::CheckPointers` |
//! Deviations: checked arena access replaces invalid pointers; no recursion.

use crate::error::Result;
use crate::topology::{
    edgeuse::p1 as eu, face::p2 as face, shell::ShellMembers, vertexuse as vu, Brep, EdgeuseId,
    EdgeuseOwner, LoopuseStart, VertexId, VertexuseId, VertexuseOwner,
};

fn vertex_loop(b: &Brep, id: VertexId, use_: VertexuseId) -> Result<bool> {
    let Some(a) = vu::loopuse(b, use_)? else {
        return Ok(false);
    };
    let Some(c) = b.loopuse(a)?.mate else {
        return Ok(false);
    };
    let LoopuseStart::Vertexuse(Some(other)) = b.loopuse(c)?.start else {
        return Ok(false);
    };
    let (Some(v1), Some(v2), Some(l1), Some(l2), Some(f1), Some(f2)) = (
        b.vertexuse(use_)?.vertex,
        b.vertexuse(other)?.vertex,
        b.loopuse(a)?.loop_,
        b.loopuse(c)?.loop_,
        b.loopuse(a)?.faceuse,
        b.loopuse(c)?.faceuse,
    ) else {
        return Ok(false);
    };
    let (Some(face1), Some(face2)) = (b.faceuse(f1)?.face, b.faceuse(f2)?.face) else {
        return Ok(false);
    };
    let mut valid = v1 == id && v2 == id;
    valid &= l1 == l2;
    valid &= face1 == face2;
    valid &= b.loopuse(a)?.mate == Some(c) && b.loopuse(c)?.mate == Some(a);
    valid &= b.faceuse(f1)?.mate == Some(f2) && b.faceuse(f2)?.mate == Some(f1);
    let upward = face::upward_faceuse(b, face1)?;
    valid &= upward == Some(f1) || upward == Some(f2);
    let primary = b.loop_(l1)?.loopuse;
    valid &= primary == Some(a) || primary == Some(c);
    valid &= b.faceuse(f1)?.loopuses.contains(&a);
    valid &= b.faceuse(f2)?.loopuses.contains(&c);
    valid &= b.loopuse(a)?.start == LoopuseStart::Vertexuse(Some(use_));
    valid &= b.loopuse(c)?.start == LoopuseStart::Vertexuse(Some(other));
    valid &= b.vertexuse(use_)?.vertex == Some(id) && b.vertexuse(other)?.vertex == Some(id);
    valid &= matches!(b.loopuse(a)?.start, LoopuseStart::Vertexuse(_))
        && matches!(b.loopuse(c)?.start, LoopuseStart::Vertexuse(_));
    Ok(valid)
}

fn loop_edge(
    b: &Brep,
    id: VertexId,
    use_: VertexuseId,
    a: EdgeuseId,
    mate: Option<EdgeuseId>,
) -> Result<bool> {
    let corner = eu::corner_mate_edgeuse(b, a, Some(id))?;
    let corner_mate = corner.map(|c| eu::mate(b, c)).transpose()?.flatten();
    let uses = [Some(a), mate, corner, corner_mate];
    let mut lus = [None; 4];
    let mut loops = [None; 4];
    let mut fus = [None; 4];
    let mut faces = [None; 4];
    let mut vus = [Some(use_), None, None, None];
    let mut vertices = [None; 4];
    // Native obtains these optional chains before the combined null guard.
    // Keep the accesses in source order, even when a previous chain was absent.
    for (out, e) in lus.iter_mut().zip(uses) {
        *out = e.map(|e| eu::loopuse(b, e)).transpose()?.flatten();
    }
    for (out, l) in loops.iter_mut().zip(lus) {
        *out = l
            .map(|l| b.loopuse(l).map(|l| l.loop_))
            .transpose()?
            .flatten();
    }
    for (out, e) in fus.iter_mut().zip(uses) {
        *out = e.map(|e| eu::faceuse(b, e)).transpose()?.flatten();
    }
    for (out, f) in faces.iter_mut().zip(fus) {
        *out = f
            .map(|f| b.faceuse(f).map(|f| f.face))
            .transpose()?
            .flatten();
    }
    for (out, e) in vus.iter_mut().skip(1).zip(uses.into_iter().skip(1)) {
        *out = e
            .map(|e| b.edgeuse(e).map(|e| e.vertexuse))
            .transpose()?
            .flatten();
    }
    for (out, v) in vertices.iter_mut().zip(vus) {
        *out = v
            .map(|v| b.vertexuse(v).map(|v| v.vertex))
            .transpose()?
            .flatten();
    }
    let (
        [Some(e1), Some(e2), Some(e3), Some(e4)],
        [Some(lu1), Some(lu2), Some(lu3), Some(lu4)],
        [Some(l1), Some(l2), Some(l3), Some(l4)],
        [Some(fu1), Some(fu2), Some(_), Some(_)],
        [Some(f1), Some(f2), Some(f3), Some(f4)],
        [Some(vu1), Some(vu2), Some(vu3), Some(vu4)],
        [Some(v1), Some(_), Some(_), Some(v4)],
    ) = (uses, lus, loops, fus, faces, vus, vertices)
    else {
        return Ok(false);
    };
    let mut valid = v1 == id && v4 == id;
    valid &= l1 == l2 && l1 == l3 && l1 == l4;
    valid &= f1 == f2 && f1 == f3 && f1 == f4;
    valid &= eu::mate(b, e1)? == Some(e2)
        && eu::mate(b, e2)? == Some(e1)
        && eu::mate(b, e3)? == Some(e4)
        && eu::mate(b, e4)? == Some(e3);
    valid &= b.loopuse(lu1)?.mate == Some(lu2) && b.loopuse(lu2)?.mate == Some(lu1);
    valid &= b.faceuse(fu1)?.mate == Some(fu2) && b.faceuse(fu2)?.mate == Some(fu1);
    let upward = face::upward_faceuse(b, f1)?;
    valid &= upward == Some(fu1) || upward == Some(fu2);
    let primary = b.loop_(l1)?.loopuse;
    valid &= primary == Some(lu1) || primary == Some(lu2);
    valid &= b.faceuse(fu1)?.loopuses.contains(&lu1);
    valid &= b.faceuse(fu2)?.loopuses.contains(&lu2);
    valid &= b.edgeuse(e1)?.vertexuse == Some(vu1)
        && b.edgeuse(e2)?.vertexuse == Some(vu2)
        && b.edgeuse(e3)?.vertexuse == Some(vu3)
        && b.edgeuse(e4)?.vertexuse == Some(vu4);
    for v in [vu1, vu2, vu3, vu4] {
        valid &= matches!(b.vertexuse(v)?.owner, VertexuseOwner::Edgeuse(_));
    }
    for e in [e1, e2, e3, e4] {
        valid &= matches!(b.edgeuse(e)?.owner, EdgeuseOwner::Loopuse(_));
    }
    for l in [lu1, lu2, lu3, lu4] {
        valid &= matches!(b.loopuse(l)?.start, LoopuseStart::Edgeuse(_));
    }
    Ok(valid)
}

/// Native vertex incidence checks for shell vertices, vertex loops, wires and
/// face corners. An empty use list retains the source routine's true result;
/// its owning base chain is a separate check. No reciprocal links are repaired.
pub fn check_pointers(b: &Brep, id: VertexId) -> Result<bool> {
    let uses = b.vertex(id)?.vertexuses.clone();
    for use_ in &uses {
        let valid = match b.vertexuse(*use_)?.owner {
            VertexuseOwner::Shell(_) => {
                if let Some(shell) = vu::shell(b, *use_)? {
                    let members = &b.shell(shell)?.members;
                    let count = match members {
                        ShellMembers::Empty => 0,
                        ShellMembers::Faceuses(v) => v.len(),
                        _ => 1,
                    };
                    uses.len() == 1 && *members == ShellMembers::Vertexuse(*use_) && count == 1
                } else {
                    false
                }
            }
            VertexuseOwner::Loopuse(_) => vertex_loop(b, id, *use_)?,
            VertexuseOwner::Edgeuse(_) => {
                let edgeuse = vu::edgeuse(b, *use_)?;
                let mate = edgeuse.map(|e| eu::mate(b, e)).transpose()?.flatten();
                if let Some(e) = edgeuse {
                    match b.edgeuse(e)?.owner {
                        EdgeuseOwner::Shell(_) => {
                            let shell = eu::shell(b, e)?;
                            if let (Some(shell), Some(mate)) = (shell, mate) {
                                let mut ok = eu::shell(b, e)? == Some(shell)
                                    && eu::shell(b, mate)? == Some(shell);
                                ok &=
                                    eu::mate(b, e)? == Some(mate) && eu::mate(b, mate)? == Some(e);
                                ok &= b.edgeuse(e)?.vertexuse == Some(*use_);
                                ok &= matches!(b.edgeuse(e)?.owner, EdgeuseOwner::Shell(_))
                                    && matches!(b.edgeuse(mate)?.owner, EdgeuseOwner::Shell(_));
                                ok
                            } else {
                                false
                            }
                        }
                        EdgeuseOwner::Loopuse(_) => loop_edge(b, id, *use_, e, mate)?,
                        EdgeuseOwner::Unknown => false,
                    }
                } else {
                    false
                }
            }
            VertexuseOwner::Unknown => false,
        };
        if !valid {
            return Ok(false);
        }
    }
    Ok(true)
}
