// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Local faceuse, loopuse and vertexuse assertions over their actual typed graph.
//!
//! Upstream: `SmFaceuse.cpp::AssertValid` (430--456),
//! `SmLoopuse.cpp::AssertValid` (1469--1520), `SmVertexuse.cpp` (3110--3216),
//! maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_faceuse`] | `SmFaceuse::AssertValid` |
//! | [`assert_loopuse`] | `SmLoopuse::AssertValid` |
//! | [`assert_vertexuse`] | `SmVertexuse::AssertValid` |
//! Deviations: shared context/type invariants are intrinsic; checked ids replace
//! unsafe dereferences. These native routines ignore their walk argument. The
//! existing closure algorithm, including its native over-tolerance bug, is reused.

use super::{local_base, AssertValidOptions, Brep, CheckLevel, Object, Reporter};
use crate::error::{Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray, ReportSpec};
use crate::kernel::Kernel;
use crate::topology::{
    loopuse, vertexuse, FaceuseId, LoopuseId, LoopuseStart, VertexuseId, VertexuseOwner,
};

/// Faceuse base/list checks followed by the native optional face/mate context
/// checks. `walk` is ignored by the original routine, even when true.
pub fn assert_faceuse(
    k: &mut Kernel,
    b: &mut Brep,
    id: FaceuseId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.faceuse(id)?;
    let mut report = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::owning(k, b, Object::Faceuse(id), &mut report)?
    };
    if let Some(face) = b.faceuse(id)?.face {
        b.face(face)?;
        valid &= report.class_predicate(16022, 0, Object::Faceuse(id), true)?;
    }
    if let Some(mate) = b.faceuse(id)?.mate {
        b.faceuse(mate)?;
        valid &= report.class_predicate(16022, 1, Object::Faceuse(id), true)?;
    }
    Ok(valid)
}

/// Loopuse base and optional-context checks, required content under a faceuse,
/// then actual `IsClosed3d(TRUE)` only on the logical loop's primary use.
/// This is structural/native evidence, not a continuous closure guarantee.
pub fn assert_loopuse(
    k: &mut Kernel,
    b: &mut Brep,
    id: LoopuseId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.loopuse(id)?;
    let mut report = Reporter { reports };
    let object = Object::Loopuse(id);
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::topology_base(k, b, object, &mut report)?
    };
    let lu = *b.loopuse(id)?;
    if let Some(loop_) = lu.loop_ {
        b.loop_(loop_)?;
        valid &= report.class_predicate(labels::LOOPUSE_TYPE, 0, object, true)?;
    }
    if let Some(mate) = lu.mate {
        b.loopuse(mate)?;
        valid &= report.class_predicate(labels::LOOPUSE_TYPE, 1, object, true)?;
    }
    let has_content = match lu.start {
        LoopuseStart::Unknown | LoopuseStart::Edgeuse(None) | LoopuseStart::Vertexuse(None) => {
            false
        }
        LoopuseStart::Edgeuse(Some(eu)) => {
            b.edgeuse(eu)?;
            true
        }
        LoopuseStart::Vertexuse(Some(vu)) => {
            b.vertexuse(vu)?;
            true
        }
    };
    if has_content {
        valid &= report.class_predicate(labels::LOOPUSE_TYPE, 2, object, true)?;
    }
    valid &= report.class_predicate(
        labels::LOOPUSE_TYPE,
        3,
        object,
        lu.faceuse.is_none() || has_content,
    )?;
    if let Some(loop_) = lu.loop_ {
        if b.loop_(loop_)?.loopuse == Some(id) {
            let closed = loopuse::is_closed_3d(k, b, id, true)?;
            let message = if closed.closed {
                None
            } else {
                let eu = closed
                    .edgeuse
                    .ok_or_else(|| SolidError::null_pointer("loopuse closure witness"))?;
                let edge = b
                    .edgeuse(eu)?
                    .edge
                    .ok_or_else(|| SolidError::null_pointer("loopuse closure edge"))?;
                // Native prints a process pointer; deterministic Rust reports
                // retain the same witness through its typed model-local id.
                Some(format!("SmLoopuse::AssertValid - found loop with out of tol of Gap/XSectTol3d:[{:16.16}/{:16.16}] starting on Edge:[{}] ",closed.max_gap,closed.xsect_tol_3d,edge.index()).into())
            };
            valid &= report.class_record(
                object,
                closed.closed,
                ReportSpec {
                    test_class: labels::LOOPUSE_TYPE,
                    test_index: 5,
                    tol3d: closed.xsect_tol_3d,
                    value: closed.max_gap,
                    message,
                    ..ReportSpec::default()
                },
            )?;
        }
    }
    Ok(valid)
}

/// Actual old-tolerance Vertexuse local checks. Native disables cached fresh-gap
/// tests and the broad sector-gap rule in this SDK profile; they stay disabled.
/// The original routine ignores its walk argument.
pub fn assert_vertexuse(
    k: &mut Kernel,
    b: &mut Brep,
    id: VertexuseId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    let record = *b.vertexuse(id)?;
    let mut report = Reporter { reports };
    let who = Object::Vertexuse(id);
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::topology_base(k, b, who, &mut report)?
    };
    let present = match record.owner {
        VertexuseOwner::Shell(Some(s)) => {
            b.shell(s)?;
            true
        }
        VertexuseOwner::Loopuse(Some(lu)) => {
            b.loopuse(lu)?;
            true
        }
        VertexuseOwner::Edgeuse(Some(eu)) => {
            b.edgeuse(eu)?;
            true
        }
        _ => false,
    };
    if present {
        valid &= report.class_predicate(labels::VERTEXUSE_TYPE, 0, who, true)?;
        // Discriminator and target class cannot disagree in this typed sum.
        valid &= report.class_predicate(labels::VERTEXUSE_TYPE, 1, who, true)?;
    }
    let missing = vertexuse::is_sector_missing_seam(k, b, id, true)?.missing;
    valid &= report.class_predicate(labels::VERTEXUSE_TYPE, 5, who, !missing)?;
    if matches!(
        record.owner,
        VertexuseOwner::Loopuse(_) | VertexuseOwner::Edgeuse(_)
    ) {
        let edgeuse = vertexuse::edgeuse(b, id)?;
        let edge = edgeuse
            .map(|eu| b.edgeuse(eu))
            .transpose()?
            .and_then(|eu| eu.edge);
        // Native evaluates GetFaceuse first and never queries IsWire when a
        // face is present. Retain that short circuit, including failure paths.
        let attached = if vertexuse::faceuse(b, id)?.is_some() {
            true
        } else {
            edge.map(|edge| crate::topology::edge::is_wire(b, edge))
                .transpose()?
                .unwrap_or(false)
        };
        valid &= report.class_predicate(labels::VERTEXUSE_TYPE, 6, who, attached)?;
    }
    Ok(valid)
}
