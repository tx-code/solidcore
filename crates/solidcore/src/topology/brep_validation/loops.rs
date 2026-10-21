// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Logical-loop assertions with native side, UV closure and paired-use order.
//!
//! Upstream: `SmLoop.cpp::AssertValid` (1550--1699), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_loop`] | `SmLoop::AssertValid` |
//! Deviations: checked ids/indices replace undefined pointer/array access. Exact
//! evaluation copies preserve defining knots; native normalization/UV generation
//! remains observable. Non-root descendant traversal is an explicit dependency.

use super::{local, local_base, uses, AssertValidOptions, Brep, CheckLevel, Object, Reporter};
use crate::error::{Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray};
use crate::kernel::Kernel;
use crate::topology::{edgeuse::p1 as eu, edgeuse::side, loopuse, LoopId, LoopuseStart};

fn missing() -> SolidError {
    SolidError::null_pointer("topology::brep_validation::assert_loop")
}

/// Execute the actual loop-local sequence. `CheckLoop`'s ordinary status does not
/// change the source boolean; its reports still belong to the caller's array.
#[allow(clippy::too_many_lines)]
pub fn assert_loop(
    k: &mut Kernel,
    b: &mut Brep,
    id: LoopId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.loop_(id)?;
    let object = Object::Loop(id);
    let mut report = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::topology_base(k, b, object, &mut report)?
    };
    if let Some(lu) = b.loop_(id)?.loopuse {
        b.loopuse(lu)?;
        valid &= report.class_predicate(labels::LOOP_TYPE, 0, object, true)?;
        valid &= report.class_predicate(
            labels::LOOP_TYPE,
            2,
            object,
            side::loopuse_side(k, b, lu, None, None)?.inside,
        )?;
    }
    // The logical loop's stored convenience face is not its defining incidence.
    if let Some(face) = loopuse::loop_face(b, id)? {
        if let Some(surface) = b.face(face)?.surface.as_ref() {
            let surface = surface.clone_for_evaluation()?;
            let tol = b.face(face)?.tolerance;
            local::ignore_native_status(crate::topology::trimming_tools::check_loop(
                k,
                b,
                &surface,
                id,
                tol,
                report.reports.as_deref_mut(),
            ))?;
        }
    }
    let first = b.loop_(id)?.loopuse;
    let second = first
        .map(|lu| b.loopuse(lu))
        .transpose()?
        .and_then(|lu| lu.mate);
    if let (Some(first), Some(second)) = (first, second) {
        let first_uses = eu::loopuse_edgeuses(b, first)?;
        let second_uses = eu::loopuse_edgeuses(b, second)?;
        valid &= report.class_predicate(
            labels::LOOP_TYPE,
            5,
            object,
            first_uses.len() == second_uses.len(),
        )?;
        let first_type = b.loopuse(first)?.start;
        valid &= report.class_predicate(
            labels::LOOP_TYPE,
            3,
            object,
            (matches!(first_type, LoopuseStart::Edgeuse(_)) && !first_uses.is_empty())
                || (matches!(first_type, LoopuseStart::Vertexuse(_)) && first_uses.is_empty()),
        )?;
        if !first_uses.is_empty() {
            let first_edges = first_uses
                .iter()
                .map(|eu| b.edgeuse(*eu)?.edge.ok_or_else(missing))
                .collect::<Result<Vec<_>>>()?;
            let second_edges = second_uses
                .iter()
                .map(|eu| b.edgeuse(*eu)?.edge.ok_or_else(missing))
                .collect::<Result<Vec<_>>>()?;
            let unique = first_edges
                .iter()
                .position(|edge| first_edges.iter().filter(|other| *other == edge).count() == 1);
            // Source idx1 remains n when no unique edge exists. Its unused
            // offset still follows that convention instead of resetting idx1.
            let idx1 = unique.unwrap_or(first_edges.len());
            let chosen = first_edges.get(unique.unwrap_or(0)).ok_or_else(missing)?;
            let idx2 = second_edges
                .iter()
                .position(|edge| edge == chosen)
                .unwrap_or(second_edges.len());
            valid &= report.class_predicate(
                labels::LOOP_TYPE,
                4,
                object,
                first_edges.iter().all(|edge| second_edges.contains(edge)),
            )?;
            if first_edges.len() > 1 {
                let offset = (idx1 + idx2) % first_edges.len();
                for i in 1..first_edges.len() {
                    let j = (first_edges.len() - i + offset) % first_edges.len();
                    if unique.is_some() {
                        let lhs = first_edges.get(i).ok_or_else(missing)?;
                        let rhs = second_edges.get(j).ok_or_else(|| {
                            SolidError::invalid_input(
                                "loop paired-use order",
                                "native mate array index outside its reported count",
                            )
                        })?;
                        valid &=
                            report.class_predicate(labels::LOOP_TYPE, 6, object, lhs == rhs)?;
                    }
                }
            }
        }
    }
    let child_options = AssertValidOptions {
        level: if matches!(options.level, CheckLevel::Given(_)) {
            CheckLevel::Given(&[])
        } else {
            options.level
        },
        walk: false,
    };
    if let Some(lu) = first {
        valid &= uses::assert_loopuse(k, b, lu, child_options, report.reports.as_deref_mut())?;
    }
    if let Some(lu) = second {
        valid &= uses::assert_loopuse(k, b, lu, child_options, report.reports.as_deref_mut())?;
    }
    if options.walk {
        valid &= super::assert_subtopology(
            k,
            b,
            crate::error::EntityRef::Loop(id),
            options.level,
            report.reports.as_deref_mut(),
        )?;
    }
    Ok(valid)
}
