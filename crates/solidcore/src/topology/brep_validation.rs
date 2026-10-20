// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Runtime validation of the maintained fork's arena `BRep` model.
//!
//! Upstream: `source/SMLib/src/SmBrep.cpp::SmBrep::ValidatePointers` at `d8ed45a`,
//! `source/SMLib/inc/SmAssertArray.h` (release report macros).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`required_links`] | required-pointer prefix, tests 112--146 |
//! | [`validate_pointers`] | `SmBrep::ValidatePointers` (remaining predicates pending) |
//!
//! Deviations: checked arena access replaces unsafe pointer dereferences. Context
//! and global list-sentinel presence are intrinsic to the Rust value model. A
//! dangling id is a Rust access error, not evidence of a different C++ assertion.
//! Report keys use type/index namespaces, scoped to this `BRep`. CCW presence walks
//! retain the source bound of 100,000 visits; region/use recursion is not used.
//! Passing the required-link prefix is not full runtime or geometric validity.

mod presence;
#[cfg(test)]
mod tests;

use super::brep::p1 as bp;
use super::{
    Brep, EdgeId, EdgeuseId, FaceId, FaceuseId, LoopId, LoopuseId, RegionId, ShellId, VertexId,
    VertexuseId,
};
use crate::error::{Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray, AssertReport, ObjectKey, ReportSpec};
use crate::kernel::Kernel;
use crate::math::UNDEF_DOUBLE;

const OP: &str = "topology::brep_validation";
const MAX_LOOP_WALK: usize = 100_000;

/// The native level/request selection; requests select predicates in source
/// execution order, not in request-array order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CheckLevel<'a> {
    /// `SM_LEVEL_0`.
    #[default]
    Zero,
    /// `SM_LEVEL_1`, including level zero.
    One,
    /// `SM_LEVEL_2`, including all lower levels.
    Two,
    /// `SM_LEVEL_GIVEN`; an empty slice also models a null request array.
    Given(&'a [u64]),
}

/// Explicit runtime-validation policy. These are the native Release/old-tolerance
/// conventions, independent of Rust's own debug-assertion build setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ValidationOptions<'a> {
    /// Native assertion selection. Required-pointer checks always run at level 0.
    pub level: CheckLevel<'a>,
    /// `SM_WALK`: a subsequent loop validator owns `CheckLoop`; skip it in
    /// `CheckFace`. The standalone `ValidatePointers` default is `false`.
    pub walk: bool,
}

/// The required-pointer prefix of `SmBrep::ValidatePointers`, in original order.
/// Stops on its first failed predicate, retains its ordinary `AssertArray`
/// report, and returns `SM_ERR`. It does not qualify the rest of validation.
pub fn required_links(b: &Brep, reports: Option<&mut AssertArray>) -> Result<()> {
    presence::check(b, &mut Reporter { reports })
}

/// Execute runtime pointer validation. The required-pointer prefix runs first;
/// the remaining validator is still an explicit unported dependency.
/// A successful prefix must never be published as full validation evidence.
pub fn validate_pointers(
    _k: &Kernel,
    b: &mut Brep,
    _options: ValidationOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<()> {
    required_links(b, reports)?;
    // UNPORTED(fork.topology_validation): SmBrep::ValidatePointers remaining checks
    Err(SolidError::Unported {
        upstream: "SmBrep::ValidatePointers remaining checks",
        unit: "fork.topology_validation",
    })
}

#[derive(Debug, Clone, Copy)]
enum Object {
    Brep,
    Region(RegionId),
    Shell(ShellId),
    Face(FaceId),
    Faceuse(FaceuseId),
    Loop(LoopId),
    Loopuse(LoopuseId),
    Edge(EdgeId),
    Edgeuse(EdgeuseId),
    Vertex(VertexId),
    Vertexuse(VertexuseId),
}
impl Object {
    const fn identity(self) -> (i64, u32, &'static str) {
        match self {
            Self::Brep => (labels::BREP_TYPE, 0, "SmBrep"),
            Self::Region(id) => (labels::REGION_TYPE, id.index(), "SmRegion"),
            Self::Shell(id) => (16013, id.index(), "SmShell"),
            Self::Face(id) => (labels::FACE_TYPE, id.index(), "SmFace"),
            Self::Faceuse(id) => (16022, id.index(), "SmFaceuse"),
            Self::Loop(id) => (labels::LOOP_TYPE, id.index(), "SmLoop"),
            Self::Loopuse(id) => (labels::LOOPUSE_TYPE, id.index(), "SmLoopuse"),
            Self::Edge(id) => (labels::EDGE_TYPE, id.index(), "SmEdge"),
            Self::Edgeuse(id) => (labels::EDGEUSE_TYPE, id.index(), "SmEdgeuse"),
            Self::Vertex(id) => (labels::VERTEX_TYPE, id.index(), "SmVertex"),
            Self::Vertexuse(id) => (labels::VERTEXUSE_TYPE, id.index(), "SmVertexuse"),
        }
    }
    fn key(self) -> ObjectKey {
        let (ty, index, _) = self.identity();
        ObjectKey(((ty as u64) << 32) | u64::from(index))
    }
}

struct Reporter<'a> {
    reports: Option<&'a mut AssertArray>,
}
impl Reporter<'_> {
    fn predicate(&mut self, test: u64, owner: Object, passed: bool) -> Result<bool> {
        if let Some(reports) = self.reports.as_deref_mut() {
            if passed {
                reports.set_last_add(false);
            } else {
                let (ty, _, name) = owner.identity();
                reports.add_unique_delete_duplicate(AssertReport::new(ReportSpec {
                    reporter: Some(Object::Brep.key()),
                    test_index: test,
                    test_class: labels::BREP_TYPE,
                    owner: Some(owner.key()),
                    owner_type: ty,
                    owner_type_string: Some(name),
                    other_type_string: Some("NULL"),
                    tol3d: UNDEF_DOUBLE,
                    value: UNDEF_DOUBLE,
                    list_index: 1,
                    ..ReportSpec::default()
                })?);
            }
        }
        Ok(passed)
    }
    fn required(&mut self, test: u64, owner: Object, passed: bool) -> Result<()> {
        if self.predicate(test, owner, passed)? {
            Ok(())
        } else {
            Err(SolidError::sm_err(OP))
        }
    }
}
