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
//! | [`validate_pointers`] | `SmBrep::ValidatePointers` (Release, old tolerances) |
//!
//! Deviations: checked arena access replaces unsafe pointer dereferences. Context
//! and global list-sentinel presence are intrinsic to the Rust value model. A
//! dangling id is a Rust access error, not evidence of a different C++ assertion.
//! Report keys use type/index namespaces, scoped to this `BRep`. CCW presence walks
//! retain the source bound of 100,000 visits; region/use recursion is not used.
//! Passing the required-link prefix is not full runtime or geometric validity.
//! A completed pointer walk can return success with reports. Full root/local
//! `AssertValid` now has an ordered entry; missing local/geometric gates remain
//! explicit errors. See `docs/BREP_VALIDATION.md` for qualification boundaries.

mod assertions;
#[cfg(test)]
mod curve_tests;
mod curves;
#[cfg(test)]
mod face_loop_tests;
mod face_tests;
mod faces;
mod geometry;
mod local;
mod local_base;
mod loops;
mod pointers;
mod presence;
mod surfaces;
#[cfg(test)]
mod tests;
mod tolerances;
mod uses;

pub use assertions::{assert_valid, AssertValidOptions};
pub use curves::{assert_edge_curve, assert_uv_curve};
pub use faces::assert_face;
pub use local::{assert_region, assert_shell};
pub use loops::assert_loop;
pub use surfaces::assert_surface;
pub use tolerances::{validate_and_update_tolerances, ToleranceReport};
pub use uses::{assert_faceuse, assert_loopuse, assert_vertexuse};

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

/// Native level/request input. The Release macros use numeric levels, so these
/// level-zero pointer checks also run for `Given`, ignoring its request list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CheckLevel<'a> {
    /// `SM_LEVEL_0`.
    #[default]
    Zero,
    /// `SM_LEVEL_1`, including level zero.
    One,
    /// `SM_LEVEL_2`, including all lower levels.
    Two,
    /// `SM_LEVEL_HEAL` (3); the Release pointer checks still run at level zero.
    Heal,
    /// `SM_LEVEL_GIVEN`; the Release pointer macros ignore this request list.
    /// An empty slice also models a null request array.
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

impl CheckLevel<'_> {
    fn native_code(self) -> i32 {
        match self {
            Self::Zero => 0,
            Self::One => 1,
            Self::Two => 2,
            Self::Heal => 3,
            Self::Given(_) => 4,
        }
    }
}

/// The required-pointer prefix of `SmBrep::ValidatePointers`, in original order.
/// Stops on its first failed predicate, retains its ordinary `AssertArray`
/// report, and returns `SM_ERR`. It does not qualify the rest of validation.
pub fn required_links(b: &Brep, reports: Option<&mut AssertArray>) -> Result<()> {
    presence::check(b, &mut Reporter { reports })
}

/// Execute the native Release/old-tolerance pointer validator. A normal completed
/// walk returns success even when reports were added, exactly as C++ does; callers
/// inspect both status and reports. Unported geometry remains an explicit error.
pub fn validate_pointers(
    k: &mut Kernel,
    b: &mut Brep,
    options: ValidationOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<()> {
    let mut reporter = Reporter { reports };
    presence::check(b, &mut reporter)?;
    pointers::check(k, b, options, &mut reporter)
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
    Placement(FaceId),
    Surface(FaceId, &'static str),
    UvCurve(EdgeuseId, &'static str),
    Curve(EdgeId, &'static str),
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
            Self::Placement(id) => (labels::AXIS2_PLACEMENT_TYPE, id.index(), "SmAxis2Placement"),
            Self::Surface(id, name) => (labels::SURFACE_TYPE, id.index(), name),
            Self::UvCurve(id, name) => (labels::B_SPLINE_CURVE_TYPE, id.index(), name),
            Self::Curve(id, name) => (labels::B_SPLINE_CURVE_TYPE, id.index(), name),
        }
    }
    fn key(self) -> ObjectKey {
        // Geometry and topology are distinct objects even when their arena index
        // and concrete type repeat. Keep physical UV ownership in the key.
        match self {
            Self::Placement(id) => return ObjectKey((1_u64 << 61) | u64::from(id.index())),
            Self::Surface(id, _) => return ObjectKey((1_u64 << 63) | u64::from(id.index())),
            Self::UvCurve(id, _) => return ObjectKey((3_u64 << 62) | u64::from(id.index())),
            Self::Curve(id, _) => return ObjectKey((1_u64 << 62) | u64::from(id.index())),
            _ => {}
        }
        let (ty, index, _) = self.identity();
        ObjectKey(((ty as u64) << 32) | u64::from(index))
    }
}

struct Reporter<'a> {
    reports: Option<&'a mut AssertArray>,
}
impl Reporter<'_> {
    fn predicate(&mut self, test: u64, owner: Object, passed: bool) -> Result<bool> {
        self.value(test, owner, passed, UNDEF_DOUBLE, UNDEF_DOUBLE)
    }
    fn value(
        &mut self,
        test: u64,
        owner: Object,
        passed: bool,
        tol: f64,
        value: f64,
    ) -> Result<bool> {
        self.list_value(1, test, owner, passed, tol, value)
    }
    fn list_value(
        &mut self,
        list: u64,
        test: u64,
        owner: Object,
        passed: bool,
        tol: f64,
        value: f64,
    ) -> Result<bool> {
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
                    tol3d: tol,
                    value,
                    list_index: list,
                    ..ReportSpec::default()
                })?);
            }
        }
        Ok(passed)
    }
    fn class_predicate(
        &mut self,
        class: i64,
        test: u64,
        owner: Object,
        passed: bool,
    ) -> Result<bool> {
        self.class_value(class, test, owner, passed, UNDEF_DOUBLE, UNDEF_DOUBLE)
    }
    fn class_value(
        &mut self,
        class: i64,
        test: u64,
        owner: Object,
        passed: bool,
        tol: f64,
        value: f64,
    ) -> Result<bool> {
        self.class_record(
            owner,
            passed,
            ReportSpec {
                test_class: class,
                test_index: test,
                tol3d: tol,
                value,
                ..ReportSpec::default()
            },
        )
    }
    fn class_record(&mut self, owner: Object, passed: bool, mut spec: ReportSpec) -> Result<bool> {
        if let Some(reports) = self.reports.as_deref_mut() {
            if passed {
                reports.set_last_add(false);
            } else {
                let (_, _, name) = owner.identity();
                spec.reporter = Some(owner.key());
                spec.owner = Some(owner.key());
                spec.owner_type = spec.test_class;
                spec.owner_type_string = Some(name);
                spec.other_type_string = Some("NULL");
                reports.add_unique_delete_duplicate(AssertReport::new(spec)?);
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
