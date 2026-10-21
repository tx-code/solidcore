// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Fixed-profile planar source rules and exact directed-region evidence.
//! Upstream: `source/STEP_PLANAR/inc/StepPlanar.h`, `src/StepPlanar.cpp` at `d8ed45a`.
//! Deviations: safe owning source views, fallible arithmetic and structured request
//! failures; no kernel, native model, repair or approximate numeric fallback.

#[rustfmt::skip]
mod catalog;
mod exact;
mod geometry;
mod global;
mod local;
mod registry;
mod report;
mod validator;

use crate::semantics;
pub use exact::{ExactError, Rational};
use serde::Serialize;
use std::collections::BTreeMap;
use std::sync::Arc;

/// Computed EXPRESS LOGICAL; an unevaluated expression uses `None`, not UNKNOWN.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Logical {
    /// Computed FALSE.
    False,
    /// Computed TRUE; the only value selected by QUERY.
    True,
    /// Computed UNKNOWN, distinct from an unevaluated expression.
    Unknown,
}
impl Logical {
    /// EXPRESS NOT, retaining UNKNOWN.
    #[must_use]
    pub const fn not(self) -> Self {
        match self {
            Self::False => Self::True,
            Self::True => Self::False,
            Self::Unknown => Self::Unknown,
        }
    }
    /// Whether QUERY selects this computed logical value.
    pub const fn query_selects(self) -> bool {
        matches!(self, Self::True)
    }
}
/// Exact 3-vector; units follow each evidence field's contract.
pub type Vector = [Rational; 3];
/// An explicitly named consistency configuration, independent of source uncertainty.
#[derive(Clone, Debug, Serialize)]
pub struct Quality {
    /// Caller-selected identity; a frozen preset cannot be reused with other values.
    pub id: String,
    /// Positive consistency distance, in millimeters.
    pub consistency_mm: Rational,
}
impl Quality {
    /// Explicit synthetic preset `Q-P0-mm-01`: 1e-7 mm, not a production default.
    pub fn planar_fixture() -> Self {
        Self {
            id: "Q-P0-mm-01".into(),
            consistency_mm: Rational::FIXTURE_MM,
        }
    }
}
/// Explicit source-rule/region work budget; never a geometric tolerance.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Limits {
    /// Maximum charged work units; implementation ceiling is ten million.
    pub work: usize,
}
impl Limits {
    /// Original synthetic preset; callers must explicitly choose their own limits.
    pub const fn fixtures() -> Self {
        Self { work: 1_000_000 }
    }
}
/// One actual local/global rule evaluation; indices address the owning source.
#[derive(Clone, Debug, Serialize)]
pub struct RuleEvidence {
    /// Stable source rule name; private numeric identities are never serialized.
    pub name: String,
    /// WHERE, `TYPE_WHERE`, UNIQUE, INVERSE or GLOBAL.
    pub kind: &'static str,
    /// Mandatory, Conditional or Guarded.
    pub applicability: &'static str,
    /// Declaration line in the fixed schema artifact.
    pub schema_line: usize,
    /// Snapshot-local instance owner, absent for full-file/empty guards.
    pub owner: Option<usize>,
    /// Computed LOGICAL; `None` means unevaluated or unestablished.
    pub value: Option<Logical>,
    /// `NotRun`, Evaluated or Indeterminate.
    pub evaluation: &'static str,
    /// Actual proof, including source/project/closed-profile qualifications.
    pub proof: String,
    /// Local: owner population only. Global: complete original FOR populations.
    pub populations: BTreeMap<String, Vec<usize>>,
    /// Ordered original source witnesses.
    pub witnesses: Vec<usize>,
    /// Exact derived observations, without guessed defaults.
    pub facts: BTreeMap<String, String>,
}
impl RuleEvidence {
    fn new(name: impl Into<String>, kind: &'static str, line: usize) -> Self {
        Self {
            name: name.into(),
            kind,
            schema_line: line,
            applicability: "Mandatory",
            owner: None,
            value: None,
            evaluation: "NotRun",
            proof: String::new(),
            populations: BTreeMap::new(),
            witnesses: Vec::new(),
            facts: BTreeMap::new(),
        }
    }
}
/// Exact consistency or metric observation, not a native measurement qualification.
#[derive(Clone, Debug, Serialize)]
pub struct Metric {
    /// Observation family.
    pub kind: &'static str,
    /// Original source record index.
    pub owner: usize,
    /// Ordered original witnesses.
    pub witnesses: Vec<usize>,
    /// Exact named quantities with units stated in their keys.
    pub exact: BTreeMap<String, Rational>,
}
/// Exact LINE interval evidence; source parameters remain dimensionless.
#[derive(Clone, Debug, Serialize)]
pub struct EdgeEvidence {
    /// Original edge, line and endpoint vertex indices.
    pub edge: usize,
    /// Original line record.
    pub line: usize,
    /// Original start vertex.
    pub start: usize,
    /// Original end vertex.
    pub end: usize,
    /// Original EC sense; no endpoint swap is a repair.
    pub same_sense: bool,
    /// Start t = coefficient / sqrt(direction norm squared).
    pub t_start_coefficient: Rational,
    /// End t = coefficient / sqrt(direction norm squared).
    pub t_end_coefficient: Rational,
    /// Exact source direction squared norm.
    pub direction_norm_squared: Rational,
    /// Squared physical segment length in mm².
    pub length_squared_mm: Rational,
}
/// One projected loop; source topology, orientation and parameterization stay intact.
#[derive(Clone, Debug, Serialize)]
pub struct LoopEvidence {
    /// Original bound record.
    pub bound: usize,
    /// Original loop record.
    pub loop_record: usize,
    /// Explicit outer-bound type.
    pub outer: bool,
    /// Source bound orientation.
    pub orientation: bool,
    /// Original ordered oriented edges.
    pub oriented_edges: Vec<usize>,
    /// Original ordered underlying edges.
    pub edges: Vec<usize>,
    /// Original ordered vertex identities.
    pub vertices: Vec<usize>,
    /// Original oriented-edge senses.
    pub edge_orientations: Vec<bool>,
    /// Exact projected coordinates in the unnormalized orthogonal basis.
    pub projected: Vec<[Rational; 2]>,
    /// Physical signed area = coefficient * sqrt(face metric squared).
    pub signed_area_coefficient: Rational,
}
/// Exact projected region evidence; does not create target UV trims or modify source.
#[derive(Clone, Debug, Serialize)]
pub struct FaceEvidence {
    /// Original face record.
    pub face: usize,
    /// Original plane record.
    pub plane: usize,
    /// Original placement record.
    pub placement: usize,
    /// Original face sense.
    pub same_sense: bool,
    /// Placement origin in millimeters.
    pub origin_mm: Vector,
    /// Exact unnormalized projected x basis.
    pub x: Vector,
    /// Exact unnormalized projected y basis.
    pub y: Vector,
    /// Exact unnormalized source normal.
    pub normal: Vector,
    /// Exact source length-to-mm scale, applied once.
    pub source_length_scale_mm: Rational,
    /// Exact algebraic mapping to original source UV; never a target trim mapping.
    pub source_uv_formula: &'static str,
    /// Squared basis x norm.
    pub x_norm_squared: Rational,
    /// Squared basis y norm.
    pub y_norm_squared: Rational,
    /// Product of squared basis norms, with radicals retained algebraically.
    pub metric_squared: Rational,
    /// Squared projected region area in mm⁴.
    pub area_squared_mm: Rational,
    /// Projected region centroid in millimeters.
    pub centroid_mm: Vector,
    /// Projected region axis-aligned minimum, in millimeters.
    pub minimum_mm: Vector,
    /// Projected region axis-aligned maximum, in millimeters.
    pub maximum_mm: Vector,
    /// Original source loops with exact projected observations.
    pub loops: Vec<LoopEvidence>,
}
/// One exactly resolved conversion definition, not a guessed name-based unit.
#[derive(Clone, Debug, Serialize)]
pub struct UnitConversion {
    /// Original unit record.
    pub unit: usize,
    /// Original dimensions record.
    pub dimensions: usize,
    /// Original factor record.
    pub factor: usize,
    /// Original base unit record.
    pub base_unit: usize,
    /// Original typed factor branch.
    pub value_type: String,
    /// Exact source factor value.
    pub factor_value: Rational,
    /// Exact normalized millimeter scale.
    pub scale_mm: Rational,
    /// Seven original dimensional exponents.
    pub exponents: [Rational; 7],
}
/// Owning diagnostic evidence; partial evidence is never an accepted native model.
#[derive(Clone)]
pub struct Evidence {
    /// The immutable original semantic/source owner, even after outcomes are dropped.
    pub source: Option<semantics::Snapshot>,
    /// Ordered actual rule evaluations.
    pub rules: Vec<RuleEvidence>,
    /// Ordered exact metric observations.
    pub metrics: Vec<Metric>,
    /// Ordered edge intervals.
    pub edges: Vec<EdgeEvidence>,
    /// Ordered projected face regions.
    pub faces: Vec<FaceEvidence>,
    /// Resolved unit definitions; shared bases are not duplicated into geometry.
    pub conversions: Vec<UnitConversion>,
}
/// Source evidence status; no variant qualifies a native model or repair.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub enum Status {
    /// All currently supported source rules and planar checks completed.
    Checked,
    /// A supported source rule or geometric consistency requirement failed.
    Rejected,
    /// The requested source/profile lies outside this rule implementation.
    Unsupported,
    /// A subsequent interpretation remains explicitly deferred.
    Deferred,
    /// Required exact evidence could not be established; no numeric fallback.
    Indeterminate,
    /// Configured work budget or implementation ceiling exceeded.
    ResourceLimit,
    /// The preceding semantic checkpoint did not qualify this layer's input.
    UpstreamFailure,
    /// Caller configuration or an incomplete semantic handoff was invalid.
    InvalidRequest,
    /// Internal binding/registry invariant failed, distinct from a source defect.
    InfrastructureFailure,
}
/// Diagnostic result, bound to one original source; native/model work is unrun.
pub struct Outcome {
    source_capture: Option<Arc<[u8]>>,
    // Only the checker can install this success witness. Public status/evidence
    // remain editable diagnostic projections and cannot mint native eligibility.
    checked_evidence: Option<Arc<Evidence>>,
    /// Final planar source status.
    pub status: Status,
    /// Explicit quality actually selected.
    pub quality: Quality,
    /// Explicit work budget.
    pub limits: Limits,
    /// Charged work, excluding a denied charge as in the C++ contract.
    pub work: usize,
    /// Actual local rule progress.
    pub local_rules: &'static str,
    /// Actual global rule progress.
    pub global_rules: &'static str,
    /// Actual planar region progress.
    pub geometry: &'static str,
    /// Actual unit conversion interpretation progress.
    pub conversion_units: &'static str,
    /// Ordered partial/complete diagnostic observations, never native eligibility.
    /// Editing these does not change the checker's immutable success witness.
    pub evidence: Evidence,
    /// Original diagnostic identities, positions, responsibility and witnesses.
    pub diagnostics: Vec<semantics::Diagnostic>,
}
impl Outcome {
    /// Borrow the original evidence from a successful source check. Only this
    /// immutable witness can authorize a subsequent native construction stage;
    /// an edited public status or diagnostic projection does not qualify it.
    pub fn checked_evidence(&self) -> Option<&Evidence> {
        self.checked_evidence.as_deref()
    }
}
/// Check source rules and directed regions using an explicit quality and budget.
pub fn check(upstream: &semantics::Outcome, quality: Quality, limits: Limits) -> Outcome {
    validator::check(upstream, quality, limits)
}
/// Source-bound report compilation failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReportError {
    /// The supplied source/semantic/planar objects do not share their original capture.
    SourceMismatch,
    /// The atomic bounded writer failed.
    Output(crate::report::Error),
}
/// Compile the complete diagnostic report before publication, under the source budget.
pub fn compile_report(
    raw: &crate::Outcome,
    sem: &semantics::Outcome,
    planar: &Outcome,
) -> Result<Vec<u8>, ReportError> {
    report::compile(raw, sem, planar)
}
/// Fixed source rule-catalog fingerprint; not proof of AP conformance.
pub fn catalog_source_sha256() -> &'static str {
    catalog::CATALOG_SHA256
}
