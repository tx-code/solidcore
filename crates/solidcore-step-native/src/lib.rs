// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! STEP-to-Rust-BRep construction, separate from the pure source reader.
//!
//! Upstream: maintained `source/STEP_IMPORT/src/StepImportPlan.cpp` and
//! `StepImportNative.cpp` at `d8ed45a`. The first executable bridge follows their
//! open planar-face construction. General/closed-shape qualification is separate.
//!
//! A construction candidate is NOT an accepted import. Native pointer/deep checks
//! and continuous source/UV/error-budget qualification must precede acceptance.
//! No C++ reader/kernel, mesh substitution, stitching or repair is used.
#![forbid(unsafe_code)]

mod bounds;
mod certify;
#[cfg(feature = "general-development")]
mod general;
mod plan;
pub use certify::{AcceptanceQuality, Certificate, Imported};
#[cfg(feature = "general-development")]
pub use general::{
    construct_general, GeneralLoopMapping, GeneralObservations, GeneralQuality, GeneralReport,
    GeneralTrimMeasurements, GeneralTrimUseMeasurement, ParameterMapping, UnitConversion,
};
#[cfg(feature = "general-development")]
pub use general::{
    GeneralMaterialEdgeMeasurement, GeneralMaterialFaceMeasurement, GeneralMaterialMeasurements,
};

use plan::{ExactPoint, Plan};

use solidcore::curve::{curve::Curve, line::Line};
use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Vec2, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::plane::Plane;
use solidcore::topology::{
    brep_constructor::{BrepConstructor, VertexNode},
    edgeuse::p1 as eu,
    Brep, EdgeId, FaceId, VertexId,
};
use solidcore_step::{planar as p, semantics as s};
use std::collections::BTreeMap;

type Result<T> = std::result::Result<T, Error>;

/// Explicit application resource/working-tolerance selection. No production
/// default or source uncertainty is inferred by the native builder.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Maximum charged construction operations.
    pub operations: usize,
    /// Native working tolerance in millimeters; separate from source acceptance.
    pub work_tolerance_mm: f64,
}

/// Construction error; no partial native candidate accompanies a failure.
#[derive(Debug)]
pub enum Error {
    /// General source checking failed; original source diagnostics are retained.
    Source {
        /// Actual source-check status, independent of native errors.
        status: s::Status,
        /// Original typed source diagnostics, without fabricated model errors.
        diagnostics: Vec<s::Diagnostic>,
        /// Original nongeometric diagnostics kept by the independent source policy.
        preserved_diagnostics: Vec<s::Diagnostic>,
        /// Original diagnostics associated with an explicit schema interpretation.
        interpretation_diagnostics: Vec<s::Diagnostic>,
        /// Original unit-marker diagnostics, including when the development
        /// proof or a later phase failed and no source snapshot was published.
        development_unit_diagnostics: Vec<s::Diagnostic>,
    },
    /// A source-addressed general construction or qualification failure.
    General {
        /// Stable maintained-fork diagnostic identity.
        key: &'static str,
        /// Original source record index.
        record: usize,
        /// Unsupported scope, unresolved numerical evidence or failed postcheck.
        kind: GeneralFailureKind,
    },
    /// The source did not finish the required planar checkpoint.
    Upstream,
    /// Construction capability is not implemented for this source shape.
    Unsupported(&'static str),
    /// A source/evidence/configuration invariant is missing or inconsistent.
    Invalid(&'static str),
    /// Exact arithmetic cannot establish the required conversion.
    Numeric(p::ExactError),
    /// The explicitly selected native operation budget was exhausted.
    ResourceLimit,
    /// The actual Rust kernel operation failed or remains unported.
    Kernel(solidcore::SolidError),
    /// A completed native validator rejected the complete candidate.
    Postcheck(&'static str, usize),
    /// Required continuous evidence could not be established or exceeded budget.
    Evidence(&'static str),
}
/// Classification of a general source-to-native failure, never a source rewrite.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeneralFailureKind {
    /// Required source conversion is not implemented for this explicit path.
    Unsupported,
    /// Required numerical evidence could not be established.
    Indeterminate,
    /// A supported native/source correspondence check failed.
    Postcheck,
}
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for Error {}
impl From<solidcore::SolidError> for Error {
    fn from(value: solidcore::SolidError) -> Self {
        Self::Kernel(value)
    }
}
impl From<p::ExactError> for Error {
    fn from(value: p::ExactError) -> Self {
        Self::Numeric(value)
    }
}

/// A computational chart vertex on an unchanged source defining curve.
/// This is not an invented source `VERTEX_POINT` or a moved original vertex.
#[derive(Clone, Debug)]
pub struct GeneralDerivedVertex {
    /// Original source edge whose complete basis curve contains this point.
    pub source_edge: usize,
    /// Actual native vertex shared by every incident target piece and chart cut.
    pub native_vertex: VertexId,
    /// Parameter on the original complete native defining curve, not an angle.
    pub native_parameter: f64,
}
/// Actual owning Rust candidate with immutable source/native correspondence.
/// It is deliberately not named Accepted: independent import gates remain unrun.
pub struct Candidate {
    brep: Brep,
    kernel: Kernel,
    plan: SourcePlan,
    vertices: BTreeMap<usize, VertexId>,
    edges: BTreeMap<usize, Vec<EdgeId>>,
    faces: BTreeMap<usize, FaceId>,
    operations: usize,
    operation_limit: usize,
    native_checks: Option<NativeChecks>,
    initial_work_tolerance_mm: f64,
}
enum SourcePlan {
    Planar(Box<Plan>),
    #[cfg(feature = "general-development")]
    General(Box<general::Plan>),
}
impl SourcePlan {
    fn source(&self) -> &s::Snapshot {
        match self {
            Self::Planar(plan) => plan.source(),
            #[cfg(feature = "general-development")]
            Self::General(plan) => plan.source(),
        }
    }
}
/// Actual executed native checks, independent of source/continuous acceptance.
#[derive(Clone, Copy, Debug)]
pub struct NativeChecks {
    /// Number of reports from the native pointer validator.
    pub pointer_reports: usize,
    /// Number of reports from the native level-two descendant walk.
    pub deep_reports: usize,
}
impl Candidate {
    #[cfg_attr(
        not(feature = "general-development"),
        expect(
            clippy::unnecessary_wraps,
            reason = "The same certificate interface must refuse general candidates when that feature is enabled"
        )
    )]
    fn planar_plan(&self) -> Result<&Plan> {
        match &self.plan {
            SourcePlan::Planar(plan) => Ok(plan),
            #[cfg(feature = "general-development")]
            SourcePlan::General(_) => Err(Error::Unsupported(
                "general development data has no planar certificate",
            )),
        }
    }
    /// Actual observations from the explicit general development constructor.
    /// They are not continuous certificates and do not establish accepted import.
    #[cfg(feature = "general-development")]
    pub fn general_observations(&self) -> Option<GeneralObservations> {
        match &self.plan {
            SourcePlan::General(plan) => Some(plan.observations()),
            SourcePlan::Planar(_) => None,
        }
    }
    /// Borrow the original general source and native construction observations.
    /// A view never upgrades a development candidate to an accepted import.
    #[cfg(feature = "general-development")]
    pub fn general_report(&self) -> Option<GeneralReport<'_>> {
        match &self.plan {
            SourcePlan::General(plan) => Some(plan.report()),
            SourcePlan::Planar(_) => None,
        }
    }
    /// Borrow the actual Rust topology/geometry, not a mesh or reconstructed view.
    pub fn brep(&self) -> &Brep {
        &self.brep
    }
    /// Borrow the candidate's own kernel context.
    pub fn kernel(&self) -> &Kernel {
        &self.kernel
    }
    /// Classify UV against this candidate's actual trimmed face and owner context.
    /// Native query cache/UV creation may mutate derived state. This does not
    /// grant import acceptance or expose mutable topology to a viewer.
    ///
    /// # Errors
    /// Nonfinite UV, an unmapped face, or the actual native query failure,
    /// including unimplemented dependencies. No copied model/default context
    /// or mesh classification substitutes for the owning native query.
    pub fn classify_face_uv(
        &mut self,
        face: FaceId,
        uv: Vec2,
    ) -> Result<solidcore::intersect::point_class::PointClassType> {
        if !uv.x.is_finite()
            || !uv.y.is_finite()
            || !self.faces.values().any(|mapped| *mapped == face)
        {
            return Err(Error::Invalid("unmapped or nonfinite face UV query"));
        }
        #[cfg(feature = "general-development")]
        if let SourcePlan::General(plan) = &mut self.plan {
            // Native UV queries may lazily change derived representation, even
            // on failure. A previous final-trim observation cannot certify the
            // later state; the complete measurement must be run again.
            plan.invalidate_final_trim_measurements();
        }
        Ok(solidcore::query::api_queries::face_classify_uv(
            &self.kernel,
            &mut self.brep,
            face,
            uv,
        )?)
    }
    /// Original immutable semantic/source owner, with original bytes and units.
    pub fn source(&self) -> &s::Snapshot {
        self.plan.source()
    }
    /// Original source-vertex indices mapped to actual native vertices.
    pub fn vertices(&self) -> &BTreeMap<usize, VertexId> {
        &self.vertices
    }
    /// Original source-edge indices mapped to every native piece, in increasing
    /// defining-curve parameter order. A computational split never hides a piece.
    pub fn edges(&self) -> &BTreeMap<usize, Vec<EdgeId>> {
        &self.edges
    }
    /// Complete provenance of computational vertices added during chart splitting.
    /// Strict planar construction adds none; original source vertices remain in `vertices()`.
    pub fn derived_vertices(&self) -> &[GeneralDerivedVertex] {
        match &self.plan {
            SourcePlan::Planar(_) => &[],
            #[cfg(feature = "general-development")]
            SourcePlan::General(plan) => &plan.derived_vertices,
        }
    }
    /// Original source-face indices mapped to actual native faces.
    pub fn faces(&self) -> &BTreeMap<usize, FaceId> {
        &self.faces
    }
    /// Charged construction operations, independent of source work counters.
    pub const fn operations(&self) -> usize {
        self.operations
    }
    /// Native checks actually completed for this owning candidate.
    pub const fn native_checks(&self) -> Option<NativeChecks> {
        self.native_checks
    }
    /// Execute pointer and level-two topology/geometry validation on the actual
    /// candidate. Failure consumes and releases the whole unpublished model.
    /// This does not substitute for continuous source/native error certificates.
    ///
    /// # Errors
    /// Actual validator dependencies/failures, or any nonempty native report.
    pub fn validate_native(mut self) -> Result<Self> {
        use solidcore::geom::assert_array::AssertArray;
        use solidcore::topology::brep_validation::{
            self as validation, AssertValidOptions, CheckLevel, ValidationOptions,
        };
        #[cfg(feature = "general-development")]
        if let SourcePlan::General(plan) = &mut self.plan {
            // The real validator can materialize or change derived UV state.
            // Revalidating a measured candidate must discard observations of
            // its previous state before any native operation starts.
            plan.invalidate_final_trim_measurements();
        }
        let mut pointers = AssertArray::new();
        self.charge()?;
        validation::validate_pointers(
            &mut self.kernel,
            &mut self.brep,
            ValidationOptions::default(),
            Some(&mut pointers),
        )?;
        if !pointers.is_empty() {
            return Err(Error::Postcheck("NativePointerValidation", pointers.len()));
        }
        let mut deep = AssertArray::new();
        self.charge()?;
        let valid = validation::assert_valid(
            &mut self.kernel,
            &mut self.brep,
            AssertValidOptions {
                level: CheckLevel::Two,
                walk: true,
            },
            Some(&mut deep),
        )?;
        if !valid || !deep.is_empty() {
            return Err(Error::Postcheck("NativeDeepValidation", deep.len()));
        }
        self.native_checks = Some(NativeChecks {
            pointer_reports: pointers.len(),
            deep_reports: deep.len(),
        });
        Ok(self)
    }
    fn charge(&mut self) -> Result<()> {
        if self.operations >= self.operation_limit {
            return Err(Error::ResourceLimit);
        }
        self.operations = self.operations.checked_add(1).ok_or(Error::ResourceLimit)?;
        Ok(())
    }
}

struct Execution {
    used: usize,
    max: usize,
}
impl Execution {
    fn step(&mut self) -> Result<()> {
        if self.used >= self.max {
            return Err(Error::ResourceLimit);
        }
        self.used = self.used.checked_add(1).ok_or(Error::ResourceLimit)?;
        Ok(())
    }
}

fn value(r: p::Rational) -> Result<f64> {
    let result = r.numerator() as f64 / r.denominator() as f64;
    if result.is_finite() {
        Ok(result)
    } else {
        Err(Error::Invalid("nonfinite native conversion"))
    }
}
fn native(p: ExactPoint) -> Result<Vec3> {
    Ok(Vec3::new(value(p[0])?, value(p[1])?, value(p[2])?))
}
/// Build the first complete open planar source shape on the Rust kernel.
/// Preserves shared identities, source senses, and defining LINE geometry.
///
/// The result is a construction candidate. No import acceptance, numerical bound,
/// validator, material volume or repair qualification is implied by this entry.
///
/// # Errors
/// Source/shape/configuration errors, bounded arithmetic exhaustion, operation
/// limit and actual kernel failures. All unpublished topology releases on error.
#[allow(clippy::too_many_lines)]
pub fn construct_planar(checked: &p::Outcome, options: Options) -> Result<Candidate> {
    let mut plan = Plan::new(checked, options.work_tolerance_mm)?;
    let closed = plan.closed();
    let material = plan.material();
    let faces_source = &checked.checked_evidence().ok_or(Error::Upstream)?.faces;
    let mut execution = Execution {
        used: 0,
        max: options.operations,
    };
    execution.step()?;
    let mut builder = BrepConstructor::new()?;
    let mut kernel = Kernel::default();
    solidcore::topology::brep::p2::set_tolerance(
        &kernel,
        builder.brep_mut(),
        options.work_tolerance_mm,
        false,
        false,
    )?;
    let mut vertices = BTreeMap::new();
    for id in faces_source
        .iter()
        .flat_map(|f| f.loops.iter())
        .flat_map(|l| l.vertices.iter())
    {
        if vertices.contains_key(id) {
            continue;
        }
        let exact = plan.prepare_vertex(*id)?;
        execution.step()?;
        let target = builder.start_vertex(native(exact)?)?;
        builder.brep_mut().vertex_mut(target)?.tolerance = options.work_tolerance_mm;
        builder.end_vertex();
        vertices.insert(*id, target);
    }
    let mut definitions = BTreeMap::new();
    for index in 0..plan.evidence().edges.len() {
        let edge = plan.prepare_edge(index)?;
        definitions.insert(
            edge.edge,
            (
                edge.same_sense,
                edge.minimum_vertex,
                edge.maximum_vertex,
                native(edge.minimum_mm)?,
                native(edge.maximum_mm)?,
            ),
        );
    }
    // Only the source's material-solid claim creates a material region. A
    // closed surface model gets a bounded VOID region, never a solid upgrade.
    if closed {
        execution.step()?;
        let region = builder.start_region(material, None)?;
        builder.brep_mut().region_mut(region)?.is_void = i32::from(!material);
    }
    execution.step()?;
    builder.start_shell()?;
    let mut edges = BTreeMap::new();
    let mut faces = BTreeMap::new();
    let mut frames = Vec::new();
    for face in &plan.evidence().faces {
        let origin = native(face.origin_mm)?;
        let x = native(face.x)? / value(face.x_norm_squared)?.sqrt();
        let y = native(face.y)? / value(face.y_norm_squared)?.sqrt();
        let uv = |p: Vec3| {
            let d = p - origin;
            Vec3::new(d.dot(x), d.dot(y), 0.)
        };
        let mut selected = std::collections::BTreeSet::new();
        let mut min = Vec2::new(f64::INFINITY, f64::INFINITY);
        let mut max = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for record in face.loops.iter().flat_map(|l| l.edges.iter()) {
            if !selected.insert(*record) {
                return Err(Error::Unsupported(
                    "repeated same-face occurrence requires seam support",
                ));
            }
            let (_, _, _, a, b) = definitions
                .get(record)
                .ok_or(Error::Invalid("unmapped required edge"))?;
            for p in [*a, *b] {
                let p = uv(p);
                min.x = min.x.min(p.x);
                min.y = min.y.min(p.y);
                max.x = max.x.max(p.x);
                max.y = max.y.max(p.y);
            }
        }
        let pad = 64.
            * f64::EPSILON
            * (1.
                + min
                    .x
                    .abs()
                    .max(min.y.abs())
                    .max(max.x.abs())
                    .max(max.y.abs()));
        let domain = Extent2d::from_bounds(
            (min.x - pad).next_down(),
            (min.y - pad).next_down(),
            (max.x + pad).next_up(),
            (max.y + pad).next_up(),
        );
        execution.step()?;
        let id = builder.start_face(if face.same_sense {
            OrientType::Same
        } else {
            OrientType::Opposite
        })?;
        builder.set_face_surface(
            Plane::new(origin, x, y, Vec2::new(1., 1.), &domain).into(),
            domain,
        )?;
        faces.insert(face.face, id);
        // Capture the actual adopted plane coefficients; its placement
        // constructor may normalize the supplied axes again.
        let Some(solidcore::surface::surface::SurfaceKind::Plane(plane)) = builder
            .brep()
            .face(id)?
            .surface
            .as_ref()
            .map(solidcore::surface::surface::Surface::kind)
        else {
            return Err(Error::Invalid("adopted planar receiver changed kind"));
        };
        frames.push((
            id,
            selected,
            plane.position().origin(),
            plane.position().x_axis(),
            plane.position().y_axis(),
        ));
        let mut loops: Vec<_> = face.loops.iter().collect();
        loops.sort_by_key(|l| !l.outer);
        for source_loop in loops {
            execution.step()?;
            builder.start_loop(OrientType::Same)?;
            let forward = source_loop.orientation == face.same_sense;
            for offset in 0..source_loop.edges.len() {
                let i = if forward {
                    offset
                } else {
                    source_loop.edges.len() - 1 - offset
                };
                let record = *source_loop
                    .edges
                    .get(i)
                    .ok_or(Error::Invalid("loop edge index"))?;
                let sense = *source_loop
                    .edge_orientations
                    .get(i)
                    .ok_or(Error::Invalid("loop sense index"))?;
                let (edge_sense, low, high, a, b) = *definitions
                    .get(&record)
                    .ok_or(Error::Invalid("required defining edge"))?;
                let along = if forward {
                    sense == edge_sense
                } else {
                    sense != edge_sense
                };
                let existing = edges.get(&record).copied();
                execution.step()?;
                let native = builder.start_edge(
                    &kernel,
                    if along {
                        OrientType::Same
                    } else {
                        OrientType::Opposite
                    },
                    existing,
                    Some(*vertices.get(&low).ok_or(Error::Invalid("minimum vertex"))?),
                    Some(
                        *vertices
                            .get(&high)
                            .ok_or(Error::Invalid("maximum vertex"))?,
                    ),
                )?;
                if existing.is_none() {
                    edges.insert(record, native);
                    builder.brep_mut().edge_mut(native)?.tolerance = options.work_tolerance_mm;
                    builder.set_edge_curve(&kernel, Line::from_points(a, b, 3).into(), false)?;
                    builder.set_limits(
                        &kernel,
                        VertexNode::Parameter(0.),
                        VertexNode::Parameter(1.),
                    )?;
                } else if existing != Some(native) {
                    return Err(Error::Invalid("shared edge identity changed"));
                }
                builder.end_edge()?;
            }
            builder.end_loop(&kernel)?;
        }
        builder.end_face()?;
    }
    if edges.len() != definitions.len() {
        return Err(Error::Invalid("required source edge was omitted"));
    }
    builder.end_shell(&mut kernel, false)?; // Strict construction: no stitching/healing.
    if closed {
        builder.end_region()?;
    }
    execution.step()?;
    let mut brep = builder.finish()?;
    for (face, selected, origin, x, y) in frames {
        let uv = |p: Vec3| {
            let d = p - origin;
            Vec3::new(d.dot(x), d.dot(y), 0.)
        };
        for record in selected {
            let edge = *edges
                .get(&record)
                .ok_or(Error::Invalid("native edge lookup"))?;
            let mut target = None;
            for use_id in &brep.edge(edge)?.edgeuses {
                if eu::face(&brep, *use_id)? == Some(face)
                    && eu::orientation(&brep, *use_id)? == OrientType::Same
                    && target.replace(*use_id).is_some()
                {
                    return Err(Error::Invalid("duplicate same-side incidence"));
                }
            }
            let use_id = target.ok_or(Error::Invalid("missing face occurrence"))?;
            // Native InstallUv uses the ORIGINAL defining Line's stored
            // control polygon, not source vertices or a fit to their positions.
            let curve = brep
                .edge_mut(edge)?
                .curve
                .as_mut()
                .ok_or(Error::Invalid("missing defining curve"))?;
            curve.materialize_stored_bspline(&kernel)?;
            let solidcore::curve::curve::CurveKind::Line(line) = curve.kind_mut() else {
                return Err(Error::Invalid("nonlinear planar defining curve"));
            };
            let controls = line
                .base()
                .gw_nurb()
                .ok_or(Error::Invalid("missing defining NURB"))?
                .cpts();
            if controls.len() != 2 {
                return Err(Error::Invalid("nonaffine defining control polygon"));
            }
            let p = |i: usize| -> Result<Vec3> {
                let p = controls.get(i).ok_or(Error::Invalid("control index"))?;
                Ok(Vec3::new(p.x, p.y, p.z))
            };
            let (a, b) = (p(0)?, p(1)?);
            execution.step()?;
            eu::set_uv_trim_curve(
                &mut brep,
                use_id,
                eu::UvTrimCurveArg::New(Box::new(Curve::from(Line::from_points(uv(a), uv(b), 2)))),
                0.,
                true,
                None,
            )?;
        }
    }
    Ok(Candidate {
        brep,
        kernel,
        plan: SourcePlan::Planar(Box::new(plan)),
        vertices,
        edges: edges
            .into_iter()
            .map(|(source, edge)| (source, vec![edge]))
            .collect(),
        faces,
        operations: execution.used,
        operation_limit: options.operations,
        native_checks: None,
        initial_work_tolerance_mm: options.work_tolerance_mm,
    })
}
