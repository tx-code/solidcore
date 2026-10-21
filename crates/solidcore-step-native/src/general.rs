// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit general development construction, independent of the P0 qualifier.
//!
//! Upstream: maintained `StepImportGeneral.cpp::Prepare/Run/BuildFace/InstallUv`
//! and `StepImport.cpp` at `d8ed45a`. This path does not claim P0 region/volume
//! proofs or complete AP validity. Source interpretation is always the existing
//! typed snapshot; source topology is never recovered from a display mesh.

mod geometry;
mod surface;

use super::{Candidate, Error, Execution, GeneralFailureKind, Options, Result, SourcePlan};
use geometry::{BuiltCurve, Geometry};
pub use geometry::{ParameterMapping, UnitConversion};
use solidcore::geom::extent3d::Extent3d;
use solidcore::kernel::Kernel;
use solidcore::math::Vec3;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::topology::{
    brep_constructor::{BrepConstructor, VertexNode},
    face::p3 as fp3,
    EdgeId, FaceId, VertexId,
};
use solidcore_step::{planar::Rational, semantics as s};
use std::collections::BTreeMap;

/// Named, explicit budgets for the general development path. No production
/// preset or source uncertainty is substituted for these application choices.
#[derive(Clone, Debug)]
pub struct GeneralQuality {
    /// Application's reusable quality identity.
    pub id: String,
    /// Defining-geometry conversion allowance, in millimeters.
    pub definition_mm: Rational,
    /// Vertex/defining-edge consistency allowance, in millimeters.
    pub consistency_mm: Rational,
    /// Derived UV projection allowance, in millimeters.
    pub derived_uv_mm: Rational,
    /// Query tolerance in millimeters, separate from a proof error bound.
    pub query_mm: Rational,
    /// Maximum observed native working tolerance in millimeters.
    pub maximum_work_tolerance_mm: Rational,
    /// Maximum growth over the explicitly selected initial tolerance, in mm.
    pub maximum_tolerance_growth_mm: Rational,
}

/// Measurements made while deriving general UV. These are native observations,
/// not the planar path's continuous interval certificates or AP qualification.
#[derive(Clone, Copy, Debug, Default)]
pub struct GeneralObservations {
    /// Largest native reported curve/surface projection gap in millimeters.
    pub maximum_curve_surface_gap_mm: f64,
    /// Largest native reported vertex gap in millimeters.
    pub maximum_vertex_gap_mm: f64,
}
/// Directed source loop receipt before equivalent computational chart cuts.
#[derive(Clone, Debug)]
pub struct GeneralLoopMapping {
    /// Original face record index.
    pub face: usize,
    /// Original `FACE_BOUND` record index.
    pub bound: usize,
    /// Original `EDGE_LOOP` record index.
    pub loop_record: usize,
    /// Whether the source explicitly declared an outer bound.
    pub declared_outer: bool,
    /// Original bound orientation.
    pub bound_orientation: bool,
    /// Oriented-edge/edge source identities and effective native-curve directions.
    pub uses: Vec<(usize, usize, bool)>,
}
/// Read-only development observations belonging to one owning candidate.
/// These retain source diagnostics and parameter conventions, without granting
/// a continuous general certificate or final application publication.
pub struct GeneralReport<'a> {
    /// Original complete source-check outcome and diagnostics.
    pub source: &'a s::Outcome,
    /// Caller-selected quality and distinct numerical budgets.
    pub quality: &'a GeneralQuality,
    /// Source-to-native defining parameter relations.
    pub parameters: &'a [ParameterMapping],
    /// Original source loop identities and effective directions.
    pub loops: &'a [GeneralLoopMapping],
    /// Original exact conversion-unit dependencies.
    pub length_unit_conversions: &'a [UnitConversion],
    /// Exact source length-to-mm factor.
    pub length_scale_mm: Rational,
    /// Source face/native derived-edge identities of chart cuts.
    pub chart_cuts: &'a [(usize, EdgeId)],
    /// Actual construction-time observations; no sampling is a proof bound.
    pub observations: GeneralObservations,
}

pub(super) struct SourceVertex {
    record: usize,
    point: Vec3,
}
pub(super) struct SourceEdge {
    record: usize,
    geometry: usize,
    vertices: [usize; 2],
    same_sense: bool,
    built: Option<BuiltCurve>,
}
#[derive(Clone, Copy)]
pub(super) struct SourceUse {
    edge: usize,
    record: usize,
    forward: bool,
}
pub(super) struct SourceLoop {
    record: usize,
    bound: usize,
    outer: bool,
    orientation: bool,
    uses: Vec<SourceUse>,
}
pub(super) struct SourceFace {
    record: usize,
    geometry: usize,
    same_sense: bool,
    loops: Vec<SourceLoop>,
}
pub(super) struct Plan {
    source: s::Snapshot,
    // Preserve actual phase results and original interoperability diagnostics.
    // The snapshot handle is shared, not another source graph interpretation.
    source_report: s::Outcome,
    quality: GeneralQuality,
    shape: usize,
    shell: usize,
    vertices: Vec<SourceVertex>,
    edges: Vec<SourceEdge>,
    faces: Vec<SourceFace>,
    mappings: Vec<ParameterMapping>,
    observations: GeneralObservations,
    normalizations: Vec<(usize, EdgeId)>,
    loop_mappings: Vec<GeneralLoopMapping>,
    unit_conversions: Vec<UnitConversion>,
    length_scale_mm: Rational,
}

impl Plan {
    pub fn source(&self) -> &s::Snapshot {
        &self.source
    }
    pub fn observations(&self) -> GeneralObservations {
        self.observations
    }
    pub fn report(&self) -> GeneralReport<'_> {
        GeneralReport {
            source: &self.source_report,
            quality: &self.quality,
            parameters: &self.mappings,
            loops: &self.loop_mappings,
            length_unit_conversions: &self.unit_conversions,
            length_scale_mm: self.length_scale_mm,
            chart_cuts: &self.normalizations,
            observations: self.observations,
        }
    }
}

fn failure(key: &'static str, record: usize, kind: GeneralFailureKind) -> Error {
    Error::General { key, record, kind }
}
fn need(condition: bool, key: &'static str, record: usize) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(failure(key, record, GeneralFailureKind::Postcheck))
    }
}
fn unsupported(key: &'static str, record: usize) -> Error {
    failure(key, record, GeneralFailureKind::Unsupported)
}
fn scalar(value: Rational) -> f64 {
    value.numerator() as f64 / value.denominator() as f64
}
fn role(source: &s::Snapshot, name: &str) -> Result<usize> {
    source
        .part_definition()
        .and_then(|p| p.roles.iter().find(|r| r.name == name))
        .map(|r| r.record_index)
        .ok_or(Error::Invalid("GeneralRequiredSourceRoleMissing"))
}

impl Plan {
    fn vertex(
        &mut self,
        geometry: &Geometry<'_>,
        record: usize,
        map: &mut BTreeMap<usize, usize>,
    ) -> Result<usize> {
        if let Some(index) = map.get(&record) {
            return Ok(*index);
        }
        let point = geometry.position(geometry.reference(record, "vertex_geometry")?)?;
        let index = self.vertices.len();
        self.vertices.push(SourceVertex { record, point });
        map.insert(record, index);
        Ok(index)
    }
    fn edge(
        &mut self,
        geometry: &Geometry<'_>,
        record: usize,
        vertices: &mut BTreeMap<usize, usize>,
        edges: &mut BTreeMap<usize, usize>,
    ) -> Result<usize> {
        if let Some(index) = edges.get(&record) {
            return Ok(*index);
        }
        let same_sense = geometry.boolean(record, "same_sense")?;
        let first =
            geometry.reference(record, if same_sense { "edge_start" } else { "edge_end" })?;
        let second =
            geometry.reference(record, if same_sense { "edge_end" } else { "edge_start" })?;
        let vertices = [
            self.vertex(geometry, first, vertices)?,
            self.vertex(geometry, second, vertices)?,
        ];
        let index = self.edges.len();
        self.edges.push(SourceEdge {
            record,
            geometry: geometry.reference(record, "edge_geometry")?,
            vertices,
            same_sense,
            built: None,
        });
        edges.insert(record, index);
        Ok(index)
    }
    fn prepare(&mut self, geometry: &Geometry<'_>) -> Result<()> {
        need(
            self.source
                .part_definition()
                .is_some_and(|p| p.shape_role == "MaterialSolid"),
            "GeneralRequiresMaterialSolid",
            self.shape,
        )?;
        if !geometry.is(self.shape, "MANIFOLD_SOLID_BREP")
            || !geometry.is(self.shell, "CLOSED_SHELL")
        {
            return Err(unsupported("GeneralUnsupportedShape", self.shape));
        }
        // Required detached definitions cannot silently disappear while their
        // conversion/publication contract is still being migrated.
        if self.source.part_definition().is_some_and(|p| {
            p.roles.iter().any(|r| {
                r.name == "wireframe_representation" || r.name == "construction_representation"
            })
        }) {
            return Err(unsupported(
                "GeneralAssociatedGeometryNotYetConstructed",
                self.shape,
            ));
        }
        let mut vertices = BTreeMap::new();
        let mut edges = BTreeMap::new();
        for record in geometry.references(self.shell, "cfs_faces")? {
            let same_sense = geometry.boolean(record, "same_sense")?;
            let mut face = SourceFace {
                record,
                geometry: geometry.reference(record, "face_geometry")?,
                same_sense,
                loops: Vec::new(),
            };
            for bound in geometry.references(record, "bounds")? {
                let loop_record = geometry.reference(bound, "bound")?;
                let orientation = geometry.boolean(bound, "orientation")?;
                let forward = orientation == same_sense;
                let mut oriented = geometry.references(loop_record, "edge_list")?;
                if !forward {
                    oriented.reverse();
                }
                let mut uses = Vec::new();
                for use_record in oriented {
                    let edge_record = geometry.reference(use_record, "edge_element")?;
                    let edge = self.edge(geometry, edge_record, &mut vertices, &mut edges)?;
                    let curve = self
                        .edges
                        .get(edge)
                        .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
                    let along = geometry.boolean(use_record, "orientation")? == curve.same_sense;
                    uses.push(SourceUse {
                        edge,
                        record: use_record,
                        forward: if forward { along } else { !along },
                    });
                }
                need(!uses.is_empty(), "GeneralSourceLoopDisconnected", bound)?;
                for (index, a) in uses.iter().enumerate() {
                    let b = uses
                        .get((index + 1) % uses.len())
                        .ok_or(Error::Invalid("GeneralLoopUseIndex"))?;
                    let a_edge = self
                        .edges
                        .get(a.edge)
                        .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
                    let b_edge = self
                        .edges
                        .get(b.edge)
                        .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
                    need(
                        a_edge.vertices.get(usize::from(a.forward))
                            == b_edge.vertices.get(usize::from(!b.forward)),
                        "GeneralSourceLoopDisconnected",
                        bound,
                    )?;
                }
                face.loops.push(SourceLoop {
                    record: loop_record,
                    bound,
                    outer: geometry.is(bound, "FACE_OUTER_BOUND"),
                    orientation,
                    uses,
                });
            }
            need(!face.loops.is_empty(), "GeneralFaceBoundsMissing", record)?;
            face.loops.sort_by_key(|l| !l.outer);
            self.faces.push(face);
        }
        need(
            !self.faces.is_empty(),
            "GeneralShellFacesMissing",
            self.shell,
        )?;
        let mut incidence: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
        for face in &self.faces {
            for bound in &face.loops {
                for u in &bound.uses {
                    incidence
                        .entry(u.edge)
                        .or_default()
                        .push(u.forward == face.same_sense);
                }
            }
        }
        for (index, edge) in self.edges.iter().enumerate() {
            let uses = incidence
                .get(&index)
                .ok_or(Error::Invalid("GeneralEdgeIncidence"))?;
            need(
                uses.len() == 2 && uses.first() != uses.get(1),
                "GeneralSourceNonManifoldEdge",
                edge.record,
            )?;
        }
        Ok(())
    }
}

/// Construct defining general geometry using the complete typed source pipeline.
/// The result is an unpublished development candidate, never a planar or general
/// acceptance certificate. All required source content must be represented.
///
/// # Errors
/// Source failures retain their original diagnostics. Unsupported required
/// conversion, an exhausted native budget, or a kernel failure releases the
/// complete unpublished candidate without a partial model.
#[allow(clippy::too_many_lines)]
pub fn construct_general(
    raw: &solidcore_step::Outcome,
    semantic_limits: s::Limits,
    source_options: &s::Options,
    native: Options,
    quality: GeneralQuality,
) -> Result<Candidate> {
    if quality.id.is_empty()
        || quality.id.len() > 256
        || [
            quality.definition_mm,
            quality.consistency_mm,
            quality.derived_uv_mm,
            quality.query_mm,
            quality.maximum_work_tolerance_mm,
        ]
        .iter()
        .any(|q| *q <= Rational::ZERO)
        || quality.maximum_tolerance_growth_mm < Rational::ZERO
        || !native.work_tolerance_mm.is_finite()
        || native.work_tolerance_mm <= 0.
    {
        return Err(Error::Invalid("GeneralExplicitPositiveQualityRequired"));
    }
    if !crate::bounds::within(native.work_tolerance_mm, quality.maximum_work_tolerance_mm) {
        return Err(Error::Evidence(
            "GeneralInitialWorkingToleranceExceedsBudget",
        ));
    }
    let mut source_options = source_options.clone();
    source_options.geometry_scope = s::GeometryScope::GeneralBrep;
    source_options.check_geometry_dependencies = true;
    source_options.check_part_and_closure = true;
    let source_report = s::check(raw, semantic_limits, &source_options);
    let source = source_report
        .snapshot
        .clone()
        .ok_or_else(|| Error::Source {
            status: source_report.status,
            diagnostics: source_report.diagnostics.clone(),
        })?;
    let shape = role(&source, "shape")?;
    let shell = role(&source, "shell")?;
    let kernel = Kernel::default();
    let geometry = Geometry::new(
        &source,
        &kernel,
        scalar(quality.consistency_mm),
        scalar(quality.query_mm),
    )?;
    let mut plan = Plan {
        source: source.clone(),
        source_report,
        quality,
        shape,
        shell,
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        mappings: Vec::new(),
        observations: GeneralObservations::default(),
        normalizations: Vec::new(),
        loop_mappings: Vec::new(),
        unit_conversions: geometry.conversions().to_vec(),
        length_scale_mm: geometry.length_scale_exact(),
    };
    plan.prepare(&geometry)?;
    for face in &plan.faces {
        for bound in &face.loops {
            let mut uses = Vec::new();
            for u in &bound.uses {
                let edge = plan
                    .edges
                    .get(u.edge)
                    .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
                uses.push((u.record, edge.record, u.forward));
            }
            plan.loop_mappings.push(GeneralLoopMapping {
                face: face.record,
                bound: bound.bound,
                loop_record: bound.record,
                declared_outer: bound.outer,
                bound_orientation: bound.orientation,
                uses,
            });
        }
    }
    let mut execution = Execution {
        used: 0,
        max: native.operations,
    };
    execution.step()?;
    let mut builder = BrepConstructor::new()?;
    solidcore::topology::brep::p2::set_tolerance(
        &kernel,
        builder.brep_mut(),
        native.work_tolerance_mm,
        false,
        false,
    )?;
    for edge in &mut plan.edges {
        let a = plan
            .vertices
            .get(edge.vertices[0])
            .ok_or(Error::Invalid("GeneralVertexIndex"))?
            .point;
        let b = plan
            .vertices
            .get(edge.vertices[1])
            .ok_or(Error::Invalid("GeneralVertexIndex"))?
            .point;
        let curve = geometry.make_curve(
            edge.geometry,
            edge.record,
            a,
            b,
            edge.vertices[0] == edge.vertices[1],
            scalar(plan.quality.definition_mm),
            &mut execution,
        )?;
        plan.mappings.extend(curve.mappings.iter().cloned());
        plan.observations.maximum_vertex_gap_mm = plan
            .observations
            .maximum_vertex_gap_mm
            .max(curve.maximum_consistency_mm);
        edge.built = Some(curve);
    }
    let mut vertices: BTreeMap<usize, VertexId> = BTreeMap::new();
    for vertex in &plan.vertices {
        execution.step()?;
        let id = builder.start_vertex(vertex.point)?;
        builder.brep_mut().vertex_mut(id)?.tolerance = native.work_tolerance_mm;
        vertices.insert(vertex.record, id);
        execution.step()?;
        builder.end_vertex();
    }
    execution.step()?;
    let region = builder.start_region(true, None)?;
    builder.brep_mut().region_mut(region)?.is_void = 0;
    execution.step()?;
    builder.start_shell()?;
    let mut edges: BTreeMap<usize, EdgeId> = BTreeMap::new();
    let mut faces: BTreeMap<usize, FaceId> = BTreeMap::new();
    for face in &mut plan.faces {
        surface::prepare_loop_head(face, &plan.edges)?;
        execution.step()?;
        let id = builder.start_face(if face.same_sense {
            OrientType::Same
        } else {
            OrientType::Opposite
        })?;
        builder.brep_mut().face_mut(id)?.tolerance = native.work_tolerance_mm;
        let built = surface::make(
            &geometry,
            face,
            &plan.edges,
            &plan.vertices,
            &plan.quality,
            &mut execution,
        )?;
        plan.mappings.extend(built.mappings);
        let seam = built.seam;
        let mut surface = built.surface;
        surface.bind_evaluation_context(&kernel);
        // Borrowed natural-domain queries must see the original receiver's lazy
        // representation, not a default or a sliced B-spline stand-in.
        solidcore::surface::validity::materialize_bspline(&kernel, &mut surface)?;
        let domain = surface.natural_uv_domain();
        execution.step()?;
        builder.set_face_surface(surface, domain)?;
        faces.insert(face.record, id);
        if let Some(seam) = seam {
            need(
                geometry::norm(seam.endpoints[1] - seam.endpoints[0]) > native.work_tolerance_mm,
                "GeneralImplicitSeamHeight",
                face.record,
            )?;
            execution.step()?;
            builder.start_loop(OrientType::Same)?;
            let first = *face
                .loops
                .first()
                .and_then(|l| l.uses.first())
                .ok_or(Error::Invalid("GeneralSeamUse"))?;
            append_use(
                &mut builder,
                &kernel,
                &mut execution,
                first,
                &mut plan.edges,
                &plan.vertices,
                &vertices,
                &mut edges,
                native.work_tolerance_mm,
            )?;
            let derived = append_seam(
                &mut builder,
                &kernel,
                &mut execution,
                &seam,
                None,
                true,
                &plan.vertices,
                &vertices,
                native.work_tolerance_mm,
            )?;
            let second = *face
                .loops
                .get(1)
                .and_then(|l| l.uses.first())
                .ok_or(Error::Invalid("GeneralSeamUse"))?;
            append_use(
                &mut builder,
                &kernel,
                &mut execution,
                second,
                &mut plan.edges,
                &plan.vertices,
                &vertices,
                &mut edges,
                native.work_tolerance_mm,
            )?;
            append_seam(
                &mut builder,
                &kernel,
                &mut execution,
                &seam,
                Some(derived),
                false,
                &plan.vertices,
                &vertices,
                native.work_tolerance_mm,
            )?;
            execution.step()?;
            builder.end_loop()?;
            plan.normalizations.push((face.record, derived));
        } else {
            for bound in &face.loops {
                execution.step()?;
                builder.start_loop(OrientType::Same)?;
                for u in &bound.uses {
                    let edge = plan
                        .edges
                        .get_mut(u.edge)
                        .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
                    let va = plan
                        .vertices
                        .get(edge.vertices[0])
                        .ok_or(Error::Invalid("GeneralVertexIndex"))?
                        .record;
                    let vb = plan
                        .vertices
                        .get(edge.vertices[1])
                        .ok_or(Error::Invalid("GeneralVertexIndex"))?
                        .record;
                    let existing = edges.get(&edge.record).copied();
                    execution.step()?;
                    let target = builder.start_edge(
                        &kernel,
                        if u.forward {
                            OrientType::Same
                        } else {
                            OrientType::Opposite
                        },
                        existing,
                        vertices.get(&va).copied(),
                        vertices.get(&vb).copied(),
                    )?;
                    need(
                        existing.is_none_or(|e| e == target),
                        "GeneralSharedEdgeIdentity",
                        edge.record,
                    )?;
                    if existing.is_none() {
                        builder.brep_mut().edge_mut(target)?.tolerance = native.work_tolerance_mm;
                        let definition = edge
                            .built
                            .as_mut()
                            .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
                        execution.step()?;
                        builder.set_edge_curve(
                            &kernel,
                            definition
                                .native
                                .take()
                                .ok_or(Error::Invalid("GeneralCurveAlreadyTransferred"))?,
                            false,
                        )?;
                        execution.step()?;
                        builder.set_limits(
                            &kernel,
                            VertexNode::Parameter(definition.interval.min()),
                            VertexNode::Parameter(definition.interval.max()),
                        )?;
                        edges.insert(edge.record, target);
                    }
                    execution.step()?;
                    builder.end_edge()?;
                }
                execution.step()?;
                builder.end_loop()?;
            }
        }
        execution.step()?;
        builder.end_face()?;
    }
    execution.step()?;
    let mut kernel = kernel;
    builder.end_shell(&mut kernel, false)?;
    execution.step()?;
    builder.end_region()?;
    execution.step()?;
    let mut brep = builder.finish()?;
    for source_face in plan.faces.iter().map(|face| face.record) {
        let id = faces
            .get(&source_face)
            .ok_or(Error::Invalid("GeneralFaceCorrespondence"))?;
        execution.step()?;
        let gaps = fp3::create_uv_trim_curves(&kernel, &mut brep, *id, false, None)?;
        for gap in [gaps.max_curve_surface_gap, gaps.max_3d_vertex_gap] {
            need(
                gap.is_finite() && gap >= 0.,
                "GeneralUvObservationUnestablished",
                source_face,
            )?;
        }
        plan.observations.maximum_curve_surface_gap_mm = plan
            .observations
            .maximum_curve_surface_gap_mm
            .max(gaps.max_curve_surface_gap);
        plan.observations.maximum_vertex_gap_mm = plan
            .observations
            .maximum_vertex_gap_mm
            .max(gaps.max_3d_vertex_gap);
        need(
            gaps.max_curve_surface_gap <= scalar(plan.quality.derived_uv_mm),
            "GeneralDerivedUvGap",
            source_face,
        )?;
        need(
            gaps.max_3d_vertex_gap <= scalar(plan.quality.consistency_mm),
            "GeneralVertexGap",
            source_face,
        )?;
    }
    // Observe all actual owners, including derived chart edges. A source
    // uncertainty or a failed projection cannot enlarge the selected budgets.
    let mut tolerances = vec![brep.tolerance()];
    for id in brep.vertex_list() {
        tolerances.push(brep.vertex(*id)?.tolerance);
    }
    for id in brep.edge_list() {
        tolerances.push(brep.edge(*id)?.tolerance);
    }
    for id in brep.face_list() {
        tolerances.push(brep.face(*id)?.tolerance);
    }
    for tolerance in tolerances {
        if !crate::bounds::within(tolerance, plan.quality.maximum_work_tolerance_mm) {
            return Err(Error::Evidence("GeneralWorkingToleranceExceedsBudget"));
        }
        if tolerance > native.work_tolerance_mm {
            let growth = crate::bounds::norm_upper([
                crate::bounds::I::point(tolerance)
                    - crate::bounds::I::point(native.work_tolerance_mm),
                crate::bounds::I::point(0.),
                crate::bounds::I::point(0.),
            ]);
            if !crate::bounds::within(growth, plan.quality.maximum_tolerance_growth_mm) {
                return Err(Error::Evidence(
                    "GeneralWorkingToleranceGrowthExceedsBudget",
                ));
            }
        }
    }
    Ok(Candidate {
        brep,
        kernel,
        plan: SourcePlan::General(Box::new(plan)),
        vertices,
        edges,
        faces,
        operations: execution.used,
        operation_limit: execution.max,
        native_checks: None,
        initial_work_tolerance_mm: native.work_tolerance_mm,
    })
}

#[allow(clippy::too_many_arguments)]
fn append_use(
    builder: &mut BrepConstructor,
    kernel: &Kernel,
    execution: &mut Execution,
    u: SourceUse,
    definitions: &mut [SourceEdge],
    source_vertices: &[SourceVertex],
    vertices: &BTreeMap<usize, VertexId>,
    edges: &mut BTreeMap<usize, EdgeId>,
    tolerance: f64,
) -> Result<()> {
    let edge = definitions
        .get_mut(u.edge)
        .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
    let [a, b] = edge.vertices;
    let va = source_vertices
        .get(a)
        .ok_or(Error::Invalid("GeneralVertexIndex"))?
        .record;
    let vb = source_vertices
        .get(b)
        .ok_or(Error::Invalid("GeneralVertexIndex"))?
        .record;
    let existing = edges.get(&edge.record).copied();
    execution.step()?;
    let target = builder.start_edge(
        kernel,
        if u.forward {
            OrientType::Same
        } else {
            OrientType::Opposite
        },
        existing,
        vertices.get(&va).copied(),
        vertices.get(&vb).copied(),
    )?;
    need(
        existing.is_none_or(|e| e == target),
        "GeneralSharedEdgeIdentity",
        edge.record,
    )?;
    if existing.is_none() {
        builder.brep_mut().edge_mut(target)?.tolerance = tolerance;
        let definition = edge
            .built
            .as_mut()
            .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
        execution.step()?;
        builder.set_edge_curve(
            kernel,
            definition
                .native
                .take()
                .ok_or(Error::Invalid("GeneralCurveAlreadyTransferred"))?,
            false,
        )?;
        execution.step()?;
        builder.set_limits(
            kernel,
            VertexNode::Parameter(definition.interval.min()),
            VertexNode::Parameter(definition.interval.max()),
        )?;
        edges.insert(edge.record, target);
    }
    execution.step()?;
    builder.end_edge()?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn append_seam(
    builder: &mut BrepConstructor,
    kernel: &Kernel,
    execution: &mut Execution,
    seam: &surface::Seam,
    existing: Option<EdgeId>,
    forward: bool,
    source_vertices: &[SourceVertex],
    vertices: &BTreeMap<usize, VertexId>,
    tolerance: f64,
) -> Result<EdgeId> {
    let [a, b] = seam.vertices;
    let va = source_vertices
        .get(a)
        .ok_or(Error::Invalid("GeneralSeamVertex"))?
        .record;
    let vb = source_vertices
        .get(b)
        .ok_or(Error::Invalid("GeneralSeamVertex"))?
        .record;
    execution.step()?;
    let id = builder.start_edge(
        kernel,
        if forward {
            OrientType::Same
        } else {
            OrientType::Opposite
        },
        existing,
        vertices.get(&va).copied(),
        vertices.get(&vb).copied(),
    )?;
    need(
        existing.is_none_or(|e| e == id),
        "GeneralTargetSeamIdentity",
        va,
    )?;
    if existing.is_none() {
        builder.brep_mut().edge_mut(id)?.tolerance = tolerance;
        let [a, b] = seam.endpoints;
        execution.step()?;
        let curve = solidcore::curve::line::Line::from_points(a, b, 3);
        execution.step()?;
        builder.set_edge_curve(kernel, curve.into(), false)?;
        execution.step()?;
        builder.set_limits(kernel, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
    }
    execution.step()?;
    builder.end_edge()?;
    Ok(id)
}

fn edge_bounds(edge: &SourceEdge) -> Result<Extent3d> {
    edge.built
        .as_ref()
        .map(|c| c.bounds)
        .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))
}
