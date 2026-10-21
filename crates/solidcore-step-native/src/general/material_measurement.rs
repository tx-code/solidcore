// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Portions of finite_midpoint(): Copyright (c) Microsoft Corporation.
// SPDX-License-Identifier: Apache-2.0 AND (Apache-2.0 WITH LLVM-exception)
//! Complete numerical material observations of an unpublished general candidate.
//!
//! Upstream: maintained `StepImportGeneral.cpp::Measure`, SHA256
//! `4dd05633834a59f52665c5b350d18534b99423312cddef6474914bdd996532f3`,
//! and `StepImportGeneralMeasurements.cpp::IntegrateGeneralSolid`, SHA256
//! `a075cf6e0658ce880d250a0dda5b3176182dc91167c72c30147f78de3244ef4c`.
//! Deviations: no implicit numerical default; complete results are returned by
//! value, after finite-output and actual working-tolerance checks. No mesh,
//! geometry fitting, source rewriting, density override, or error bound is used.

use super::{
    failure, need, scalar, Candidate, Error, Execution, GeneralFailureKind, Plan, Result,
    SourcePlan,
};
use solidcore::geom::attribute::ids;
use solidcore::geom::extent3d::Extent3d;
use solidcore::mass::integrator::properties::{self as properties, PreciseProperties};
use solidcore::math::{Vec3, EFF_ZERO};
use solidcore::query::{brep as brep_query, face as face_query};
use solidcore::topology::brep::{p1 as brep_lists, p2 as brep_native};
use solidcore::topology::face::p2::{
    PPF_AREA, PPF_AREA_FIRST_MOMENTS, PPF_VOLUME, PPF_VOLUME_FIRST_MOMENTS,
};
use solidcore::topology::{topology_traverser, Brep, EdgeId, FaceId, RegionId};
use solidcore::EntityRef;
use std::collections::{BTreeMap, BTreeSet};

/// One source-associated actual native face's completed numerical measurements.
#[derive(Clone, Debug)]
pub struct GeneralMaterialFaceMeasurement {
    /// Original face record; this vector follows source face order.
    pub source_face: usize,
    /// Actual owning face identity.
    pub native_face: FaceId,
    /// Query position in canonical native face order, independently of source order.
    pub native_query_index: usize,
    /// Tight native box including trimmed interior extrema, in millimeters.
    pub bounds_mm: Extent3d,
    /// Absolute numerically integrated area, in square millimeters.
    pub area_mm2: f64,
    /// Local origin used directly for the area moments, in millimeters.
    pub area_origin_mm: Vec3,
    /// Signed area first moments about the local origin, in cubic millimeters.
    pub area_first_moment_mm3: Vec3,
    /// Numerical area centroid, in millimeters.
    pub centroid_mm: Vec3,
    /// Actual distinct native edges, including derived chart cuts.
    pub edges: usize,
    /// Actual distinct native vertices on this face.
    pub vertices: usize,
}

/// One actual target edge's length, including explicitly derived chart cuts.
#[derive(Clone, Debug)]
pub struct GeneralMaterialEdgeMeasurement {
    /// Original edge record, absent only for an explicitly derived chart cut.
    pub source_edge: Option<usize>,
    /// Original face producing a derived chart cut; absent on source edges.
    pub derived_from_face: Option<usize>,
    /// Actual owning edge identity.
    pub native_edge: EdgeId,
    /// Numerical length over the actual edge interval, in millimeters.
    pub length_mm: f64,
}

/// Complete numerical material checkpoint; presence never grants import acceptance.
#[derive(Clone, Debug)]
pub struct GeneralMaterialMeasurements {
    /// Explicit dimensionless numerical stopping control requested by the caller.
    pub requested_relative_accuracy: f64,
    /// Actual un-clamped control supplied to every completed face integral.
    pub applied_relative_accuracy: f64,
    /// Union of actual tight face boxes, in millimeters.
    pub bounds_mm: Extent3d,
    /// Common reference frame for geometric volume moments, in millimeters.
    pub volume_origin_mm: Vec3,
    /// Complete numerical area, in square millimeters.
    pub area_mm2: f64,
    /// Complete numerical material volume, in cubic millimeters.
    pub material_volume_mm3: f64,
    /// Numerical volume centroid, in millimeters.
    pub centroid_mm: Vec3,
    /// Sum of unique target-edge lengths, including chart cuts, in millimeters.
    /// This is the maintained `boundaryMm` statistic, not an exterior perimeter.
    pub boundary_mm: f64,
    /// Actual complete native vertex count.
    pub vertices: usize,
    /// Complete face results in source order.
    pub faces: Vec<GeneralMaterialFaceMeasurement>,
    /// Source defining edges first, then derived cuts, in original target order.
    pub edges: Vec<GeneralMaterialEdgeMeasurement>,
}

#[derive(Clone, Copy)]
struct FaceBinding {
    source: usize,
    native: FaceId,
}
struct EdgeBinding {
    source: Option<usize>,
    derived_face: Option<usize>,
    native: EdgeId,
}
impl EdgeBinding {
    fn owner(&self) -> Result<usize> {
        self.source
            .or(self.derived_face)
            .ok_or(Error::Invalid("GeneralMaterialEdgeCorrespondence"))
    }
}
struct Inventory {
    faces: Vec<FaceBinding>,
    native_faces: Vec<FaceId>,
    source_index: BTreeMap<FaceId, usize>,
    edges: Vec<EdgeBinding>,
    vertices: usize,
}

fn inventory(model: &Candidate, plan: &Plan) -> Result<Inventory> {
    plan.check_vertex_bounds(&model.brep, &model.vertices, &model.faces)?;
    let native_faces = brep_lists::faces(&model.brep, None)?;
    let native_edges = brep_lists::edges(&model.brep, None)?;
    let native_vertices = brep_lists::vertices(&model.brep, None)?;
    let mut faces = Vec::new();
    let mut source_index = BTreeMap::new();
    for source in &plan.faces {
        let native = *model
            .faces
            .get(&source.record)
            .ok_or(Error::Invalid("GeneralMaterialFaceCorrespondence"))?;
        if source_index.insert(native, faces.len()).is_some() {
            return Err(Error::Invalid("GeneralMaterialFaceCorrespondence"));
        }
        faces.push(FaceBinding {
            source: source.record,
            native,
        });
    }
    if model.faces.len() != faces.len()
        || native_faces.len() != faces.len()
        || native_faces.iter().any(|f| !source_index.contains_key(f))
    {
        return Err(Error::Invalid("GeneralMaterialFaceCorrespondence"));
    }
    let mut edges = Vec::new();
    let mut seen = BTreeSet::new();
    for source in &plan.edges {
        let pieces = model
            .edges
            .get(&source.record)
            .ok_or(Error::Invalid("GeneralMaterialEdgeCorrespondence"))?;
        if pieces.is_empty() {
            return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
        }
        let original = source
            .built
            .as_ref()
            .ok_or(Error::Invalid("GeneralMaterialEdgeCorrespondence"))?;
        let mut parameter = original.interval.min();
        for &native in pieces {
            let edge = model.brep.edge(native)?;
            // This inventory covers the complete original defining interval,
            // not just a subset with plausible aggregate area or volume. Exact
            // adjacent split parameters are retained by the constructor.
            if edge.interval.min() != parameter
                || edge.interval.max().partial_cmp(&parameter) != Some(std::cmp::Ordering::Greater)
            {
                return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
            }
            parameter = edge.interval.max();
            if !seen.insert(native) {
                return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
            }
            edges.push(EdgeBinding {
                source: Some(source.record),
                derived_face: None,
                native,
            });
        }
        if parameter != original.interval.max() {
            return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
        }
    }
    if model.edges.len() != plan.edges.len() {
        return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
    }
    for &(source_face, native) in &plan.normalizations {
        if !model.faces.contains_key(&source_face) || !seen.insert(native) {
            return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
        }
        edges.push(EdgeBinding {
            source: None,
            derived_face: Some(source_face),
            native,
        });
    }
    if native_edges.len() != edges.len() || native_edges.iter().any(|e| !seen.contains(e)) {
        return Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"));
    }
    let mut source_vertices: BTreeSet<_> = model.vertices.values().copied().collect();
    if source_vertices.len() != plan.vertices.len() || model.vertices.len() != plan.vertices.len() {
        return Err(Error::Invalid("GeneralMaterialVertexCorrespondence"));
    }
    for vertex in &plan.derived_vertices {
        let pieces = model
            .edges
            .get(&vertex.source_edge)
            .ok_or(Error::Invalid("GeneralMaterialVertexCorrespondence"))?;
        if !vertex.native_parameter.is_finite()
            || pieces.len() < 2
            || !source_vertices.insert(vertex.native_vertex)
        {
            return Err(Error::Invalid("GeneralMaterialVertexCorrespondence"));
        }
        let mut lower = false;
        let mut upper = false;
        for piece in pieces {
            let edge = model.brep.edge(*piece)?;
            lower |= edge.interval.max() == vertex.native_parameter;
            upper |= edge.interval.min() == vertex.native_parameter;
        }
        if !lower || !upper {
            return Err(Error::Invalid("GeneralMaterialVertexCorrespondence"));
        }
    }
    if native_vertices.len() != source_vertices.len()
        || native_vertices.iter().any(|v| !source_vertices.contains(v))
    {
        return Err(Error::Invalid("GeneralMaterialVertexCorrespondence"));
    }
    // Measure's tight-face union is valid only for the COMPLETE selected face
    // boundary. Free wire/point records cannot disappear from that statistic.
    let mut boundary_edges = BTreeSet::new();
    let mut boundary_vertices = BTreeSet::new();
    for face in &faces {
        boundary_edges.extend(topology_traverser::face_edges(&model.brep, face.native)?);
        boundary_vertices.extend(face_query::p2::face_vertices(
            &model.brep,
            face.native,
            false,
            None,
        )?);
    }
    if boundary_edges != seen || boundary_vertices != source_vertices {
        return Err(Error::Invalid("GeneralMaterialCompleteBoundary"));
    }
    Ok(Inventory {
        faces,
        native_faces,
        source_index,
        edges,
        vertices: native_vertices.len(),
    })
}

pub(super) fn validate_accuracy(accuracy: f64) -> Result<()> {
    // Fixed source: IntegrateGeneralSolid's first guard, SHA a075cf6e...4ef4c;
    // StepImport.h::NumericalIntegrationSettings; NumericalIntegrationContract.
    // This range is a numerical algorithm policy, not a production default,
    // source uncertainty, millimeter tolerance, or certified relative error.
    if !accuracy.is_finite() || !(1e-8..=1e-4).contains(&accuracy) {
        return Err(Error::Invalid("GeneralMaterialIntegrationAccuracy"));
    }
    Ok(())
}

fn finite_point(p: Vec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}
fn finite_box(bounds: &Extent3d, strict: bool) -> bool {
    let a = bounds.min();
    let b = bounds.max();
    finite_point(a)
        && finite_point(b)
        && if strict {
            b.x > a.x && b.y > a.y && b.z > a.z
        } else {
            b.x >= a.x && b.y >= a.y && b.z >= a.z
        }
}
fn union(a: &Extent3d, b: &Extent3d) -> Extent3d {
    let lo = a.min();
    let hi = a.max();
    let other_lo = b.min();
    let other_hi = b.max();
    // std::min/max keep their first operand on equality (including signed zero).
    let min = |a, b| if b < a { b } else { a };
    let max = |a, b| if a < b { b } else { a };
    Extent3d::from_bounds(
        min(lo.x, other_lo.x),
        min(lo.y, other_lo.y),
        min(lo.z, other_lo.z),
        max(hi.x, other_hi.x),
        max(hi.y, other_hi.y),
        max(hi.z, other_hi.z),
    )
}

// MSVC 14.50.35717 include/numeric::midpoint finite-double runtime path,
// 745--801. Only the model VOLUME origin uses this; the old face-local area
// origin remains SmExtent3d::GetMid, preserving the source query's numerics.
// License: LICENSES/Microsoft-STL.txt; see ATTRIBUTION.md.
fn finite_midpoint(a: f64, b: f64) -> f64 {
    let high = f64::MAX / 2.;
    let aa = a.abs();
    let ba = b.abs();
    if aa <= high && ba <= high {
        return (a + b) / 2.;
    }
    let low = f64::MIN_POSITIVE * 2.;
    if aa < low {
        return a + b / 2.;
    }
    if ba < low {
        return a / 2. + b;
    }
    a / 2. + b / 2.
}
fn area_estimate(bounds: &Extent3d) -> f64 {
    let size = bounds.size();
    let mut largest = size.x;
    if largest < size.y {
        largest = size.y;
    }
    if largest < size.z {
        largest = size.z;
    }
    largest * largest / 100.
}

fn observe_working_tolerances(model: &Candidate, plan: &Plan) -> Result<()> {
    let check = |value: f64, owner: usize| -> Result<()> {
        need(
            value > 0.
                && value.is_finite()
                && crate::bounds::within(value, plan.quality.maximum_work_tolerance_mm),
            "GeneralUnexpectedToleranceGrowth",
            owner,
        )?;
        if value > model.initial_work_tolerance_mm {
            let growth = crate::bounds::norm_upper([
                crate::bounds::I::point(value)
                    - crate::bounds::I::point(model.initial_work_tolerance_mm),
                crate::bounds::I::point(0.),
                crate::bounds::I::point(0.),
            ]);
            need(
                crate::bounds::within(growth, plan.quality.maximum_tolerance_growth_mm),
                "GeneralUnexpectedToleranceGrowth",
                owner,
            )?;
        }
        Ok(())
    };
    check(model.brep.tolerance(), plan.shape)?;
    for id in model.brep.face_list() {
        check(model.brep.face(*id)?.tolerance, plan.shape)?;
    }
    for id in model.brep.edge_list() {
        check(model.brep.edge(*id)?.tolerance, plan.shape)?;
    }
    for id in model.brep.vertex_list() {
        check(model.brep.vertex(*id)?.tolerance, plan.shape)?;
    }
    Ok(())
}

struct FaceIntegral {
    area: f64,
    first: Vec3,
    origin: Vec3,
    native_query_index: usize,
}
struct Integrals {
    area: f64,
    volume: f64,
    centroid: Vec3,
    faces: Vec<FaceIntegral>,
}
fn material_orientation(
    b: &Brep,
    face: FaceId,
    material: RegionId,
    owner: usize,
) -> Result<solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType> {
    let up = b
        .face(face)?
        .faceuse
        .ok_or(Error::Invalid("GeneralMaterialFaceuse"))?;
    let down = b
        .faceuse(up)?
        .mate
        .ok_or(Error::Invalid("GeneralMaterialFaceuse"))?;
    let region = |fu| -> Result<RegionId> {
        let shell = b
            .faceuse(fu)?
            .shell
            .ok_or(Error::Invalid("GeneralMaterialFaceuse"))?;
        b.shell(shell)?
            .region
            .ok_or(Error::Invalid("GeneralMaterialFaceuse"))
    };
    let up_region = region(up)?;
    let down_region = region(down)?;
    let (selected, opposite) = if up_region == material {
        (up, down_region)
    } else if down_region == material {
        (down, up_region)
    } else {
        return Err(failure(
            "GeneralMaterialRegion",
            owner,
            GeneralFailureKind::Postcheck,
        ));
    };
    need(
        b.region(material)?.is_void() == 0 && b.region(opposite)?.is_void() != 0,
        "GeneralMaterialRegion",
        owner,
    )?;
    Ok(b.faceuse(selected)?.orientation)
}

fn integrate(
    model: &Candidate,
    plan: &Plan,
    inventory: &Inventory,
    origin: Vec3,
    accuracy: f64,
    execution: &mut Execution,
) -> Result<Integrals> {
    let material = plan
        .material_region
        .ok_or(Error::Invalid("GeneralMaterialRegion"))?;
    need(
        brep_native::is_manifold_solid(&model.brep)?,
        "GeneralMaterialManifoldSolid",
        plan.shape,
    )?;
    for entity in brep_lists::faces(&model.brep, None)?
        .into_iter()
        .map(EntityRef::Face)
        .chain(
            brep_lists::edges(&model.brep, None)?
                .into_iter()
                .map(EntityRef::Edge),
        )
        .chain(
            brep_lists::vertices(&model.brep, None)?
                .into_iter()
                .map(EntityRef::Vertex),
        )
    {
        let key = brep_native::object_ref(&model.brep, entity)?.key;
        need(
            model
                .brep
                .notifications()
                .attributes
                .find_attribute(key, ids::MASS_PROPERTIES)
                .is_none(),
            "GeneralMaterialMassAttributes",
            plan.shape,
        )?;
    }
    // These ordinary boxes supply only the native integral's area scale and
    // legacy local area origin. They NEVER replace the tight output bounds or
    // its centered model volume origin. This is IntegrateGeneralSolid's body.
    let ordinary = brep_query::calculate_bounding_box(
        &model.kernel,
        &model.brep,
        brep_query::BoundingBoxOptions::default(),
    )?;
    let model_estimate = area_estimate(&ordinary);
    let groups = PPF_AREA | PPF_AREA_FIRST_MOMENTS | PPF_VOLUME | PPF_VOLUME_FIRST_MOMENTS;
    let mut area = 0.;
    let mut volume = 0.;
    let mut volume_first = Vec3::ZERO;
    let mut by_source: Vec<Option<FaceIntegral>> = inventory.faces.iter().map(|_| None).collect();
    for (native_query_index, &face) in inventory.native_faces.iter().enumerate() {
        let index = *inventory
            .source_index
            .get(&face)
            .ok_or(Error::Invalid("GeneralMaterialFaceCorrespondence"))?;
        let binding = inventory
            .faces
            .get(index)
            .ok_or(Error::Invalid("GeneralMaterialFaceCorrespondence"))?;
        // beforeFace(sourceIndex) charges immediately before the fused native
        // face query, in canonical native order, without sorting source records.
        execution.step()?;
        let orientation = material_orientation(&model.brep, face, material, binding.source)?;
        let ordinary =
            face_query::p1::calculate_bounding_box(&model.kernel, &model.brep, face, false)?;
        let area_origin = ordinary.mid();
        let face_estimate = area_estimate(&ordinary);
        let estimate = if face_estimate < model_estimate {
            face_estimate
        } else {
            model_estimate
        };
        let PreciseProperties {
            area: face_area,
            volume: face_volume,
            moments,
            ..
        } = properties::face_properties_with_origins(
            &model.kernel,
            &model.brep,
            face,
            orientation,
            accuracy,
            origin,
            area_origin,
            estimate,
            0.,
            groups,
        )?;
        area += face_area;
        volume += face_volume;
        let first = *moments
            .get(4)
            .ok_or(Error::Invalid("GeneralMaterialMomentLayout"))?;
        volume_first += first;
        *by_source
            .get_mut(index)
            .ok_or(Error::Invalid("GeneralMaterialFaceCorrespondence"))? = Some(FaceIntegral {
            area: face_area,
            first: *moments
                .first()
                .ok_or(Error::Invalid("GeneralMaterialMomentLayout"))?,
            origin: area_origin,
            native_query_index,
        });
    }
    need(
        volume.is_finite() && volume >= EFF_ZERO,
        "GeneralSolidMeasures",
        plan.shape,
    )?;
    let centroid = origin + volume_first / volume;
    let faces = by_source
        .into_iter()
        .map(|value| value.ok_or(Error::Invalid("GeneralMaterialFaceCorrespondence")))
        .collect::<Result<Vec<_>>>()?;
    Ok(Integrals {
        area,
        volume,
        centroid,
        faces,
    })
}

pub(super) fn measure(
    model: &mut Candidate,
    accuracy: f64,
    execution: &mut Execution,
) -> Result<GeneralMaterialMeasurements> {
    let SourcePlan::General(plan) = &model.plan else {
        return Err(Error::Invalid("GeneralMaterialRequiresGeneralCandidate"));
    };
    let inventory = inventory(model, plan)?;
    observe_working_tolerances(model, plan)?;
    let mut face_bounds = Vec::new();
    let mut bounds = None;
    for binding in &inventory.faces {
        execution.step()?;
        let face_box = face_query::p3::calculate_tight_bounding_box_in(
            &model.kernel,
            &mut model.brep,
            binding.native,
        )?;
        need(
            finite_box(&face_box, false),
            "GeneralFiniteBounds",
            binding.source,
        )?;
        bounds = Some(
            bounds
                .as_ref()
                .map_or(face_box, |previous| union(previous, &face_box)),
        );
        face_bounds.push(face_box);
    }
    let bounds = bounds.ok_or_else(|| {
        failure(
            "GeneralFiniteBounds",
            plan.shape,
            GeneralFailureKind::Postcheck,
        )
    })?;
    need(finite_box(&bounds, true), "GeneralFiniteBounds", plan.shape)?;
    execution.step()?;
    let lo = bounds.min();
    let hi = bounds.max();
    let volume_origin = Vec3::new(
        finite_midpoint(lo.x, hi.x),
        finite_midpoint(lo.y, hi.y),
        finite_midpoint(lo.z, hi.z),
    );
    let integrals = integrate(model, plan, &inventory, volume_origin, accuracy, execution)?;
    need(
        integrals.area.is_finite()
            && integrals.area > 0.
            && integrals.volume.is_finite()
            && integrals.volume > 0.,
        "GeneralSolidMeasures",
        plan.shape,
    )?;
    need(
        finite_point(integrals.centroid),
        "GeneralFiniteCentroid",
        plan.shape,
    )?;
    let mut edges = Vec::new();
    let mut boundary = 0.;
    for edge in &inventory.edges {
        execution.step()?;
        let length = properties::edge_length(
            &model.kernel,
            &model.brep,
            edge.native,
            scalar(plan.quality.query_mm),
        )?;
        need(
            length.is_finite() && length > 0.,
            "GeneralEdgeLength",
            edge.owner()?,
        )?;
        boundary += length;
        edges.push(GeneralMaterialEdgeMeasurement {
            source_edge: edge.source,
            derived_from_face: edge.derived_face,
            native_edge: edge.native,
            length_mm: length,
        });
    }
    need(
        boundary.is_finite() && boundary > 0.,
        "GeneralEdgeLength",
        plan.shape,
    )?;
    let mut faces = Vec::new();
    for ((binding, integral), bounds) in
        inventory.faces.iter().zip(integrals.faces).zip(face_bounds)
    {
        need(
            integral.area.is_finite() && integral.area.abs() >= EFF_ZERO,
            "GeneralSolidMeasures",
            binding.source,
        )?;
        let centroid = integral.origin + integral.first / integral.area;
        need(
            finite_point(integral.first) && finite_point(centroid),
            "GeneralFiniteCentroid",
            binding.source,
        )?;
        faces.push(GeneralMaterialFaceMeasurement {
            source_face: binding.source,
            native_face: binding.native,
            native_query_index: integral.native_query_index,
            bounds_mm: bounds,
            area_mm2: integral.area.abs(),
            area_origin_mm: integral.origin,
            area_first_moment_mm3: integral.first,
            centroid_mm: centroid,
            edges: topology_traverser::face_edges(&model.brep, binding.native)?.len(),
            vertices: face_query::p2::face_vertices(&model.brep, binding.native, false, None)?
                .len(),
        });
    }
    observe_working_tolerances(model, plan)?;
    Ok(GeneralMaterialMeasurements {
        requested_relative_accuracy: accuracy,
        applied_relative_accuracy: accuracy,
        bounds_mm: bounds,
        volume_origin_mm: volume_origin,
        area_mm2: integrals.area,
        material_volume_mm3: integrals.volume,
        centroid_mm: integrals.centroid,
        boundary_mm: boundary,
        vertices: inventory.vertices,
        faces,
        edges,
    })
}

#[cfg(test)]
mod tests {
    use super::super::{construct_general, GeneralQuality};
    use super::*;
    use crate::Options;
    use solidcore::geom::attribute::{Attribute, Behavior};
    use solidcore::math::{Vec2, PI};
    use solidcore_step::{planar::Rational, semantics as s};

    const BOX: &[u8] = include_bytes!("../../../solidcore-step/tests/fixtures/material-box.step");
    const CYLINDER: &[u8] = include_bytes!("../../tests/fixtures/cylinder.step");

    fn candidate(bytes: &[u8], operations: usize) -> Result<Candidate> {
        let raw =
            solidcore_step::inspect(bytes.to_vec(), solidcore_step::Limits::planar_fixtures());
        construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations,
                work_tolerance_mm: 1e-6,
            },
            GeneralQuality {
                id: "Q-General-material-public-controls".into(),
                definition_mm: Rational::decimal("0.0000001")?,
                consistency_mm: Rational::decimal("0.000001")?,
                derived_uv_mm: Rational::decimal("0.000001")?,
                query_mm: Rational::decimal("0.000001")?,
                maximum_work_tolerance_mm: Rational::decimal("0.00001")?,
                maximum_tolerance_growth_mm: Rational::ZERO,
            },
        )
    }
    fn ready(bytes: &[u8], operations: usize) -> Result<Candidate> {
        candidate(bytes, operations)?
            .validate_native()?
            .measure_general_trims()
    }
    fn close(actual: f64, expected: f64, screen: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= screen,
            "actual={actual}, independent={expected}, screen={screen}"
        );
    }

    #[test]
    fn public_box_material_checkpoint_keeps_source_results_and_native_integration_order() {
        let mut model = ready(BOX, 100_000).unwrap();
        assert!(model
            .general_report()
            .unwrap()
            .material_measurements
            .is_none());
        // A legal native list reordering changes accumulation order, not source
        // definitions. Revalidate and remeasure trims before the new checkpoint.
        let mut native_order = model.brep.face_list().to_vec();
        native_order.reverse();
        model.brep.set_face_list(native_order.clone()).unwrap();
        model = model
            .validate_native()
            .unwrap()
            .measure_general_trims()
            .unwrap();
        let before = model.operations();
        let source_order = if let SourcePlan::General(plan) = &model.plan {
            plan.faces
                .iter()
                .map(|face| face.record)
                .collect::<Vec<_>>()
        } else {
            panic!("general box")
        };
        let measured = model.measure_general_material(1e-7).unwrap();
        let report = measured
            .general_report()
            .unwrap()
            .material_measurements
            .unwrap();
        close(report.area_mm2, 2200., 1e-6);
        close(report.material_volume_mm3, 6000., 1e-5);
        close(report.boundary_mm, 240., 1e-9);
        assert!((report.centroid_mm - Vec3::new(7., 7., 20.)).length() < 1e-7);
        assert_eq!(report.bounds_mm.min(), Vec3::new(2., -3., 5.));
        assert_eq!(report.bounds_mm.max(), Vec3::new(12., 17., 35.));
        assert_eq!(report.volume_origin_mm, Vec3::new(7., 7., 20.));
        assert_eq!(report.requested_relative_accuracy, 1e-7);
        assert_eq!(report.applied_relative_accuracy, 1e-7);
        assert_eq!(report.faces.len(), 6);
        assert_eq!(report.edges.len(), 12);
        assert_eq!(report.vertices, 8);
        assert_eq!(measured.operations(), before + 2 * 6 + 12 + 1 + 24);
        for (face, source) in report.faces.iter().zip(source_order) {
            assert_eq!(face.source_face, source);
            assert_eq!(native_order[face.native_query_index], face.native_face);
            assert_eq!(face.edges, 4);
            assert_eq!(face.vertices, 4);
            assert!(face.area_mm2 > 0. && finite_point(face.centroid_mm));
        }
        assert_eq!(measured.source().source().bytes(), BOX);
        assert!(measured
            .general_report()
            .unwrap()
            .final_trim_measurements
            .is_some());
    }

    #[test]
    fn material_checkpoint_requires_both_prior_gates_and_exact_n_n_minus_one_budget() {
        assert!(matches!(
            candidate(BOX, 100_000)
                .unwrap()
                .measure_general_material(1e-7),
            Err(Error::Invalid("GeneralMaterialRequiresNativeChecks"))
        ));
        assert!(matches!(
            candidate(BOX, 100_000)
                .unwrap()
                .validate_native()
                .unwrap()
                .measure_general_material(1e-7),
            Err(Error::Invalid("GeneralMaterialRequiresFinalTrims"))
        ));
        let measured = ready(BOX, 100_000)
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        let exact = ready(BOX, measured.operations())
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        assert_eq!(exact.operations(), measured.operations());
        let short = ready(BOX, measured.operations() - 1).unwrap();
        assert!(matches!(
            short.measure_general_material(1e-7),
            Err(Error::ResourceLimit)
        ));
        // A failed consuming checkpoint cannot alter another owning result.
        close(
            measured
                .general_report()
                .unwrap()
                .material_measurements
                .unwrap()
                .material_volume_mm3,
            6000.,
            1e-5,
        );
    }

    #[test]
    fn invalid_numerical_requests_cannot_select_a_clamped_or_budget_derived_default() {
        for accuracy in [0., -1., 1e-9, 1e-3, f64::INFINITY, f64::NAN] {
            assert!(matches!(
                candidate(BOX, 100_000)
                    .unwrap()
                    .measure_general_material(accuracy),
                Err(Error::Invalid("GeneralMaterialIntegrationAccuracy"))
            ));
        }
        for accuracy in [1e-8, 1e-4] {
            validate_accuracy(accuracy).unwrap();
        }
    }

    #[test]
    fn later_owning_uv_query_invalidates_trim_and_material_receipts_together() {
        let mut measured = ready(BOX, 100_000)
            .unwrap()
            .measure_general_material(1e-7)
            .unwrap();
        let face = *measured.faces.values().next().unwrap();
        measured.classify_face_uv(face, Vec2::ZERO).unwrap();
        let report = measured.general_report().unwrap();
        assert!(report.final_trim_measurements.is_none());
        assert!(report.material_measurements.is_none());
        assert!(matches!(
            measured.measure_general_material(1e-7),
            Err(Error::Invalid("GeneralMaterialRequiresFinalTrims"))
        ));
    }

    #[test]
    fn weighted_models_and_post_validation_working_growth_are_refused() {
        let mut weighted = ready(BOX, 100_000).unwrap();
        let face = *weighted.faces.values().next().unwrap();
        let key = brep_native::object_ref(&weighted.brep, EntityRef::Face(face))
            .unwrap()
            .key;
        let store = &mut weighted.brep.notifications_mut().attributes;
        let attribute = store
            .insert(Attribute::vector3d(
                ids::MASS_PROPERTIES,
                Vec3::new(2., 0., 0.),
                Behavior::Copy,
            ))
            .unwrap();
        store.add_attribute(key, attribute).unwrap();
        assert!(matches!(
            weighted.measure_general_material(1e-7),
            Err(Error::General {
                key: "GeneralMaterialMassAttributes",
                ..
            })
        ));
        let mut grown = ready(BOX, 100_000).unwrap();
        let face = *grown.faces.values().next().unwrap();
        grown.brep.face_mut(face).unwrap().tolerance = 2e-6;
        assert!(matches!(
            grown.measure_general_material(1e-7),
            Err(Error::General {
                key: "GeneralUnexpectedToleranceGrowth",
                ..
            })
        ));
    }

    #[test]
    fn incomplete_source_edge_correspondence_cannot_measure_a_subset() {
        let mut model = ready(BOX, 100_000).unwrap();
        let source = *model.edges.keys().next().unwrap();
        assert!(model.edges.remove(&source).is_some());
        assert!(matches!(
            model.measure_general_material(1e-7),
            Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"))
        ));
    }

    #[test]
    fn a_shortened_native_edge_cannot_claim_complete_original_source_correspondence() {
        let mut model = ready(BOX, 100_000).unwrap();
        let native = model.edges.values().next().unwrap()[0];
        let interval = model.brep.edge(native).unwrap().interval;
        model.brep.edge_mut(native).unwrap().interval =
            solidcore::geom::extent1d::Extent1d::from_min_max(
                interval.min(),
                interval.evaluate(0.5),
            );
        assert!(matches!(
            model.measure_general_material(1e-7),
            Err(Error::Invalid("GeneralMaterialEdgeCorrespondence"))
        ));
    }

    #[test]
    fn the_retained_material_region_is_checked_on_every_face_not_inferred_from_face_count() {
        let mut model = ready(BOX, 100_000).unwrap();
        let region = if let SourcePlan::General(plan) = &model.plan {
            plan.material_region.unwrap()
        } else {
            panic!("general box")
        };
        model.brep.region_mut(region).unwrap().is_void = 1;
        assert!(matches!(
            model.measure_general_material(1e-7),
            Err(Error::General {
                key: "GeneralMaterialRegion",
                ..
            })
        ));
    }

    fn annular_cylinder() -> Vec<u8> {
        // Public tube R=3,r=2,h=7, authored from the public cylinder with an
        // explicit inner boundary and an explicit seam on each cylinder. The
        // source role, units and original outer definitions are unchanged.
        let mut text = std::str::from_utf8(CYLINDER).unwrap().to_owned();
        text = text.replace(
            "#22=CLOSED_SHELL('',(#180,#181,#182));",
            "#22=CLOSED_SHELL('',(#180,#181,#182,#282));",
        );
        text = text.replace(
            "#180=ADVANCED_FACE('',(#170),#140,.F.);",
            "#180=ADVANCED_FACE('',(#170,#270),#140,.F.);",
        );
        text = text.replace(
            "#181=ADVANCED_FACE('',(#171),#141,.T.);",
            "#181=ADVANCED_FACE('',(#171,#271),#141,.T.);",
        );
        let inner="\n#202=CARTESIAN_POINT('',(0.,2.,0.));\n#203=CARTESIAN_POINT('',(0.,2.,7.));\n#210=CIRCLE('',#106,2.);\n#211=CIRCLE('',#107,2.);\n#212=LINE('',#202,#113);\n#220=VERTEX_POINT('',#202);\n#221=VERTEX_POINT('',#203);\n#230=EDGE_CURVE('',#220,#220,#210,.T.);\n#231=EDGE_CURVE('',#221,#221,#211,.T.);\n#232=EDGE_CURVE('',#220,#221,#212,.T.);\n#242=CYLINDRICAL_SURFACE('',#106,2.);\n#250=ORIENTED_EDGE('',*,*,#230,.T.);\n#251=ORIENTED_EDGE('',*,*,#231,.F.);\n#252=ORIENTED_EDGE('',*,*,#230,.F.);\n#253=ORIENTED_EDGE('',*,*,#232,.T.);\n#254=ORIENTED_EDGE('',*,*,#231,.T.);\n#255=ORIENTED_EDGE('',*,*,#232,.F.);\n#260=EDGE_LOOP('',(#250));\n#261=EDGE_LOOP('',(#251));\n#262=EDGE_LOOP('',(#252,#253,#254,#255));\n#270=FACE_BOUND('',#260,.T.);\n#271=FACE_BOUND('',#261,.T.);\n#272=FACE_OUTER_BOUND('',#262,.T.);\n#282=ADVANCED_FACE('',(#272),#242,.F.);\n";
        let end = text.rfind("ENDSEC;").unwrap();
        text.insert_str(end, inner);
        text.into_bytes()
    }

    #[test]
    fn public_cylinder_and_annular_material_follow_independent_area_volume_and_centroid() {
        let annular = annular_cylinder();
        for (bytes, inner_radius, face_count, edge_count, vertex_count) in
            [(CYLINDER, 0., 3, 3, 2), (annular.as_slice(), 2., 4, 6, 4)]
        {
            let before = ready(bytes, 100_000).unwrap();
            let operations = before.operations();
            let measured = before.measure_general_material(1e-7).unwrap();
            let observation = measured
                .general_report()
                .unwrap()
                .material_measurements
                .unwrap();
            let r = 3.;
            let h = 7.;
            let area =
                2. * PI * (r * r - inner_radius * inner_radius) + 2. * PI * h * (r + inner_radius);
            let volume = PI * h * (r * r - inner_radius * inner_radius);
            // Same independent finite-accuracy comparison screen used by the
            // maintained general-development numerical volume controls. It is
            // separate from both the requested stopping control and mm budgets.
            close(observation.area_mm2, area, area * 2e-4);
            close(observation.material_volume_mm3, volume, volume * 2e-4);
            assert!((observation.centroid_mm - Vec3::new(0., 0., h / 2.)).length() < h * 2e-4);
            let seams = if inner_radius == 0. { 1. } else { 2. };
            close(
                observation.boundary_mm,
                4. * PI * (r + inner_radius) + seams * h,
                1e-5,
            );
            assert_eq!(observation.faces.len(), face_count);
            assert_eq!(observation.edges.len(), edge_count);
            assert_eq!(observation.vertices, vertex_count);
            assert_eq!(
                measured.operations(),
                operations
                    + 2 * face_count
                    + edge_count
                    + 1
                    + if inner_radius == 0. { 6 } else { 12 }
            );
            assert_eq!(measured.source().source().bytes(), bytes);
        }
    }

    #[test]
    fn finite_model_midpoint_preserves_native_runtime_branches_without_overflow() {
        assert_eq!(finite_midpoint(2., 12.), 7.);
        assert_eq!(finite_midpoint(-3., 17.), 7.);
        assert_eq!(finite_midpoint(f64::MAX, f64::MAX), f64::MAX);
        assert_eq!(finite_midpoint(-f64::MAX, f64::MAX), 0.);
        assert_eq!(finite_midpoint(f64::from_bits(1), f64::MAX), f64::MAX / 2.);
    }
}
