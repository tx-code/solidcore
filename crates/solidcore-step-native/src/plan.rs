// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Owning exact source interpretation shared by construction and certificates.
//!
//! Upstream: maintained `source/STEP_IMPORT/src/StepImportPlan.cpp::MakePlan`
//! at `d8ed45a`; source units and LINE projections follow `STEP_IMPORT/README.md`.
//! Deviations: exact definitions are prepared at the original construction
//! checkpoints so source arithmetic failures keep their order relative to native
//! operation limits. The completed candidate retains those same definitions.

use super::{Error, Result};
use solidcore_step::{planar as p, semantics as s};
use std::collections::BTreeMap;

pub(super) type ExactPoint = [p::Rational; 3];

/// The increasing native [0,1] interval maps to these exact source LINE points.
/// Endpoint identities follow `EDGE_CURVE.same_sense`, independently of each use.
#[derive(Clone, Copy)]
pub(super) struct DefiningEdge {
    pub edge: usize,
    pub same_sense: bool,
    pub minimum_vertex: usize,
    pub maximum_vertex: usize,
    pub minimum_mm: ExactPoint,
    pub maximum_mm: ExactPoint,
}

/// One source owner and one exact interpretation; no native handles or acceptance.
pub(super) struct Plan {
    source: s::Snapshot,
    // The optional source owner is transferred here into `source`, rather than
    // retaining another snapshot handle inside the diagnostic evidence.
    evidence: p::Evidence,
    scale: p::Rational,
    closed: bool,
    material: bool,
    vertices: BTreeMap<usize, ExactPoint>,
    edges: BTreeMap<usize, DefiningEdge>,
}

impl Plan {
    pub fn new(checked: &p::Outcome, work_tolerance_mm: f64) -> Result<Self> {
        if checked.status != p::Status::Checked {
            return Err(Error::Upstream);
        }
        let checked_evidence = checked.checked_evidence().ok_or(Error::Upstream)?;
        let source = checked_evidence.source.as_ref().ok_or(Error::Upstream)?;
        let role = source
            .part_definition()
            .ok_or(Error::Upstream)?
            .shape_role
            .as_str();
        let closed = matches!(role, "MaterialSolid" | "ClosedSurfaceModel");
        let material = role == "MaterialSolid";
        let faces = &checked_evidence.faces;
        if (!closed && (role != "OpenSurfaceModel" || !(1..=2).contains(&faces.len())))
            || (closed && faces.len() != 6)
        {
            return Err(Error::Unsupported(
                "planar construction requires one/two open faces or a six-face closed boundary",
            ));
        }
        if !work_tolerance_mm.is_finite() || work_tolerance_mm <= 0. {
            return Err(Error::Invalid("positive finite working tolerance required"));
        }
        let scale = faces
            .first()
            .ok_or(Error::Invalid("missing source face"))?
            .source_length_scale_mm;
        if faces.iter().any(|f| f.source_length_scale_mm != scale) {
            return Err(Error::Invalid("inconsistent selected-part unit context"));
        }
        let mut evidence = checked_evidence.clone();
        let source = evidence.source.take().ok_or(Error::Upstream)?;
        Ok(Self {
            source,
            evidence,
            scale,
            closed,
            material,
            vertices: BTreeMap::new(),
            edges: BTreeMap::new(),
        })
    }

    pub fn source(&self) -> &s::Snapshot {
        &self.source
    }

    pub const fn evidence(&self) -> &p::Evidence {
        &self.evidence
    }

    pub const fn closed(&self) -> bool {
        self.closed
    }

    pub const fn material(&self) -> bool {
        self.material
    }

    /// Prepare immediately before the original native vertex checkpoint. Source
    /// length is scaled exactly once; ratios and LINE parameters stay unitless.
    pub fn prepare_vertex(&mut self, record: usize) -> Result<ExactPoint> {
        let exact = point(&self.source, record, self.scale)?;
        self.vertices.insert(record, exact);
        Ok(exact)
    }

    pub fn vertex(&self, record: usize) -> Result<ExactPoint> {
        self.vertices
            .get(&record)
            .copied()
            .ok_or(Error::Invalid("required edge endpoint is unmapped"))
    }

    /// Retain the defining LINE, including a nonzero source endpoint residual;
    /// projecting onto it is equivalent representation normalization, not repair.
    pub fn prepare_edge(&mut self, index: usize) -> Result<DefiningEdge> {
        let edge = self
            .evidence
            .edges
            .get(index)
            .ok_or(Error::Invalid("source edge index"))?;
        let minimum_vertex = if edge.same_sense {
            edge.start
        } else {
            edge.end
        };
        let maximum_vertex = if edge.same_sense {
            edge.end
        } else {
            edge.start
        };
        let minimum_mm =
            projected_line_end(&self.source, edge, self.vertex(minimum_vertex)?, self.scale)?;
        let maximum_mm =
            projected_line_end(&self.source, edge, self.vertex(maximum_vertex)?, self.scale)?;
        let definition = DefiningEdge {
            edge: edge.edge,
            same_sense: edge.same_sense,
            minimum_vertex,
            maximum_vertex,
            minimum_mm,
            maximum_mm,
        };
        self.edges.insert(edge.edge, definition);
        Ok(definition)
    }

    pub fn edge(&self, record: usize) -> Result<&DefiningEdge> {
        self.edges
            .get(&record)
            .ok_or(Error::Invalid("required defining edge"))
    }
}

fn field(source: &s::Snapshot, record: usize, name: &str) -> Result<s::FieldView> {
    source
        .field(record, None, name)
        .ok_or(Error::Invalid("missing bound source field"))
}

fn reference(source: &s::Snapshot, record: usize, name: &str) -> Result<usize> {
    field(source, record, name)?
        .reference_target()
        .ok_or(Error::Invalid("missing bound reference"))
}

fn vector(source: &s::Snapshot, record: usize, name: &str) -> Result<ExactPoint> {
    let f = field(source, record, name)?;
    let values = &f
        .value()
        .ok_or(Error::Invalid("missing source value"))?
        .children;
    if values.len() != 3 {
        return Err(Error::Invalid("expected three source coordinates"));
    }
    let get = |i: usize| -> Result<p::Rational> {
        Ok(p::Rational::decimal(
            &values
                .get(i)
                .ok_or(Error::Invalid("coordinate index"))?
                .text,
        )?)
    };
    Ok([get(0)?, get(1)?, get(2)?])
}

pub(super) fn point(source: &s::Snapshot, vertex: usize, scale: p::Rational) -> Result<ExactPoint> {
    let point = reference(source, vertex, "vertex_geometry")?;
    let v = vector(source, point, "coordinates")?;
    Ok([
        v[0].multiply(scale)?,
        v[1].multiply(scale)?,
        v[2].multiply(scale)?,
    ])
}

fn subtract(a: ExactPoint, b: ExactPoint) -> Result<ExactPoint> {
    Ok([
        a[0].subtract(b[0])?,
        a[1].subtract(b[1])?,
        a[2].subtract(b[2])?,
    ])
}

fn dot(a: ExactPoint, b: ExactPoint) -> Result<p::Rational> {
    Ok(a[0]
        .multiply(b[0])?
        .checked_add(a[1].multiply(b[1])?)?
        .checked_add(a[2].multiply(b[2])?)?)
}

pub(super) fn projected_line_end(
    source: &s::Snapshot,
    edge: &p::EdgeEvidence,
    point: ExactPoint,
    scale: p::Rational,
) -> Result<ExactPoint> {
    let origin = vector(source, reference(source, edge.line, "pnt")?, "coordinates")?;
    let origin = [
        origin[0].multiply(scale)?,
        origin[1].multiply(scale)?,
        origin[2].multiply(scale)?,
    ];
    let direction = reference(source, reference(source, edge.line, "dir")?, "orientation")?;
    let d = vector(source, direction, "direction_ratios")?;
    let t = dot(subtract(point, origin)?, d)?.divide(dot(d, d)?)?;
    Ok([
        origin[0].checked_add(d[0].multiply(t)?)?,
        origin[1].checked_add(d[1].multiply(t)?)?,
        origin[2].checked_add(d[2].multiply(t)?)?,
    ])
}
