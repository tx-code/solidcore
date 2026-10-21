// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Detached source geometry owned beside, never inside, the material boundary.
//!
//! Upstream: maintained `STEP_IMPORT` at `200aa2f`, StepImportGeneral.cpp
//! `Prepare/BuildWireframes/BuildSupportingGeometry`,
//! StepImportCurvePointDefinitions.cpp and StepImportWireframe.cpp.
//! Deviations: source-indexed value owners and a borrowed view replace native
//! pointers/archive arrays. No mixed archive, final publication or native-call
//! cancellation capability is established by this construction module.

use super::geometry::{norm, Frame, Geometry, ParameterMapping};
use super::{GeneralQuality, Rational};
use crate::{Error, Execution, GeneralFailureKind, Result};
use solidcore::curve::curve::Curve;
use solidcore::geom::{axis2_placement::Axis2Placement, global_solver::SolverOperation};
use solidcore::kernel::Kernel;
use solidcore::math::{Vec2, Vec3};
use solidcore::surface::plane::Plane;
use std::collections::BTreeMap;

/// Source role of an independently associated representation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssociatedRole {
    /// A linked finite wireframe; no material boundary or inferred connectivity.
    Wireframe,
    /// Supporting constructive geometry; no material boundary or instance transform.
    Supporting,
}

/// Owner-local definition observation, not a material topology or archive handle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssociatedDefinitionRef {
    /// Index into this owner's independent curve definitions.
    Curve(usize),
    /// Index into this owner's independent point definitions.
    Point(usize),
    /// Index into this owner's coordinate-frame definitions.
    Frame(usize),
    /// Index into this owner's unbounded plane definitions.
    Plane(usize),
}

/// One original membership occurrence, including repeated uses of a definition.
#[derive(Clone, Debug)]
pub struct AssociatedOccurrence {
    /// Index in the immutable source snapshot; never a native object identity.
    pub source_record: usize,
    /// Original canonical STEP instance ID.
    pub source_id: String,
    /// Original simple/complex component type names, in original component order.
    pub source_types: Vec<String>,
    /// Shared owner-local definition selected by original source identity.
    pub definition: AssociatedDefinitionRef,
}

/// A representation item, preserving its original source occurrence order.
#[derive(Clone, Debug)]
pub enum AssociatedRepresentationItem {
    /// A source `GEOMETRIC_CURVE_SET`, whose member occurrences form one group.
    CurveSet {
        /// Original set record.
        source_record: usize,
        /// Original canonical set ID.
        source_id: String,
        /// Index into the same view's groups.
        group_index: usize,
    },
    /// A direct source definition, including retained wireframe placements.
    Definition(AssociatedOccurrence),
}

/// Original associated representation and relationship, separate from body roles.
#[derive(Clone, Debug)]
pub struct AssociatedRepresentation {
    /// Explicit physical role; it is never inferred from a display name.
    pub role: AssociatedRole,
    /// Original representation record.
    pub representation_record: usize,
    /// Original representation ID.
    pub representation_id: String,
    /// Original relationship record.
    pub relationship_record: usize,
    /// Original relationship ID.
    pub relationship_id: String,
    /// Original shared geometric/unit context record.
    pub context_record: usize,
    /// Original shared context ID.
    pub context_id: String,
    /// Original representation items in source occurrence order.
    pub items: Vec<AssociatedRepresentationItem>,
}

/// One linked curve set or direct supporting representation, in source order.
#[derive(Clone, Debug)]
pub struct AssociatedGroup {
    /// Physical source role of this group.
    pub role: AssociatedRole,
    /// Original representation record.
    pub representation_record: usize,
    /// Original representation ID.
    pub representation_id: String,
    /// Original relationship record.
    pub relationship_record: usize,
    /// Original relationship ID.
    pub relationship_id: String,
    /// Original shared geometric/unit context record.
    pub context_record: usize,
    /// Original shared context ID.
    pub context_id: String,
    /// Original curve-set record; absent for a direct supporting representation.
    pub set_record: Option<usize>,
    /// Original curve-set ID; absent for a direct supporting representation.
    pub set_id: Option<String>,
    /// Every original membership occurrence; equal coordinates are not identity.
    pub items: Vec<AssociatedOccurrence>,
    /// Finite wireframe bounds in mm; unbounded supporting geometry has no box.
    pub bounds_mm: Option<[Vec3; 2]>,
    /// Actual source-trim observations, absent when no relevant check ran.
    pub source_residual_mm: Option<f64>,
    /// Actual defining-endpoint observations, absent when no endpoint check ran.
    pub endpoint_error_mm: Option<f64>,
}

/// Original source and actual native parameter correspondence of one curve.
#[derive(Clone, Debug)]
pub struct AssociatedCurveInfo {
    /// Original source curve or trim-wrapper record.
    pub source_record: usize,
    /// Original curve or trim-wrapper ID.
    pub source_id: String,
    /// Original complete basis record; equals `source_record` without a wrapper.
    pub basis_record: usize,
    /// Original basis ID.
    pub basis_id: String,
    /// Actual native evaluator interval; conic parameters are not STEP angles.
    pub parameter_domain: [f64; 2],
    /// Whether increasing source use follows increasing native parameter.
    pub source_forward_in_native: bool,
    /// Actual source-trim residual; never fabricated for an unmeasured definition.
    pub source_residual_mm: Option<f64>,
    /// Actual endpoint error; never fabricated for an unmeasured definition.
    pub endpoint_error_mm: Option<f64>,
}

/// Independent source point, not a source/native body `VERTEX_POINT`.
#[derive(Clone, Debug)]
pub struct AssociatedPointInfo {
    /// Original source record.
    pub source_record: usize,
    /// Original source ID.
    pub source_id: String,
    /// Coordinate after the source-owned conversion to millimeters.
    pub point_mm: Vec3,
}

/// An actual coordinate definition; unit axes are dimensionless.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AssociatedCoordinateFrame {
    /// Original location after conversion to millimeters.
    pub origin_mm: Vec3,
    /// Dimensionless native X axis.
    pub x: Vec3,
    /// Dimensionless native Y axis.
    pub y: Vec3,
    /// Dimensionless native Z axis.
    pub z: Vec3,
}

/// A standalone placement; not an added point, plane locus or model transform.
#[derive(Clone, Debug)]
pub struct AssociatedFrameInfo {
    /// Original `AXIS2_PLACEMENT_3D` record.
    pub source_record: usize,
    /// Original placement ID.
    pub source_id: String,
    /// Coordinate definition retained by the actual native placement owner.
    pub frame: AssociatedCoordinateFrame,
}

/// An unbounded source plane, whose native finite sentinels are not source trims.
#[derive(Clone, Debug)]
pub struct AssociatedPlaneInfo {
    /// Original PLANE record.
    pub source_record: usize,
    /// Original plane ID.
    pub source_id: String,
    /// Original placement record.
    pub position_record: usize,
    /// Original placement ID.
    pub position_id: String,
    /// Actual retained origin and dimensionless axes.
    pub frame: AssociatedCoordinateFrame,
    /// Native representable UV domain [umin,vmin,umax,vmax], in millimeters.
    pub parameter_domain: [f64; 4],
}

/// A numerical independent-geometry answer; no certified global error bound.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AssociatedClosestPoint {
    /// Actual native point in millimeters.
    pub point_mm: Vec3,
    /// Unsquared Euclidean distance in millimeters.
    pub distance_mm: f64,
}

#[derive(Default)]
pub(super) struct OwnedAssociated {
    // Definitions are moved once into this unpublished owner. Every error drops
    // all registered values together; nothing is inserted into the body arenas.
    curves: Vec<Curve>,
    points: Vec<Vec3>,
    frames: Vec<Axis2Placement>,
    frame_adapters: Vec<Option<Plane>>,
    planes: Vec<Plane>,
    curve_info: Vec<AssociatedCurveInfo>,
    point_info: Vec<AssociatedPointInfo>,
    frame_info: Vec<AssociatedFrameInfo>,
    plane_info: Vec<AssociatedPlaneInfo>,
    representations: Vec<AssociatedRepresentation>,
    groups: Vec<AssociatedGroup>,
    mappings: Vec<ParameterMapping>,
    query_mm: f64,
}

/// Read-only associated geometry bound to the candidate and its executing Kernel.
/// It cannot outlive that owner or substitute a different/default context.
pub struct AssociatedGeometry<'a> {
    owned: &'a OwnedAssociated,
    kernel: &'a Kernel,
}

fn finite(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
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
fn id(geometry: &Geometry<'_>, record: usize) -> Result<String> {
    geometry
        .source()
        .source()
        .records()
        .get(record)
        .map(|record| record.id.clone())
        .ok_or(Error::Invalid("GeneralAssociatedSourceRecord"))
}
fn selected(geometry: &Geometry<'_>, name: &str) -> Option<usize> {
    geometry
        .source()
        .part_definition()?
        .roles
        .iter()
        .find(|role| role.name == name)
        .map(|role| role.record_index)
}
fn required_role(geometry: &Geometry<'_>, name: &str) -> Result<usize> {
    selected(geometry, name).ok_or(Error::Invalid("GeneralAssociatedSourceRole"))
}
fn detached_curve_supported(geometry: &Geometry<'_>, record: usize) -> Result<bool> {
    if geometry.is(record, "TRIMMED_CURVE") {
        let basis = geometry.reference(record, "basis_curve")?;
        Ok(geometry.is(basis, "LINE")
            || geometry.is(basis, "CIRCLE")
            || geometry.is(basis, "ELLIPSE"))
    } else {
        Ok(geometry.is(record, "CIRCLE")
            || geometry.is(record, "ELLIPSE")
            || geometry.is(record, "B_SPLINE_CURVE_WITH_KNOTS"))
    }
}

/// Reject any unsupported required member before allocating native geometry.
/// Source ownership/unit checking has already selected these exact physical roles.
pub(super) fn preflight(geometry: &Geometry<'_>) -> Result<()> {
    if let Some(rep) = selected(geometry, "construction_representation") {
        let _ = required_role(geometry, "construction_relationship")?;
        for record in geometry.references(rep, "items")? {
            if !geometry.is(record, "CARTESIAN_POINT")
                && !geometry.is(record, "AXIS2_PLACEMENT_3D")
                && !geometry.is(record, "PLANE")
                && !detached_curve_supported(geometry, record)?
            {
                return Err(failure(
                    "GeneralSupportingGeometryUnsupported",
                    record,
                    GeneralFailureKind::Unsupported,
                ));
            }
        }
    }
    if let Some(rep) = selected(geometry, "wireframe_representation") {
        let _ = required_role(geometry, "wireframe_shape_relationship")?;
        for set in geometry.references(rep, "items")? {
            if geometry.is(set, "AXIS2_PLACEMENT_3D") {
                continue;
            }
            if !geometry.is(set, "GEOMETRIC_CURVE_SET") {
                return Err(failure(
                    "GeneralWireframeGeometryUnsupported",
                    set,
                    GeneralFailureKind::Unsupported,
                ));
            }
            for record in geometry.references(set, "elements")? {
                if !geometry.is(record, "CARTESIAN_POINT")
                    && !detached_curve_supported(geometry, record)?
                {
                    return Err(failure(
                        "GeneralWireframeGeometryUnsupported",
                        record,
                        GeneralFailureKind::Unsupported,
                    ));
                }
            }
        }
    }
    Ok(())
}

impl OwnedAssociated {
    pub(super) fn empty() -> Self {
        Self::default()
    }
    pub(super) fn is_empty(&self) -> bool {
        self.representations.is_empty()
    }
    pub(super) fn parameter_mappings(&self) -> &[ParameterMapping] {
        &self.mappings
    }
    pub(super) fn view<'a>(&'a self, kernel: &'a Kernel) -> AssociatedGeometry<'a> {
        AssociatedGeometry {
            owned: self,
            kernel,
        }
    }
    pub(super) fn build(
        geometry: &Geometry<'_>,
        quality: &GeneralQuality,
        execution: &mut Execution,
    ) -> Result<Self> {
        preflight(geometry)?;
        let scalar = |value: Rational| value.numerator() as f64 / value.denominator() as f64;
        let mut builder = Builder {
            geometry,
            execution,
            definition_mm: scalar(quality.definition_mm),
            owned: Self {
                query_mm: scalar(quality.query_mm),
                ..Self::default()
            },
            curves: BTreeMap::new(),
            points: BTreeMap::new(),
            frames: BTreeMap::new(),
            planes: BTreeMap::new(),
        };
        if let Some(rep) = selected(geometry, "wireframe_representation") {
            builder.wireframe(
                rep,
                required_role(geometry, "wireframe_shape_relationship")?,
            )?;
        }
        if let Some(rep) = selected(geometry, "construction_representation") {
            builder.supporting(rep, required_role(geometry, "construction_relationship")?)?;
        }
        Ok(builder.owned)
    }
}

struct Builder<'a, 'source> {
    geometry: &'a Geometry<'source>,
    execution: &'a mut Execution,
    definition_mm: f64,
    owned: OwnedAssociated,
    curves: BTreeMap<usize, usize>,
    points: BTreeMap<usize, usize>,
    frames: BTreeMap<usize, usize>,
    planes: BTreeMap<usize, usize>,
}

fn coordinates(frame: Frame) -> AssociatedCoordinateFrame {
    AssociatedCoordinateFrame {
        origin_mm: frame.origin,
        x: frame.x,
        y: frame.y,
        z: frame.z,
    }
}
fn plane_from_frame(frame: Frame, record: usize, execution: &mut Execution) -> Result<Plane> {
    execution.step()?;
    let plane = Plane::create_canonical(&frame.native())?;
    let actual = plane.position();
    // Fixed C++ BuildSupportingGeometry compares the retained native frame
    // exactly. No acceptance/working tolerance may excuse a changed definition.
    need(
        actual.origin() == frame.origin && actual.x_axis() == frame.x && actual.y_axis() == frame.y,
        "GeneralSupportingFrameChanged",
        record,
    )?;
    Ok(plane)
}
fn measured_max(current: &mut Option<f64>, measured: Option<f64>) {
    if let Some(value) = measured {
        *current = Some(current.map_or(value, |previous| previous.max(value)));
    }
}

fn union_bounds([low, high]: [Vec3; 2], [other_low, other_high]: [Vec3; 2]) -> [Vec3; 2] {
    // std::min/std::max preserve the first value on equality, including signed
    // zero. Keep that source order instead of an implementation-dependent SIMD min.
    let lower = |a, b| if b < a { b } else { a };
    let upper = |a, b| if a < b { b } else { a };
    [
        Vec3::new(
            lower(low.x, other_low.x),
            lower(low.y, other_low.y),
            lower(low.z, other_low.z),
        ),
        Vec3::new(
            upper(high.x, other_high.x),
            upper(high.y, other_high.y),
            upper(high.z, other_high.z),
        ),
    ]
}

impl Builder<'_, '_> {
    fn occurrence(
        &self,
        record: usize,
        definition: AssociatedDefinitionRef,
    ) -> Result<AssociatedOccurrence> {
        let source = self
            .geometry
            .source()
            .source()
            .records()
            .get(record)
            .ok_or(Error::Invalid("GeneralAssociatedSourceRecord"))?;
        Ok(AssociatedOccurrence {
            source_record: record,
            source_id: source.id.clone(),
            source_types: source
                .components
                .iter()
                .map(|component| component.name.clone())
                .collect(),
            definition,
        })
    }
    fn point(&mut self, record: usize) -> Result<AssociatedOccurrence> {
        let index = if let Some(index) = self.points.get(&record) {
            *index
        } else {
            let point_mm = self.geometry.position(record)?;
            let index = self.owned.points.len();
            self.owned.point_info.push(AssociatedPointInfo {
                source_record: record,
                source_id: id(self.geometry, record)?,
                point_mm,
            });
            self.owned.points.push(point_mm);
            self.points.insert(record, index);
            index
        };
        self.occurrence(record, AssociatedDefinitionRef::Point(index))
    }
    fn curve(&mut self, record: usize, role: AssociatedRole) -> Result<AssociatedOccurrence> {
        let index = if let Some(index) = self.curves.get(&record) {
            *index
        } else {
            let built = self.geometry.make_detached_curve(
                record,
                role,
                self.definition_mm,
                self.execution,
            )?;
            let interval = built.native.natural_interval();
            need(
                built.native.dim() == 3
                    && interval.min().is_finite()
                    && interval.max().is_finite()
                    && interval.max() > interval.min(),
                "GeneralWireNativeDomain",
                record,
            )?;
            let index = self.owned.curves.len();
            self.owned.curve_info.push(AssociatedCurveInfo {
                source_record: record,
                source_id: id(self.geometry, record)?,
                basis_record: built.basis_record,
                basis_id: id(self.geometry, built.basis_record)?,
                parameter_domain: [interval.min(), interval.max()],
                source_forward_in_native: built.source_forward_in_native,
                source_residual_mm: built.source_residual_mm,
                endpoint_error_mm: built.endpoint_error_mm,
            });
            self.owned.mappings.extend(built.mappings);
            self.owned.curves.push(built.native);
            self.curves.insert(record, index);
            index
        };
        self.occurrence(record, AssociatedDefinitionRef::Curve(index))
    }
    fn frame(&mut self, record: usize, supporting: bool) -> Result<AssociatedOccurrence> {
        let index = if let Some(index) = self.frames.get(&record) {
            *index
        } else {
            let frame = self.geometry.placement(record)?;
            let index = self.owned.frames.len();
            self.owned.frame_info.push(AssociatedFrameInfo {
                source_record: record,
                source_id: id(self.geometry, record)?,
                frame: coordinates(frame),
            });
            let native = frame.native();
            need(
                native.origin() == frame.origin
                    && native.x_axis() == frame.x
                    && native.y_axis() == frame.y,
                "GeneralSupportingFrameChanged",
                record,
            )?;
            self.owned.frames.push(native);
            self.owned.frame_adapters.push(None);
            self.frames.insert(record, index);
            index
        };
        if supporting
            && self
                .owned
                .frame_adapters
                .get(index)
                .is_some_and(Option::is_none)
        {
            // Wireframe placements remain coordinate definitions. Only a
            // supporting frame needs C++'s explicit canonical-plane adapter;
            // the adapter never becomes a source PLANE or a body face.
            let native =
                plane_from_frame(self.geometry.placement(record)?, record, self.execution)?;
            *self
                .owned
                .frame_adapters
                .get_mut(index)
                .ok_or(Error::Invalid("GeneralAssociatedFrameIndex"))? = Some(native);
        }
        self.occurrence(record, AssociatedDefinitionRef::Frame(index))
    }
    fn plane(&mut self, record: usize) -> Result<AssociatedOccurrence> {
        let index = if let Some(index) = self.planes.get(&record) {
            *index
        } else {
            let position_record = self.geometry.reference(record, "position")?;
            let frame = self.geometry.placement(position_record)?;
            let native = plane_from_frame(frame, record, self.execution)?;
            let domain = native.natural_uv_domain();
            let index = self.owned.planes.len();
            self.owned.plane_info.push(AssociatedPlaneInfo {
                source_record: record,
                source_id: id(self.geometry, record)?,
                position_record,
                position_id: id(self.geometry, position_record)?,
                frame: coordinates(frame),
                parameter_domain: [
                    domain.min().x,
                    domain.min().y,
                    domain.max().x,
                    domain.max().y,
                ],
            });
            self.owned.planes.push(native);
            self.planes.insert(record, index);
            index
        };
        self.occurrence(record, AssociatedDefinitionRef::Plane(index))
    }
    fn representation(
        &self,
        rep: usize,
        relationship: usize,
        role: AssociatedRole,
    ) -> Result<AssociatedRepresentation> {
        let context_record = self.geometry.reference(rep, "context_of_items")?;
        Ok(AssociatedRepresentation {
            role,
            representation_record: rep,
            representation_id: id(self.geometry, rep)?,
            relationship_record: relationship,
            relationship_id: id(self.geometry, relationship)?,
            context_record,
            context_id: id(self.geometry, context_record)?,
            items: Vec::new(),
        })
    }
    fn group(
        &self,
        representation: &AssociatedRepresentation,
        set: Option<usize>,
    ) -> Result<AssociatedGroup> {
        Ok(AssociatedGroup {
            role: representation.role,
            representation_record: representation.representation_record,
            representation_id: representation.representation_id.clone(),
            relationship_record: representation.relationship_record,
            relationship_id: representation.relationship_id.clone(),
            context_record: representation.context_record,
            context_id: representation.context_id.clone(),
            set_record: set,
            set_id: set.map(|set| id(self.geometry, set)).transpose()?,
            items: Vec::new(),
            bounds_mm: None,
            source_residual_mm: None,
            endpoint_error_mm: None,
        })
    }
    fn wireframe(&mut self, rep: usize, relationship: usize) -> Result<()> {
        let mut representation =
            self.representation(rep, relationship, AssociatedRole::Wireframe)?;
        for set in self.geometry.references(rep, "items")? {
            if self.geometry.is(set, "AXIS2_PLACEMENT_3D") {
                let occurrence = self.frame(set, false)?;
                representation
                    .items
                    .push(AssociatedRepresentationItem::Definition(occurrence));
                continue;
            }
            let mut group = self.group(&representation, Some(set))?;
            for record in self.geometry.references(set, "elements")? {
                let occurrence = if self.geometry.is(record, "CARTESIAN_POINT") {
                    self.point(record)?
                } else {
                    self.curve(record, AssociatedRole::Wireframe)?
                };
                let bounds = match occurrence.definition {
                    AssociatedDefinitionRef::Point(index) => {
                        let point = *self
                            .owned
                            .points
                            .get(index)
                            .ok_or(Error::Invalid("GeneralAssociatedPointIndex"))?;
                        [point, point]
                    }
                    AssociatedDefinitionRef::Curve(index) => {
                        let curve = self
                            .owned
                            .curves
                            .get(index)
                            .ok_or(Error::Invalid("GeneralAssociatedCurveIndex"))?;
                        let info = self
                            .owned
                            .curve_info
                            .get(index)
                            .ok_or(Error::Invalid("GeneralAssociatedCurveIndex"))?;
                        measured_max(&mut group.source_residual_mm, info.source_residual_mm);
                        measured_max(&mut group.endpoint_error_mm, info.endpoint_error_mm);
                        // C++ Wire charges this query for EACH original use,
                        // including source-definition cache hits in later sets.
                        self.execution.step()?;
                        let bounds = curve.calculate_tight_bounding_box(
                            self.geometry.kernel(),
                            &curve.natural_interval(),
                        )?;
                        [bounds.min(), bounds.max()]
                    }
                    _ => return Err(Error::Invalid("GeneralWireframeMemberKind")),
                };
                need(
                    finite(bounds[0])
                        && finite(bounds[1])
                        && bounds[0].x <= bounds[1].x
                        && bounds[0].y <= bounds[1].y
                        && bounds[0].z <= bounds[1].z,
                    "GeneralWireFiniteBounds",
                    set,
                )?;
                group.bounds_mm = Some(
                    group
                        .bounds_mm
                        .map_or(bounds, |previous| union_bounds(previous, bounds)),
                );
                group.items.push(occurrence);
            }
            need(!group.items.is_empty(), "GeneralWireEmptySet", set)?;
            let group_index = self.owned.groups.len();
            self.owned.groups.push(group);
            representation
                .items
                .push(AssociatedRepresentationItem::CurveSet {
                    source_record: set,
                    source_id: id(self.geometry, set)?,
                    group_index,
                });
        }
        self.owned.representations.push(representation);
        Ok(())
    }
    fn supporting(&mut self, rep: usize, relationship: usize) -> Result<()> {
        let mut representation =
            self.representation(rep, relationship, AssociatedRole::Supporting)?;
        let mut group = self.group(&representation, None)?;
        for record in self.geometry.references(rep, "items")? {
            let occurrence = if self.geometry.is(record, "CARTESIAN_POINT") {
                self.point(record)?
            } else if self.geometry.is(record, "AXIS2_PLACEMENT_3D") {
                self.frame(record, true)?
            } else if self.geometry.is(record, "PLANE") {
                self.plane(record)?
            } else {
                let occurrence = self.curve(record, AssociatedRole::Supporting)?;
                let AssociatedDefinitionRef::Curve(index) = occurrence.definition else {
                    return Err(Error::Invalid("GeneralAssociatedCurveKind"));
                };
                let info = self
                    .owned
                    .curve_info
                    .get(index)
                    .ok_or(Error::Invalid("GeneralAssociatedCurveIndex"))?;
                measured_max(&mut group.source_residual_mm, info.source_residual_mm);
                measured_max(&mut group.endpoint_error_mm, info.endpoint_error_mm);
                occurrence
            };
            representation
                .items
                .push(AssociatedRepresentationItem::Definition(occurrence.clone()));
            group.items.push(occurrence);
        }
        self.owned.groups.push(group);
        self.owned.representations.push(representation);
        Ok(())
    }
}

impl AssociatedGeometry<'_> {
    /// Associated source representations, retaining original item order.
    pub fn representations(&self) -> &[AssociatedRepresentation] {
        &self.owned.representations
    }
    /// Linked curve sets followed by the direct supporting group, if present.
    pub fn groups(&self) -> &[AssociatedGroup] {
        &self.owned.groups
    }
    /// One observation for each source-owned independent curve definition.
    pub fn curve_definitions(&self) -> &[AssociatedCurveInfo] {
        &self.owned.curve_info
    }
    /// One observation per original point identity, including distinct equal points.
    pub fn point_definitions(&self) -> &[AssociatedPointInfo] {
        &self.owned.point_info
    }
    /// Coordinate definitions; frames never become extra loci or transformations.
    pub fn frame_definitions(&self) -> &[AssociatedFrameInfo] {
        &self.owned.frame_info
    }
    /// Read the actual independent native placement, not a synthesized plane locus.
    ///
    /// # Errors
    /// An index outside this owner's coordinate-frame definitions.
    pub fn coordinate_frame(&self, index: usize) -> Result<AssociatedCoordinateFrame> {
        let frame = self
            .owned
            .frames
            .get(index)
            .ok_or(Error::Invalid("GeneralAssociatedFrameIndex"))?;
        Ok(AssociatedCoordinateFrame {
            origin_mm: frame.origin(),
            x: frame.x_axis(),
            y: frame.y_axis(),
            z: frame.z_axis(),
        })
    }
    /// Unbounded plane definitions; finite native sentinels are not source trims.
    pub fn plane_definitions(&self) -> &[AssociatedPlaneInfo] {
        &self.owned.plane_info
    }
    /// Evaluate the actual unchanged independent curve in its native parameter.
    ///
    /// # Errors
    /// Foreign/out-of-range definition index, nonfinite/domain-exceeding parameter,
    /// or the actual native evaluator failure. No model/Kernel copy is created.
    pub fn evaluate_curve(&self, index: usize, parameter: f64) -> Result<Vec3> {
        let curve = self
            .owned
            .curves
            .get(index)
            .ok_or(Error::Invalid("GeneralAssociatedCurveIndex"))?;
        let interval = curve.natural_interval();
        if !parameter.is_finite() || parameter < interval.min() || parameter > interval.max() {
            return Err(Error::Invalid("GeneralAssociatedCurveParameter"));
        }
        let point = curve.evaluate_point(self.kernel, parameter)?;
        if !finite(point) {
            return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
        }
        Ok(point)
    }
    /// Evaluate an actual independent PLANE; u and v are millimeters.
    ///
    /// # Errors
    /// Invalid index/UV or a projection outside the native representable chart.
    /// Source PLANE has no trim; values are rejected rather than clamped.
    pub fn evaluate_plane(&self, index: usize, uv: Vec2) -> Result<Vec3> {
        let plane = self
            .owned
            .planes
            .get(index)
            .ok_or(Error::Invalid("GeneralAssociatedPlaneIndex"))?;
        let domain = plane.natural_uv_domain();
        if !uv.x.is_finite()
            || !uv.y.is_finite()
            || uv.x < domain.min().x
            || uv.y < domain.min().y
            || uv.x > domain.max().x
            || uv.y > domain.max().y
        {
            return Err(Error::Invalid("GeneralAssociatedPlaneParameter"));
        }
        let point = plane.evaluate_point(uv);
        if !finite(point) {
            return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
        }
        Ok(point)
    }
    /// Project onto the selected infinite source plane, never a material boundary.
    ///
    /// # Errors
    /// Invalid input/index, degenerate retained frame or an unrepresentable native
    /// UV. The numerical query does not establish a certified global error bound.
    pub fn closest_plane(&self, index: usize, point_mm: Vec3) -> Result<AssociatedClosestPoint> {
        if !finite(point_mm) {
            return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
        }
        let plane = self
            .owned
            .planes
            .get(index)
            .ok_or(Error::Invalid("GeneralAssociatedPlaneIndex"))?;
        let frame = plane.position();
        let origin = frame.origin();
        let x = frame.x_axis();
        let y = frame.y_axis();
        // Fixed C++ ClosestPlane solves the Gram system of the actual binary64
        // axes. Replacing it with unit-axis projections changes that geometry.
        let mut dx = 0.;
        let mut dy = 0.;
        let mut xx = 0.;
        let mut xy = 0.;
        let mut yy = 0.;
        for (p, o, x, y) in [
            (point_mm.x, origin.x, x.x, y.x),
            (point_mm.y, origin.y, x.y, y.y),
            (point_mm.z, origin.z, x.z, y.z),
        ] {
            let delta = p - o;
            dx += delta * x;
            dy += delta * y;
            xx += x * x;
            xy += x * y;
            yy += y * y;
        }
        let determinant = xx * yy - xy * xy;
        if !determinant.is_finite() || determinant <= 0. {
            return Err(Error::Invalid("GeneralSupportingProjectionFrame"));
        }
        let projected = self.evaluate_plane(
            index,
            Vec2::new(
                (dx * yy - dy * xy) / determinant,
                (dy * xx - dx * xy) / determinant,
            ),
        )?;
        let distance_mm = norm(projected - point_mm);
        if !distance_mm.is_finite() {
            return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
        }
        Ok(AssociatedClosestPoint {
            point_mm: projected,
            distance_mm,
        })
    }
    /// Query exactly one linked finite source set using actual native definitions.
    ///
    /// # Errors
    /// Invalid input/group role, an empty set or any necessary native point solve
    /// without a valid answer. An exact zero from a member proves the minimum and
    /// may return immediately. Display tessellation is never consulted.
    pub fn closest_wireframe(
        &self,
        group_index: usize,
        point_mm: Vec3,
    ) -> Result<AssociatedClosestPoint> {
        if !finite(point_mm) {
            return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
        }
        let group = self
            .owned
            .groups
            .get(group_index)
            .ok_or(Error::Invalid("GeneralAssociatedGroupIndex"))?;
        if group.role != AssociatedRole::Wireframe || group.set_record.is_none() {
            return Err(Error::Invalid("GeneralAssociatedWireframeRole"));
        }
        let mut best: Option<AssociatedClosestPoint> = None;
        let mut take = |point: Vec3| -> Result<()> {
            let distance_mm = norm(point - point_mm);
            if !finite(point) || !distance_mm.is_finite() {
                return Err(Error::Invalid("GeneralAssociatedQueryNonfinite"));
            }
            if best.is_none_or(|previous| distance_mm < previous.distance_mm) {
                best = Some(AssociatedClosestPoint {
                    point_mm: point,
                    distance_mm,
                });
            }
            Ok(())
        };
        // C++ searches points before curves, retaining occurrence order within
        // each family. This can prove a zero without an unrelated failed solver.
        for occurrence in &group.items {
            if let AssociatedDefinitionRef::Point(index) = occurrence.definition {
                let point = *self
                    .owned
                    .points
                    .get(index)
                    .ok_or(Error::Invalid("GeneralAssociatedPointIndex"))?;
                take(point)?;
                if point == point_mm {
                    return Ok(AssociatedClosestPoint {
                        point_mm: point,
                        distance_mm: 0.,
                    });
                }
            }
        }
        for occurrence in &group.items {
            if let AssociatedDefinitionRef::Curve(index) = occurrence.definition {
                let curve = self
                    .owned
                    .curves
                    .get(index)
                    .ok_or(Error::Invalid("GeneralAssociatedCurveIndex"))?;
                let interval = curve.natural_interval();
                let dropped = curve.drop_point(
                    self.kernel,
                    &interval,
                    point_mm,
                    None,
                    self.owned.query_mm,
                    None,
                    SolverOperation::Minimize,
                )?;
                if !dropped.success
                    || !dropped.parameter.is_finite()
                    || !dropped.distance.is_finite()
                    || dropped.distance < 0.
                    || dropped.parameter < interval.min()
                    || dropped.parameter > interval.max()
                {
                    return Err(Error::Invalid("GeneralAssociatedClosestUnestablished"));
                }
                let candidate = self.evaluate_curve(index, dropped.parameter)?;
                take(candidate)?;
                if candidate == point_mm {
                    return Ok(AssociatedClosestPoint {
                        point_mm: candidate,
                        distance_mm: 0.,
                    });
                }
            }
        }
        best.ok_or(Error::Invalid("GeneralAssociatedEmptyQuerySet"))
    }
}
