// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Portions of norm(): Copyright (c) Microsoft Corporation.
// SPDX-License-Identifier: Apache-2.0 AND (Apache-2.0 WITH LLVM-exception)
//! Typed source geometry and unchanged defining curves for general import.
//!
//! Upstream: maintained `STEP_IMPORT/src/StepImportGeometryConstruction.h` and
//! `StepImportGeneral.cpp::MakeCurve/CurveParameter`, fixed fork `d8ed45a`.
//! Deviations: checked Rust source indices, owned return values and explicit
//! execution charges replace native out-parameters and allocation ownership.
//! Raw source units, parameters, weights and topology are never rewritten.

use super::super::{Error, Execution, GeneralFailureKind, Result};
use solidcore::curve::{
    curve::Curve,
    ellipse::{Ellipse, EllipseOptions},
    line::Line,
};
use solidcore::geom::{
    axis2_placement::Axis2Placement, extent1d::Extent1d, extent3d::Extent3d,
    global_solver::SolverOperation,
};
use solidcore::kernel::Kernel;
use solidcore::math::{Vec3, EFF_ZERO};
use solidcore::nurbs::b_spline_curve::{
    BSplineCurve, BSplineCurveForm, BoundingBoxRequest, CanonicalData, KnotType,
};
use solidcore::nurbs::math::CPoint;
use solidcore_step::{planar::Rational, semantics as s, Value, ValueKind};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

mod detached;

#[derive(Clone, Copy, Debug)]
pub(super) struct Frame {
    pub origin: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    pub fn native(self) -> Axis2Placement {
        Axis2Placement::new(self.origin, self.x, self.y)
    }
}

/// Original source parameter relation retained by general development construction.
#[derive(Clone, Debug)]
pub struct ParameterMapping {
    /// Original geometry record index.
    pub source_geometry: usize,
    /// Original use or edge record index.
    pub source_use: usize,
    /// Original basis record for a wrapper, when present.
    pub source_basis_geometry: Option<usize>,
    /// Stable maintained-fork mapping identity.
    pub kind: &'static str,
    /// Source/native relation; native conic parameters are not angles.
    pub equation: String,
    /// Named original and converted coefficients, including parameter conventions.
    pub parameters: Vec<(String, Vec<f64>)>,
}

pub(super) struct BuiltCurve {
    pub native: Option<Curve>,
    pub interval: Extent1d,
    pub endpoints: [Vec3; 2],
    pub bounds: Extent3d,
    pub mappings: Vec<ParameterMapping>,
    pub maximum_consistency_mm: f64,
}

/// Original exact conversion-unit dependency used by the native constructor.
#[derive(Clone, Debug)]
pub struct UnitConversion {
    /// Original conversion-based unit record.
    pub unit: usize,
    /// Original measure-with-unit record.
    pub factor: usize,
    /// Original base-unit record.
    pub base: usize,
    /// Original measure type name.
    pub value_type: String,
    /// Original exact measure value.
    pub factor_value: Rational,
    /// Resolved exact length factor to millimeters.
    pub scale_mm: Rational,
}

pub(super) struct Geometry<'a> {
    source: &'a s::Snapshot,
    kernel: &'a Kernel,
    length_scale: f64,
    length_scale_exact: Rational,
    conversions: Vec<UnitConversion>,
    consistency_mm: f64,
    query_mm: f64,
}

fn failure(key: &'static str, record: usize, kind: GeneralFailureKind) -> Error {
    Error::General { key, record, kind }
}

fn need(passed: bool, key: &'static str, record: usize) -> Result<()> {
    if passed {
        Ok(())
    } else {
        Err(failure(key, record, GeneralFailureKind::Postcheck))
    }
}

fn finite(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

// StepImportGeometryConstruction.h::TrimmedConic at maintained 200aa2f uses
// Add(origin, Add(xterm, yterm)). Preserve that binary64 addition order: large
// translations can otherwise change a trim residual by an ULP and incorrectly
// cross the caller's consistency threshold. This is source interpretation, not
// a fit or a tolerance adjustment; independent curve endpoints use it too.
fn source_conic_point(frame: Frame, radius_x: f64, radius_y: f64, angle: f64) -> Vec3 {
    frame.origin + (frame.x * (radius_x * angle.cos()) + frame.y * (radius_y * angle.sin()))
}

// The maintained Windows source helper uses std::hypot(x,y,z), not the kernel's
// tolerance-based Unitize. Match this SDK's Microsoft STL _Hypot3 exactly:
// MSVC 14.50.35717 crt/src/stl/special_math.cpp:483-509 (Apache-2.0 WITH
// LLVM-exception). Keep swap order, epsilon shortcut and left-associated sum;
// two nested binary hypot calls can round a source frame or gap differently.
// Portions Copyright (c) Microsoft Corporation. The corresponding license is
// retained in LICENSES/Microsoft-STL.txt; see the repository ATTRIBUTION.md.
pub(super) fn norm(point: Vec3) -> f64 {
    let mut x = point.x.abs();
    let mut y = point.y.abs();
    let mut z = point.z.abs();
    if x == f64::INFINITY || y == f64::INFINITY || z == f64::INFINITY {
        return f64::INFINITY;
    }
    if y > x {
        std::mem::swap(&mut x, &mut y);
    }
    if z > x {
        std::mem::swap(&mut x, &mut z);
    }
    if x * f64::EPSILON >= y && x * f64::EPSILON >= z {
        return x;
    }
    let yx = y / x;
    let zx = z / x;
    x * (1. + yx * yx + zx * zx).sqrt()
}

fn unit(point: Vec3, record: usize) -> Result<Vec3> {
    let length = norm(point);
    if !(length > 0. && length.is_finite()) {
        return Err(failure(
            "GeneralDegenerateFrame",
            record,
            GeneralFailureKind::Indeterminate,
        ));
    }
    Ok(point * (1. / length))
}

fn numeric(value: &Value, record: usize) -> Result<f64> {
    let text = value.text.strip_prefix('+').unwrap_or(&value.text);
    let result = text.parse::<f64>().map_err(|_| {
        failure(
            "GeneralNumericRangeUnestablished",
            record,
            GeneralFailureKind::Indeterminate,
        )
    })?;
    // from_chars rejects a nonzero number that underflows binary64. Rust's
    // parser returns zero, so retain that refusal explicitly, including exponents.
    let nonzero_mantissa = text
        .split(['e', 'E'])
        .next()
        .is_some_and(|mantissa| mantissa.bytes().any(|b| (b'1'..=b'9').contains(&b)));
    if !result.is_finite() || (result == 0. && nonzero_mantissa) {
        return Err(failure(
            "GeneralNumericRangeUnestablished",
            record,
            GeneralFailureKind::Indeterminate,
        ));
    }
    Ok(result)
}

impl<'a> Geometry<'a> {
    pub fn new(
        source: &'a s::Snapshot,
        kernel: &'a Kernel,
        consistency_mm: f64,
        query_mm: f64,
    ) -> Result<Self> {
        let mut geometry = Self {
            source,
            kernel,
            length_scale: 0.,
            length_scale_exact: Rational::decimal("1")?,
            conversions: Vec::new(),
            consistency_mm,
            query_mm,
        };
        geometry.resolve_units()?;
        Ok(geometry)
    }

    pub const fn length_scale(&self) -> f64 {
        self.length_scale
    }

    pub const fn length_scale_exact(&self) -> Rational {
        self.length_scale_exact
    }

    pub fn conversions(&self) -> &[UnitConversion] {
        &self.conversions
    }

    pub const fn kernel(&self) -> &Kernel {
        self.kernel
    }

    pub const fn source(&self) -> &s::Snapshot {
        self.source
    }

    pub fn is(&self, record: usize, name: &str) -> bool {
        self.source.is_a(record, name) == Some(true)
    }

    pub fn field(&self, record: usize, name: &str) -> Result<s::FieldView> {
        self.source
            .field(record, None, name)
            .ok_or(Error::Invalid("missing general bound source field"))
    }

    pub fn reference(&self, record: usize, name: &str) -> Result<usize> {
        self.field(record, name)?
            .reference_target()
            .ok_or(Error::Invalid("missing general bound source reference"))
    }

    pub fn reference_value(&self, value: &Value) -> Result<usize> {
        self.source
            .reference_target(value)
            .ok_or(Error::Invalid("missing general source value reference"))
    }

    pub fn references(&self, record: usize, name: &str) -> Result<Vec<usize>> {
        let field = self.field(record, name)?;
        field
            .value()
            .ok_or(Error::Invalid("missing general source aggregate"))?
            .children
            .iter()
            .map(|value| self.reference_value(value))
            .collect()
    }

    pub fn number(&self, record: usize, name: &str) -> Result<f64> {
        let field = self.field(record, name)?;
        numeric(
            field
                .value()
                .ok_or(Error::Invalid("missing general source number"))?,
            record,
        )
    }

    pub fn numbers(&self, record: usize, name: &str) -> Result<Vec<f64>> {
        let field = self.field(record, name)?;
        field
            .value()
            .ok_or(Error::Invalid("missing general source numbers"))?
            .children
            .iter()
            .map(|value| numeric(value, record))
            .collect()
    }

    pub fn boolean(&self, record: usize, name: &str) -> Result<bool> {
        let field = self.field(record, name)?;
        Ok(field
            .value()
            .ok_or(Error::Invalid("missing general source boolean"))?
            .text
            == "T")
    }

    fn triple(&self, record: usize, name: &str) -> Result<Vec3> {
        let field = self.field(record, name)?;
        let values = &field
            .value()
            .ok_or(Error::Invalid("missing general source triple"))?
            .children;
        let [x, y, z] = values.as_slice() else {
            return Err(failure(
                "GeneralRequires3DGeometry",
                record,
                GeneralFailureKind::Unsupported,
            ));
        };
        Ok(Vec3::new(
            numeric(x, record)?,
            numeric(y, record)?,
            numeric(z, record)?,
        ))
    }

    pub fn position(&self, record: usize) -> Result<Vec3> {
        let point = self.triple(record, "coordinates")? * self.length_scale;
        if !finite(point) {
            return Err(failure(
                "GeneralLengthRangeUnestablished",
                record,
                GeneralFailureKind::Indeterminate,
            ));
        }
        Ok(point)
    }

    pub fn direction(&self, record: usize) -> Result<Vec3> {
        unit(self.triple(record, "direction_ratios")?, record)
    }

    fn unset(&self, record: usize, name: &str) -> Result<bool> {
        let field = self.field(record, name)?;
        Ok(field
            .value()
            .ok_or(Error::Invalid("missing optional general field"))?
            .kind
            == ValueKind::Unset)
    }

    pub fn placement(&self, record: usize) -> Result<Frame> {
        let origin = self.position(self.reference(record, "location")?)?;
        let z = if self.unset(record, "axis")? {
            Vec3::Z
        } else {
            self.direction(self.reference(record, "axis")?)?
        };
        let reference = if self.unset(record, "ref_direction")? {
            if z.y != 0. || z.z != 0. {
                Vec3::X
            } else {
                Vec3::Y
            }
        } else {
            self.direction(self.reference(record, "ref_direction")?)?
        };
        // AP214 build_axes / FIRST_PROJ_AXIS: the double cross projects the
        // dimensionless reference direction; no source length scale applies.
        let x = unit(z.cross(reference.cross(z)), record)?;
        let y = unit(z.cross(x), record)?;
        Ok(Frame { origin, x, y, z })
    }

    fn resolve_units(&mut self) -> Result<()> {
        let units = self.source.units();
        let root = units
            .length_unit
            .ok_or(Error::Invalid("missing general selected length unit"))?;
        if let Some(power) = units.length_power10_mm {
            if power.unsigned_abs() > 15 {
                return Err(failure(
                    "GeneralUnitScaleRange",
                    root,
                    GeneralFailureKind::Indeterminate,
                ));
            }
            self.length_scale = 10_f64.powf(f64::from(power));
            self.length_scale_exact = Rational::decimal(&format!("1E{power}"))?;
        } else {
            // StepImportUnits.cpp::ResolveLengthScale: nominal SI-rooted length
            // only, iterative path and base-to-root exact products. Semantic
            // dimensions are prerequisites, never a guessed physical factor.
            let si: BTreeMap<_, _> = units
                .si_definitions
                .iter()
                .map(|definition| (definition.record_index, definition))
                .collect();
            let mut active = BTreeSet::new();
            let mut path = Vec::new();
            let mut unit = root;
            let mut scale;
            loop {
                if active.contains(&unit) {
                    return Err(failure(
                        "GeneralCyclicUnitDefinition",
                        unit,
                        GeneralFailureKind::Unsupported,
                    ));
                }
                if let Some(terminal) = si.get(&unit) {
                    if terminal.quantity != "Length" || terminal.si_name != "METRE" {
                        return Err(failure(
                            "GeneralConversionQuantityUnsupported",
                            unit,
                            GeneralFailureKind::Unsupported,
                        ));
                    }
                    scale = Rational::decimal(&format!("1E{}", terminal.canonical_power10))
                        .map_err(|_| {
                            failure(
                                "GeneralUnitScaleRange",
                                unit,
                                GeneralFailureKind::Indeterminate,
                            )
                        })?;
                    break;
                }
                if !self.is(unit, "LENGTH_UNIT") || !self.is(unit, "CONVERSION_BASED_UNIT") {
                    return Err(failure(
                        "GeneralConversionBasisUnsupported",
                        unit,
                        GeneralFailureKind::Unsupported,
                    ));
                }
                active.insert(unit);
                let factor = self.reference(unit, "conversion_factor")?;
                let base = self.reference(factor, "unit_component")?;
                let field = self.field(factor, "value_component")?;
                let value = field
                    .value()
                    .ok_or(Error::Invalid("missing general typed conversion factor"))?;
                let [number] = value.children.as_slice() else {
                    return Err(Error::Invalid(
                        "general conversion requires one typed number",
                    ));
                };
                let factor_value = Rational::decimal(&number.text).map_err(|_| {
                    failure(
                        "GeneralUnitScaleRange",
                        unit,
                        GeneralFailureKind::Indeterminate,
                    )
                })?;
                if factor_value.numerator() <= 0 {
                    return Err(failure(
                        "GeneralUnitScaleNotPositive",
                        factor,
                        GeneralFailureKind::Unsupported,
                    ));
                }
                path.push(UnitConversion {
                    unit,
                    factor,
                    base,
                    value_type: value.text.clone(),
                    factor_value,
                    scale_mm: factor_value,
                });
                unit = base;
            }
            while let Some(mut definition) = path.pop() {
                scale = definition.factor_value.multiply(scale).map_err(|_| {
                    failure(
                        "GeneralUnitScaleRange",
                        definition.unit,
                        GeneralFailureKind::Indeterminate,
                    )
                })?;
                definition.scale_mm = scale;
                self.conversions.push(definition);
            }
            self.length_scale_exact = scale;
            self.length_scale = scale.numerator() as f64 / scale.denominator() as f64;
        }
        need(
            self.length_scale > 0. && self.length_scale.is_finite(),
            "GeneralUnitScaleRange",
            root,
        )
    }
}

#[derive(Clone, Copy)]
struct ConicTrim {
    first: f64,
    last: f64,
    low: f64,
    high: f64,
    full: bool,
    angle_scale: Option<f64>,
    maximum_consistency_mm: f64,
}

impl Geometry<'_> {
    pub fn degree(&self, record: usize, name: &str) -> Result<usize> {
        let degree = self.number(record, name)?;
        need(
            (1. ..=32.).contains(&degree) && degree.floor() == degree,
            "GeneralSplineDegree",
            record,
        )?;
        Ok(degree as usize)
    }

    pub fn multiplicities(&self, record: usize, name: &str) -> Result<Vec<usize>> {
        self.numbers(record, name)?
            .into_iter()
            .map(|number| {
                need(
                    (1. ..=33.).contains(&number) && number.floor() == number,
                    "GeneralKnotMultiplicity",
                    record,
                )?;
                Ok(number as usize)
            })
            .collect()
    }

    pub fn spline_weights(
        &self,
        geometry: usize,
        surface: bool,
        controls: &[Vec3],
    ) -> Result<Vec<f64>> {
        let mut result = Vec::new();
        if !self.is(
            geometry,
            if surface {
                "RATIONAL_B_SPLINE_SURFACE"
            } else {
                "RATIONAL_B_SPLINE_CURVE"
            },
        ) {
            return Ok(result);
        }
        let field = self.field(geometry, "weights_data")?;
        let values = &field
            .value()
            .ok_or(Error::Invalid("missing general rational weights"))?
            .children;
        let mut append = |value: &Value| -> Result<()> {
            let weight = numeric(value, geometry)?;
            need(weight > 0., "GeneralWeightRangeUnestablished", geometry)?;
            result.push(weight);
            Ok(())
        };
        for value in values {
            if surface {
                for weight in &value.children {
                    append(weight)?;
                }
            } else {
                append(value)?;
            }
        }
        need(
            result.len() == controls.len(),
            "GeneralWeightNetShape",
            geometry,
        )?;
        for (weight, point) in result.iter().zip(controls) {
            for coordinate in [point.x, point.y, point.z] {
                let homogeneous = coordinate * weight;
                need(
                    homogeneous.is_finite() && (coordinate == 0. || homogeneous != 0.),
                    "GeneralHomogeneousRangeUnestablished",
                    geometry,
                )?;
            }
        }
        Ok(result)
    }

    pub fn verify_rational_net(
        &self,
        geometry: usize,
        source_use: usize,
        controls: &[Vec3],
        weights: &[f64],
        actual: &[CPoint],
        rational: bool,
        definition_mm: f64,
    ) -> Result<Option<ParameterMapping>> {
        need(
            actual.len() == controls.len() && weights.len() == controls.len(),
            "GeneralRationalNetShapeChanged",
            geometry,
        )?;
        let field = self.field(geometry, "weights_data")?;
        let source_weights = &field
            .value()
            .ok_or(Error::Invalid("missing general source weights"))?
            .children;
        let surface = self.is(geometry, "RATIONAL_B_SPLINE_SURFACE");
        let mut lexemes = source_weights
            .iter()
            .flat_map(|value| {
                if surface {
                    value.children.as_slice()
                } else {
                    std::slice::from_ref(value)
                }
            })
            .map(|value| value.text.as_str());
        let first = lexemes
            .next()
            .ok_or(Error::Invalid("missing first general source weight"))?;
        let uniform_source = lexemes.all(|text| text == first);
        let mut cancelled = false;
        for ((point, weight), control) in actual.iter().zip(weights).zip(controls) {
            let actual_weight = if rational { point.w } else { 1. };
            if actual_weight != *weight {
                // Equal rounded doubles do not prove equal source weights.
                // Only identical source lexemes everywhere authorize cancellation.
                need(
                    uniform_source && !rational && actual_weight == 1.,
                    "GeneralSourceWeightsChanged",
                    geometry,
                )?;
                cancelled = true;
            }
            let euclidean = Vec3::new(
                point.x / actual_weight,
                point.y / actual_weight,
                point.z / actual_weight,
            );
            need(
                finite(euclidean),
                "GeneralHomogeneousRangeUnestablished",
                geometry,
            )?;
            need(
                norm(euclidean - *control) <= definition_mm,
                "GeneralControlRoundtripUnestablished",
                geometry,
            )?;
        }
        Ok(if cancelled {
            Some(ParameterMapping {
                source_geometry: geometry,
                source_use,
                source_basis_geometry: None,
                kind: "ExactUniformWeightCancellation",
                equation: "identical positive source weights cancel from numerator/denominator; source parameters and Euclidean controls unchanged".into(),
                parameters: vec![
                    (
                        "source_uniform_weight".into(),
                        vec![*weights.first().ok_or(Error::Invalid("missing uniform weight"))?],
                    ),
                    ("native_uniform_weight".into(), vec![1.]),
                ],
            })
        } else {
            None
        })
    }

    fn spline_curve(
        &self,
        geometry: usize,
        source_use: usize,
        definition_mm: f64,
        execution: &mut Execution,
    ) -> Result<(Curve, Option<ParameterMapping>)> {
        let controls = self
            .references(geometry, "control_points_list")?
            .into_iter()
            .map(|record| self.position(record))
            .collect::<Result<Vec<_>>>()?;
        let multiplicities = self.multiplicities(geometry, "knot_multiplicities")?;
        let knots = self.numbers(geometry, "knots")?;
        let weights = self.spline_weights(geometry, false, &controls)?;
        // STEP Cartesian controls stay Euclidean; the native canonical factory
        // performs the sole conversion to homogeneous coordinates.
        execution.step()?;
        let native = BSplineCurve::create_canonical(&CanonicalData {
            dim: 3,
            degree: self.degree(geometry, "degree")?,
            control_points: &controls,
            form: BSplineCurveForm::Unspecified,
            knot_multiplicities: &multiplicities,
            knots: &knots,
            knot_type: KnotType::Unspecified,
            weights: (!weights.is_empty()).then_some(weights.as_slice()),
            trim: None,
        })?;
        let mapping = if weights.is_empty() {
            None
        } else {
            self.verify_rational_net(
                geometry,
                source_use,
                &controls,
                &weights,
                native
                    .gw_nurb()
                    .ok_or(Error::Invalid("missing canonical general curve NURB"))?
                    .cpts(),
                native.is_rational()?,
                definition_mm,
            )?
        };
        Ok((native.into(), mapping))
    }

    pub fn angle_scale(&self, record: usize, key: &'static str) -> Result<f64> {
        let units = self.source.units();
        let scale = units
            .si_definitions
            .iter()
            .find(|unit| {
                Some(unit.record_index) == units.plane_angle_unit && unit.si_name == "RADIAN"
            })
            .map(|unit| 10_f64.powf(f64::from(unit.canonical_power10)))
            .or_else(|| {
                // Only the source-owned development proof can admit a
                // converted physical angle here. Do not rebuild a second unit
                // graph, infer degrees from a label, or rewrite source values.
                // Source syntax remains unverified in that separate receipt.
                self.source
                    .development_unit_receipt()?
                    .scale_to_radians(units.plane_angle_unit?)
            });
        scale
            .filter(|scale| scale.is_finite() && *scale > 0.)
            .ok_or_else(|| failure(key, record, GeneralFailureKind::Unsupported))
    }

    fn trimmed_conic(
        &self,
        wrapper: usize,
        frame: Frame,
        radius_x: f64,
        radius_y: f64,
    ) -> Result<ConicTrim> {
        let field = self.field(wrapper, "master_representation")?;
        let preferred = &field
            .value()
            .ok_or(Error::Invalid("missing general trim master representation"))?
            .text;
        let evaluate = |angle: f64| source_conic_point(frame, radius_x, radius_y, angle);
        let angle = |point: Vec3| {
            let delta = point - frame.origin;
            (delta.dot(frame.y) / radius_y).atan2(delta.dot(frame.x) / radius_x)
        };
        let mut angle_scale = None;
        let mut maximum_consistency_mm: f64 = 0.;
        let mut cut = |name: &str| -> Result<f64> {
            let mut parameter = None;
            let mut point_angle = None;
            let mut point = None;
            let field = self.field(wrapper, name)?;
            for value in &field
                .value()
                .ok_or(Error::Invalid("missing general trim set"))?
                .children
            {
                if value.kind == ValueKind::Reference {
                    let cartesian = self.position(self.reference_value(value)?)?;
                    let inferred = angle(cartesian);
                    let gap = norm(evaluate(inferred) - cartesian);
                    maximum_consistency_mm = maximum_consistency_mm.max(gap);
                    need(
                        gap.is_finite() && gap <= self.consistency_mm,
                        "GeneralTrimmedPointOffBasis",
                        wrapper,
                    )?;
                    point = Some(cartesian);
                    point_angle = Some(inferred);
                } else {
                    if angle_scale.is_none() {
                        angle_scale = Some(
                            self.angle_scale(wrapper, "GeneralTrimmedAngleUnitUnestablished")?,
                        );
                    }
                    let scale =
                        angle_scale.ok_or(Error::Invalid("missing general trim angle scale"))?;
                    let number = value
                        .children
                        .first()
                        .ok_or(Error::Invalid("missing typed trim parameter"))?;
                    let angle = numeric(number, wrapper)? * scale;
                    if !angle.is_finite() || angle.abs() > 2. * TAU {
                        return Err(failure(
                            "GeneralTrimmedAngleRangeUnestablished",
                            wrapper,
                            GeneralFailureKind::Unsupported,
                        ));
                    }
                    parameter = Some(angle);
                }
            }
            if let (Some(parameter), Some(point)) = (parameter, point) {
                let gap = norm(evaluate(parameter) - point);
                maximum_consistency_mm = maximum_consistency_mm.max(gap);
                need(
                    gap.is_finite() && gap <= self.consistency_mm,
                    "GeneralTrimmedRepresentationsConflict",
                    wrapper,
                )?;
            }
            // The master preference chooses a cut; it cannot excuse a second
            // supplied representation that conflicts with that defining cut.
            if preferred == "CARTESIAN" && point_angle.is_some() {
                point_angle.ok_or(Error::Invalid("missing cartesian trim angle"))
            } else {
                parameter
                    .or(point_angle)
                    .ok_or(Error::Invalid("empty general trim set"))
            }
        };
        let mut first = cut("trim_1")?;
        let mut last = cut("trim_2")?;
        let same = self.boolean(wrapper, "sense_agreement")?;
        let mut extent = if same { last - first } else { first - last };
        if extent < 0. && extent > -TAU {
            if same {
                last += TAU;
            } else {
                first += TAU;
            }
            extent += TAU;
        }
        // This envelope accounts only for binary64 angle conversion. A physical
        // consistency tolerance never upgrades a short source arc to a full one.
        let roundoff = 32. * f64::EPSILON * TAU;
        if extent.partial_cmp(&0.) != Some(std::cmp::Ordering::Greater) || extent > TAU + roundoff {
            return Err(failure(
                "GeneralTrimmedConicExtentUnestablished",
                wrapper,
                GeneralFailureKind::Unsupported,
            ));
        }
        Ok(ConicTrim {
            first,
            last,
            low: first.min(last),
            high: first.max(last),
            full: (extent - TAU).abs() <= roundoff,
            angle_scale,
            maximum_consistency_mm,
        })
    }
}

impl Geometry<'_> {
    fn conic(
        &self,
        geometry: usize,
        frame: Frame,
        radius_x: f64,
        radius_y: f64,
        domain: Extent1d,
        checked: bool,
        execution: &mut Execution,
    ) -> Result<Curve> {
        let circle = self.is(geometry, "CIRCLE");
        execution.step()?;
        let native = if checked {
            if circle {
                Ellipse::create_canonical_circle(
                    self.kernel,
                    &frame.native(),
                    radius_x,
                    Some(&domain),
                    None,
                )?
            } else {
                Ellipse::create_canonical(
                    self.kernel,
                    &frame.native(),
                    radius_x,
                    radius_y,
                    Some(&domain),
                    None,
                )?
            }
        } else if circle {
            Ellipse::new_circle(
                self.kernel,
                frame.origin,
                frame.x,
                frame.y,
                domain,
                radius_x,
                &EllipseOptions::default(),
            )?
        } else {
            Ellipse::new(
                self.kernel,
                frame.origin,
                frame.x,
                frame.y,
                domain,
                radius_x,
                radius_y,
                &EllipseOptions::default(),
            )?
        };
        Ok(native.into())
    }

    fn curve_parameter(
        &self,
        curve: &Curve,
        point: Vec3,
        endpoint: f64,
        record: usize,
        execution: &mut Execution,
    ) -> Result<f64> {
        execution.step()?;
        let actual = curve.evaluate_point(self.kernel, endpoint)?;
        if norm(actual - point) <= self.consistency_mm {
            return Ok(endpoint);
        }
        execution.step()?;
        let dropped = curve.drop_point(
            self.kernel,
            &curve.natural_interval(),
            point,
            None,
            self.query_mm,
            None,
            SolverOperation::Minimize,
        )?;
        need(
            dropped.success && dropped.distance <= self.consistency_mm,
            "GeneralSplineEndpointInversion",
            record,
        )?;
        Ok(dropped.parameter)
    }

    pub fn make_curve(
        &self,
        geometry: usize,
        use_record: usize,
        a: Vec3,
        b: Vec3,
        closed: bool,
        definition_mm: f64,
        execution: &mut Execution,
    ) -> Result<BuiltCurve> {
        let trimmed = self.is(geometry, "TRIMMED_CURVE");
        let basis = if trimmed {
            self.reference(geometry, "basis_curve")?
        } else {
            geometry
        };
        if trimmed && !self.is(basis, "CIRCLE") && !self.is(basis, "ELLIPSE") {
            return Err(failure(
                "GeneralTrimmedBasisUnsupported",
                geometry,
                GeneralFailureKind::Unsupported,
            ));
        }
        let mut mapping = ParameterMapping {
            source_geometry: basis,
            source_use: use_record,
            source_basis_geometry: None,
            kind: "",
            equation: String::new(),
            parameters: Vec::new(),
        };
        let mut mappings = Vec::new();
        let mut maximum_consistency_mm: f64 = 0.;
        let native = if self.is(basis, "LINE") {
            let origin = self.position(self.reference(basis, "pnt")?)?;
            let vector = self.reference(basis, "dir")?;
            let magnitude = self.number(vector, "magnitude")?;
            need(magnitude > 0., "GeneralDegenerateLine", basis)?;
            let direction = self.direction(self.reference(vector, "orientation")?)?;
            let t0 = (a - origin).dot(direction);
            let t1 = (b - origin).dot(direction);
            need(t1 > t0, "GeneralLineParameterOrder", use_record)?;
            mapping.kind = "LineAffineParameter";
            mapping.equation = "source_t=lerp(source_parameter_at_native_ends,(native_t-native_begin)/(native_end-native_begin))".into();
            mapping.parameters.push((
                "source_parameter_at_native_ends".into(),
                vec![
                    t0 / (magnitude * self.length_scale),
                    t1 / (magnitude * self.length_scale),
                ],
            ));
            // Define the original source LINE, not a fit to the possibly gapped
            // source topology vertices. Only its native parameter chart changes.
            execution.step()?;
            Line::create_line_segment(3, origin + direction * t0, origin + direction * t1, None)?
                .into()
        } else if self.is(basis, "CIRCLE") || self.is(basis, "ELLIPSE") {
            let frame = self.placement(self.reference(basis, "position")?)?;
            let circle = self.is(basis, "CIRCLE");
            let radius_x = self.number(basis, if circle { "radius" } else { "semi_axis_1" })?
                * self.length_scale;
            let radius_y = if circle {
                radius_x
            } else {
                self.number(basis, "semi_axis_2")? * self.length_scale
            };
            if trimmed
                && (radius_x.partial_cmp(&EFF_ZERO) != Some(std::cmp::Ordering::Greater)
                    || radius_y.partial_cmp(&EFF_ZERO) != Some(std::cmp::Ordering::Greater)
                    || !radius_x.is_finite()
                    || !radius_y.is_finite())
            {
                return Err(failure(
                    "GeneralTrimmedConicRadiusRange",
                    basis,
                    GeneralFailureKind::Unsupported,
                ));
            }
            let angle = |point: Vec3| {
                let delta = point - frame.origin;
                (delta.dot(frame.y) / radius_y).atan2(delta.dot(frame.x) / radius_x)
            };
            let mut t0 = angle(a);
            let mut t1 = angle(b);
            if closed {
                t1 = t0 + TAU;
            } else {
                // atan2 returns one finite period; at most one unwrapping step
                // is required. Keep the source increasing-basis branch.
                while t1 <= t0 {
                    t1 += TAU;
                }
            }
            let trim = if trimmed {
                let trim = self.trimmed_conic(geometry, frame, radius_x, radius_y)?;
                maximum_consistency_mm = maximum_consistency_mm.max(trim.maximum_consistency_mm);
                if closed {
                    need(
                        trim.full,
                        "GeneralTrimmedClosedEdgeRequiresFullConic",
                        use_record,
                    )?;
                } else if !trim.full {
                    // A few conversion ulps select the ORIGINAL periodic trim
                    // branch. Physical tolerance does not enlarge its domain.
                    let inside = |t: f64| {
                        let roundoff =
                            32. * f64::EPSILON * 1_f64.max(trim.low.abs()).max(trim.high.abs());
                        let mut candidate = t + TAU * ((trim.low - t - roundoff) / TAU).ceil();
                        if candidate < trim.low && candidate >= trim.low - roundoff {
                            candidate = trim.low;
                        }
                        if candidate > trim.high && candidate <= trim.high + roundoff {
                            candidate = trim.high;
                        }
                        candidate
                    };
                    t0 = inside(angle(a));
                    t1 = inside(angle(b));
                    need(
                        t0 >= trim.low && t1 <= trim.high && t1 > t0,
                        "GeneralTrimmedEdgeOutsideSourceDomain",
                        use_record,
                    )?;
                }
                let field = self.field(geometry, "master_representation")?;
                let preferred = &field
                    .value()
                    .ok_or(Error::Invalid("missing general trim preference"))?
                    .text;
                let mut trace = ParameterMapping {
                    source_geometry: geometry,
                    source_use: use_record,
                    source_basis_geometry: Some(basis),
                    kind: "TrimmedConicSourceDomain",
                    equation: format!(
                        "canonical_trim_s_rad=sign*(basis_angle_rad-trim_1_rad); periodic_branch_when_full; source_s=canonical_trim_s_rad/angle_parameter_to_rad_when_established; native_t_is_rational_not_angle; preferred={preferred}"
                    ),
                    parameters: vec![
                        ("basis_trim_rad".into(), vec![trim.first, trim.last]),
                        ("canonical_trim_parameter_range_rad".into(), vec![0., trim.high - trim.low]),
                        (
                            "source_sense".into(),
                            vec![if self.boolean(geometry, "sense_agreement")? { 1. } else { -1. }],
                        ),
                        ("edge_basis_angle_branch_rad".into(), vec![t0, t1]),
                    ],
                };
                if let Some(scale) = trim.angle_scale {
                    trace
                        .parameters
                        .push(("angle_parameter_to_rad".into(), vec![scale]));
                }
                Some(trace)
            } else {
                None
            };
            let source_branch_begin = t0;
            if trimmed {
                let shift = TAU * (t0 / TAU).floor();
                t0 -= shift;
                t1 -= shift;
            }
            if t1 > TAU {
                t0 -= TAU;
                t1 -= TAU;
            }
            if let Some(mut trace) = trim {
                trace.parameters.push((
                    "native_chart_period_shift_rad".into(),
                    vec![source_branch_begin - t0],
                ));
                trace
                    .parameters
                    .push(("native_chart_angle_range_rad".into(), vec![t0, t1]));
                mappings.push(trace);
            }
            mapping.kind = "ConicRationalParameter";
            mapping.equation = "source_angle=atan2(dot(P(native_t)-origin,y)/ry,dot(P(native_t)-origin,x)/rx),unwrapped_to_angle_range_rad; native_t_is_not_angle".into();
            mapping.parameters = vec![
                ("angle_range_rad".into(), vec![t0, t1]),
                ("radii_mm".into(), vec![radius_x, radius_y]),
                (
                    "origin_mm".into(),
                    vec![frame.origin.x, frame.origin.y, frame.origin.z],
                ),
                ("x".into(), vec![frame.x.x, frame.x.y, frame.x.z]),
                ("y".into(), vec![frame.y.x, frame.y.y, frame.y.z]),
            ];
            // FullPeriodDegrees converts the phase once and adds exactly one
            // native period, avoiding a separately rounded span above 360.
            let first = t0 * 180. / PI;
            let last = if closed { first + 360. } else { t1 * 180. / PI };
            if trimmed
                && (last.partial_cmp(&first) != Some(std::cmp::Ordering::Greater)
                    || first < -360.
                    || last > 360.)
            {
                return Err(failure(
                    "GeneralTrimmedNativeAngleRangeUnestablished",
                    geometry,
                    GeneralFailureKind::Unsupported,
                ));
            }
            self.conic(
                basis,
                frame,
                radius_x,
                radius_y,
                Extent1d::from_min_max(first, last),
                trimmed,
                execution,
            )?
        } else if self.is(basis, "B_SPLINE_CURVE_WITH_KNOTS") {
            let (native, cancellation) =
                self.spline_curve(basis, use_record, definition_mm, execution)?;
            if let Some(mapping) = cancellation {
                mappings.push(mapping);
            }
            native
        } else {
            return Err(failure(
                "GeneralUnsupportedCurve",
                basis,
                GeneralFailureKind::Unsupported,
            ));
        };
        let domain = native.natural_interval();
        let mut begin = domain.min();
        let mut end = domain.max();
        if self.is(basis, "B_SPLINE_CURVE_WITH_KNOTS") {
            // Endpoint inversion chooses the interval only. Original spline
            // controls, weights, distinct knots and source t remain unchanged.
            mapping.kind = "SplineIdentityWithInvertedTrim";
            mapping.equation =
                "source_t=native_t; endpoint_inversion_only; defining_coefficients_unchanged"
                    .into();
            begin = self.curve_parameter(&native, a, begin, basis, execution)?;
            end = self.curve_parameter(&native, b, end, basis, execution)?;
            need(end > begin, "GeneralSplineParameterOrder", use_record)?;
        }
        mapping.parameters.push((
            "native_natural_interval".into(),
            vec![domain.min(), domain.max()],
        ));
        mapping
            .parameters
            .push(("native_edge_interval".into(), vec![begin, end]));
        mappings.push(mapping);
        let evaluate = |parameter: f64, point: Vec3, execution: &mut Execution| -> Result<Vec3> {
            execution.step()?;
            let actual = native.evaluate_point(self.kernel, parameter)?;
            let gap = norm(actual - point);
            need(
                gap.is_finite() && gap <= self.consistency_mm,
                "GeneralEdgeVertexGap",
                use_record,
            )?;
            Ok(actual)
        };
        let first = evaluate(begin, a, execution)?;
        let last = evaluate(end, b, execution)?;
        maximum_consistency_mm = maximum_consistency_mm
            .max(norm(first - a))
            .max(norm(last - b));
        let interval = Extent1d::from_min_max(begin, end);
        execution.step()?;
        let bounds = native
            .calculate_bounding_box(
                self.kernel,
                &interval,
                BoundingBoxRequest {
                    normal: true,
                    ..BoundingBoxRequest::default()
                },
                true,
            )?
            .normal
            .ok_or(Error::Invalid("missing general defining curve bound"))?;
        Ok(BuiltCurve {
            native: Some(native),
            interval,
            endpoints: [first, last],
            bounds,
            mappings,
            maximum_consistency_mm,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcore_step::{inspect, Limits};

    fn source(extra: &str) -> s::Snapshot {
        let fixture = include_str!("../../../solidcore-step/tests/fixtures/plane.step");
        let split = fixture.rfind("ENDSEC;").unwrap();
        let text = format!("{}\n{extra}\n{}", &fixture[..split], &fixture[split..]);
        let raw = inspect(text.into_bytes(), Limits::planar_fixtures());
        // These tests exercise isolated defining geometry. Full selected-part
        // qualification belongs to the public general entry, not this helper.
        let outcome = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                geometry_scope: s::GeometryScope::GeneralBrep,
                physical_context: Some("12".into()),
                check_geometry_dependencies: true,
                ..s::Options::default()
            },
        );
        outcome
            .snapshot
            .unwrap_or_else(|| panic!("source prerequisites failed: {:?}", outcome.diagnostics))
    }

    fn record(source: &s::Snapshot, id: &str) -> usize {
        source
            .source()
            .records()
            .iter()
            .position(|record| record.id == id)
            .unwrap()
    }

    #[test]
    fn translated_conic_trim_preserves_fixed_cpp_addition_order_at_tight_budget() {
        let source = source(
            "#900=CIRCLE('',#133,1.);\n\
             #901=CARTESIAN_POINT('',(100000000.46656057,100000000.88448925,0.));\n\
             #902=TRIMMED_CURVE('',#900,(PARAMETER_VALUE(0.3),#901),(PARAMETER_VALUE(0.8)),.T.,.PARAMETER.);",
        );
        let kernel = Kernel::default();
        let geometry = Geometry::new(&source, &kernel, 1e-9, 1e-9).unwrap();
        // Isolated defining-geometry control: the caller-supplied frame is the
        // private helper's input, not an assertion of full selected-part import.
        let q = 0.5_f64.sqrt();
        let frame = Frame {
            origin: Vec3::new(1e8, 1e8, 0.),
            x: Vec3::new(q, q, 0.),
            y: Vec3::new(-q, q, 0.),
            z: Vec3::Z,
        };
        let expected = Vec3::new(100_000_000.466_560_57, 100_000_000.884_489_25, 0.);
        assert_eq!(source_conic_point(frame, 1., 1., 0.3), expected);
        let old = frame.origin + frame.x * 0.3_f64.cos() + frame.y * 0.3_f64.sin();
        assert!(norm(old - expected) > 1e-9);
        let trim = geometry
            .trimmed_conic(record(&source, "902"), frame, 1., 1.)
            .unwrap();
        assert_eq!(trim.maximum_consistency_mm, 0.);
        assert_eq!(trim.first, 0.3);
        assert_eq!(trim.last, 0.8);
    }

    #[test]
    fn physical_angle_conversion_consumes_only_the_matching_sealed_source_proof() {
        let text = include_str!("../../../solidcore-step/tests/fixtures/plane.step")
            .replace(
                "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}",
                "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}",
            )
            .replace(
                "#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
                "#11=(CONVERSION_BASED_UNIT('not a scale inference',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());",
            )
            .replace("ENDSEC;\nEND-ISO", "#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#901);\n#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n#902=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND-ISO");
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        let options = s::Options {
            geometry_scope: s::GeometryScope::GeneralBrep,
            check_geometry_dependencies: true,
            check_part_and_closure: true,
            ..Default::default()
        };
        assert_eq!(
            s::check(&raw, s::Limits::fixtures(), &options).status,
            s::Status::Rejected
        );
        let checked = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                development_unit_policy:
                    s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions,
                ..options
            },
        );
        let source = checked.snapshot.unwrap();
        let kernel = Kernel::default();
        let geometry = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
        let actual = geometry
            .angle_scale(record(&source, "11"), "missing angle proof")
            .unwrap();
        assert_eq!(actual, 0.0174532925199433);
        assert_eq!(source.source().bytes(), text.as_bytes());
        assert!(!source
            .development_unit_receipt()
            .unwrap()
            .source_syntax_verified());
    }

    #[test]
    fn source_line_is_projected_without_fitting_and_native_calls_are_precharged() {
        let source = source("");
        let kernel = Kernel::default();
        let geometry = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
        let origin = geometry.position(record(&source, "100")).unwrap();
        let last = geometry.position(record(&source, "101")).unwrap();
        let displacement = Vec3::new(0., 0., 4e-7);
        let build = |execution: &mut Execution| {
            geometry.make_curve(
                record(&source, "202"),
                record(&source, "203"),
                origin + displacement,
                last + displacement,
                false,
                1e-9,
                execution,
            )
        };
        let mut execution = Execution { used: 0, max: 4 };
        let built = build(&mut execution).unwrap();
        assert_eq!(execution.used, 4);
        assert_eq!(built.endpoints[0].z, 5.);
        assert_eq!(built.endpoints[1].z, 5.);
        assert!(norm(built.endpoints[0] - origin) < 1e-12);
        assert!(norm(built.endpoints[1] - last) < 1e-12);
        assert!(built.maximum_consistency_mm > 3e-7);
        let mapping = built
            .mappings
            .iter()
            .find(|mapping| mapping.kind == "LineAffineParameter")
            .unwrap();
        let values = &mapping
            .parameters
            .iter()
            .find(|(name, _)| name == "source_parameter_at_native_ends")
            .unwrap()
            .1;
        assert!(values[0].abs() < 1e-12 && (values[1] - 1.).abs() < 1e-12);
        for maximum in 0..4 {
            let mut limited = Execution {
                used: 0,
                max: maximum,
            };
            assert!(matches!(build(&mut limited), Err(Error::ResourceLimit)));
            assert_eq!(limited.used, maximum);
        }
    }

    #[test]
    fn conic_cuts_preserve_basis_sense_and_native_nonangular_parameter_mapping() {
        let source = source(
            "#900=CIRCLE('',#133,2.);\n\
             #901=ELLIPSE('',#133,2.,1.);\n\
             #902=TRIMMED_CURVE('',#900,(PARAMETER_VALUE(1.5707963267948966)),(PARAMETER_VALUE(0.)),.F.,.PARAMETER.);",
        );
        let kernel = Kernel::default();
        let geometry = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
        let frame = geometry.placement(record(&source, "133")).unwrap();
        for (id, radius_y) in [("900", 2.), ("901", 1.), ("902", 2.)] {
            let a = frame.origin + frame.x * 2.;
            let b = frame.origin + frame.y * radius_y;
            let mut execution = Execution { used: 0, max: 4 };
            let built = geometry
                .make_curve(
                    record(&source, id),
                    record(&source, "203"),
                    a,
                    b,
                    false,
                    1e-9,
                    &mut execution,
                )
                .unwrap();
            assert_eq!(execution.used, 4);
            assert!(norm(built.endpoints[0] - a) < 1e-12);
            assert!(norm(built.endpoints[1] - b) < 1e-12);
            assert_ne!(built.interval.max(), std::f64::consts::FRAC_PI_2);
            assert!(built
                .mappings
                .iter()
                .any(|mapping| mapping.kind == "ConicRationalParameter"));
            if id == "902" {
                let trace = built
                    .mappings
                    .iter()
                    .find(|mapping| mapping.kind == "TrimmedConicSourceDomain")
                    .unwrap();
                assert_eq!(trace.source_basis_geometry, Some(record(&source, "900")));
                assert!(trace
                    .parameters
                    .iter()
                    .any(|(name, values)| name == "source_sense" && values == &[-1.]));
            }
        }
        let a = frame.origin + frame.x * 2.;
        let mut execution = Execution { used: 0, max: 4 };
        let closed = geometry
            .make_curve(
                record(&source, "900"),
                record(&source, "203"),
                a,
                a,
                true,
                1e-9,
                &mut execution,
            )
            .unwrap();
        assert!(norm(closed.endpoints[0] - closed.endpoints[1]) < 1e-12);
    }

    #[test]
    fn spline_endpoint_inversion_preserves_interior_parameters_and_strict_consistency() {
        // Independently designed polynomial: P(s)=(2s,2s(1-s),0),
        // source t=2+3s. At t=2.75/4.25 the exact points are
        // (0.5,0.375,0)/(1.5,0.375,0), strictly inside its natural [2,5].
        // Neither endpoint can use CurveParameter's natural-end fast path.
        let source = source(
            "#910=CARTESIAN_POINT('',(0.,0.,0.));\n\
             #911=CARTESIAN_POINT('',(1.,1.,0.));\n\
             #912=CARTESIAN_POINT('',(2.,0.,0.));\n\
             #913=B_SPLINE_CURVE_WITH_KNOTS('',2,(#910,#911,#912),.UNSPECIFIED.,.F.,.F.,(3,3),(2.,5.),.UNSPECIFIED.);",
        );
        let kernel = Kernel::default();
        let consistency = 1e-6;
        let geometry = Geometry::new(&source, &kernel, consistency, 1e-6).unwrap();
        let a = Vec3::new(0.5, 0.375, 0.);
        let b = Vec3::new(1.5, 0.375, 0.);
        let mut execution = Execution { used: 0, max: 100 };
        let built = geometry
            .make_curve(
                record(&source, "913"),
                record(&source, "203"),
                a,
                b,
                false,
                1e-9,
                &mut execution,
            )
            .unwrap();
        assert!((built.interval.min() - 2.75).abs() < 1e-9);
        assert!((built.interval.max() - 4.25).abs() < 1e-9);
        assert!(norm(built.endpoints[0] - a) <= consistency);
        assert!(norm(built.endpoints[1] - b) <= consistency);
        let native = built.native.as_ref().unwrap();
        assert!(matches!(
            native.kind(),
            solidcore::curve::curve::CurveKind::BSpline(_)
        ));
        assert_eq!(native.natural_interval().min(), 2.);
        assert_eq!(native.natural_interval().max(), 5.);
        // The whole defining curve is in z=0, so every curve point is at
        // least 2*consistency away from either displaced topology endpoint.
        // This is an analytic lower bound, not a sampled residual estimate.
        let displacement = Vec3::new(0., 0., 2. * consistency);
        let mut rejected_execution = Execution { used: 0, max: 100 };
        assert!(matches!(
            geometry.make_curve(
                record(&source, "913"),
                record(&source, "203"),
                a + displacement,
                b + displacement,
                false,
                1e-9,
                &mut rejected_execution,
            ),
            Err(Error::General {
                key: "GeneralSplineEndpointInversion",
                kind: GeneralFailureKind::Postcheck,
                ..
            })
        ));
    }

    #[test]
    fn rational_spline_retains_source_knots_weights_and_euclidean_controls() {
        let source = source(
            "#910=CARTESIAN_POINT('',(0.,0.,0.));\n\
             #911=CARTESIAN_POINT('',(1.,1.,0.));\n\
             #912=CARTESIAN_POINT('',(2.,0.,0.));\n\
             #913=(BOUNDED_CURVE() B_SPLINE_CURVE(2,(#910,#911,#912),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,3),(2.,5.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,2.,1.)) REPRESENTATION_ITEM(''));",
        );
        let kernel = Kernel::default();
        let geometry = Geometry::new(&source, &kernel, 1e-6, 1e-6).unwrap();
        let mut execution = Execution { used: 0, max: 6 };
        let built = geometry
            .make_curve(
                record(&source, "913"),
                record(&source, "203"),
                Vec3::ZERO,
                Vec3::new(2., 0., 0.),
                false,
                1e-9,
                &mut execution,
            )
            .unwrap();
        assert_eq!(execution.used, 6);
        assert_eq!(built.interval.min(), 2.);
        assert_eq!(built.interval.max(), 5.);
        let point = built
            .native
            .as_ref()
            .unwrap()
            .evaluate_point(&kernel, 3.5)
            .unwrap();
        assert!(norm(point - Vec3::new(1., 2. / 3., 0.)) < 1e-12);
        let solidcore::curve::curve::CurveKind::BSpline(curve) =
            built.native.as_ref().unwrap().kind()
        else {
            panic!("defining rational spline changed native kind")
        };
        assert!(curve.is_rational().unwrap());
        let controls = curve.gw_nurb().unwrap().cpts();
        assert_eq!(
            controls.iter().map(|point| point.w).collect::<Vec<_>>(),
            [1., 2., 1.]
        );
        assert_eq!(controls[1].x / controls[1].w, 1.);
        assert_eq!(controls[1].y / controls[1].w, 1.);
        assert!(built
            .mappings
            .iter()
            .any(|mapping| mapping.kind == "SplineIdentityWithInvertedTrim"));
    }
}
