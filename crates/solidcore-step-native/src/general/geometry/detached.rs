// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native detached curve definitions, without invented `BRep` endpoints.
//!
//! Upstream: maintained `StepImportCurvePointDefinitions.cpp::AddCurve` at
//! `200aa2f`. Source trim parameters determine the whole native definition;
//! LINE parameters are dimensionless, conic physical angles retain their unit
//! proof, and explicit-knot splines keep their complete original natural domain.

use super::{
    finite, need, norm, numeric, source_conic_point, ConicTrim, Geometry, ParameterMapping,
};
use crate::general::associated::AssociatedRole;
use crate::{Error, Execution, GeneralFailureKind, Result};
use solidcore::curve::{curve::Curve, line::Line};
use solidcore::geom::extent1d::Extent1d;
use solidcore::math::EFF_ZERO;
use solidcore_step::ValueKind;
use std::f64::consts::{PI, TAU};

pub(in crate::general) struct DetachedCurve {
    pub native: Curve,
    pub basis_record: usize,
    pub source_forward_in_native: bool,
    pub mappings: Vec<ParameterMapping>,
    pub source_residual_mm: Option<f64>,
    pub endpoint_error_mm: Option<f64>,
}

impl Geometry<'_> {
    /// Source-only finite definition path. Never call the BRep-edge factory with
    /// manufactured topology anchors: they would replace trim/natural domains
    /// and could hide unsupported independent source geometry.
    pub(in crate::general) fn make_detached_curve(
        &self,
        record: usize,
        role: AssociatedRole,
        definition_mm: f64,
        execution: &mut Execution,
    ) -> Result<DetachedCurve> {
        let trimmed = self.is(record, "TRIMMED_CURVE");
        let basis = if trimmed {
            self.reference(record, "basis_curve")?
        } else {
            record
        };
        let source_forward_in_native = !trimmed || self.boolean(record, "sense_agreement")?;
        let measured_source_trim = if trimmed {
            ["trim_1", "trim_2"]
                .into_iter()
                .try_fold(false, |found, name| {
                    let field = self.field(record, name)?;
                    let value = field
                        .value()
                        .ok_or(Error::Invalid("missing detached trim set"))?;
                    Ok::<_, Error>(
                        found
                            || value
                                .children
                                .iter()
                                .any(|v| v.kind == ValueKind::Reference),
                    )
                })?
        } else {
            false
        };
        let mut mappings = Vec::new();
        let mut mapping = ParameterMapping {
            source_geometry: record,
            source_use: record,
            source_basis_geometry: Some(basis),
            kind: "",
            equation: String::new(),
            parameters: Vec::new(),
        };
        let mut source_consistency: f64 = 0.;
        let mut expected_ends = None;
        let curve = if self.is(basis, "LINE") && trimmed {
            // ISO 10303-42:2003 4.4.40, and the fixed AddCurve contract:
            // P(t)=origin+direction*(magnitude*t). Scale VECTOR once in mm;
            // a source LINE trim parameter t has no physical length unit.
            let origin = self.position(self.reference(basis, "pnt")?)?;
            let vector = self.reference(basis, "dir")?;
            let direction = self.direction(self.reference(vector, "orientation")?)?;
            let magnitude = self.number(vector, "magnitude")? * self.length_scale();
            need(
                magnitude.is_finite() && magnitude > 0.,
                "GeneralWireLineMagnitude",
                record,
            )?;
            let evaluate = |t| origin + direction * (magnitude * t);
            let master = self.field(record, "master_representation")?;
            let preferred = &master
                .value()
                .ok_or(Error::Invalid("missing detached trim master"))?
                .text;
            let mut cut = |name| -> Result<f64> {
                let mut parameter = None;
                let mut point_parameter = None;
                let mut point = None;
                let field = self.field(record, name)?;
                let value = field
                    .value()
                    .ok_or(Error::Invalid("missing detached line trim"))?;
                for value in &value.children {
                    if value.kind == ValueKind::Reference {
                        let p = self.position(self.reference_value(value)?)?;
                        let t = (p - origin).dot(direction) / magnitude;
                        let gap = norm(evaluate(t) - p);
                        source_consistency = source_consistency.max(gap);
                        need(
                            gap.is_finite() && gap <= self.consistency_mm,
                            "GeneralWireTrimPointOffBasis",
                            record,
                        )?;
                        point = Some(p);
                        point_parameter = Some(t);
                    } else {
                        let [number] = value.children.as_slice() else {
                            return Err(Error::Invalid(
                                "detached line trim requires one typed parameter",
                            ));
                        };
                        parameter = Some(numeric(number, record)?);
                    }
                }
                if let (Some(t), Some(p)) = (parameter, point) {
                    let gap = norm(evaluate(t) - p);
                    source_consistency = source_consistency.max(gap);
                    need(
                        gap.is_finite() && gap <= self.consistency_mm,
                        "GeneralTrimmedRepresentationsConflict",
                        record,
                    )?;
                }
                if preferred == "CARTESIAN" && point_parameter.is_some() {
                    point_parameter.ok_or(Error::Invalid("missing detached cartesian trim"))
                } else {
                    parameter
                        .or(point_parameter)
                        .ok_or(Error::Invalid("empty detached line trim"))
                }
            };
            let first = cut("trim_1")?;
            let last = cut("trim_2")?;
            need(
                if source_forward_in_native {
                    last > first
                } else {
                    first > last
                },
                "GeneralWireLineTrimOrder",
                record,
            )?;
            let low = first.min(last);
            let high = first.max(last);
            let ends = [evaluate(low), evaluate(high)];
            need(
                ends.iter().all(|p| finite(*p)),
                "GeneralWireLineRange",
                record,
            )?;
            expected_ends = Some(ends);
            mapping.kind = match role {
                AssociatedRole::Wireframe => "WireframeTrimmedLine",
                AssociatedRole::Supporting => "SupportingTrimmedLine",
            };
            mapping.equation = "basis_t=lerp(basis_range,(native_t-native_begin)/(native_end-native_begin)); source_s=sign*(basis_t-trim_1)".into();
            mapping.parameters = vec![
                ("basis_trim_parameters".into(), vec![first, last]),
                ("basis_range".into(), vec![low, high]),
                ("vector_magnitude_mm".into(), vec![magnitude]),
            ];
            execution.step()?;
            // Use the checked native factory, as the maintained owner does:
            // representational collapse of distinct source cuts must fail,
            // rather than fabricate a zero-length independent curve.
            Line::create_line_segment(3, ends[0], ends[1], None)?.into()
        } else if self.is(basis, "CIRCLE") || self.is(basis, "ELLIPSE") {
            let frame = self.placement(self.reference(basis, "position")?)?;
            let circle = self.is(basis, "CIRCLE");
            let rx = self.number(basis, if circle { "radius" } else { "semi_axis_1" })?
                * self.length_scale();
            let ry = if circle {
                rx
            } else {
                self.number(basis, "semi_axis_2")? * self.length_scale()
            };
            need(
                rx.is_finite() && ry.is_finite() && rx > EFF_ZERO && ry > EFF_ZERO,
                "GeneralWireConicRadiusRange",
                record,
            )?;
            let trim = if trimmed {
                self.trimmed_conic(record, frame, rx, ry)?
            } else {
                ConicTrim {
                    first: 0.,
                    last: TAU,
                    low: 0.,
                    high: TAU,
                    full: true,
                    angle_scale: None,
                    maximum_consistency_mm: 0.,
                }
            };
            source_consistency = trim.maximum_consistency_mm;
            let shift = TAU * (trim.low / TAU).floor();
            let mut low = trim.low - shift;
            let mut high = trim.high - shift;
            if high > TAU {
                low -= TAU;
                high -= TAU;
            }
            let begin = low * 180. / PI;
            let end = if trim.full {
                begin + 360.
            } else {
                high * 180. / PI
            };
            need(
                end > begin && begin >= -360. && end <= 360.,
                "GeneralWireConicAngleRange",
                record,
            )?;
            expected_ends =
                Some([trim.low, trim.high].map(|a| source_conic_point(frame, rx, ry, a)));
            mapping.kind = match role {
                AssociatedRole::Wireframe => "WireframeConicRationalParameter",
                AssociatedRole::Supporting => "SupportingConicRationalParameter",
            };
            mapping.equation = "basis_angle_rad=atan2(dot(P(native_t)-origin,y)/ry,dot(P(native_t)-origin,x)/rx),unwrapped_to_basis_range_rad; native_t_is_not_angle".into();
            mapping.parameters = vec![
                ("basis_trim_rad".into(), vec![trim.first, trim.last]),
                ("basis_range_rad".into(), vec![trim.low, trim.high]),
                ("native_chart_range_rad".into(), vec![low, high]),
                ("radii_mm".into(), vec![rx, ry]),
                (
                    "origin_mm".into(),
                    vec![frame.origin.x, frame.origin.y, frame.origin.z],
                ),
                ("x".into(), vec![frame.x.x, frame.x.y, frame.x.z]),
                ("y".into(), vec![frame.y.x, frame.y.y, frame.y.z]),
            ];
            if let Some(scale) = trim.angle_scale {
                mapping
                    .parameters
                    .push(("angle_parameter_to_rad".into(), vec![scale]));
            }
            self.conic(
                basis,
                frame,
                rx,
                ry,
                Extent1d::from_min_max(begin, end),
                true,
                execution,
            )?
        } else if !trimmed && self.is(basis, "B_SPLINE_CURVE_WITH_KNOTS") {
            let (native, weight_mapping) =
                self.spline_curve(basis, record, definition_mm, execution)?;
            if let Some(weight_mapping) = weight_mapping {
                mappings.push(weight_mapping);
            }
            mapping.kind = match role {
                AssociatedRole::Wireframe => "WireframeSplineIdentity",
                AssociatedRole::Supporting => "SupportingSplineIdentity",
            };
            mapping.equation = "source_t=native_t; entire_source_natural_interval; defining_coefficients_unchanged".into();
            native
        } else {
            return Err(Error::General {
                key: "GeneralDetachedCurveNotYetConstructed",
                record,
                kind: GeneralFailureKind::Unsupported,
            });
        };
        need(curve.dim() == 3, "GeneralWireCurveAllocation", record)?;
        let domain = curve.natural_interval();
        need(
            domain.min().is_finite() && domain.max().is_finite() && domain.max() > domain.min(),
            "GeneralWireNativeDomain",
            record,
        )?;
        let endpoint_error_mm = if let Some(ends) = expected_ends {
            let mut maximum: f64 = 0.;
            for (t, expected) in [domain.min(), domain.max()].into_iter().zip(ends) {
                execution.step()?;
                let actual = curve.evaluate_point(self.kernel(), t)?;
                let gap = norm(actual - expected);
                maximum = maximum.max(gap);
                need(
                    gap.is_finite() && gap <= definition_mm,
                    "GeneralWireDefinitionEndpointGap",
                    record,
                )?;
            }
            Some(maximum)
        } else {
            None
        };
        mapping.parameters.push((
            "native_natural_interval".into(),
            vec![domain.min(), domain.max()],
        ));
        mapping.parameters.push((
            "source_forward_in_native".into(),
            vec![if source_forward_in_native { 1. } else { -1. }],
        ));
        mappings.push(mapping);
        Ok(DetachedCurve {
            native: curve,
            basis_record: basis,
            source_forward_in_native,
            mappings,
            source_residual_mm: measured_source_trim.then_some(source_consistency),
            endpoint_error_mm,
        })
    }
}
