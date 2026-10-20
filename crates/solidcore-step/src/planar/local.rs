// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Named local clause evaluation and subject-specific work/evidence drivers.
//! Upstream: `StepPlanarRules.cpp` LocalValue/TypeValue/Local at `d8ed45a`.
//! Deviations: checked bindings and explicit fallible predicates. Entity rows
//! survive failed evaluation; TYPE rows are inserted only after numeric success.
use super::catalog::{self, Clause, LocalId as L};
use super::validator::{cross, squared, Checked, Failure, Validator};
use super::{Logical, Rational as R, RuleEvidence, Status};
use crate::{Value, ValueKind};

impl Validator<'_, '_> {
    pub(super) fn unit_dimensions(&self, r: usize, witnesses: &mut Vec<usize>) -> Checked<[R; 7]> {
        for unit in &self.sem.units().si_definitions {
            if unit.record_index == r {
                let mut values = [R::ZERO; 7];
                for (value, exponent) in values.iter_mut().zip(unit.dimensions) {
                    *value = R::new(i64::from(exponent), 1)?;
                }
                return Ok(values);
            }
        }
        if !self.is(r, "CONVERSION_BASED_UNIT") {
            return Err(Failure::Numeric("UnsupportedNamedUnitDimensions"));
        }
        let dims = self.reference(r, "dimensions")?;
        for witness in [r, dims] {
            if !witnesses.contains(&witness) {
                witnesses.push(witness);
            }
        }
        let mut values = [R::ZERO; 7];
        for (value, name) in values.iter_mut().zip([
            "length_exponent",
            "mass_exponent",
            "time_exponent",
            "electric_current_exponent",
            "thermodynamic_temperature_exponent",
            "amount_of_substance_exponent",
            "luminous_intensity_exponent",
        ]) {
            *value = R::decimal(&self.field(dims, name)?.text)?;
        }
        Ok(values)
    }
    fn forall(
        &mut self,
        values: Vec<usize>,
        mut predicate: impl FnMut(&mut Self, usize) -> Checked<bool>,
    ) -> Checked<bool> {
        let mut value = true;
        for member in values {
            self.tick(member)?;
            value = predicate(self, member)? && value;
        }
        Ok(value)
    }
    fn local_value(&mut self, c: Clause, r: usize, e: &mut RuleEvidence) -> Checked<bool> {
        match c.id {
            L::ProductDefinitionFormationUr1 | L::ProductDefinitionShapeUr1 => {
                e.proof = "Singleton full-file supertype population; pairwise UNIQUE has no distinct pair".into();
                let pop = self.population(c.type_name)?;
                let valid = pop.len() == 1;
                e.populations.insert(c.type_name.into(), pop);
                Ok(valid)
            }
            L::ApplicationContextContextElements
            | L::RepresentationContextRepresentationsInContext => {
                let ac = c.id == L::ApplicationContextContextElements;
                let uses = self
                    .sem
                    .used_in(
                        &self.record(r)?.id,
                        if ac {
                            "APPLICATION_CONTEXT_ELEMENT"
                        } else {
                            "REPRESENTATION"
                        },
                        if ac {
                            "frame_of_reference"
                        } else {
                            "context_of_items"
                        },
                    )
                    .ok_or(Failure::Infrastructure)?;
                for index in &uses {
                    e.witnesses.push(
                        self.sem
                            .source()
                            .references()
                            .get(*index)
                            .ok_or(Failure::Infrastructure)?
                            .record_index,
                    );
                }
                e.facts.insert("count".into(), uses.len().to_string());
                e.proof =
                    "Complete DATA reverse index with declaring role; SET[1:?] cardinality".into();
                Ok(!uses.is_empty())
            }
            L::ApplicationContextWr1
            | L::ApplicationContextWr2
            | L::ProductCategoryWr1
            | L::ProductDefinitionWr1
            | L::PropertyDefinitionWr1
            | L::PropertyDefinitionRepresentationWr1
            | L::PropertyDefinitionRepresentationWr2
            | L::RepresentationWr1
            | L::RepresentationWr2 => {
                e.proof = "ClosedProfileTypeExclusion: ID_ATTRIBUTE/DESCRIPTION_ATTRIBUTE/NAME_ATTRIBUTE; inverse count 0 <= 1".into();
                e.facts.insert("assignment_count".into(), "0".into());
                e.facts.insert(
                    "derived_getter".into(),
                    "INDETERMINATE_VALUE(?) from zero assignments, not an empty string".into(),
                );
                Ok(true)
            }
            L::RepresentationItemWr1 => {
                e.witnesses = vec![self.role("representation")?];
                let used = self
                    .sem
                    .used_by_representation(r)
                    .ok_or(Failure::Infrastructure)?;
                e.facts.insert(
                    "using_representation_count".into(),
                    if used { "1" } else { "0" }.into(),
                );
                e.proof = "using_representations: full RI-filtered transitive uses plus REPRESENTATION.items, management users excluded".into();
                Ok(used)
            }
            L::GeometricRepresentationItemWr1 => {
                e.witnesses = vec![self.role("representation")?, self.role("unit_context")?];
                e.proof = "Single admitted representation; every collected using representation has a GRC context".into();
                Ok(self.is(
                    self.role("unit_context")?,
                    "GEOMETRIC_REPRESENTATION_CONTEXT",
                ))
            }
            L::DirectionWr1 => {
                e.proof =
                    "QUERY of source ratios <> 0, using exact decimal values, no epsilon".into();
                for value in &self.field(r, "direction_ratios")?.children {
                    if R::decimal(&value.text)? != R::ZERO {
                        return Ok(true);
                    }
                }
                Ok(false)
            }
            L::VectorWr1 => {
                e.proof = "Source magnitude >= 0; zero is allowed by this WHERE".into();
                Ok(R::decimal(&self.field(r, "magnitude")?.text)? >= R::ZERO)
            }
            L::Axis2Placement3dWr1
            | L::Axis2Placement3dWr2
            | L::Axis2Placement3dWr3
            | L::Axis2Placement3dWr4 => {
                e.proof = "Original optional predicates; WR4 requires cross magnitude > 0 only when both fields exist".into();
                match c.id {
                    L::Axis2Placement3dWr1 => Ok(self
                        .field(self.reference(r, "location")?, "coordinates")?
                        .children
                        .len()
                        == 3),
                    L::Axis2Placement3dWr2 => Ok(self.field(r, "axis")?.kind == ValueKind::Unset
                        || self
                            .field(self.reference(r, "axis")?, "direction_ratios")?
                            .children
                            .len()
                            == 3),
                    L::Axis2Placement3dWr3 => Ok(self.field(r, "ref_direction")?.kind
                        == ValueKind::Unset
                        || self
                            .field(self.reference(r, "ref_direction")?, "direction_ratios")?
                            .children
                            .len()
                            == 3),
                    L::Axis2Placement3dWr4 => Ok(self.field(r, "axis")?.kind == ValueKind::Unset
                        || self.field(r, "ref_direction")?.kind == ValueKind::Unset
                        || squared(cross(
                            self.direction(self.reference(r, "axis")?)?,
                            self.direction(self.reference(r, "ref_direction")?)?,
                        )?)? > R::ZERO),
                    _ => Err(Failure::Infrastructure),
                }
            }
            L::LineWr1 => {
                e.proof =
                    "Dimension of dir.orientation ratios equals number of pnt coordinates".into();
                Ok(self
                    .field(
                        self.reference(self.reference(r, "dir")?, "orientation")?,
                        "direction_ratios",
                    )?
                    .children
                    .len()
                    == self
                        .field(self.reference(r, "pnt")?, "coordinates")?
                        .children
                        .len())
            }
            L::OrientedEdgeWr1 => {
                e.proof = "edge_element TYPEOF excludes ORIENTED_EDGE".into();
                Ok(!self.is(self.reference(r, "edge_element")?, "ORIENTED_EDGE"))
            }
            L::EdgeLoopWr1 | L::PathWr1 => {
                let edges = self.refs(r, "edge_list")?;
                e.witnesses.clone_from(&edges);
                if c.id == L::EdgeLoopWr1 {
                    let start = self.ends(*edges.first().ok_or(Failure::Infrastructure)?)?.0;
                    let end = self.ends(*edges.last().ok_or(Failure::Infrastructure)?)?.1;
                    e.facts.insert("ne".into(), edges.len().to_string());
                    e.facts
                        .insert("first_start_id".into(), self.record(start)?.id.clone());
                    e.facts
                        .insert("last_end_id".into(), self.record(end)?.id.clone());
                    e.proof =
                        "First derived OE.start :=: last derived OE.end; instance identity".into();
                    Ok(start == end)
                } else {
                    let mut bad = 0;
                    for pair in edges.windows(2) {
                        let [a, b] = pair else {
                            return Err(Failure::Infrastructure);
                        };
                        self.tick(r)?;
                        bad += usize::from(self.ends(*a)?.1 != self.ends(*b)?.0);
                    }
                    e.facts
                        .insert("adjacent_identity_mismatches".into(), bad.to_string());
                    e.proof = "path_head_to_tail checks adjacent identities only; final closure belongs to EDGE_LOOP.WR1".into();
                    Ok(bad == 0)
                }
            }
            L::FaceWr1 | L::FaceWr2 => {
                let bounds = self.refs(r, "bounds")?;
                let mut outer = 0;
                let mut poly = 0;
                for bound in &bounds {
                    outer += usize::from(self.is(*bound, "FACE_OUTER_BOUND"));
                    poly += usize::from(self.has(self.reference(*bound, "bound")?, "POLY_LOOP")?);
                }
                e.proof = if c.id == L::FaceWr1 {
                    "mixed_loop_type_set: all actual bounds use EDGE_LOOP"
                } else {
                    "Explicit FACE_OUTER_BOUND count <= 1; not the profile's count = 1"
                }
                .into();
                e.facts.insert("outer_count".into(), outer.to_string());
                e.facts.insert("poly_loop_count".into(), poly.to_string());
                Ok(if c.id == L::FaceWr1 {
                    poly == 0 || poly == bounds.len()
                } else {
                    outer <= 1
                })
            }
            L::FaceSurfaceWr1 => {
                e.proof = "Direct PLANE TYPEOF excludes ORIENTED_SURFACE".into();
                Ok(!self.has(self.reference(r, "face_geometry")?, "ORIENTED_SURFACE")?)
            }
            L::AdvancedFaceWr1
            | L::AdvancedFaceWr2
            | L::AdvancedFaceWr3
            | L::AdvancedFaceWr4
            | L::AdvancedFaceWr5
            | L::AdvancedFaceWr6
            | L::AdvancedFaceWr7
            | L::AdvancedFaceWr8
            | L::AdvancedFaceWr9
            | L::AdvancedFaceWr10 => {
                let geometry = self.reference(r, "face_geometry")?;
                let mut oe = Vec::new();
                for bound in self.refs(r, "bounds")? {
                    oe.extend(self.refs(self.reference(bound, "bound")?, "edge_list")?);
                }
                e.witnesses = self.refs(r, "bounds")?;
                e.witnesses.push(geometry);
                e.proof = "Actual direct PLANE / EDGE_LOOP / EDGE_CURVE / LINE / VERTEX_POINT / CARTESIAN_POINT types, including ancestors".into();
                match c.id {
                    L::AdvancedFaceWr1 => Ok(self.count_types(
                        geometry,
                        &["ELEMENTARY_SURFACE", "B_SPLINE_SURFACE", "SWEPT_SURFACE"],
                    )? == 1),
                    L::AdvancedFaceWr2 => self.forall(oe, |s, x| {
                        Ok(s.is(s.reference(x, "edge_element")?, "EDGE_CURVE"))
                    }),
                    L::AdvancedFaceWr3 => self.forall(oe, |s, x| {
                        Ok(s.count_types(
                            s.reference(s.reference(x, "edge_element")?, "edge_geometry")?,
                            &[
                                "LINE",
                                "CONIC",
                                "POLYLINE",
                                "SURFACE_CURVE",
                                "B_SPLINE_CURVE",
                            ],
                        )? == 1)
                    }),
                    L::AdvancedFaceWr4 => self.forall(oe, |s, x| {
                        let (a, b) = s.ends(x)?;
                        Ok(s.is(a, "VERTEX_POINT")
                            && s.is(b, "VERTEX_POINT")
                            && s.is(s.reference(a, "vertex_geometry")?, "CARTESIAN_POINT")
                            && s.is(s.reference(b, "vertex_geometry")?, "CARTESIAN_POINT"))
                    }),
                    L::AdvancedFaceWr5 => self.forall(self.refs(r, "bounds")?, |s, b| {
                        Ok(!s.has(s.reference(b, "bound")?, "ORIENTED_PATH")?)
                    }),
                    L::AdvancedFaceWr6 | L::AdvancedFaceWr10 => {
                        e.proof = "No SWEPT_SURFACE; for WR10 also no POLYLINE edge geometry, proven by actual types".into();
                        if self.has(geometry, "SWEPT_SURFACE")? {
                            Ok(false)
                        } else {
                            self.forall(oe, |s, x| {
                                Ok(!s.has(
                                    s.reference(s.reference(x, "edge_element")?, "edge_geometry")?,
                                    "POLYLINE",
                                )?)
                            })
                        }
                    }
                    L::AdvancedFaceWr7 => {
                        e.proof =
                            "Actual bounds have no VERTEX_LOOP, so failing QUERY is empty".into();
                        self.forall(self.refs(r, "bounds")?, |s, b| {
                            Ok(!s.has(s.reference(b, "bound")?, "VERTEX_LOOP")?)
                        })
                    }
                    L::AdvancedFaceWr8 => self.forall(self.refs(r, "bounds")?, |s, b| {
                        Ok(
                            s.count_types(s.reference(b, "bound")?, &["EDGE_LOOP", "VERTEX_LOOP"])?
                                == 1,
                        )
                    }),
                    L::AdvancedFaceWr9 => {
                        e.proof =
                            "All actual EC geometries are direct LINE; SURFACE_CURVE query empty"
                                .into();
                        self.forall(oe, |s, x| {
                            Ok(!s.has(
                                s.reference(s.reference(x, "edge_element")?, "edge_geometry")?,
                                "SURFACE_CURVE",
                            )?)
                        })
                    }
                    _ => Err(Failure::Infrastructure),
                }
            }
            L::AdvancedBrepShapeRepresentationWr1
            | L::AdvancedBrepShapeRepresentationWr2
            | L::AdvancedBrepShapeRepresentationWr3
            | L::AdvancedBrepShapeRepresentationWr4
            | L::AdvancedBrepShapeRepresentationWr5
            | L::AdvancedBrepShapeRepresentationWr6
            | L::ManifoldSurfaceShapeRepresentationWr1
            | L::ManifoldSurfaceShapeRepresentationWr2
            | L::ManifoldSurfaceShapeRepresentationWr3
            | L::ManifoldSurfaceShapeRepresentationWr4
            | L::ManifoldSurfaceShapeRepresentationWr5
            | L::ManifoldSurfaceShapeRepresentationWr6
            | L::ManifoldSurfaceShapeRepresentationWr7
            | L::ManifoldSurfaceShapeRepresentationWr8
            | L::ManifoldSurfaceShapeRepresentationWr9
            | L::ManifoldSurfaceShapeRepresentationWr10
            | L::ManifoldSurfaceShapeRepresentationWr11
            | L::ManifoldSurfaceShapeRepresentationWr12
            | L::ManifoldSurfaceShapeRepresentationWr13
            | L::ManifoldSurfaceShapeRepresentationWr14 => {
                let solid = matches!(
                    c.id,
                    L::AdvancedBrepShapeRepresentationWr1
                        | L::AdvancedBrepShapeRepresentationWr2
                        | L::AdvancedBrepShapeRepresentationWr3
                        | L::AdvancedBrepShapeRepresentationWr4
                        | L::AdvancedBrepShapeRepresentationWr5
                        | L::AdvancedBrepShapeRepresentationWr6
                );
                let items = self.refs(r, "items")?;
                let shell = self.role("shell")?;
                e.witnesses.clone_from(&items);
                e.witnesses.push(shell);
                e.proof = "Complete admitted items and shell face population; mapped/void/oriented wrapper populations excluded by exact profile signatures".into();
                match c.id {
                    L::AdvancedBrepShapeRepresentationWr1
                    | L::ManifoldSurfaceShapeRepresentationWr1 => self.forall(items, |s, x| {
                        Ok(s.count_types(
                            x,
                            if solid {
                                &[
                                    "MANIFOLD_SOLID_BREP",
                                    "FACETED_BREP",
                                    "MAPPED_ITEM",
                                    "AXIS2_PLACEMENT_3D",
                                ]
                            } else {
                                &[
                                    "SHELL_BASED_SURFACE_MODEL",
                                    "MAPPED_ITEM",
                                    "AXIS2_PLACEMENT_3D",
                                ]
                            },
                        )? == 1)
                    }),
                    L::AdvancedBrepShapeRepresentationWr2
                    | L::ManifoldSurfaceShapeRepresentationWr2 => Ok(items.iter().any(|x| {
                        self.is(
                            *x,
                            if solid {
                                "MANIFOLD_SOLID_BREP"
                            } else {
                                "SHELL_BASED_SURFACE_MODEL"
                            },
                        )
                    })),
                    L::AdvancedBrepShapeRepresentationWr3 => self
                        .forall(self.refs(shell, "cfs_faces")?, |s, x| {
                            Ok(s.is(x, "ADVANCED_FACE"))
                        }),
                    L::AdvancedBrepShapeRepresentationWr4 => {
                        Ok(!self.has(shell, "ORIENTED_CLOSED_SHELL")?)
                    }
                    L::AdvancedBrepShapeRepresentationWr5
                    | L::AdvancedBrepShapeRepresentationWr6 => self.forall(items, |s, x| {
                        Ok(!s.has(
                            x,
                            if c.id == L::AdvancedBrepShapeRepresentationWr5 {
                                "BREP_WITH_VOIDS"
                            } else {
                                "MAPPED_ITEM"
                            },
                        )?)
                    }),
                    L::ManifoldSurfaceShapeRepresentationWr3 => {
                        self.forall(items, |s, x| Ok(!s.has(x, "MAPPED_ITEM")?))
                    }
                    L::ManifoldSurfaceShapeRepresentationWr4 => Ok(self.count_types(
                        shell,
                        &["OPEN_SHELL", "ORIENTED_CLOSED_SHELL", "CLOSED_SHELL"],
                    )? == 1),
                    L::ManifoldSurfaceShapeRepresentationWr5 => self
                        .forall(self.refs(shell, "cfs_faces")?, |s, x| {
                            Ok(s.is(x, "FACE_SURFACE"))
                        }),
                    L::ManifoldSurfaceShapeRepresentationWr6
                    | L::ManifoldSurfaceShapeRepresentationWr7
                    | L::ManifoldSurfaceShapeRepresentationWr8
                    | L::ManifoldSurfaceShapeRepresentationWr9
                    | L::ManifoldSurfaceShapeRepresentationWr10
                    | L::ManifoldSurfaceShapeRepresentationWr11
                    | L::ManifoldSurfaceShapeRepresentationWr12
                    | L::ManifoldSurfaceShapeRepresentationWr13
                    | L::ManifoldSurfaceShapeRepresentationWr14 => {
                        e.proof = "Every face is ADVANCED_FACE: original OR first term TRUE. Plain-FS branch unexecuted; N2635 WR8-WR14 access paths unchanged".into();
                        e.facts
                            .insert("plain_face_surface_branch".into(), "NotRun".into());
                        self.forall(self.refs(shell, "cfs_faces")?, |s, x| {
                            Ok(s.is(x, "ADVANCED_FACE"))
                        })
                    }
                    _ => Err(Failure::Infrastructure),
                }
            }
            L::ShellBasedSurfaceModelWr1 => {
                e.proof = "constraints_geometry_shell_based_surface_model: every boundary is OPEN_SHELL or CLOSED_SHELL".into();
                self.forall(self.refs(r, "sbsm_boundary")?, |s, x| {
                    Ok(s.is(x, "OPEN_SHELL") || s.is(x, "CLOSED_SHELL"))
                })
            }
            L::ProductDefinitionShapeWr1 => {
                e.proof =
                    "definition is PRODUCT_DEFINITION, in characterized_product_definition SELECT"
                        .into();
                Ok(self.is(self.reference(r, "definition")?, "PRODUCT_DEFINITION"))
            }
            L::ShapeDefinitionRepresentationWr1 | L::ShapeDefinitionRepresentationWr2 => {
                e.proof = "Original direct-PDS first disjunct or shape representation type".into();
                Ok(if c.id == L::ShapeDefinitionRepresentationWr1 {
                    self.is(self.reference(r, "definition")?, "PRODUCT_DEFINITION_SHAPE")
                } else {
                    self.is(
                        self.reference(r, "used_representation")?,
                        "SHAPE_REPRESENTATION",
                    )
                })
            }
            L::SiUnitWr1 => {
                e.proof = "Not MASS_UNIT and no DERIVED_UNIT_ELEMENT population; original antecedent FALSE".into();
                Ok(!self.has(r, "MASS_UNIT")?
                    && self.population("DERIVED_UNIT_ELEMENT")?.is_empty())
            }
            L::LengthUnitWr1 | L::PlaneAngleUnitWr1 | L::SolidAngleUnitWr1 => {
                for unit in &self.sem.units().si_definitions {
                    if unit.record_index == r {
                        e.proof = "dimensions_for_si_unit from actual SI name, tested against seven physical exponents".into();
                        let expected = if c.id == L::LengthUnitWr1 {
                            [1, 0, 0, 0, 0, 0, 0]
                        } else {
                            [0; 7]
                        };
                        return Ok(unit.dimensions == expected);
                    }
                }
                let mut expected = [R::ZERO; 7];
                if c.id == L::LengthUnitWr1 {
                    *expected.first_mut().ok_or(Failure::Infrastructure)? = R::ONE;
                }
                e.proof = "Explicit conversion dimensions must match the nominal quantity's seven physical exponents".into();
                Ok(self.unit_dimensions(r, &mut e.witnesses)? == expected)
            }
            L::LengthMeasureWithUnitWr1 | L::MeasureWithUnitWr1 => {
                e.proof = "Length-family measure value retains base LENGTH_MEASURE type; actual unit has LENGTH_UNIT and length dimensions".into();
                let mut expected = [R::ZERO; 7];
                *expected.first_mut().ok_or(Failure::Infrastructure)? = R::ONE;
                Ok(self.is(self.reference(r, "unit_component")?, "LENGTH_UNIT")
                    && self
                        .unit_dimensions(self.reference(r, "unit_component")?, &mut e.witnesses)?
                        == expected)
            }
            L::UncertaintyMeasureWithUnitWr1 => {
                e.proof = "valid_measure_value: length-family value has REAL base type, numeric value > 0".into();
                Ok(R::decimal(
                    &self
                        .field(r, "value_component")?
                        .children
                        .first()
                        .ok_or(Failure::Infrastructure)?
                        .text,
                )? > R::ZERO)
            }
            L::MechanicalContextWr1 => {
                e.proof = "Actual inherited discipline_type = mechanical".into();
                Ok(self.field(r, "discipline_type")?.text == "mechanical")
            }
            L::DesignContextWr1 => {
                e.proof = "Actual inherited life_cycle_stage = design".into();
                Ok(self.field(r, "life_cycle_stage")?.text == "design")
            }
            L::ConversionBasedUnitWr1 => {
                let factor = self.reference(r, "conversion_factor")?;
                let base = self.reference(factor, "unit_component")?;
                let declared = self.unit_dimensions(r, &mut e.witnesses)?;
                e.witnesses.push(factor);
                e.witnesses.push(base);
                let actual = self.unit_dimensions(base, &mut e.witnesses)?;
                e.proof="Explicit conversion dimensions equal derive_dimensional_exponents(unit_component), by seven values; no factor recursion".into();
                let mut length = [R::ZERO; 7];
                *length.first_mut().ok_or(Failure::Infrastructure)? = R::ONE;
                if self.is(r, "LENGTH_UNIT") && declared != length {
                    e.facts.insert("nominal_length_rule".into(), "FALSE".into());
                }
                e.facts.insert(
                    "declared_dimensions".into(),
                    declared
                        .iter()
                        .map(|v| format!("{}/{}", v.numerator(), v.denominator()))
                        .collect::<Vec<_>>()
                        .join(","),
                );
                e.facts.insert(
                    "factor_unit_dimensions".into(),
                    actual
                        .iter()
                        .map(|v| format!("{}/{}", v.numerator(), v.denominator()))
                        .collect::<Vec<_>>()
                        .join(","),
                );
                Ok(declared == actual)
            }
            L::DerivedUnitWr1
            | L::DerivedUnitWr2
            | L::FoundedItemWr1
            | L::FoundedItemWr2
            | L::MassUnitWr1
            | L::NonNegativeLengthMeasureWr1
            | L::PositiveLengthMeasureWr1
            | L::DimensionCountWr1 => Err(Failure::Infrastructure),
        }
    }
    fn type_value(id: L, value: &Value) -> Checked<bool> {
        let number = R::decimal(&value.text)?;
        match id {
            L::NonNegativeLengthMeasureWr1 => Ok(number >= R::ZERO),
            L::PositiveLengthMeasureWr1 | L::DimensionCountWr1 => Ok(number > R::ZERO),
            _ => Err(Failure::Infrastructure),
        }
    }
    pub(super) fn local(&mut self) -> Checked<()> {
        for id in catalog::ENTITY_ORDER {
            let clause = catalog::local(*id);
            let population = self.population(clause.type_name)?;
            let mut empty = RuleEvidence::new(
                format!("{}.{}", clause.type_name, clause.label),
                clause.kind,
                clause.line,
            );
            empty
                .populations
                .insert(clause.type_name.into(), Vec::new());
            if population.is_empty() {
                empty.applicability = "Guarded";
                empty.evaluation = "Evaluated";
                empty.value = Some(Logical::True);
                empty.proof =
                    "Full-file supertype population empty under admitted complete signatures"
                        .into();
                self.out.evidence.rules.push(empty);
                continue;
            }
            for r in population {
                self.tick(r)?;
                let mut e = empty.clone();
                e.owner = Some(r);
                e.populations.insert(clause.type_name.into(), vec![r]);
                let result = self.local_value(clause, r, &mut e);
                if let Ok(value) = result {
                    e.evaluation = "Evaluated";
                    e.value = Some(if value { Logical::True } else { Logical::False });
                }
                // Save partial entity proof before propagating numeric/resource
                // failure; TYPE drivers below intentionally have different timing.
                self.out.evidence.rules.push(e);
                if !result? {
                    let active = self
                        .out
                        .evidence
                        .rules
                        .last()
                        .ok_or(Failure::Infrastructure)?;
                    let conversion = clause.id == L::ConversionBasedUnitWr1;
                    let key = if conversion && active.facts.contains_key("nominal_length_rule") {
                        "SourceRuleViolation:LENGTH_UNIT.WR1+CONVERSION_BASED_UNIT.WR1".into()
                    } else {
                        format!("SourceRuleViolation:{}", active.name)
                    };
                    let proof = active.proof.clone();
                    let related = if conversion {
                        active.witnesses.clone()
                    } else {
                        Vec::new()
                    };
                    let expected = if conversion {
                        active
                            .facts
                            .get("factor_unit_dimensions")
                            .cloned()
                            .unwrap_or_default()
                    } else {
                        String::new()
                    };
                    let actual = if conversion {
                        active
                            .facts
                            .get("declared_dimensions")
                            .cloned()
                            .unwrap_or_default()
                    } else {
                        String::new()
                    };
                    return Err(self.fail(
                        &key,
                        "D3",
                        r,
                        clause.failure_field,
                        &proof,
                        Status::Rejected,
                        &related,
                        &expected,
                        &actual,
                    ));
                }
            }
        }
        for r in self.population("MEASURE_WITH_UNIT")? {
            let value = self.field(r, "value_component")?;
            for id in catalog::length_predicates(&value.text) {
                let p = catalog::local(*id);
                self.tick(r)?;
                let good =
                    Self::type_value(*id, value.children.first().ok_or(Failure::Infrastructure)?)?;
                let mut e =
                    RuleEvidence::new(format!("{}.{}", p.type_name, p.label), p.kind, p.line);
                e.owner = Some(r);
                e.evaluation = "Evaluated";
                e.value = Some(if good { Logical::True } else { Logical::False });
                e.proof="Named length value and inherited TYPE WHERE; base LENGTH_MEASURE itself has no positivity WHERE".into();
                self.out.evidence.rules.push(e.clone());
                if !good {
                    return Err(self.reject(
                        &format!("SourceRuleViolation:{}", e.name),
                        "D3",
                        r,
                        p.failure_field,
                        &e.proof,
                    ));
                }
            }
        }
        let p = catalog::local(catalog::DIMENSION);
        let owner = self.role("unit_context")?;
        let mut e = RuleEvidence::new(format!("{}.{}", p.type_name, p.label), p.kind, p.line);
        e.owner = Some(owner);
        e.evaluation = "Evaluated";
        e.value = Some(
            if Self::type_value(
                catalog::DIMENSION,
                self.field(owner, "coordinate_space_dimension")?,
            )? {
                Logical::True
            } else {
                Logical::False
            },
        );
        e.proof =
            "Positive INTEGER; 3D capability guard already checked independently by ticket 03"
                .into();
        self.out.evidence.rules.push(e);
        Ok(())
    }
}
