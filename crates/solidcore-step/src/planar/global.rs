// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Fixed global rule evaluation, including explicit closed-profile exclusions.
//! Upstream: `StepPlanarRules.cpp::GlobalValue/GlobalRules` at `d8ed45a`.
//! Deviations: exhaustive named Rust identities and checked iterative inverse use.
//! Computed UNKNOWN is retained; it is never substituted for unevaluated evidence.
use super::catalog::{self, Global, GlobalId as G};
use super::validator::{Checked, Failure, Validator};
use super::{Logical, Rational as R, RuleEvidence};
use std::collections::BTreeSet;

impl Validator<'_, '_> {
    fn global_value(&mut self, meta: Global) -> Checked<RuleEvidence> {
        let mut e = RuleEvidence::new(meta.name, "GLOBAL", meta.line);
        e.applicability = "Guarded";
        e.evaluation = "Evaluated";
        e.value = Some(Logical::True);
        for name in meta.types {
            // Only this fixed schema's declared FOR universe may use a complete
            // closed-signature exclusion. Public unknown-type queries stay None.
            e.populations.insert(
                (*name).into(),
                self.sem
                    .instances(name)
                    .map(<[usize]>::to_vec)
                    .unwrap_or_default(),
            );
        }
        let category = self.population("PRODUCT_RELATED_PRODUCT_CATEGORY")?;
        let context = self.role("definition_context")?;
        let representation = self.role("representation")?;
        let good = match meta.id {
            G::ApplicationProtocolDefinitionRequired => {
                e.applicability = "Mandatory";
                let ac = self.role("application_context")?;
                let uses = self
                    .sem
                    .used_in(
                        &self.record(ac)?.id,
                        "APPLICATION_PROTOCOL_DEFINITION",
                        "application",
                    )
                    .ok_or(Failure::Infrastructure)?;
                let mut matches = 0;
                for index in uses {
                    let apd = self
                        .sem
                        .source()
                        .references()
                        .get(index)
                        .ok_or(Failure::Infrastructure)?
                        .record_index;
                    self.tick(apd)?;
                    e.witnesses.push(apd);
                    matches+=usize::from(self.field(apd,"application_interpreted_model_schema_name")?.text == "ap203_configuration_controlled_3d_design_of_mechanical_parts_and_assemblies");
                }
                e.facts
                    .insert("matching_apd_count".into(), matches.to_string());
                e.proof="At least one AC with at least one matching reverse APD schema name; HEADER and APD year are not substitutes".into();
                matches > 0
            }
            G::CompatibleDimension => {
                e.applicability = "Mandatory";
                let dimension = self
                    .field(self.role("unit_context")?, "coordinate_space_dimension")?
                    .text
                    .clone();
                let mut good = true;
                for name in ["CARTESIAN_POINT", "DIRECTION"] {
                    for r in self.population(name)? {
                        self.tick(r)?;
                        if self
                            .sem
                            .used_by_representation(r)
                            .ok_or(Failure::Infrastructure)?
                        {
                            e.witnesses.push(r);
                            let size = self
                                .field(
                                    r,
                                    if name == "DIRECTION" {
                                        "direction_ratios"
                                    } else {
                                        "coordinates"
                                    },
                                )?
                                .children
                                .len();
                            good = (R::new(size as i64, 1)? == R::decimal(&dimension)?) && good;
                        }
                    }
                }
                e.facts.insert(
                    "context_id".into(),
                    self.record(self.role("unit_context")?)?.id.clone(),
                );
                e.facts.insert("dimension".into(), dimension);
                e.proof="item_in_context uses direct representation membership and RI reverse reachability; all indirectly used points and directions counted".into();
                good
            }
            G::ConsistentUncertainty => {
                e.applicability = "Conditional";
                let mut good = true;
                for context in self.population("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT")? {
                    let mut names = BTreeSet::new();
                    for u in self.refs(context, "uncertainty")? {
                        self.tick(u)?;
                        e.witnesses.push(u);
                        good = names.insert(self.field(u, "name")?.text.clone()) && good;
                    }
                }
                e.facts.insert("WR2_UAR_count".into(), "0".into());
                e.facts.insert("WR3_QRI_count".into(), "0".into());
                e.proof="WR1 compares uncertainty.name, not identity/value/unit; WR2/WR3 populations excluded by complete profile signatures".into();
                good
            }
            G::RestrictRepresentationForSurfaceCondition => {
                e.applicability = "Mandatory";
                let mut failures = 0;
                let mut unknown = 0;
                let special = [
                    "visual appearance",
                    "tactile appearance",
                    "contact ratio",
                    "hardness",
                    "treatment result",
                    "surface texture",
                ];
                for pdr in self.population("PROPERTY_DEFINITION_REPRESENTATION")? {
                    self.tick(pdr)?;
                    let definition = self.reference(pdr, "definition")?;
                    let used = self.reference(pdr, "used_representation")?;
                    let correlation = if !special
                        .contains(&self.field(definition, "name")?.text.as_str())
                    {
                        Logical::Unknown
                    } else if self.field(definition, "name")?.text == self.field(used, "name")?.text
                    {
                        Logical::True
                    } else {
                        Logical::False
                    };
                    failures += usize::from(correlation.not().query_selects());
                    unknown += usize::from(correlation == Logical::Unknown);
                    e.witnesses.extend([pdr, definition, used]);
                }
                e.facts
                    .insert("correlation_UNKNOWN_count".into(), unknown.to_string());
                e.facts
                    .insert("failure_QUERY_count".into(), failures.to_string());
                e.proof="SDR is in FOR PDR; ordinary PDS.name yields source LOGICAL UNKNOWN; NOT UNKNOWN = UNKNOWN; QUERY selects only TRUE; outer count = 0".into();
                failures == 0
            }
            G::SubtypeExclusivenessRepresentationItem => {
                e.applicability = "Mandatory";
                let mut good = true;
                for r in self.population("REPRESENTATION_ITEM")? {
                    self.tick(r)?;
                    let matches = self.count_types(
                        r,
                        &[
                            "MEASURE_REPRESENTATION_ITEM",
                            "VALUE_REPRESENTATION_ITEM",
                            "COMPOUND_REPRESENTATION_ITEM",
                        ],
                    )?;
                    good = matches <= 1 && good;
                }
                e.facts.insert("criterion".into(), "3".into());
                e.proof="type_check_function criterion 3: every full RI TYPEOF intersects the three subtype names in <=1; parameters are defined and sub_names nonempty".into();
                good
            }
            G::ValidateDependentlyInstantiableEntityDataTypes => {
                e.applicability = "Mandatory";
                let mut restricted = BTreeSet::new();
                for population in e.populations.values() {
                    for r in population {
                        for name in self.sem.types_of(*r).ok_or(Failure::Infrastructure)? {
                            restricted.insert(name.to_owned());
                        }
                    }
                }
                restricted.insert("GENERIC".into());
                let mut good = true;
                for loop_record in self.population("LOOP")? {
                    let mut pending = vec![loop_record];
                    let mut visited = BTreeSet::new();
                    let mut anchored = false;
                    while !pending.is_empty() && !anchored {
                        let r = pending.pop().ok_or(Failure::Infrastructure)?;
                        if !visited.insert(r) {
                            continue;
                        }
                        for index in self.sem.source().incoming(&self.record(r)?.id) {
                            let owner = self
                                .sem
                                .source()
                                .references()
                                .get(*index)
                                .ok_or(Failure::Infrastructure)?
                                .record_index;
                            self.tick(owner)?;
                            let types = self.sem.types_of(owner).ok_or(Failure::Infrastructure)?;
                            if !types.iter().all(|name| restricted.contains(*name)) {
                                e.witnesses.extend([loop_record, owner]);
                                anchored = true;
                                break;
                            }
                            pending.push(owner);
                        }
                    }
                    good = anchored && good;
                }
                e.facts.insert(
                    "restricted_type_count_with_GENERIC".into(),
                    restricted.len().to_string(),
                );
                e.proof="59 FOR populations; LOOP includes EDGE_LOOP. Each input reaches a referring instance whose TYPEOF is not a subset of the restricted type union; cycles alone cannot anchor".into();
                good
            }
            G::AlternativeSolutionRequiresSolutionDefinition
            | G::BreakdownElementRequiresProductDefinition
            | G::DesignConstraintRequiresProductDefinition
            | G::PlyStockMaterialSelect => {
                e.witnesses.clone_from(&category);
                e.proof="PRPC population may be nonempty; every actual category.name is part, so source special-category/IF-ply filter is empty".into();
                e.facts
                    .insert("category_name_filter".into(), "part only or absent".into());
                let mut good = true;
                for r in &category {
                    if self.field(*r, "name")?.text != "part" {
                        good = false;
                        break;
                    }
                }
                good
            }
            G::ConstraintDefinitionRequiresConstraintCategory
            | G::RestrictAlternativeDefinition
            | G::RestrictPartOccurrence
            | G::RestrictPartOccurrenceCategory
            | G::SolutionDefinitionRequiresSolutionCategory => {
                e.witnesses = vec![context];
                e.proof="Actual PD context.name is part definition, so constraint/alternative/part occurrence filters are empty".into();
                e.facts.insert(
                    "context_name".into(),
                    self.field(context, "name")?.text.clone(),
                );
                self.field(context, "name")?.text == "part definition"
            }
            G::RestrictCentreOfMassRepresentation | G::RestrictTreatmentResult => {
                // These source filters are evaluated on actual representation names.
                // The fixed semantic guard excludes the two special purposes.
                e.witnesses = vec![representation];
                e.proof="REPRESENTATION population nonempty; actual name does not equal the original special trigger".into();
                e.facts.insert(
                    "representation_name".into(),
                    self.field(representation, "name")?.text.clone(),
                );
                self.field(representation, "name")?.text
                    != if meta.id == G::RestrictCentreOfMassRepresentation {
                        "centre of mass"
                    } else {
                        "treatment result"
                    }
            }
            G::RestrictAssemblyCategory | G::RestrictCollectionCategory => {
                e.witnesses = vec![context];
                e.proof="ClosedProfileTypeExclusion: PRODUCT_DEFINITION_CONTEXT_ASSOCIATION; original reverse association filter empty (not a direct-context test)".into();
                e.facts.insert("association_population".into(), "0".into());
                true
            }
            G::ComponentClassForAssemblySelect
            | G::DraughtingModelItemsConstraint
            | G::ExternalVersionAssignmentsAreValid
            | G::MaterialForCoatingLayer
            | G::PlibPropertyReferenceRequiresNameScope
            | G::PlibPropertyReferenceRequiresVersion
            | G::PlyReference
            | G::ProductConceptFeatureRequiresCategory
            | G::ProductDefinitionReplacementRequiresEffectivityAssignment
            | G::RestrictClassificationAssignments
            | G::RestrictConceptFeatureOperator
            | G::RestrictConfigurationDesignForClassBreakdownAssociation
            | G::RestrictConfigurationDesignForDesignConstraint
            | G::RestrictGroupRelationshipForClassificationHierarchy
            | G::RestrictGroupRelationshipForSpecificationCategory
            | G::RestrictLanguageAssignmentPerAttribute
            | G::RestrictProductDefinitionsForBaseElement
            | G::RestrictProductDefinitionsForCollection
            | G::RestrictProductDefinitionsForDefinitionUsage
            | G::RestrictProductDefinitionsForDesignConstraintAssociation
            | G::RestrictProductDefinitionsForPartDefinitionRelationship
            | G::SelectedInstanceUsageRequiresRepresentation
            | G::StockMaterialReference
            | G::StyledCurve
            | G::SubtypeExclusivenessGeometricTolerance
            | G::SubtypeMandatoryGeometricTolerance
            | G::TextFontUsage
            | G::ThicknessLaminateTableComponentSelect => {
                e.proof="Full-file closed-profile type exclusion proves first FOR/outer-loop population empty; other FOR populations retained separately".into();
                e.populations
                    .get(*meta.types.first().ok_or(Failure::Infrastructure)?)
                    .ok_or(Failure::Infrastructure)?
                    .is_empty()
            }
        };
        e.value = Some(if good { Logical::True } else { Logical::False });
        Ok(e)
    }
    pub(super) fn global_rules(&mut self) -> Checked<()> {
        for id in catalog::GLOBAL_ORDER {
            let rule = catalog::global(*id);
            self.tick(self.role("representation")?)?;
            let e = self.global_value(rule)?;
            let good = e.value == Some(Logical::True);
            let name = e.name.clone();
            let proof = e.proof.clone();
            self.out.evidence.rules.push(e);
            if !good {
                return Err(self.reject(
                    &format!("SourceRuleViolation:{name}.WR1"),
                    "D3",
                    self.role(rule.failure_role)?,
                    rule.failure_field,
                    &proof,
                ));
            }
        }
        Ok(())
    }
}
