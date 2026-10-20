// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent source-identity/closure truth. Geometry remains unqualified.
use solidcore_step::{inspect, semantics, Limits, ValueKind};
use std::fmt::Write;

const AP203:&str="AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
const MBX: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 1 1 1 1}";
fn plane(schema: &str) -> String {
    include_str!("fixtures/plane.step").replace(AP203, schema)
}
fn append(source: &str, records: &str) -> String {
    source.replace("ENDSEC;\nEND-ISO", &format!("{records}\nENDSEC;\nEND-ISO"))
}
fn check(data: &str, general: bool) -> (solidcore_step::Outcome, semantics::Outcome) {
    let source = inspect(data.as_bytes().to_vec(), Limits::planar_fixtures());
    let result = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options {
            geometry_scope: if general {
                semantics::GeometryScope::GeneralBrep
            } else {
                semantics::GeometryScope::ExplicitOuterBounds
            },
            check_part_and_closure: true,
            ..semantics::Options::default()
        },
    );
    (source, result)
}
// Integration-test helpers use the same deliberate assertions as #[test] bodies.
#[allow(clippy::unwrap_used)]
fn index(snapshot: &semantics::Snapshot, id: &str) -> usize {
    snapshot
        .source()
        .records()
        .iter()
        .position(|r| r.id == id)
        .unwrap()
}
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
fn role_id<'a>(snapshot: &'a semantics::Snapshot, name: &str) -> &'a str {
    let role = snapshot
        .part_definition()
        .unwrap()
        .roles
        .iter()
        .find(|r| r.name == name)
        .unwrap();
    &snapshot.source().records()[role.record_index].id
}
const PROPERTY: &str = "#900=PROPERTY_DEFINITION('source label',$,#7);
#901=PROPERTY_DEFINITION_REPRESENTATION(#900,#902);
#902=REPRESENTATION('metadata',(#903),#904);
#903=DESCRIPTIVE_REPRESENTATION_ITEM('caption','verbatim source value');
#904=REPRESENTATION_CONTEXT('property context','metadata');";

#[test]
fn planar_identity_chain_source_claims_and_owning_views_survive_outcomes() {
    let data = plane(AP203);
    let (source, outcome) = check(&data, false);
    assert_eq!(outcome.status, semantics::Status::Checked);
    assert_eq!(outcome.part, "Passed");
    assert_eq!(outcome.si_units, "Passed");
    let snapshot = outcome.snapshot.as_ref().unwrap();
    let part = snapshot.part_definition().unwrap();
    assert_eq!(part.shape_role, "OpenSurfaceModel");
    assert_eq!(part.roles.len(), 13);
    assert_eq!(role_id(snapshot, "product"), "4");
    assert_eq!(role_id(snapshot, "shape"), "21");
    assert_eq!(role_id(snapshot, "unit_context"), "12");
    assert_eq!(snapshot.source().bytes(), data.as_bytes());
    assert_eq!(snapshot.units().length_power10_mm, Some(0));
    let field = snapshot
        .field(index(snapshot, "9"), None, "used_representation")
        .unwrap();
    assert_eq!(field.reference_target(), Some(index(snapshot, "20")));
    let retained = snapshot.clone();
    drop(outcome);
    drop(source);
    assert_eq!(field.value().unwrap().text, "20");
    assert_eq!(role_id(&retained, "shell"), "22");
}

#[test]
fn identical_closed_shell_geometry_preserves_surface_and_material_roles() {
    for (data, expected) in [
        (include_str!("fixtures/material-box.step"), "MaterialSolid"),
        (
            include_str!("fixtures/surface-box.step"),
            "ClosedSurfaceModel",
        ),
    ] {
        let (_, result) = check(data, false);
        assert_eq!(result.status, semantics::Status::Checked);
        let snapshot = result.snapshot.unwrap();
        assert_eq!(snapshot.part_definition().unwrap().shape_role, expected);
        assert_eq!(
            snapshot.source().find("22").unwrap().components[0].name,
            "CLOSED_SHELL"
        );
    }
}

#[test]
fn all_five_fixed_catalogs_keep_their_own_identity_and_source_metadata() {
    for (schema, general, profile) in [
        (AP203, false, "203e2-P0-Planar-0.1"),
        (AP203, true, "203e2-G0-Source-0.1"),
        (AP214, true, "214e3-P0-Source-0.1"),
        (MBX, true, "214-MBx2010-Source-0.1"),
        ("CONFIG_CONTROL_DESIGN", true, "203-N916-Source-0.1"),
    ] {
        let mut data = plane(schema);
        if schema == "CONFIG_CONTROL_DESIGN" {
            data = data
                .lines()
                .map(|line| {
                    if ["#4=", "#5=", "#7=", "#8="]
                        .iter()
                        .any(|p| line.starts_with(p))
                    {
                        line.replace('$', "''")
                    } else {
                        line.into()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
        }
        let (_, result) = check(&data, general);
        assert_eq!(
            result.status,
            semantics::Status::Checked,
            "{schema}: {:?}",
            result.diagnostics
        );
        let snapshot = result.snapshot.unwrap();
        assert_eq!(snapshot.profile().profile, profile);
        assert_eq!(snapshot.part_definition().unwrap().apd_year, "2011");
        assert_eq!(role_id(&snapshot, "unit_context"), "12");
    }
}

#[test]
fn source_year_and_claim_do_not_select_or_repair_a_schema() {
    let data = plane(AP203).replace(",2011,#1)", ",+0001994,#1)").replace(
        "'ap203_configuration_controlled_3d_design_of_mechanical_parts_and_assemblies'",
        "'unverified source claim'",
    );
    let (_, result) = check(&data, false);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert_eq!(snapshot.profile().registry_id, "ap203e2-n2635");
    assert_eq!(snapshot.part_definition().unwrap().apd_year, "+0001994");
    assert_eq!(
        snapshot.part_definition().unwrap().apd_schema_claim,
        "unverified source claim"
    );
}

#[test]
fn orphans_retain_all_source_positions_and_never_publish_a_partial_part() {
    let data = append(
        &plane(AP203),
        "#900=CARTESIAN_POINT('',(0.,0.,0.));\n#901=DIRECTION('',(1.,0.,0.));",
    );
    let (source, result) = check(&data, false);
    assert_eq!(result.status, semantics::Status::Unsupported);
    assert_eq!(result.part, "Unsupported");
    assert_eq!(result.si_units, "NotRun");
    assert!(result.snapshot.is_none());
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "UnownedOrOrphanProfileContent");
    assert_eq!(d.source.record_id, "900");
    assert!(d.source.parameter_path.is_empty());
    assert_eq!(d.actual, "#900 #901 ");
    assert_eq!(d.responsibility, "profile ownership, not RI.WR1");
    let raw = source.source.unwrap();
    assert_eq!(d.source.primary, raw.find("900").unwrap().span);
    assert_eq!(d.source.related, [raw.find("901").unwrap().id_span]);
    assert_eq!(raw.bytes(), data.as_bytes());
}

#[test]
fn duplicate_product_labels_do_not_merge_distinct_source_definitions() {
    let data = append(
        &plane(AP203),
        "#905=PRODUCT('P0-PART','other source product',$,(#3));",
    );
    let (source, result) = check(&data, false);
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "UnsupportedSinglePartGraph");
    assert_eq!(d.expected, "PRODUCT count=1");
    assert_eq!(d.actual, "2 #4 #905 ");
    assert_eq!(d.source.record_id, "905");
    assert_eq!(
        d.source.related,
        [source.source.unwrap().find("4").unwrap().id_span]
    );
    assert!(result.snapshot.is_none());
}

#[test]
fn wrong_identity_link_is_not_a_type_error_or_a_source_rewrite() {
    let data = plane(AP203).replace(
        "#8=PRODUCT_DEFINITION_SHAPE('',$,#7)",
        "#8=PRODUCT_DEFINITION_SHAPE('',$,#8)",
    );
    let (source, result) = check(&data, false);
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "UnsupportedSinglePartGraph");
    assert_eq!(d.source.record_id, "8");
    assert!(d.source.parameter_path.is_empty());
    assert_eq!(d.field, "");
    assert_eq!(d.expected, "7");
    assert_eq!(d.actual, "8");
    assert_eq!(
        source.source.unwrap().raw(d.source.primary),
        Some(b"#8".as_slice())
    );
}

#[test]
fn property_sharing_keeps_every_link_and_its_original_context() {
    let data=append(&append(&plane(AP214),PROPERTY),"#905=PROPERTY_DEFINITION_REPRESENTATION(#900,#902);\n#906=PROPERTY_DEFINITION('source label',$,#8);\n#907=PROPERTY_DEFINITION_REPRESENTATION(#906,#902);");
    let (_, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    assert_eq!(result.property_attachment_checks, 3);
    let snapshot = result.snapshot.unwrap();
    let p = snapshot.part_definition().unwrap();
    assert_eq!(p.property_attachments.len(), 3);
    assert_eq!(
        p.property_attachments
            .iter()
            .map(|p| snapshot.source().records()[p.link].id.as_str())
            .collect::<Vec<_>>(),
        ["901", "905", "907"]
    );
    assert!(p
        .property_attachments
        .iter()
        .all(
            |p| snapshot.source().records()[p.representation].id == "902"
                && snapshot.source().records()[p.context].id == "904"
        ));
    assert_eq!(role_id(&snapshot, "unit_context"), "12");
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "903")),
        Some(true)
    );
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "904")),
        Some(false)
    );
    assert_eq!(
        snapshot
            .field(index(&snapshot, "903"), None, "description")
            .unwrap()
            .value()
            .unwrap()
            .text,
        "verbatim source value"
    );
}

#[test]
fn a_property_link_cannot_smuggle_geometry_or_claim_an_orphan_owner() {
    let data = append(&plane(AP214), PROPERTY).replace("'metadata',(#903)", "'metadata',(#100)");
    let (_, result) = check(&data, true);
    let d = &result.diagnostics[0];
    assert_eq!(d.source.key, "UnsupportedPropertyItem");
    assert_eq!(d.source.record_id, "902");
    assert_eq!(d.source.parameter_path, [2, 1]);
    assert_eq!(d.actual, "100");
    assert!(result.snapshot.is_none());
    let data =
        append(&plane(AP214), PROPERTY).replace("'source label',$,#7", "'source label',$,#900");
    let (_, result) = check(&data, true);
    assert_ne!(result.status, semantics::Status::Checked);
    assert!(result.snapshot.is_none());
}

#[test]
fn source_wrapper_relationship_preserves_separate_representation_identities() {
    let data=append(&plane(AP214).replace("SHAPE_DEFINITION_REPRESENTATION(#8,#20)","SHAPE_DEFINITION_REPRESENTATION(#8,#900)"),"#900=SHAPE_REPRESENTATION('',(#133),#12);\n#901=SHAPE_REPRESENTATION_RELATIONSHIP('',$,#900,#20);");
    let (_, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert_eq!(role_id(&snapshot, "definition_representation"), "900");
    assert_eq!(role_id(&snapshot, "representation"), "20");
    assert_eq!(
        role_id(&snapshot, "shape_representation_relationship"),
        "901"
    );
    assert_eq!(
        snapshot.source().find("901").unwrap().components[0].parameters[2].text,
        "900"
    );
}

#[test]
fn linked_wireframe_and_supporting_geometry_keep_their_distinct_physical_roles() {
    let wire="#910=GEOMETRIC_CURVE_SET('source curves',(#913));#911=DIRECTION('',(1.,0.,0.));#912=VECTOR('',#911,10.);#913=LINE('',#100,#912);#920=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#910),#12);#930=SHAPE_REPRESENTATION_RELATIONSHIP('',$,#20,#920);";
    let support="#960=CONSTRUCTIVE_GEOMETRY_REPRESENTATION('',(#133,#961),#12);#961=PLANE('source plane',#133);#962=CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP('',$,#20,#960);";
    let data = append(&append(&plane(AP214), wire), support);
    let (_, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert_eq!(role_id(&snapshot, "representation"), "20");
    assert_eq!(role_id(&snapshot, "wireframe_representation"), "920");
    assert_eq!(role_id(&snapshot, "construction_representation"), "960");
    assert_eq!(
        snapshot.part_definition().unwrap().shape_role,
        "OpenSurfaceModel"
    );
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "913")),
        Some(true)
    );
    let (_, result) = check(&data.replace("'',$,#20,#960", "'',$,#960,#20"), true);
    assert_eq!(
        result.diagnostics[0].source.key,
        "SourceRuleViolation:CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP.WR2"
    );
}

#[test]
fn filtered_representation_use_crosses_only_the_admitted_composite_bridge() {
    let data=append(&plane(AP203).replace("#202=LINE('',#100,#201);","#202=COMPOSITE_CURVE('',(#910),.U.);"),"#910=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#911);#911=TRIMMED_CURVE('',#912,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(10.)),.T.,.PARAMETER.);#912=LINE('',#100,#201);");
    let (_, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "910")),
        Some(false)
    );
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "911")),
        Some(true)
    );
    assert_eq!(
        snapshot.used_by_representation(index(&snapshot, "912")),
        Some(true)
    );
    assert_eq!(
        snapshot
            .field(index(&snapshot, "202"), None, "self_intersect")
            .unwrap()
            .value()
            .unwrap()
            .text,
        "U"
    );
}

#[test]
fn layer_and_invisibility_require_preexisting_ownership() {
    let data=append(&plane(MBX),"#950=PRESENTATION_LAYER_ASSIGNMENT('layer','unchanged source label',(#21));#951=INVISIBILITY((#950));");
    let (_, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    let snapshot = result.snapshot.unwrap();
    assert!(snapshot
        .part_definition()
        .unwrap()
        .roles
        .iter()
        .any(|r| r.name.starts_with("administrative_attachment_")
            && snapshot.source().records()[r.record_index].id == "950"));
    assert!(snapshot
        .part_definition()
        .unwrap()
        .roles
        .iter()
        .any(|r| r.name.starts_with("invisibility_")
            && snapshot.source().records()[r.record_index].id == "951"));
    let data=append(&plane(MBX),"#950=PRESENTATION_LAYER_ASSIGNMENT('layer','',(#960));#960=CARTESIAN_POINT('',(0.,0.,0.));");
    let (_, result) = check(&data, true);
    assert_eq!(
        result.diagnostics[0].source.key,
        "UnsupportedAdministrativeAttachment"
    );
    let data = append(
        &plane(MBX),
        "#950=REPRESENTATION('',(#100),#12);#951=INVISIBILITY((#950));",
    );
    let (_, result) = check(&data, true);
    assert_eq!(
        result.diagnostics[0].source.key,
        "UnsupportedInvisibilityAttachment"
    );
    assert!(result.snapshot.is_none());
}

#[test]
fn ordinary_bounds_remain_deferred_without_hiding_later_orphans() {
    let data = plane(AP203).replace("FACE_OUTER_BOUND(", "FACE_BOUND(");
    let (_, p0) = check(&data, false);
    assert_eq!(
        p0.diagnostics[0].source.key,
        "UnsupportedExplicitOuterBoundRequired"
    );
    let (_, general) = check(&data, true);
    assert_eq!(general.status, semantics::Status::Deferred);
    assert_eq!(general.part, "Deferred");
    let snapshot = general.snapshot.unwrap();
    assert_eq!(
        snapshot
            .part_definition()
            .unwrap()
            .deferred_region_faces
            .len(),
        1
    );
    assert_eq!(
        snapshot.source().find("301").unwrap().components[0].name,
        "FACE_BOUND"
    );
    let data = append(&data, "#950=CARTESIAN_POINT('',(0.,0.,0.));");
    let (_, result) = check(&data, true);
    assert_eq!(result.part, "Unsupported");
    assert_eq!(result.status, semantics::Status::Unsupported);
    assert!(result.snapshot.is_none());
}

#[test]
fn part_budget_and_report_failure_cannot_publish_partial_ownership() {
    let data = append(&plane(AP214), PROPERTY);
    let (mut source, good) = check(&data, true);
    let visits = good.dependency_visits;
    let options = semantics::Options {
        geometry_scope: semantics::GeometryScope::GeneralBrep,
        check_part_and_closure: true,
        ..semantics::Options::default()
    };
    assert_eq!(
        semantics::check(
            &source,
            semantics::Limits {
                dependency_visits: visits,
                preserved_diagnostics: 1024,
            },
            &options
        )
        .status,
        semantics::Status::Checked
    );
    let failed = semantics::check(
        &source,
        semantics::Limits {
            dependency_visits: visits - 1,
            preserved_diagnostics: 1024,
        },
        &options,
    );
    assert_eq!(failed.status, semantics::Status::ResourceLimit);
    assert!(failed.snapshot.is_none());
    assert_eq!(failed.dependency_visits, visits);
    source.limits.report_bytes = 1;
    assert_eq!(
        semantics::compile_report(&source, &good),
        Err(semantics::ReportError::Output(
            solidcore_step::report::Error::ReportByteLimit
        ))
    );
}

#[test]
fn optional_mode_and_explicit_context_cannot_fabricate_a_part_or_replace_units() {
    let data = append(&plane(AP214), PROPERTY);
    let source = inspect(data.as_bytes().to_vec(), Limits::planar_fixtures());
    let unchecked = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options::default(),
    );
    let snapshot = unchecked.snapshot.unwrap();
    assert!(snapshot.part_definition().is_none());
    assert_eq!(snapshot.used_by_representation(0), None);
    let result = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_part_and_closure: true,
            physical_context: Some("904".into()),
            ..semantics::Options::default()
        },
    );
    assert_eq!(result.status, semantics::Status::InvalidRequest);
    assert_eq!(
        result.diagnostics[0].source.key,
        "PhysicalContextSelectionMismatch"
    );
    assert!(result.snapshot.is_none());
    let (source, result) = check(&data, true);
    let report: serde_json::Value =
        serde_json::from_slice(&semantics::compile_report(&source, &result).unwrap()).unwrap();
    assert_eq!(report["schema"], "solidcore.step.part-closure.v1");
    assert_eq!(report["part"], "Passed");
    assert_eq!(report["geometry_dependencies"], "NotRun");
    assert_eq!(report["native_build"], "NotRun");
    assert_eq!(
        source.source.unwrap().find("903").unwrap().components[0].parameters[1].kind,
        ValueKind::String
    );
}

#[test]
fn source_reference_cycles_terminate_without_claiming_geometry_validity() {
    let data=append(&plane(AP214).replace("#202=LINE('',#100,#201);","#202=TRIMMED_CURVE('',#202,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(10.)),.T.,.PARAMETER.);"),"#910=GEOMETRIC_CURVE_SET('',(#911));#911=LINE('',#100,#201);#920=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#910),#12);#930=SHAPE_REPRESENTATION_RELATIONSHIP('',$,#20,#920);");
    let (source, result) = check(&data, true);
    assert_eq!(result.status, semantics::Status::Checked);
    assert!(result.dependency_visits < 10000);
    let snapshot = result.snapshot.as_ref().unwrap();
    assert_eq!(
        snapshot.source().find("202").unwrap().components[0].parameters[1].text,
        "202"
    );
    let report: serde_json::Value =
        serde_json::from_slice(&semantics::compile_report(&source, &result).unwrap()).unwrap();
    assert_eq!(report["geometry_dependencies"], "NotRun");
    assert_eq!(report["native_build"], "NotRun");
}

#[test]
fn shared_dependencies_are_visited_once_per_reach_and_budget_stop_withholds_part() {
    let mut records = String::from("#960=CONSTRUCTIVE_GEOMETRY_REPRESENTATION('',(");
    for i in 0..200 {
        if i != 0 {
            records.push(',');
        }
        write!(records, "#{}", 10000 + i).unwrap();
    }
    records.push_str(
        "),#12);#961=CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP('',$,#20,#960);\n",
    );
    for i in 0..200 {
        writeln!(records, "#{}=PLANE('',#133);", 10000 + i).unwrap();
    }
    let data = append(&plane(AP214), &records);
    let (source, good) = check(&data, true);
    assert_eq!(good.status, semantics::Status::Checked);
    assert!(good.dependency_visits < 50 * source.source.as_ref().unwrap().records().len());
    let mut options = semantics::Options {
        geometry_scope: semantics::GeometryScope::GeneralBrep,
        ..semantics::Options::default()
    };
    let before =
        semantics::check(&source, semantics::Limits::fixtures(), &options).dependency_visits;
    options.check_part_and_closure = true;
    let failed = semantics::check(
        &source,
        semantics::Limits {
            dependency_visits: before + 3,
            preserved_diagnostics: 1024,
        },
        &options,
    );
    assert_eq!(failed.status, semantics::Status::ResourceLimit);
    assert_eq!(failed.part, "ResourceLimit");
    assert_eq!(failed.si_units, "NotRun");
    assert!(failed.snapshot.is_none());
}

#[test]
fn inverse_roles_return_original_reference_occurrences_and_validate_the_role() {
    let data = append(
        &append(&plane(AP214), PROPERTY),
        "#905=PROPERTY_DEFINITION_REPRESENTATION(#900,#902);",
    );
    let (_, result) = check(&data, true);
    let snapshot = result.snapshot.unwrap();
    let uses = snapshot
        .used_in("900", "PROPERTY_DEFINITION_REPRESENTATION", "definition")
        .unwrap();
    assert_eq!(
        uses.iter()
            .map(
                |i| snapshot.source().records()[snapshot.source().references()[*i].record_index]
                    .id
                    .as_str()
            )
            .collect::<Vec<_>>(),
        ["901", "905"]
    );
    assert!(snapshot.used_in("900", "PRODUCT", "unknown_role").is_none());
    assert!(snapshot.used_in("999999", "PRODUCT", "name").is_none());
    assert!(snapshot
        .used_in("900", "UNKNOWN_TYPE", "definition")
        .is_none());
    let ac = snapshot
        .used_in("1", "APPLICATION_CONTEXT_ELEMENT", "frame_of_reference")
        .unwrap();
    assert_eq!(ac.len(), 2);
    let uses = snapshot
        .used_in("12", "REPRESENTATION", "context_of_items")
        .unwrap();
    assert_eq!(uses.len(), 1);
    assert_eq!(snapshot.source().references()[uses[0]].parameter_path, [3]);
}
