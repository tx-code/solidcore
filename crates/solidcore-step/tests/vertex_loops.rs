// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public source-only controls for the separately identified vertex-loop addition.
//!
//! Independent geometry description: a right cone with source apex (0,0,0),
//! base z=3 mm and base-circle radius 3 mm. The source semi-angle is the original
//! decimal 0.7853981633974483 rad, not a recomputed or exact symbolic pi/4.
//! There are two source vertices, one shared defining circle edge, two faces,
//! and one point loop. These tests qualify source facts only, not native `BRep`,
//! pole coincidence, exact area/volume or completed import.
//!
//! The common carrier uses empty STRING descriptions for PRODUCT, formation,
//! definition and PDS. Fixed modified N916 (020b4d25...), lines 2189--2196,
//! 2250--2256, 2231--2236 and 2305--2309 requires TEXT in those slots; '$' is
//! legal in the newer profiles but is not a legal N916 baseline for mutations.
use sha2::{Digest, Sha256};
use solidcore_step::{inspect, semantics as s, Limits, ValueKind};

const CONE: &str = include_str!("fixtures/cone-pole.step");
const AP214: &str = "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}";
const AP203: &str = "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
const PROFILES: [(&str, &str, usize); 4] = [
    (AP203, "Ap203e2GeneralSource", 139),
    (AP214, "Ap214e3Source", 136),
    (
        "AUTOMOTIVE_DESIGN {1 0 10303 214 1 1 1 1}",
        "Ap214MbxSource",
        151,
    ),
    ("CONFIG_CONTROL_DESIGN", "Ap203N916Source", 139),
];

fn options() -> s::Options {
    s::Options {
        geometry_scope: s::GeometryScope::GeneralBrep,
        check_geometry_dependencies: true,
        check_part_and_closure: true,
        ..s::Options::default()
    }
}

fn run(
    bytes: &[u8],
    options: &s::Options,
    limits: s::Limits,
) -> (solidcore_step::Outcome, s::Outcome) {
    let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
    let checked = s::check(&raw, limits, options);
    (raw, checked)
}

#[allow(clippy::unwrap_used)]
fn index(source: &s::Snapshot, id: &str) -> usize {
    source
        .source()
        .records()
        .iter()
        .position(|r| r.id == id)
        .unwrap()
}

fn minimal(data: &str, schema: &str) -> Vec<u8> {
    format!("ISO-10303-21;HEADER;FILE_DESCRIPTION(('public source-only'),'2;1');FILE_NAME('public.step','',(''),(''),'','','');FILE_SCHEMA(('{schema}'));ENDSEC;DATA;{data}ENDSEC;END-ISO-10303-21;").into_bytes()
}

#[test]
fn cpp_baseline_and_standard_addition_have_separate_compiled_identities() {
    fn canonical_addition(raw: &[u8]) -> Vec<u8> {
        // Match scripts/generate_step_catalog.py::canonical_addition: normalize
        // every checkout CRLF, then strip all trailing CR/LF bytes. The compiled
        // manifest identity must not depend on core.autocrlf or a final newline.
        let text = std::str::from_utf8(raw).unwrap().replace("\r\n", "\n");
        text.trim_end_matches(['\r', '\n']).as_bytes().to_vec()
    }

    assert_eq!(
        s::catalog_source_sha256(),
        "5aaff0fcd6256f93b8816e6a376818d46c76610c8fdaf3a1b46e056760cbe13b"
    );
    let manifest = include_bytes!("../catalog/standard-additions.v1.json");
    let canonical = canonical_addition(manifest);
    let mut lf = canonical.clone();
    lf.push(b'\n');
    let crlf = std::str::from_utf8(&lf)
        .unwrap()
        .replace('\n', "\r\n")
        .into_bytes();
    assert_ne!(lf, crlf);
    assert!(crlf.ends_with(b"\r\n"));
    let mut trailing_layout = crlf.clone();
    trailing_layout.extend_from_slice(b"\r\n\n\r");
    for bytes in [
        manifest.as_slice(),
        lf.as_slice(),
        crlf.as_slice(),
        trailing_layout.as_slice(),
    ] {
        let normalized = canonical_addition(bytes);
        assert_eq!(normalized, canonical);
        let fingerprint = format!("{:x}", Sha256::digest(&normalized));
        assert_eq!(s::catalog_extensions_sha256(), Some(fingerprint.as_str()));
    }
    assert_ne!(
        s::catalog_extensions_sha256(),
        Some(s::catalog_source_sha256())
    );
}

#[test]
fn four_general_profiles_preserve_loop_identity_original_fields_and_source_only_scope() {
    for (schema, kind, old_count) in PROFILES {
        let text = CONE.replace(AP214, schema);
        let (raw, checked) = run(text.as_bytes(), &options(), s::Limits::fixtures());
        assert_eq!(checked.status, s::Status::Checked, "{kind}");
        assert_eq!(checked.geometry_dependencies, "Passed");
        assert_eq!(checked.part, "Passed");
        assert_eq!(checked.si_units, "Passed");
        // Two WHERE predicates per face; separate existing cone-radius and
        // circle positive-length predicates remain separately counted.
        assert_eq!(checked.geometry_dependency_checks.entity_predicates, 5);
        assert_eq!(checked.geometry_dependency_checks.type_predicates, 1);
        assert_eq!(checked.geometry_dependency_checks.total, 6);
        let source = checked.snapshot.as_ref().unwrap();
        assert_eq!(source.profile().kind, kind);
        // This is one deliberately authored carrier legal under all four fixed
        // profiles, not a compatibility repair or a rewrite of an input capture.
        // Retain the distinction between an explicit empty TEXT and missing '$'.
        for id in ["4", "5", "7", "8"] {
            let description = source
                .field(index(source, id), None, "description")
                .unwrap();
            assert_eq!(description.value().unwrap().kind, ValueKind::String);
            assert_eq!(description.value().unwrap().text, "");
            if kind == "Ap203N916Source" {
                assert!(!description.declaration().unwrap().optional);
            }
        }
        let types = source.registered_types().collect::<Vec<_>>();
        assert_eq!(types.len(), old_count + 1);
        assert_eq!(types.last(), Some(&"VERTEX_LOOP"));
        assert_eq!(
            source.instances("VERTEX_LOOP").unwrap(),
            [index(source, "163")]
        );
        assert_eq!(source.instances("VERTEX_POINT").unwrap().len(), 2);
        assert_eq!(source.instances("EDGE_CURVE").unwrap().len(), 1);
        assert_eq!(source.instances("ADVANCED_FACE").unwrap().len(), 2);
        let loop_record = index(source, "163");
        for parent in [
            "LOOP",
            "TOPOLOGICAL_REPRESENTATION_ITEM",
            "REPRESENTATION_ITEM",
        ] {
            assert_eq!(source.is_a(loop_record, parent), Some(true));
        }
        for other in ["GEOMETRIC_REPRESENTATION_ITEM", "PATH", "EDGE_LOOP"] {
            assert_eq!(source.is_a(loop_record, other), Some(false));
        }
        let attrs = source.attributes(loop_record).unwrap();
        assert_eq!(
            attrs.iter().map(|a| a.name).collect::<Vec<_>>(),
            ["name", "loop_vertex"]
        );
        assert_eq!(attrs[0].declared_in, "REPRESENTATION_ITEM");
        assert_eq!(attrs[1].declared_in, "VERTEX_LOOP");
        assert_eq!(attrs[1].declared_type, "VERTEX");
        assert_eq!(attrs[1].parameter, 1);
        let loop_vertex = source.field(loop_record, None, "loop_vertex").unwrap();
        assert_eq!(loop_vertex.value().unwrap().kind, ValueKind::Reference);
        assert_eq!(loop_vertex.value().unwrap().text, "120");
        assert_eq!(loop_vertex.reference_target(), Some(index(source, "120")));
        let point = source
            .field(index(source, "120"), None, "vertex_geometry")
            .unwrap();
        assert_eq!(point.reference_target(), Some(index(source, "100")));
        let coords = source
            .field(index(source, "100"), None, "coordinates")
            .unwrap();
        assert_eq!(
            coords
                .value()
                .unwrap()
                .children
                .iter()
                .map(|v| v.text.as_str())
                .collect::<Vec<_>>(),
            ["0.", "0.", "0."]
        );
        let angle = source
            .field(index(source, "142"), None, "semi_angle")
            .unwrap();
        assert_eq!(angle.value().unwrap().text, "0.7853981633974483");
        assert_eq!(source.units().length_power10_mm, Some(0));
        assert_eq!(
            source.part_definition().unwrap().shape_role,
            "MaterialSolid"
        );
        let report: serde_json::Value =
            serde_json::from_slice(&s::compile_report(&raw, &checked).unwrap()).unwrap();
        for key in [
            "geometry_consistency",
            "planar_rules",
            "native_build",
            "full_import",
        ] {
            assert_eq!(report[key], "NotRun");
        }
        let retained = source.clone();
        drop(checked);
        drop(raw);
        assert_eq!(retained.source().bytes(), text.as_bytes());
        assert_eq!(loop_vertex.value().unwrap().text, "120");
        assert_eq!(angle.value().unwrap().text, "0.7853981633974483");
    }
}

#[test]
fn wrong_vertex_reference_is_a_prior_typed_error_at_the_original_source_slot() {
    for (schema, _, _) in PROFILES {
        let text = CONE
            .replace(AP214, schema)
            .replace("#163=VERTEX_LOOP('',#120);", "#163=VERTEX_LOOP('',#100);");
        let (raw, bad) = run(text.as_bytes(), &options(), s::Limits::fixtures());
        assert_eq!(bad.status, s::Status::Rejected);
        assert!(bad.snapshot.is_none());
        assert_eq!(bad.geometry_dependencies, "NotRun");
        assert_eq!(bad.geometry_dependency_checks.total, 0);
        let d = &bad.diagnostics[0];
        assert_eq!(d.source.key, "ReferenceTypeMismatch");
        assert_eq!(d.source.stage, "D1");
        assert_eq!(d.source.record_id, "163");
        assert_eq!(d.source.component, "VERTEX_LOOP");
        assert_eq!(d.field, "loop_vertex");
        assert_eq!(d.declared_type, "VERTEX");
        assert_eq!(d.source.parameter_path, [2]);
        assert_eq!(d.source.target_id, "100");
        let graph = raw.source.as_ref().unwrap();
        assert_eq!(graph.raw(d.source.primary).unwrap(), b"#100");
        assert_eq!(d.source.related, [graph.find("100").unwrap().id_span]);
        assert_eq!(raw.captured().unwrap(), text.as_bytes());
    }
}

#[test]
fn absent_required_vertex_and_missing_slot_do_not_reach_source_where_checks() {
    for (replacement, key, path) in [
        (
            "#163=VERTEX_LOOP('',$);",
            "RequiredAttributeMissing",
            vec![2],
        ),
        ("#163=VERTEX_LOOP('');", "AttributeCountMismatch", vec![]),
    ] {
        let text = CONE.replace("#163=VERTEX_LOOP('',#120);", replacement);
        let (raw, bad) = run(text.as_bytes(), &options(), s::Limits::fixtures());
        assert_eq!(bad.status, s::Status::Rejected);
        assert!(bad.snapshot.is_none());
        assert_eq!(bad.geometry_dependencies, "NotRun");
        assert_eq!(bad.diagnostics[0].source.key, key);
        assert_eq!(bad.diagnostics[0].source.stage, "D1");
        assert_eq!(bad.diagnostics[0].source.record_id, "163");
        assert_eq!(bad.diagnostics[0].source.parameter_path, path);
        assert_eq!(raw.captured().unwrap(), text.as_bytes());
    }
}

#[test]
fn wr8_qualification_does_not_admit_poly_loops_or_expand_the_planar_profile() {
    // A POLY_LOOP is outside this closed transport subset; do not relabel the
    // earlier unsupported-type guard as a WR8 observation or fabricate a pass.
    let text = CONE.replace("#163=VERTEX_LOOP('',#120);", "#163=POLY_LOOP('',(#100));");
    let (_, bad) = run(text.as_bytes(), &options(), s::Limits::fixtures());
    assert_eq!(bad.status, s::Status::Unsupported);
    assert_eq!(bad.diagnostics[0].source.key, "UnsupportedEntityType");
    assert_eq!(bad.diagnostics[0].source.record_id, "163");
    assert_eq!(bad.geometry_dependencies, "NotRun");
    let point_loop = minimal(
        "#1=VERTEX_LOOP('',#2);#2=VERTEX_POINT('',#3);#3=CARTESIAN_POINT('',(0.,0.,0.));",
        AP203,
    );
    let (_, p0) = run(
        &point_loop,
        &s::Options {
            check_geometry_dependencies: true,
            ..s::Options::default()
        },
        s::Limits::fixtures(),
    );
    assert_eq!(p0.status, s::Status::Unsupported);
    assert_eq!(p0.diagnostics[0].source.key, "UnsupportedEntityType");
    assert_eq!(p0.diagnostics[0].source.record_id, "1");
    assert_eq!(p0.geometry_dependencies, "NotRun");
    let (_, plane) = run(
        include_bytes!("fixtures/plane.step"),
        &s::Options {
            check_geometry_dependencies: true,
            check_part_and_closure: true,
            ..s::Options::default()
        },
        s::Limits::fixtures(),
    );
    assert_eq!(plane.status, s::Status::Checked);
    assert_eq!(plane.geometry_dependencies, "NotRun");
    assert_eq!(plane.snapshot.unwrap().registered_types().count(), 72);
}

#[test]
fn shorter_checks_do_not_silently_run_the_added_advanced_face_where_predicates() {
    let (_, shortened) = run(
        CONE.as_bytes(),
        &s::Options {
            geometry_scope: s::GeometryScope::GeneralBrep,
            ..s::Options::default()
        },
        s::Limits::fixtures(),
    );
    assert_eq!(shortened.status, s::Status::Checked);
    assert_eq!(shortened.geometry_dependencies, "NotRun");
    assert_eq!(shortened.geometry_dependency_checks.total, 0);
    assert_eq!(shortened.part, "NotRun");
    assert_eq!(shortened.si_units, "NotRun");
}

#[test]
fn where_budget_boundary_withholds_the_snapshot_and_replays_unchanged_source() {
    let raw = inspect(CONE.as_bytes().to_vec(), Limits::planar_fixtures());
    let options = s::Options {
        check_geometry_dependencies: true,
        geometry_scope: s::GeometryScope::GeneralBrep,
        ..s::Options::default()
    };
    let complete = s::check(&raw, s::Limits::fixtures(), &options);
    assert_eq!(complete.status, s::Status::Checked);
    let visits = complete.dependency_visits;
    assert!(visits > 0);
    let with_budget = |dependency_visits| {
        s::check(
            &raw,
            s::Limits {
                dependency_visits,
                ..s::Limits::fixtures()
            },
            &options,
        )
    };
    let insufficient = with_budget(visits - 1);
    assert_eq!(insufficient.status, s::Status::ResourceLimit);
    assert_eq!(insufficient.geometry_dependencies, "ResourceLimit");
    assert!(insufficient.snapshot.is_none());
    assert_eq!(
        insufficient.diagnostics[0].source.key,
        "DependencyVisitLimit"
    );
    let exact = with_budget(visits);
    assert_eq!(exact.status, s::Status::Checked);
    assert_eq!(exact.dependency_visits, visits);
    assert_eq!(exact.geometry_dependency_checks.total, 6);
    assert_eq!(exact.snapshot.unwrap().source().bytes(), CONE.as_bytes());
    assert_eq!(raw.captured().unwrap(), CONE.as_bytes());
    assert_eq!(raw.source.as_ref().unwrap().bytes(), CONE.as_bytes());
    let (_, replay) = run(
        CONE.as_bytes(),
        &options,
        s::Limits {
            dependency_visits: visits,
            ..s::Limits::fixtures()
        },
    );
    assert_eq!(replay.status, s::Status::Checked);
    assert_eq!(replay.dependency_visits, visits);
    assert_eq!(replay.snapshot.unwrap().source().bytes(), CONE.as_bytes());
}

#[test]
fn n916_verified_vertex_loop_never_admits_borrowed_foreign_metadata() {
    let text = CONE.replace(AP214, "CONFIG_CONTROL_DESIGN").replace(
        "ENDSEC;\nEND-ISO",
        "#900=MEASURE_REPRESENTATION_ITEM('foreign',LENGTH_MEASURE(1.),#10);\nENDSEC;\nEND-ISO",
    );
    let (_, bad) = run(text.as_bytes(), &options(), s::Limits::fixtures());
    assert_eq!(bad.status, s::Status::Unsupported);
    assert_eq!(bad.diagnostics[0].source.key, "UnsupportedEntityType");
    assert_eq!(bad.diagnostics[0].source.record_id, "900");
    assert_eq!(bad.geometry_dependencies, "NotRun");
}
