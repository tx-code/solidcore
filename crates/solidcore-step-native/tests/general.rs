// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! General construction through the same public interface used by the CLI.
//!
//! Public analytic box truth is independent of construction: eight distinct
//! corners, twelve shared edges, six directed faces, and one material region.
//! Construction alone never establishes accepted general import.
#![cfg(feature = "general-development")]

use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};
use solidcore_step_native::{construct_general, Error, GeneralQuality, Options};

fn quality() -> Result<GeneralQuality, Error> {
    Ok(GeneralQuality {
        id: "Q-General-analytic-fixture".into(),
        definition_mm: Rational::decimal("0.0000001")?,
        consistency_mm: Rational::decimal("0.000001")?,
        derived_uv_mm: Rational::decimal("0.000001")?,
        query_mm: Rational::decimal("0.000001")?,
        maximum_work_tolerance_mm: Rational::decimal("0.00001")?,
        maximum_tolerance_growth_mm: Rational::ZERO,
    })
}

fn build(bytes: &[u8], operations: usize) -> Result<solidcore_step_native::Candidate, Error> {
    let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
    construct_general(
        &raw,
        s::Limits::fixtures(),
        &s::Options::default(),
        Options {
            operations,
            work_tolerance_mm: 1e-6,
        },
        quality()?,
    )
}

#[test]
fn general_box_retains_owner_shared_topology_and_source_receipts() {
    let bytes = include_bytes!("../../solidcore-step/tests/fixtures/material-box.step");
    let model = build(bytes, 100_000).unwrap();
    assert_eq!(
        (
            model.vertices().len(),
            model.edges().len(),
            model.faces().len()
        ),
        (8, 12, 6)
    );
    assert_eq!(model.source().source().bytes(), bytes);
    assert_eq!(
        model.source().geometry_scope(),
        s::GeometryScope::GeneralBrep
    );
    assert_eq!(
        model.source().part_definition().unwrap().shape_role,
        "MaterialSolid"
    );
    let report = model.general_report().unwrap();
    assert_eq!(report.quality.id, "Q-General-analytic-fixture");
    assert_eq!(report.loops.len(), 6);
    assert!(report.chart_cuts.is_empty());
    assert_eq!(report.length_scale_mm, Rational::ONE);
    let mut incidence = std::collections::BTreeMap::new();
    for loop_receipt in report.loops {
        assert_eq!(loop_receipt.uses.len(), 4);
        for (_, edge, _) in &loop_receipt.uses {
            *incidence.entry(*edge).or_insert(0) += 1;
        }
    }
    assert_eq!(incidence.len(), 12);
    assert!(incidence.values().all(|n| *n == 2));
    assert_eq!(
        model
            .brep()
            .region_list()
            .iter()
            .filter(|id| model.brep().region(**id).unwrap().is_void == 0)
            .count(),
        1
    );
    // No planar certificate may be borrowed for a general candidate.
    assert!(model.general_observations().is_some());
    // Exercise the actual owning coincidence/ray paths, not just constructor
    // arena counts. This remains native validity, not general import acceptance.
    let checked = model.validate_native().unwrap();
    assert!(checked.general_report().is_some());
}

#[test]
fn general_budget_exhaustion_does_not_modify_the_next_request() {
    let bytes = include_bytes!("../../solidcore-step/tests/fixtures/material-box.step");
    let operations = build(bytes, 100_000).unwrap().operations();
    let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
    let run = |limit| {
        construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations: limit,
                work_tolerance_mm: 1e-6,
            },
            quality().unwrap(),
        )
    };
    assert!(matches!(run(operations - 1), Err(Error::ResourceLimit)));
    assert_eq!(run(operations).unwrap().operations(), operations);
    assert_eq!(run(operations).unwrap().source().source().bytes(), bytes);
}

#[test]
fn general_source_checks_cannot_be_disabled_or_surface_roles_promoted() {
    for bytes in [
        include_bytes!("../../solidcore-step/tests/fixtures/plane.step").as_slice(),
        include_bytes!("../../solidcore-step/tests/fixtures/surface-box.step").as_slice(),
    ] {
        let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
        let result = construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
            quality().unwrap(),
        );
        assert!(
            result.is_err(),
            "this general material-only increment must preserve surface roles"
        );
    }
    let bytes = include_bytes!("../../solidcore-step/tests/fixtures/material-box.step");
    let text = std::str::from_utf8(bytes)
        .unwrap()
        .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.MILLI.,.SECOND.)");
    assert_ne!(text.as_bytes(), bytes);
    let raw = inspect(text.into_bytes(), Limits::planar_fixtures());
    let result = construct_general(
        &raw,
        s::Limits::fixtures(),
        &s::Options::default(),
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
        quality().unwrap(),
    );
    assert!(matches!(result, Err(Error::Source { .. })));
}

#[test]
fn failed_development_unit_proof_keeps_the_original_marker_diagnostic_for_native_callers() {
    let data = include_str!("../../solidcore-step/tests/fixtures/material-box.step")
        .replace("AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}", "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}")
        .replace("#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));", "#11=(CONVERSION_BASED_UNIT('arbitrary',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());")
        .replace("ENDSEC;\nEND-ISO", "#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.5),#901);\n#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n#902=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND-ISO");
    let raw = inspect(data.into_bytes(), Limits::planar_fixtures());
    let result = construct_general(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            development_unit_policy: s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions,
            ..Default::default()
        },
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
        quality().unwrap(),
    );
    let Err(Error::Source {
        status,
        diagnostics,
        development_unit_diagnostics,
        preserved_diagnostics,
        interpretation_diagnostics,
    }) = result
    else {
        panic!("invalid physical dimensions must not construct a native model");
    };
    assert_eq!(status, s::Status::Rejected);
    assert!(diagnostics
        .iter()
        .any(|d| d.source.key == "DevelopmentPlaneAngleDimensionsMustBeZero"));
    assert_eq!(development_unit_diagnostics.len(), 1);
    assert_eq!(
        development_unit_diagnostics[0].source.key,
        "DerivedSlotMarkerRequired"
    );
    assert!(preserved_diagnostics.is_empty());
    assert!(interpretation_diagnostics.is_empty());
}
