// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Public analytic fixture through the actual reader, source checks and Rust `BRep`.
use solidcore_step::{inspect, planar as p, semantics as s, Limits};
use solidcore_step_native::{construct_planar, AcceptanceQuality, Error, Options};

fn fixture_quality() -> Result<AcceptanceQuality, p::ExactError> {
    Ok(AcceptanceQuality {
        id: "Q-Rust-affine-fixture-01".into(),
        definition_mm: p::Rational::decimal("0.0000001")?,
        derived_uv_mm: p::Rational::decimal("0.0000001")?,
        consistency_mm: p::Rational::decimal("0.0000001")?,
        maximum_work_tolerance_mm: p::Rational::decimal("0.00001")?,
    })
}

#[test]
fn open_plane_requires_continuous_evidence_and_explicit_quality() {
    let source = checked();
    let build = || {
        construct_planar(
            &source,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap()
    };
    let imported = build()
        .accept_open_planar(fixture_quality().unwrap())
        .unwrap();
    assert_eq!(
        (
            imported.vertices().len(),
            imported.edges().len(),
            imported.faces().len()
        ),
        (4, 4, 1)
    );
    let certificate = imported.certificate();
    for bound in [
        certificate.definition_mm,
        certificate.derived_uv_mm,
        certificate.consistency_mm,
    ] {
        assert!(bound.is_finite() && bound > 0. && bound <= 1e-7);
    }
    assert_eq!(
        imported.source().part_definition().unwrap().shape_role,
        "OpenSurfaceModel"
    );

    let mut too_tight = fixture_quality().unwrap();
    too_tight.definition_mm = p::Rational::decimal("0.000000000000000001").unwrap();
    assert!(matches!(
        build().accept_open_planar(too_tight),
        Err(Error::Evidence(_))
    ));
    let mut working = fixture_quality().unwrap();
    working.maximum_work_tolerance_mm = p::Rational::decimal("0.0000001").unwrap();
    assert!(matches!(
        build().accept_open_planar(working),
        Err(Error::Evidence(_))
    ));
    let mut invalid = fixture_quality().unwrap();
    invalid.id.clear();
    assert!(matches!(
        build().accept_open_planar(invalid),
        Err(Error::Invalid(_))
    ));
    // A failed request never mutates the immutable source or the next request.
    assert!(build()
        .accept_open_planar(fixture_quality().unwrap())
        .is_ok());
}

#[test]
fn closed_surface_and_material_box_do_not_borrow_open_plane_qualification() {
    for bytes in [
        include_bytes!("../../solidcore-step/tests/fixtures/material-box.step").as_slice(),
        include_bytes!("../../solidcore-step/tests/fixtures/surface-box.step").as_slice(),
    ] {
        let source = checked_bytes(bytes);
        let candidate = construct_planar(
            &source,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap();
        assert!(matches!(
            candidate.accept_open_planar(fixture_quality().unwrap()),
            Err(Error::Unsupported(_))
        ));
    }
}

#[test]
fn exact_plan_preserves_units_edge_sense_and_the_final_budget_gate() {
    let text = std::str::from_utf8(include_bytes!(
        "../../solidcore-step/tests/fixtures/plane.step"
    ))
    .unwrap();
    let text = text
        .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)")
        .replace(
            "EDGE_CURVE('',#110,#111,#202,.T.)",
            "EDGE_CURVE('',#111,#110,#202,.F.)",
        )
        .replace(
            "ORIENTED_EDGE('',*,*,#203,.T.)",
            "ORIENTED_EDGE('',*,*,#203,.F.)",
        );
    let source = checked_bytes(text.as_bytes());
    assert_eq!(source.status, p::Status::Checked);
    let candidate = construct_planar(
        &source,
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )
    .unwrap();
    let construction_operations = candidate.operations();
    assert_eq!(construction_operations, 17);
    let source_id = source.evidence.faces[0].loops[0].vertices[0];
    let vertex = candidate
        .brep()
        .vertex(candidate.vertices()[&source_id])
        .unwrap();
    assert_eq!(vertex.point, solidcore::math::Vec3::new(30., -20., 50.));
    let imported = candidate
        .accept_open_planar(fixture_quality().unwrap())
        .unwrap();
    assert_eq!(imported.source().source().bytes(), text.as_bytes());
    for (operations, succeeds) in [
        (construction_operations + 3, false),
        (construction_operations + 4, true),
    ] {
        let candidate = construct_planar(
            &source,
            Options {
                operations,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap();
        let result = candidate.accept_open_planar(fixture_quality().unwrap());
        if succeeds {
            assert!(result.is_ok());
        } else {
            assert!(matches!(result, Err(Error::ResourceLimit)));
        }
    }
}

#[test]
fn mutable_diagnostic_projection_cannot_rewrite_checked_source_geometry() {
    let mut source = checked();
    let original_origin = source.evidence.faces[0].origin_mm;
    source.evidence.faces[0].origin_mm = [p::Rational::ZERO; 3];
    source.evidence.faces.clear();
    source.evidence.edges.clear();
    let model = construct_planar(
        &source,
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )
    .unwrap();
    let imported = model
        .accept_open_planar(fixture_quality().unwrap())
        .unwrap();
    assert_eq!(imported.faces().len(), 1);
    assert_eq!(
        source.checked_evidence().unwrap().faces[0].origin_mm,
        original_origin
    );

    let text = std::str::from_utf8(include_bytes!(
        "../../solidcore-step/tests/fixtures/plane.step"
    ))
    .unwrap();
    let broken = text.replace(
        "#203=EDGE_CURVE('',#110,#111,#202,.T.)",
        "#203=EDGE_CURVE('',#110,#110,#202,.T.)",
    );
    let mut rejected = checked_bytes(broken.as_bytes());
    assert_ne!(rejected.status, p::Status::Checked);
    rejected.status = p::Status::Checked;
    rejected.evidence = source.checked_evidence().unwrap().clone();
    assert!(matches!(
        construct_planar(
            &rejected,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6
            }
        ),
        Err(Error::Upstream)
    ));
}

fn checked() -> p::Outcome {
    checked_bytes(include_bytes!(
        "../../solidcore-step/tests/fixtures/plane.step"
    ))
}
fn checked_bytes(bytes: &[u8]) -> p::Outcome {
    let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
    let sem = s::check(
        &raw,
        s::Limits::fixtures(),
        &s::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    p::check(&sem, p::Quality::planar_fixture(), p::Limits::fixtures())
}

#[test]
fn identical_box_boundary_retains_material_and_closed_surface_roles() {
    for (bytes, material) in [
        (
            include_bytes!("../../solidcore-step/tests/fixtures/material-box.step").as_slice(),
            true,
        ),
        (
            include_bytes!("../../solidcore-step/tests/fixtures/surface-box.step").as_slice(),
            false,
        ),
    ] {
        let source = checked_bytes(bytes);
        assert_eq!(source.status, p::Status::Checked);
        let model = construct_planar(
            &source,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap();
        assert_eq!(
            (
                model.vertices().len(),
                model.edges().len(),
                model.faces().len()
            ),
            (8, 12, 6)
        );
        let material_count = model
            .brep()
            .region_list()
            .iter()
            .filter(|r| model.brep().region(**r).unwrap().is_void == 0)
            .count();
        assert_eq!(material_count, usize::from(material));
    }
}
#[test]
fn actual_step_source_constructs_owning_rust_topology_and_passes_native_checks() {
    let source = checked();
    let model = construct_planar(
        &source,
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )
    .unwrap();
    assert_eq!(model.vertices().len(), 4);
    assert_eq!(model.edges().len(), 4);
    assert_eq!(model.faces().len(), 1);
    assert!(model.native_checks().is_none());
    let model = model.validate_native().unwrap();
    drop(source);
    assert_eq!(model.native_checks().unwrap().pointer_reports, 0);
    assert_eq!(model.native_checks().unwrap().deep_reports, 0);
    assert_eq!(
        model.source().part_definition().unwrap().shape_role,
        "OpenSurfaceModel"
    );
}
#[test]
fn construction_budget_failure_does_not_damage_the_source_or_next_request() {
    let source = checked();
    for operations in [0, 1, 3, 6, 10, 16] {
        assert!(matches!(
            construct_planar(
                &source,
                Options {
                    operations,
                    work_tolerance_mm: 1e-6
                }
            ),
            Err(Error::ResourceLimit)
        ));
        assert_eq!(source.status, p::Status::Checked);
    }
    assert!(construct_planar(
        &source,
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6
        }
    )
    .is_ok());
    for operations in [17, 18] {
        let candidate = construct_planar(
            &source,
            Options {
                operations,
                work_tolerance_mm: 1e-6,
            },
        )
        .unwrap();
        assert!(matches!(
            candidate.validate_native(),
            Err(Error::ResourceLimit)
        ));
    }
}
