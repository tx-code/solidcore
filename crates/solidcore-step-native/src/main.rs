// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit synthetic construction checkpoint; not a full-import success CLI.
use solidcore_step::semantics::DevelopmentUnitPolicy;
use std::io::Read;
use std::path::PathBuf;

struct Arguments {
    path: PathBuf,
    general: bool,
    interop: bool,
    ap214: bool,
    development_unit_policy: DevelopmentUnitPolicy,
    quality: Option<PathBuf>,
    material_accuracy: Option<f64>,
    carrier: solidcore_step::InspectOptions,
}

fn arguments() -> Result<Option<Arguments>, Box<dyn std::error::Error>> {
    parse_arguments(std::env::args_os().skip(1))
}

fn parse_arguments(
    mut args: impl Iterator<Item = std::ffi::OsString>,
) -> Result<Option<Arguments>, Box<dyn std::error::Error>> {
    let mut result = Arguments {
        path: PathBuf::new(),
        general: false,
        interop: false,
        ap214: false,
        development_unit_policy: DevelopmentUnitPolicy::Disabled,
        quality: None,
        material_accuracy: None,
        carrier: solidcore_step::InspectOptions::default(),
    };
    let mut fixture = false;
    while let Some(arg) = args.next() {
        match arg.to_str() {
            Some("--help" | "-h") => {
                println!("step-native-import --fixture-limits FILE [--general-development --quality JSON] [--interop] [--ap214-unregistered-as-2010] [--opaque-header-name] [--opaque-data-strings] [--level1-as-corrected-encoding] [--development-ap214-angle-dimensions] [--measure-material-accuracy VALUE]\n\nConstruction checkpoints only; accepted_import remains false. General mode runs complete typed source checks, never the P0 rule subset. Its quality JSON must explicitly provide all budgets in mm. Input and work limits selected by --fixture-limits are development values, not production defaults. Carrier exceptions require explicit general + interoperability mode. Opaque strings retain bytes with unestablished text; their typed uses still require semantic approval. Level-1 interpretation remains an unverified assumption, never a corrected declaration. The independent development angle-unit policy only continues after source-reference, dimension and conversion proofs; original syntax remains unverified and results are separate from strict qualification. Material integration requires an independent explicit accuracy in [1e-8,1e-4]; numerical measurements do not grant import acceptance.");
                return Ok(None);
            }
            Some("--fixture-limits") => fixture = true,
            Some("--general-development") => result.general = true,
            Some("--interop") => result.interop = true,
            Some("--ap214-unregistered-as-2010") => result.ap214 = true,
            Some("--development-ap214-angle-dimensions") => {
                result.development_unit_policy =
                    DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions;
            }
            Some("--opaque-header-name") => {
                result.carrier.header_name_policy =
                    solidcore_step::HeaderNamePolicy::PreserveOpaqueBytes;
            }
            Some("--opaque-data-strings") => {
                result.carrier.data_string_policy =
                    solidcore_step::DataStringPolicy::PreserveOpaqueBytes;
            }
            Some("--level1-as-corrected-encoding") => {
                result.carrier.carrier_interpretation =
                    solidcore_step::CarrierInterpretation::Level1AsCorrectedEncoding;
            }
            Some("--quality") => {
                result.quality = Some(args.next().ok_or("--quality requires a JSON file")?.into());
            }
            Some("--measure-material-accuracy") => {
                let value = args
                    .next()
                    .ok_or("--measure-material-accuracy requires a value")?;
                let value: f64 = value
                    .to_str()
                    .ok_or("Accuracy must be numeric ASCII")?
                    .parse()?;
                if !value.is_finite() || !(1e-8..=1e-4).contains(&value) {
                    return Err(
                        "Material integration accuracy must be finite and within [1e-8,1e-4]"
                            .into(),
                    );
                }
                result.material_accuracy = Some(value);
            }
            Some(flag) if flag.starts_with('-') => {
                return Err(format!("Unknown option {flag}; use --help").into())
            }
            _ => {
                if !result.path.as_os_str().is_empty() {
                    return Err("Supply exactly one STEP file".into());
                }
                result.path = arg.into();
            }
        }
    }
    if !fixture || result.path.as_os_str().is_empty() {
        return Err("Explicit --fixture-limits and a STEP file are required; use --help".into());
    }
    if result.general != result.quality.is_some()
        || (!result.general && result.interop)
        || result.ap214 && !result.interop
    {
        return Err("General mode requires --quality; source policy/schema options require general mode, and schema assumptions require --interop".into());
    }
    if result.carrier != solidcore_step::InspectOptions::default()
        && (!result.general || !result.interop)
    {
        return Err(
            "Carrier exceptions require explicit --general-development and --interop".into(),
        );
    }
    if result.development_unit_policy != DevelopmentUnitPolicy::Disabled && !result.general {
        return Err("--development-ap214-angle-dimensions requires --general-development and an explicit quality configuration".into());
    }
    if result.material_accuracy.is_some() && !result.general {
        return Err("Material measurements require explicit general mode and quality".into());
    }
    Ok(Some(result))
}

#[cfg(feature = "general-development")]
fn general(
    raw: &solidcore_step::Outcome,
    args: &Arguments,
) -> Result<solidcore_step_native::Candidate, Box<dyn std::error::Error>> {
    use solidcore_step::semantics as s;
    use solidcore_step_native::{parse_general_quality_json, Options};
    let file = args
        .quality
        .as_ref()
        .ok_or("Missing explicit quality JSON")?;
    let mut bytes = Vec::new();
    std::fs::File::open(file)?
        .take(65537)
        .read_to_end(&mut bytes)?;
    let selection = parse_general_quality_json(&bytes)?;
    let initial = selection.initial_work_tolerance_mm;
    let options = s::Options {
        development_unit_policy: args.development_unit_policy,
        source_policy: if args.interop {
            s::SourcePolicy::PreserveNonGeometric
        } else {
            s::SourcePolicy::Strict
        },
        schema_interpretation: if args.ap214 {
            s::SchemaInterpretation::UnregisteredAp214As2010
        } else {
            s::SchemaInterpretation::DeclaredOnly
        },
        ..Default::default()
    };
    Ok(solidcore_step_native::construct_general(
        raw,
        s::Limits::fixtures(),
        &options,
        Options {
            operations: 100_000,
            work_tolerance_mm: initial.numerator() as f64 / initial.denominator() as f64,
        },
        selection.quality,
    )?)
}

// CLI development diagnostics are bounded independently from the library's
// complete owning array. No raw STEP bytes, model or filesystem path is included.
#[cfg(feature = "general-development")]
const NATIVE_DIAGNOSTIC_LIMIT: usize = 128;

#[cfg(feature = "general-development")]
fn native_diagnostic_number(value: f64) -> serde_json::Value {
    if value == solidcore::math::UNDEF_DOUBLE {
        serde_json::json!("Undefined")
    } else if value.is_finite() {
        serde_json::json!(value)
    } else if value.is_nan() {
        serde_json::json!("NaN")
    } else if value.is_sign_negative() {
        serde_json::json!("-Infinity")
    } else {
        serde_json::json!("Infinity")
    }
}

#[cfg(feature = "general-development")]
fn capture_source_semantics(source: &solidcore_step::semantics::Snapshot) -> serde_json::Value {
    // Borrow this candidate's sealed owner BEFORE validation consumes it. The
    // serialized receipt retains its original diagnostics/false syntax flag;
    // neither resettable progress nor the native failure can upgrade it.
    serde_json::json!({
        "source_policy": source.source_policy(),
        "schema_interpretation": source.applied_interpretation(),
        "development_unit_policy": source.development_unit_policy(),
        "development_unit_receipt": source.development_unit_receipt(),
        "development_unit_diagnostics": source.development_unit_diagnostics(),
        "source_syntax_verified": source.development_unit_receipt().map(solidcore_step::semantics::DevelopmentUnitReceipt::source_syntax_verified)
    })
}

#[cfg(feature = "general-development")]
fn native_validation_failure_summary(
    failure: &solidcore_step_native::NativeValidationFailure,
    source_semantics: &serde_json::Value,
) -> serde_json::Value {
    use solidcore_step_native::NativeValidationStage;
    let diagnostics = failure
        .reports
        .reports()
        .iter()
        .take(NATIVE_DIAGNOSTIC_LIMIT)
        .map(|r| {
            serde_json::json!({
                "name": r.name, "type": format!("{:?}", r.assert_type),
                "typeCode": r.assert_type.code(), "test": r.test_index,
                "reportingType": r.reporting_type, "listIndex": r.list_index,
                // Hexadecimal strings retain all 64 bits through JSON/JS readers.
                // These keys describe the discarded model, never live query handles.
                "reporterKey": r.reporter.map(|k| format!("{:016x}", k.0)),
                "ownerKey": r.owner.map(|k| format!("{:016x}", k.0)),
                "ownerType": r.owner_type, "ownerTypeName": r.owner_type_string,
                "otherKey": r.other.map(|k| format!("{:016x}", k.0)),
                "otherType": r.other_type, "otherTypeName": r.other_type_string,
                "message": r.message.as_deref(), "messageForFile": r.message_for_file,
                "ok": r.ok,
                "ownerParam": [native_diagnostic_number(r.owner_param.x), native_diagnostic_number(r.owner_param.y)],
                "otherParam": [native_diagnostic_number(r.other_param.x), native_diagnostic_number(r.other_param.y)],
                "tol": native_diagnostic_number(r.tol3d),
                "value": native_diagnostic_number(r.value),
                "distance3d": native_diagnostic_number(r.dist3d)
            })
        })
        .collect::<Vec<_>>();
    serde_json::json!({
        "schema": "solidcore.step.native-validation-failure.v1",
        "status": "NativeValidationFailed", "accepted_import": false,
        "stage": match failure.stage { NativeValidationStage::Pointer => "Pointer", NativeValidationStage::Deep => "Deep" },
        "completed": failure.completed, "total_reports": failure.reports.len(),
        "diagnostics_first": diagnostics,
        "truncated": failure.reports.len() > NATIVE_DIAGNOSTIC_LIMIT,
        "native_ids": "diagnostic_keys_from_discarded_candidate_not_live_handles",
        "source_semantics": source_semantics
    })
}

#[cfg(feature = "general-development")]
fn run_general(
    raw: &solidcore_step::Outcome,
    args: &Arguments,
) -> Result<(), Box<dyn std::error::Error>> {
    let candidate = general(raw, args)?;
    let source_semantics = capture_source_semantics(candidate.source());
    let checked = match candidate.validate_native_detailed() {
        Ok(checked) => checked,
        Err(failure) => {
            eprintln!(
                "{}",
                native_validation_failure_summary(&failure, &source_semantics)
            );
            // Keep the original Error display and process exit after the extra
            // owning diagnostic evidence; failure never returns a partial model.
            return Err(failure.error.into());
        }
    };
    let mut model = checked.measure_general_trims()?;
    if let Some(accuracy) = args.material_accuracy {
        model = model.measure_general_material(accuracy)?;
    }
    let general_report = model.general_report().ok_or("Missing general report")?;
    let trims = general_report
        .final_trim_measurements
        .ok_or("Missing final trim measurements")?;
    let material = general_report.material_measurements.map(|m| serde_json::json!({
        "status":"Passed", "requested_relative_accuracy":m.requested_relative_accuracy,
        "area_mm2":m.area_mm2, "material_volume_mm3":m.material_volume_mm3,
        "centroid_mm":[m.centroid_mm.x,m.centroid_mm.y,m.centroid_mm.z],
        "boundary_mm":m.boundary_mm, "evidence":"numerical_observations_not_continuous_error_bounds"
    }));
    let unrun = if material.is_some() {
        vec![
            "general_continuous_correspondence",
            "general_development_publication",
        ]
    } else {
        vec![
            "material_measurements",
            "general_continuous_correspondence",
            "general_development_publication",
        ]
    };
    println!(
        "{}",
        serde_json::json!({"status":"NativeConstructed","accepted_import":false,
        "general_development":true,"source_policy":model.source().source_policy(),
        "development_unit_policy":model.source().development_unit_policy(),
        "development_unit_receipt":model.source().development_unit_receipt(),
        "source_sha256":raw.sha256,"carrier_options":raw.options(),
        "carrier_assumption":raw.carrier_assumption(),
        "schema_interpretation":model.source().applied_interpretation(),
        "vertices":model.brep().vertex_list().len(),"edges":model.brep().edge_list().len(),"faces":model.brep().face_list().len(),
        "operations":model.operations(),"native_pointer_validation":"Passed","native_deep_validation":"Passed",
        "correspondence":{"source_vertices":model.vertices().len(),"source_edges":model.edges().len(),
        "native_edge_pieces":model.edges().values().map(Vec::len).sum::<usize>(),
        "derived_vertices":model.derived_vertices().len(),"chart_cuts":general_report.chart_cuts.len()},
        "final_trim_measurements":{"status":"Passed","maximum_mm":trims.maximum_mm,
        "samples":trims.samples,"edge_uses":trims.edge_uses,"evidence":"finite_same_parameter_samples_not_a_continuous_bound"},
        "material_measurements":material,"unrun":unrun,"repair":"None"})
    );
    Ok(())
}

#[cfg(not(feature = "general-development"))]
fn run_general(
    _raw: &solidcore_step::Outcome,
    _args: &Arguments,
) -> Result<(), Box<dyn std::error::Error>> {
    Err("General construction requires building with --features cli,general-development".into())
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let Some(args) = arguments()? else {
        return Ok(());
    };
    let limits = solidcore_step::Limits::planar_fixtures();
    let mut bytes = Vec::new();
    std::fs::File::open(&args.path)?
        .take(u64::try_from(limits.input_bytes)?.saturating_add(1))
        .read_to_end(&mut bytes)?;
    // The existing source reader owns byte preservation and the immutable
    // assumption receipt (StepSource.cpp at d8ed45a). These flags select that
    // contract; they neither rewrite raw text nor approve opaque typed fields.
    let raw = solidcore_step::inspect_with_options(bytes, limits, args.carrier);
    if args.general {
        return run_general(&raw, &args);
    }
    let sem = solidcore_step::semantics::check(
        &raw,
        solidcore_step::semantics::Limits::fixtures(),
        &solidcore_step::semantics::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    let planar = solidcore_step::planar::check(
        &sem,
        solidcore_step::planar::Quality::planar_fixture(),
        solidcore_step::planar::Limits::fixtures(),
    );
    let model = solidcore_step_native::construct_planar(
        &planar,
        solidcore_step_native::Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )?
    .validate_native()?;
    println!(
        "{}",
        serde_json::json!({"status":"NativeConstructed","accepted_import":false,
        "native_kernel":"Rust solidcore","vertices":model.brep().vertex_list().len(),"edges":model.brep().edge_list().len(),
        "faces":model.brep().face_list().len(),"operations":model.operations(),
        "native_pointer_validation":"Passed","native_deep_validation":"Passed",
        "unrun":["continuous_source_correspondence","error_budgets","final_publication"],"repair":"None"})
    );
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_arguments, DevelopmentUnitPolicy};
    use solidcore_step::{
        CarrierInterpretation, DataStringPolicy, HeaderNamePolicy, InspectOptions,
    };

    #[test]
    fn development_units_require_general_mode_and_are_independent_of_property_interop() {
        let parse = |args: &[&str]| parse_arguments(args.iter().copied().map(Into::into));
        let strict = parse(&["--fixture-limits", "synthetic.step"])
            .unwrap()
            .unwrap();
        assert_eq!(
            strict.development_unit_policy,
            DevelopmentUnitPolicy::Disabled
        );
        assert!(parse(&[
            "--fixture-limits",
            "synthetic.step",
            "--development-ap214-angle-dimensions"
        ])
        .is_err());
        let requested = parse(&[
            "--fixture-limits",
            "synthetic.step",
            "--general-development",
            "--quality",
            "quality.json",
            "--development-ap214-angle-dimensions",
        ])
        .unwrap()
        .unwrap();
        assert_eq!(
            requested.development_unit_policy,
            DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions
        );
        assert!(!requested.interop);
        assert_eq!(requested.carrier, InspectOptions::default());
    }

    #[test]
    fn material_accuracy_is_explicit_and_cannot_be_clamped_or_selected_in_planar_mode() {
        let parse = |extra: &[&str], general| {
            let mut words = vec!["--fixture-limits", "synthetic.step"];
            if general {
                words.extend(["--general-development", "--quality", "quality.json"]);
            }
            words.extend_from_slice(extra);
            parse_arguments(words.into_iter().map(Into::into))
        };
        assert!(parse(&[], true)
            .unwrap()
            .unwrap()
            .material_accuracy
            .is_none());
        for value in ["0.00000001", "0.0000001", "0.0001"] {
            assert_eq!(
                parse(&["--measure-material-accuracy", value], true)
                    .unwrap()
                    .unwrap()
                    .material_accuracy,
                Some(value.parse().unwrap())
            );
            assert!(parse(&["--measure-material-accuracy", value], false).is_err());
        }
        for value in ["NaN", "inf", "-1", "0", "1e-9", "1e-3"] {
            assert!(parse(&["--measure-material-accuracy", value], true).is_err());
        }
        assert!(parse(&["--measure-material-accuracy"], true).is_err());
    }

    #[test]
    fn carrier_policies_are_explicit_independent_and_never_implicit_in_interop() {
        let base = [
            "--fixture-limits",
            "synthetic.step",
            "--general-development",
            "--quality",
            "quality.json",
            "--interop",
        ];
        let parse = |flags: &[&str]| {
            parse_arguments(
                base.into_iter()
                    .chain(flags.iter().copied())
                    .map(Into::into),
            )
            .unwrap()
            .unwrap()
        };
        assert_eq!(parse(&[]).carrier, InspectOptions::default());
        let header = parse(&["--opaque-header-name"]).carrier;
        assert_eq!(
            header.header_name_policy,
            HeaderNamePolicy::PreserveOpaqueBytes
        );
        assert_eq!(header.data_string_policy, DataStringPolicy::StrictAscii);
        assert_eq!(
            header.carrier_interpretation,
            CarrierInterpretation::DeclaredOnly
        );
        let all = parse(&[
            "--opaque-header-name",
            "--opaque-data-strings",
            "--level1-as-corrected-encoding",
        ])
        .carrier;
        assert_eq!(
            all.data_string_policy,
            DataStringPolicy::PreserveOpaqueBytes
        );
        assert_eq!(
            all.carrier_interpretation,
            CarrierInterpretation::Level1AsCorrectedEncoding
        );
        for flag in [
            "--opaque-header-name",
            "--opaque-data-strings",
            "--level1-as-corrected-encoding",
        ] {
            for prefix in [
                vec!["--fixture-limits", "synthetic.step"],
                vec![
                    "--fixture-limits",
                    "synthetic.step",
                    "--general-development",
                    "--quality",
                    "quality.json",
                ],
            ] {
                assert!(parse_arguments(prefix.into_iter().chain([flag]).map(Into::into)).is_err());
            }
        }
    }

    #[cfg(feature = "general-development")]
    #[test]
    fn failure_summary_uses_real_native_reports_and_never_returns_a_model() {
        use solidcore::geom::assert_array::AssertArray;
        use solidcore::topology::{brep_validation, Brep};
        use solidcore_step_native::{Error, NativeValidationFailure, NativeValidationStage};
        // Native required-links validation produces test113 for this real empty
        // BRep without infinite-region infrastructure; no report is fabricated.
        let mut reports = AssertArray::new();
        let error = brep_validation::required_links(&Brep::new(), Some(&mut reports)).unwrap_err();
        let failure = NativeValidationFailure {
            error: Error::Kernel(error),
            stage: NativeValidationStage::Pointer,
            completed: false,
            reports,
        };
        let original_error = failure.error.to_string();
        let summary = super::native_validation_failure_summary(&failure, &serde_json::json!({}));
        assert_eq!(summary["status"], "NativeValidationFailed");
        assert_eq!(summary["accepted_import"], false);
        assert_eq!(summary["stage"], "Pointer");
        assert_eq!(summary["completed"], false);
        assert_eq!(summary["total_reports"], failure.reports.len());
        assert_eq!(summary["truncated"], false);
        assert_eq!(summary["diagnostics_first"][0]["test"], 113);
        assert!(summary["diagnostics_first"][0]["ownerKey"].is_string());
        assert!(!summary["diagnostics_first"][0]["name"]
            .as_str()
            .unwrap()
            .is_empty());
        assert!(summary.get("model").is_none());
        assert!(summary.get("raw").is_none());
        assert!(summary.get("path").is_none());
        assert_eq!(failure.error.to_string(), original_error);
        assert_eq!(failure.reports.len(), 1);
        assert!(serde_json::to_vec(&summary).is_ok());
    }

    #[cfg(feature = "general-development")]
    #[test]
    fn failure_source_context_keeps_original_development_receipt_after_owner_drop() {
        use solidcore_step::{inspect, semantics as s, Limits};
        let text = include_str!("../../solidcore-step/tests/fixtures/plane.step")
            .replace("AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}", "AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}")
            .replace("#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));", "#11=(CONVERSION_BASED_UNIT('public angle',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());")
            .replace("ENDSEC;\nEND-ISO", "#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#901);\n#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n#902=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND-ISO");
        let raw = inspect(text.as_bytes().to_vec(), Limits::planar_fixtures());
        let source = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                geometry_scope: s::GeometryScope::GeneralBrep,
                check_geometry_dependencies: true,
                check_part_and_closure: true,
                development_unit_policy: DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions,
                ..s::Options::default()
            },
        );
        assert_eq!(source.status, s::Status::CheckedWithDiagnostics);
        let snapshot = source.snapshot.as_ref().unwrap();
        let context = super::capture_source_semantics(snapshot);
        drop(source);
        drop(raw);
        assert_eq!(
            context["development_unit_policy"],
            "Ap214ExplicitPlaneAngleDimensions"
        );
        assert_eq!(context["source_syntax_verified"], false);
        assert_eq!(
            context["development_unit_receipt"]["sourceSyntaxVerified"],
            false
        );
        assert_eq!(
            context["development_unit_receipt"]["formalAPValidity"],
            "Unestablished"
        );
        assert_eq!(
            context["development_unit_diagnostics"][0]["source"]["key"],
            "DerivedSlotMarkerRequired"
        );
        assert!(context.get("raw").is_none());
    }
}
