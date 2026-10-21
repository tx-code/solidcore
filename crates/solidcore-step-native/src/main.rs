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
        carrier: solidcore_step::InspectOptions::default(),
    };
    let mut fixture = false;
    while let Some(arg) = args.next() {
        match arg.to_str() {
            Some("--help" | "-h") => {
                println!("step-native-import --fixture-limits FILE [--general-development --quality JSON] [--interop] [--ap214-unregistered-as-2010] [--opaque-header-name] [--opaque-data-strings] [--level1-as-corrected-encoding] [--development-ap214-angle-dimensions]\n\nConstruction checkpoints only; accepted_import remains false. General mode runs complete typed source checks, never the P0 rule subset. Its quality JSON must explicitly provide all budgets in mm. Input and work limits selected by --fixture-limits are development values, not production defaults. Carrier exceptions require explicit general + interoperability mode. Opaque strings retain bytes with unestablished text; their typed uses still require semantic approval. Level-1 interpretation remains an unverified assumption, never a corrected declaration. The independent development angle-unit policy only continues after source-reference, dimension and conversion proofs; original syntax remains unverified and results are separate from strict qualification.");
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
    Ok(Some(result))
}

#[cfg(feature = "general-development")]
fn general(
    raw: &solidcore_step::Outcome,
    args: &Arguments,
) -> Result<solidcore_step_native::Candidate, Box<dyn std::error::Error>> {
    use solidcore_step::{planar::Rational, semantics as s};
    use solidcore_step_native::{GeneralQuality, Options};
    const FIELDS: [&str; 8] = [
        "id",
        "definition_mm",
        "consistency_mm",
        "derived_uv_mm",
        "query_mm",
        "initial_work_tolerance_mm",
        "maximum_work_tolerance_mm",
        "maximum_tolerance_growth_mm",
    ];
    let file = args
        .quality
        .as_ref()
        .ok_or("Missing explicit quality JSON")?;
    let mut bytes = Vec::new();
    std::fs::File::open(file)?
        .take(65537)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 65536 {
        return Err("Quality JSON exceeds 64 KiB".into());
    }
    let value: serde_json::Value = serde_json::from_slice(&bytes)?;
    let object = value.as_object().ok_or("Quality JSON must be an object")?;
    if object.len() != FIELDS.len() || object.keys().any(|k| !FIELDS.contains(&k.as_str())) {
        return Err(
            "Quality JSON must contain exactly id and the seven named mm budget strings".into(),
        );
    }
    let text = |key: &str| -> Result<&str, Box<dyn std::error::Error>> {
        object
            .get(key)
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| format!("Quality field {key} must be a string").into())
    };
    let number = |key: &str| -> Result<Rational, Box<dyn std::error::Error>> {
        Ok(Rational::decimal(text(key)?).map_err(solidcore_step_native::Error::Numeric)?)
    };
    let initial = number("initial_work_tolerance_mm")?;
    let quality = GeneralQuality {
        id: text("id")?.into(),
        definition_mm: number("definition_mm")?,
        consistency_mm: number("consistency_mm")?,
        derived_uv_mm: number("derived_uv_mm")?,
        query_mm: number("query_mm")?,
        maximum_work_tolerance_mm: number("maximum_work_tolerance_mm")?,
        maximum_tolerance_growth_mm: number("maximum_tolerance_growth_mm")?,
    };
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
        quality,
    )?)
}

#[cfg(not(feature = "general-development"))]
fn general(
    _raw: &solidcore_step::Outcome,
    _args: &Arguments,
) -> Result<solidcore_step_native::Candidate, Box<dyn std::error::Error>> {
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
        let model = general(&raw, &args)?
            .validate_native()?
            .measure_general_trims()?;
        let general_report = model.general_report().ok_or("Missing general report")?;
        let trims = general_report
            .final_trim_measurements
            .ok_or("Missing final trim measurements")?;
        println!(
            "{}",
            serde_json::json!({"status":"NativeConstructed","accepted_import":false,
            "general_development":true,"source_policy":model.source().source_policy(),
            "development_unit_policy":model.source().development_unit_policy(),
            "development_unit_receipt":model.source().development_unit_receipt(),
            "source_sha256":raw.sha256,"carrier_options":raw.options(),
            "carrier_assumption":raw.carrier_assumption(),
            "schema_interpretation":model.source().applied_interpretation(),
            "vertices":model.vertices().len(),"edges":model.edges().len(),"faces":model.faces().len(),
            "operations":model.operations(),"native_pointer_validation":"Passed","native_deep_validation":"Passed",
            "final_trim_measurements":{"status":"Passed","maximum_mm":trims.maximum_mm,
            "samples":trims.samples,"edge_uses":trims.edge_uses,"evidence":"finite_same_parameter_samples_not_a_continuous_bound"},
            "unrun":["material_measurements","general_development_publication"],"repair":"None"})
        );
        return Ok(());
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
        "native_kernel":"Rust solidcore","vertices":model.vertices().len(),"edges":model.edges().len(),
        "faces":model.faces().len(),"operations":model.operations(),
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
}
