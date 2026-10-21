// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Explicit application loading choices over the owning native import pipeline.
//! Quality comes from the shared exact parser; source interpretation, complete
//! semantics and publication remain owned by the existing reader/native layers.

use crate::scene::{self, Result, Scene};
use solidcore_step::{planar, semantics as s, InspectOptions};
use solidcore_step_native::Options;
use std::ffi::{OsStr, OsString};
use std::io::Read;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, Default, PartialEq)]
#[allow(clippy::struct_excessive_bools)] // Independent explicit source-policy switches.
pub(crate) struct LoadOptions {
    pub general_development: bool,
    pub quality: Option<PathBuf>,
    pub interop: bool,
    pub ap214_unregistered_as_2010: bool,
    pub opaque_header_name: bool,
    pub opaque_data_strings: bool,
    pub level1_as_corrected_encoding: bool,
    pub development_ap214_angle_dimensions: bool,
    pub measure_material_accuracy: Option<f64>,
}

impl LoadOptions {
    pub fn parse_argument(
        &mut self,
        argument: &OsStr,
        arguments: &mut impl Iterator<Item = OsString>,
    ) -> Result<bool> {
        match argument.to_str() {
            Some("--general-development") => self.general_development = true,
            Some("--quality") => {
                self.quality = Some(
                    arguments
                        .next()
                        .ok_or("--quality requires a JSON file")?
                        .into(),
                );
            }
            Some("--interop") => self.interop = true,
            Some("--ap214-unregistered-as-2010") => self.ap214_unregistered_as_2010 = true,
            Some("--opaque-header-name") => self.opaque_header_name = true,
            Some("--opaque-data-strings") => self.opaque_data_strings = true,
            Some("--level1-as-corrected-encoding") => self.level1_as_corrected_encoding = true,
            Some("--development-ap214-angle-dimensions") => {
                self.development_ap214_angle_dimensions = true;
            }
            Some("--measure-material-accuracy") => {
                let value = arguments
                    .next()
                    .ok_or("--measure-material-accuracy requires a value")?;
                let value: f64 = value
                    .to_str()
                    .ok_or("Accuracy must be numeric ASCII")?
                    .parse::<f64>()
                    .map_err(|error| format!("Invalid material accuracy: {error}"))?;
                if !value.is_finite() || !(1e-8..=1e-4).contains(&value) {
                    return Err(
                        "Material integration accuracy must be finite and within [1e-8,1e-4]"
                            .into(),
                    );
                }
                self.measure_material_accuracy = Some(value);
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    pub fn validate(&self) -> Result<()> {
        // Match step-native-import's independent policy gates. Selecting interop
        // never enables a carrier or physical-unit exception implicitly.
        if self.general_development != self.quality.is_some()
            || (!self.general_development && self.interop)
            || (self.ap214_unregistered_as_2010 && !self.interop)
        {
            return Err("General mode requires --quality; source policy/schema options require general mode, and schema assumptions require --interop".into());
        }
        if self.carrier() != InspectOptions::default()
            && (!self.general_development || !self.interop)
        {
            return Err(
                "Carrier exceptions require explicit --general-development and --interop".into(),
            );
        }
        if self.development_ap214_angle_dimensions && !self.general_development {
            return Err("--development-ap214-angle-dimensions requires --general-development and an explicit quality configuration".into());
        }
        if let Some(accuracy) = self.measure_material_accuracy {
            if !self.general_development
                || !accuracy.is_finite()
                || !(1e-8..=1e-4).contains(&accuracy)
            {
                return Err("Material measurements require general mode, quality and accuracy within [1e-8,1e-4]".into());
            }
        }
        Ok(())
    }

    fn carrier(&self) -> InspectOptions {
        InspectOptions {
            header_name_policy: if self.opaque_header_name {
                solidcore_step::HeaderNamePolicy::PreserveOpaqueBytes
            } else {
                solidcore_step::HeaderNamePolicy::StrictAscii
            },
            data_string_policy: if self.opaque_data_strings {
                solidcore_step::DataStringPolicy::PreserveOpaqueBytes
            } else {
                solidcore_step::DataStringPolicy::StrictAscii
            },
            carrier_interpretation: if self.level1_as_corrected_encoding {
                solidcore_step::CarrierInterpretation::Level1AsCorrectedEncoding
            } else {
                solidcore_step::CarrierInterpretation::DeclaredOnly
            },
        }
    }

    fn semantics(&self) -> s::Options {
        s::Options {
            source_policy: if self.interop {
                s::SourcePolicy::PreserveNonGeometric
            } else {
                s::SourcePolicy::Strict
            },
            schema_interpretation: if self.ap214_unregistered_as_2010 {
                s::SchemaInterpretation::UnregisteredAp214As2010
            } else {
                s::SchemaInterpretation::DeclaredOnly
            },
            development_unit_policy: if self.development_ap214_angle_dimensions {
                s::DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions
            } else {
                s::DevelopmentUnitPolicy::Disabled
            },
            ..s::Options::default()
        }
    }
}

fn read_bounded(path: &Path, maximum: usize, kind: &str) -> Result<Vec<u8>> {
    let limit = u64::try_from(maximum).map_err(|error| error.to_string())?;
    let mut bytes = Vec::new();
    std::fs::File::open(path)
        .map_err(|error| error.to_string())?
        .take(limit.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|error| error.to_string())?;
    if bytes.len() > maximum {
        return Err(format!("{kind} exceeds {maximum} bytes"));
    }
    Ok(bytes)
}

pub(crate) fn load(
    path: &Path,
    options: &LoadOptions,
    progress: impl Fn(&str) -> Result<()>,
) -> Result<Scene> {
    options.validate()?;
    progress("Reading bounded STEP source")?;
    let bytes = read_bounded(
        path,
        solidcore_step::Limits::planar_fixtures().input_bytes,
        "STEP input",
    )?;
    from_bytes(bytes, options, progress)
}

pub(crate) fn from_bytes(
    bytes: Vec<u8>,
    options: &LoadOptions,
    progress: impl Fn(&str) -> Result<()>,
) -> Result<Scene> {
    options.validate()?;
    progress("Inspecting original STEP bytes")?;
    let raw = solidcore_step::inspect_with_options(
        bytes,
        solidcore_step::Limits::planar_fixtures(),
        options.carrier(),
    );
    if raw.status != solidcore_step::Status::Inspected {
        return Err(format!(
            "Source {:?}: {:?}",
            raw.status,
            raw.diagnostics.first()
        ));
    }
    let candidate = if options.general_development {
        progress("Reading explicit General quality configuration")?;
        let path = options
            .quality
            .as_ref()
            .ok_or("Missing General quality configuration")?;
        let bytes = read_bounded(path, 65_536, "Quality JSON")?;
        let selected = solidcore_step_native::parse_general_quality_json(&bytes)
            .map_err(|error| error.to_string())?;
        let initial = selected.initial_work_tolerance_mm;
        progress("Checking complete General source semantics and constructing native geometry")?;
        // construct_general owns the complete semantic pass and its source proof;
        // do not create a parallel interpretation or promote a resettable report.
        solidcore_step_native::construct_general(
            &raw,
            s::Limits::fixtures(),
            &options.semantics(),
            Options {
                operations: 100_000,
                work_tolerance_mm: initial.numerator() as f64 / initial.denominator() as f64,
            },
            selected.quality,
        )
        .map_err(|error| error.to_string())?
    } else {
        // This is the existing strict P0 path, with the same explicit fixture
        // quality/resource selection. General recognition is never a fallback.
        progress("Checking P0 source roles and units")?;
        let semantic = s::check(
            &raw,
            s::Limits::fixtures(),
            &s::Options {
                check_part_and_closure: true,
                check_geometry_dependencies: true,
                ..s::Options::default()
            },
        );
        if semantic.status != s::Status::Checked {
            return Err(format!(
                "Semantics {:?}: {:?}",
                semantic.status,
                semantic.diagnostics.first()
            ));
        }
        progress("Checking the explicit P0 planar profile")?;
        let planar = planar::check(
            &semantic,
            planar::Quality::planar_fixture(),
            planar::Limits::fixtures(),
        );
        if planar.status != planar::Status::Checked {
            return Err(format!(
                "Planar {:?}: {:?}",
                planar.status,
                planar.diagnostics.first()
            ));
        }
        progress("Constructing the P0 native geometry")?;
        solidcore_step_native::construct_planar(
            &planar,
            Options {
                operations: 100_000,
                work_tolerance_mm: 1e-6,
            },
        )
        .map_err(|error| error.to_string())?
    };
    progress("Running native pointer and level-two checks")?;
    let mut candidate = candidate.validate_native_detailed().map_err(|failure| {
        let reports = failure.reports.reports();
        let shown = reports.iter().take(8).collect::<Vec<_>>();
        format!(
            "Native {:?} check failed (completed={}): {}; reports={} (first eight): {shown:#?}",
            failure.stage,
            failure.completed,
            failure.error,
            reports.len()
        )
    })?;
    if options.general_development {
        progress("Measuring final native UV trim correspondence")?;
        candidate = candidate
            .measure_general_trims()
            .map_err(|error| error.to_string())?;
        if let Some(accuracy) = options.measure_material_accuracy {
            progress("Measuring numerical native material properties")?;
            candidate = candidate
                .measure_general_material(accuracy)
                .map_err(|error| error.to_string())?;
        }
    }
    progress("Deriving display geometry from the checked native owner")?;
    let scene =
        scene::from_candidate_with_options(candidate, scene::DisplayOptions::default(), &progress)?;
    // A close/cancel arriving during extraction must also drop the complete
    // unpublished scene. The main worker checks its request again before delivery.
    progress("Complete native scene prepared")?;
    Ok(scene)
}

pub(crate) fn qualification_lines(scene: &Scene, options: &LoadOptions) -> Vec<String> {
    let mut lines = vec!["Native pointer and level-two checks passed.".into()];
    let source = scene.owner.source();
    if let Some(report) = scene.owner.general_report() {
        lines.push(format!("Quality: {} · mm budgets", report.quality.id));
        lines.push(
            if report.final_trim_measurements.is_some() {
                "Final UV correspondence: measured observations"
            } else {
                "Final UV observations invalidated by a later query"
            }
            .into(),
        );
        lines.push(
            if report.material_measurements.is_some() {
                "Material properties: numerical observations"
            } else if options.measure_material_accuracy.is_some() {
                "Material observations invalidated; remeasurement required"
            } else {
                "Material properties: not requested"
            }
            .into(),
        );
        if source.source_policy() == s::SourcePolicy::PreserveNonGeometric {
            lines.push(format!(
                "Nongeometric interop: {} original diagnostics retained",
                source.preserved_diagnostics().len()
            ));
        }
        if let Some(receipt) = source.development_unit_receipt() {
            lines.push(format!(
                "Unit compatibility applied to {} unit(s): source syntax unverified",
                receipt.affected_units().len()
            ));
        } else if options.development_ap214_angle_dimensions {
            lines.push("Unit compatibility requested; no exception applied".into());
        }
        if source.applied_interpretation() != s::SchemaInterpretation::DeclaredOnly {
            lines.push("Schema assumption applied: original identity unverified".into());
        }
        if source.source().carrier_assumption().is_some() {
            lines.push("Carrier interpretation applied: declaration unverified".into());
        }
        let carrier = source.source().options();
        if carrier.header_name_policy != solidcore_step::HeaderNamePolicy::StrictAscii
            || carrier.data_string_policy != solidcore_step::DataStringPolicy::StrictAscii
        {
            lines.push("Opaque byte preservation selected; text is not inferred".into());
        }
    } else {
        lines.push("P0 planar fixture quality selected".into());
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::io::Write;
    use std::sync::atomic::{AtomicUsize, Ordering};

    const PLANE: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/plane.step");
    const CONE: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/cone-pole.step");
    const CYLINDER: &[u8] =
        include_bytes!("../../solidcore-step-native/tests/fixtures/cylinder-phase-rings.step");
    const QUALITY: &[u8] = br#"{"id":"Q-Viewer-public-controls","definition_mm":"0.0000001","consistency_mm":"0.000001","derived_uv_mm":"0.000001","query_mm":"0.000001","initial_work_tolerance_mm":"0.000001","maximum_work_tolerance_mm":"0.00001","maximum_tolerance_growth_mm":"0"}"#;
    static NEXT_FILE: AtomicUsize = AtomicUsize::new(0);

    struct TemporaryQuality(PathBuf);
    impl TemporaryQuality {
        fn new(bytes: &[u8]) -> Self {
            for _ in 0..100 {
                let serial = NEXT_FILE.fetch_add(1, Ordering::Relaxed);
                let path = std::env::temp_dir().join(format!(
                    "solidcore-viewer-quality-{}-{serial}.json",
                    std::process::id()
                ));
                match std::fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(&path)
                {
                    Ok(mut file) => {
                        file.write_all(bytes).unwrap();
                        return Self(path);
                    }
                    Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {}
                    Err(error) => panic!("public quality fixture: {error}"),
                }
            }
            panic!("public quality fixture name collision limit")
        }
        fn options(&self) -> LoadOptions {
            LoadOptions {
                general_development: true,
                quality: Some(self.0.clone()),
                ..LoadOptions::default()
            }
        }
    }
    impl Drop for TemporaryQuality {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    fn parse(words: &[&str]) -> Result<crate::Cli> {
        crate::parse_arguments(words.iter().map(|word| OsString::from(*word)))?
            .ok_or("unexpected help".into())
    }

    #[test]
    fn cli_defaults_remain_p0_and_general_quality_is_never_inferred() {
        let default = parse(&["synthetic.step"]).unwrap();
        assert!(!default.fixture);
        assert_eq!(default.load, LoadOptions::default());
        for words in [
            vec!["synthetic.step", "--general-development"],
            vec!["synthetic.step", "--quality", "q.json"],
            vec!["synthetic.step", "--interop"],
            vec!["synthetic.step", "--development-ap214-angle-dimensions"],
            vec!["synthetic.step", "--measure-material-accuracy", "1e-7"],
            vec!["synthetic.step", "--check"],
            vec![
                "synthetic.step",
                "--fixture-limits",
                "--check",
                "--screenshot",
                "frame.png",
            ],
        ] {
            assert!(parse(&words).is_err(), "{words:?}");
        }
        let selected = parse(&[
            "synthetic.step",
            "--fixture-limits",
            "--check",
            "--general-development",
            "--quality",
            "q.json",
        ])
        .unwrap();
        assert!(selected.check && selected.fixture && selected.load.general_development);
        assert_eq!(selected.load.quality, Some(PathBuf::from("q.json")));
        assert_eq!(selected.load.measure_material_accuracy, None);
    }

    #[test]
    fn cli_policy_gates_match_the_independent_native_choices() {
        let prefix = [
            "synthetic.step",
            "--general-development",
            "--quality",
            "q.json",
        ];
        let ordinary = parse(&prefix).unwrap().load;
        assert_eq!(ordinary.carrier(), InspectOptions::default());
        assert_eq!(
            ordinary.semantics().development_unit_policy,
            s::DevelopmentUnitPolicy::Disabled
        );
        for flag in [
            "--ap214-unregistered-as-2010",
            "--opaque-header-name",
            "--opaque-data-strings",
            "--level1-as-corrected-encoding",
        ] {
            let mut words = prefix.to_vec();
            words.push(flag);
            assert!(parse(&words).is_err(), "{flag}");
            words.push("--interop");
            assert!(parse(&words).is_ok(), "{flag}");
        }
        let mut words = prefix.to_vec();
        words.extend([
            "--interop",
            "--ap214-unregistered-as-2010",
            "--opaque-header-name",
            "--opaque-data-strings",
            "--level1-as-corrected-encoding",
            "--development-ap214-angle-dimensions",
            "--measure-material-accuracy",
            "1e-7",
        ]);
        let selected = parse(&words).unwrap().load;
        assert_eq!(
            selected.carrier(),
            InspectOptions {
                header_name_policy: solidcore_step::HeaderNamePolicy::PreserveOpaqueBytes,
                data_string_policy: solidcore_step::DataStringPolicy::PreserveOpaqueBytes,
                carrier_interpretation:
                    solidcore_step::CarrierInterpretation::Level1AsCorrectedEncoding,
            }
        );
        assert_eq!(
            selected.semantics().schema_interpretation,
            s::SchemaInterpretation::UnregisteredAp214As2010
        );
        assert_eq!(selected.measure_material_accuracy, Some(1e-7));
        let mut words = prefix.to_vec();
        words.push("--development-ap214-angle-dimensions");
        let independent = parse(&words).unwrap().load;
        assert!(!independent.interop);
        assert_eq!(independent.carrier(), InspectOptions::default());
        for value in ["NaN", "inf", "0", "1e-9", "1e-3", "-1"] {
            let mut words = prefix.to_vec();
            words.extend(["--measure-material-accuracy", value]);
            assert!(parse(&words).is_err(), "{value}");
        }
    }

    #[test]
    fn default_loader_executes_the_original_public_p0_scene() {
        let scene = from_bytes(PLANE.to_vec(), &LoadOptions::default(), |_| Ok(())).unwrap();
        assert_eq!(scene.owner.source().profile().kind, "Ap203e2Planar");
        assert_eq!(scene.owner.source().source().bytes(), PLANE);
        assert!(scene.owner.general_report().is_none());
        assert!(scene.owner.native_checks().is_some());
        assert!(!scene.triangles.is_empty() && !scene.lines.is_empty());
    }

    #[test]
    fn explicit_general_loader_reaches_the_actual_curved_scene_without_implicit_material_accuracy()
    {
        let quality = TemporaryQuality::new(QUALITY);
        for bytes in [CONE, CYLINDER] {
            let scene = from_bytes(bytes.to_vec(), &quality.options(), |_| Ok(())).unwrap();
            assert_eq!(scene.owner.source().source().bytes(), bytes);
            assert!(scene.owner.native_checks().is_some());
            let report = scene.owner.general_report().unwrap();
            assert_eq!(report.quality.id, "Q-Viewer-public-controls");
            assert!(report.final_trim_measurements.is_some());
            assert!(report.material_measurements.is_none());
            assert!(!scene.triangles.is_empty() && !scene.lines.is_empty());
        }
        let mut options = quality.options();
        options.measure_material_accuracy = Some(1e-7);
        let scene = from_bytes(CONE.to_vec(), &options, |_| Ok(())).unwrap();
        let measured = scene
            .owner
            .general_report()
            .unwrap()
            .material_measurements
            .unwrap();
        assert_eq!(measured.requested_relative_accuracy, 1e-7);
        assert!((measured.material_volume_mm3 - 9. * std::f64::consts::PI).abs() < 1e-3);
    }

    #[test]
    fn cancellation_before_capture_native_work_and_after_complete_extraction_never_returns_a_scene()
    {
        let cancelled = load(
            Path::new("absent-public.step"),
            &LoadOptions::default(),
            |_| Err("cancelled".into()),
        );
        assert_eq!(cancelled.err().unwrap(), "cancelled");
        let quality = TemporaryQuality::new(QUALITY);
        let reached = Cell::new(false);
        let cancelled = from_bytes(CONE.to_vec(), &quality.options(), |stage| {
            if stage.starts_with("Checking complete General") {
                reached.set(true);
                Err("cancelled before native".into())
            } else {
                Ok(())
            }
        });
        assert!(reached.get());
        assert_eq!(cancelled.err().unwrap(), "cancelled before native");
        let cancelled = from_bytes(PLANE.to_vec(), &LoadOptions::default(), |stage| {
            if stage == "Complete native scene prepared" {
                Err("cancelled at handoff".into())
            } else {
                Ok(())
            }
        });
        assert_eq!(cancelled.err().unwrap(), "cancelled at handoff");
        assert!(from_bytes(PLANE.to_vec(), &LoadOptions::default(), |_| Ok(())).is_ok());
    }

    #[test]
    fn quality_input_is_bounded_and_uses_the_shared_no_default_parser() {
        for bytes in [vec![b' '; 65_537], b"{}".to_vec()] {
            let quality = TemporaryQuality::new(&bytes);
            assert!(from_bytes(CONE.to_vec(), &quality.options(), |_| Ok(())).is_err());
        }
        let quality = TemporaryQuality::new(QUALITY);
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../solidcore-step/tests/fixtures/cone-pole.step");
        let scene = load(&path, &quality.options(), |_| Ok(())).unwrap();
        assert_eq!(
            scene.owner.general_report().unwrap().quality.id,
            "Q-Viewer-public-controls"
        );
    }

    fn cone_with_explicit_angle_marker_defect() -> String {
        std::str::from_utf8(CONE).unwrap()
            .replace("#11=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));","#11=(CONVERSION_BASED_UNIT('not a scale inference',#900) NAMED_UNIT(#902) PLANE_ANGLE_UNIT());")
            .replace("#142=CONICAL_SURFACE('',#106,0.,0.7853981633974483);","#142=CONICAL_SURFACE('',#106,0.,45.);")
            .replace("ENDSEC;\nEND-ISO","#900=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#901);\n#901=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n#902=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND-ISO")
    }

    #[test]
    fn verified_unit_exception_stays_explicit_and_the_hud_keeps_original_syntax_unverified() {
        let text = cone_with_explicit_angle_marker_defect();
        let quality = TemporaryQuality::new(QUALITY);
        let strict = from_bytes(text.as_bytes().to_vec(), &quality.options(), |_| Ok(()));
        assert!(strict.err().unwrap().contains("DerivedSlotMarkerRequired"));
        let mut options = quality.options();
        options.development_ap214_angle_dimensions = true;
        let scene = from_bytes(text.as_bytes().to_vec(), &options, |_| Ok(())).unwrap();
        assert_eq!(scene.owner.source().source().bytes(), text.as_bytes());
        let receipt = scene.owner.source().development_unit_receipt().unwrap();
        assert!(!receipt.source_syntax_verified());
        assert!(scene
            .owner
            .source()
            .development_unit_diagnostics()
            .iter()
            .any(|diagnostic| diagnostic.source.key == "DerivedSlotMarkerRequired"));
        let rows = qualification_lines(&scene, &options);
        assert_eq!(
            rows.iter()
                .filter(|row| row.contains("source syntax unverified"))
                .count(),
            1
        );
        assert!(rows.iter().all(|row| !row.contains("accepted import")));
    }

    #[test]
    fn actual_ray_invalidates_hud_observations_without_losing_source_or_unit_proof() {
        use solidcore::math::Vec3;

        let text = cone_with_explicit_angle_marker_defect();
        let quality = TemporaryQuality::new(QUALITY);
        let mut options = quality.options();
        options.development_ap214_angle_dimensions = true;
        options.measure_material_accuracy = Some(1e-7);
        let mut scene = from_bytes(text.as_bytes().to_vec(), &options, |_| Ok(())).unwrap();
        let source = scene.owner.source().clone();
        let receipt = source.development_unit_receipt().unwrap();
        assert!(!receipt.source_syntax_verified());
        assert!(!receipt.affected_units().is_empty());
        assert!(receipt
            .original_diagnostics()
            .iter()
            .any(|diagnostic| diagnostic.source.key == "DerivedSlotMarkerRequired"));
        let before = qualification_lines(&scene, &options);
        assert!(before
            .iter()
            .any(|row| row == "Final UV correspondence: measured observations"));
        assert!(before
            .iter()
            .any(|row| row == "Material properties: numerical observations"));
        let unit_row = before
            .iter()
            .find(|row| row.contains("source syntax unverified"))
            .unwrap();

        // Owning native queries can populate caches, so previous numerical
        // observations expire even though the immutable source proof survives.
        // Independent right-cone truth: at z=1.5 the radius is 1.5 mm; a ray
        // from +Y first meets the conical face at (0,1.5,1.5), 6.5 mm away.
        let hit = scene
            .owner
            .ray_hit(Vec3::new(0., 8., 1.5), -Vec3::Y, 1e-6, None)
            .unwrap()
            .unwrap();
        assert!(matches!(hit.entity, solidcore::EntityRef::Face(_)));
        assert!(hit.point_mm.distance(Vec3::new(0., 1.5, 1.5)) < 1e-7);
        assert!((hit.distance_mm - 6.5).abs() < 1e-7);
        let report = scene.owner.general_report().unwrap();
        assert!(report.final_trim_measurements.is_none());
        assert!(report.material_measurements.is_none());

        let after = qualification_lines(&scene, &options);
        assert!(after
            .iter()
            .any(|row| row == "Final UV observations invalidated by a later query"));
        assert!(after
            .iter()
            .any(|row| row == "Material observations invalidated; remeasurement required"));
        assert!(after.iter().all(|row| {
            row != "Final UV correspondence: measured observations"
                && row != "Material properties: numerical observations"
        }));
        assert!(after.contains(unit_row));
        assert_eq!(scene.owner.source().source().bytes(), text.as_bytes());
        assert_eq!(
            scene.owner.source().units().plane_angle_unit,
            source.units().plane_angle_unit
        );
        // The source snapshot keeps the very same sealed proof, including its
        // original diagnostics and conversion-factor lexemes, after ray execution.
        assert!(std::ptr::eq(
            receipt,
            scene.owner.source().development_unit_receipt().unwrap()
        ));
    }
}
