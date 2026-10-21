// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Pure shared parsing of the explicit native-development quality selection.
//!
//! Contract: this crate's README, Explicit general development construction.
//! The constructor remains responsible for the semantic budget checks. Parsing
//! introduces no defaults, unit inference, filesystem access or floating roundtrip.

use crate::GeneralQuality;
use solidcore_step::planar::{ExactError, Rational};

const MAXIMUM_BYTES: usize = 65536;
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

/// An explicit application selection, retaining exact rational millimeter values.
/// Parsing alone does not establish that the constructor accepts the budgets.
#[derive(Clone, Debug)]
pub struct GeneralQualitySelection {
    /// Named source/native quality budgets; no values are inferred.
    pub quality: GeneralQuality,
    /// Separate initial native working tolerance in millimeters.
    pub initial_work_tolerance_mm: Rational,
}

/// A bounded quality document could not be decoded with the existing CLI contract.
#[derive(Debug)]
pub enum GeneralQualityJsonError {
    /// More than 64 KiB of input, checked before JSON parsing.
    TooLarge,
    /// Invalid JSON, including malformed UTF-8.
    Json(serde_json::Error),
    /// The root value is not an object.
    NotObject,
    /// A required field is absent or an additional field was supplied.
    Fields,
    /// The named field is not a string.
    Field(&'static str),
    /// A decimal cannot be represented by the bounded exact rational contract.
    Numeric(ExactError),
}
impl std::fmt::Display for GeneralQualityJsonError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooLarge => f.write_str("Quality JSON exceeds 64 KiB"),
            Self::Json(error) => std::fmt::Display::fmt(error, f),
            Self::NotObject => f.write_str("Quality JSON must be an object"),
            Self::Fields => f.write_str(
                "Quality JSON must contain exactly id and the seven named mm budget strings",
            ),
            Self::Field(key) => write!(f, "Quality field {key} must be a string"),
            // Preserve the prior CLI's Error::Numeric display, not a new
            // floating-parser message or an invented source diagnostic.
            Self::Numeric(error) => std::fmt::Display::fmt(&crate::Error::Numeric(*error), f),
        }
    }
}
impl std::error::Error for GeneralQualityJsonError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Json(error) => Some(error),
            _ => None,
        }
    }
}

/// Parse the same exact eight-string, at-most-64-KiB quality document as the CLI.
/// Lengths stay exact rational millimeters; no source uncertainty or defaults
/// enter the selection. The general constructor still validates budget meaning.
///
/// # Errors
/// Input size, JSON/schema shape, field types or bounded decimal conversion.
pub fn parse_general_quality_json(
    bytes: &[u8],
) -> Result<GeneralQualitySelection, GeneralQualityJsonError> {
    if bytes.len() > MAXIMUM_BYTES {
        return Err(GeneralQualityJsonError::TooLarge);
    }
    let value: serde_json::Value =
        serde_json::from_slice(bytes).map_err(GeneralQualityJsonError::Json)?;
    let object = value
        .as_object()
        .ok_or(GeneralQualityJsonError::NotObject)?;
    if object.len() != FIELDS.len() || object.keys().any(|key| !FIELDS.contains(&key.as_str())) {
        return Err(GeneralQualityJsonError::Fields);
    }
    let text = |key: &'static str| {
        object
            .get(key)
            .and_then(serde_json::Value::as_str)
            .ok_or(GeneralQualityJsonError::Field(key))
    };
    let number =
        |key: &'static str| Rational::decimal(text(key)?).map_err(GeneralQualityJsonError::Numeric);
    // Preserve the previous CLI's error precedence: initial tolerance first,
    // followed by id and the declared quality budgets in their original order.
    let initial_work_tolerance_mm = number("initial_work_tolerance_mm")?;
    let quality = GeneralQuality {
        id: text("id")?.into(),
        definition_mm: number("definition_mm")?,
        consistency_mm: number("consistency_mm")?,
        derived_uv_mm: number("derived_uv_mm")?,
        query_mm: number("query_mm")?,
        maximum_work_tolerance_mm: number("maximum_work_tolerance_mm")?,
        maximum_tolerance_growth_mm: number("maximum_tolerance_growth_mm")?,
    };
    Ok(GeneralQualitySelection {
        quality,
        initial_work_tolerance_mm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn document() -> serde_json::Value {
        serde_json::json!({
            "id": "public-explicit-quality",
            "definition_mm": "0.0000001",
            "consistency_mm": "0.000001",
            "derived_uv_mm": "0.000002",
            "query_mm": "0.000003",
            "initial_work_tolerance_mm": "0.000004",
            "maximum_work_tolerance_mm": "0.00001",
            "maximum_tolerance_growth_mm": "0"
        })
    }

    fn parse(
        value: &serde_json::Value,
    ) -> Result<GeneralQualitySelection, GeneralQualityJsonError> {
        parse_general_quality_json(&serde_json::to_vec(value).unwrap())
    }

    #[test]
    fn exact_quality_strings_and_the_64_kib_boundary_are_shared_without_defaults() {
        let mut bytes = serde_json::to_vec(&document()).unwrap();
        bytes.resize(MAXIMUM_BYTES, b' ');
        let selection = parse_general_quality_json(&bytes).unwrap();
        assert_eq!(selection.quality.id, "public-explicit-quality");
        assert_eq!(
            selection.quality.definition_mm,
            Rational::new(1, 10_000_000).unwrap()
        );
        assert_eq!(
            selection.quality.consistency_mm,
            Rational::new(1, 1_000_000).unwrap()
        );
        assert_eq!(
            selection.quality.derived_uv_mm,
            Rational::new(2, 1_000_000).unwrap()
        );
        assert_eq!(
            selection.quality.query_mm,
            Rational::new(3, 1_000_000).unwrap()
        );
        assert_eq!(
            selection.initial_work_tolerance_mm,
            Rational::new(4, 1_000_000).unwrap()
        );
        assert_eq!(
            selection.quality.maximum_work_tolerance_mm,
            Rational::new(1, 100_000).unwrap()
        );
        assert_eq!(
            selection.quality.maximum_tolerance_growth_mm,
            Rational::ZERO
        );
        bytes.push(b' ');
        assert_eq!(
            parse_general_quality_json(&bytes).unwrap_err().to_string(),
            "Quality JSON exceeds 64 KiB"
        );
    }

    #[test]
    fn quality_shape_field_types_and_decimal_error_order_keep_the_cli_contract() {
        assert_eq!(
            parse(&serde_json::json!([])).unwrap_err().to_string(),
            "Quality JSON must be an object"
        );
        let mut missing = document();
        missing.as_object_mut().unwrap().remove("query_mm");
        assert!(matches!(
            parse(&missing),
            Err(GeneralQualityJsonError::Fields)
        ));
        let mut extra = document();
        extra["units"] = serde_json::json!("mm");
        assert!(matches!(
            parse(&extra),
            Err(GeneralQualityJsonError::Fields)
        ));
        let mut numeric = document();
        numeric["query_mm"] = serde_json::json!(0.000003);
        assert_eq!(
            parse(&numeric).unwrap_err().to_string(),
            "Quality field query_mm must be a string"
        );
        let mut precedence = document();
        precedence["id"] = serde_json::json!(false);
        precedence["initial_work_tolerance_mm"] = serde_json::json!("not-a-decimal");
        assert_eq!(
            parse(&precedence).unwrap_err().to_string(),
            crate::Error::Numeric(ExactError::InvalidDecimal).to_string()
        );
        assert!(matches!(
            parse_general_quality_json(b"{\xff}"),
            Err(GeneralQualityJsonError::Json(_))
        ));
    }

    #[test]
    fn parsing_does_not_mint_constructor_budget_eligibility() {
        let mut value = document();
        value["id"] = serde_json::json!("");
        value["definition_mm"] = serde_json::json!("0");
        value["initial_work_tolerance_mm"] = serde_json::json!("-1");
        let selection = parse(&value).unwrap();
        assert_eq!(selection.quality.definition_mm, Rational::ZERO);
        assert_eq!(
            selection.initial_work_tolerance_mm,
            Rational::new(-1, 1).unwrap()
        );
        assert!(selection.quality.id.is_empty());
    }
}
