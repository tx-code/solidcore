// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent synthetic byte/receipt truth; no encoding guessing or native model.

use solidcore_step::{
    inspect, inspect_with_options, report, semantics, CarrierInterpretation, DataStringPolicy,
    HeaderNamePolicy, InspectOptions, Limits, Status, ValueKind,
};

fn file(data: &[u8]) -> Vec<u8> {
    let mut bytes = b"ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('synthetic'),'2;1');\nFILE_NAME('public.step','',(''),(''),'','','');\nFILE_SCHEMA(('TEST'));\nENDSEC;\nDATA;\n".to_vec();
    bytes.extend_from_slice(data);
    bytes.extend_from_slice(b"\nENDSEC;\nEND-ISO-10303-21;\n");
    bytes
}
fn legacy(data: &[u8]) -> Vec<u8> {
    file(data)
        .windows(5)
        .position(|w| w == b"'2;1'")
        .map_or_else(Vec::new, |position| {
            let mut bytes = file(data);
            bytes.splice(position..position + 5, b"'1'".iter().copied());
            bytes
        })
}
fn policies(
    header: bool,
    data: bool,
    carrier_interpretation: CarrierInterpretation,
) -> InspectOptions {
    InspectOptions {
        header_name_policy: if header {
            HeaderNamePolicy::PreserveOpaqueBytes
        } else {
            HeaderNamePolicy::StrictAscii
        },
        data_string_policy: if data {
            DataStringPolicy::PreserveOpaqueBytes
        } else {
            DataStringPolicy::StrictAscii
        },
        carrier_interpretation,
    }
}

#[test]
fn strict_value_json_keeps_the_literal_wire_contract_without_policy_fields() {
    let bytes = file(b"#1=A('abc',-0.,#1);");
    let begin = bytes.windows(5).position(|w| w == b"'abc'").unwrap();
    let original = inspect(bytes, Limits::planar_fixtures());
    let value = &original
        .source
        .as_ref()
        .unwrap()
        .find("1")
        .unwrap()
        .components[0]
        .parameters[0];
    assert_eq!(
        serde_json::to_string(value).unwrap(),
        format!(
            r#"{{"kind":"String","span":{{"begin":{begin},"end":{}}},"text":"abc","children":[]}}"#,
            begin + 5
        )
    );
    for result in [
        original,
        inspect(
            file(b"#1=A('\\X2\\4e2D\\X0\\');"),
            Limits::planar_fixtures(),
        ),
        inspect(legacy(b"#1=A();"), Limits::planar_fixtures()),
    ] {
        let value: serde_json::Value =
            serde_json::from_slice(&report::compile(&result).unwrap()).unwrap();
        assert!(value.get("carrier_interpretation").is_none());
        assert!(value.get("header_name_policy").is_none());
        assert!(value.get("raw_encoding").is_none());
    }
}

#[test]
fn opaque_data_bytes_have_no_text_and_round_trip_without_an_encoding_guess() {
    let bytes = file(b"#1=A('prefix\x81\\X2\\bad','it''s','\\X\\E9',#1);");
    let strict = inspect(bytes.clone(), Limits::planar_fixtures());
    assert_eq!(strict.status, Status::Unsupported);
    let result = inspect_with_options(
        bytes.clone(),
        Limits::planar_fixtures(),
        policies(false, true, CarrierInterpretation::DeclaredOnly),
    );
    assert_eq!(result.status, Status::Inspected);
    let source = result.source.as_ref().unwrap();
    let p = &source.find("1").unwrap().components[0].parameters;
    assert_eq!(p[0].kind, ValueKind::OpaqueString);
    assert_eq!(p[0].interpreted_text(), None);
    assert_eq!(
        source.raw(p[0].span),
        Some(b"'prefix\x81\\X2\\bad'".as_slice())
    );
    assert_eq!(p[1].interpreted_text(), Some("it's"));
    assert_eq!(p[2].interpreted_text(), Some("é"));
    assert_eq!(source.references().len(), 1);
    assert_eq!(result.diagnostics[0].key, "OpaqueDataStringsRetained");
    let json: serde_json::Value =
        serde_json::from_slice(&report::compile(&result).unwrap()).unwrap();
    assert!(json["records"][0]["components"][0]["parameters"][0]["text"].is_null());
    let recovered: Vec<u8> = json["raw"]
        .as_str()
        .unwrap()
        .chars()
        .map(|c| u8::try_from(u32::from(c)).unwrap())
        .collect();
    assert_eq!(recovered, bytes);
    assert_eq!(json["raw_encoding"], "ByteValuesU0000To00FF");
}

#[test]
fn header_exception_is_exactly_the_first_filename_token() {
    let opaque = file(b"#1=A();")
        .windows(11)
        .position(|w| w == b"public.step")
        .unwrap();
    for (header, data, expected) in [
        (false, false, Status::Unsupported),
        (true, false, Status::Inspected),
        (false, true, Status::Unsupported),
        (true, true, Status::Inspected),
    ] {
        let mut bytes = file(b"#1=A();");
        bytes[opaque] = 0xff;
        let result = inspect_with_options(
            bytes,
            Limits::planar_fixtures(),
            policies(header, data, CarrierInterpretation::DeclaredOnly),
        );
        assert_eq!(result.status, expected);
        if expected == Status::Inspected {
            assert_eq!(result.diagnostics[0].key, "OpaqueHeaderFileName");
            assert_eq!(result.diagnostics[0].parameter_path, [1]);
            assert_eq!(
                result.source.as_ref().unwrap().header()[1].parameters[0].kind,
                ValueKind::OpaqueString
            );
        }
    }
    for (marker, offset) in [(b"'',('')".as_slice(), 1), (b"(''),('')".as_slice(), 2)] {
        let mut bytes = file(b"#1=A();");
        let position = bytes
            .windows(marker.len())
            .position(|w| w == marker)
            .unwrap()
            + offset;
        bytes.insert(position, 0xff);
        let result = inspect_with_options(
            bytes,
            Limits::planar_fixtures(),
            policies(true, true, CarrierInterpretation::DeclaredOnly),
        );
        assert_eq!(result.status, Status::Unsupported);
        assert_eq!(result.diagnostics[0].key, "UnsupportedEncoding");
        assert!(result.source.is_none());
    }
}

#[test]
fn high_bytes_in_comments_and_controls_cannot_escape_token_policy() {
    for data in [
        b"/*\xff*/#1=A('x');".as_slice(),
        b"#1=A();/*\x81*/",
        b"#1=A('\x7f');",
        b"#1=A('\x01');",
        b"#1=\xff();",
    ] {
        let bytes = file(data);
        let result = inspect_with_options(
            bytes.clone(),
            Limits::planar_fixtures(),
            policies(true, true, CarrierInterpretation::DeclaredOnly),
        );
        assert_eq!(result.status, Status::Unsupported);
        assert!(result.source.is_none());
        assert_eq!(result.captured(), Some(bytes.as_slice()));
    }
}

#[test]
fn ascii_escapes_keep_strict_failure_and_budget_precedence() {
    for data in [
        b"#1=A('\\Q\\ascii');".as_slice(),
        b"#1=A('\\X2\\D800\\X0\\');",
        b"#1=A('\\PB\\name');",
    ] {
        for string_bytes in [2, 7, 30, 32769] {
            let mut limits = Limits::planar_fixtures();
            limits.string_bytes = string_bytes;
            let strict = inspect(file(data), limits);
            let opaque = inspect_with_options(
                file(data),
                limits,
                policies(true, true, CarrierInterpretation::DeclaredOnly),
            );
            assert_eq!(strict.status, opaque.status);
            assert_eq!(strict.statistics, opaque.statistics);
            assert_eq!(strict.diagnostics, opaque.diagnostics);
        }
    }
}

#[test]
fn level1_keeps_original_declaration_and_receipt_after_outcome_notices_are_cleared() {
    let bytes = legacy(b"#1=A('source');");
    let strict = inspect(bytes.clone(), Limits::planar_fixtures());
    assert_eq!(strict.status, Status::Unsupported);
    assert_eq!(strict.diagnostics[0].key, "UnsupportedImplementationLevel");
    let mut result = inspect_with_options(
        bytes.clone(),
        Limits::planar_fixtures(),
        policies(
            false,
            false,
            CarrierInterpretation::Level1AsCorrectedEncoding,
        ),
    );
    assert_eq!(result.status, Status::Inspected);
    assert_eq!(
        result.source.as_ref().unwrap().header()[0].parameters[1].text,
        "1"
    );
    assert_eq!(
        result.carrier_assumption().unwrap().original_diagnostics,
        strict.diagnostics
    );
    result.diagnostics.clear();
    let source = result.source.as_ref().unwrap().clone();
    let json: serde_json::Value =
        serde_json::from_slice(&report::compile(&result).unwrap()).unwrap();
    assert_eq!(json["carrier_interpretation"]["applied"], true);
    assert_eq!(
        json["carrier_interpretation"]["declaration_verified"],
        false
    );
    drop(result);
    assert_eq!(source.bytes(), bytes);
    assert_eq!(source.carrier_assumption().unwrap().declared_level, "1");
}

#[test]
fn lowercase_policy_requires_applied_level1_and_only_affects_data_payloads() {
    let data = b"#1=A('\\X\\e9','\\X2\\4e2d\\X0\\','\\X4\\0001f642\\X0\\');";
    let policy = policies(
        false,
        false,
        CarrierInterpretation::Level1AsCorrectedEncodingWithLowercaseHex,
    );
    let corrected = inspect_with_options(file(data), Limits::planar_fixtures(), policy);
    assert_eq!(corrected.status, Status::Rejected);
    assert!(corrected.carrier_assumption().is_none());
    let mut result = inspect_with_options(legacy(data), Limits::planar_fixtures(), policy);
    assert_eq!(result.status, Status::Inspected);
    let source = result.source.as_ref().unwrap();
    let p = &source.find("1").unwrap().components[0].parameters;
    assert_eq!(
        p.iter().map(|v| v.text.as_str()).collect::<Vec<_>>(),
        ["é", "中", "🙂"]
    );
    let receipt = result.carrier_assumption().unwrap();
    assert_eq!(receipt.string_hex_interpretations.len(), 3);
    assert_eq!(
        receipt
            .string_hex_interpretations
            .iter()
            .map(|r| r.lowercase_digits)
            .collect::<Vec<_>>(),
        [1, 2, 1]
    );
    for (value, evidence) in p.iter().zip(&receipt.string_hex_interpretations) {
        assert!(evidence.complete);
        assert_eq!(evidence.token_span, value.span);
        assert!(evidence.first_strict_span.begin > value.span.begin);
        assert!(evidence.first_strict_span.end < value.span.end);
    }
    result.diagnostics.clear();
    let typed = semantics::check(
        &result,
        semantics::Limits::fixtures(),
        &semantics::Options::default(),
    );
    assert_eq!(typed.status, semantics::Status::InvalidRequest);
    assert!(typed.snapshot.is_none());
}

#[test]
fn unused_option_is_explicit_and_cannot_enter_strict_semantics() {
    let result = inspect_with_options(
        include_bytes!("fixtures/plane.step").to_vec(),
        Limits::planar_fixtures(),
        policies(
            false,
            false,
            CarrierInterpretation::Level1AsCorrectedEncoding,
        ),
    );
    assert_eq!(result.status, Status::Inspected);
    assert!(result.carrier_assumption().is_none());
    let json: serde_json::Value =
        serde_json::from_slice(&report::compile(&result).unwrap()).unwrap();
    assert_eq!(json["carrier_interpretation"]["status"], "NotApplied");
    let typed = semantics::check(
        &result,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_part_and_closure: true,
            ..Default::default()
        },
    );
    assert_eq!(typed.status, semantics::Status::InvalidRequest);
    assert!(typed.snapshot.is_none());
    assert_eq!(typed.part, "NotRun");
}

#[test]
fn partial_hex_evidence_survives_errors_without_publishing_a_graph() {
    for data in [
        b"#1=A('\\X2\\4e2Z\\X0\\');".as_slice(),
        b"#1=A('\\X2\\d800\\X0\\');",
        b"#1=A('\\X\\e9unfinished",
    ] {
        let bytes = legacy(data);
        let result = inspect_with_options(
            bytes.clone(),
            Limits::planar_fixtures(),
            policies(
                false,
                false,
                CarrierInterpretation::Level1AsCorrectedEncodingWithLowercaseHex,
            ),
        );
        assert_eq!(result.status, Status::Rejected);
        assert!(result.source.is_none());
        assert_eq!(result.captured(), Some(bytes.as_slice()));
        let receipt = result.carrier_assumption().unwrap();
        let evidence = &receipt.string_hex_interpretations[0];
        assert!(!evidence.complete);
        assert!(evidence.lowercase_digits > 0);
        assert!(evidence.token_span.end <= bytes.len());
        assert!(evidence.first_strict_span.end <= evidence.token_span.end);
        assert_eq!(result.diagnostics.len(), 1);
    }
}

#[test]
fn completed_receipt_survives_unresolved_reference_failure() {
    let result = inspect_with_options(
        legacy(b"#1=A('\\X\\e9',#9);"),
        Limits::planar_fixtures(),
        policies(
            false,
            false,
            CarrierInterpretation::Level1AsCorrectedEncodingWithLowercaseHex,
        ),
    );
    assert_eq!(result.status, Status::Rejected);
    assert_eq!(result.diagnostics[0].key, "UnresolvedRequiredReference");
    assert!(
        result
            .carrier_assumption()
            .unwrap()
            .string_hex_interpretations[0]
            .complete
    );
    assert!(result.source.is_none());
}

#[test]
fn opaque_strings_keep_report_and_token_budget_boundaries() {
    let data = b"#1=A('\xff\x80\x81\xfe\xfd\xfc\xfb\xfa\xf9\xf8\xf7\xf6\xf5\xf4\xf3\xf2');";
    let options = policies(false, true, CarrierInterpretation::DeclaredOnly);
    let good = inspect_with_options(file(data), Limits::planar_fixtures(), options);
    assert_eq!(good.status, Status::Inspected);
    let maximum = good.statistics.max_string_bytes;
    for boundary in [maximum, maximum - 1] {
        let mut limits = Limits::planar_fixtures();
        limits.string_bytes = boundary;
        let result = inspect_with_options(file(data), limits, options);
        assert_eq!(
            result.status,
            if boundary == maximum {
                Status::Inspected
            } else {
                Status::ResourceLimit
            }
        );
        if boundary != maximum {
            assert!(result.source.is_none());
        }
    }
    let mut limits = Limits::planar_fixtures();
    limits.report_bytes = 1000;
    let result = inspect_with_options(file(data), limits, options);
    assert_eq!(
        report::compile(&result),
        Err(report::Error::ReportByteLimit)
    );
    let mut limits = Limits::planar_fixtures();
    // Stabilize the limit's own decimal width before asserting the exact N/N-1
    // output boundary. The final LF belongs to that same byte budget.
    let mut length = report::compile(&good).unwrap().len();
    for _ in 0..4 {
        limits.report_bytes = length;
        length = report::compile(&inspect_with_options(
            file(data),
            Limits::planar_fixtures(),
            options,
        ))
        .unwrap()
        .len()
            - Limits::planar_fixtures().report_bytes.to_string().len()
            + length.to_string().len();
    }
    limits.report_bytes = length;
    let exact = inspect_with_options(file(data), limits, options);
    assert_eq!(report::compile(&exact).unwrap().len(), length);
    limits.report_bytes -= 1;
    assert_eq!(
        report::compile(&inspect_with_options(file(data), limits, options)),
        Err(report::Error::ReportByteLimit)
    );
}

#[test]
fn opaque_source_values_are_not_accepted_as_typed_identity_strings() {
    let mut bytes = include_bytes!("fixtures/plane.step").to_vec();
    let text = String::from_utf8(bytes.clone()).unwrap();
    // Use the actual PRODUCT id span in this public fixture, not a hard-coded
    // source entity number. The header's filename exception is independent.
    let prefix = "PRODUCT('";
    let position = text.find(prefix).unwrap() + prefix.len();
    bytes[position] = 0xff;
    let source = inspect_with_options(
        bytes,
        Limits::planar_fixtures(),
        policies(false, true, CarrierInterpretation::DeclaredOnly),
    );
    assert_eq!(source.status, Status::Inspected);
    let result = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_part_and_closure: true,
            ..Default::default()
        },
    );
    assert_eq!(result.status, semantics::Status::Unsupported);
    assert_eq!(
        result.diagnostics[0].source.key,
        "OpaqueDataTextUnestablished"
    );
    assert_eq!(result.diagnostics[0].actual, "OpaqueString");
    assert_eq!(result.types, "Unsupported");
    assert_eq!(result.part, "NotRun");
    assert!(result.snapshot.is_none());
}
