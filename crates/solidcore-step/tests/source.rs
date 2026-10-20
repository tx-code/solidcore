// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Independent synthetic source contracts; no customer data or CAD reader.

use solidcore_step::{inspect, report, Limits, Location, Span, Status, ValueKind};

fn file(data: &str) -> Vec<u8> {
    format!("ISO-10303-21;\r\nHEADER;\r\nFILE_DESCRIPTION(('synthetic'),'2;1');\r\nFILE_NAME('public.step','',(''),(''),'','','');\r\nFILE_SCHEMA(('TEST_SCHEMA'));\r\nENDSEC;\r\nDATA;\r\n{data}\r\nENDSEC;\r\nEND-ISO-10303-21;\r\n").into_bytes()
}
fn read(data: &str) -> solidcore_step::Outcome {
    inspect(file(data), Limits::planar_fixtures())
}
fn key(bytes: impl AsRef<[u8]>, expected: &str, status: Status) {
    let bytes = bytes.as_ref();
    let result = inspect(bytes.to_vec(), Limits::planar_fixtures());
    assert_eq!(result.status, status);
    assert_eq!(
        result.diagnostics.first().map(|d| d.key.as_str()),
        Some(expected)
    );
    assert!(
        result.source.is_none(),
        "failure must not publish a partial graph"
    );
    assert_eq!(result.captured(), Some(bytes));
}

#[test]
fn owns_bytes_preserves_values_forward_cycles_complex_and_sparse_ids() {
    let huge = "9".repeat(100);
    let data = format!("#0001=(A('it''s',-0.,+12,2.5E-300,() ,$,*,.TRUE.,REAL(1.),#2) B());\n#2=OTHER((#{huge},#1));\n#{huge}=LAST(#2);");
    let bytes = file(&data);
    let original = bytes.clone();
    let result = inspect(bytes, Limits::planar_fixtures());
    assert_eq!(result.status, Status::Inspected);
    let source = result.source.unwrap();
    assert_eq!(source.bytes(), original);
    let first = source.find("1").unwrap();
    assert!(first.complex);
    assert_eq!(source.raw(first.id_span), Some(b"#0001".as_slice()));
    let values = &first.components[0].parameters;
    assert_eq!(values[0].text, "it's");
    assert_eq!(source.raw(values[0].span), Some(b"'it''s'".as_slice()));
    assert_eq!(values[1].text, "-0.");
    assert_eq!(values[2].text, "+12");
    assert_eq!(values[3].text, "2.5E-300");
    assert_eq!(values[4].kind, ValueKind::Aggregate);
    assert!(values[4].children.is_empty());
    assert_eq!(values[5].kind, ValueKind::Unset);
    assert_eq!(values[6].kind, ValueKind::Derived);
    assert_eq!(values[7].text, "TRUE");
    assert_eq!(values[8].kind, ValueKind::Typed);
    assert_eq!(source.references().len(), 4);
    assert_eq!(source.references()[0].parameter_path, [10]);
    assert_eq!(source.references()[1].parameter_path, [1, 1]);
    assert_eq!(source.references()[1].target_record_index, 2);
    assert_eq!(source.incoming("2"), [0, 3]);
    assert!(source.find(&huge).is_some());
    assert!(source.find("0001").is_none());
    assert_eq!(source.locate(0), Some(Location { line: 1, column: 1 }));
    assert_eq!(source.locate(15), Some(Location { line: 2, column: 1 }));
    assert!(source.locate(source.bytes().len()).is_some());
    assert!(source.locate(source.bytes().len() + 1).is_none());
    assert!(source.raw(Span { begin: 4, end: 2 }).is_none());
}

#[test]
fn canonical_duplicate_and_unresolved_diagnostics_keep_locations_and_paths() {
    let duplicate = read("#1=A();\n#0001=B();");
    assert_eq!(duplicate.status, Status::Rejected);
    let d = &duplicate.diagnostics[0];
    assert_eq!(
        (&*d.key, &*d.stage, &*d.record_id, &*d.target_id),
        ("DuplicateInstanceId", "D0", "1", "1")
    );
    let bytes = duplicate.captured().unwrap();
    assert_eq!(&bytes[d.primary.begin..d.primary.end], b"#0001");
    assert_eq!(&bytes[d.related[0].begin..d.related[0].end], b"#1");
    let unresolved = read("#9=SHAPE_DEFINITION_REPRESENTATION((T(#99999)),#9);");
    let d = &unresolved.diagnostics[0];
    assert_eq!(
        (
            &*d.key,
            &*d.stage,
            &*d.record_id,
            &*d.component,
            &*d.target_id
        ),
        (
            "UnresolvedRequiredReference",
            "D1",
            "9",
            "SHAPE_DEFINITION_REPRESENTATION",
            "99999"
        )
    );
    assert_eq!(d.parameter_path, [1, 1, 1]);
    assert!(unresolved.source.is_none());
}

#[test]
fn strings_decode_unicode_and_skip_layout_inside_directives_and_quotes() {
    for (token, expected) in [
        (r"'\X\E9'", "é"),
        (r"'\X2\4E2D6587\X0\'", "中文"),
        (r"'\X4\0001F642\X0\'", "🙂"),
        (r"'\PA\\S\i'", "é"),
        (r"'a\\b'", "a\\b"),
        ("'a\r\n\tb'", "ab"),
        ("'it'\r\n\t's'", "it's"),
        ("'\\X\r\n2\\4E\t2D\\X\n0\\'", "中"),
    ] {
        let result = read(&format!("#1=A({token});"));
        assert_eq!(
            result.status,
            Status::Inspected,
            "{token}: {:?}",
            result.diagnostics
        );
        assert_eq!(
            result.source.unwrap().records()[0].components[0].parameters[0].text,
            expected
        );
    }
}

#[test]
fn malformed_strings_and_unsupported_alphabets_are_distinct() {
    for token in [
        r"'\X2\D800\X0\'",
        r"'\X2\D83DDE42\X0\'",
        r"'\X4\00110000\X0\'",
        r"'\X2\\X0\'",
        r"'\X2\4e2D\X0\'",
        r"'\X\G0'",
        r"'\Z\'",
        r"'\X2\123\X0\'",
        r"'\PJ\'",
    ] {
        key(
            file(&format!("#1=A({token});")),
            if token.contains("D800") || token.contains("D83D") || token.contains("00110000") {
                "InvalidStringCodePoint"
            } else {
                "InvalidStringEncoding"
            },
            Status::Rejected,
        );
    }
    for token in [r"'\PB\x'", r"'\N\'"] {
        key(
            file(&format!("#1=A({token});")),
            "UnsupportedStringEncoding",
            Status::Unsupported,
        );
    }
    key(
        file("#1=A('unfinished);"),
        "UnterminatedString",
        Status::Rejected,
    );
}

#[test]
fn malformed_tokens_headers_and_carriers_fail_without_graph() {
    for (data, expected) in [
        ("#0=A();", "InvalidInstanceId"),
        ("#=A();", "InvalidInstanceId"),
        ("#1=A(+);", "InvalidNumber"),
        ("#1=A(1.E+);", "InvalidNumber"),
        ("#1=A(.1.);", "InvalidEnumeration"),
        ("#1=A(.ABC);", "InvalidEnumeration"),
        ("#1=A(T(1,2));", "SyntaxError"),
        ("#1=A(T());", "SyntaxError"),
        ("#1=A(.5);", "InvalidEnumeration"),
        ("#1=A(1E3);", "SyntaxError"),
        ("#1=A();/*unfinished", "UnterminatedComment"),
    ] {
        key(file(data), expected, Status::Rejected);
    }
    for token in ["!CUSTOM()", "@1", "\"00\""] {
        key(
            file(&format!("#1=A({token});")),
            "UnsupportedToken",
            Status::Unsupported,
        );
    }
    for data in [b"PK00".as_slice(), b"<xml>", b"\x1fgzip"] {
        key(data, "UnsupportedCarrier", Status::Unsupported);
    }
    let original = String::from_utf8(file("#1=A();")).unwrap();
    for (from, to, expected, status) in [
        (
            "'2;1'",
            "'1'",
            "UnsupportedImplementationLevel",
            Status::Unsupported,
        ),
        (
            "DATA;",
            "DATA('x');",
            "UnsupportedDataSection",
            Status::Unsupported,
        ),
        (
            "END-ISO-10303-21;",
            "DATA;ENDSEC;END-ISO-10303-21;",
            "UnsupportedSection",
            Status::Unsupported,
        ),
        (
            "END-ISO-10303-21;",
            "END-ISO-10303-21;TRAIL",
            "TrailingContent",
            Status::Rejected,
        ),
        (
            "FILE_SCHEMA(('TEST_SCHEMA'));",
            "FILE_SCHEMA('TEST_SCHEMA');",
            "InvalidHeader",
            Status::Rejected,
        ),
    ] {
        key(original.replace(from, to).into_bytes(), expected, status);
    }
    for byte in [0_u8, 0x1b, 0x7f, 0x80, 0xff] {
        let mut bytes = file("#1=A('x');");
        let pos = bytes.iter().position(|c| *c == b'x').unwrap();
        bytes[pos] = byte;
        key(bytes, "UnsupportedEncoding", Status::Unsupported);
    }
}

#[test]
fn exact_resource_boundaries_and_invalid_safety_limits() {
    let bytes = file("#001=A('abc',((#2)),1.);\n#2=B();");
    let result = inspect(bytes.clone(), Limits::planar_fixtures());
    assert_eq!(result.status, Status::Inspected);
    let stats = result.statistics;
    for (field, boundary, key) in [
        (0, bytes.len(), "InputByteLimit"),
        (1, stats.records, "RecordLimit"),
        (2, stats.values, "ValueLimit"),
        (3, stats.references, "ReferenceLimit"),
        (4, stats.nesting, "NestingLimit"),
        (5, stats.max_string_bytes, "StringByteLimit"),
        (6, stats.max_record_bytes, "RecordByteLimit"),
        (7, 3, "IdDigitLimit"),
    ] {
        let set = |limits: &mut Limits, n| match field {
            0 => limits.input_bytes = n,
            1 => limits.records = n,
            2 => limits.values = n,
            3 => limits.references = n,
            4 => limits.nesting = n,
            5 => limits.string_bytes = n,
            6 => limits.record_bytes = n,
            _ => limits.id_digits = n,
        };
        let mut limits = Limits::planar_fixtures();
        set(&mut limits, boundary);
        assert_eq!(
            inspect(bytes.clone(), limits).status,
            Status::Inspected,
            "{key} at boundary"
        );
        set(&mut limits, boundary - 1);
        let failure = inspect(bytes.clone(), limits);
        assert_eq!(
            failure.status,
            Status::ResourceLimit,
            "{key} below boundary"
        );
        assert_eq!(failure.diagnostics[0].key, key);
        assert!(failure.source.is_none());
        assert_eq!(failure.captured().is_some(), field != 0);
    }
    let mut limits = Limits::planar_fixtures();
    limits.nesting = 129;
    let failure = inspect(bytes, limits);
    assert_eq!(failure.diagnostics[0].key, "InvalidLimits");
    assert!(failure.captured().is_none());
}

#[test]
fn report_roundtrips_source_without_claiming_units_geometry_or_native_acceptance() {
    let mut result = read("#1=A('\\X2\\4E2D\\X0\\',-0.);");
    let first = report::compile(&result).unwrap();
    assert_eq!(first, report::compile(&result).unwrap());
    let json: serde_json::Value = serde_json::from_slice(&first).unwrap();
    assert_eq!(json["schema"], report::SCHEMA);
    for scope in ["schema_semantics", "units", "geometry", "native_build"] {
        assert_eq!(json["scope"][scope], "NotRun");
    }
    assert_eq!(
        json["raw"].as_str().unwrap().as_bytes(),
        result.captured().unwrap()
    );
    assert_eq!(
        json["records"][0]["components"][0]["parameters"][0]["text"],
        "中"
    );
    // Budget is itself part of the report. Establish an ample temporary budget
    // with the same digit width, so changing its digits preserves total size.
    result.limits.report_bytes = 9999;
    let size = report::compile(&result).unwrap().len();
    result.limits.report_bytes = size;
    let exact = report::compile(&result).unwrap();
    assert_eq!(exact.len(), size);
    result.limits.report_bytes -= 1;
    assert_eq!(
        report::compile(&result),
        Err(report::Error::ReportByteLimit)
    );
}

#[test]
fn hash_vectors_and_no_schema_specific_semantic_shortcuts() {
    let empty = inspect(Vec::new(), Limits::planar_fixtures());
    assert_eq!(
        empty.sha256.as_deref(),
        Some("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855")
    );
    assert!(empty.source.is_none());
    assert_eq!(
        read("#1=UNKNOWN_BUT_SYNTACTICALLY_VALID(-1.,.WHATEVER.);").status,
        Status::Inspected
    );
    assert_eq!(
        read("#1=SI_UNIT(.MILLI.,.METRE.);").status,
        Status::Inspected
    );
}

#[test]
fn bounded_nesting_truncation_and_deterministic_byte_mutations_do_not_panic() {
    key(
        file(&format!("#1=A({}0{});", "(".repeat(200), ")".repeat(200))),
        "NestingLimit",
        Status::ResourceLimit,
    );
    let valid = file("#1=(A('x',#2) B());\n#2=C((1.,-0.,REAL(2.)));/*ok*/");
    let marker_end = valid.len() - 2; // Final CRLF is optional; the final semicolon is required.
    for length in 0..marker_end {
        assert_ne!(
            inspect(valid[..length].to_vec(), Limits::planar_fixtures()).status,
            Status::Inspected
        );
    }
    for pos in 0..valid.len() {
        for byte in [0_u8, b'\'', b'(', b'#', b'\\', 0xff] {
            let mut bytes = valid.clone();
            bytes[pos] = byte;
            let result = inspect(bytes, Limits::planar_fixtures());
            assert_eq!(result.source.is_some(), result.status == Status::Inspected);
            assert!(report::compile(&result).is_ok());
        }
    }
}

#[test]
fn public_planar_and_box_samples_remain_distinct_source_roles() {
    for (bytes, expected_role, records) in [
        (
            include_bytes!("fixtures/plane.step").as_slice(),
            "SHELL_BASED_SURFACE_MODEL",
            51,
        ),
        (
            include_bytes!("fixtures/material-box.step").as_slice(),
            "MANIFOLD_SOLID_BREP",
            145,
        ),
        (
            include_bytes!("fixtures/surface-box.step").as_slice(),
            "SHELL_BASED_SURFACE_MODEL",
            145,
        ),
    ] {
        let result = inspect(bytes.to_vec(), Limits::planar_fixtures());
        assert_eq!(result.status, Status::Inspected);
        let source = result.source.unwrap();
        assert_eq!(source.find("21").unwrap().components[0].name, expected_role);
        // Counts are independent source occurrence counts, not native face counts.
        assert_eq!(source.records().len(), records);
    }
}
