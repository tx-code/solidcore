#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare strict C++/Rust source readers on explicitly selected local files.

No network, CAD kernel or source mutation. Reports are kept in memory and only
aggregate counts are printed. Synthetic generators are independent of either
reader. This checks source compatibility, never geometric import correctness.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def span(value):
    return [value["begin"], value["end"]] if isinstance(value, dict) else value


def normalize(value):
    """Retain every source identity, value, span and graph edge in the comparison."""
    if isinstance(value, list):
        return [normalize(item) for item in value]
    if not isinstance(value, dict):
        return value
    result = {}
    for name, item in value.items():
        if value.get("kind") == "OpaqueString" and name in {"interpretation", "notice"}:
            assert value.get("interpretation") == "Unestablished"
            assert value.get("text") is None
            continue  # C++ prose is not interpreted text or a stable discriminator.
        if name in {"raw", "location", "record_occurrence", "message"}:
            continue  # Raw coverage is checked independently below, not discarded.
        name = "span" if name == "primary" else name
        if name in {"span", "id_span", "token_span", "first_strict_span"}:
            result[name] = span(item)
        elif name == "related":
            result[name] = [span(r.get("span", r)) for r in item]
        else:
            result[name] = normalize(item)
    return result


def verify_raw(value, original):
    if isinstance(value, list):
        for child in value:
            verify_raw(child, original)
    elif isinstance(value, dict):
        if "span" in value and "raw" in value:
            begin, end = span(value["span"])
            assert original[begin:end].decode("latin1") == value["raw"]
        for child in value.values():
            verify_raw(child, original)


def invoke(binary, path, options=()):
    result = subprocess.run([str(binary), "--fixture-limits", *options, str(path)],
                            capture_output=True, timeout=30, check=False)
    assert result.returncode in {0, 1, 2, 3}, "reader failed outside source contract"
    return result.returncode, json.loads(result.stdout)


def compare(cpp, rust, path, cpp_options=(), rust_options=()):
    original = path.read_bytes()
    c_code, c = invoke(cpp, path, cpp_options)
    r_code, r = invoke(rust, path, rust_options)
    assert c_code == r_code, "exit status differs"
    assert c["status"] == r["status"], "source status differs"
    assert c["input"]["sha256"] == r["sha256"], "capture identity differs"
    if r["sha256"] is not None:
        assert r["sha256"] == hashlib.sha256(original).hexdigest()
    cs = c["statistics"]
    mapped = dict(records=cs["data_records"], values=cs["values_including_header"],
                  references=cs["references"], nesting=cs["value_depth"],
                  max_string_bytes=cs["max_string_bytes"], max_record_bytes=cs["max_record_bytes"])
    assert mapped == r["statistics"], "resource accounting differs"
    assert normalize(c["diagnostics"]) == normalize(r["diagnostics"]), "diagnostic identity/position differs"
    for field in ["header_name_policy", "data_string_policy"]:
        assert c.get(field) == r.get(field), "explicit source policy differs"
    cc, rc = c.get("carrier_interpretation"), r.get("carrier_interpretation")
    assert (cc is None) == (rc is None), "carrier qualification missing"
    if cc is not None:
        cc = dict(cc)
        evidence = []
        for item in cc.get("string_hex_interpretations", []):
            item = dict(item)
            assert item.pop("strict_key") == "InvalidStringEncoding"
            evidence.append(item)
        if evidence:
            cc["string_hex_interpretations"] = evidence
        assert normalize(cc) == normalize(rc), "original carrier/token evidence differs"
    for field in ["header", "records", "references"]:
        assert normalize(c[field]) == normalize(r[field]), f"{field} graph differs"
    if r["status"] == "Inspected":
        assert c["input"]["raw"].encode("latin1") == original
        assert r["raw"].encode("latin1") == original
        verify_raw(c["header"], original)
        verify_raw(c["records"], original)
        assert all(r["scope"][s] == "NotRun" for s in ["schema_semantics", "units", "geometry", "native_build"])
    else:
        assert r["records"] is None and r["references"] is None


def carrier(data):
    return ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('synthetic'),'2;1');\n"
            "FILE_NAME('public.step','',(''),(''),'','','');\nFILE_SCHEMA(('TEST'));\n"
            f"ENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n").encode("ascii")


def generated():
    for data in [
        "#1=A();", "#0001=A();#1=B();", "#9=A((T(#99999)),#9);",
        "#1=(A(-0.,+1,1.E-300,$,*,.TRUE.,()) B(#2));#2=C(#1);",
        "#" + "9" * 100 + "=A();", "#1=A('it''s','a\\\\b');", "#1=A(.A_B.);",
        "#1=A('\\X2\\4E2D6587\\X0\\','\\X4\\0001F642\\X0\\');",
        "#1=A('\\PA\\\\S\\i','\\X\\E9');",
        "#1=A('\\X2\\D800\\X0\\');", "#1=A('\\X2\\D83DDE42\\X0\\');",
        "#1=A('\\X4\\00110000\\X0\\');", "#1=A('\\X2\\4e2D\\X0\\');",
        "#1=A('\\X2\\\\X0\\');", "#1=A('\\PB\\x');", "#1=A('\\N\\');",
        "#1=A('it'\r\n\t's');", "#1=A('\\X\n2\\4E\t2D\\X\r\n0\\');",
        "#1=A(T());", "#1=A(T(1,2));", "#0=A();", "#=A();", "#1=A(+);",
        "#1=A(1.E+);", "#1=A(.1.);", "#1=A(.ABC);", "#1=A(1E3);",
        "#1=A(!CUSTOM());", "#1=A(@1);", '#1=A("00");', "#1=A();/*unfinished",
        "#1=A(" + "(" * 70 + "0" + ")" * 70 + ");",
    ]:
        yield carrier(data)
    base = carrier("#1=A('x',#1);")
    for old, new in [(b"'2;1'", b"'1'"), (b"DATA;", b"DATA('x');"),
                     (b"END-ISO-10303-21;", b"DATA;ENDSEC;END-ISO-10303-21;"),
                     (b"END-ISO-10303-21;", b"END-ISO-10303-21;TAIL"),
                     (b"FILE_SCHEMA(('TEST'));", b"FILE_SCHEMA('TEST');")]:
        yield base.replace(old, new)
    for byte in [0, 27, 127, 128, 255]:
        yield base.replace(b"'x'", b"'" + bytes([byte]) + b"'")
    for content in [b"PK00", b"<xml>", b"\x1fgzip", b"", b"abc"]:
        yield content
    # Every strict prefix through the last required delimiter; trailing layout is optional.
    for length in range(len(base)):
        yield base[:length]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpp", type=Path, required=True)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--corpus", type=Path, help="Optional local ticket-01 synthetic cases directory")
    args = parser.parse_args()
    count = 0
    if args.corpus:
        cases = sorted(args.corpus.glob("S203-*/input.step"))
        assert len(cases) == 49, "expected the frozen 49-case synthetic source contract"
        for case in cases:
            compare(args.cpp.resolve(), args.rust.resolve(), case.resolve())
            count += 1
    with tempfile.TemporaryDirectory(prefix="solidcore-step-public-") as temp:
        path = Path(temp) / "synthetic.step"
        for index, content in enumerate(generated()):
            path.write_bytes(content)
            try:
                compare(args.cpp.resolve(), args.rust.resolve(), path)
            except AssertionError as error:
                raise AssertionError(f"synthetic case {index}: {error}") from error
            count += 1
        path.write_bytes(carrier("#001=A('abc',((#2)),1.);#2=B();"))
        _, c = invoke(args.cpp.resolve(), path)
        stats = c["statistics"]
        for c_flag, r_flag, boundary in [
            ("input-bytes", "input-bytes", path.stat().st_size),
            ("records", "records", stats["data_records"]),
            ("values", "values", stats["values_including_header"]),
            ("references", "references", stats["references"]),
            ("depth", "nesting", stats["value_depth"]),
            ("string-bytes", "string-bytes", stats["max_string_bytes"]),
            ("record-bytes", "record-bytes", stats["max_record_bytes"]),
            ("id-digits", "id-digits", 3),
        ]:
            for value in [boundary, boundary - 1]:
                compare(args.cpp.resolve(), args.rust.resolve(), path,
                        ["--max-" + c_flag, str(value)], ["--" + r_flag, str(value)])
                count += 1
    print(json.dumps({"compared": count, "differences": 0, "scope": "strict source only"}))


if __name__ == "__main__":
    main()
