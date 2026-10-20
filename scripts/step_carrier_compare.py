#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual C++/Rust source policies on public synthetic bytes only.

No model search or network. The original readers execute independently; compare
raw ownership, source graph, diagnostics, actual costs and interpretation receipts.
Legacy alternatives never qualify source declarations or native construction.
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import subprocess
import tempfile

from step_source_compare import carrier, compare, invoke


HEADER = "--preserve-opaque-header-name"
DATA = "--preserve-opaque-data-strings"
CORRECTED = "--interpret-level1-as-corrected"
LOWER = "--interpret-level1-with-lowercase-hex"


def cases():
    """Inputs have independent byte intent; expected answers are not captured."""
    records = [
        b"#1=A();", b"#1=A('ordinary','it''s');", b"#1=A('\x81\\legacy',#1);",
        b"#1=A('\xff');#2=B('\x80');", b"#1=A(('\xff',T('\x81')));",
        b"#1=A('\\X\\e9');", b"#1=A('\\X2\\4e2d\\X0\\');",
        b"#1=A('\\X4\\0001f642\\X0\\');", b"#1=A('\\X2\\d800\\X0\\');",
        b"#1=A('\\X4\\0011000a\\X0\\');", b"#1=A('\\X2\\4e2Z\\X0\\');",
        b"#1=A('\\X2\\4e2d\\X0\\',#9);", b"#1=A('\\X\\e9');#1=B();",
        b"#1=A('\\X\\e9\\X2\\4e2d\\X0\\');",
        b"#1=A('\\Q\\ascii');", b"#1=A('\\Q\\before\xff');",
        b"#1=A('\\X\n2\\4e\t2d\\X\r\n0\\');",
        b"#1=A('\xff'\r\n\t's');", b"#1=A('\\X\\E9');",
        b"#1=A('\\PB\\ascii');", b"#1=A('\\N\\');",
        b"#1=A('\\X\\e9\\Q\\');", b"#1=A('\\X\\e9unterminated",
        b"#1=A('\\Q\\" + b"x" * 96 + b"\xff');",
        b"/*\xff*/#1=A();", b"#1=A();/*\x81*/", b"#1=\xff();",
        b"#1=A('\x7f');", b"#1=A('\x01');",
    ]
    for data in records:
        source = carrier("").replace(b"DATA;\n\n", b"DATA;\n" + data + b"\n")
        yield source
        yield source.replace(b"'2;1'", b"'1'")
    base = carrier("#1=A('abc');")
    for before, after in [
        (b"public.step", b"\xff\x81\\legacy"),
        (b"public.step", b"\\X2\\4e2d\\X0\\"),
        (b"'synthetic'", b"'\xff'"), (b"(''),('')", b"('\xff'),('')"),
        (b"FILE_SCHEMA(('TEST'))", b"FILE_SCHEMA(('\xff'))"),
        (b"FILE_NAME('public.step',''", b"FILE_NAME('public.step','\xff'"),
        (b"HEADER;", b"HEADER;/*\xff*/"), (b"'2;1'", b"'2;2'"),
        (b"'2;1'", b"'3;1'"), (b"'2;1'", b"'1'"),
    ]:
        yield base.replace(before, after)
    yield base.replace(b"public.step", b"\xff").replace(b"'abc'", b"'\x81'").replace(b"'2;1'", b"'1'")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpp", type=Path, required=True)
    parser.add_argument("--rust", type=Path, required=True)
    args = parser.parse_args()
    cpp, rust = args.cpp.resolve(), args.rust.resolve()
    policies = [tuple(([HEADER] if header else []) + ([DATA] if data else []) + list(interpretation))
                for header, data, interpretation in itertools.product([False, True], [False, True], [(), (CORRECTED,), (LOWER,)])]
    count = budgets = truncations = 0
    with tempfile.TemporaryDirectory(prefix="solidcore-carrier-public-") as directory:
        path = Path(directory) / "public.step"
        for number, source in enumerate(cases()):
            path.write_bytes(source)
            for flags in policies:
                try:
                    compare(cpp, rust, path, flags, flags)
                except AssertionError as error:
                    raise AssertionError(f"synthetic carrier {number}, flags {flags}: {error}") from error
                count += 1
            if (number + 1) % 20 == 0:
                print(json.dumps({"policy_comparisons": count, "differences": 0}), flush=True)
        # Budgets cannot be relaxed by opaque preservation or by an interpretation
        # receipt. Include successful-token sizes and failure precedence before it.
        for source, flags in [
            (carrier("#1=A('" + "\\X\\e9" * 8 + "',#1);").replace(b"'2;1'", b"'1'"), (LOWER,)),
            (carrier("#1=A('abc');").replace(b"'abc'", b"'" + b"\xff" * 40 + b"'"), (DATA,)),
            (carrier("#1=A('\\Q\\" + "a" * 90 + "');").replace(b"a'", b"a\xff'"), (DATA,)),
        ]:
            path.write_bytes(source)
            for name, rust_name, values in [
                ("string-bytes", "string-bytes", [0, 7, 13, 30, 41, 42, 49, 50, 32769]),
                ("values", "values", [0, 1, 10, 20, 25, 100]),
                ("record-bytes", "record-bytes", [0, 20, 40, 60, 1000]),
                ("input-bytes", "input-bytes", [len(source), len(source) - 1]),
            ]:
                for limit in values:
                    compare(cpp, rust, path, (*flags, "--max-" + name, str(limit)), (*flags, "--" + rust_name, str(limit)))
                    budgets += 1
        source = carrier("#1=A('\\X2\\4e2d\\X0\\','second');").replace(b"'2;1'", b"'1'")
        start = source.index(b"#1=A")
        for end in range(start, len(source)):
            path.write_bytes(source[:end])
            compare(cpp, rust, path, (LOWER,), (LOWER,))
            truncations += 1
        # Each writer's exact UTF-8 byte boundary is measured independently,
        # including its own limit field and final LF; schemas intentionally differ.
        path.write_bytes(source)
        for binary, prefix in [(cpp, "--max-report-bytes"), (rust, "--report-bytes")]:
            command = [str(binary), "--fixture-limits", LOWER, prefix]
            high = 16 * 1024 * 1024
            output = subprocess.run([*command, str(high), str(path)], capture_output=True, timeout=30, check=False)
            assert output.returncode == 0
            length = len(output.stdout)
            for _ in range(4):
                length = len(output.stdout) - len(str(high)) + len(str(length))
            assert len(str(length)) == len(str(length - 1))
            exact = subprocess.run([*command, str(length), str(path)], capture_output=True, timeout=30, check=False)
            assert exact.returncode == 0 and len(exact.stdout) == length and exact.stdout.endswith(b"\n")
            short = subprocess.run([*command, str(length - 1), str(path)], capture_output=True, timeout=30, check=False)
            assert short.returncode == 5 and not short.stdout, "partial JSON published past report budget"
    print(json.dumps({"policy_comparisons": count, "budget_comparisons": budgets,
                      "truncation_comparisons": truncations, "differences": 0,
                      "independent_report_boundaries": 4,
                      "scope": "source policies/bytes/receipts only; no semantic approval or native build"}))


if __name__ == "__main__":
    main()
