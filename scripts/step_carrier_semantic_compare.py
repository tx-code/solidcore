#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Exercise actual strict C++/Rust semantic boundaries for opaque source text.

Only public plane/box fixtures are read. No interoperability exceptions, private
models, BRep construction or network. Compare both original full C++ checks and
the original-method checkpoint adapter; neither is an expected-answer export.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
from step_part_closure_compare import direct_source
from step_type_unit_compare import compare

AP203 = b"AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}"
PROFILES = [(AP203, []), (AP203, ['--general-brep-source']),
            (b'AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}', ['--general-brep-source']),
            (b'AUTOMOTIVE_DESIGN {1 0 10303 214 1 1 1 1}', ['--general-brep-source']),
            (b'CONFIG_CONTROL_DESIGN', ['--general-brep-source'])]


def variants(source):
    yield source
    # All replacements preserve structural grammar. Each high byte is placed in
    # a specific original source token; no guessed name/identity is supplied.
    for marker in [b"FILE_NAME('", b"PRODUCT('", b"PRODUCT_DEFINITION_FORMATION('",
                   b"PRODUCT_DEFINITION('"]:
        position = source.index(marker) + len(marker)
        yield source[:position] + b'\xff\x81' + source[position:]
    yield source.replace(b'HEADER;', b'HEADER;/*\xff*/')
    yield source.replace(b'DATA;', b'DATA;/*\xff*/')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--cpp-full', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    count = 0
    with tempfile.TemporaryDirectory(prefix='solidcore-opaque-semantic-public-') as directory:
        path = Path(directory) / 'public.step'
        for fixture in ['plane.step', 'material-box.step', 'surface-box.step']:
            original = (root / 'crates/solidcore-step/tests/fixtures' / fixture).read_bytes()
            for schema, scope in PROFILES:
                for number, source in enumerate(variants(original.replace(AP203, schema))):
                    path.write_bytes(source)
                    for policies in [[], ['--preserve-opaque-header-name'], ['--preserve-opaque-data-strings'],
                                     ['--preserve-opaque-header-name', '--preserve-opaque-data-strings']]:
                        flags = [*scope, *policies, '--source-semantics']
                        try:
                            compare(args.cpp.resolve(), args.rust.resolve(), path, flags)
                            direct_source(args.cpp_full.resolve(), args.rust.resolve(), path,
                                          [*scope, *policies], flags)
                        except AssertionError as error:
                            raise AssertionError(f'public fixture {fixture}, variant {number}, {flags}: {error}') from error
                        count += 1
                print(json.dumps({'opaque_semantic_requests': count, 'differences': 0}), flush=True)
    print(json.dumps({'checkpoint_comparisons': count, 'direct_original_cpp_comparisons': count,
                      'profiles': len(PROFILES), 'differences': 0,
                      'scope': 'strict typed/part closure boundary; interoperability and native build NotRun'}))


if __name__ == '__main__':
    main()
