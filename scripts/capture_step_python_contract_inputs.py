#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Exercise explicit public C++ contract tests and retain only synthetic CLI inputs.

The original tests and subprocess results are unchanged. The recording hook
observes only calls to the explicitly selected semantic CLI and only requests
using strict/default carrier options, optionally general source scope. Inputs,
not expected answers, are retained locally; no model library is scanned.
"""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import unittest

MODULES = [
    'test_ap214_property', 'test_ap214_source_roles', 'test_ap214_metadata',
    'test_ap214_curve_style', 'test_ap214_predefined_colour', 'test_point_style',
    'test_supporting_geometry', 'test_wireframe', 'test_value_item',
    'test_ap203_general', 'test_ap203_n916', 'test_composite_curve',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    binary = args.binary.resolve(strict=True)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.source.resolve()))
    os.environ['STEP_SEMANTIC_BINARY'] = str(binary)
    original_run = subprocess.run

    def record(command, *positional, **keywords):
        if isinstance(command, (list, tuple)) and command and Path(command[0]).resolve() == binary:
            paths = [Path(v) for v in command[1:] if str(v).lower().endswith(('.step', '.stp'))]
            if len(paths) == 1 and paths[0].is_file():
                flags = [str(v) for v in command[1:] if str(v) != str(paths[0])]
                if '--fixture-limits' in flags and set(flags) <= {'--fixture-limits', '--general-brep-source'}:
                    data = paths[0].read_bytes()
                    scope = 'general' if '--general-brep-source' in flags else 'planar'
                    name = hashlib.sha256(data).hexdigest() + '-' + scope
                    (output / (name + '.step')).write_bytes(data)
                    (output / (name + '.options')).write_text(scope + '\n1000000\n', encoding='utf-8')
        return original_run(command, *positional, **keywords)

    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(
        'tools.step_semantic_validation.tests.' + module) for module in MODULES)
    subprocess.run = record
    try:
        result = unittest.TextTestRunner(verbosity=1).run(suite)
    finally:
        subprocess.run = original_run
    print(f'Public synthetic strict requests retained: {len(list(output.glob("*.step")))}')
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == '__main__':
    main()
