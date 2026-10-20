#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Exercise explicit public C++ contract tests and retain only synthetic CLI inputs.

The original tests and subprocess results are unchanged. The recording hook
observes calls to the explicitly selected semantic CLI. Strict/default capture is
unchanged; opt-in policy capture retains exact public requests as local JSON.
Inputs, not expected answers, are retained locally; no model library is scanned.
"""
import argparse
import hashlib
import json
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
GEOMETRY_MODULES = ['test_ap214_geometry', 'test_rational', 'test_trimmed',
                    'test_surface_role', 'test_ap214_angles', 'test_ap214_entities']
POLICY_MODULES = ['test_application_contexts', 'test_ap214', 'test_ap214_units', 'test_ap214_measure',
                  'test_ap214_mbx', 'test_ap214_foreign_real', 'test_carrier_level1', 'test_interoperability',
                  'test_interpretation', 'test_name_only_interpretation', 'test_opaque_data', 'test_opaque_identity',
                  'test_semantics']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--source-binary', type=Path, help='Actual source inspector for mixed carrier/semantic contract modules')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--module', choices=MODULES+GEOMETRY_MODULES+POLICY_MODULES, action='append',
                        help='Run only explicitly selected public contract modules')
    parser.add_argument('--all-modules', action='store_true', help='Run the fixed public source contract module list')
    parser.add_argument('--all-source-policies', action='store_true', help='Retain exact source CLI requests as local JSON, including interoperability/interpretation')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    binary = args.binary.resolve(strict=True)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.source.resolve()))
    os.environ['STEP_SEMANTIC_BINARY'] = str(binary)
    if args.source_binary:
        os.environ['STEP_INSPECT_BINARY'] = str(args.source_binary.resolve(strict=True))
    original_run = subprocess.run

    def record(command, *positional, **keywords):
        if isinstance(command, (list, tuple)) and command and Path(command[0]).resolve() == binary:
            paths = [Path(v) for v in command[1:] if str(v).lower().endswith(('.step', '.stp'))]
            if len(paths) == 1 and paths[0].is_file():
                flags = [str(v) for v in command[1:] if str(v) != str(paths[0])]
                if args.all_source_policies and '--fixture-limits' in flags:
                    data = paths[0].read_bytes()
                    request = {'cpp_flags': flags}
                    name = hashlib.sha256(data + b'\0' + json.dumps(request, sort_keys=True).encode()).hexdigest()
                    (output / (name + '.step')).write_bytes(data)
                    (output / (name + '.request.json')).write_text(json.dumps(request, indent=2) + '\n', encoding='utf-8')
                if '--fixture-limits' in flags and set(flags) <= {'--fixture-limits', '--general-brep-source'}:
                    data = paths[0].read_bytes()
                    scope = 'general' if '--general-brep-source' in flags else 'planar'
                    name = hashlib.sha256(data).hexdigest() + '-' + scope
                    (output / (name + '.step')).write_bytes(data)
                    (output / (name + '.options')).write_text(scope + '\n1000000\n', encoding='utf-8')
        return original_run(command, *positional, **keywords)

    modules = list(dict.fromkeys(MODULES+GEOMETRY_MODULES+POLICY_MODULES)) if args.all_modules else (args.module or MODULES)
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(
        'tools.step_semantic_validation.tests.' + module) for module in modules)
    subprocess.run = record
    try:
        result = unittest.TextTestRunner(verbosity=1).run(suite)
    finally:
        subprocess.run = original_run
    print(json.dumps({'public_synthetic_inputs': len(list(output.glob('*.step'))),
                      'exact_policy_requests': len(list(output.glob('*.request.json'))),
                      'original_tests_run': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors)}))
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == '__main__':
    main()
