#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Run the fixed public C++ planar contract modules and capture only CLI inputs.

Original tests/reader results are unchanged. No expected answers, private models
or identifying inventories are exported. Captures must remain in local output.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    binary = args.binary.resolve(strict=True)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.source.resolve()))
    os.environ['STEP_PLANAR_BINARY'] = str(binary)
    original = subprocess.run

    def record(command, *positional, **keywords):
        if isinstance(command, (list, tuple)) and command and Path(command[0]).resolve() == binary:
            paths = [Path(v) for v in command[1:] if str(v).lower().endswith(('.step', '.stp'))]
            if len(paths) == 1 and paths[0].is_file():
                flags = [str(v) for v in command[1:] if str(v) != str(paths[0])]
                data = paths[0].read_bytes()
                name = hashlib.sha256(data + b'\0' + json.dumps(flags).encode()).hexdigest()
                (args.output / (name + '.step')).write_bytes(data)
                (args.output / (name + '.request.json')).write_text(json.dumps({'cpp_flags': flags}, indent=2) + '\n', encoding='utf-8')
        return original(command, *positional, **keywords)

    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName('tools.step_planar_validation.tests.' + name)
                               for name in ['test_planar', 'test_ap214'])
    subprocess.run = record
    try:
        result = unittest.TextTestRunner(verbosity=1).run(suite)
    finally:
        subprocess.run = original
    print(json.dumps({'original_tests': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
                      'public_requests': len(list(args.output.glob('*.request.json')))}))
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == '__main__':
    main()
