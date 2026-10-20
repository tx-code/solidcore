#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare real C++/Rust constructor graphs on the same synthetic input matrix.

All identity/order/orientation/ownership fields are exact. Numeric values use the
repository's 1e-9 relative rule, without an absolute floor. No native acceptance,
UV approximation bound or whole-kernel qualification follows from this observer.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess


def compare(a, b, path='$'):
    if isinstance(a, bool) or isinstance(b, bool):
        if type(a) is not type(b) or a != b:
            return f'{path}: {a!r} != {b!r}'
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if not (math.isfinite(a) and math.isfinite(b)):
            return f'{path}: nonfinite value'
        if isinstance(a, int) and isinstance(b, int):
            if a != b:
                return f'{path}: {a} != {b}'
        elif abs(a - b) > 1e-9 * max(abs(a), abs(b)):
            return f'{path}: {a} != {b}'
    elif isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys():
            return f'{path}: different keys'
        for key in a:
            if difference := compare(a[key], b[key], f'{path}.{key}'):
                return difference
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return f'{path}: lengths {len(a)} != {len(b)}'
        for i, (x, y) in enumerate(zip(a, b)):
            if difference := compare(x, y, f'{path}[{i}]'):
                return difference
    elif type(a) is not type(b) or a != b:
        return f'{path}: {a!r} != {b!r}'
    return None


def cases():
    for shape, walk, outside, vertices, uv, point_limits in itertools.product(
            ['plane', 'hole', 'two', 'solid', 'surface'], [1, -1], [1, -1], [1, 0], [0, 1], [0, 1]):
        yield [shape, walk, outside, vertices, uv, point_limits, 1, 0]
    for shape, scale, offset in itertools.product(['plane', 'hole', 'two', 'solid', 'surface'],
                                                 [1e-3, 1e3], [0, 1e4]):
        yield [shape, 1, 1, 1, 1, 1, scale, offset]
    for outside, scale in itertools.product([1, -1], [1, 1e-3]):
        yield ['vertex', 1, outside, 0, 0, 0, scale, 0]
    for walk, vertices, uv in itertools.product([1, -1], [0, 1], [0, 1]):
        yield ['radial', walk, 1, vertices, uv, 0, 1, 0]
    for walk, outside, vertices, point_limits in itertools.product([1, -1], [1, -1], [0, 1], [0, 1]):
        yield ['lift', walk, outside, vertices, 1, point_limits, 1, 0]


def run(executable, args):
    p = subprocess.run([str(executable), *map(str, args)], capture_output=True, text=True,
                       encoding='utf-8', timeout=30, check=False)
    if p.returncode:
        raise RuntimeError(f'{executable.name}: exit {p.returncode}: {p.stderr.strip()}')
    return json.loads(p.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True, help='Ignored local output')
    args = parser.parse_args()
    rows = []
    for case in cases():
        try:
            cpp = run(args.cpp.resolve(), case)
            rust = run(args.rust.resolve(), case)
            if 'deferred' in rust:
                row = {'input': case, 'status': 'deferred', 'dependency': rust['deferred']}
            else:
                difference = compare(cpp, rust)
                row = {'input': case, 'status': 'fail' if difference else 'pass'}
                if difference:
                    row.update(difference=difference, cpp=cpp, rust=rust)
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            row = {'input': case, 'status': 'fail', 'error': str(error)}
        rows.append(row)
    counts = {status: sum(row['status'] == status for row in rows) for status in ['pass', 'fail', 'deferred']}
    receipt = {'scope': 'synthetic real constructor graph/geometry comparison',
               'native_acceptance': 'not established', 'numeric_rule': '1e-9 relative, no absolute floor',
               'counts': counts, 'cases': rows}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [r for r in rows if r['status'] == 'fail'][:5]:
        print(json.dumps({k: v for k, v in row.items() if k not in {'cpp', 'rust'}}))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
