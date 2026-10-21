#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare maintained SDK Curve/BSpline/Line assertions and full selected report rows.

Public synthetic inputs; operation equivalence is not import/model qualification.
Includes real fallback GPA findings and Given radius-selection behavior.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run

SHAPES = ['line', 'line_bad', 'linear', 'quad', 'cubic', 'rational', 'kink', 'point',
          'unclamped', 'mult', 'start_repeat', 'end_repeat', 'pole', 'slow', 'tag2_z', 'missing_nurb']


def cases():
    for shape, slot, level, dim in itertools.product(
            SHAPES, ['edge', 'uv'],
            ['zero', 'one', 'two', 'heal', 'given', 'given_radius', 'given_empty', 'no_reports', 'repeat', 'bad_owner'], [2, 3]):
        yield [shape, slot, level, dim, 1, 0]
    for shape, dim, transform in itertools.product(
            ['line_bad', 'quad', 'rational', 'kink', 'slow', 'pole'], [2, 3],
            [(1e-3, 0), (1e3, 0), (1, 10)]):
        yield [shape, 'edge', 'zero', dim, *transform]
    # The domain-loss regression also compares the actual consolidated candidates.
    for dim, slot in itertools.product([2,3],['edge','uv']):
        yield ['slow',slot,'debug_gpa',dim,1e-3,0]


def reference_contract(case, actual):
    """Independent fixture identity/state checks before cross-implementation comparison."""
    shape, _, level, *_ = case
    if level.startswith('debug_'):
        return
    expected_type = 14010 if shape in ['line', 'line_bad'] else 14033
    if actual['curve_type'] != expected_type:
        raise ValueError(f'wrong reference fixture type for {shape}')
    # This caught the original ambiguous "linear"/"line" factory prefix.
    before = shape not in ['line', 'missing_nurb']
    after = shape != 'missing_nurb' and not (shape == 'line' and level.startswith('given'))
    if actual['before_nurb'] != before or actual['after_nurb'] != after:
        raise ValueError(f'wrong reference materialization transition for {case}')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp', type=Path, required=True)
    p.add_argument('--rust', type=Path, required=True)
    p.add_argument('--receipt', type=Path, required=True)
    args = p.parse_args()
    rows = []
    for case in cases():
        try:
            cpp, rust = run(args.cpp.resolve(), case), run(args.rust.resolve(), case)
            reference_contract(case, cpp)
            if 'deferred' in rust:
                rows.append({'input': case, 'status': 'deferred', 'dependency': rust['deferred'], 'cpp': cpp})
                continue
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass', 'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {s: sum(row['status'] == s for row in rows) for s in ['pass', 'fail', 'deferred']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [r for r in rows if r['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
