#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare native grid/control-net/surface assertions, not model import acceptance."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run


SHAPES = ['plane', 'bilinear', 'quad', 'quad_rational', 'fold_u', 'fold_v', 'point', 'curve',
          'row_zero', 'late_row_zero', 'repeat', 'pole_u', 'pole_v', 'false_pole_u', 'false_pole_v',
          'mult_u', 'mult_v']


def cases():
    for name, flags, action in itertools.product(
            ['normal', 'row_fold', 'col_fold', 'point', 'repeat', 'short_row'], range(4), ['none', 'clear']):
        yield ['grid', name, flags, action]
    for name in SHAPES:
        yield ['query', name, 'zero', 'none']
    for name, level in itertools.product(SHAPES, ['zero', 'one', 'two', 'heal', 'given', 'no_reports', 'repeat']):
        yield ['assert', name, level, 'none']
    for name, transform, family in itertools.product(
            ['plane', 'quad_rational', 'row_zero', 'late_row_zero', 'repeat', 'false_pole_u', 'false_pole_v'],
            [(1e-3, 0), (1e3, 0), (1, 10)], ['query', 'assert']):
        yield [family, name, 'zero', 'none', *transform]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        try:
            cpp = run(args.cpp.resolve(), case)
            rust = run(args.rust.resolve(), case)
            if 'deferred' in rust:
                rows.append({'input': case, 'status': 'deferred', 'dependency': rust['deferred'], 'cpp': cpp})
                continue
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass',
                         'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {status: sum(row['status'] == status for row in rows) for status in ['pass', 'fail', 'deferred']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': 'selected native point-grid/control-net/dynamic-surface checks; no model acceptance',
                                      'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
