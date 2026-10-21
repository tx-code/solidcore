#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Polymorphic iso caches on synthetic surfaces; these are not STEP model counts."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run


def truth(case, result):
    shape, _, mode, *_ = case
    calls = result['calls']
    assert len(calls) == 4
    assert [c['cached'] for c in calls] == ([True, True, False, True] if mode == 'normal' else [False] * 4)
    assert all(c['status'] == 1000 and len(c['curves']) == 4 for c in calls)
    if shape == 'plane':
        assert all(curve['line'] for c in calls for curve in c['curves'])
    if shape in ['flat', 'saddle', 'cylinder', 'cone']:
        assert not any(curve['line'] for c in calls for curve in c['curves'])
    if shape in ['revolution', 'revolution_swap']:
        # Native extracts a BSD in the generator direction, and returns analytic
        # circles in the sweep direction. A straight generator does not make the
        # extracted boundary an SmLine. Exact native type IDs are compared below.
        assert not any(curve['line'] for c in calls for curve in c['curves'])
        assert all(sorted([sum(c['type'] == t for c in call['curves'])
                           for t in {c['type'] for c in call['curves']}]) == [2, 2]
                   for call in calls)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['cpp', 'rust', 'receipt']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for shape, mode, (scale, offset) in itertools.product(
            ['plane', 'flat', 'saddle', 'cylinder', 'cone', 'revolution', 'revolution_swap'],
            ['normal', 'subset'], [(1., 0.), (1e-3, 0.), (1e3, 0.), (1., 100.)]):
        case = [shape, 'iso', mode, 1, scale, offset, 'cache']
        row = {'input': case}
        try:
            native, actual = run(args.cpp.resolve(), case), run(args.rust.resolve(), case)
            truth(case, native)
            truth(case, actual)
            difference = compare(native, actual)
            row.update(status='fail' if difference else 'pass', difference=difference, cpp=native, rust=actual)
        except (AssertionError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            row.update(status='fail', error=str(error))
        rows.append(row)
    counts = {s: sum(r['status'] == s for r in rows) for s in ['pass', 'fail']}
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in rows:
        if row['status'] == 'fail':
            print(json.dumps(row['input']), row.get('difference', row.get('error')))
    raise SystemExit(bool(counts['fail']))


if __name__ == '__main__':
    main()
