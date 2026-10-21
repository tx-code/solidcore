#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Synthetic complete-curve projections: actual SDK versus Rust, no tolerance floor."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run


def cases():
    for shape, path in [('plane', 'diagonal'), ('plane', 'quadratic'), ('plane', 'bulge'),
                        ('flat', 'iso'), ('flat', 'diagonal'),
                        ('flat', 'quadratic'), ('flat', 'outside'), ('flat', 'cross'),
                        ('saddle', 'curved'), ('cylinder', 'seam'), ('cylinder', 'radial'),
                        ('cone', 'pole')]:
        for mode, keep in itertools.product(['normal', 'trim', 'subset', 'outside', 'fallback'], [0, 1]):
            yield [shape, path, mode, keep, 1., 0.]
    for (shape, path), scale, offset, keep in itertools.product(
            [('flat', 'diagonal'), ('flat', 'quadratic'), ('saddle', 'curved')],
            [1e-3, 1e3], [0., 100.], [0, 1]):
        yield [shape, path, 'normal', keep, scale, offset]
    for shape, path in [('flat', 'iso'), ('flat', 'cross'), ('flat', 'quadratic'),
                        ('plane', 'iso'), ('cylinder', 'seam')]:
        yield [shape, path, 'trace', 1, 1., 0.]


def analytic_truth(case, result):
    shape, path, mode, _, _, _ = case
    assert result['flags'][0] == result['flags'][1], ('temporary flag leaked', case)
    if mode == 'outside' and shape != 'plane':
        assert result['status'] == 1001 and result['failure'] == [1020, 1, 18]
        assert not result['curves']
    if result['status'] != 1000 or not result['curves']:
        return
    for curve in result['curves']:
        if mode not in ['trim', 'trace']:
            assert curve['knots'][0] == 2. and curve['knots'][-1] == 5., ('lost source parameters', case)
    if shape == 'cylinder' and path == 'seam' and mode in ['normal', 'fallback']:
        assert len(result['curves']) == 2
        assert [c['samples'][0][0] for c in result['curves']] == [0., 1.]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['cpp', 'rust', 'receipt']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        row = {'input': case}
        try:
            native, actual = run(args.cpp.resolve(), case), run(args.rust.resolve(), case)
            analytic_truth(case, native)
            if 'deferred' in actual:
                row.update(status='deferred', dependency=actual['deferred'], cpp=native)
            else:
                analytic_truth(case, actual)
                difference = compare(native, actual)
                row.update(status='fail' if difference else 'pass', difference=difference, cpp=native, rust=actual)
        except (AssertionError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            row.update(status='fail', error=str(error))
        rows.append(row)
        if len(rows) % 32 == 0:
            print(json.dumps({'completed': len(rows)}), flush=True)
    counts = {s: sum(r['status'] == s for r in rows) for s in ['pass', 'fail', 'deferred']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    raise SystemExit(bool(counts['fail']))


if __name__ == '__main__':
    main()
