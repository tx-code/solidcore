#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare concrete CrvOnSurf operations and SDK-written archive reads.

Observes numerical dispatch, concrete kind, status and geometry after transform.
Does not qualify external borrowed aliases, const-copy cache side effects or
notifications. All inputs are public synthetic runtime geometry.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run


def cases():
    for shape, owner, action, t in itertools.product(
            ['plane', 'rotated_plane', 'bilinear'], range(4),
            ['none', 'copy', 'transform', 'copy_transform', 'identity', 'nonuniform'], [0, .456, 1]):
        yield [shape, owner, action, t, 1, 0]
    for shape, action, dimensions in itertools.product(
            ['plane', 'rotated_plane', 'bilinear'], ['none', 'copy_transform', 'binary', 'ascii'],
            [(1e-3, 0), (1e3, 0), (1, 10)]):
        yield [shape, 2, action, .456, *dimensions]


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
            archive = cpp.pop('archive_hex', None)
            rust = run(args.rust.resolve(), case + ([archive] if archive else []))
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass',
                         'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {s: sum(row['status'] == s for row in rows) for s in ['pass', 'fail']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
