#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual Region/Shell assertions, ordered reports and persistent mark effects.

Synthetic runtime graphs only. Curveless wire topology qualifies these local
structural checks, never native geometry or import acceptance. Invalid-pointer
cases whose native graph is unsafe are not replayed as numerical truth.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run


def cases():
    levels = ['zero', 'one', 'two', 'heal', 'given', 'given_empty', 'no_reports', 'repeat']
    for mode, defects in [
            ('assert_region', ['none', 'region_no_owner', 'shell_no_owner', 'region_mark3_ahead',
                               'shell_mark3_ahead', 'empty_shell']),
            ('assert_shell', ['none', 'empty_shell', 'fu_no_loops', 'fu_no_shell',
                              'region_mark3_ahead', 'shell_mark3_ahead'])]:
        for defect, level in itertools.product(defects, levels):
            yield [defect, level, 'none', mode]
    for shape, defect, level in itertools.product(['vertex', 'wire', 'wire_chain', 'wire_branch'],
                                                 ['none', 'empty_shell'], levels):
        yield [defect, level, 'none', 'assert_shell_' + shape]
    for shape, defect, level in itertools.product(['vertex', 'wire', 'wire_branch'],
                                                 ['none', 'shell_mark3_ahead'], levels):
        yield [defect, level, 'none', 'assert_region_' + shape]


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
    args.receipt.write_text(json.dumps({'scope': 'selected native Region/Shell no-walk and mark contracts, not full validity',
                                      'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
