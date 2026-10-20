#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare selected actual AssertValid root calls; missing walk/geometry is never a pass."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run
from brep_validation_compare import DEFECTS


def cases():
    for defect, level in itertools.product(DEFECTS, ['zero', 'one', 'given', 'given_empty', 'no_reports', 'repeat']):
        yield [defect, level, 'none', 'assert_root']
    for defect, level in itertools.product(['none', 'root_zero_tolerance', 'gap_half', 'gap_large'],
                                         ['zero', 'one', 'given', 'given_empty', 'no_reports', 'repeat']):
        yield [defect, level, 'none', 'assert_root']
    for defect, level, mode in itertools.product(['none', 'face_no_surface'], ['two', 'given15'],
                                                ['assert_root', 'assert_root_walk']):
        yield [defect, level, 'none', mode]

    for level in ['zero', 'given']:
        yield ['none', level, 'none', 'assert_root_walk']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp', type=Path, required=True)
    p.add_argument('--rust', type=Path, required=True)
    p.add_argument('--receipt', type=Path, required=True)
    args = p.parse_args()
    rows = []
    for case in cases():
        try:
            cpp = run(args.cpp.resolve(), case)
            rust = run(args.rust.resolve(), case)
            if 'deferred' in rust:
                rows.append({'input': case, 'status': 'deferred', 'dependency': rust['deferred'], 'cpp': cpp})
                continue
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass', 'difference': difference,
                         'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {s: sum(row['status'] == s for row in rows) for s in ['pass', 'fail', 'deferred']}
    receipt = {'scope': 'selected native root level/request/no-walk contracts, not full descendant or import validity',
               'counts': counts, 'cases': rows}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [r for r in rows if r['status'] == 'fail'][:5]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
