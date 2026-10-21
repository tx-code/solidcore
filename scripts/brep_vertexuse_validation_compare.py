#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual local VU checks; does not establish complete Vertex/Edge validity."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run

# SDK ComputeUVSector dereferences the missing edge in these raw faults.
# Preserve the excluded requests without replaying the native access violation.
UNSAFE_NATIVE_INPUTS = {(defect, level, 'none', 'assert_vertexuse') for defect in ['eu_no_edge','vu_no_owner'] for level in
                        ['zero', 'one', 'two', 'heal', 'given', 'given_empty', 'no_reports', 'repeat']}


def cases():
    levels = ['zero', 'one', 'two', 'heal', 'given', 'given_empty', 'no_reports', 'repeat']
    for defect, level in itertools.product(
            ['none', 'vu_no_vertex', 'face_no_surface'], levels):
        yield [defect, level, 'none', 'assert_vertexuse']
    for mode, level in itertools.product(
            ['assert_vertexuse_walk', 'assert_vertexuse_vertexloop', 'assert_vertexuse_shellvertex',
             'assert_vertexuse_wire', 'assert_vertexuse_wire_chain', 'assert_vertexuse_wire_branch'], levels):
        yield ['none', level, 'none', mode]
    yield from sorted(UNSAFE_NATIVE_INPUTS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        if tuple(case) in UNSAFE_NATIVE_INPUTS:
            rows.append({'input': case, 'status': 'unsafe_native', 'reason': 'missing edge in native sector query; excluded'})
            continue
        try:
            cpp, rust = run(args.cpp.resolve(), case), run(args.rust.resolve(), case)
            if 'deferred' in rust:
                rows.append({'input': case, 'status': 'deferred', 'dependency': rust['deferred'], 'cpp': cpp})
                continue
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass',
                         'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {s: sum(row['status'] == s for row in rows) for s in ['pass', 'fail', 'deferred', 'unsafe_native']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
