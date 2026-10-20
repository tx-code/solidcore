#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Actual SDK faceuse/loopuse/loop and sampled-side comparison on synthetic graphs.

These are selected native operations, not imported models or continuous geometric
acceptance. Unsafe native array/pointer damage is excluded from oracle requests.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run

# Actual maintained Windows Release SDK exits 0xC0000005 on these invalid-domain
# classification requests. Preserve their identity without replaying undefined
# native behavior as a numerical oracle. See plan/upstream-issues.md.
UNSAFE_NATIVE_INPUTS = {
    ('face_domain_line', 'given_face_vertices', 'none', 'assert_face'),
    ('face_domain_line', 'given_face_all', 'none', 'assert_face'),
}


def cases():
    levels = ['zero', 'one', 'two', 'heal', 'given', 'given_empty', 'no_reports', 'repeat']
    for mode, defects in [
            ('assert_faceuse', ['none', 'fu_no_face', 'fu_no_mate', 'fu_no_shell', 'fu_empty_loops']),
            ('assert_loopuse', ['none', 'lu_no_loop', 'lu_no_mate', 'lu_no_faceuse', 'loop_no_primary']),
            ('assert_loop', ['none', 'mate_rotate', 'mate_reversed_order', 'face_no_surface', 'fu_no_face']),
            ('assert_loop_vertexloop', ['none'])]:
        for defect, level in itertools.product(defects, levels):
            yield [defect, level, 'none', mode]
    for mode, level in itertools.product(['assert_faceuse_mate', 'assert_loopuse_mate',
                                          'assert_faceuse_walk', 'assert_loopuse_walk'], levels):
        yield ['none', level, 'none', mode]
    for mode, selection in itertools.product(
            ['edge_side', 'edge_side_mate', 'edge_side_last', 'edge_side_last_mate',
             'edge_side_uv', 'edge_side_mate_uv', 'edge_side_last_uv', 'edge_side_last_mate_uv'],
            ['samples_0', 'samples_1', 'samples_2', 'samples_6', 'samples_17', 'periodic_u', 'periodic_v']):
        yield ['none', selection, 'none', mode]
    for defect, selection in itertools.product(
            ['none', 'face_domain_outside', 'face_domain_line', 'surface_no_owner', 'coincident_vertices'],
            ['given', 'given_empty', 'given_face_owner', 'given_face_vertices', 'given_face_all']):
        yield [defect, selection, 'none', 'assert_face']
    for selection in ['zero', 'one', 'two', 'heal', 'no_reports', 'repeat']:
        yield ['none', selection, 'none', 'assert_face']
    for mode, selection in itertools.product(
            ['assert_face_vertexloop', 'assert_face_hole', 'assert_face_hole_outside', 'assert_face_hole_cross',
             'assert_face_hole_shared', 'assert_face_hole_nested'],
            ['given', 'given_face_owner', 'given_face_vertices', 'given_face_all']):
        yield ['none', selection, 'none', mode]
    for mode in ['loop_side', 'loop_side_mate', 'loop_side_uv', 'loop_side_mate_uv', 'loop_side_vertexloop']:
        for selection in ['zero', 'periodic_u', 'periodic_v']:
            yield ['none', selection, 'none', mode]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        if tuple(case) in UNSAFE_NATIVE_INPUTS:
            rows.append({'input': case, 'status': 'unsafe_native', 'reason': 'known SDK access violation; not a pass/deferred'})
            continue
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
    counts = {status: sum(row['status'] == status for row in rows)
              for status in ['pass', 'fail', 'deferred', 'unsafe_native']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': 'selected native local/use/sampled-side checks, not complete validity',
                                      'counts': counts, 'cases': rows}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
