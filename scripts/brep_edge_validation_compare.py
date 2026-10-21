#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual Edge/Edgeuse routines and their UV query callees on synthetic graphs.

Reports, owner parameters, stored UV/NURB/gap state, marks and order are observed.
Missing callees remain deferred; this is not complete model/import qualification.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run


# Actual Release SDK access violations observed on these damaged runtime graphs.
# Never replay them or count them as numerical/diagnostic passes. These are exact
# inputs, not a catch-all for unexpected native errors or ordinary mismatches.
UNSAFE_NATIVE_INPUTS = {
    ('edge_interval_outside', 'given', 'none', 'assert_edge_manifold'),
    *((defect, level, 'none', mode)
      for defect, modes in [
          ('surface_offset', ['assert_edge_uv', 'assert_edge_manifold', 'assert_edge_spine']),
          ('eu_no_vertexuse', ['assert_edge_manifold'])]
      for mode, level in itertools.product(modes, ['zero', 'two', 'given', 'repeat']))
}


def validation_cases():
    levels = ['zero', 'one', 'two', 'heal', 'given', 'given_empty', 'no_reports', 'repeat']
    for routine, shape, level in itertools.product(
            ['assert_edge', 'assert_edgeuse', 'assert_edgeuse_mate'],
            ['', '_uv', '_manifold', '_manifold_uv', '_spine', '_spine_uv', '_wire'], levels):
        yield ['none', level, 'none', routine + shape]
    defects = ['curve_no_owner', 'edge_interval_short', 'edge_interval_outside',
               'edge_oversized_tolerance', 'edge_low_tolerance', 'gap_half', 'gap_full',
               'gap_above', 'gap_large', 'surface_offset', 'eu_no_vertexuse', 'eu_vertex_backpointer']
    for defect, shape, level in itertools.product(
            defects, ['', '_uv', '_manifold', '_spine'], ['zero', 'two', 'given', 'repeat']):
        yield [defect, level, 'none', 'assert_edge' + shape]
    for defect, routine, shape, level in itertools.product(
            ['uv_no_owner', 'uv_zero_length', 'uv_bad_dimension', 'uv_interval'],
            ['assert_edgeuse', 'assert_edgeuse_mate'], ['', '_manifold', '_spine'],
            ['zero', 'two', 'given', 'repeat']):
        yield [defect, level, 'none', routine + shape + '_uv']
    for routine, level in itertools.product(['assert_edge', 'assert_edgeuse'], ['zero', 'given', 'repeat']):
        yield ['eu_wrong_orientation', level, 'none', routine]
    for routine, shape in itertools.product(['assert_edge', 'assert_edgeuse'], ['', '_manifold', '_spine']):
        yield ['none', 'zero', 'none', routine + shape + '_walk']


def query_cases():
    for shape, flags, dimension in itertools.product(
            ['line', 'short', 'jump', 'below', 'jump_v', 'dog', 'turn', 'v_bug',
             'turn_before', 'turn_at', 'turn_right', 'turn_top', 'turn_bottom', 'dog_end'], range(4), [2, 3]):
        yield ['periodic', shape, str(flags), str(dimension), '0']
    for shape, surface, mate in itertools.product(
            ['line', 'short', 'jump', 'dog', 'turn', 'v_bug',
             'turn_before', 'turn_at', 'turn_right', 'turn_top', 'turn_bottom', 'dog_end'],
            ['flat', 'u_min', 'u_max', 'v_min', 'v_max'], [0, 1]):
        yield ['dogleg', shape, surface, '2', str(mate)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['cpp', 'rust', 'queries-cpp', 'queries-rust', 'receipt']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for family, cases, cpp, rust in [
            ('validation', validation_cases(), args.cpp, args.rust),
            ('uv_queries', query_cases(), args.queries_cpp, args.queries_rust)]:
        for case in cases:
            row = {'family': family, 'input': case}
            if family == 'validation' and tuple(case) in UNSAFE_NATIVE_INPUTS:
                rows.append(dict(row, status='unsafe_native', reason='observed SDK access violation 0xc0000005'))
                continue
            try:
                native, actual = run(cpp.resolve(), case), run(rust.resolve(), case)
                if 'deferred' in actual:
                    row.update(status='deferred', dependency=actual['deferred'], cpp=native)
                else:
                    difference = compare(native, actual)
                    row.update(status='fail' if difference else 'pass', difference=difference,
                               cpp=native, rust=actual)
            except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
                row.update(status='fail', error=str(error))
            rows.append(row)
            if len(rows) % 64 == 0:
                print(json.dumps({'family': family, 'completed': len(rows)}), flush=True)
    counts = {status: sum(row['status'] == status for row in rows) for status in ['pass', 'fail', 'deferred', 'unsafe_native']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n',
                            encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:8]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
