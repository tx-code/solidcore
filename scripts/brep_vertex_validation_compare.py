#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare real Vertex assertions, incidence and corner queries, including cache effects.

Only public synthetic constructor graphs are used. Analytic plane/corner truth
supplements SDK parity. Missing dependencies are deferred, never promoted to pass.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run


# Exact discovery inputs caused SDK access violations; do not replay them or
# generalize this list to other failures. A native crash is not numerical truth.
UNSAFE_NATIVE_INPUTS = {
    ('surface_offset', 'two', 'none', mode) for mode in [
        'vertex_corner', 'vertex_corner_manifold', 'vertex_corner_trihedral',
        'vertex_parallel_trihedral', 'assert_vertex_trihedral']
}


def cases():
    shapes = ['', '_uv', '_manifold', '_manifold_uv', '_spine', '_spine_uv',
              '_trihedral', '_trihedral_uv', '_vertexloop', '_shellvertex',
              '_wire', '_wire_chain', '_wire_branch']
    for shape, level in itertools.product(shapes, ['zero', 'one', 'two', 'heal', 'given',
                                                  'given_empty', 'no_reports', 'repeat']):
        yield ['none', level, 'none', 'assert_vertex' + shape]
    for defect, shape, level in itertools.product(
            ['gap_half', 'gap_full', 'gap_above', 'gap_large', 'vertex_shift',
             'vertex_low_tolerance', 'vertex_high_tolerance', 'root_zero_tolerance'],
            ['', '_uv', '_manifold', '_trihedral'], ['zero', 'two', 'given', 'repeat']):
        yield [defect, level, 'none', 'assert_vertex' + shape]
    for routine, shape, defect in itertools.product(
            ['vertex_pointers', 'vertex_within', 'vertex_within_force', 'vertex_corner', 'vertex_on_corner'],
            ['', '_uv', '_manifold', '_spine', '_trihedral', '_vertexloop', '_wire'],
            ['none', 'vertex_shift', 'gap_above']):
        yield [defect, 'zero', 'none', routine + shape]
    for shape, defect in itertools.product(['', '_manifold', '_spine', '_trihedral'], ['none', 'vertex_shift']):
        yield [defect, 'zero', 'none', 'vertex_parallel' + shape]
    for defect, shape in itertools.product(
            ['face_no_surface', 'face_no_primary', 'fu_no_face', 'fu_no_mate',
             'loop_no_primary', 'lu_no_loop', 'lu_no_mate', 'lu_no_faceuse',
             'eu_no_vertexuse', 'eu_no_ccw', 'eu_no_cw', 'eu_vertex_backpointer',
             'vu_no_owner', 'vertex_no_uses'], ['', '_vertexloop']):
        yield [defect, 'zero', 'none', 'vertex_pointers' + shape]
    for defect, mode in itertools.product(
            ['bspline_inside', 'bspline_outside', 'bspline_boundary'],
            ['assert_vertex', 'vertex_corner', 'vertex_corner_trihedral', 'vertex_within_force']):
        yield [defect, 'two', 'none', mode]
    for mode in ['assert_vertex_uv','vertex_corner_uv','vertex_corner_trihedral_uv','vertex_on_corner_uv','vertex_within_force_uv']:
        yield ['warm_bspline','two','none',mode]
    for shape in ['', '_manifold', '_spine', '_trihedral', '_wire']:
        yield ['none', 'two', 'none', 'assert_vertex' + shape + '_walk']
    yield from map(list, sorted(UNSAFE_NATIVE_INPUTS))


def analytic_truth(case, result):
    """Independent unit-plane truth, not an absolute floor for SDK comparison."""
    defect, _, _, mode = case
    if not mode.startswith('vertex_corner') or 'vertexloop' in mode or 'wire' in mode:
        return
    point = {'none': [0., 0., 0.], 'vertex_shift': [.0003, .0004, .0005],
             'gap_above': [0., 0., 2e-5]}.get(defect)
    if point is None:
        return
    x, y, _ = point
    expected = [0., 0., 0.] if '_trihedral' in mode else [x, 0., 0.] if (
        '_manifold' in mode or '_spine' in mode) else [x, y, 0.]
    query = result['vertex_query']
    assert query[1] is True, ('no analytic corner', case)
    # 1e-14 here is the declared analytic fixture check, in its unit coordinates.
    # The subsequent SDK/Rust comparison retains 1e-9 relative with no floor.
    assert all(abs(a - b) <= 1e-14 for a, b in zip(query[4], expected)), (case, query[4], expected)
    gap = math.sqrt(sum((a - b) ** 2 for a, b in zip(point, expected)))
    assert abs(query[2] - gap) <= 1e-14, (case, query[2], gap)
    assert result['vertex_point'] == point, ('query changed defining vertex', case)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['cpp', 'rust', 'receipt']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        row = {'input': case}
        if tuple(case) in UNSAFE_NATIVE_INPUTS:
            rows.append(dict(row, status='unsafe_native', reason='observed SDK access violation 0xc0000005'))
            continue
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
        if len(rows) % 64 == 0:
            print(json.dumps({'completed': len(rows)}), flush=True)
    counts = {s: sum(r['status'] == s for r in rows) for s in ['pass', 'fail', 'deferred', 'unsafe_native']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n',
                            encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:6]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
