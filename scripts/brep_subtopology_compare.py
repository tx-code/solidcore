#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Actual SDK optional/marked/append subgraph collection and descendant-walk comparisons.

Source duplicates, output order, current mark epochs and counter advances are
compared exactly. This establishes selected runtime behavior, not STEP acceptance.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run

ROOTS = ['brep', 'region', 'shell', 'face', 'loop', 'edge', 'vertex']
USES = ['faceuse', 'loopuse', 'edgeuse', 'vertexuse']
UNSAFE_NATIVE = {
    ('none', '32,1,2,1,1,0,0', 'none', 'sub_face'):
        'observed SDK access violation: marked non-root partial outputs dereference a NULL work array',
    ('none', 'zero', 'none', 'assert_shell_wire_walk'):
        'observed SDK access violation: curveless wire reaches Edge::AssertValid',
}


def cfg(mask=63, reset=1, mark=0, lock=0, repeats=1, seed=0, premark=0):
    return ','.join(map(str, [mask, reset, mark, lock, repeats, seed, premark]))


def cases():
    for kind, mask in itertools.product(ROOTS, range(64)):
        yield ['none', cfg(mask=mask), 'none', 'sub_' + kind]
    for kind, mask in itertools.product(USES, [0, 1, 4, 32, 63]):
        yield ['none', cfg(mask=mask, reset=0, seed=2), 'none', 'sub_' + kind]
    for kind, mask, seed in itertools.product(ROOTS, [1, 2, 4, 8, 16, 32, 21, 63], [1, 2]):
        yield ['none', cfg(mask=mask, reset=0, seed=seed, repeats=2), 'none', 'sub_' + kind]
    for kind, mark, lock, pattern in itertools.product(ROOTS, [1, 2, 4, 8, 16, 31], [0, 1], range(3)):
        config = [cfg(mark=mark, lock=lock), cfg(reset=0, mark=mark, lock=lock, seed=1, premark=2),
                  cfg(reset=0, mark=mark, lock=lock, seed=2, premark=1, repeats=2)][pattern]
        yield ['none', config, 'none', 'sub_' + kind]
    # Partial marked outputs are defined in the root branch, and invalid use
    # types return before the unsafe non-root mark setup.
    for mask, mark, lock in itertools.product([1, 4, 21, 32, 63], [1, 2, 4, 8, 16], [0, 1]):
        yield ['none', cfg(mask=mask, mark=mark, lock=lock, premark=2), 'none', 'sub_brep']
    for kind, mask, mark in itertools.product(USES, [1, 32], [1, 2]):
        yield ['none', cfg(mask=mask, mark=mark, lock=1), 'none', 'sub_' + kind]
    for shape, kind, config in itertools.product(
            ['hole', 'hole_shared', 'vertexloop', 'wire', 'shellvertex', 'manifold', 'spine', 'trihedral'],
            ROOTS, [cfg(), cfg(reset=0, seed=1), cfg(mark=2, lock=1, repeats=2)]):
        yield ['none', config, 'none', 'sub_' + kind + '_' + shape]
    for kind, level in itertools.product(['region', 'shell', 'face', 'loop', 'edge'],
                                         ['zero', 'one', 'two', 'given', 'given_empty', 'repeat']):
        yield ['none', level, 'none', 'assert_' + kind + '_walk']
    for kind, shape in itertools.product(['region', 'shell', 'face', 'loop', 'edge'],
                                         ['hole', 'manifold', 'trihedral']):
        yield ['none', 'zero', 'none', 'assert_' + kind + '_' + shape + '_walk']
    yield ['none', 'zero', 'none', 'assert_shell_wire_geometry_walk']
    yield from map(list, UNSAFE_NATIVE)


def invariants(case, result):
    if not case[3].startswith('sub_'):
        return
    mask, reset, mark, _, _, seed, _ = map(int, case[1].split(','))
    out = result['sub_output']
    assert len(out) == 6
    for i, values in enumerate(out):
        assert (values is not None) == bool(mask & (1 << i)), ('optional output', case, i)
    kind = case[3][4:].split('_')[0]
    if kind in USES and mask:
        assert result['status'] == 1001 and all(v in (None, []) for v in out), ('invalid type', case)
    if not mask:
        assert result['status'] == 1000 and result['sub_advances'] == [0] * 5, ('no-output fast return', case)
    if reset and not seed and kind in ROOTS[1:]:
        index = ROOTS.index(kind) - 1
        if out[index] is not None:
            assert out[index] == [], ('receiver should be excluded', case)
    if case[3] == 'sub_region' and mask == 63 and reset and mark == 0 and seed == 0:
        assert len(out[2]) == 2 and out[2][0] == out[2][1], ('source faceuse mark quirk', case)
        assert list(map(len, out[3:])) == [1, 3, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['cpp', 'rust', 'receipt']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for case in cases():
        row = {'input': case}
        if tuple(case) in UNSAFE_NATIVE:
            rows.append(dict(row, status='unsafe_native', reason=UNSAFE_NATIVE[tuple(case)]))
            continue
        try:
            native, actual = run(args.cpp.resolve(), case), run(args.rust.resolve(), case)
            invariants(case, native)
            if 'deferred' in actual:
                row.update(status='deferred', dependency=actual['deferred'], cpp=native)
            else:
                invariants(case, actual)
                difference = compare(native, actual)
                row.update(status='fail' if difference else 'pass', difference=difference, cpp=native, rust=actual)
        except (RuntimeError, ValueError, AssertionError, subprocess.TimeoutExpired) as error:
            row.update(status='fail', error=str(error))
        rows.append(row)
        if len(rows) % 128 == 0:
            print(json.dumps({'completed': len(rows)}), flush=True)
    counts = {s: sum(r['status'] == s for r in rows) for s in ['pass', 'fail', 'deferred', 'unsafe_native']}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({'scope': __doc__, 'counts': counts, 'cases': rows}, indent=2) + '\n',
                            encoding='utf-8')
    print(json.dumps(counts))
    for row in [r for r in rows if r['status'] == 'fail'][:6]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
