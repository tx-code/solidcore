#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare SDK batch subgraphs, distribution, partial writes and true array aliases.

Inputs are public synthetic runtime graphs. IDs/order, status and mark epochs
must agree exactly; this is not whole-model or STEP acceptance evidence.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from brep_constructor_compare import compare, run
from brep_subtopology_compare import cfg

RECIPES = ['empty', 'brep', 'twobrep', 'doublemixed', 'mixbrep', 'mixed', 'faceedge',
           'duplicate', 'vertex', 'firstuse', 'lateuse', 'alluses', 'face', 'edge', 'loop', 'shell', 'region']
ALIASES = ['regions', 'shells', 'faces', 'loops', 'edges', 'vertices']
UNSAFE_NATIVE = {('none', cfg(seed=1), 'none', 'sub_batchaliasfaces'):
                 'observed SDK 0xc0000005: reset zeroes the aliased input, then its first pointer is dereferenced'}


def cases():
    for recipe, mask in itertools.product(RECIPES, range(64)):
        yield ['none', cfg(mask=mask, seed=1), 'none', 'sub_batch' + recipe]
    for recipe, mask, mark, lock in itertools.product(['face', 'mixed', 'firstuse', 'mixbrep'],
                                                     [1, 2, 4, 8, 16, 32, 21, 63],
                                                     [1, 2, 4, 8, 16, 31], [0, 1]):
        yield ['none', cfg(mask=mask, mark=mark, lock=lock), 'none', 'sub_batch' + recipe]
    for recipe, mask, mark in itertools.product(['mixed', 'duplicate', 'vertex'], [4, 32, 63], [1, 2, 4, 8, 16]):
        yield ['none', cfg(mask=mask, reset=0, mark=mark, lock=1, repeats=2, seed=2, premark=1),
               'none', 'sub_batch' + recipe]
    for i, alias in enumerate(ALIASES):
        for mask in range(64):
            if mask & (1 << i):
                yield ['none', cfg(mask=mask, reset=0, seed=2), 'none', 'sub_batchalias' + alias]
        for mask, mark, lock in itertools.product(sorted({1 << i, (1 << i) | 32, 63}), [1, 2, 16, 31], [0, 1]):
            yield ['none', cfg(mask=mask, reset=0, mark=mark, lock=lock, seed=1, repeats=2),
                   'none', 'sub_batchalias' + alias]
        yield ['none', cfg(seed=0), 'none', 'sub_batchalias' + alias]
    for profile, recipe in itertools.product(['hole', 'vertexloop', 'wire', 'shellvertex', 'manifold', 'spine', 'trihedral'],
                                            ['mixed', 'faceedge', 'duplicate', 'mixbrep', 'aliasfaces']):
        yield ['none', cfg(reset=0, seed=1, repeats=2), 'none', 'sub_batch' + recipe + '_' + profile]
    yield ['none', cfg(mask=4), 'none', 'sub_batchduplicate']
    yield ['none', cfg(mask=32), 'none', 'sub_batchvertex']
    yield from map(list, UNSAFE_NATIVE)


def invariants(case, result):
    mask, reset, _, _, _, seed, _ = map(int, case[1].split(','))
    out = result['sub_output']
    recipe = case[3][9:].split('_')[0]
    assert len(out) == 6
    assert all((v is not None) == bool(mask & (1 << i)) for i, v in enumerate(out)), ('output mask', case)
    if not mask or recipe == 'empty':
        assert result['status'] == 1000 and result['sub_advances'] == [0] * 5
        assert all(v is None or len(v) == seed for v in out), ('early return reset outputs', case)
    elif recipe in ['twobrep', 'doublemixed']:
        assert result['status'] == 1007 and result['sub_advances'] == [0] * 5
    elif recipe in ['firstuse', 'alluses']:
        assert result['status'] == 1001 and all(v in (None, []) for v in out)
    elif case[3] == 'sub_batchduplicate' and mask == 4 and reset and seed == 0:
        assert len(out[2]) == 2 and out[2][0] == out[2][1], ('source Add retains duplicates', case)
    elif case[3] == 'sub_batchvertex' and mask == 32 and reset and seed == 0:
        assert len(out[5]) == 1, ('distributed input vertex retained', case)


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
