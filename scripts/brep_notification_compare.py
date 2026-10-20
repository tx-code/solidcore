#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual root notifications, persistent metadata and callbacks on synthetic geometry."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import compare, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    events = ['pre_edit', 'destruction', 'post_edit', 'add_v', 'remove_v', 'add_e', 'remove_e',
              'add_f', 'remove_f', 'split', 'split_alias', 'merge', 'copy', 'construct', 'coincident', 'unknown']
    rows = []
    for event, cached, repeated in itertools.product(events, [0, 1], [0, 1]):
        case = [event, cached, repeated]
        try:
            cpp = run(args.cpp.resolve(), case)
            rust = run(args.rust.resolve(), case)
            if 'deferred' in rust:
                rows.append({'input': case, 'status': 'deferred', 'dependency': rust['deferred']})
                continue
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass',
                         'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    counts = {s: sum(row['status'] == s for row in rows) for s in ['pass', 'fail', 'deferred']}
    receipt = {'scope': 'same-model root notifications and actual spatial/attribute/callback state; not full AssertValid',
               'counts': counts, 'cases': rows}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(counts))
    for row in [row for row in rows if row['status'] == 'fail'][:5]:
        print(json.dumps(row))
    if counts['fail']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
