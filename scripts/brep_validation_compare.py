#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare required-link diagnostics against actual C++ ValidatePointers.

The Rust observer checks only the required-link prefix. Its positive cases do
not qualify complete pointer/geometry validation or native publication.
"""
import argparse
import itertools
import json
from pathlib import Path
from brep_constructor_compare import compare, run

DEFECTS = ['none', 'no_infinite_region', 'region_no_owner', 'shell_no_owner', 'shell_empty',
           'face_no_surface', 'face_no_primary', 'fu_no_face', 'fu_no_mate', 'fu_no_shell',
           'fu_no_loops', 'loop_no_primary', 'lu_no_loop', 'lu_no_mate', 'lu_no_faceuse',
           'lu_no_start', 'edge_no_curve', 'edge_no_uses', 'eu_no_edge', 'eu_no_vertexuse',
           'eu_no_owner', 'eu_no_ccw', 'eu_no_cw', 'vu_no_vertex', 'vu_no_owner', 'vertex_no_uses',
           'loop_bad_ring']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp', type=Path, required=True)
    p.add_argument('--rust', type=Path, required=True)
    p.add_argument('--receipt', type=Path, required=True)
    p.add_argument('--full', action='store_true', help='Compare complete pointer execution, not AssertValid')
    args = p.parse_args()
    cases = [[d, level] for d, level in itertools.product(DEFECTS, ['zero', 'two', 'given', 'no_reports', 'repeat'])]
    cases += [[a, 'zero', b] for a, b in [('face_no_surface', 'fu_no_mate'),
                                         ('shell_no_owner', 'face_no_surface'),
                                         ('eu_no_ccw', 'edge_no_curve'),
                                         ('vertex_no_uses', 'face_no_primary')]]
    if args.full:
        body=['fu_wrong_orientation','mate_wrong_orientation','lu_wrong_orientation','mate_lu_wrong_orientation',
              'face_low_tolerance','face_high_tolerance','edge_low_tolerance','edge_high_tolerance',
              'vertex_low_tolerance','vertex_high_tolerance','root_zero_tolerance','surface_no_owner',
              'reverse_front','reverse_back','reverse_both']
        cases += [[d,level] for d,level in itertools.product(body,['zero','two','given','no_reports','repeat'])]
        cases=[case+(['none'] if len(case)==2 else [])+['full'] for case in cases]
        # Direct NLib cases compare its failure bits and actual control-point
        # mutation; these are callee evidence, not extra valid-model claims.
        raw=['reverse_front','reverse_back','reverse_both','reverse_2d','reverse_rational',
             'reverse_flat','reverse_tiny','reverse_limit','reverse_above']
        cases += [[d,'zero','none',mode] for d,mode in itertools.product(raw,['raw_fix','raw_check'])]
    rows = []
    for case in cases:
        try:
            cpp = run(args.cpp.resolve(), case)
            rust = run(args.rust.resolve(), case)
            difference = compare(cpp, rust)
            rows.append({'input': case, 'status': 'fail' if difference else 'pass',
                         'difference': difference, 'cpp': cpp, 'rust': rust})
        except (RuntimeError, ValueError) as error:
            rows.append({'input': case, 'status': 'fail', 'error': str(error)})
    failed = [r for r in rows if r['status'] == 'fail']
    result = {'scope': 'complete pointer execution on selected fixtures; AssertValid not established' if args.full else 'required-link prefix only; full validation not established',
              'cases': rows, 'passed': len(rows) - len(failed), 'failed': len(failed)}
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'cases'}))
    for row in failed[:5]:
        print(json.dumps(row))
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
