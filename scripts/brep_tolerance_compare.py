#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual native tolerance and bounding-box queries on synthetic models.

These are SDK operation results, not imported-model or continuous-error claims.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
from brep_constructor_compare import run, compare


def cases():
    gaps=['none','gap_half','gap_between','gap_full','gap_above','gap_large']
    for defect,selection,prior,mode in itertools.product(gaps,['zero','two','given','no_reports','repeat'],
                                                        ['none','initial_validate_only'],['tolerance_check','tolerance_update']):
        yield [defect,selection,prior,mode]
    for defect,mode in itertools.product(['none','gap_above','locked_box'],
                                         [*[f'bbox_{n}' for n in range(8)],'bbox_tight']):
        yield [defect,'zero','none',mode]

    for defect,selection,prior,mode in itertools.product(gaps,['zero','repeat'],
                                                        ['none','initial_validate_only'],['cache_build','cache_editing']):
        yield [defect,selection,prior,mode]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp',type=Path,required=True)
    p.add_argument('--rust',type=Path,required=True)
    p.add_argument('--receipt',type=Path,required=True)
    args=p.parse_args()
    rows=[]
    for case in cases():
        try:
            a=run(args.cpp.resolve(),case);b=run(args.rust.resolve(),case)
            if 'deferred' in b:
                rows.append({'input':case,'status':'deferred','dependency':b['deferred'],'cpp':a})
                continue
            difference=compare(a,b)
            rows.append({'input':case,'status':'fail' if difference else 'pass','difference':difference,'cpp':a,'rust':b})
        except (RuntimeError,ValueError,subprocess.TimeoutExpired) as e:
            rows.append({'input':case,'status':'fail','error':str(e)})
    failed=[r for r in rows if r['status']=='fail']
    deferred=[r for r in rows if r['status']=='deferred']
    receipt={'scope':'native tolerance/check-update and box operation contracts; no import acceptance',
             'passed':sum(r['status']=='pass' for r in rows),'failed':len(failed),'deferred':len(deferred),'cases':rows}
    args.receipt.parent.mkdir(parents=True,exist_ok=True)
    args.receipt.write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in receipt.items() if k!='cases'}))
    for row in failed[:5]:print(json.dumps(row))
    if failed:raise SystemExit(1)


if __name__=='__main__':main()
