#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare strict source phases with both original C++ CLI and scoped phase runner.

Only explicit public corpus/capture directories are read. No private model scan,
network, geometry fitting or expected-answer export. All unrun capabilities remain
unrun; a strict source success is not a native import or full AP validity claim.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode=True
from step_type_unit_compare import compare
from step_part_closure_compare import direct_source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp',type=Path,required=True)
    parser.add_argument('--cpp-full',type=Path,required=True)
    parser.add_argument('--rust',type=Path,required=True)
    parser.add_argument('--corpus',type=Path,required=True)
    parser.add_argument('--captured',type=Path,action='append',default=[])
    args=parser.parse_args();cpp=args.cpp.resolve();full=args.cpp_full.resolve();rust=args.rust.resolve()
    cases=sorted(args.corpus.glob('S203-*/input.step'));assert len(cases)==49
    requests=[(path.resolve(),'planar','1000000') for path in cases]
    for root in args.captured:
        for path in sorted(root.glob('*.step')):
            scope,budget=path.with_suffix('.options').read_text().splitlines()
            requests.append((path.resolve(),scope,budget))
    seen=set();count=0;boundaries={}
    for path,scope,budget in requests:
        identity=(hashlib.sha256(path.read_bytes()).hexdigest(),scope,budget)
        if identity in seen:continue
        seen.add(identity)
        c_flags=['--max-semantic-visits',budget];r_flags=['--source-semantics','--dependency-visits',budget]
        if scope=='general':c_flags.append('--general-brep-source');r_flags.append('--general-brep-source')
        try:
            result=compare(cpp,rust,path,r_flags)
            direct_source(full,rust,path,c_flags,r_flags)
        except AssertionError as error:
            raise AssertionError(f'public source {path.name} ({scope}): {error}') from error
        count+=1
        if result['geometry_dependencies']=='Passed':
            kind=result['schema_selection']['qualified']['profile']
            # Prefer an input that actually attempts a named geometry predicate.
            previous=boundaries.get(kind)
            if previous is None or previous[2]['geometry_dependency_checks']['total']==0:
                boundaries[kind]=(path,scope,result)
        if count%100==0:print(json.dumps({'strict_source_requests':count,'direct_cpp_matched':count,'differences':0}),flush=True)
    probes=0
    for path,scope,good in boundaries.values():
        base=['--general-brep-source'] if scope=='general' else []
        typed=compare(cpp,rust,path,base)['dependency_visits']
        for budget in sorted({typed,typed+1,typed+3,good['dependency_visits']-1,good['dependency_visits'],10000001}):
            r_flags=['--source-semantics',*base,'--dependency-visits',str(budget)]
            c_flags=[*base,'--max-semantic-visits',str(budget)]
            compare(cpp,rust,path,r_flags);direct_source(full,rust,path,c_flags,r_flags);probes+=1
    print(json.dumps({'strict_source_requests':count,'phase_comparisons':count+probes,'direct_original_cpp_comparisons':count+probes,'geometry_budget_probes':probes,'geometry_profiles':len(boundaries),'differences':0,'native_and_full_import':'NotRun'}))


if __name__=='__main__':main()
