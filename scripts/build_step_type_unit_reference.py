#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Build a LOCAL selected source checkpoint without modifying the reference checkout.

The copied Checker changes only its private/public access label. Original phase
bodies and catalogs are used verbatim; the appended test entrypoint selects only
the requested phases. No C++ reader is a Rust runtime dependency. MSVC environment
must already be configured, or supplied via an explicit local allowlisted JSON.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import shutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--library-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--toolchain-env',type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    reference=args.source/'source/STEP_SEMANTICS/src/StepSemantics.cpp'
    raw=reference.read_bytes()
    text=raw.decode('utf-8').replace('\r\n','\n')
    assert text.count('\nprivate:\n')==1, 'reference Checker layout changed; review adapter'
    shim=Path(__file__).with_name('step_type_unit_reference.cpp').read_text(encoding='utf-8')
    copied=text.replace('\nprivate:\n','\npublic:\n',1)+'\n'+shim
    unit=args.output/'reference.cpp'
    unit.write_text(copied,encoding='utf-8')
    env=dict(os.environ)
    if args.toolchain_env:
        supplied=json.loads(args.toolchain_env.read_text(encoding='utf-8'))
        allowed={'PATH','Path','INCLUDE','LIB','LIBPATH','VSLANG'}
        env.update({k:v for k,v in supplied.items() if k in allowed})
    compiler=shutil.which('cl',path=env.get('Path',env.get('PATH','')))
    if not compiler:
        compiler=shutil.which('cl',path=env.get('PATH',''))
    if not compiler:
        raise RuntimeError('MSVC compiler not found in the selected toolchain PATH')
    command=[compiler,'/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG',
             '/I'+str(args.source/'source/STEP_SEMANTICS/inc'),
             '/I'+str(args.source/'source/STEP_SEMANTICS/src'),
             '/I'+str(args.source/'source/STEP_SOURCE/inc'),
             '/I'+str(args.source/'third_party/microsoft-gsl/include'),
             str(unit.resolve()),str((args.library_dir/'STEP_SEMANTICS.lib').resolve()),
             str((args.library_dir.parent/'source-reader/Release/STEP_SOURCE.lib').resolve()),
             '/link','/OUT:'+str((args.output/'step-type-unit-reference.exe').resolve())]
    subprocess.run(command,cwd=args.output,env=env,check=True)
    assert reference.read_bytes()==raw, 'reference source changed during the build'
    (args.output/'reference-identity.local.json').write_text(json.dumps({
        'source_sha256':hashlib.sha256(raw).hexdigest(),
        'access_change':'private -> public, one Checker access label only',
        'entrypoint_sha256':hashlib.sha256(shim.encode()).hexdigest(),
        'supported_scopes':['TypesAndUnitsCheckpoint','PartAndClosureCheckpoint','GeometryDependenciesCheckpoint','StrictSourceSemantics'],
        'always_unrun':['native construction','interoperability approval','continuous geometry consistency','planar/global rules']
    },indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
