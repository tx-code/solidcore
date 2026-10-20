#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Capture only the synthetic inputs constructed by the existing C++ contract suite.

Builds a test application that redirects Check calls to a recording wrapper and
then calls the original Check unchanged. No expected results are exported. The
reference checkout is read-only; source, manifests and binaries stay in an ignored
local output directory. Only explicit strict-source/policy cases are retained.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--library-dir',type=Path,required=True)
    parser.add_argument('--corpus',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--toolchain-env',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ)
    if args.toolchain_env:
        values=json.loads(args.toolchain_env.read_text(encoding='utf-8'))
        env.update({k:v for k,v in values.items() if k in {'PATH','Path','INCLUDE','LIB','LIBPATH','VSLANG'}})
    compiler=shutil.which('cl',path=env.get('Path',env.get('PATH',''))) or shutil.which('cl',path=env.get('PATH',''))
    if not compiler:raise RuntimeError('selected MSVC compiler unavailable')
    binary=(args.output/'capture-public-contracts.exe').resolve()
    command=[compiler,'/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG',
        '/I'+str(args.source/'source/STEP_SEMANTICS/inc'),'/I'+str(args.source/'source/STEP_SEMANTICS/tests'),
        '/I'+str(args.source/'source/STEP_SOURCE/inc'),str(Path(__file__).with_name('step_capture_contract_inputs.cpp').resolve()),
        str((args.library_dir/'STEP_SEMANTICS.lib').resolve()),str((args.library_dir.parent/'source-reader/Release/STEP_SOURCE.lib').resolve()),
        '/link','/OUT:'+str(binary)]
    subprocess.run(command,cwd=args.output,env=env,check=True)
    inputs=(args.output/'inputs').resolve();inputs.mkdir(exist_ok=True)
    env['SOLIDCORE_PUBLIC_CONTRACT_CAPTURE']=str(inputs)
    subprocess.run([str(binary),str(args.corpus.resolve())],env=env,check=True)
    print(json.dumps({'public_synthetic_strict_inputs':len(list(inputs.glob('*.step')))}))


if __name__=='__main__':main()
