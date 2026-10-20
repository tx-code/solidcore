#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Build/run an original C++ planar API observer against existing source libraries.

No product source mutation or kernel build; only public contract inputs captured.
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
    parser.add_argument('--toolchain-env',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--corpus',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ);supplied=json.loads(args.toolchain_env.read_text(encoding='utf-8'))
    env.update({k:v for k,v in supplied.items() if k in {'Path','PATH','INCLUDE','LIB','LIBPATH','VSLANG'}})
    compiler=shutil.which('cl',path=env.get('Path',env.get('PATH','')))
    if not compiler:raise RuntimeError('selected compiler unavailable')
    output=(args.output/'step-planar-reference.exe').resolve()
    include=[args.source/'source'/name for name in ['STEP_SOURCE/inc','STEP_SEMANTICS/inc','STEP_PLANAR/inc','STEP_PLANAR/tests']]
    libraries=[args.library_dir/'STEP_PLANAR.lib',args.library_dir.parent/'source-semantics/Release/STEP_SEMANTICS.lib',args.library_dir.parent/'source-semantics/source-reader/Release/STEP_SOURCE.lib']
    command=[compiler,'/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG',*['/I'+str(p.resolve()) for p in include],str(Path(__file__).with_name('step_planar_reference.cpp').resolve()),*[str(p.resolve()) for p in libraries],'/link','/OUT:'+str(output)]
    subprocess.run(command,cwd=args.output,env=env,check=True)
    inputs=(args.output/'inputs').resolve();inputs.mkdir(exist_ok=True);env['SOLIDCORE_PLANAR_PUBLIC_CAPTURE']=str(inputs)
    subprocess.run([str(output),'--capture',str(args.corpus.resolve())],env=env,check=True)
    print(json.dumps({'public_api_requests':len(list(inputs.glob('*.request.json')))}))


if __name__=='__main__':main()
