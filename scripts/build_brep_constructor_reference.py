#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Build the synthetic constructor observer against an existing Windows SMLib SDK."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--sdk', type=Path, required=True, help='Release SDK directory, matching source headers')
    parser.add_argument('--toolchain-env', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe',
                        choices=['constructor', 'validation', 'notification', 'surface', 'curve',
                                 'curve_validation', 'edge_queries'], default='constructor')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    supplied = json.loads(args.toolchain_env.read_text(encoding='utf-8'))
    env.update({k: v for k, v in supplied.items() if k in {'Path', 'PATH', 'INCLUDE', 'LIB', 'LIBPATH', 'VSLANG'}})
    compiler = shutil.which('cl', path=env.get('Path', env.get('PATH', '')))
    if not compiler:
        raise RuntimeError('selected compiler unavailable')
    output = (args.output / f'brep-{args.probe}-reference.exe').resolve()
    command = [compiler, '/nologo', '/std:c++20', '/EHsc', '/MD', '/O2', '/utf-8', '/DNDEBUG',
               '/fp:precise', '/DNOMINMAX', '/DUNICODE', '/D_UNICODE', '/DNL_DEBUG_CODE',
               '/DSM_GRAPHICS_CALLBACKS=1', '/I' + str((args.source / 'source/SMLib/inc').resolve()),
               str(Path(__file__).with_name(f'brep_{args.probe}_reference.cpp').resolve()),
               str((args.sdk / 'SMLib.lib').resolve()), '/link', '/OUT:' + str(output)]
    subprocess.run(command, cwd=args.output, env=env, check=True)
    shutil.copy2(args.sdk / 'SMLib.dll', args.output / 'SMLib.dll')
    print(output)


if __name__ == '__main__':
    main()
