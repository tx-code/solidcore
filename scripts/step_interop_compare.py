#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare the original full C++ source checks to Rust, including policy approval.

Only explicitly captured public synthetic requests are read. No network/model
library scan, expected-answer substitution, geometry tolerance or native build.
"""
import argparse
import json
from pathlib import Path
import subprocess

from step_part_closure_compare import diagnostic_facts, direct_source
from step_type_unit_compare import norm


def rust_flags(flags):
    mapping = {'--max-semantic-visits': '--dependency-visits', '--max-preserved-diagnostics': '--preserved-diagnostics',
               '--max-depth': '--nesting'}
    return ['--source-semantics', *(mapping.get(v, v.replace('--max-', '--')) for v in flags)]


def observe(binary, path, flags):
    result = subprocess.run([str(binary), *flags, str(path)], capture_output=True, timeout=30, check=False)
    assert result.returncode in {0, 1, 2, 3, 5, 7, 64}, 'reader infrastructure failure'
    return result.returncode, json.loads(result.stdout) if result.stdout else None


def compare(cpp, rust, path, flags):
    if '--max-report-bytes' in flags:
        at = flags.index('--max-report-bytes')
        if at + 1 < len(flags) and flags[at + 1].isdigit():
            limit = int(flags[at + 1])
            base = [*flags[:at], *flags[at + 2:]]
            # The source contracts agree; versioned JSON layouts deliberately
            # differ in size. Compare underlying complete source facts, then
            # verify EACH writer's own actual publication budget independently.
            compare(cpp, rust, path, base)
            for binary, full, budget_flag in [(cpp, flags, '--max-report-bytes'), (rust, rust_flags(flags), '--report-bytes')]:
                p = subprocess.run([str(binary), *full, str(path)], capture_output=True, timeout=30, check=False)
                if p.returncode == 5:
                    assert not p.stdout and b'ReportByteLimit' in p.stderr
                else:
                    assert len(p.stdout) <= limit and p.stdout.endswith(b'\n')
                    json.loads(p.stdout)
                clean = base if binary == cpp else rust_flags(base)
                high = 16 * 1024 * 1024
                full = subprocess.run([str(binary), *clean, budget_flag, str(high), str(path)], capture_output=True, timeout=30, check=False)
                assert full.stdout, 'complete report unavailable for format-specific boundary test'
                needle = b'"report_bytes":' + str(high).encode()
                copies = full.stdout.count(needle)
                assert copies > 0
                length = len(full.stdout)
                for _ in range(5):
                    length = len(full.stdout) + copies * (len(str(length)) - len(str(high)))
                assert len(str(length)) == len(str(length - 1))
                exact = subprocess.run([str(binary), *clean, budget_flag, str(length), str(path)], capture_output=True, timeout=30, check=False)
                assert exact.returncode == full.returncode and len(exact.stdout) == length
                short = subprocess.run([str(binary), *clean, budget_flag, str(length - 1), str(path)], capture_output=True, timeout=30, check=False)
                assert short.returncode == 5 and not short.stdout
            return 'format_specific_report_budget'
    rf = rust_flags(flags)
    c = observe(cpp, path, flags)
    r = observe(rust, path, rf)
    assert c[0] == r[0], f'exit differs {c[0]}/{r[0]}'
    if c[1] is None or r[1] is None:
        assert c[1] is None and r[1] is None
        return 'argument_or_report_guard'
    direct_source(cpp, rust, path, flags, rf, (c, r))
    c, r = c[1], r[1]
    assert c['checks'].get('interoperability_approval', 'NotRun') == r['interoperability'], 'final approval state differs'
    if 'geometry_scope' in r:
        assert c.get('geometry_scope', 'ExplicitOuterBounds') == r['geometry_scope']
    if 'policy' in r:
        assert c['source_policy'] == r['policy']
    assert diagnostic_facts(c.get('preserved_diagnostics', [])) == norm(r.get('preserved_diagnostics', [])), 'approved original diagnostics differ'
    interpretation = c.get('schema_interpretation')
    if interpretation:
        assert interpretation['requested'] == r['requested_interpretation']
        assert interpretation['applied'] == (r['applied_interpretation'] != 'DeclaredOnly')
        if interpretation['applied']:
            assert interpretation['source_binding_verified'] is False
        assert diagnostic_facts(interpretation['original_binding_diagnostics']) == norm(r['interpretation_diagnostics'])
    else:
        assert 'requested_interpretation' not in r
    assert c['source']['input']['sha256'] == r['source']['sha256'], 'source capture changed'
    if r['snapshot'] is not None:
        ids = [record['id'] for record in r['source']['records']]
        assert c['snapshot'].get('unattached_non_geometric_declarations', []) == [ids[i] for i in r.get('unattached_non_geometric', [])], 'residue identities differ'
        assert (c['snapshot']['part']['apd_status'] is not None) == r['snapshot']['part_definition']['application_protocol_present']
    return c['status']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--captured', type=Path, action='append', required=True)
    args = parser.parse_args()
    count = 0
    statuses = {}
    for directory in args.captured:
        for request in sorted(directory.glob('*.request.json')):
            path = request.with_name(request.name.removesuffix('.request.json') + '.step')
            flags = json.loads(request.read_text(encoding='utf-8'))['cpp_flags']
            try:
                status = compare(args.cpp.resolve(), args.rust.resolve(), path.resolve(), flags)
            except AssertionError as error:
                raise AssertionError(f'public request {request.stem}, options {flags}: {error}') from error
            count += 1
            statuses[status] = statuses.get(status, 0) + 1
            if count % 100 == 0:
                print(json.dumps({'full_source_comparisons': count, 'differences': 0}), flush=True)
    print(json.dumps({'full_source_comparisons': count, 'statuses': statuses, 'differences': 0,
                      'scope': 'source policy/interpretation/approval; planar/native/full import NotRun'}))


if __name__ == '__main__':
    main()
