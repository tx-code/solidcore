#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual C++/Rust planar source facts, exact metrics and failure evidence.

Public explicit synthetic inputs only; no model library discovery, network or
expected-answer replay. Native build/full import and repair remain unrun.
"""
import argparse
import json
from pathlib import Path
import subprocess

from step_part_closure_compare import diagnostic_facts, direct_source
from step_type_unit_compare import norm


def rust_flags(flags):
    special = {'--max-planar-work': '--planar-work', '--max-semantic-visits': '--dependency-visits', '--max-depth': '--nesting'}
    return [special.get(v, v.replace('--max-', '--')) for v in flags]


def invoke(binary, path, flags):
    result = subprocess.run([str(binary), *flags, str(path)], capture_output=True, timeout=30, check=False)
    assert result.returncode in {0, 1, 2, 3, 5, 7, 8, 64}, 'planar reader infrastructure failure'
    report = json.loads(result.stdout) if result.stdout else None
    if report and 'original_wire' in report:
        report['original_wire']['full_api_diagnostics'] = report['full_api_diagnostics']
        report = report['original_wire']
    return result.returncode, report


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
            return {'status': 'format_specific_report_budget'}
    c_code, c = invoke(cpp, path, flags)
    r_code, r = invoke(rust, path, rust_flags(flags))
    assert c_code == r_code, f'exit differs: {c_code}/{r_code}'
    if c is None or r is None:
        assert c is None and r is None
        return {'status': 'request_or_report_guard'}
    for name in ['status', 'quality', 'limits', 'work', 'checks', 'qualified_source_evidence', 'model', 'arithmetic']:
        assert c[name] == r[name], f'{name} differs'
    assert len(c['diagnostics']) == len(r['diagnostics']), 'planar diagnostic count differs'
    for a, b in zip(c['diagnostics'], r['diagnostics'], strict=True):
        # The original planar wire format intentionally omits some API facts;
        # compare every emitted field, and check full upstream facts below.
        observed = {**{key: b[key] for key in ['classification', 'field', 'declared_in', 'declared_type', 'responsibility', 'expected', 'actual']},
                    **{key: b['source'][key] for key in ['key', 'stage', 'record_id', 'component', 'parameter_path', 'target_id']},
                    'span': norm(b['source']['primary']), 'related_spans': norm(b['source']['related'])}
        for key, value in a.items():
            if key != 'location':
                assert key in observed and observed[key] == value, 'planar diagnostic fact differs: ' + key
    if c['status'] == 'UpstreamFailure':
        assert diagnostic_facts(c['source_semantics']['diagnostics']) == norm(r['diagnostics']), 'full upstream diagnostic provenance lost'
    if 'full_api_diagnostics' in c:
        assert norm(c['full_api_diagnostics']) == norm(r['diagnostics']), 'original API diagnostic facts differ'
    direct_source(cpp, rust, path, (), (), ((c_code, c['source_semantics']), (r_code, r['source_semantics'])))
    records = r['source_semantics']['source']['records'] or []
    ids = [value['id'] for value in records]

    def identity(index):
        return None if index is None else ids[index]

    assert len(c['rules']) == len(r['rules']), 'rule row count differs'
    for a, b in zip(c['rules'], r['rules'], strict=True):
        expected = {key: b[key] for key in ['name', 'kind', 'schema_line', 'applicability', 'evaluation', 'proof', 'facts']}
        expected.update(owner_id=identity(b['owner']), logical_value=b['value'], witness_ids=[identity(i) for i in b['witnesses']],
                        population_scope='OwnerInstance' if b['owner'] is not None else 'FullFile',
                        populations={k: [identity(i) for i in v] for k, v in b['populations'].items()})
        assert a == expected, 'rule identity/value/population/proof differs: ' + a['name']
    assert c['metrics'] == [dict(kind=b['kind'], owner_id=identity(b['owner']), witness_ids=[identity(i) for i in b['witnesses']], exact=b['exact']) for b in r['metrics']], 'exact metrics differ'
    assert c['edges'] == [dict(id=identity(b['edge']), line_id=identity(b['line']), start_id=identity(b['start']), end_id=identity(b['end']),
                               same_sense=b['same_sense'], t_start_coefficient=b['t_start_coefficient'], t_end_coefficient=b['t_end_coefficient'],
                               t_divisor_squared=b['direction_norm_squared'], length_squared_mm=b['length_squared_mm']) for b in r['edges']], 'exact edge intervals differ'
    expected_faces = []
    for b in r['faces']:
        e = dict(id=identity(b['face']), plane_id=identity(b['plane']), placement_id=identity(b['placement']), same_sense=b['same_sense'],
                 source_length_scale_mm=b['source_length_scale_mm'], origin_mm=b['origin_mm'], basis_x=b['x'], basis_y=b['y'], basis_normal=b['normal'],
                 x_norm_squared=b['x_norm_squared'], y_norm_squared=b['y_norm_squared'], metric_squared=b['metric_squared'],
                 projected_region_area_squared_mm=b['area_squared_mm'], projected_region_centroid_mm=b['centroid_mm'],
                 projected_region_bounds_mm=[b['minimum_mm'], b['maximum_mm']], source_uv_formula=b['source_uv_formula'], loops=[])
        for loop in b['loops']:
            e['loops'].append(dict(bound_id=identity(loop['bound']), loop_id=identity(loop['loop_record']), outer=loop['outer'], orientation=loop['orientation'],
                                   oriented_edge_ids=[identity(i) for i in loop['oriented_edges']], edge_ids=[identity(i) for i in loop['edges']], vertex_ids=[identity(i) for i in loop['vertices']],
                                   edge_orientations=loop['edge_orientations'], signed_area_coefficient=loop['signed_area_coefficient'], projected=loop['projected']))
        expected_faces.append(e)
    assert c['faces'] == expected_faces, 'exact projected regions, senses or source UV mapping differ'
    expected_units = [dict(unit_id=identity(b['unit']), dimensions_id=identity(b['dimensions']), factor_id=identity(b['factor']), base_unit_id=identity(b['base_unit']),
                           value_type=b['value_type'], factor_value=b['factor_value'], scale_mm=b['scale_mm'], dimensions=b['exponents']) for b in r['unit_conversions']]
    assert c.get('unit_conversions', []) == expected_units, 'conversion definitions/scales differ'
    return r


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--corpus', type=Path)
    parser.add_argument('--captured', type=Path, action='append', default=[])
    args = parser.parse_args()
    requests = []
    if args.corpus:
        paths = sorted(args.corpus.glob('S203-*/input.step'))
        assert len(paths) == 49
        requests += [(path, ['--fixture-limits', '--quality', 'Q-P0-mm-01']) for path in paths]
    for directory in args.captured:
        for request in sorted(directory.glob('*.request.json')):
            requests.append((request.with_name(request.name.removesuffix('.request.json') + '.step'), json.loads(request.read_text(encoding='utf-8'))['cpp_flags']))
    statuses = {}
    for count, (path, flags) in enumerate(requests, 1):
        try:
            observed = compare(args.cpp.resolve(), args.rust.resolve(), path.resolve(), flags)
        except AssertionError as error:
            raise AssertionError(f'public request {path.parent.name}/{path.name}, flags {flags}: {error}') from error
        statuses[observed['status']] = statuses.get(observed['status'], 0) + 1
        if count % 50 == 0:
            print(json.dumps({'planar_source_comparisons': count, 'differences': 0}), flush=True)
    print(json.dumps({'planar_source_comparisons': len(requests), 'statuses': statuses, 'differences': 0, 'native_and_full_import': 'NotRun'}))


if __name__ == '__main__':
    main()
