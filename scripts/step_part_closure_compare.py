#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare source part/closure facts with actual C++ methods and direct P0 CLI.

Explicit public synthetic corpus/capture directories only; no model-library scan,
network or expected-answer export. JSON representation differences are mapped,
while identity, diagnostics, role order, source slots and counters must match.
"""
import argparse
from pathlib import Path
import json
import subprocess
import sys

sys.dont_write_bytecode = True
from step_type_unit_compare import compare, norm, selection  # local test tooling


def invoke(binary, path, flags):
    result = subprocess.run([str(binary), '--fixture-limits', *flags, str(path)],
                            capture_output=True, timeout=30, check=False)
    assert result.returncode in {0, 1, 2, 3, 7}, 'reference infrastructure failure'
    return result.returncode, json.loads(result.stdout)


def direct_source(cpp, rust, path, cpp_flags, rust_flags):
    c_code, c = invoke(cpp, path, cpp_flags)
    r_code, r = invoke(rust, path, rust_flags)
    assert c_code == r_code and c['status'] == r['status']
    for c_key, r_key in [('schema_binding', 'binding'), ('type_fields', 'types'),
                         ('single_part_profile', 'part'), ('si_units', 'si_units')]:
        assert c['checks'][c_key] == r[r_key], 'direct P0 phase differs'
    if 'geometry_dependency_checks' in r:
        assert c['checks'].get('unit_dependencies','NotRun')==r['unit_dependencies']
        assert c['checks'].get('geometry_dependencies','NotRun')==r['geometry_dependencies']
        stats=c['statistics']
        assert r['geometry_dependency_checks']==dict(total=stats.get('geometry_dependency_checks',0),type_predicates=stats.get('geometry_type_predicate_checks',0),entity_predicates=stats.get('geometry_entity_predicate_checks',0))
        assert r['unit_dependency_checks']==dict(total=stats.get('unit_dependency_checks',0),unit_definitions=stats.get('unit_dimension_definitions',0),type_predicates=stats.get('unit_type_predicate_checks',0))
        assert r['property_attachment_checks']==stats.get('property_attachment_checks',0)
    assert c['statistics']['dependency_visits'] == r['dependency_visits']
    assert selection(c['schema_selection']) == selection(r['schema_selection'])
    diagnostics = []
    for d in c['diagnostics']:
        source = {key: d[key] for key in ['key', 'stage', 'record_id', 'component',
                                         'parameter_path', 'target_id']}
        source['primary'] = d['span']
        source['related'] = [related['span'] for related in d['related']]
        diagnostics.append(dict(source=source, **{key: d[key] for key in
            ['classification', 'field', 'declared_in', 'declared_type', 'expected', 'actual', 'responsibility']}))
    assert norm(diagnostics) == norm(r['diagnostics']), 'direct P0 diagnostic contract differs'
    if c['snapshot'] is None:
        assert r['snapshot'] is None
        return
    cs, rs = c['snapshot'], r['snapshot']
    for cr, rr in zip(cs['records'], rs['records'], strict=True):
        assert cr['id'] == rr['id'] and cr['types'] == rr['types']
        assert cr['used_by_representation'] == rr['used_by_representation']
        for ca, ra in zip(cr['attributes'], rr['attributes'], strict=True):
            ca = dict(ca)
            ca['component'] = ca.pop('component_index')
            ca['parameter'] = ca.pop('parameter_index')
            expected_span = ca.pop('span')
            ca.setdefault('nested_aggregates', [])
            assert norm(ca) == norm(ra)
            raw = r['source']['records'][cr['source_record_index']]['components'][ra['component']]['parameters'][ra['parameter']]
            assert expected_span == norm(raw['span'])
    part = rs['part_definition']
    def source_id(index):
        return None if index is None else rs['records'][index]['id']
    assert list(cs['part']['roles'].items()) == [(role['name'], source_id(role['record_index'])) for role in part['roles']]
    assert cs['part']['source_shape_role'] == part['shape_role']
    assert cs['part']['apd_status'] == part['apd_status']
    assert cs['part']['apd_schema_claim'] == part['apd_schema_claim']
    assert cs['part']['apd_year_lexeme'] == part['apd_year']
    assert cs['part']['shape_geometry_verified'] is False
    assert cs['part'].get('property_attachments',[])==[{key:source_id(value) for key,value in item.items()} for item in part['property_attachments']]
    if 'deferred_region_faces' in cs['part']:
        assert cs['part']['deferred_region_faces']==[source_id(i) for i in part['deferred_region_faces']]
    for quantity in ['length_unit', 'plane_angle_unit', 'solid_angle_unit']:
        assert cs['units'][quantity] == source_id(rs['units'][quantity])
    def scale(power):
        if power is None:
            return None
        return dict(kind='rational', num='1' + '0' * max(power, 0), den='1' + '0' * max(-power, 0))
    assert cs['units']['length_scale_mm'] == scale(rs['units']['length_power10_mm'])
    for cu, ru in zip(cs['units']['si_definitions'], rs['units']['si_definitions'], strict=True):
        assert cu['source_id'] == source_id(ru['record_index'])
        assert cu['quantity'] == ru['quantity'] and cu['name'] == ru['si_name']
        assert (cu['prefix'] or '') == ru['prefix']
        assert cu['scale'] == scale(ru['canonical_power10'])
        assert cu['dimensions'] == ru['dimensions']
    if 'dimension_definitions' in cs['units']:
        expected_dimensions=[dict(source_id=source_id(d['record_index']),nominal=d['nominal'],dimensions=[dict(kind='rational',num=x['numerator'],den=x['denominator']) for x in d['dimensions']]) for d in rs['units']['dimension_definitions']]
        assert cs['units']['dimension_definitions']==expected_dimensions
    expected = {name: [record['id'] for record in rs['records'] if name in record['types']]
                for name in rs['registered_types']}
    assert cs['type_populations'] == expected
    assert c['source']['input']['sha256'] == r['source']['sha256']
    assert c['source']['input']['raw'] == r['source']['raw'] == path.read_bytes().decode('ascii')


def direct_p0(cpp,rust,path):
    return direct_source(cpp,rust,path,[],['--part-and-closure'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp', type=Path, required=True)
    parser.add_argument('--rust', type=Path, required=True)
    parser.add_argument('--corpus', type=Path, required=True)
    parser.add_argument('--cpp-full-p0', type=Path)
    parser.add_argument('--captured', type=Path, action='append', default=[])
    args = parser.parse_args()
    cpp, rust = args.cpp.resolve(), args.rust.resolve()
    cases = sorted(args.corpus.glob('S203-*/input.step'))
    assert len(cases) == 49
    checkpoints = 0
    direct = 0
    successful = []
    for case in cases:
        result = compare(cpp, rust, case.resolve(), ['--part-and-closure'])
        checkpoints += 1
        if result['status'] in {'Checked', 'Deferred'}:
            successful.append((case.resolve(), ['--part-and-closure'], result))
        if args.cpp_full_p0:
            try:
                direct_p0(args.cpp_full_p0.resolve(), rust, case.resolve())
            except AssertionError as error:
                raise AssertionError(f'direct P0 {case.parent.name}: {error}') from error
            direct += 1
    seen = set()
    for root in args.captured:
        for case in sorted(root.glob('*.step')):
            scope, budget = case.with_suffix('.options').read_text().splitlines()
            flags = ['--part-and-closure', '--dependency-visits', budget]
            if scope == 'general':
                flags.append('--general-brep-source')
            identity = (case.read_bytes(), scope, budget)
            if identity in seen:
                continue
            seen.add(identity)
            try:
                result = compare(cpp, rust, case.resolve(), flags)
            except AssertionError as error:
                raise AssertionError(f'captured public request {case.name}: {error}') from error
            checkpoints += 1
            if result['status'] in {'Checked', 'Deferred'}:
                successful.append((case.resolve(), flags, result))
    # Probe boundaries *inside* the ownership phase, not just its final visit.
    # Reuse one actual successful input per profile. Exact stage/primary position
    # and withheld-snapshot behavior must match at every selected resource stop.
    profiles = set()
    boundary = 0
    for case, flags, result in successful:
        profile = result['schema_selection']['qualified']['profile']
        if profile in profiles:
            continue
        profiles.add(profile)
        visits = result['dependency_visits']
        base = [flag for flag in flags if flag not in {'--dependency-visits'}]
        if '--dependency-visits' in flags:
            position = flags.index('--dependency-visits')
            base = flags[:position] + flags[position + 2:]
        before = compare(cpp, rust, case, [flag for flag in base if flag != '--part-and-closure'])['dependency_visits']
        for limit in sorted({0, 1, before, before+1, before+3, visits // 3, visits // 2, visits - 1, visits, 10000001}):
            compare(cpp, rust, case, [*base, '--dependency-visits', str(limit)])
            boundary += 1
    print(json.dumps(dict(checkpoint_comparisons=checkpoints, direct_cpp_p0=direct,
                          boundary_comparisons=boundary, covered_profiles=len(profiles),
                          differences=0, geometry_and_native='NotRun')))


if __name__ == '__main__':
    main()
