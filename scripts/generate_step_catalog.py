#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Generate immutable Rust transport descriptors from the C++ catalog snapshot.

The input is the public step_catalog_snapshot test tool's complete descriptor
export, never a customer STEP file or an independent standards witness. Optional
versioned standard additions have a separate identity and never rewrite that
export or its fingerprint. This is an offline maintainer step; building/running
Rust does not invoke C++, Python, or the network.
"""
import argparse
import copy
import hashlib
import json
import re
from pathlib import Path


def string(value):
    return json.dumps(value, ensure_ascii=True)


def array(items):
    return '&[' + ','.join(items) + ']'


def optional(value):
    return 'None' if value is None else f'Some({value})'


def field(f):
    nested = array('Dimension { minimum: %s, maximum: %s, unique: %s, set: %s }' %
                   (d['minimum'], optional(d['maximum']), str(d['unique']).lower(),
                    str(d['aggregate'] == 'SET').lower()) for d in f['nested_aggregates'])
    return ('Field { name: %s, declared_type: %s, kind: Kind::%s, optional: %s, element: Kind::%s, '
            'minimum: %s, maximum: %s, unique: Unique::%s, nested: %s }') % (
                string(f['name']), string(f['type']), f['kind'], str(f['optional']).lower(),
                f['element_kind'], f['minimum'], optional(f['maximum']), f['unique'], nested)



# Deliberately one audited entity, not a general EXPRESS-extension compiler.
# Each profile must match its already selected, fixed source artifact; adding a
# name to this manifest does not introduce another schema or relax P0.
ARTIFACT_BY_PROFILE = {
    'Ap203e2GeneralSource': 'N2635',
    'Ap214e3Source': 'AP214E3_2010',
    'Ap214MbxSource': 'AP214E3_2010',
    'Ap203N916Source': 'N916Modified',
}
VERTEX_LOOP = {
    'name': 'VERTEX_LOOP', 'parent': 'LOOP', 'simple': True,
    'field': {'name': 'loop_vertex', 'type': 'VERTEX',
              'kind': 'Reference', 'optional': False},
}


def add_standard_vertex_loop(baseline, manifest):
    """Append verified transport metadata without modifying the C++ export."""
    if (manifest.get('format') != 'solidcore.step.catalog.standard-additions.v1'
            or manifest.get('id') != 'vertex-loop-v1'
            or manifest.get('entity') != VERTEX_LOOP
            or set(manifest.get('profiles', [])) != set(ARTIFACT_BY_PROFILE)
            or len(manifest['profiles']) != len(ARTIFACT_BY_PROFILE)
            or set(manifest.get('artifacts', {})) != set(ARTIFACT_BY_PROFILE.values())):
        raise ValueError('unsupported standard-addition manifest; only vertex-loop-v1 is audited')
    data = copy.deepcopy(baseline)
    profiles = {p['kind']: p for p in data['profiles']}
    for kind, artifact_id in ARTIFACT_BY_PROFILE.items():
        artifact = manifest['artifacts'][artifact_id]
        if (set(artifact.get('clauses', {})) != {
                'LOOP', 'VERTEX_LOOP', 'ADVANCED_FACE.WR7', 'ADVANCED_FACE.WR8'}
                or not re.fullmatch('[0-9a-f]{64}', artifact.get('sha256', ''))
                or not artifact.get('source_url', '').startswith('https://')
                or not artifact.get('file')):
            raise ValueError('incomplete fixed-artifact provenance')
        for interval in artifact['clauses'].values():
            if (not isinstance(interval, list) or len(interval) != 2
                    or any(type(n) is not int or n <= 0 for n in interval)
                    or interval[1] < interval[0]):
                raise ValueError('invalid fixed-artifact clause lines')
        profile = profiles.get(kind)
        if profile is None or profile['schema_sha256'] != artifact['sha256']:
            raise ValueError(f'{kind}: addition is not bound to the C++ baseline source artifact')
        declarations = profile['declarations']
        names = {d['name']: d for d in declarations}
        if 'VERTEX_LOOP' in names:
            raise ValueError('C++ baseline already has VERTEX_LOOP; audit before changing provenance')
        if (profile['registered_types'] != [d['name'] for d in declarations]
                or [d['id'] for d in declarations] != list(range(len(declarations)))
                or 'LOOP' not in names or 'VERTEX' not in names):
            raise ValueError('unsupported C++ baseline declaration layout')
        identity = len(declarations)
        if identity >= min(profile['membership_capacity'], 256):
            raise ValueError('standard addition exceeds fixed membership capacity')
        # Fixed AP214/N2635/N916 VERTEX_LOOP has exactly LOOP as parent.
        # In particular there is no GRI ancestor and the reference type is
        # VERTEX; the stronger VP/CP condition belongs to ADVANCED_FACE.WR7.
        loop = names['LOOP']
        declaration = {
            'id': identity, 'name': 'VERTEX_LOOP', 'parents': ['LOOP'],
            'simple': True,
            'fields': [{'name': 'loop_vertex', 'type': 'VERTEX', 'kind': 'Reference',
                        'optional': False, 'element_kind': 'Reference', 'minimum': 0,
                        'maximum': None, 'unique': 'None', 'nested_aggregates': []}],
            'ancestors': sorted([*loop['ancestors'], identity]),
            'flattened': [*copy.deepcopy(loop['flattened']), {
                'declaring_type': 'VERTEX_LOOP', 'declaring_type_id': identity,
                'local_field_index': 0}],
        }
        declarations.append(declaration)
        profile['registered_types'].append('VERTEX_LOOP')
        profile['name_index'].append({'name': 'VERTEX_LOOP', 'id': identity})
        profile['name_index'].sort(key=lambda entry: entry['name'])
    return data


def generate(data, fingerprint, additions=None, extensions_fingerprint=None):
    assert data['format'] == 'step-catalog-snapshot.v3'
    if (additions is None) != (extensions_fingerprint is None):
        raise ValueError('addition content and separate fingerprint must be supplied together')
    if additions is not None:
        data = add_standard_vertex_loop(data, additions)
    out = ['// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.',
           '// SPDX-License-Identifier: Apache-2.0',
           '//! Generated transport metadata, not complete EXPRESS conformance.',
           '//! Upstream: `source/STEP_SEMANTICS/src/StepCatalog*.h` and `tests/catalog/StepCatalogSnapshot.cpp`.',
           '//! Unchanged C++ baseline descriptors plus separately identified standard additions.',
           '//! New `VERTEX_LOOP` descriptors, when enabled, do not originate in the C++ export.',
           '//! Deviations: checked Rust access; no runtime catalog initialization.',
           '// @generated by scripts/generate_step_catalog.py; edit the generator, not this file.',
           'use super::catalog::{Catalog, Declaration, Dimension, Field, Kind, Membership, Redeclaration, Slot, Unique};',
           f'pub(super) const SOURCE_SHA256: &str = "{fingerprint}";',
           'pub(super) const EXTENSIONS_SHA256: Option<&str> = ' +
           ('None;' if extensions_fingerprint is None else f'Some("{extensions_fingerprint}");'),
           '#[rustfmt::skip]', 'pub(super) const CATALOGS: &[Catalog] = &[']
    for p in data['profiles']:
        ids = {d['name']: d['id'] for d in p['declarations']}
        out.append('Catalog {')
        for name, value in [('kind', p['kind']), ('profile', p['profile']), ('schema_sha256', p['schema_sha256'])]:
            out.append(f'{name}: {string(value)},')
        out.append('declarations: &[')
        for d in p['declarations']:
            bits = [sum(1 << (a % 64) for a in d['ancestors'] if a // 64 == word) for word in range(4)]
            out.append('Declaration { name: %s, simple: %s, fields: %s, ancestors: Membership([%s]), flattened: %s },' % (
                string(d['name']), str(d['simple']).lower(), array(field(f) for f in d['fields']),
                ','.join(str(bit) for bit in bits), array('(%s,%s)' % (f['declaring_type_id'],f['local_field_index']) for f in d['flattened'])))
        out.append('],')
        out.append('name_index: '+array('(%s,%s)' % (string(v['name']),v['id']) for v in p['name_index'])+',')
        for output, input_name in [('selects','reference_selects'), ('enumerations','enumerations')]:
            out.append(output+': '+array('(%s,%s)' % (string(v['name']),array(string(b) for b in v['branches'])) for v in p[input_name])+',')
        out.append('complex: '+array(array(string(s) for s in v) for v in p['complex_signatures'])+',')
        out.append('measures: '+array(string(s) for s in p['measure_branches'])+',')
        out.append('derived: '+array('Slot { owner: %s, origin: %s, field: %s }' %
                   (ids[v['record_type']], ids[v['declared_in']], next(i for i,f in enumerate(p['declarations'][ids[v['declared_in']]]['fields']) if f['name']==v['field'])) for v in p['derived_slots'])+',')
        out.append('explicit: '+array('Redeclaration { slot: Slot { owner: %s, origin: %s, field: %s }, declared_type: %s, kind: Kind::%s }' %
                   (v['record_type_id'],v['declaring_type_id'],v['local_field_index'],string(v['type']),v['kind']) for v in p['explicit_redeclarations'])+',')
        out.append('exclusive: '+array(array(str(ids[s]) for s in v) for v in p['exclusive_groups'])+',')
        out.append('prefixes: '+array('(%s,%s)' % (string(v['name']), v['power']) for v in p['si_prefixes'])+',')
        out.append('},')
    out.append('];\n')
    return '\n'.join(out)


def canonical_addition(raw):
    """Stable metadata identity across Git LF/CRLF checkout conventions."""
    return raw.replace(b'\r\n', b'\n').rstrip(b'\r\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--additions',type=Path,help='explicit versioned standard additions; no C++ export mutation')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    raw=args.snapshot.read_bytes()
    # C++ console mode may translate only the final LF on Windows. The metadata
    # fingerprint follows the canonical payload, never the platform newline.
    canonical=raw.rstrip(b'\r\n')
    addition_bytes = None if args.additions is None else canonical_addition(args.additions.read_bytes())
    additions = None if addition_bytes is None else json.loads(addition_bytes)
    extensions_fingerprint = None if addition_bytes is None else hashlib.sha256(addition_bytes).hexdigest()
    generated=generate(json.loads(canonical),hashlib.sha256(canonical).hexdigest(),additions,extensions_fingerprint)
    if args.check:
        assert args.output.read_text(encoding='utf-8')==generated, 'generated catalog differs'
    else:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(generated,encoding='utf-8',newline='\n')


if __name__=='__main__':
    main()
