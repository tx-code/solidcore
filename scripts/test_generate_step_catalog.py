#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Transport/provenance unit controls; no CAD input or C++ observation is truth."""
import copy
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('catalog_generator', ROOT / 'scripts/generate_step_catalog.py')
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)
MANIFEST_BYTES = (ROOT / 'crates/solidcore-step/catalog/standard-additions.v1.json').read_bytes().replace(b'\r\n', b'\n').rstrip(b'\r\n')
MANIFEST = json.loads(MANIFEST_BYTES)


def small_export():
    """A public test-owned transport export, not a source-schema oracle."""
    text = {'name': 'name', 'type': 'LABEL', 'kind': 'String', 'optional': False,
            'element_kind': 'Reference', 'minimum': 0, 'maximum': None,
            'unique': 'None', 'nested_aggregates': []}
    names = ['REPRESENTATION_ITEM', 'GEOMETRIC_REPRESENTATION_ITEM',
             'TOPOLOGICAL_REPRESENTATION_ITEM', 'LOOP', 'VERTEX']
    ancestors = [[0], [0, 1], [0, 2], [0, 2, 3], [0, 2, 4]]
    parents = [[], ['REPRESENTATION_ITEM'], ['REPRESENTATION_ITEM'],
               ['TOPOLOGICAL_REPRESENTATION_ITEM'], ['TOPOLOGICAL_REPRESENTATION_ITEM']]
    declarations = [
        {'id': i, 'name': name, 'parents': parents[i], 'simple': False,
         'fields': [copy.deepcopy(text)] if i == 0 else [], 'ancestors': ancestors[i],
         'flattened': [{'declaring_type': 'REPRESENTATION_ITEM',
                        'declaring_type_id': 0, 'local_field_index': 0}]}
        for i, name in enumerate(names)]
    profiles = []
    for kind in ['Ap203e2Planar', *GENERATOR.ARTIFACT_BY_PROFILE]:
        artifact = GENERATOR.ARTIFACT_BY_PROFILE.get(kind, 'N2635')
        profiles.append({
            'kind': kind, 'profile': f'{kind}-test',
            'schema_sha256': MANIFEST['artifacts'][artifact]['sha256'],
            'membership_capacity': 256, 'registered_types': names.copy(),
            'name_index': sorted([{'name': n, 'id': i} for i, n in enumerate(names)],
                                 key=lambda entry: entry['name']),
            'declarations': copy.deepcopy(declarations), 'reference_selects': [],
            'enumerations': [], 'complex_signatures': [], 'measure_branches': [],
            'derived_slots': [], 'explicit_redeclarations': [], 'exclusive_groups': [],
            'si_prefixes': []})
    return {'format': 'step-catalog-snapshot.v3', 'profiles': profiles}


class StandardAdditions(unittest.TestCase):
    def test_appends_one_identity_without_mutating_export_or_p0(self):
        baseline = small_export()
        before = copy.deepcopy(baseline)
        expanded = GENERATOR.add_standard_vertex_loop(baseline, MANIFEST)
        self.assertEqual(baseline, before)
        for old, new in zip(before['profiles'], expanded['profiles']):
            if old['kind'] == 'Ap203e2Planar':
                self.assertEqual(old, new)
                continue
            count = len(old['declarations'])
            self.assertEqual(new['declarations'][:count], old['declarations'])
            self.assertEqual(new['registered_types'][:count], old['registered_types'])
            self.assertEqual(len(new['declarations']), count + 1)
            added = new['declarations'][-1]
            self.assertEqual(added['id'], count)
            self.assertEqual(added['parents'], ['LOOP'])
            self.assertEqual(added['ancestors'], [0, 2, 3, count])
            self.assertNotIn(1, added['ancestors'])  # GRI is NOT an ancestor.
            self.assertEqual(added['fields'][0]['type'], 'VERTEX')
            self.assertFalse(added['fields'][0]['optional'])
            self.assertEqual(added['flattened'], [
                {'declaring_type': 'REPRESENTATION_ITEM', 'declaring_type_id': 0, 'local_field_index': 0},
                {'declaring_type': 'VERTEX_LOOP', 'declaring_type_id': count, 'local_field_index': 0}])
            for old_entry in old['name_index']:
                self.assertIn(old_entry, new['name_index'])
            self.assertEqual(new['name_index'], sorted(new['name_index'], key=lambda entry: entry['name']))
            for key in set(old) - {'declarations', 'registered_types', 'name_index'}:
                self.assertEqual(old[key], new[key])

    def test_separate_identity_and_baseline_only_none(self):
        baseline = small_export()
        base_sha = hashlib.sha256(json.dumps(baseline).encode()).hexdigest()
        baseline_only = GENERATOR.generate(baseline, base_sha)
        self.assertIn(f'SOURCE_SHA256: &str = "{base_sha}";', baseline_only)
        self.assertIn('EXTENSIONS_SHA256: Option<&str> = None;', baseline_only)
        self.assertNotIn('name: "VERTEX_LOOP"', baseline_only)
        extra_sha = hashlib.sha256(MANIFEST_BYTES).hexdigest()
        expanded = GENERATOR.generate(baseline, base_sha, MANIFEST, extra_sha)
        self.assertIn(f'SOURCE_SHA256: &str = "{base_sha}";', expanded)
        self.assertIn(f'EXTENSIONS_SHA256: Option<&str> = Some("{extra_sha}");', expanded)
        self.assertEqual(expanded.count('Declaration { name: "VERTEX_LOOP"'), 4)
        self.assertIn('do not originate in the C++ export', expanded)
        with self.assertRaises(ValueError):
            GENERATOR.generate(baseline, base_sha, MANIFEST)

    def test_cli_addition_identity_is_independent_of_checkout_newlines(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            snapshot = directory / 'snapshot.json'
            snapshot.write_text(json.dumps(small_export()), encoding='utf-8')
            outputs = []
            for label, data in [('lf', MANIFEST_BYTES + b'\n'),
                                ('crlf', (MANIFEST_BYTES + b'\n').replace(b'\n', b'\r\n'))]:
                addition = directory / f'{label}.json'
                output = directory / f'{label}.rs'
                addition.write_bytes(data)
                command = [sys.executable, str(ROOT / 'scripts/generate_step_catalog.py'),
                           '--snapshot', str(snapshot), '--additions', str(addition),
                           '--output', str(output)]
                completed = subprocess.run(command, capture_output=True, text=True, check=False)
                self.assertEqual(completed.returncode, 0, completed.stderr)
                outputs.append(output.read_bytes())
            self.assertEqual(outputs[0], outputs[1])

    def test_rejects_unverified_scope_fields_artifacts_and_existing_entity(self):
        changes = [
            lambda m: m.update(format='other'),
            lambda m: m['profiles'].__setitem__(0, 'Ap203e2Planar'),
            lambda m: m['profiles'].append('Ap203N916Source'),
            lambda m: m['entity'].update(name='POLY_LOOP'),
            lambda m: m['entity']['field'].update(type='VERTEX_POINT'),
            lambda m: m['artifacts']['N916Modified'].update(sha256='0' * 64),
            lambda m: m['artifacts']['N2635']['clauses'].pop('ADVANCED_FACE.WR7'),
            lambda m: m['artifacts']['AP214E3_2010']['clauses'].update(LOOP=[0, 1]),
        ]
        for change in changes:
            manifest = copy.deepcopy(MANIFEST)
            change(manifest)
            with self.assertRaises(ValueError):
                GENERATOR.add_standard_vertex_loop(small_export(), manifest)
        expanded = GENERATOR.add_standard_vertex_loop(small_export(), MANIFEST)
        with self.assertRaises(ValueError):
            GENERATOR.add_standard_vertex_loop(expanded, MANIFEST)

    def test_artifact_binding_and_capacity_are_checked_before_publication(self):
        for change in [lambda p: p.update(schema_sha256='0' * 64),
                       lambda p: p.update(membership_capacity=len(p['declarations']))]:
            baseline = small_export()
            change(baseline['profiles'][-1])
            before = copy.deepcopy(baseline)
            with self.assertRaises(ValueError):
                GENERATOR.add_standard_vertex_loop(baseline, MANIFEST)
            self.assertEqual(baseline, before)


if __name__ == '__main__':
    unittest.main()
