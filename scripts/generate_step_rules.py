#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Generate immutable rule metadata/plans from the fixed public C++ catalog.

This imports declaration facts, never executable predicates or expected test
answers. Rust evaluation dispatches exhaustively on named identities. No network.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re


def block(source, marker):
    return source.split(marker, 1)[1].split('\n};', 1)[0]


def generate(raw):
    source = raw.decode('utf-8').replace('\r\n', '\n')
    local_ids = re.findall(r'(\w+)\s*=\s*\d+', block(source, 'enum class LocalRuleId'))
    global_ids = re.findall(r'(\w+)\s*=\s*\d+', block(source, 'enum class GlobalRuleId'))
    local = re.findall(r'\{\s*LocalRuleId::(\w+),\s*"([^"]+)",\s*"([^"]+)",\s*"([^"]+)",\s*(\d+),\s*LocalSubject::(\w+),\s*"([^"]*)"\s*\}', block(source, 'inline constexpr Clause LocalRules[] = {'))
    order = re.findall(r'LocalRuleId::(\w+)', block(source, 'inline constexpr std::array EntityLocalExecutionOrder = {'))
    globals_ = re.findall(r'\{\s*GlobalRuleId::(\w+),\s*"([^"]+)",\s*(\d+),\s*(For\w+)(?:,\s*"([^"]*)",\s*"([^"]*)")?\s*\}', block(source, 'inline constexpr Global Globals[] = {'))
    global_order = re.findall(r'GlobalRuleId::(\w+)', block(source, 'inline constexpr std::array GlobalExecutionOrder = {'))
    for_sets = {name: re.findall(r'"([^"]+)"', text) for name, text in re.findall(r'inline constexpr std::string_view (For\w+)\[\] = \{(.*?)\n\};', source, re.S)}
    assert len(local_ids) == len(local) == 80 and len(order) == 77
    assert len(global_ids) == len(globals_) == len(global_order) == 47
    assert set(local_ids) == {row[0] for row in local}
    assert set(global_ids) == {row[0] for row in globals_} == set(global_order)
    assert len(set(order)) == len(order) and len(set(global_order)) == len(global_order)
    assert {row[0] for row in local if row[5] == 'EntityPopulation'} == set(order)
    assert len({row[1:4] for row in local}) == 80
    text = ['// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.',
            '// SPDX-License-Identifier: Apache-2.0',
            '//! Fixed N2635 clause facts and identity-based execution plans; generated offline.',
            '//! Upstream: `source/STEP_PLANAR/src/StepRuleCatalog.h` at `d8ed45a`.',
            '//! Deviations: exhaustive Rust matches, compile-time registry checks; IDs are never serialized.',
            f'pub(super) const CATALOG_SHA256: &str = "{hashlib.sha256(raw).hexdigest()}";']
    for name, ids in [('LocalId', local_ids), ('GlobalId', global_ids)]:
        text += ['#[derive(Clone, Copy, Debug, Eq, PartialEq)]', f'pub(super) enum {name} {{', *[f'    {v},' for v in ids], '}']
    text += ['#[derive(Clone, Copy, Debug, Eq, PartialEq)]', 'pub(super) enum Subject { EntityPopulation, MeasureValue, ContextAttribute }',
             '#[derive(Clone, Copy)]', "pub(super) struct Clause { pub id: LocalId, pub type_name: &'static str, pub kind: &'static str, pub label: &'static str, pub line: usize, pub subject: Subject, pub failure_field: &'static str }",
             'pub(super) const fn local(id: LocalId) -> Clause { match id {']
    q = json.dumps
    for identity, type_name, kind, label, line, subject, field in local:
        text.append(f'LocalId::{identity} => Clause {{ id, type_name: {q(type_name)}, kind: {q(kind)}, label: {q(label)}, line: {line}, subject: Subject::{subject}, failure_field: {q(field)} }},')
    text += ['} }', '#[derive(Clone, Copy)]', "pub(super) struct Global { pub id: GlobalId, pub name: &'static str, pub line: usize, pub types: &'static [&'static str], pub failure_role: &'static str, pub failure_field: &'static str }", 'pub(super) const fn global(id: GlobalId) -> Global { match id {']
    for identity, name, line, for_set, role, field in globals_:
        names = ','.join(q(v) for v in for_sets[for_set])
        text.append(f'GlobalId::{identity} => Global {{ id, name: {q(name)}, line: {line}, types: &[{names}], failure_role: {q(role or "representation")}, failure_field: {q(field)} }},')
    text += ['} }']
    for name, kind, ids in [('ALL_LOCAL', 'LocalId', local_ids), ('ENTITY_ORDER', 'LocalId', order), ('ALL_GLOBAL', 'GlobalId', global_ids), ('GLOBAL_ORDER', 'GlobalId', global_order)]:
        text.append(f'pub(super) const {name}: &[{kind}] = &[{",".join(kind+"::"+v for v in ids)}];')
    text += ['pub(super) const DIMENSION: LocalId = LocalId::DimensionCountWr1;',
             'pub(super) const LENGTH_PLANS: &[(&str, &[LocalId])] = &[("LENGTH_MEASURE", &[]), ("NON_NEGATIVE_LENGTH_MEASURE", &[LocalId::NonNegativeLengthMeasureWr1]), ("POSITIVE_LENGTH_MEASURE", &[LocalId::NonNegativeLengthMeasureWr1, LocalId::PositiveLengthMeasureWr1])];',
             'pub(super) fn length_predicates(name: &str) -> &\'static [LocalId] { LENGTH_PLANS.iter().find(|plan| plan.0 == name).map_or(&[], |plan| plan.1) }',
             'const _: () = assert!(super::registry::valid());']
    return '\n'.join(text) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    rendered = generate(args.source.read_bytes())
    if args.check:
        assert args.output.read_text(encoding='utf-8') == rendered, 'rule facts/plans differ from fixed C++ source'
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
