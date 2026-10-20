# Strict source types and units

This document describes the unchanged default checkpoint. The new opt-in
[part/closure checkpoint](PART_CLOSURE.md) extends it; it does not silently
run ownership or infer a part when this earlier checkpoint is requested.

This checkpoint migrates the C++ `STEP_SEMANTICS` Binding, Mapping,
CheckForeignDeclarations, Fields, UnitDependencies and optional UnitContext
contracts. It preserves their diagnostic identities, classifications, spans,
owners, parameter paths, target/related positions, declared fields, expected and
actual values, responsibility labels, phase order and actual visit accounting.
Human message prose and the versioned JSON layout are not compatibility keys.

It is **not the full C++ semantic pipeline**. Product/selected-part ownership,
physical closure, geometry dependencies, representation-use constraints (including
VRI context WR1), planar/global rules, interoperability approval and native
construction are not run. Reports label those stages `NotRun`; no geometry is
constructed and no source error is repaired or suppressed. An orphan geometric
item can pass this checkpoint without qualifying a complete import.

## Library and CLI

```sh
cargo run -p solidcore-step --features cli --bin step-semantic-inspect -- \
  --fixture-limits --unit-context 12 tests/fixtures/plane.step
```

Run the command from this crate directory, or use the corresponding repository
relative fixture path. The library entrypoint is `semantics::check(&source,
semantics::Limits::fixtures(), &options)`. Source capture retains its own explicit
nine budgets. The semantic test preset permits one million dependency visits;
the independent hard ceiling is ten million. These are resource presets, not
production quality settings or precision tolerances.

`Options.geometry_scope` / `--general-brep-source` uses the same profile selection
policy as C++. It admits the N2635 general catalog or the separately identified
bare CONFIG_CONTROL_DESIGN/N916 source subset. It does not authenticate an edition
or silently fall back after a check fails. AP214's unknown bare/eight-arc
interpretation policies and legacy carrier/opaque-string options are not migrated;
their CLI flags are refused. This checkpoint is strict-only.

`Options.physical_context` / `--unit-context ID` is an explicit canonical source
instance ID. Its physical UnitContext contract checks dimension_count, explicit
unit assignment, nominal counts, SI name/quantity pairs and exact SI metadata.
The caller must later establish that context's selected-part ownership. With no
context selected, `si_units=NotRun`; no root, context or unit is guessed. An absent
or nongeometric context selection is `InvalidRequest`, not a source-invalid error.
Length conversion-based contexts retain `Deferred`, never a guessed mm scale.

The CLI requires `--fixture-limits`; additional resource flags are listed by
`--help`. Exit codes follow source inspection: 0 checked/deferred checkpoint,
1 rejected/source failure, 2 unsupported, 3 resource limit, 4 capture I/O failure,
5 report budget, 6 infrastructure/output failure, 64 invalid arguments/context.
`Deferred` is visible in JSON and is not an import success. No partial JSON is
published on report-budget failure. OS write failures can leave a partial stdout
write; only successful exit establishes complete publication.

## Fixed profile boundaries

| Implemented source catalog | Selection | Declaration count |
|---|---|---:|
| `203e2-P0-Planar-0.1` | Exact N2635 AP203e2 identifier, default scope | 72 |
| `203e2-G0-Source-0.1` | Same identity, explicit general scope | 139 |
| `214e3-P0-Source-0.1` | Registered AP214 seven-arc edition-3 identifier | 136 |
| `214-MBx2010-Source-0.1` | Separately registered MBx eight-arc identifier | 151 |
| `203-N916-Source-0.1` | Bare CONFIG_CONTROL_DESIGN, explicit general scope | 139 |

Names, OID arcs, HEADER order and spans remain original. Known AP242 or other
registered protocol identities do not select an unimplemented catalog. The N916
and MBx foreign-declaration guards stay strict; borrowed diagnostic descriptors
do not admit foreign source entities. No APD/year inference or filename rule is
used. Catalog registration and a type population alone do not prove AP validity.

All source type IDs and registry order are copied from the complete C++ descriptor
export. The arrays are immutable compile-time Rust data, with checked lookup and
no startup allocation/global cache. `scripts/generate_step_catalog.py` regenerates
them offline from `step_catalog_snapshot` output and supports `--check`. No
generated C++ or external schema is part of the Rust build. The generator's
fingerprint covers the canonical JSON payload excluding its final line ending.

## Values, fields and ownership

- Simple and complex mapping retain inherited slot origins, explicit narrowing,
  DERIVE marker identity, complete constituent order and mutually exclusive
  branches. Missing/duplicate/unsorted facets are diagnosed, never reordered.
- Required `$` differs from an empty STRING; INTEGER differs from REAL except at
  an explicitly declared NUMBER. BOOLEAN differs from LOGICAL unknown U. SELECT
  references, measure tags and nested LIST/SET dimensions remain source-specific.
- Nested grids stay nested. LIST occurrences retain order/repetition; SET/UNIQUE
  identity and exact mixed trimming SELECT equality follow the C++ contract.
- `Snapshot` and `FieldView` retain the original source through immutable shared
  ownership. Fields keep source component/parameter indices and borrow actual
  original values. No parallel entity/value graph is rebuilt. A field survives
  dropping its original outcome. Unknown type lookup differs from a proved empty
  population; missing/ambiguous field lookup returns no view.
- Semantic reports are bound to the original capture owner. Accidentally combining
  another source request with a checkpoint is `SourceMismatch`, even if the two
  input hashes happen to match. The shared bounded writer compiles the entire
  report (including original source and final LF) before returning it. Private
  inputs therefore produce private reports and remain local.

## Unit prerequisites

The P0 catalog retains C++'s intentional `unit_dependencies=NotRun`; its optional
SI context contract still runs when requested. The other four profiles execute
the existing exact dimensional pass, preserving its counters and deterministic
iterative DFS order. SI/CBU/derived-unit edges include zero-power uses; a zero
coefficient never erases a cycle or a mass SI-prefix obligation.

Dimensions are reduced rational exponents in L/M/T/I/temperature/amount/luminous
order. Checked arithmetic retains the same signed-i64 capacity on native and
wasm. It cross-cancels before multiplication and reduces decimal coefficients
before enforcing capacity. A capacity stop is `ResourceLimit`, not evidence of
bad source or permission to round. Exact decimal comparison preserves signed zero
and arbitrarily long exponents without binary floats or power expansion.

Source nominal quantity remains distinct from equal dimension vectors. The pass
checks the existing unit/measure prerequisites, positive and nonnegative measure
TYPE predicates, conversion dimension coherence and applicable positive-scale
guards. N916 project prerequisites remain `ProfileUnsupported` where the fixed
artifact has no corresponding WHERE clause. Generic measure dimensions do not
inherit nominal constraints from a more specific MWU subtype.

SI metadata uses exact powers to mm/rad/sr/kg: e.g. source GRAM remains GRAM and
its original prefix remains intact. Coordinates, parameter values and source
uncertainty are never scaled or rewritten here. Property/value items do not
inherit the selected body's units. Context uses, ownership and physical exclusion
still need their later checks; strict errors cannot use interoperability exceptions.

## C++ comparison and independent truth

The local test adapter `scripts/build_step_type_unit_reference.py` copies
`StepSemantics.cpp` into an ignored output directory, changes exactly its one
Checker access label from private to public, then appends
`scripts/step_type_unit_reference.cpp`. **Checker bodies, catalogs, failure paths
and counters are unchanged**. It invokes only the phase methods named above,
with the same optional context. The original C++ checkout is never modified.
This avoids confusing failures in not-yet-migrated ownership/geometry phases with
the requested type/unit checkpoint. It is not a second expected-answer generator.

Build the ordinary C++ source/semantic libraries first; then, on MSVC:

```sh
python scripts/build_step_type_unit_reference.py --source /path/to/step-fork \
  --library-dir /path/to/source-semantics/Release --output bin/type-unit-reference
python scripts/step_type_unit_compare.py \
  --cpp bin/type-unit-reference/step-type-unit-reference.exe \
  --rust bin/debug/step-semantic-inspect \
  --corpus /path/to/ap203e2-planar/v1/cases --catalog /path/to/catalog-snapshot.json
```

Run from the repository root. `--toolchain-env` optionally supplies an explicitly
selected local allowlisted toolchain environment; credentials are not read.
The comparison covers all 49 original public synthetic cases, every catalog
declaration and admitted complex signature, unit dimensions/cycles/capacity,
schema recognition and exact visit boundaries. Report status, typed slots and
memberships, original source hash/text, all stable diagnostic fields, unit facts
and counters must agree. JSON-only representation differences (null/empty registry
absence, range object/array, profile wrapper keys) are mapped explicitly.

Rust tests independently assert analytic dimensional vectors/fractions, SI powers,
original scalar kinds/lexemes, source positions and owning-view lifetime. Their
expected values do not come from C++. Source-parser regression remains separately
checked. These checks establish compatibility and the listed synthetic scope,
not full AP conformance or geometric import qualification. Per-run receipts and
any private model evidence remain outside Git.
