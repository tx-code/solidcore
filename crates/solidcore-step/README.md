# Rust STEP migration

This crate ports the native usd-brep STEP reader into safe Rust. The first stage
reads **source**, not accepted geometry. `Inspected` means syntax, HEADER shape,
instance identity and all DATA references passed. It does not mean an AP profile,
unit, surface, solid or modeling operation has passed. The JSON report explicitly
marks every later stage `NotRun` and repair `None`.

A separate [strict types/units checkpoint](TYPES_UNITS.md) now binds the five
fixed source catalogs and checks exact unit dependencies. The opt-in
`step-semantic-inspect` CLI can also check an explicitly selected physical unit
context. Source inspection remains unchanged; checkpoint success does not
establish part ownership, complete AP validity, geometry or a native model.

## Use

```sh
cargo run -p solidcore-step --features cli --bin step-inspect -- \
  --fixture-limits crates/solidcore-step/tests/fixtures/material-box.step
```

`--fixture-limits` explicitly selects a **synthetic test preset**, not production
quality or resource defaults. `--help` lists budget overrides. The source reader
is a normal Rust library, with no kernel, filesystem, FFI or global mutable state:

```rust
use solidcore_step::{inspect, Limits, Status};

let bytes = include_bytes!("tests/fixtures/plane.step").to_vec();
let outcome = inspect(bytes, Limits::planar_fixtures());
assert_eq!(outcome.status, Status::Inspected);
let source = outcome.source.as_ref().ok_or("source unavailable")?;
let entity = source.find("21").ok_or("entity unavailable")?;
assert_eq!(entity.components[0].name, "SHELL_BASED_SURFACE_MODEL");
# Ok::<(), Box<dyn std::error::Error>>(())
```

The public source fields preserve grammar values, not interpreted EXPRESS
attributes. No numeric lexeme is narrowed or rounded; `-0.`, exponent spelling,
units and orientation parameters remain original input values. Unknown entity
keywords are preserved, never claimed as supported geometry. Shared topology
and assembly relationships remain references, not duplicated geometry.

## Current contract

- Self-contained ASCII Part 21, corrected implementation declaration `2;1`;
  exactly one unparameterized DATA section and the three required HEADER records.
  Complex external mapping keeps all constituents in source order.
- Values: exact integer/real lexemes, strings, enumeration, references, `$`, `*`,
  aggregates and typed parameters. Positive decimal IDs are canonical strings;
  leading-zero collisions are rejected, including IDs too large for integers.
- Strings: doubled apostrophe/backslash, X/X2/X4 scalar escapes, Latin-1 PAGE
  and alphabet A. CR/LF/TAB layout is ignored inside strings/directives. Invalid
  scalars (including surrogate groups) fail, never undergo replacement decoding.
- Original bytes are owned by an immutable snapshot; byte spans are zero-based,
  half open. Diagnostic parameter paths are one-based; graph indices zero-based.
  Locations use one-based byte columns and count CRLF as one newline.
- Forward/cyclic raw references are allowed; duplicate or unresolved identities
  reject the complete request. Failure retains the complete captured bytes and
  SHA-256 when capture succeeded, with **no partial graph**. SHA-256 establishes
  content identity, not authenticity.
- The nine explicit limits match the C++ source contract, including independent
  ceilings and depth at most 128. No sparse allocation from IDs or file counts.
  This bounds counted resources; it is not a total-process memory guarantee or
  hard cancellation/isolation mechanism.
- `report::compile` borrows the graph, produces deterministic versioned JSON and
  checks actual UTF-8 output bytes, including escapes and final LF, before returning
  anything. It does not stream partial JSON on a limit failure. The report retains
  original source text, so private inputs produce private reports.
- The CLI reads at most the input budget plus one byte. Exits: 0 inspected,
  1 rejected, 2 unsupported, 3 resource limit, 4 capture I/O failure, 5 report limit,
  6 report infrastructure/output failure, 64 invalid arguments. I/O errors go to
  stderr without a source report. OS output failures may leave a partial external
  write; only successful exit establishes complete stdout publication.

The JSON schema is `solidcore.step.source.v1`, intentionally distinct from C++
`step-source.v1`. Source fields correspond, but JSON bytes are not promised equal.
The complete original source is `raw`; spans address it directly. The inspector
never infers units from names, promotes a closed surface to a material solid,
heals source defects, or relaxes quality thresholds.

## Deliberately not migrated yet

| Stage | Exit gate before claiming the capability |
|---|---|
| Explicit carrier options | Opaque HEADER/DATA and legacy level-1 interpretation retain source diagnostics/receipts; strict behavior remains unchanged |
| Full semantic qualification | Strict type/unit checkpoint is implemented; selected-part ownership, representation-use constraints and interoperability proofs remain separate |
| Planar qualification | Stable rule identities/order, budgets, oriented regions, shared edges, holes and source surface/solid roles pass the analytic fixtures |
| Native adapter | Validate solidcore geometry/topology construction and failed-candidate cleanup; retain source-to-native correspondence and UV definitions |
| General exact BRep | Analytic/NURBS, trims, pcurves, seams, degeneracies and assemblies meet their individual error and ownership contracts |
| Publication and repair | Complete owning read-only result plus diagnostics, cancellation/commit boundary; optional repair remains separately budgeted |

Source parsing must land before typed binding; typed binding before construction.
Neither a mesh nor a successful kernel method qualifies the intervening stages.
The new crate does not depend on the kernel, so importer edits do not rebuild it.
Kernel arena APIs exist, but their existence does not establish a verified STEP
construction path. The C++ kernel's custom model/translator APIs must not be
assumed to exist or have the same ownership rules in Rust.

## Validation

```sh
cargo fmt -p solidcore-step --check
cargo clippy -p solidcore-step --all-features --all-targets -- -D warnings
cargo test -p solidcore-step --all-features
cargo check -p solidcore-step --lib --target wasm32-unknown-unknown
```

Tests use synthetic truth for exact values, graph paths, duplicate positions,
Unicode, boundaries, immutable ownership, malformed/truncated input and mutation
robustness. The plane/material-box/surface-box fixtures retain their distinct
source roles. The library builds for wasm; the CLI is opt-in for host systems.

For C++ compatibility, install the standalone `STEP_SOURCE` inspector from the
STEP fork, build this CLI, then run from the repository root:

```sh
python scripts/step_source_compare.py --cpp /path/to/step_inspect \
  --rust bin/debug/step-inspect \
  --corpus /path/to/ap203e2-planar/v1/cases
```

The optional corpus is the original 49 public synthetic cases. The script also
generates malformed/string/sparse/cyclic/truncation inputs and N/N-1 budget cases.
It compares exit/status, original SHA-256, complete HEADER/DATA/value/constituent
spans, reference paths/targets, resource accounting and diagnostic identities,
stages, owners, paths and positions. It independently checks raw-byte coverage;
human message wording and the versioned JSON byte layout are intentionally not
compared. A mismatch fails the run, never changes a tolerance. Per-run receipts
stay local; the script prints only aggregate counts. Same-source C++ parity is
compatibility evidence, not independent geometric truth.

Provenance: [ATTRIBUTION.md](../../ATTRIBUTION.md), source snapshot
`d8ed45ab85d52dba07660cafdceacafdca7217ed`. Standards-facing behavior follows the
owning `STEP_SOURCE/README.md` carrier contract; this migration does not claim
full ISO 10303-21 conformance or AP schema validity.
