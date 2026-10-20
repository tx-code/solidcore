# Attribution

solidcore is a Rust port of **SMLib**, the NURBS B-rep solid modeling kernel by
NVIDIA, published as part of USD BRep:

- Repository: <https://github.com/NVIDIA-Omniverse/usd-brep>
- Pinned commit: `c9c5979`
- Licence: Apache License, Version 2.0 (see [`LICENSE`](LICENSE), copied verbatim
  from upstream, and [`NOTICE`](NOTICE))
- Copyright: (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES

solidcore is an independent project. It is not affiliated with or endorsed by
NVIDIA, and "SMLib" and "NVIDIA" are used here only to identify the origin of the
code, not as branding.

## Upstream material used

### STEP migration in this fork

`crates/solidcore-step` additionally ports the independently developed native
STEP reader from the usd-brep STEP fork, source snapshot
`d8ed45ab85d52dba07660cafdceacafdca7217ed` (separate from the kernel pin above).
References are `source/STEP_SOURCE/inc/StepSource.h`,
`source/STEP_SOURCE/src/{StepSource,StepInspectJson,StepSourceReport}.cpp` and
`source/STEP_SOURCE/app/step_inspect.cpp`, under Apache-2.0 with their NVIDIA
copyright notices retained in the ported files.

The strict types/units checkpoint additionally ports
`source/STEP_SEMANTICS/inc/StepSemantics.h` and the selected
Binding/Mapping/Fields/UnitContext methods in `StepSemantics.cpp`, plus
`StepSchemaSelection.cpp`, `StepUnitDependencies.cpp`, `StepUnitRational.cpp`,
`StepSourceNumber.cpp` and the immutable catalog query contracts. The generated
Rust descriptors come from the public `StepCatalogSnapshot.cpp` export of all
five fixed catalogs. No EXPRESS artifact, C++ product reader or schema generator
is needed to build or run Rust. The generated file records the exact descriptor
payload fingerprint; it is a porting identity, not an independent schema witness.

The optional ownership checkpoint also ports the strict `Product`, `Closure`,
`Reach` and associated property/presentation/administrative predicates in
`StepSemantics.cpp`, plus source inverse/use queries. Public synthetic tests
under `tools/step_semantic_validation/tests` may be exercised by the local input
capture adapter; their original C++ observations/assertions are not changed or
used as Rust expected values. Their Python product/evidence implementation is
not a Rust runtime dependency or a Python translator port. Captures remain local.

The fixed source geometry prerequisite pass ports
`source/STEP_SEMANTICS/src/StepGeometryDependencies.cpp` from the same STEP fork
snapshot. Its spline/weight constraints, curve graphs and representation member
predicates retain the fixed artifact responsibilities in source diagnostics.
The expanded complete-fork goal also authorizes maintained SDK/binding/tool
references as recorded in `docs/FORK_MIGRATION.md`; those layers remain pending
until their real implementations and parity evidence exist.

The three STEP inputs under `crates/solidcore-step/tests/fixtures` are public
analytic design fixtures S203-P01, S203-P04 and S203-P05 from the STEP fork's
`tools/step_import_validation/corpus/ap203e2-planar/v1/cases`; they contain no
private/customer geometry. Other tests generate synthetic source text. This
exception covers these explicit test inputs, not the upstream Python tools.
No OCCT, Truck/ruststep or C++ reader is linked or invoked by the Rust product.
The optional comparison script invokes an installed C++ source reader only as a
test oracle. See the [migration contract](crates/solidcore-step/README.md).

### Kernel port

Only these upstream paths are read, ported or exercised:

| Upstream path                  | Use                                                    |
|--------------------------------|--------------------------------------------------------|
| `source/SMLib/`                | Kernel being ported (geometry, topology, algorithms)   |
| `source/SM_API/`               | High-level API being ported; reference for public ops  |
| `tests/SM_API_test/`           | Reference tests; source of difftest cases              |
| `tests/prog_test/`             | Reference tests; source of difftest cases              |
| `TestFiles/pt_TestFiles/`      | Test fixtures                                          |
| `AGENTS.md`, `.agents/`        | Upstream developer documentation (OCCT/USD files excluded) |
| `LICENSE`, `THIRD_PARTY_NOTICES.md` | Licence text and notices                          |

The C++ reference build (`harness/cpp`) compiles only `source/SMLib` and
`source/SM_API`, from a local clone that is never committed (`upstream/` is
gitignored).

## Upstream material not used

The following upstream layers are **not** read, ported, built or linked:

- OCCT translation: `source/OCCT_BREP_IMPORT`, `source/OCCT_BREP_EXPORT`,
  `TestFiles/occt_breps*`
- USD / Hydra integration: `source/SM_API_USD`, `source/BREP_SM_USD`,
  `source/BREP_USD_DATA`, `source/HYDRA_USD_BREP`, `source/USD_BREP_UTILS`,
  `source/schema`, `TestFiles/usd_TestFiles`
- Python bindings and tooling: `source/SmPyLib`, `source/SmPyDevLib`, `tools/`
- Proposals: `proposals/`

No GPL/LGPL code is used anywhere in solidcore.

## Per-file provenance

Every ported Rust file begins with a header naming the upstream source file(s) it
derives from and a table mapping its Rust items to upstream names (see
[`AGENTS.md`](https://github.com/tangency-labs/solidcore/blob/main/AGENTS.md#file-header)).

## Test corpora

These are fetched at run time (never committed) and used only by difftest's
`--ssi-corpus` correctness report (`crates/difftest/README.md`).

### ssi-corpus

- Repository: <https://github.com/jyviko/ssi-corpus> (fetched by
  `scripts/fetch-ssi-corpus.sh` into the gitignored `corpus/ssi/`)
- Pinned commit: `b04c8a6`
- Author: Iraklis Kourtis (jyviko), published for solidcore in
  [issue #17](https://github.com/tangency-labs/solidcore/issues/17)
- Licence: MIT OR Apache-2.0
- Copyright: (c) 2026 SolidCraft contributors
- Use: 119 surface/surface intersection cases with exact ground truth. The scorer in
  `crates/difftest/src/ssi/` (`score.rs`, `polyset.rs`, `geom.rs`) is ported from the
  corpus' `scoring/score.py` and `scoring/ssi_geom.py`, and the C++ shim
  `harness/cpp/shim/ssi_corpus.cpp` mirrors its `runner/ssi_runner.cpp`; each of those
  files says so in its header. The truth generators are not ported.

### Yang, Jia & Yan (2023)

- Yang, Jia and Yan, "Topology Guaranteed B-Spline Surface/Surface Intersection",
  *ACM Transactions on Graphics* 42(6), Article 211, 2023. Licence: CC-BY.
- Use: the corpus' 25 `y23_*` cases come from this paper (its published data, or surface
  pairs reconstructed from its figures); each such case cites it in its `source` field.
