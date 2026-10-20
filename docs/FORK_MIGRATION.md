# Complete C++ fork migration

The active objective is **move every implementation/capability in the maintained
C++ fork into solidcore**. STEP source checkpoints are increments toward that
objective, not a replacement objective or completion criterion. The migration
includes the kernel, public API, STEP pipeline, bindings, viewer/workbench and
supporting build/validation/distribution tools. Preserve caller-visible behavior,
source semantics, numerical/ownership contracts and actual failure behavior.

## Authoritative source and evidence

The maintained usd-brep fork is the source, not just NVIDIA's kernel pin
`c9c5979f04cea8c38213004d4f04df0dcfb40d05`. Check its live checkout and delivered
worktrees before assuming a capability or delta is absent. The STEP contract
reference used so far is `d8ed45ab85d52dba07660cafdceacafdca7217ed`. The delivered
associated-geometry viewer additionally exists at `200aa2f`; separate face/edge/
vertex hover work remains uncommitted. Preserve and account for that work,
distinguishing delivered behavior from incomplete experiments. Do not overwrite
the C++ working changes or claim their unvalidated behavior as a parity oracle.

solidcore's own `plan/modules.toml`, live `Unported` sites and difftest evidence
remain authoritative for the base kernel port. A partial unit satisfies only
dependency scheduling: its deferred calls are still missing implementation.
The C++ fork's own changes must be audited against the kernel pin and mapped to
the corresponding Rust owners; upstream base-port progress cannot prove them.
Keep per-run source inventories, receipts and private data in ignored local output.

## Full coverage ledger

| C++ owner | Required solidcore end state | Evidence needed for completion |
|---|---|---|
| `source/SMLib` | Analytic/NURBS geometry, topology, queries, modeling, healing, tessellation and numerical solvers, including all fork deltas | Every source capability mapped to actual Rust execution; base-unit deferred/todo calls and fork-delta gaps resolved; numerical/topological contracts and adversarial cases verified |
| `source/SM_API` | Usable public Rust operations and required compatibility interfaces with the original ownership/status/query contracts | Every maintained entrypoint accounted for; operation/error/cleanup/mutation tests and realistic end-to-end use |
| `source/STEP_SOURCE` | All maintained carrier/encoding/interpretation options, lossless values, budgets and receipts | Existing C++ source contracts plus independent malformed/Unicode/resource truth; strict behavior unchanged |
| `source/STEP_SEMANTICS` | All maintained profiles, fields, necessary predicates, selected-part closure, schema/carrier assumptions and nongeometric interoperability proofs | Phase/order/budget/diagnostic parity, retained original semantics and explicit qualification boundaries |
| `source/STEP_PLANAR` | Every registered local/global rule, oriented planar region/metric check and its evidence policy | Rule identity/order/cost parity and the full synthetic correctness matrix; no inferred continuous proof from samples |
| `source/STEP_IMPORT` | Exact planar and general construction, curves/surfaces/UV/seams/degeneracies, correspondence, queries, snapshots, archives, cancellation/publication, repairs and CLIs | Working Rust-kernel construction with complete cleanup and representation/quality contracts; analytic and independent comparisons; no parser/mesh-only substitute |
| `source/SmPyLib`, `source/SmPyDevLib`, package tooling | Stable and developer APIs with the maintained source functionality over the Rust implementation | Export/stub/behavior inventories, lifetime/mutation/error tests, packaged clean-environment operation; renaming/removal does not count as migration |
| `tools/smlib_viewer`, `tools/smlib_gui` and GUI entrypoints | CLI-launched standalone viewing and workbench behavior, source roles, progress, HUD, rendering, selection/measurements and supported file workflows | Actual source and packaged UI operation, visual/interaction tests, retained source/native correspondence; account for unfinished source WIP explicitly |
| Build, install, release, diagnostic, benchmark and regression tools | Reproducible builds/distribution and the maintained SDK's runnable workflows | Focused and integration validation, clean packaging/runtime dependencies, meaningful comparable timings and independent truth; no evidence-package-only replacement |

Documentation, fixtures and manifests support these requirements; they cannot
substitute for the implementations. Keep languages appropriate to each layer:
existing Python tools may be adapted to a Rust backend, but product conversion
must not call the old C++ reader/kernel to conceal an unported path.

## Implementation order and gates

1. Complete the pure source semantic/geometry prerequisites, planar rules and
   strict/interoperability/carrier options against the current C++ contracts.
2. Verify Rust topology/geometry construction and cleanup, then migrate exact
   native STEP construction and query/publication/archive behavior.
3. Resolve all remaining kernel/API units and fork-specific changes while wiring
   real dependency calls; expand modeling and numerical parity continuously.
4. Connect bindings, CLI, GUI and distribution to actual Rust implementations;
   retain the maintained frontend behavior and independent model validation.
5. Audit the entire owner/entrypoint inventory, all named contracts, artifacts,
   test gates, realistic workflows and unported sites against current execution.

The full goal is complete only when all rows have direct evidence and no required
implementation remains. It stays active across turns. Do not mark it achieved
because a source stage, compatibility shim, narrow profile or subset passes.
Historical design-stage and source-only pauses apply to those phases, not to
the newly authorized complete migration. Continue through later construction
when its prerequisites are ready. User policy remains no new PRs, local-only
private models/inventories/derived material, and source-preserving bounded checks.

Merge `upstream/main` regularly during migration and check for updates before
each deliverable increment. Preserve fork commits and published history; use
merge commits for upstream integration, then rerun validation for the affected
contracts. Fast-forward local `main` only to verified commits. Do not push remote
`main` or create/update PRs under the current delivery policy.

## Current partial state

Lossless strict Part 21 inspection, five fixed type/unit catalogs, strict part
identity/closure, owning views, source-bound reports and their opt-in CLIs exist.
The fixed general source geometry dependencies are now implemented and can run
with part/closure in the maintained strict C++ phase order. They still do not
establish native import or complete AP/source validity. Explicit opaque/legacy
source carrier policies now retain immutable receipts and downstream refusal.
Exact schema interpretation choices and the original nongeometric approval path
are implemented separately from default strict checks. The fixed P0 planar
implementation now includes local/global rules, exact directed regions, incidence
and unit conversion evidence; it still creates no native model. Native STEP
construction, bindings/tool backends, kernel/API gaps and the
remaining fork changes are still required work. Consult the live source and
component contracts rather than treating this paragraph as a frozen completion
count.
