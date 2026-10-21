# Complete C++ fork migration

The active objective is **move every implementation/capability in the maintained
C++ fork into solidcore**. STEP source checkpoints are increments toward that
objective, not a replacement objective or completion criterion. The migration
includes the kernel, public API, STEP pipeline, bindings, viewer/workbench and
supporting build/validation/distribution tools. Preserve caller-visible behavior,
source semantics, numerical/ownership contracts and actual failure behavior.

STEP model-import milestones belong in solidcore: successful source inspection
or kernel test counts cannot replace actual complete Rust BRep imports. Connect
the maintained source pipeline to the Rust builder before unrelated kernel work.
There is currently no native viewer implementation in this Rust repository.
The requested standalone viewer must use wgpu and support CLI file launch while
retaining the maintained progress/HUD, rendering and selection/measurement goals.

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
After verified fast-forward integration, remove merged local checkpoint branches
that no worktree uses. Keep unmerged work and worktree-owned branches; maintain
recovery references locally when pruning a batch.

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

The actual Rust runtime constructor is being migrated separately from source
qualification; see [its ownership and verification contract](BREP_CONSTRUCTION.md).
Root `SmBrep::ValidatePointers` / `AssertValid` and their local validators are a
required additional owner, `fork.topology_validation`, omitted by the base unit
inventory. The required-link prefix and Release pointer body now execute, including
mutating reverse-direction validity checks. Missing geometric/notification
dispatches and full root/local AssertValid plus gap/tolerance checks still gate
native acceptance. Pointer success or base reader-unit status cannot stand in
for these implementations.

Native box and gap/tolerance operations now have their own
[contract and remaining dependencies](BREP_TOLERANCES.md). Their operation-level
evidence does not complete native import or the full SDK migration.

Scoped surface-cache owner access now connects actual face working tolerances,
editing guards, UV generation, typed trim/vertex handles and face classification.
The planar virtual curve drop is wired; comparisons include direct cache state
and check-mode UV/tolerance side effects. Full cache-manager notification/reuse,
remaining B-spline owner context and general projection
remain required work; see `BREP_TOLERANCES.md`.

The native owned getter now retains plain/trimmed cache roles, single-slot reuse,
modified/editing invalidation, ordinary-failure plain fallback, Boolean-delayed
trees and virtual GetTree builds. Model-bound slots are explicitly owned by the
caller; full notification/registration/checkout integration is still required.
The B-spline local solver carries live face tolerance into its seam test, with
analytic and actual native point comparisons; this does not qualify other
remaining owner or general-projection paths.


Root BRep notifications now own persistent metadata/attributes/spatial cache,
execute the actual cache/base/attribute/user/system callback chain, and are wired
to Stitch's pre-edit path. See [notification scope and remaining integration](BREP_NOTIFICATIONS.md).
Cross-model resolution, all element producers, checkout/display-list behavior,
full root/local validation and native import acceptance remain required work.


The ordered root AssertValid entry now executes its base/pointer/tolerance and
root region/connectivity logic with explicit missing geometric/descendant gates;
see [root validation contract](BREP_VALIDATION.md). Native root booleans can be
true with failed reports and cannot stand in for the import acceptance decision.
Region membership and owner backpointers are stored separately for actual base
validation rather than inferred from the notification records.


Region and Shell local validators now execute their actual base/structural and
connectivity checks; the root descendant sequence invokes them before Face/Loop
and the remaining Edge/Vertex barriers. Native marks persist in owning state
through shared traversal adapters. This is operation-level migration progress,
not complete local/geometric validation or import acceptance; see
[the validation contract](BREP_VALIDATION.md).


Face and Loop ordered assertions and their faceuse/loopuse bodies now execute
real geometry/classification/closure/coincidence/loop checks. Shared natural-side
queries and checked owner tolerance resolution are reusable native operations.
Dynamic surface assertions, periodic seam classification, Edge/Vertex validators
and non-root traversal still gate complete validation. Selected Given calls do
not qualify skipped base/surface checks; see [validation scope](BREP_VALIDATION.md).


Native Plane/Surface/BSplineSurface assertions and geometric PointGrid dependencies
now execute real control-net/geometry checks, preserve existing NURBs and retain
native failed-status partial outputs. Concrete Revolution/other subclass bodies,
periodic seam checks, remaining local validators and root dependencies still gate
complete validity and strict native import. Selected operation equivalence does
not establish all SDK ownership/attribute or diagnostic ABI behavior.

Concrete `CrvOnSurf` dispatch and local Vertexuse assertions now execute over the
typed model. Plane definitions survive the curve-on-surface path instead of
requiring a B-spline replacement. The maintained UV-only copy rule and explicit
context transform entry are preserved. External borrowed aliases, const-copy
source-cache changes, geometry notifications, full Edge/Vertex/curve assertions
and the remaining native import pipeline are still required work; see
[validation scope](BREP_VALIDATION.md).

The actual Curve/B-spline/Line assertion chain now retains its owned NURB,
ordered diagnostics and fallback property analysis. The curve-property solver
uses its borrowed geometry's natural parameter domain for duplicate comparison,
including calls without an output handle. This does not complete Edge/Vertex
assertions or other concrete curve subclasses. Portable software interval
arithmetic covers the bounded Bezier proof; nondefault native floating-control
and cache-side-effect parity still need separate qualification.


The actual Edge/Edgeuse local assertion bodies now follow the stored curve/use,
UV ownership/parameterization, endpoint-gap, manifold/spine sector and radial
ordering paths. Root descendants invoke them before the actual Vertex assertions.
Periodic-jump and near-pole polygon queries execute over original definitions;
owned gap sampling and binormal cache creation retain their lazy side effects.
Source-native unsafe graphs and unresolved geometry/non-root traversal calls
remain separate evidence categories. See `BREP_VALIDATION.md`; this does not
complete native import, topology validation or the full SDK migration.


Vertex local assertions now execute their real incidence and face-gap/corner
callees, including the one/two/three-surface target branches and owner-bound
point-drop context. The qualified planar root walk completes all descendants;
missing coincidence/ray predicates, non-root traversal and general projection
still prevent broader claims. Generic B-spline point solves now carry real face tolerance through seam/local
checks without eager trim construction; actual general curve projection remains
a separate required dependency. See `BREP_VALIDATION.md`; acceptance/publication and the
remaining full ledger are still required work.


Optional root/non-root subgraph collection and local walk dispatch now retain
native output masks, append seeds, mark epochs, order and failure cleanup.
Undefined native marked-partial-output accesses remain checked failures; source
Faceuse marking and duplicate Face behavior are preserved. Whole-root
coincidence/ray checks and general projection are still separate implementation
requirements; this does not complete the full migration ledger.


Batch subgraph collection now shares the actual singular collector, with native
input distribution, duplicate/precedence/early-return rules, all-six temporary
arrays and explicit safe Rust output-alias selectors. Native reset-alias invalid
reads remain checked errors with the reset state retained. Root coincidence,
general projection and the rest of the full SDK/import/binding/viewer ledger
remain required implementation work; batch topology parity is not completion.
