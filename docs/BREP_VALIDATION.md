# Native root validation

`topology::brep_validation::assert_valid` migrates the maintained `SmBrep.cpp`
root sequence at `d8ed45a` (34820–35214). It returns `Result<bool>` and appends to
the caller's existing `AssertArray`. The default follows the native level-zero
plus-walk arguments. Complete model qualification requires level two, all
applicable descendants and every required geometric predicate to finish.

## Order and evidence

The root runs its base chain unless the level is Given, then always calls
`ValidatePointers` at level zero with the requested walk convention. Ordinary
pointer status failure returns false immediately. It then calls the actual
check-mode tolerance routine, accumulates ordinary failure and continues through
root context, region/shell containment and connectivity checks. The coincidence
query has the original explicit level/request guard. Test21 adds a warning
without changing the returned boolean. The requested descendant walk comes last.

The Rust root cannot be an intrusive peer of another model. Root peer/context
and reciprocal vector/count invariants are intrinsic to this representation;
native base tests read the actual root region membership and owner backpointers.
They do not read the notification field map's default list links as graph
evidence. AObject attribute container types and value context are intrinsic as
documented in its owning module. No additional reciprocal-attribute or graphics
validation is invented here.

The native Release boolean macros compare levels; Given requests do not suppress
the forced pointer prefix or ordinary root boolean tests. The separate
`SM_ASSERT_TEST_NEED(15, LEVEL2)` guard does select the coincidence query by request
and excludes LEVEL_HEAL. Disabled/composite/optional build-profile tests remain
disabled. Missing operations propagate as explicit `Unported` errors and cannot
yield a completed model-validation result.

## Root membership

The model now stores root region-list membership independently from a region's
`in_brep` owner backpointer. These represent different native facts:

- A raw damaged backpointer remains observable while the region is still on the
  root list, so owning-base diagnostics precede the pointer-prefix diagnostic.
- A legitimate detach removes membership and clears ownership while retaining
  the arena record; the list-head update is part of the unlink operation.

Construction, explicit attach/detach and the existing region-removal/BrepData
paths update both facts. `GetRegions` traverses actual membership and semantic
head order; it no longer hides a broken owner by filtering it out.

## Remaining gates

The ordered root implementation still reaches missing operations:

- Multi-region `FindInfiniteRegion` ray selection; its native single-region
  branch executes before any ray/bounding-box work.
- Ray-based `IsInnerShellOf` and complete coincidence detection.
- The root `AssertSubTopology` branch now collects the actual stored lists and
  face-derived loops, then runs Region, Shell, Face and Loop checks in native
  order. Edge/Vertex local validators and the non-root `GetSubTopology` branch
  remain explicit missing operations. Face execution still requires dynamic
  surface assertions and, on periodic surfaces, full seam classification.
- Underlying geometric paths such as a repeated gap check constructing a
  `CrvOnSurf` over an analytic plane, where concrete surface dispatch is missing.

Accordingly a root-only no-walk success establishes only its selected root and
underlying checks. A walk encountering a missing descendant or selected missing
predicate returns `Unported`. It cannot silently omit the remaining objects.
Neither source-schema recognition, root-only success, pointer success nor a mesh
qualifies strict native STEP import or the [complete migration](FORK_MIGRATION.md).

## Interpreting the result

The actual SDK can return **true with failed pointer/tolerance reports** because
the root combines those routines' statuses rather than treating a nonempty
report array as false. This behavior is preserved and directly compared. Import
acceptance must separately classify reports and apply its quality policy; the
low-level boolean alone is not an acceptance gate.

Validation also retains original lazy geometry/cache and working-tolerance side
effects. It is not a read-only operation, a transaction or repair authorization.
Unpublished import candidates still require their own cleanup, source/target
comparison and publication decision.

## Focused native comparison

```text
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/root-validation-reference --probe validation
python scripts/brep_root_validation_compare.py --cpp bin/root-validation-reference/brep-validation-reference.exe --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-root-validation.local.json
```

The matrix uses actual SDK calls on public synthetic runtime graphs. It compares
booleans, ordered diagnostic identity/content, LastAdd and observable NURB state
across levels, requests, missing links, tolerance faults and repeated calls.
Deferred cases are recorded separately. Run receipts and private model data stay
local; these are operation requests, not model-library import qualification.


## Region and Shell local checks

`assert_region` and `assert_shell` accept the same explicit level/walk options.
Region has only the actual native base chain; its source contains no additional
Region-local predicates. Shell checks vertex, wire and face-use contents against
independent root scans and actual connectivity traversal. A structurally valid
curveless wire can pass Shell-local checks; this does not validate its geometry.

The base chain reads membership and backpointers from the graph and mark/flag
state from the owning model. A mark3 failure intentionally cascades into the
source ring test7 because native `bOk` is not reset. Shell test24 is used twice
by the source for different predicates; its emitted identity is preserved.
Null shell heads and missing first loop heads retain the original early returns.
Disabled radial/sector checks remain owned by the later local validators.

The same model must retain its owning Kernel context across calls; replacing
it with a fresh context is not equivalent to constructing a new native graph.
The existing traversal algorithms now accept owning marks through a shared
adapter. It loads retained metadata, persists traversal effects even on error,
and preserves existing flags/user fields. It never uses metadata list links as
a replacement for the actual graph. Context mark locks are released before
propagating a traversal error. Non-root walk requests still explicitly require
the unported non-root `GetSubTopology` branch.

The local native comparison adds vertex shells, single wires, chains and
branches, missing owner/list/loop cases, mark-limit faults, levels zero/one/two/
Heal/Given, absent report arrays and repeated calls. It compares ordered
reports, boolean, LastAdd, context counter advances and final current-mark
membership on live vertices/edges/faceuses/vertexuses. Counter advances are
relative to each constructor's starting context; they are not an assertion of
absolute constructor counter parity. Run with:

```text
python scripts/brep_local_validation_compare.py --cpp bin/local-validation-reference/brep-validation-reference.exe --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-local-validation.local.json
```


## Face, Loop and use checks

`assert_faceuse` and `assert_loopuse` now execute their actual local bodies.
Their native walk argument is ignored; it does not request descendant traversal.
Loopuse 3D closure is checked only on the logical loop's primary use, preserving
its existing native over-tolerance bug. A returned boolean is not a closure or
import-acceptance certificate. Typed list counts are intrinsic; raw native head
corruption with an independently stale count has no equivalent vector mutation.
The native label-table defect and equivalent-fixture boundary are recorded in
[upstream issues](../plan/upstream-issues.md).

`assert_loop` runs the actual natural-boundary inside-side query, CheckLoop,
paired-use counts/edge membership/reversed order alignment, and both use checks.
Rotating the mate's starting edge is legal. Reversing its traversal order is
reported. CheckLoop's ordinary status is ignored exactly where native ignores
it, while reports and checked Rust access failures remain observable.

The new edgeuse/loopuse side module preserves the six-sample default, optional
known periodicities, existing/missing UV conventions, boundary priority, and
zero/one sample behavior. It evaluates an existing UV curve at the EDGE
parameter, without silently normalizing it. Only previous sample state is
retained; the native sample arrays have no other non-debug consumer. This is
an equivalent implementation of the native sampled predicate, not a continuous
orientation proof. Failed native point-drop out-parameters unavailable from the
current fallible solver interface remain explicitly unported.

`assert_face` now follows the original domain, owner/pole, vertex classification,
faceuse, degeneracy, coincidence, loop nesting/intersection/shared-vertex, and
seam sequence. Explicit `SM_ASSERT_TEST_NEED` guards differ from Release macros:
Given does not suppress ordinary macro predicates, and Heal does not enter the
explicit request guards. Faceuses run outside the vertex-classification guard.
Vertexuse tolerance is the inherited fixed old-tolerance base value from
`SmTopology.h`/`SmMath.h`, not the vertex's stored tolerance. Actual owner working
tolerances use the canonical geometry identity and a checked constant-time
arena lookup; notification identities are not accepted as geometry owners.

Loop tests37/38/39 use the original sampled containment, pairwise UV intersections
and shared logical vertices. Inconclusive numerical source paths retain their
original result; unavailable algorithms remain errors. These predicates add no new fitting or interval-normalization policy; actual
native callees retain their documented UV/tolerance/trim side effects. Exact
UV references are reacquired after mutable curve generation so physical aliasing
still selects the native self-intersection path.

Face remains partial: dynamic Surface/BSplineSurface assertions and periodic
FaceProps seam classification are explicit dependencies, as are unavailable
coincidence failure outputs. Given-selected planar calls can finish, but do not
include the skipped base/surface validation. Root walk reaches these actual
dependencies, or Edge/Vertex after completed selected Face/Loop checks, rather
than stopping at an unexecuted blanket Face placeholder. Non-root walk remains
unported. Complete migration and native import acceptance remain unestablished.

```text
python scripts/brep_face_loop_validation_compare.py --cpp bin/face-loop-validation-reference/brep-validation-reference.exe --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-face-loop-validation.local.json
```

The synthetic comparison includes normal, outside, crossing, nested and
vertex-sharing holes. These are native operation requests, not private-library
model counts. The cases demonstrate why sampled test37 and a low-level boolean
cannot replace pairwise intersection and independent quality evidence.

Known native invalid-domain classification crashes are listed separately as
`unsafe_native`, and never counted as matches or deferred numerical evidence.
Rust safety checks require rejection/error and preserve the original invalid
input; the SDK crash is not used as an approximation oracle.
