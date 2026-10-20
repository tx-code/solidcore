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
- `AssertSubTopology`'s once-only Region/Shell/Face/Loop/Edge/Vertex local checks.
- Underlying geometric paths such as a repeated gap check constructing a
  `CrvOnSurf` over an analytic plane, where concrete surface dispatch is missing.

Accordingly a root-only no-walk success establishes only its selected root and
underlying checks. A walk or selected missing predicate returns `Unported`.
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
