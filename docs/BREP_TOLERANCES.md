# Native boxes and tolerance validation

The maintained source is `SmBrep.cpp` at
`d8ed45ab85d52dba07660cafdceacafdca7217ed`: `CalculateBoundingBox`,
`CalculateTightBoundingBox`, `LockBoundingBox`, `UnLockBoundingBox` and
`ValidateAndUpdateTolerances`. These operations remain distinct from source
uncertainty, import quality configuration and repair permission.

## Box queries

`query::brep` unions actual vertex, edge and face boxes in source list order.
Options retain the native per-member tolerance expansion, tight member requests,
and approximate mode. Approximate mode skips faces; its vertex-only shortcut
requires **more than 500 vertices and more than 100 edges**. These legacy culling
boxes and approximation choices do not establish a continuous geometric bound.

The explicit locked box overrides normal box options. Relocking releases the old
box first, so failure leaves no stale lock. The independent tight-box operation
does not use that shortcut; it retains native contained-member shortcuts, ignored
ordinary member statuses and previous local output values. Missing callees still
propagate `Unported`. No sampled stand-in is used for an absent query.

Public query wrappers and available cache, property, HCR and stitch callers now
delegate to the owning box implementation with their actual `Kernel`. Interfaces
that carry only unresolved opaque handles still require a model resolver. The
remaining functions of base unit `query.brep` are not qualified by this co-port.

## Tolerance operation

`topology::brep_validation::validate_and_update_tolerances` visits upward face
uses, marks each use and its mate, and calls the existing gap getters in native
order. The original `validate_only` model flag is restored on success and error.
Lazy caches can change in check mode; check mode does not authorize repair.

Native behavior retained deliberately:

- The violation/update flag compares vertex gaps with the full vertex tolerance,
  while reports compare against half that tolerance. Reports can therefore exist
  with a false flag. Ordinary completion also returns success with reports.
- Explicit update mode raises a vertex's working tolerance to twice its largest
  qualifying gap. Edge adjustment uses the native model-size/800 condition and
  existing boundary-cascade setter. The BRep tolerance remains unchanged.
- Strict maximum comparisons preserve the first tied vertex-use witness. Native
  unused outputs remain zero or absent. The source edge/face-maximum accumulator
  is shadowed by a local self-comparison; this port preserves that zero result.
- Level/request inputs follow the fixed Release macro behavior described in the
  [construction and pointer-validation contract](BREP_CONSTRUCTION.md).

Surface values hold owner handles, not the containing model. Topological callers
now resolve actual face working tolerances explicitly for point drops, local
point solves and gap sampling. Geometry owners remain attached; existing solver
algorithms, convergence rules and failure paths are reused. Unresolved lookups
remain errors. Supplied owner information does not choose or enlarge import
acceptance budgets.

Scoped surface-cache topology access now resolves face tolerance, pole tests,
editing state, upward loops, stored trims, vertex uses and face classification.
The shared trim-cache algorithm retains native guard restoration and virtual
containment classification. Planar UV construction dispatches to the actual
`SmPlane::DropCurve` port and retains its line/B-spline kinds and intervals.

`topology::surface_cache::make_face_cache` builds an owning temporary cache with
live typed model handles. `ComputeUVPoint` builds it before temporarily changing
its point/boundary flags, then restores those flags on every return. The context
cannot outlive its model borrow; cache reuse/invalidation across edits and the
full native cache manager remain separate required work. Generic callers without
an owning-model resolver still report missing dependencies. Nonplanar projection
and remaining geometric dispatch are not implied by the planar comparisons.

The current `ComputeUVPoint` integration qualifies successful trim-cache builds.
The native owned `smsurf_GetSurfaceCache` fallback to a plain surface cache after
an ordinary trim-cache build failure still needs to be wired; this caller
currently returns the build error. Its remaining owned B-spline local-solver
paths can also reach unresolved face tolerance/seam context. These cases do not
establish full method parity or native import acceptance.

Native check mode can still update working tolerances indirectly when it creates
UV curves inside this cache path: `CreateUVTrimCurves(TRUE, ...)` invokes the native
vertex setter. This source behavior is deliberately compared, not hidden by a
copy or an early cache shortcut. A future strict import transaction must detect
and qualify such side effects under its source/target and repair contracts;
calling this low-level checker alone is not permission to change source quality.

## Evidence and remaining scope

The C++ observer invokes the actual SDK operations on public synthetic geometry.
The matrix varies gap thresholds, check/update mode, report selection, repeated
calls, prior validation flags, box flags and locks. It compares diagnostics and
values, maxima and witnesses, resulting working tolerances, restored flags and
boxes. Cache cases also compare flag/domain state, UV curve kinds and parameters,
point classifications and actual topology identities. Deferred cache/solver paths
are reported separately, never passed.

```text
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/tolerance-reference --probe validation
python scripts/brep_tolerance_compare.py --cpp bin/tolerance-reference/brep-validation-reference.exe --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-tolerances-validation.local.json
```

Receipts, SDK identities and private models stay local. These are operation-level
contracts, not imported-model qualification. Full root/local `AssertValid`,
notification/cache integration, geometric dispatch gaps and native STEP
source/target acceptance/publication remain in the
[complete fork migration](FORK_MIGRATION.md).
