# Runtime BRep construction

`solidcore::topology::brep_constructor::BrepConstructor` ports the maintained
fork's `SmBrepConstructor` and `SmVertexNode` in `SmBrepData.h` / `SmBrepData.cpp`
at `d8ed45ab85d52dba07660cafdceacafdca7217ed`. It constructs the actual kernel
`Brep`, including regions, shells, paired face/loop uses, radial edge uses, vertex
incidence, defining curves/surfaces, intervals and optional UV curves.

The base `topology.brep_data` function inventory primarily covers flattened-data
readers. Its existing status does not qualify this constructor, native STEP import
or root runtime validation. These are separately required by the
[full fork migration](FORK_MIGRATION.md).

## Ownership and completion

A constructor owns one unpublished candidate. `finish(self)` consumes it and
requires region, shell, face and loop scopes to have ended. Failure or dropping
the builder releases all owned arenas and geometry. Intermediate calls can leave
an incomplete graph, including after a delegated error; callers discard that
candidate rather than resuming publication. `brep_mut()` is kernel friend access,
not an accepted-model handle. `finish` establishes scope closure only. A completed
candidate still needs structural and geometric qualification before publication.

Rust geometry values transfer into the owner; cloning retains the concrete kind
and removes the previous owner. The C++ pointer-sharing form of `SetFaceSurface`
is not exposed: its already-owned branch constructs `SmSurface(*pSurface)` and
slices the concrete kind. The Rust owned-value interface retains the defining
geometry. It does not model pointer aliasing between faces.

Geometry owner handles use the existing BRep-local `arena_identity` convention.
Cache consumers combining multiple BReps must provide their own model-qualified
keys, as `BrepCache::build_trees` requires; this constructor does not establish a
global model identity or kernel thread safety.

## Source conventions retained

Coordinates and working tolerances use the caller's model units. Curve parameters
and surface UV retain their defining parameter systems; this layer performs no
unit conversion or quality-profile selection.

- The first region is infinite/void. The legacy solid hint does not classify the
  created region; an importer sets material/void semantics explicitly.
- A face's primary use is SAME. The requested outside orientation routes the
  pair between outside/inside shells. Open surface shells retain both uses.
- The first loop is outer; subsequent loops are inner. Loop walking orientation
  remains separate from this role and from each edge's underlying-curve sense.
- Shared edges reuse identity, endpoints and geometry. Mate/radial insertion uses
  the existing kernel operation. Coordinate coincidence never welds vertices.
- `VertexNode::Parameter` retains the supplied curve parameter. Point/UV nodes
  invoke the legacy point-drop retry at 1x then 100x working tolerance and its
  strict endpoint-distance snapping. This is constructor policy, not permission
  for a STEP importer to repair a model or enlarge an acceptance budget.
- UV setters retain the original current-use assignment. Canonical SAME-side
  installation is a separate operation. UV-only input lifts through the real
  surface operation, restoring the surface owner on success and failure.
- The existing surface lift now preserves concrete analytic lazy NURB creation
  when crossing a base B-spline interface. It materializes exact analytic data on
  a copy, retains parameters, and borrows ordinary B-spline input without another
  full buffer. It does not fit or rewrite defining geometry.

## Missing dependencies remain explicit

`EndShell(true)` invokes `SmBrep::ValidatePointers` before stitching in C++. Root
`ValidatePointers` / `AssertValid` and their local validator contracts are not
complete. This additional fork owner is tracked as `fork.topology_validation`,
outside the base unit inventory. The optional stitch path returns `Unported`
before the omitted call; it cannot silently succeed by skipping validation.

Three-or-more-face radial insertion can reach `SmSurface::DropCurve`
(`intersect.surface_drop_lib_p3`); non-iso lifting on non-B-spline surfaces can
reach the `curve.crv_on_surf` dispatch gap. These failures propagate, with scoped
temporary-edge and surface-owner cleanup. They are not accepted models. General
geometry, repair, native STEP correspondence and publication require their own
implementation and evidence; neither this constructor nor a rendered mesh
establishes full native import.

### Root required-link prefix

`topology::brep_validation::required_links` implements the `ValidatePointers`
required-pointer prefix (tests 112--146) using the existing `AssertArray` and
fixed labels. It visits regions/shells, global faces and both loop sides, global
edges, and global vertices in source order. It stops on the first absent required
link; the CCW walk retains the source's 100,000-visit bound. These checks always
run at level zero even when the outer request selects other tests.

Context/global list-container pointers are intrinsic to the owned Rust value
model. Raw circular sibling pointers have no independent nullable field; their
relationships come from the owner's typed list. Dangling arena ids or inconsistent
radial membership are Rust access errors, not invented C++ null-pointer reports.
The report keys distinguish types and arena indices within one BRep. Cross-model
report accumulation still needs model-qualified identity; these local keys are
not global object handles.

Passing this prefix establishes required-link presence only. The public
`validate_pointers` entry still returns `Unported` for the remaining predicates;
constructor stitching/publication gates remain in place. Complete validation must
also preserve CheckFace/CheckLoop, geometric checks, live-owner and backpointer
checks, selected-test behavior, root/local AssertValid and gap/tolerance checks.
The C++ validator's ordinary final success code does not mean its report is empty.
Its geometry callees can materialize caches and even adjust NURBS control points;
those effects must be preserved and checked against source before acceptance.

The synthetic negative observer invokes the actual C++ `ValidatePointers`.
It obtains test-only data-member access through explicit template instantiation;
class declarations, function access, layouts and vtables stay unchanged. Mutated
links are restored before native context teardown. The comparison covers the
first failed label/owner/message, selection-independent prefix, duplicate report
handling and optional report arrays, not deep validation or native acceptance.

```text
cargo test --locked -p solidcore --lib topology::brep_validation
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/validation-reference --probe validation
python scripts/brep_validation_compare.py --cpp bin/validation-reference/brep-validation-reference.exe --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-required-links-validation.local.json
```

## Reusable verification

Library tests exercise scope failure, duplicate definitions, parameter retention,
identity preservation, vertex loops, root-validation refusal, UV lift success and
failure, and temporary-edge cleanup on the actual radial dependency error.

```text
cargo test --locked -p solidcore --lib topology::brep_constructor
cargo build --locked -p solidcore --example brep_constructor_probe
```

The host-only C++ observer uses an existing compatible Release SDK and matching
headers. On Windows, select the same CRT/layout/debug/graphics macros as the SDK
through the supplied MSVC environment. The builder currently targets the fork's
Unicode callback-enabled SDK configuration; it does not rebuild the kernel.

```text
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/constructor-reference
python scripts/brep_constructor_compare.py --cpp bin/constructor-reference/brep-constructor-reference.exe --rust bin/debug/examples/brep_constructor_probe.exe --receipt bin/brep-constructor-validation.local.json
```

Both programs construct the same public synthetic shapes: plane, hole, two shared
faces, material/surface boxes, single-vertex loops, three-face radial edges and
UV-only iso lifting. Inputs vary orientations, explicit/inferred vertices, UV
storage, point/parameter limits, scale and translation. The observer compares
ordered topology, incidence, mate/radial/CW/CCW links, roles, geometry owners,
intervals, surface evaluations and curve/UV endpoints and midpoints. Discrete
fields are exact; numeric fields use `1e-9` relative with no absolute floor.
Midpoint samples characterize these inputs; they are not a continuous error bound.

Dependency results are `deferred`, separate from passes. A C++/Rust failure or
timeout fails the comparison; it is not relabeled unsupported to pass the gate.
No validator is replaced with this observer. Per-run receipts, SDK identities,
compiler output and private models remain local and ignored.
