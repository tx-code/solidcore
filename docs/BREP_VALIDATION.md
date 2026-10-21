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
  order, followed by the real Edge/Edgeuse and Vertex calls. Non-root
  descendants now use the shared native subgraph collector. Face execution now includes Plane and
  BSplineSurface assertions; other concrete surface subclasses and periodic
  seam classification remain required.
- Other concrete curve/surface assertion bodies and unavailable geometric
  dependencies, together with live shared cache registration/checkout identity.

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

CheckLoop's conditional singularity searches resolve the working tolerance
through the loop's primary-use face and the supplied surface's geometry owner.
Exact evaluation copies keep that owner. An unknown or different face cannot
supply the loop's tolerance. Plane overrides and loops without UV gaps retain
their early paths; the pole search threshold remains a separate argument. This
lookup changes neither defining geometry nor the installed UV curves.

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

Face remains partial: remaining concrete surface subclass assertions and periodic
FaceProps seam classification are explicit dependencies, as are unavailable
coincidence failure outputs. Selected planar calls and their non-Given Plane/BSplineSurface assertions can
finish; Given still does not include skipped base chains. Root walk reaches these actual
dependencies after completed selected Face/Loop checks, rather
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


## Plane and B-spline surface assertions

`assert_surface` now executes the native Surface -> BSplineSurface -> Plane
chain with the concrete virtual class convention. A Given Plane call skips its
BSpline parent and may retain an absent NURB; a non-Given call performs the
inherited checks and materializes the actual model-owned representation. Plane
position, analytic domain, analytic/NURB corner agreement and positive UV scales
are checked in source order. Revolution and other concrete subclass bodies
remain explicit dependencies; a BSpline base check cannot stand in for them.

Surface checks use actual flat corners, degeneracy, knot continuity, exact versus
approximate singular sides and opposite-boundary coincidence. Native continuity
status is separate from its partial outputs: the existing fallible API remains,
while an internal output-preserving entry lets this original caller consume its
initialized minimum after an ordinary failure. Failure before initialization is
explicitly unestablished; no guessed continuity is supplied.

BSpline checks include actual NURB presence, knot multiplicity, internal poles,
degenerate control rows and repeated end-point queries, followed by the native
control-grid monotonicity predicate. Repeated-end output is warning-only; its
query is executed without inventing a failed assertion. Native warning-stream
transport is outside the selected report-field comparison. The
source asymmetry in internal-pole confirmation and early row-exit behavior are
preserved. `GetMaxDimension` means the native absolute-coordinate convention,
not a replacement span measure. No control point or knot is moved to obtain a
pass. GetOrCreate never regenerates an already stored NURB: a conflicting
analytic/cache representation is retained and diagnosed.

`IsClosed`'s third pointer is INPUT tolerance (`SmSurface.h1498`); it does not
modify the BSD grid tolerance. The native `1e-6` defaults and `dTol*1000` grid
argument are retained as SDK behavior, not production import-quality defaults.
The U closure flag is the grid's closed-column argument and V is closed-row.

The separate geometric `PointGrid` follows `SmPointSet.cpp` query/cache order.
It preserves setters that keep cached results, ClearCache that resets monotonicity
but not coincidence, the stale first-length comparison and the column branch
writing the row flag. Undefined native zero-row/undersized accesses become
checked Rust failures. This predicate remains a local-turn heuristic, not an
injectivity or continuous-geometry proof.

Report observers now also compare the virtual owner class string, preserving
concrete Plane/UV curve names separately from static assertion class codes.
This extends the selected record-field evidence, without claiming full binary
record/ABI or all optional diagnostic parameter qualification.

```text
cargo build --locked -p solidcore --example brep_surface_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/surface-validation-reference --probe surface
python scripts/brep_surface_validation_compare.py --cpp bin/surface-validation-reference/brep-surface-reference.exe --rust bin/debug/examples/brep_surface_probe.exe --receipt bin/brep-surface-validation.local.json
```

Synthetic inputs cover folds, repeated points, translated/late collapsed rows,
U/V pole asymmetry, rational weights and full interior multiplicity, alongside
levels, repeated arrays and cached-grid flags. They qualify selected native
operations only. Dynamic geometry attributes, other subclass validators, full
root/local validation and strict STEP acceptance remain required work.

## Concrete curve-on-surface dispatch and vertex uses

`CrvOnSurf` now holds the concrete `Surface` enum. Its constructors accept
`Into<Surface>` and its `surface()` accessor returns `&Surface` rather than
`&BSplineSurface`. Plane definitions, UV parameters and scales survive evaluation,
derivative lifts, copy, transforms and supported archive dispatch. Topology gap
checks no longer reject a Plane merely because it is not the B-spline enum case.
Missing classification and other virtual dependencies remain explicit errors.

`transform_with_kernel` uses the actual caller context; `Curve::transform` routes
through it. The older context-free entry still handles B-splines and Planes, and
requires an explicit context for other concrete kinds. Native owner flags decide
whether the base surface moves. Cached approximation transforms precede the base
surface call. An ordinary failure can leave a modified NURB and an unchanged
analytic definition, exactly as the native Plane operation does; this is not an
atomic modeling operation or permission to repair geometry.

The maintained fork's UV-only copy rule is preserved: owning the UV curve alone
does not invoke a virtual surface copy. Unowned values are exact snapshots in
this representation, without copy-constructor knot normalization. External live
borrowed aliases, source mutation through const-copy cache materialization and
geometry notification ownership are still separate migration gaps, not qualified
by these numerical operation comparisons.

`assert_vertexuse` follows the actual old-tolerance SDK local routine: the base
chain except for Given, optional target context/type checks, the real missing-seam
query and the face-or-wire attachment test. It ignores walk as the source does.
Disabled fresh-gap and broad sector tests stay disabled. Checked arena failures
and missing dependencies propagate; they cannot yield a complete validity result.
The base membership adapters are distinct from the local validators. Root
traversal now executes Edge/Edgeuse and Vertex on the qualified planar graph;
other geometry can still stop at explicit dependencies.

The native observer uses public synthetic Plane, rotated Plane and bilinear
surface receivers, all ownership flags, copies, identity/rotation/scales, failed
nonuniform Plane transforms and explicit version-40 ASCII/binary archives.
Both primitive database width selection and nested record version are explicit;
the Windows version-30 reader cannot consume the current writer's 64-bit longs.
Byte-swapped native writer output is not a truth source (see the database issue
in `plan/upstream-issues.md`). No full import or shared-alias qualification follows.

```text
cargo build --locked -p solidcore --example brep_curve_probe --example brep_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/curve-surface-reference --probe curve
python scripts/brep_curve_surface_compare.py --cpp bin/curve-surface-reference/brep-curve-reference.exe --rust bin/debug/examples/brep_curve_probe.exe --receipt bin/brep-curve-surface.local.json
python scripts/brep_vertexuse_validation_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-vertexuse.local.json
```

## Owned curve assertions and property-solver domains

`assert_edge_curve` and `assert_uv_curve` execute the stored curve's actual
Curve -> BSplineCurve -> Line chain. The UV entry checks only its own stored
curve, without a mate fallback. The Line Given override skips its parent and may
keep an absent NURB; the B-spline Given routine still executes its numerical
checks. Other concrete curve assertion bodies remain explicit dependencies.
These methods do not assert the edge/use or establish import qualification.

The B-spline body retains prefix early returns, end/internal knot multiplicities,
representation counts, endpoint/control-point coincidence, one-sided derivative
and kink checks, global speed/radius analysis and internal poles. Source rule4
uses the start-pair predicate/value even when only the end pair coincides;
both one-sided zero-derivative branches use indices5/6. These source quirks are
preserved, not used to redefine mathematical validity. The final pole tolerance
is the SDK's fixed `SM_ZONE_TOL_3D/10`, not an import-quality default.

Radius rules13/14 have the maintained fork's explicit request guard: Heal and
unrelated Given requests skip that search. Release macros for the other rules
retain their original numeric level behavior. GPA's initialized partial outputs
survive ordinary status failures. Missing dependencies or inaccessible failed
evaluation outputs remain errors rather than fabricated numerical evidence.
Reports receive owner parameters and final values only after a successful new
insertion; duplicate reports keep their original metadata.

The bounded Bezier proof handles only concrete B-splines with already-separated
degree1--3 spans (at most16 spans, depth3). It never inserts knots, fits geometry,
rejects a curve or invents a worst-case parameter. Inconclusive bounds retain the
original GPA path. Pinned `softfloat`1.0.0 performs the same binary64 operations
and outward one-ULP expansions without depending on hardware rounding/FTZ/DAZ
state. Default-environment native operations are compared separately; this does
not qualify the native SSE control-state guard or all cache side effects in
nondefault environments. Algebraic line/parabola and arithmetic-boundary tests
supplement operation comparisons; sampled results are not continuous proofs.

`GlobalSolver` now shares its original duplicate comparison through an internal
domain-provider seam. `CurvePropertyGs` supplies the actual borrowed curve's
natural domain, independently of optional output-handle metadata. The source
`1/10000` natural-domain rule is unchanged. This resolves the small-scale
near-cusp case where missing domain data previously left several approximate
minima ahead of the tighter midpoint candidate. An unrelated solver still uses
the existing handle-based default and its original failure behavior.

Shared curve materialization uses the actual owner and context, preserves
existing representations, and serves both assertion and reverse-direction checks.
Internal-pole queries never create a missing NURB; the native base `SmCurve`
receivers retain their literal false default. Geometry attributes, warning-stream
transport and other subclass assertions remain
required SDK work.

```text
cargo build --locked -p solidcore --example brep_curve_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/curve-validation-reference --probe curve_validation
python scripts/brep_curve_validation_compare.py --cpp bin/curve-validation-reference/brep-curve_validation-reference.exe --rust bin/debug/examples/brep_curve_validation_probe.exe --receipt bin/brep-curve-validity.local.json
```


## Edge, edgeuse and pole/periodicity queries

`assert_edge` follows `SmEdge.cpp`7480--7750 at maintained fork `d8ed45a`:
owning base, curve/context/domain, the real curve and use assertions, length,
level-two on-face form, alternating orientation, endpoint incidence/gaps and
manifold/spine radial order. The native numeric-level behavior is retained:
Given still executes rule13. Child calls keep the requested level but receive
no parent's Given request array. Non-root walk uses the same source-ordered descendant dispatcher.
Root walking now invokes these edge routines followed by Vertex validation.

`assert_edgeuse` retains optional context/type checks, the **own stored** UV
owner/length/periodic/parameter rules, loop/wire membership, reciprocal corners,
SAME-use radial/mate sector checks, the concrete UV assertion and the pole query.
Its walk flag is ignored as native. Loop membership actually reports index14,
despite the source comment calling it12. Duplicate diagnostics and owner
parameters retain the existing AssertArray protocol. No check edits UV parameters,
changes source geometry or relaxes a tolerance.

`Curve::assert_no_periodic_jump` preserves the two-control-point exemption and
inclusive `0.9 * domain length` boundary (`SmBSplineCurve.cpp`7623--7691).
`edgeuse::validity::dog_leg_near_pole` follows `SmEdgeuse.cpp`3663--3895:
it can borrow a mate's UV, returns the first polygon index, and retains both the
intersector-approximation endpoint branch and the ordinary branch. The native
ordinary branch overwrites its singularity direction with X for both U and V;
that observable quirk is preserved, without presenting it as a mathematical rule.

Gap calculation now preserves the native `BuildSampleSet -> GetKnots` lazy
materialization on the actual owned curve (`SmGap.cpp`1730--1743 and
`SmBSplineCurve.cpp`4075--4085). Existing stored NURBs are never regenerated.
Binormal projection uses the real face-owned cache getter; cache construction may
create other trims even for a 3D-only request. Missing geometry and inaccessible
failed-binormal outputs still return explicit errors. Shared cache identity,
registration, checkout and other subclass assertions remain incomplete;
a completed local comparison cannot qualify whole-model acceptance or import.

The maintained comparison uses actual constructor-built lamina, two-face manifold,
three-face spine and wire graphs; own/mate UV, all levels, single defects, repeats,
report parameters, control-point materialization, gap-cache state and marks are
compared. Independent binary64 endpoint equality/next-value tests supplement the
SDK comparisons. The query matrix includes all four singular boundaries and the
source's V-direction quirk. Native-unsafe graphs are not used as a validity oracle.

```text
cargo build --locked -p solidcore --example brep_validation_probe --example brep_edge_queries_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/edge-queries-reference --probe edge_queries
python scripts/brep_edge_validation_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --queries-cpp bin/edge-queries-reference/brep-edge_queries-reference.exe --queries-rust bin/debug/examples/brep_edge_queries_probe.exe --receipt bin/brep-edge-validity.local.json
```


## Vertex assertions and corner queries

`assert_vertex` executes `SmVertex.cpp`2550--2579 and2711--2776 at `d8ed45a`:
the owning base (except Given), the unconditional face-gap/corner helper, rule2
incidence checks, then actual Vertexuse assertions. Child Given calls receive
no parent's request array. Walk is ignored as native; root descendants now run
Vertices after Edges. The obsolete tolerance-consistency rule remains disabled
under the selected SDK build macros.

`vertex::validity::check_pointers` preserves the separate shell-vertex,
vertex-loop, wire and loop-corner branches (1446--1707), including reciprocal
mate/loop/face links and the original empty-use-list result. Owning-base checks
remain separate. It introduces no extra vertex-loop cardinality rule or repair.
Checked ids replace undefined native accesses; raw runtime validators are not
hostile-container parsers.

`within_faces` uses the original **strict** gap `<` model intersection threshold,
clearing all relevant caches first when forced. Vertex rule1 only reports when
the largest observed face gap is **greater** than that threshold at level2 or
above. At equality, the within predicate is false but no Vertex rule1 is added.
A failed tight-corner predicate alone can similarly leave the local Boolean true.
These behaviors are source compatibility, not import acceptance policy.

`has_parallel_faces` retains oriented-normal angles, last-wins equal-gap selection
and the shared `GetVertexuseOfFace` accessor's source quirk (the face argument is
ignored). `calc_corner_from_faces` retains every branch: no face, offset parallel
midpoint, one-surface drop, two-surface refinement and three-surface intersection.
Nonparallel targets are selected in source use order with its one-degree test;
later uses still execute their UV/normal checks but do not add a fourth target.
The disabled residual scan stays disabled. Outside the parallel branch, its gap
is movement to the solution on the first target surface and its max-face witness
usually remains unset. It is not a certificate over every attached surface.
`is_on_face_corner` retains native true for a no-answer result. No defining
vertex is moved; temporary midpoint evaluation bypasses Notify and restores the
point and invalidates the same caches on success or failure.

The point-drop pipeline now accepts an explicit `PointDropContext`. Default and
face-lookup entries share the same fast/local/global phases and result selection.
`surface_cache::drop_face_point` supplies the actual model's working tolerance
and global cache/classification environment. Fast success does not build a cache
or generate trims. Normalized edgeuse evaluation reacquires borrowed geometry
after model mutation. The two-pole recovery keeps its already-rescued unowned
surface, rather than reattaching a face and causing trim-generation recursion.
The new global solver receives actual owner tolerance in seam tests and local
refinement. Native cache acquisition stays inside GlobalPointSolve's fast-result
trim branch (`SmSurface.cpp`20065); generic numerical solving does not eagerly
generate UV. Point minimization and corner queries are the qualified modes here; directed
owned bounds and shared cache identity retain their own dependencies.
Cold and pretrimmed concrete B-spline receivers retain their type,
parameters and source checks. General curve projection remains a separate
dependency where actually reached, never a Plane substitution.

Selected SDK comparisons cover all topology roles, source rule levels, scalar
faults, repeated arrays, returned corner/face witnesses, original vertex points,
cache state and marks. Single/two/three-plane analytic truth is checked separately
with a declared1e-14 fixture bound; the subsequent SDK comparison keeps1e-9
relative with no absolute floor. Native access-violation inputs are listed
explicitly and not counted as pass or deferred. Complete coincidence/ray checks,
other geometric dependencies, shared cache ownership and native STEP publication
remain separate full-migration requirements.

```text
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/brep_vertex_validation_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-vertex-validity.local.json
```


## Optional subgraph outputs and non-root walks

`topology_traverser::get_subtopology` follows `SmTopologyTraverser.cpp`1667--2066
at maintained fork `d8ed45a`. `SubTopology` exposes six optional arrays: None is a
NULL output, and `all()` requests every array. `SubTopologyOptions` retains reset
and caller-owned target-mark inputs. An all-None request returns before reading
the receiver. Invalid use-object receivers clear requested arrays and return
SM_ERR. Root collection always resets, even when append was requested.

Non-root collection traverses source input seeds as well as the temporary
receiver. Seeded higher arrays can therefore contribute objects unrelated to the
receiver. The temporary receiver is removed only after the stages finish; it can
remain in a requested output on a checked traversal failure. NoMark writes
arrays directly (the Rust work storage is published on both success and error);
target-mark mode copies to output only at the source's final transfer step.
Neither mode sorts or normalizes existing input duplicates.

A work mark is claimed excluding the target mark. The target mark itself is not
advanced or implicitly locked. Native nested getters may reuse an unlocked
target; callers that require a stable target epoch must lock it as source usage
requires. Existing mark primitives and wire/root getters are reused. Work locks
are released on errors while earlier mark writes, output writes and unrelated
model metadata are retained.

The source Shell-to-Face guard checks the Face's work mark, then stamps its
**Faceuse** (line1911). Region/Shell collection can consequently contain duplicate
Face entries. This behavior and its downstream diagnostic order are preserved;
the output comment's uniqueness claim is not treated as permission to fix it.
The marked non-root branch also leaves unrequested work pointers NULL and then
dereferences all six. Rust returns a checked error with the defined marking
prefix retained. That native undefined case cannot establish a passing result.
Root partial marked outputs and invalid-type early returns remain supported.

`assert_subtopology` follows `SmBrep.cpp`35303--35495. It collects every array
before asserting Region, Shell, Face, Loop, Edge and Vertex in that order. It
passes no parent Given requests and no recursive walk. Source summary-report and
composite branches stay disabled. An ordinary collector status is ignored as
native; checked graph failures and missing geometric dependencies propagate.
Region, Shell, Face, Loop and Edge walk paths share this dispatcher; the use and
Vertex routines retain their native ignored-walk behavior. Source geometry and
parameter definitions are not touched by collection.

The maintained comparison exercises every unmarked output mask, reset/append,
seed duplicates, all mark types, target lock/no-lock, premarking, repeat calls,
vertex loops, wire/shell vertices, holes and shared/manifold/spine/trihedral faces.
Output IDs/order, mark epochs and counter advances are checked. Independent
triangle invariants and checked-failure/lock-release tests supplement SDK parity.
Selected local walks still depend on their actual geometric callees and do not
qualify whole-model or native STEP acceptance.

```text
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/brep_subtopology_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-subtopology.local.json
```

The plural entry now has its own implementation and qualification below;
single-receiver results alone do not establish its distribution/alias contract.


## Batch inputs and output-array aliases

`get_subtopologies` follows `SmTopologyTraverser.cpp`2094--2181 at `d8ed45a`.
`SubTopologyInput::Objects` supplies a separate heterogeneous input slice;
`OutputRegions`/`OutputShells`/`OutputFaces`/`OutputLoops`/`OutputEdges`/
`OutputVertices` explicitly select an existing output as the actual input.
These selectors preserve native shared-array behavior without unsafe Rust aliasing
or heterogeneous pointer-array casts. All identities belong to the borrowed model.

The batch routine returns before reset for empty input or no requested output.
Otherwise it resets requested arrays when asked, distributes inputs with **Add**,
then delegates to the singular routine with reset=false and **all six** working
arrays. Omitted outputs use local arrays and can therefore affect traversal and
marks before being discarded. Unlike singular marked partial outputs, batch
marked partial outputs are defined. No new traversal algorithm or sorting is used.

A single Brep overrides the other roots and triggers the singular root's reset.
A second Brep returns native status1007 with earlier distribution writes retained.
Non-Brep inputs are included in their typed working arrays; duplicates and even
an input Vertex can remain in output despite the source uniqueness/descendant
comments. Invalid use types are ignored during distribution. If the first input
is invalid and no Brep overrides it, singular resets the work arrays and returns
SM_ERR; a later invalid use does not select the receiver.

Actual input/output aliasing skips distribution and reads the selected array
**after** reset. Append aliases retain their actual input storage and seed order.
An initially empty alias returns before clearing any other output. Reset of a
nonempty alias clears every requested output, then native reads the now-empty
first pointer. `SmTArray.h`1075--1084 zeroes those entries, while its Release
operator[]454--478 does not bound the read: the SDK access violation is recorded
separately. Rust keeps the reset outputs and returns a checked error; it never
continues on a saved copy of the old input. A selector for an absent output is
also a checked error. Ordinary batch status failures retain their partial outputs.

The batch SDK comparison covers all output masks, mixed/duplicate/empty inputs,
Brep precedence and duplicate-Brep errors, marked partial outputs, each typed
alias, append/reset, target locking and repeated calls over the synthetic graph
roles. Its status/arrays/order and current mark epochs/counter advances are exact;
independent distribution and early-return checks supplement parity. The unsafe
native reset-alias case is not a passing or deferred numerical result. This
runtime topology capability does not qualify general STEP import or the full SDK.

```text
cargo build --locked -p solidcore --example brep_validation_probe
python scripts/brep_batch_subtopology_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-batch-subtopology.local.json
```

## Complete-curve projection and owned knot effects

`intersect::surface_drop_lib::p3` implements the maintained fork's actual
`SmSurface::DropCurve` body (`SmSurfaceDropLib.cpp`4194--6255), with virtual Plane
dispatch. This is separate from `DropAndTrimCurve`/`DoTrace`: a complete projection
can return two seam images, or no curves when the walk crosses a boundary/pole.
The existing tracer and fillet callers now call the shared implementation; the
previous `DropAndTrimCurve` bridge to `DoTrace` is also wired.
Plane's failed control-polygon shortcut explicitly calls the base body while
retaining its concrete evaluator; re-entering virtual Plane dispatch would recurse.

The base path retains domain fallback/containment, original curve intervals,
per-axis UV tolerances, duplicate/seam start-point ordering, pole treatment,
iso shortcuts, curve-cache spans, six-try midpoint subdivision, native short-span
acceptance, boundary tangent/polygon handling, sampled normal-distance/deviation
checks, and all four failure-record scopes. It preserves the source's repeated
X-domain degeneracy test and signed-distance filtering quirks: the postpass only
culls when more than one curve exists and tests distance strictly below tolerance.
Those are compatibility behavior, not new repair or import-quality allowances.

Candidates are owned values and are released on error. Temporary out-of-bounds
evaluation uses exact concrete snapshots; caller geometry/parameters are not
normalized. The model bridge materializes the original edge curve's lazy NURB
at the native `DropIsoCurve::GetKnots` or curve-cache query site, including a
failed walk. It does not move that effect ahead of domain/start-point rejection.
Source ownership handles remain in failure reports. Borrowed standalone calls
cannot mutate an external owner's cache. Shared cache-manager checkout, persistent
cache aliases, and all owned solver-context paths remain separate requirements.
Value-only gap-query errors propagate where failed native output variables are
inaccessible; no zero-error success is invented. Iterative subdivision has an
explicit resource bound and rejects excess work rather than relaxing tolerance.

The direct SDK observer uses real unowned raw B-spline and Plane receivers:
flat/saddle patches, a rational cylinder with two seam images, a conical pole,
nonlinear curves, clipped intervals, subdomains, outside/fallback domains, and
small/large/translated coordinates. It compares status, failure identity, UV
degree/dimension/knots/multiplicities/sample order, sampled gap/deviation and
temporary flags at `1e-9` relative with no absolute floor. Independent analytic
checks establish original parameters and the two cylinder seam sides. These are
synthetic operation comparisons, not imported-model counts, continuous bounds,
complete native BRep validity, or promotion of the base FFI unit's green gate.

`DoTrace` now receives actual analytic iso curves from the cache. Deeper owned
tolerance resolution is still incomplete. Existing cleanup
regressions check the actual next dependency while retaining model flag/owner
restoration, instead of continuing to expect the removed entrypoint stubs.
The traced B-spline cases also reach the pre-existing classification-reset
bridge (`intersect.curve_class_p4`), whose geometry handles still lack a complete
resolver. The NLib rational-curve-to-UV helper still lacks its originating Kernel
parameter and remains an explicit projection dependency; it must not manufacture
a default context. These are outstanding migration work, not successful cases.

```text
cargo build --locked -p solidcore --example brep_drop_curve_probe --example brep_validation_probe
python scripts/build_brep_constructor_reference.py --source <cpp-fork> --sdk <existing-sdk> --toolchain-env <local-env> --output bin/drop-curve-reference --probe drop_curve
python scripts/brep_drop_curve_compare.py --cpp bin/drop-curve-reference/brep-drop_curve-reference.exe --rust bin/debug/examples/brep_drop_curve_probe.exe --receipt bin/brep-drop-curve.local.json
```

## Analytic iso-curve cache dispatch

`IsoBoundaryCurves.curves`, the stored cache array, and patch-boundary extraction
now retain `Curve` values. Plane produces actual Lines. Revolution sweep curves
retain the native Circle/Ellipse type; generator-direction extraction remains a
B-spline, even when its generator was an analytic Line. Boundary boxes, lengths,
silhouette properties and intersections dispatch through those actual kinds.
The caller's approximation tolerance reaches the general surface extractor;
the size-tolerance helper retains its separate native literal-zero argument.

Natural-domain cache eligibility retains the original tolerance rule: a finer
request returns uncached curves without replacing the old cached objects, while
updating the recorded tolerance. Read-only return values remain copies of the
cache; shared-pointer mutation/lazy-cache alias equivalence is not qualified.

The revolution constructor performs its native virtual generator plane test and
materializes the owned generator at that query, rather than querying an empty
Line B-spline base. Owning representation preparation precedes the observer's
borrowed natural-domain query, matching native `GetNaturalUVDomain`'s lazy effect.
Existing NURBs/definitions and raw parameter domains are retained.

Mapped-curve callers now supply the concrete surface snapshot to `CrvOnSurf`.
They no longer impose a B-spline-only restriction on its already polymorphic
implementation. Unresolved solver/geometry-handle contexts still return their
actual dependency errors; this does not establish general import acceptance.

The SDK observer compares actual type IDs, analytic Line identity, original
intervals, virtual lengths, points/tangents and repeated cache eligibility over
Plane, polynomial/rational BSD, poles, revolution and swapped revolution, with
subdomains and small/large/translated inputs. Relative comparisons retain no
absolute floor. Analytic shape checks supplement parity. These are geometry
operations, never a count of successfully imported STEP models.

```text
python scripts/brep_iso_cache_compare.py --cpp <drop-curve-observer> --rust bin/debug/examples/brep_drop_curve_probe.exe --receipt bin/brep-iso-cache.local.json
```

## Root first-coincidence query

The root LEVEL2 gate now executes its actual first-witness coincidence search.
A private typed witness separates ordered geometry traversal from the single
native report mapping. Reports do not determine the truth value. Vertex equality
at the root tolerance remains inclusive, while each edge/face test retains its
own native strict comparison and subordinate combined tolerance. Pair traversal,
first-hit exit and no-report behavior are independent of the diagnostic sink.

The selected root SDK comparison now completes all of its cases, including the
previously deferred LEVEL2 root guard. Separate Rust regressions exercise a real
duplicate vertex, first-pair order, equality and the original root/list3 report
identities. These do not qualify every coincidence kind or malformed-geometry
failure path. The native public all-witness/independent-output-array interface is
not implemented by this private root helper. Edge/face `CurveOnClassify` remains
an explicit dependency. Native's ignored peer surface-evaluation failure can
continue with a retained output point; this helper currently stops that receiver
search instead, so complete failure-path equivalence remains unestablished.

```text
python scripts/brep_root_validation_compare.py --cpp <validation-observer> --rust bin/debug/examples/brep_validation_probe.exe --receipt bin/brep-root-validation.local.json
```

## Owning ray and coincidence execution

The native STEP bridge now supplies actual typed BRep owners to the existing
infinite-region ray search and first edge/face coincidence search. Its private
execution adapters share the original algorithms with the existing borrowed
entrypoints. Point/curve duplicate comparison uses the defining curve's natural
domain, and sorted solutions use their actual vertex/edge/face/geometry working
tolerances. Foreign handles still refuse; system tolerance is used only in the
native branches that explicitly request it.

Face-owned surface boxes expand normal/pseudo bounds by the actual face tolerance,
leaving polar bounds unchanged. Both coincidence and face-query callers use the
same final-expansion helper. Each projection/classification releases its geometry
borrow before running the required mutable owner/cache operation. The owning ray
path restores its spatial cache after success or failure; it does not infer the
answer from a stored infinite-region flag.

The public analytic material box completes pointer and LEVEL2 descendant checks
through this path. Its geometry/topology construction and native validity do not
establish general STEP acceptance. Remaining general point-cache owner routes, periodic
face classification, other concrete subclass assertions, remaining query dispatches
and the other documented failure-path limitations still require implementation.

The owning generic ray now builds its surface cache through the live BRep context
before entering the shared line/tree solver. Actual Kernel and face tolerance
reach the LSI leaf's singularity query without changing the public solver layout.
A B-spline triangle regression covers this route with distinct system and face
tolerances, including Boolean-delayed cache construction. Typed cache-use metadata
is checked in its scoped namespace. This does not complete the distinct owning
point-maximize cache route, periodic gap/classification or full cache-manager reuse.

Surface duplicate comparison now resolves matching non-null tree handles against
the actual borrowed defining domains. The original native null-object and
parameter-index behavior is retained; poles/seams use the executing Kernel.
The legacy context-free comparison refuses a context-dependent branch rather
than manufacturing a default. Candidate enumeration is not a guarantee of global
extrema; the independent public maximization limitation is recorded in findings.

Concrete Cone, Cylinder and Torus assertions now execute their own rules and
the required Revolution/B-spline parent chain, with native Given-level rules,
report indices and cascading removal. The actual parameterization knot query
materializes only missing native NURBs and restores their owning slot on failure.
Nested generator containment is intrinsic to the uniquely value-owned Rust
representation; conflicting external owner metadata is rejected. This does not
represent C++ independently corruptible generator/context backlinks. The public
SDK control comparison covers four actual geometry kinds with and without a
face owner; it does not qualify every malformed primitive or failure-label path.

The owning periodic seam path now shares its ordered controller with the borrowed
entrypoint. Vertex/edge intersections, lazy gaps, UV sectors and binormals use the
actual mutable BRep at their native call positions. Short geometry borrows end
before mutations; the owning path does not clone the BRep. The legacy borrowed
adapter retains its separate snapshot behavior. Source Stage2 partial outputs and
the truthy `SM_SUCCESS` boolean convention remain distinct from the seam enums.
Actual curve domains resolve CCI duplicate comparisons; typed edge intervals and
connectivity retain foreign-arena/error guards. Missing two-surface refinement and
tolerance mutation still refuse at their actual execution points.

Circle and Ellipse assertions run the owning B-spline parent and their original
concrete/polar rules, preserving Circle's GIVEN parent-skip convention. Tight face
bounds have a mutable-owner entry sharing the original extrema/classification
algorithm; directed global and local solver constructors receive the real face
tolerance before their reference-plane bounding-box queries. Numerical native
validity and finite material measurements still do not establish general source
equivalence or a continuous error bound.
