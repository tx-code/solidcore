# Strict selected-part ownership and dependency closure

The opt-in part/closure checkpoint migrates the strict C++ `Product`, `Closure`
and their selected-property, administrative, presentation and supporting-geometry
prerequisites from `source/STEP_SEMANTICS/src/StepSemantics.cpp`, source snapshot
`d8ed45ab85d52dba07660cafdceacafdca7217ed`. Original entity identities, values,
unit/parameter conventions, field provenance and source order remain immutable.

This is source ownership, **not geometry qualification**. The independently
selected C++ `GeometryDependencies` phase, planar/global rules, interoperability
approval, native construction and repair remain unrun. The report states that
scope explicitly. For example a reference cycle terminates safely during
ownership traversal, but a geometric-definition cycle still needs its later
geometry prerequisite check. A source `MaterialSolid` role does not certify
closed/manifold/oriented geometry or create a native material object.

## API and execution

```sh
cargo run -p solidcore-step --features cli --bin step-semantic-inspect -- \
  --fixture-limits --part-and-closure \
  crates/solidcore-step/tests/fixtures/plane.step
```

Run from the repository root. Use `--general-brep-source` to select the existing
general source catalogs; this does not relax source errors or authenticate a
schema edition. In Rust set `Options.check_part_and_closure=true`. The default
remains the earlier types/units checkpoint, with ownership `NotRun` and no
ownership queries presented as false/empty proof.

The selected phases are Binding → Types → existing UnitDependencies (outside P0)
→ Product/Closure → selected UnitContext. GeometryDependencies is explicitly
`NotRun`, not silently treated as passed. The selected part proves its own actual
unit-context identity. If `Options.physical_context` / `--unit-context` is also
provided, it must equal that proved context; it cannot replace body units with
a property/presentation context. A conflicting request is `InvalidRequest`.

An expanded report uses `solidcore.step.part-closure.v1` and
`PartAndClosureCheckpoint`; unexpanded reports keep `solidcore.step.types-units.v1`.
Both retain source-bound report compilation, actual UTF-8 byte budgets and a final
LF. Report-budget failure publishes no partial JSON. A failed requested phase
publishes no partially checked semantic snapshot, even if earlier roles or
properties were provisionally established. The complete original capture and
stable diagnostic facts remain available through their existing source owner.

Expanded CLI exits match C++ source semantics: 0 checked checkpoint, **7 deferred**,
1 rejected source, 2 unsupported source/profile, 3 resource limit, 4 capture I/O,
5 report budget, 6 infrastructure/output error, 64 invalid request. A source
inspection failure retains its nested source status and propagates the applicable
source exit code. The earlier types/units-only mode keeps its original exit-code
behavior, including its reported-Deferred exit 0. No exit qualifies a native model.
OS write errors can leave a partial external stdout write; successful completion
is required for publication. I/O errors retain the existing Rust CLI stderr-only
policy rather than claiming C++ JSON byte compatibility.

## Identity and role rules

- Select the unique application/APD/product context/definition context/product/
  formation/PD/PDS/SDR chain by actual reference identity. Equal product labels
  do not merge records, and APD status/year/schema text does not select an edition.
- P0 retains its unique representation/context guard. General catalogs select
  the SDR root, its unambiguous exact-body association, optional distinct linked
  wireframe, and AP214's directed constructive-geometry association. No relation
  is treated as a transform, scale, equivalence or Boolean composition.
- Linked physical representations must retain the required identical source
  context identity. Supporting geometry stays a distinct source role; wrappers,
  wireframes, surfaces and placements are not merged into a material boundary.
- Preserve `MaterialSolid`, `OpenSurfaceModel` and `ClosedSurfaceModel` as source
  claims. A closed shell inside SBSM remains `ClosedSurfaceModel`.
- Preserve explicit outer-bound requirements. In explicit general scope, a face
  with ordinary bounds and no explicit outer may retain its source type and be
  deferred for later region evidence. A later orphan/failure still overrides
  that provisional deferral; no FACE_BOUND is relabelled or guessed as outer.

## Two distinct traversals

Broad ownership follows every resolved DATA reference from the proved source
roles. Its stack, outgoing-edge order, visited checks and charged visits match
C++; shared definitions and cycles do not trigger recursive graph expansion.
Unreachable records are refused together, with the first original occurrence as
primary and every remaining orphan's original ID span as related evidence.
Records are not silently discarded, normalized into another role or repaired.

Representation-use traversal starts at actual representation items and follows
RI dependencies, plus the admitted COMPOSITE_CURVE → segment → parent bridge.
It does not turn arbitrary founded/style/unit/management paths into physical
geometry. `Snapshot::used_by_representation` further filters results to RIs;
an ordinary composite segment can be traversed but is not reported as an RI.
Part attachment and “used by any representation” remain different facts.

`Snapshot::used_in(target_id, owner_type, field)` exposes validated inverse roles
as original source **reference indices**, preserving repeated LIST/BAG occurrences
and order. Unknown targets, unregistered types and undeclared roles return `None`,
not a proved empty set. Original source/value storage is shared, never rebuilt as
a second interpreted entity graph. Owning snapshots and field views survive
dropping their originating outcomes.

## Metadata cannot manufacture ownership

Properties require real PDR links to the selected PD/PDS and the admitted plain
metadata representation/item boundary. Every original shared link, value, nominal
unit and metadata context is retained. Property contexts cannot replace body
units; geometry/style records cannot be smuggled through a descriptive property.
VRI's assigned-context prerequisite checks all relevant using representations.

General-property associations, style overrides, selected presentation targets,
administrative assignments, category hierarchies and invisibility retain their
existing strict C++ checks and ordering. Administrative roots use the original
fixed-point snapshots: an unattached cycle cannot create its own ownership.
Category/override cycles use iterative checks. Invisibility targets must already
be owned. A style/presentation/management relationship cannot introduce another
shape or suppress a required source record. Necessary style sizes, names, nominal
type constraints and exact ranges are checked without applying a renderer.

No `PreserveNonGeometric` exceptions or carrier/schema assumptions are enabled.
The [source carrier API](CARRIER.md) can now retain opaque tokens and immutable
legacy interpretation receipts. Strict typed/ownership checkpoints refuse legacy
assumptions and retain unsupported diagnostics for unavailable DATA text; those
source options cannot silently manufacture a checked product identity or role.
Those require their later original nonphysical-exclusion and approval contracts.
That separate explicit path is now available through [source interoperability](INTEROPERABILITY.md);
the strict checkpoint described here retains its original behavior and scope.
Unknown physical members and strict source errors remain fatal.

## Compatibility and validation

Stable compatibility facts are diagnostic key, stage/classification, primary and
related spans, owner/component/path, target, declared field/type, expected/actual
value, responsibility, role order, source memberships, property attachments,
representation-use results and actual visit counters. Human message prose and
JSON-only representation differences are not byte-compatibility promises.

The existing local C++ test adapter now accepts `--part-and-closure` and invokes
the original `Product` and `Closure` bodies before UnitContext. The access-only
copied Checker and all original C++ source files remain unchanged in behavior.
For P0, `scripts/step_part_closure_compare.py --cpp-full-p0` additionally compares
directly with the existing complete `step_semantic_inspect` executable, including
diagnostics, populations, role identities, typed slots, units, counters and exits.
General-profile comparisons retain the explicit unrun geometry-phase boundary.

Additional public input coverage can be captured locally by
`scripts/capture_step_contract_inputs.py` (the original C++ contract suite) and
`scripts/capture_step_python_contract_inputs.py` (an explicit list of public source
contract tests). Both still execute the original tests/Check results unchanged;
they retain **inputs only**, never use expected answers as actual observations.
Only applicable strict/default-carrier requests are replayed for this checkpoint.
Assumptions attached to the immutable C++ source owner cannot be hidden by mutable
outcome annotations. The original checkout is read-only, no model library is
scanned, and captured files/executables/receipts stay in ignored local output.

```sh
python -B scripts/step_part_closure_compare.py \
  --cpp bin/part-reference/step-type-unit-reference.exe \
  --rust bin/debug/step-semantic-inspect \
  --cpp-full-p0 /path/to/step_semantic_inspect \
  --corpus /path/to/ap203e2-planar/v1/cases \
  --captured bin/public-contract-inputs
```

The runner compares exact and just-insufficient budgets inside the ownership
phase for each profile, not just an end-of-request limit. Independent Rust tests
assert source graph identity, shell/material role distinction, inverse occurrences,
shared properties/contexts, linked geometry, administrative/invisibility boundaries,
deferral/failure precedence, source cycles/shared DAGs, no partial snapshot and
report/exit behavior. Existing source and type/unit regression remain separate.
All private inputs and any derived reports retain their local confidentiality.
