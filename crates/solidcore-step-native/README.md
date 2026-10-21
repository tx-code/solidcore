# Native Rust STEP bridge

This module connects the existing immutable source/semantic/planar checkpoints
to the actual Rust `solidcore` kernel. It uses no foreign reader or kernel.
It is an initial construction and affine-certification increment, **not general
STEP import parity** and not a completed real-model milestone.

## Ownership and responsibilities

`Plan` privately owns the source snapshot and its checked evidence. It retains
one exact interpretation of source vertices, units and defining LINE endpoints;
construction and certification consume the same definitions. Native endpoint
projection does not move the source vertex or replace the source LINE with a fit.
The original `EDGE_CURVE.same_sense` and occurrence orientations remain distinct.
Construction reads the source checker's private immutable success witness.
Editing the public diagnostic projection or rewriting its status cannot mint
eligibility or replace the geometry that actually passed the source checks.

`construct_planar` returns an unpublished, owning `Candidate`. Current construction
accepts one/two open planar faces or a six-face closed boundary. Shared source
vertex/edge identities are reused. Closed surface-model roles remain void; a
geometrically identical material box retains its separate material claim.
Six faces alone do not prove a box, a valid solid or correct native material volume.
No repair or stitching runs. Failures return no partial native model.

`Candidate::validate_native` executes the actual pointer validator and level-two
descendant walk. Both boolean/status and reports must pass. It consumes the
candidate on failure. Native validity alone never establishes source equivalence.

`Candidate::accept_open_planar` currently qualifies **one open planar face** only.
The application must supply a named `AcceptanceQuality` with separate positive
definition, derived-UV, consistency and working-tolerance budgets in millimeters.
There are no production defaults. The owning `Imported` result offers read-only
native/source/correspondence access and its continuous certificate. Closed shapes,
material-volume claims and general curves/surfaces need additional proofs.

The affine certificate checks directed topology (allowing only cyclic rotation
of a loop's start), source/native identities, native [0,1] edge mapping, actual
analytic and stored polynomial definitions, UV-domain containment, continuous
UV lifts and endpoint consistency. Polynomial bilinear surface differences are
bounded over their entire rectangle by their four control-corner differences;
affine line differences are bounded over their complete interval by their ends.
These are convexity arguments, not sampled-error estimates. Every stored
representation must meet the proven polynomial degree/knot restrictions.

`bounds` owns outward software-binary64 arithmetic and exact rational budget
comparisons. NaN, infinity, unavailable intervals and overflow cannot disappear
through `f64::max`; unavailable evidence yields an infinite bound and rejection.
This does not qualify general kernel floating-environment parity. No validator
may silently grow the candidate's selected working tolerance in this strict path.

## Current limits

The operation counter bounds charged construction/checkpoint actions; it is not
a hard CPU, memory or cancellation guarantee for all nested kernel operations.
Exact source caches add retained data per unique source vertex/edge. Successful
planar outcomes additionally retain an immutable evidence copy so their existing
mutable diagnostic interface can remain compatible; source bytes are shared.
This is not a total-process memory guarantee. Original source and native objects
remain owned until their candidate/result is dropped;
ordinary Rust destruction releases unpublished objects. Hard process cancellation,
application workspace publication and the maintained full diagnostic/report
contract are not yet migrated. No cross-process native model transport is implied.

The optional CLI deliberately reports `NativeConstructed`, `accepted_import:false`
and unrun import stages. It does not expose the library certificate as a general
model-import success:

```sh
cargo run -p solidcore-step-native --features cli --bin step-native-import -- \
  --fixture-limits crates/solidcore-step/tests/fixtures/plane.step
```

`--fixture-limits` and test quality values are synthetic-development selections.
They are not production quality or resource configurations. Private files,
identifying inventories and derived geometry/results must remain local.

## Validation

```sh
cargo test -p solidcore-step-native --features cli
cargo clippy -p solidcore-step-native --all-targets --features cli -- -D warnings
cargo check -p solidcore-step-native --lib --target wasm32-unknown-unknown
```

Public synthetic tests exercise actual reader-to-native construction, both source
roles for the same closed boundary, immutable source retention, centimeter units,
reversed source EDGE/OE senses, continuous-error/working-tolerance refusal and
construction/final-gate N−1/N budgets. Independent bit/rational tests cover
nonfinite values, subnormals and threshold neighbors. Kernel coincidence checks
have separate native differential evidence; they do not establish all native
failure-path equivalence or real STEP model coverage.

## Explicit general development construction

`general-development` adds `construct_general` through a separate source route.
It always executes the complete typed source checks with `GeneralBrep` scope,
including geometry dependencies and part/dependency closure; caller flags cannot
disable these checks. It does not borrow the P0 planar profile's qualification.
The current construction increment requires one material-solid closed shell.
An identical surface-model boundary remains a surface model and is refused by
this material-only route rather than promoted.

The owning candidate retains the original source outcome and diagnostics, exact
unit dependencies, defining curve parameter mappings, directed loop occurrences,
shared native vertex/edge identities, and computational chart-cut correspondence.
`Candidate::edges()` and `Imported::edges()` return every target piece for each
source edge, ordered by the original defining-curve parameter. Strict planar
certification requires exactly one piece. General two-ring cylindrical chart
normalization can partition a closed circle without changing its complete
analytic/NURBS definition or moving either original source vertex. Cuts are
planned globally before native construction and reused by all neighboring faces;
reversed uses reverse both piece order and direction. `derived_vertices()` records
the source edge and original native basis parameter of each added chart point.
Source loop receipts retain the original unsplit directed source boundaries.
Additional full-curve copies are charged before copying; unsplit curves move
without an added copy. Material and trim checkpoints account for every piece and
derived vertex. This does not grant general continuous qualification.

LINE, conics, trimmed conics and supported rational B-splines are materialized as
defining native curves. Supported receivers retain their actual plane, cylinder,
cone, torus or B-spline kind. Derived UV observations and native tolerance/growth
checks are separate from continuous approximation bounds. No fitting or repair is
silently substituted. Unsupported required content returns an error with no model.

Use `--features cli,general-development` for the CLI route, then explicitly select
`--general-development --quality FILE.json`. Quality JSON contains exactly these
eight **string** fields: `id`, `definition_mm`, `consistency_mm`, `derived_uv_mm`,
`query_mm`, `initial_work_tolerance_mm`, `maximum_work_tolerance_mm`, and
`maximum_tolerance_growth_mm`. All lengths are millimeters; the growth allowance
may be zero. The named configuration and source uncertainty remain distinct.

General interoperability is also opt-in: `--interop` selects the existing proven
nongeometric exceptions, and `--ap214-unregistered-as-2010` requests the existing
unverified schema interpretation. Independently selected `--opaque-header-name`,
`--opaque-data-strings`, and `--level1-as-corrected-encoding` require general plus
interoperability mode. They select the existing source reader's byte-preservation
and interpretation receipts. Opaque bytes do not become invented text or approved
typed fields; a level-1 declaration remains unchanged and unverified. Interop alone
does not enable any carrier option. Physical unit/type checks remain in force.

`--development-ap214-angle-dimensions` independently requests the verified
development unit policy in general mode. It is disabled by default and does not
expand the nongeometric interoperability policy. A supported explicit AP214
plane-angle dimensions reference may continue only after the complete source
pipeline verifies its dimensional and conversion dependencies. Original bytes,
conversion factors and source diagnostics remain owned; the separate receipt
does not establish valid source syntax or formal AP conformance. Such results
must remain separate from strict qualification. A requested option is never proof
that it applied or that native construction/acceptance completed.

After native validation, `measure_general_trims` checks the actual installed UV
curves at the original common edge parameter, using both knot sets and nine
samples per union span. It rejects incompatible domains, nonfinite evaluations
and measured gaps exceeding the selected budget. Its optional owning receipt
remains absent until every use succeeds. These are finite measurements, not a
continuous bound. A later owner UV query invalidates this receipt before it can
change derived state; callers must measure the complete result again.

`measure_general_material(relative_accuracy)` adds numerical tight bounds, area,
material volume, volume centroid, per-face area centroids and unique target-edge
lengths. It requires the previous native and trim checkpoints, the retained
material region and complete source correspondence. The caller supplies a separate
relative integration control in `[1e-8,1e-4]`, as in the maintained C++ contract;
no value is inferred from source uncertainty or millimeter budgets. Actual owner
queries retain lazy state in the candidate, and every final trim is measured again
after the last consumer. Measurement charges `2*F+E+1`; the final refresh separately
charges each edge use. All results attach only after the whole consuming call
succeeds. Revalidation or a later owning UV query invalidates both receipts.

The CLI selects this checkpoint with `--measure-material-accuracy VALUE`. It keeps
`accepted_import:false` and distinguishes numerical observations from continuous
error bounds. Public box, explicit-seam cylinder, phase-mismatched two-ring cylinder and annular-cylinder tests use
independent analytic area/volume/centroid truth. They do not qualify arbitrary
general sources or production tolerances.

This increment still returns an unpublished **candidate**. General qualification,
complete continuous correspondence qualification,
application cancellation/publication, and several kernel back-edges remain to be
migrated. Concrete cone/cylinder/torus validators execute their actual rules and
the revolution/B-spline parent chain. Uniquely value-owned generator containment
does not qualify C++ independently corruptible owner/context backlinks. The
optional CLI keeps `accepted_import:false`. Neither this route nor a viewer
screenshot counts toward a real-model import acceptance target. Failed source
checks also retain the original independent development, nongeometric and schema
diagnostics in `Error::Source`; no missing snapshot silently erases them.

```sh
cargo test -p solidcore-step-native --features cli,general-development
cargo clippy -p solidcore-step-native --all-targets --features cli,general-development -- -D warnings
```

Authoritative references are the maintained C++ fork at `d8ed45a`,
`source/STEP_IMPORT/README.md`, `StepImportPlan.cpp`, `StepImportNative.cpp` and
`StepImportInterval.h`; the source checkpoint contract remains in
[`solidcore-step`](../solidcore-step/README.md).
