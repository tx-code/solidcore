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

Authoritative references are the maintained C++ fork at `d8ed45a`,
`source/STEP_IMPORT/README.md`, `StepImportPlan.cpp`, `StepImportNative.cpp` and
`StepImportInterval.h`; the source checkpoint contract remains in
[`solidcore-step`](../solidcore-step/README.md).
