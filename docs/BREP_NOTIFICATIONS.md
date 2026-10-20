# BRep notifications

The maintained reference is `SmBrep.cpp` at `d8ed45a`: `Notify`,
`sm_AddToSpatialTree` and `sm_RmFromSpatialTree`, followed by the existing
`SmSAGObject`, `SmTopology`, `SmAObject` and context callback layers.

`topology::brep::p2::notify_owned` now executes that root chain on the actual model. The
owning adapter supplies the shared upstream dispatcher with its notification
marks/flags/user fields, bidirectional attribute
store and optional real BRep spatial cache. These values persist between calls.
Geometry-owner key lookup and region lookup remain explicit services, so
notification/attribute keys do not overwrite existing geometry owner identities.
The arena graph remains authoritative for ownership/list links; the notification
field records are not an alternative graph or input to a topology validator.

## Identity and ownership

Notification references belong to one model. The root uses the model key;
children use their concrete topology type and arena index. Set a distinct root
key before attaching attributes or retaining references. Cross-model root events
return an explicit missing-resolver error; foreign child identities must not be
inferred from equal model-local indices. A shared model/geometry registry,
cross-model copy/attribute transfer and fillet-subclass spatial resolution remain
required work.

The value-snapshot `Clone` retains metadata/attributes and drops its spatial
cache. This preserves independent cache ownership but does not implement the
complete native `SmBrep::Copy` protocol. Direct geometry edits must still notify
the owning model. Existing element operations using external `NotifyEnv` stores
have not all been bridged to this owning state yet. The generic dispatcher
refuses to discard configured owning state through an arena-only adapter; it
reports the missing producer bridge explicitly.

## Execution order

1. Perform the root cache action. Pre-edit/destruction drop the actual cache;
   split/add insert affected vertices, curves or surfaces; merge/remove delete
   the specified object from its spatial tree. Other events retain the cache.
2. Propagate the original topology flags/general marks and user fields through
   the existing shared base implementation.
3. Run the existing attribute handlers, including copies, merges and destruction.
4. Restore the owning runtime state, then invoke the actual `Kernel` context's
   user callback followed by its system callback. Callback statuses are ignored,
   matching native behavior; no replacement context is fabricated.

Cache insertion uses the actual existing box queries and per-member working
tolerances. Ordinary insertion failure deletes the old cache before attempting
the real factory. Removal preserves geometry-owner guards, the wire edge's full
natural curve interval and the native curve-box expansion default. Missing
callees propagate separately. Existing geometry-box result APIs do not expose
all partial outputs after ordinary query failure; those exceptional numerical
paths are not qualified by the positive notification matrix.

Errors retain already-applied native effects and restore the model's owning
state. This operation is not an import transaction or authorization for repair.
Graphics display lists, global registration/checkout and automatic propagation
from every element mutation remain outside this increment.

## Evidence

The host observers call actual SDK `SmBrep::Notify` and the actual Rust root
implementation on a connected synthetic planar triangle. The matrix varies
16 events, cache presence and repeated calls. It compares spatial membership,
flags, general marks, long-attribute values/order and every user/system callback
argument. Both callbacks deliberately return failure to verify the ignored-status
contract. It does not qualify nonzero user-index behavior of every compile
configuration, callback-driven model edits, or every exceptional numerical path.

```text
cargo build --locked -p solidcore --example brep_notification_probe
python scripts/build_brep_constructor_reference.py --source <cpp-checkout> --sdk <release-sdk> --toolchain-env <msvc-env.json> --output bin/notification-reference --probe notification
python scripts/brep_notification_compare.py --cpp bin/notification-reference/brep-notification-reference.exe --rust bin/debug/examples/brep_notification_probe.exe --receipt bin/brep-notification-validation.local.json
```

Stitch's root pre-edit now uses this real state/context path. Its canonical
already-connected constructor fixtures reach native comparisons beyond the old
root placeholder. That does not establish general healing, full root/local
`AssertValid`, strict native STEP import or the
[complete SDK migration](FORK_MIGRATION.md). Receipts and private models remain local.


## Integration with upstream

The owning adapter delegates the upstream `p2` notification algorithm through
`NotifyServices`. Metadata identities are resolved from validated registrations;
geometry-owner keys and live regions are supplied independently. The original
arena-only interfaces remain available, with an explicit refusal when they would
discard configured owning state.

The upstream iso-curve lift now retains its analytic curve kind. The maintained
lazy analytic NURB view remains at the input boundary. Native radial constructor
comparisons also exposed a missing owner assignment in `SetUVTrimCurve`; the
setter now binds the actual SAME-oriented holder without changing the curve's
parameters or definition. Full constructor graphs and UV ownership are compared
separately from pointer and notification checks.
