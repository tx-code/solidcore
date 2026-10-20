# Explicit source interoperability and schema interpretation

This ports the maintained source policy, exact schema alternatives and final
`ApprovePreservedMetadata` contract from `source/STEP_SEMANTICS` at
`d8ed45ab85d52dba07660cafdceacafdca7217ed`. It preserves source errors with evidence;
it does not repair definitions or establish native BRep, AP/global conformance,
planar validity, continuous geometric accuracy or downstream modeling fitness.

## Request and execution

`Options.source_policy` defaults to `Strict`. Explicit
`SourcePolicy::PreserveNonGeometric` executes the full migrated source sequence:
Binding → Types → UnitDependencies → GeometryDependencies → Product/Closure →
actual physical UnitContext → final interoperability approval. P0 retains its
original unrun dependency/approval boundaries and cannot acquire these exceptions.
A shortened type/unit or ownership checkpoint cannot publish provisional issues.
No source byte, field, type, nominal quantity, reference, UV/parameter convention
or source shape role is rewritten. Strict requests keep their existing behavior.

`Limits.preserved_diagnostics` is a separate explicit storage quota, shared by
schema interpretation and metadata diagnostics. The fixture preset is 1024, not
a production policy. Unit prerequisites retain their original earlier capacity
check and field diagnostic; the final combined check still counts interpretations.
Visit charging, predicate short-circuiting and failure ordering remain source
contracts. No diagnostic is silently dropped to fit either quota.
Rust callers using literal `Limits` initializers must supply the new capacity or
use an explicitly chosen preset with struct-update syntax. This adds a request
parameter, not an implicit geometry precision or repair allowance.

The CLI adds `--interop-nongeometric`, `--preserved-diagnostics N`, two schema
choices and the existing opaque/legacy carrier choices. All nine source resource
overrides are available. An explicit interpretation requires both general scope
and interoperability, even if normal binding would leave it unused:

```powershell
cargo run -p solidcore-step --features cli --bin step-semantic-inspect -- --fixture-limits --general-brep-source --interop-nongeometric --interpret-unregistered-ap214-2010 public.step
```

The preset is for public synthetic testing. No production quality or tolerance
configuration is inferred. Both `Checked` and `CheckedWithDiagnostics` exit 0;
`Deferred` retains exit 7 for the complete source pipeline. Source, unsupported,
resource, argument and report/publication failures remain distinct.

## Exact implementation choices

`SchemaInterpretation::UnregisteredAp214As2010` applies only to the exact
unregistered `AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1 1}` identity.
`Ap214NameOnlyAs2010` applies only to bare `AUTOMOTIVE_DESIGN`. Neither accepts
another OID, leading-zero variant, alias, unknown schema, multiple identifiers or
already registered binding. An eligible request selects the existing fixed
`214-MBx2010-Source-0.1` implementation without authenticating the HEADER.

Preserve every original identity observation and the exact strict
`UnsupportedSchemaBinding` diagnostic. Apply the choice only after its diagnostic
fits the quota. Binding becomes `InterpretedUnverified`; later failure still
retains the applied interpretation and original refusal. On a registered identity
the option remains unapplied. APD text/year never selects a schema implementation.
Carrier assumptions similarly remain immutable in their source owner and cannot
be erased by clearing outcome notices. The [carrier contract](CARRIER.md) still
governs raw bytes, unknown text and per-token decoding evidence.

## Provisional errors and final proof

Eligible exceptions follow the original C++ whitelist, not message matching:

- Selected descriptive labels, missing nondefining names, and the admitted
  unknown PRODUCT/PD/PDF, solid-caption, attached-layer or administrative text.
  Opaque text remains unavailable, never an invented empty name. Representation
  purpose names and unproven owners do not acquire opaque-text approval.
- Actual attached property MRI/VRI quantity errors; metadata-only mass units
  with the original marker/SI prefix violations. Explicit mass dimensions must
  agree with factor-derived dimensions. Contradictions, bad physical scales,
  unsupported quantities and other source errors remain fatal.
- The fixed foreign context/layer/REAL-item transport and approval/style override
  mismatches, only under their original exact profile, field and attachment proofs.
- A missing administrative APD in the admitted MBx/N916 subsets, or distinct
  selected application contexts/protocols proved by references. No record, year,
  claim, revision or root is fabricated; extra/ambiguous contexts still fail.
- Admitted detached palette/style/category residue after its complete outgoing
  graph check. Retain these IDs as unattached metadata, without adding part roots
  or hiding extra geometry. Invisibility cannot establish ownership.

All preserved failures are provisional until source roles, properties, presentation
and administrative attachment, geometry prerequisites and physical units pass.
Final approval reuses the same indexed source graph and iterative reach order as
ownership. Physical roots include body representation/context, linked wireframe
and supporting geometry. Property, selected-role and presentation use graphs are
distinct lazy proofs; exported role labels never replace administrative/style
evidence. Sharing a defective unit with the body's complete dependency closure
makes the original error fatal. Field-specific caption exceptions retain their
separate exact proof even though the owner itself is physical.

Promote the complete original diagnostic set only after all proofs and quotas
succeed. Failure withholds the semantic snapshot and approved diagnostic set,
retains original fatal provenance and interpretation observations, and preserves
the actual phase progress/counters. No partially approved residue escapes. Repair
remains `None`; native construction remains `NotRun`.

## Owners, reports and validation

The immutable semantic snapshot owns policy, exact source geometry scope, applied interpretation, original
interpretation/preserved diagnostics and accepted residue indices, together with
its original source. Outcome accessors use the successful owner. Field views and
qualification survive dropping their originating outcomes. Source-bound report
compilation retains actual UTF-8 byte limits and all-or-nothing output. Explicit
policy uses `solidcore.step.interoperability-source.v1`; default strict report
fields/bytes stay unchanged. An unverified schema choice reports
`source_binding_verified: false`, never clean source binding.

Independent tests author exact synthetic source facts for quantity mismatches,
unit contradictions/physical sharing, absent/distinct administrative contexts,
opaque identity fields, exact schema alternatives, combined quotas, resource
boundaries, owning views, residue and defining geometry failures. Real CLI and
wasm checks exercise public boundaries. `capture_step_python_contract_inputs.py`
can record exact requests from the fixed public C++ contract suite while running
the original tests unchanged; captures contain inputs/options, never expected
answers. `step_interop_compare.py` executes original C++ and Rust readers and
compares actual statuses, phases, diagnostics, metadata, source roles, fields,
identities and counts. Their different JSON layouts have independently measured
report budgets; byte budget equality is not inferred from differently sized JSON.
Existing strict source/type/unit/geometry/ownership checks remain regressions.
All per-run captures/receipts stay local; no private model inventory is needed.
