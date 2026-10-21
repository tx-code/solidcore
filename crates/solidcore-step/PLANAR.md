# Fixed-profile planar source qualification

This ports the maintained `source/STEP_PLANAR` implementation at
`d8ed45ab85d52dba07660cafdceacafdca7217ed`: bounded exact arithmetic, the 80 local
identities, all 47 global rules, directed LINE/PLANE regions, source edge incidence
and explicit conversion-unit interpretation. It has no kernel dependency and
returns diagnostic evidence with **no native model**. Passing this scope does not
establish complete AP validity, target trims, material interior/volume, shell
nonintersection or downstream modeling/query fitness.

## Inputs, execution and owners

`planar::check(&semantic_outcome, explicit_quality, explicit_limits)` requires the
complete selected-part/physical-unit semantic handoff. A types-only snapshot
cannot be promoted by calling this layer. The supported rule implementation is
exactly the original `203e2-P0-Planar-0.1` profile. Other checked/deferred profiles
are refused before rule admission; no AP214/global evidence is fabricated. Only
the original complete conversion-unit deferral may resume. Source errors and
unsupported, resource, request and numeric failures retain distinct statuses.

Successful outcomes retain a private immutable witness exposed through
`Outcome::checked_evidence()`. A downstream native builder uses that witness,
not the editable public diagnostic projection or a rewritten status. Failure
outcomes have no witness. Keeping the existing diagnostic interface currently
requires an additional evidence copy on success; original source bytes remain
shared. No extra rule work is charged and report content/order remains unchanged.

The initial per-record admission charge precedes all phases. Local rules, global
rules and geometry execute in the original order, with the original predicate
short-circuiting, denied-charge accounting and evidence insertion timing. An
admission stop leaves every phase `NotRun`. Entered failed phases retain their
failure classification and partial observations; later phases remain unrun.
Entity-clause rows survive incomplete numeric evaluation. Named TYPE rows are
inserted only after numeric evaluation succeeds. Computed LOGICAL UNKNOWN is
distinct from unevaluated evidence; QUERY selects only TRUE.

Evidence owns the original immutable semantic/source snapshot. Field reads,
references, type populations, inverse roles and failure positions reuse those
bindings. No separate field-name/ID interpretation is constructed. Per-instance
local populations retain only their owner; complete populations remain in the
semantic snapshot. Global rows retain every FOR population. Only the fixed
closed-profile exclusion proof treats an unregistered FOR type as excluded;
public unknown population queries still return `None`.

The rule catalog is generated offline by `scripts/generate_step_rules.py` from
the fixed public C++ declaration facts. It imports metadata/plans, not executable
predicates or expected answers. Named enum matches dispatch exhaustively; storage
order and numeric discriminants never dispatch a rule. Compile-time checks reject
duplicate identities/declarations, missing execution, invalid subjects and changed
inherited predicate chains. Runtime/report order follows separate fixed plans.
Catalog fingerprint/regeneration checks bind that metadata to its source.

## Numeric and geometry contract

`planar::Rational` has a reduced signed-63-bit-magnitude numerator and positive
denominator, exposed only through read-only accessors. Construction and arithmetic
are checked. Continued fractions compare without overflowing cross products.
The original decimal/intermediate capacity and error precedence are retained,
including exponent checks before zero-mantissa acceptance. This deliberately
differs from the semantic unit rational parser; accepting a wider value there
does not establish planar numeric evidence. Exhaustion returns `Indeterminate`,
not a source-invalid verdict or rounded pass. Undefined division has its own key.

Length units normalize exactly to millimeters; directions and LINE parameters
remain dimensionless. Edge parameters keep their coefficient and algebraic norm
divisor. Plane axes stay in an exact unnormalized orthogonal basis. Evidence
states the metric factors and algebraic mapping to original source UV. No target
UV window, fitted geometry or source point/curve/topology/sense rewrite occurs.

Endpoint/curve and edge/plane consistency use exact squared distances and the
explicit inclusive threshold. Projected region quantities retain residuals and
are labeled as projected observations. Loops require source identity connection,
simple nonzero boundaries, correct effective winding, strictly contained holes,
no inter-bound contact and no nested-hole ambiguity. Shared source edge use checks
orientation and manifold incidence; connectedness follows source edge identity.
A closed surface stays a surface under its original semantic role.

Conversion units retain explicit dimensions, factor/base identities, typed factor
values and exact scales. A bounded iterative dependency path and shared memo
require a positive SI length root; names never select a scale. Cycles/nonpositive
scales remain project interpretation failures, not invented WHERE clauses.
Source uncertainty does not enlarge quality or select a preset.

`Quality::planar_fixture()` explicitly selects frozen `Q-P0-mm-01`, consistency
`1e-7 mm`, solely as a synthetic candidate. Its identity cannot alias another
value. A caller may select a different named positive rational quality; none is a
production default. Work limits are separate from that distance and from all
native tolerance/repair budgets. No repair, material volume, shell self-intersection
or native construction is run here.

## CLI and report

```powershell
cargo run -p solidcore-step --features cli --bin step-planar-inspect -- --fixture-limits --quality Q-P0-mm-01 public.step
```

`--planar-work N` and `--dependency-visits N` control their respective stages;
all nine source budget overrides remain available. The Rust CLI additionally
exposes the original custom-quality library contract through
`--quality NAME --consistency-mm N/D` (or an exact decimal). A custom name requires
an explicit distance; the frozen name must retain its frozen value. This does
not add a default or infer precision from data. Configuration errors exit 64.

CLI exits: 0 checked source evidence, 1 source rejection, 2 unsupported source/
profile, 3 resource limit, 4 capture I/O, 5 report limit, 6 infrastructure, 7
deferred and 8 unestablished numeric evidence. No exit qualifies native geometry.
Original source I/O remains the Rust CLI's stderr-only policy. Reports compile
completely under actual UTF-8 byte limits, including the final LF; no partial
success JSON is emitted on report overflow. OS write failure still requires a
failed completion even if some external stdout bytes were written.

`solidcore.step.planar.v1` embeds the preceding semantic/source report once,
borrowing the same writer protocol without allocating a second report graph.
Record indices reference `source_semantics.source.records`; original IDs/raw bytes
remain there and in the owning API. Exact fractions, all rules/witnesses, metrics,
regions, directions, conversions and diagnostics remain explicit. Every result
has `model=null`; only completed current source checks set
`qualified_source_evidence=true`. Per-run files/receipts stay local.

## Validation boundary

Independent synthetic tests use analytic plane/box/hole area, centroid, intervals,
incidence, rational capacity and source-role truth. Real CLI tests cover explicit
quality, inclusive/error thresholds, source preservation and exact report budgets.
The existing C++ CLI/corpus plus additional public Python contracts are executed
unchanged, with inputs/options captured locally for actual-reader comparison.
`step_planar_compare.py` compares every emitted C++ rule/population/proof, metric,
region, source UV mapping, identity, direction, conversion, diagnostic and counter.
Different JSON layouts use independently measured report budgets. The C++ API
observer in `step_planar_reference.cpp` adds complete actual diagnostic fields
and custom-quality requests without replacing any predicate or original assertion.
This is source compatibility evidence, not a substitute for independent geometry
truth. Full-kernel/workspace/native qualification remains a separate gate.
