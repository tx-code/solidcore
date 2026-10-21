# Explicit development angle-unit compatibility

This is a separately selected Rust development policy. The fixed AP214 source
catalog still requires `*` in the redeclared `NAMED_UNIT.dimensions` slot. An
explicit reference in that slot remains the original `DerivedSlotMarkerRequired`
source error. The policy neither changes that catalog nor claims a formally valid
AP encoding. Original bytes, identifiers, field values, units and roles stay owned
by the unchanged source snapshot. No source unit is rewritten or replaced.

Select `Options.development_unit_policy =
DevelopmentUnitPolicy::Ap214ExplicitPlaneAngleDimensions` and explicitly select
`GeometryScope::GeneralBrep`. The semantic CLI uses:

```text
step-semantic-inspect --fixture-limits --general-brep-source \
  --development-ap214-angle-dimensions FILE
```

The policy forces the complete type, unit, geometry-prerequisite, ownership and
physical-context source sequence; it cannot publish a shortened checkpoint and
finish its proof later. Other schema and carrier assumptions still require their
own existing options. `SourcePolicy::PreserveNonGeometric` remains independent:
this physical-unit policy does not broaden its property-only exceptions or its
physical-exclusion proof. Selecting both requires both sets of checks to pass.

`DevelopmentUnitPolicy::Disabled` is the default. It preserves the original strict
phase order, diagnostic classification, visit accounting and report bytes. In
particular, nongeometric interoperability by itself continues to reject physical
plane-angle units with this marker defect.

## Bounded proof

Only an AP214 complex record with exactly these actual external-mapping facets
is eligible: `CONVERSION_BASED_UNIT`, `NAMED_UNIT`, `PLANE_ANGLE_UNIT`. Only the
original reference-valued `NAMED_UNIT.dimensions` derived slot is provisionally
retained. `$`, other values, missing or extra facets, other nominal quantities and
other source catalogs are not admitted by this rule. A proper `*` requires no
exception and produces no applied receipt.

The existing type checker first validates all original fields and reference target
types. The existing unit dependency graph and iterative DFS then supply the same
dependencies, cycle checks, dimensional interpretation and measure prerequisites.
No parallel unit graph or alternative field parser is built. The additional proof
walks only the dependency closure of eligible units and requires:

- All seven original dimensional exponents are exact bounded-rational zero, and
  agree with the existing factor-derived dimensional vector. No numeric epsilon
  or source uncertainty is involved.
- Every dependency is a nominal plane-angle unit with zero dimensions. The chain
  consists of conversion-based angle units and terminates at source SI `RADIAN`,
  with its actual catalog-validated SI prefix. A generic dimensionless ratio,
  solid angle, length unit or unanchored named unit cannot stand in for it.
- Each conversion factor is an actual typed `MEASURE_WITH_UNIT` dependency whose
  value is a `PLANE_ANGLE_MEASURE` or `POSITIVE_PLANE_ANGLE_MEASURE`. Its source
  scalar is exactly positive. Its binary64 observation and each accumulated
  conversion to radians must remain finite and positive. Overflow, underflow or
  an unestablished branch is refused rather than clamped or guessed.
- All normal unit/measure checks and every later requested source check also pass.
  Cycles, visit limits, exact-arithmetic capacity and retained-diagnostic limits
  remain failures. The shared retained-diagnostic quota includes the independent
  development diagnostics and any separately approved nongeometric diagnostics.

Factor lexemes, measure tags and original dependency identities are retained at
every level. Names such as `DEGREE` or `RADIAN` on conversion-based units are not
used to infer a scale, pi or a replacement definition. The computed binary64
scale is a finite conversion observation, not a continuous geometric error bound
or a normalized source value.

## Owning receipt and reporting

Only after the complete source sequence succeeds does the checker construct an
immutable `DevelopmentUnitReceipt` in the source-owned semantic snapshot. Its
constructor and fields are not public. `Snapshot::development_unit_receipt()` and
`development_unit_diagnostics()` borrow it; cloning the snapshot shares its owner.
`scale_to_radians(record)` addresses only a unit proved in that same closure.
Changing a public outcome status or clearing public diagnostic annotations cannot
upgrade, replace or remove this source-owned proof. Reports reject a snapshot
transplanted from another source capture, even if its receipt is genuine.

Applied outcomes use `CheckedWithDiagnostics` when their ordinary source checkpoint
would otherwise be `Checked`; an independent deferred source phase remains
deferred. They are reported separately under
`solidcore.step.development-unit-source.v1`, with explicit requested/applied policy,
original diagnostics, source-record correspondence and the proved unit definitions.
The receipt says `sourceSyntaxVerified=false`, `formalAPValidity=Unestablished` and
`qualification=ExplicitDevelopmentUnitCompatibilityOnly`. Native construction,
full import and continuous geometry consistency remain separate checks.

Selecting the policy does not make a result strict-qualified, even when no candidate
was encountered. Consumers must inspect the sealed development policy and receipt
in addition to the independent nongeometric source policy. A successful native
validator or a mesh cannot upgrade this source interpretation into qualified import.

On a failed proof or later phase, no semantic snapshot or applied receipt is
published. Original marker errors remain available with their original spans,
owners and classification alongside the failure that stopped the request. Failure
does not silently fall back to a guessed unit or erase the original source error.

Synthetic tests in `tests/development_units.rs` cover exact zeros and each nonzero
dimension, one-marker-only changes, factor types/sign/range, dependency quantities,
multi-level conversions, cycles, complete-source ownership, N/N-1 visit budgets,
diagnostic capacity, immutable proof ownership and explicit CLI reporting. These
are development-policy contract tests, not ISO/AP or real-model qualification.
