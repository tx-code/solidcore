# Fixed source geometry prerequisites

`Options.check_geometry_dependencies` / `--geometry-dependencies` migrates the
maintained C++ `StepGeometryDependencies.cpp` pass. `--source-semantics` selects
both this pass and the existing strict part/closure checkpoint, in the original
order: Binding → Types → UnitDependencies → GeometryDependencies → Product/Closure
→ selected UnitContext. P0 preserves its intentional unrun geometry-dependency
phase; its separate planar/local/global rules are still required work.

The new modes do not alter earlier types/units-only or part/closure requests.
No phase is silently added to those requests. New reports use
`solidcore.step.geometry-dependencies.v1` or `solidcore.step.source-semantics.v1`.
They retain separate actual total, entity and TYPE predicate counters, charged
dependency visits, source-bound report budgets and withheld snapshots on failure.
Continuous geometry consistency, planar rules, native construction and full import
remain explicitly `NotRun`. A successful prerequisite pass is not full AP validity
or evidence of a usable native model.

## Source facts checked

- Positive lengths retain exact source-decimal comparison, including signed zero
  and enormous exponents. Conical radius permits zero; no invented bound clamps
  its semi-angle. The original source nominal types and values remain unchanged.
- B-spline curve and U/V surface directions check declared degrees, control
  counts, multiplicity/knot list sizes, positive bounded multiplicities and their
  required total, and strictly increasing distinct source knots. Knots are not
  expanded, normalized or remapped, and U/V are never exchanged.
- Control grids and rational weight arrays keep nested source shape. Ragged rows,
  mismatched dimensions and nonpositive weights fail at their real slots. No
  padding, truncation, fitting or tolerance relaxation is performed.
- Trim SELECT sets retain their actual member kinds. Repeated numeric values,
  type/SET checks and trim-specific WHERE prerequisites remain distinct checks.
- Composite transitions count source occurrences and derive the closed declaration
  only from the last transition. Ordinary segment parents must be BOUNDED_CURVE;
  a bare conic is not silently wrapped. Required inverse uses remain BAG uses,
  not deduplicated records or synthetic transmitted fields.
- Iterative definition traversals reject trimmed and mixed composite/trim cycles.
  Only forward definition edges participate; inverse users do not manufacture
  cycles. Related segment-use spans are retained without inventing nested paths
  through a reference hop. Shared definitions use memoized postorder while every
  repeated occurrence still consumes its original visits.
- Wireframe sets and representations check their fixed source category/member
  predicates. A bare LINE differs from a trimmed line; conic placement remains
  the source's actual 3D placement. Memoization preserves the C++ visit order.
- Bounded-surface representations require a direct geometric-set item containing
  a direct SURFACE element. An unrelated surface, indirect dependency or geometry
  role elsewhere in the file cannot satisfy that source prerequisite. This clause
  does not add native support or satisfy the representation's other WHERE rules.

No source field graph is rebuilt. The pass borrows existing immutable values and
bindings, and allocates only the checked traversal/membership state needed by the
dependency algorithms. No native geometry, units or parameterization is changed.
Failures retain original key/stage/classification, source owner/component/slot,
suffix/related spans, expected/actual values and responsibility. A request budget
is charged before predicate counting or decimal work, exactly as in C++.

## Validation

Independent Rust tests cover signed-zero/large-exponent predicates, spline counts
and order, rational ragged/negative weights, trim/compound cycles, inverse-use and
transition rules, direct surface membership, wireframe reference-hop evidence,
deep iterative chains and resource/report boundaries. Existing checkpoint tests
remain applicable; the complete migration goal also requires all other owners in
[the full ledger](../../docs/FORK_MIGRATION.md).

`scripts/step_geometry_source_compare.py` compares the actual scoped C++ phase
runner and, separately, the original unmodified `step_semantic_inspect` executable
against Rust. It checks phase order/status, diagnostics, counts, raw source,
typed declarations/populations, roles, reference-use facts, property links and
exact SI/dimension metadata. Its explicit captured directories contain public
synthetic contract inputs only; expected answers are never used as observations.
Per-profile budget probes stop before and inside geometry checking and verify
the withheld-snapshot behavior. Source numerical truth is tested independently
of the common C++ implementation; no continuous geometric truth is claimed.

```sh
python -B scripts/step_geometry_source_compare.py \
  --cpp bin/geometry-reference/step-type-unit-reference.exe \
  --cpp-full /path/to/step_semantic_inspect \
  --rust bin/debug/step-semantic-inspect \
  --corpus /path/to/ap203e2-planar/v1/cases \
  --captured bin/public-synthetic-contract-inputs
```

Run receipts, captured inputs, binaries and any private model-derived material
stay local. No C++ reader/kernel is called by the Rust product. General source
interop/interpretation options, remaining planar/global rules, native exact BRep
construction, kernel/API gaps, bindings and tools are still migration work.
