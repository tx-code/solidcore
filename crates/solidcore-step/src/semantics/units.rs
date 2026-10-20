// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Exact dimensional dependencies, nominal predicates and measure prerequisites.
//! Upstream: `source/STEP_SEMANTICS/src/StepUnitDependencies.cpp` (`UnitDependencies`).
//! Deviations: safe Rust indices and provisional source-error retention. Explicit DFS frames
//! preserve visits/postorder, so input graph depth never consumes the Rust stack.
use super::checker::{internal, Checked, Checker};
use super::number::{compare, Rational};
use super::{DimensionExponent, Status, UnitDimension};
use crate::Span;

const DIMENSIONS: &[&str] = &[
    "length_exponent",
    "mass_exponent",
    "time_exponent",
    "electric_current_exponent",
    "thermodynamic_temperature_exponent",
    "amount_of_substance_exponent",
    "luminous_intensity_exponent",
];
type Vector = [Rational; 7];
#[derive(Clone)]
struct Edge {
    target: usize,
    owner: usize,
    field: &'static str,
}
struct Node {
    record: usize,
    edges: Vec<Edge>,
    dimensions: Vector,
}
fn basis(i: usize) -> Vector {
    let mut dims = [Rational::default(); 7];
    if let Some(d) = dims.get_mut(i) {
        *d = Rational::new(1, 1);
    }
    dims
}
fn nominal(checker: &Checker<'_>, r: usize) -> &'static str {
    for (name, label) in [
        ("DERIVED_UNIT", "Derived"),
        ("LENGTH_UNIT", "Length"),
        ("MASS_UNIT", "Mass"),
        ("PLANE_ANGLE_UNIT", "PlaneAngle"),
        ("SOLID_ANGLE_UNIT", "SolidAngle"),
    ] {
        if checker.is_a(r, name) {
            return label;
        }
    }
    "Named"
}
fn fail(
    checker: &Checker<'_>,
    status: Status,
    r: usize,
    field: &str,
    key: &str,
    expected: &str,
    actual: &str,
    responsibility: &str,
) -> Box<super::checker::Failure> {
    let stage = if status == Status::Unsupported {
        "D2"
    } else {
        "D3"
    };
    // This unit pass has its own source diagnostic builder in C++; it does not
    // reinterpret a foreign-field failure or invent a pcurve/geometry owner.
    checker.field_failure(
        status,
        key,
        stage,
        r,
        field,
        expected,
        actual,
        responsibility,
    )
}
fn rule(
    checker: &Checker<'_>,
    r: usize,
    field: &str,
    clause: &str,
) -> Box<super::checker::Failure> {
    let raw = checker
        .field(r, field)
        .map_or_else(|_| String::new(), |(_, v)| checker.raw(v.span));
    let authority = checker
        .catalog
        .map_or("", super::catalog::Catalog::authority);
    fail(
        checker,
        Status::Rejected,
        r,
        field,
        &format!("SourceRuleViolation:{clause}"),
        "declared dimensional/source predicate",
        &raw,
        &format!("{authority} {clause}"),
    )
}
fn capacity(checker: &Checker<'_>, r: usize, field: &str) -> Box<super::checker::Failure> {
    let actual = checker
        .field(r, field)
        .map_or_else(|_| String::new(), |(_, v)| v.text.clone());
    fail(
        checker,
        Status::ResourceLimit,
        r,
        field,
        "UnitDimensionCapacityExceeded",
        "exact bounded rational capacity",
        &actual,
        "unit dimension arithmetic capacity; not a source-invalid verdict or tolerance",
    )
}
fn decimal(checker: &mut Checker<'_>, r: usize, name: &str) -> Checked<Rational> {
    let (_, value) = checker.field(r, name)?;
    checker.visit(value.span)?;
    Rational::decimal(&value.text).map_err(|_| capacity(checker, r, name))
}
fn count(checker: &mut Checker<'_>, span: Span) -> Checked<()> {
    checker.visit(span)?;
    checker.outcome.unit_dependency_checks.total += 1;
    Ok(())
}
fn node_index(indices: &[Option<usize>], record: usize) -> Checked<usize> {
    indices.get(record).copied().flatten().ok_or_else(internal)
}
fn evaluate(checker: &mut Checker<'_>, nodes: &[Node], node: usize) -> Checked<Vector> {
    let current = nodes.get(node).ok_or_else(internal)?;
    let r = current.record;
    let mut dimensions = [Rational::default(); 7];
    if checker.is_a(r, "DERIVED_UNIT") {
        let (_, elements) = checker.field(r, "elements")?;
        count(checker, elements.span)?;
        if current.edges.len() == 1 {
            let edge = current.edges.first().ok_or_else(internal)?;
            let (_, power) = checker.field(edge.owner, "exponent")?;
            if compare(&power.text, "1.") == 0 {
                return Err(rule(checker, r, "elements", "DERIVED_UNIT.WR1"));
            }
        }
        for edge in &current.edges {
            let dims = nodes.get(edge.target).ok_or_else(internal)?.dimensions;
            let (_, exponent) = checker.field(edge.owner, "exponent")?;
            if dims.iter().all(|v| v.n == 0) {
                checker.visit(exponent.span)?;
                continue;
            }
            let power = decimal(checker, edge.owner, "exponent")?;
            for (value, dimension) in dimensions.iter_mut().zip(dims) {
                checker.visit(exponent.span)?;
                *value = power
                    .multiply(dimension)
                    .and_then(|n| value.add(n))
                    .map_err(|_| capacity(checker, edge.owner, "exponent"))?;
            }
        }
    } else if checker.is_a(r, "SI_UNIT") {
        let (_, name) = checker.field(r, "name")?;
        dimensions = match name.text.as_str() {
            "METRE" => basis(0),
            "GRAM" => basis(1),
            "RADIAN" | "STERADIAN" => [Rational::default(); 7],
            _ => {
                return Err(fail(
                    checker,
                    Status::Unsupported,
                    r,
                    "name",
                    "UnsupportedUnitDimensionKind",
                    "metre/gram/radian/steradian source subset",
                    &name.text,
                    "profile SI dimensional interpretation subset",
                ))
            }
        };
        if checker.is_a(r, "MASS_UNIT") {
            let mut used = false;
            for incoming in checker.source.incoming(&checker.record(r)?.id) {
                let reference = checker
                    .source
                    .references()
                    .get(*incoming)
                    .ok_or_else(internal)?;
                checker.visit(reference.span)?;
                used = used || checker.is_a(reference.record_index, "DERIVED_UNIT_ELEMENT");
            }
            if used {
                let (_, prefix) = checker.field(r, "prefix")?;
                count(checker, prefix.span)?;
                if prefix.text != "KILO" {
                    let failure = rule(checker, r, "prefix", "SI_UNIT.WR1");
                    if !checker.schema()?.ap214()
                        || checker.outcome.source_policy()
                            != super::SourcePolicy::PreserveNonGeometric
                        || name.text != "GRAM"
                    {
                        return Err(failure);
                    }
                    preserve_pending(checker, failure, r, "prefix")?;
                }
            }
        }
    } else if checker.is_a(r, "CONVERSION_BASED_UNIT") {
        let edge = current.edges.first().ok_or_else(internal)?;
        dimensions = nodes.get(edge.target).ok_or_else(internal)?.dimensions;
        if checker.schema()?.ap214() {
            let (_, bound) = checker.field(r, "dimensions")?;
            if bound.kind == crate::ValueKind::Reference {
                // The provisional mass-unit exception preserves a source error,
                // not an AP214 transport value. Its seven original exponents
                // must still agree with the factor-derived vector before approval.
                let source_dimensions = checker.target(bound)?;
                let mut explicit = [Rational::default(); 7];
                for (d, name) in explicit.iter_mut().zip(DIMENSIONS) {
                    *d = decimal(checker, source_dimensions, name)?;
                }
                count(checker, bound.span)?;
                if explicit != dimensions {
                    return Err(fail(checker, Status::Rejected, r, "dimensions",
                        "MetadataMassUnitDimensionsConflict", "factor-derived dimensional vector",
                        "contradictory explicit source vector",
                        "property-only compatibility guard; no contradictory unit dimensions retained"));
                }
            }
        }
        if matches!(
            checker.schema()?.kind,
            "Ap203e2GeneralSource" | "Ap203N916Source"
        ) {
            let (_, bound) = checker.field(r, "dimensions")?;
            let source_dimensions = checker.target(bound)?;
            let mut explicit = [Rational::default(); 7];
            for (d, name) in explicit.iter_mut().zip(DIMENSIONS) {
                *d = decimal(checker, source_dimensions, name)?;
            }
            count(checker, bound.span)?;
            if explicit != dimensions {
                if checker.schema()?.kind == "Ap203N916Source" {
                    return Err(fail(
                        checker,
                        Status::Unsupported,
                        r,
                        "dimensions",
                        "SourceConversionDimensionsUnestablished",
                        "explicit dimensions coherent with the conversion factor",
                        "different dimensional vectors",
                        "project physical conversion prerequisite; not a N916 CBU WHERE rule",
                    ));
                }
                return Err(rule(checker, r, "dimensions", "CONVERSION_BASED_UNIT.WR1"));
            }
            if checker.schema()?.kind == "Ap203N916Source" {
                dimensions = explicit;
            }
            let (_, factor) = checker.field(r, "conversion_factor")?;
            let factor = checker.target(factor)?;
            let (_, value) = checker.field(factor, "value_component")?;
            count(checker, value.span)?;
            let scalar = value.children.first().ok_or_else(internal)?;
            if compare(&scalar.text, "0.") <= 0 {
                return Err(fail(
                    checker,
                    if checker.schema()?.kind == "Ap203N916Source" {
                        Status::Unsupported
                    } else {
                        Status::Rejected
                    },
                    factor,
                    "value_component",
                    "SourceUnitScaleNonPositive",
                    ">0",
                    &scalar.text,
                    "positive physical conversion-scale guard",
                ));
            }
        }
    } else {
        let (_, bound) = checker.field(r, "dimensions")?;
        let source_dimensions = checker.target(bound)?;
        for (d, name) in dimensions.iter_mut().zip(DIMENSIONS) {
            *d = decimal(checker, source_dimensions, name)?;
        }
    }
    if !checker.is_a(r, "DERIVED_UNIT") {
        let (required, clause) = match nominal(checker, r) {
            "Length" => (basis(0), "LENGTH_UNIT.WR1"),
            "Mass" => (basis(1), "MASS_UNIT.WR1"),
            "PlaneAngle" => ([Rational::default(); 7], "PLANE_ANGLE_UNIT.WR1"),
            "SolidAngle" => ([Rational::default(); 7], "SOLID_ANGLE_UNIT.WR1"),
            _ => return Ok(dimensions),
        };
        let name = if checker.is_a(r, "SI_UNIT") {
            "name"
        } else {
            "dimensions"
        };
        let (_, value) = checker.field(r, name)?;
        count(checker, value.span)?;
        if required != dimensions {
            return Err(rule(checker, r, name, clause));
        }
    }
    Ok(dimensions)
}
// UnitDependencies::PreservePending has its own capacity timing/field diagnostic.
// Do not replace it with the outer Preserve helper: schema assumptions share the
// final quota but this original unit-phase check counts pending issues alone.
fn preserve_pending(
    checker: &mut Checker<'_>,
    failure: Box<super::checker::Failure>,
    r: usize,
    field: &str,
) -> Checked<()> {
    if checker.pending.len() >= checker.outcome.limits.preserved_diagnostics {
        return Err(fail(
            checker,
            Status::ResourceLimit,
            r,
            field,
            "PreservedDiagnosticLimit",
            "configured preserved diagnostic capacity",
            &(checker.pending.len() + 1).to_string(),
            "explicit interoperability capacity; no diagnostics silently discarded",
        ));
    }
    checker.pending.push(super::interop::Pending {
        failure,
        proof: super::interop::Proof::ExistingMetadata,
    });
    Ok(())
}
fn measures(checker: &mut Checker<'_>, nodes: &[Node], indices: &[Option<usize>]) -> Checked<()> {
    for r in 0..checker.source.records().len() {
        let value_item = checker.is_a(r, "VALUE_REPRESENTATION_ITEM")
            && checker.schema()?.kind != "Ap203N916Source";
        if !checker.is_a(r, "MEASURE_WITH_UNIT") && !value_item {
            continue;
        }
        let result = (|| -> Checked<()> {
            let (_, value) = checker.field(r, "value_component")?;
            let tag = value.text.as_str();
            if matches!(
                tag,
                "POSITIVE_RATIO_MEASURE"
                    | "POSITIVE_LENGTH_MEASURE"
                    | "NON_NEGATIVE_LENGTH_MEASURE"
                    | "POSITIVE_PLANE_ANGLE_MEASURE"
            ) {
                count(checker, value.span)?;
                checker.outcome.unit_dependency_checks.type_predicates += 1;
                let sign = compare(&value.children.first().ok_or_else(internal)?.text, "0.");
                if sign < 0 || (sign == 0 && tag != "NON_NEGATIVE_LENGTH_MEASURE") {
                    return Err(rule(checker, r, "value_component", &format!("{tag}.WR1")));
                }
            }
            if value_item {
                return Ok(());
            } // A VRI has no unit_component; never borrow the body's unit.
            let (_, unit) = checker.field(r, "unit_component")?;
            let target = checker.target(unit)?;
            let required = match tag {
                "MASS_MEASURE" => basis(1),
                "LENGTH_MEASURE" | "NON_NEGATIVE_LENGTH_MEASURE" | "POSITIVE_LENGTH_MEASURE" => {
                    basis(0)
                }
                "PLANE_ANGLE_MEASURE"
                | "POSITIVE_PLANE_ANGLE_MEASURE"
                | "RATIO_MEASURE"
                | "POSITIVE_RATIO_MEASURE" => [Rational::default(); 7],
                _ => {
                    return Err(fail(
                        checker,
                        Status::Unsupported,
                        r,
                        "value_component",
                        "UnsupportedMeasureDimensionBranch",
                        "interpreted length/mass/ratio measure family",
                        tag,
                        "profile dimensional interpretation subset; no guessed dimension",
                    ))
                }
            };
            count(checker, unit.span)?;
            if nodes
                .get(node_index(indices, target)?)
                .ok_or_else(internal)?
                .dimensions
                != required
            {
                return Err(rule(checker, r, "unit_component", "MEASURE_WITH_UNIT.WR1"));
            }
            for (subtype, nominal_type) in [
                ("LENGTH_MEASURE_WITH_UNIT", "LENGTH_UNIT"),
                ("MASS_MEASURE_WITH_UNIT", "MASS_UNIT"),
                ("PLANE_ANGLE_MEASURE_WITH_UNIT", "PLANE_ANGLE_UNIT"),
            ] {
                if checker.is_a(r, subtype) {
                    count(checker, unit.span)?;
                    if !checker.is_a(target, nominal_type) {
                        return Err(rule(
                            checker,
                            r,
                            "unit_component",
                            &format!("{subtype}.WR1"),
                        ));
                    }
                }
            }
            Ok(())
        })();
        if let Err(failure) = result {
            let key = failure.diagnostic.source.key.as_str();
            if checker.outcome.source_policy() != super::SourcePolicy::PreserveNonGeometric
                || (!checker.is_a(r, "MEASURE_REPRESENTATION_ITEM") && !value_item)
                || failure.status != Status::Rejected
                || !matches!(
                    key,
                    "SourceRuleViolation:MEASURE_WITH_UNIT.WR1"
                        | "SourceRuleViolation:POSITIVE_RATIO_MEASURE.WR1"
                )
            {
                return Err(failure);
            }
            preserve_pending(checker, failure, r, "value_component")?;
        }
    }
    Ok(())
}
pub(super) fn check(checker: &mut Checker<'_>) -> Checked<()> {
    // These sizes come from completed, budgeted source records, never an input
    // header count or sparse ID. No edge/use is pruned by exponent or magnitude.
    let mut indices = vec![None; checker.source.records().len()];
    let mut nodes = Vec::new();
    for (r, record) in checker.source.records().iter().enumerate() {
        checker.visit(record.span)?;
        if checker.is_a(r, "NAMED_UNIT") || checker.is_a(r, "DERIVED_UNIT") {
            *indices.get_mut(r).ok_or_else(internal)? = Some(nodes.len());
            nodes.push(Node {
                record: r,
                edges: Vec::new(),
                dimensions: [Rational::default(); 7],
            });
        }
    }
    for node in &mut nodes {
        let r = node.record;
        if checker.is_a(r, "DERIVED_UNIT") {
            let (_, elements) = checker.field(r, "elements")?;
            for element in &elements.children {
                checker.visit(element.span)?;
                let owner = checker.target(element)?;
                let (_, unit) = checker.field(owner, "unit")?;
                node.edges.push(Edge {
                    target: node_index(&indices, checker.target(unit)?)?,
                    owner,
                    field: "unit",
                });
            }
        } else if checker.is_a(r, "CONVERSION_BASED_UNIT") {
            let (_, factor) = checker.field(r, "conversion_factor")?;
            let owner = checker.target(factor)?;
            let (_, unit) = checker.field(owner, "unit_component")?;
            checker.visit(unit.span)?;
            node.edges.push(Edge {
                target: node_index(&indices, checker.target(unit)?)?,
                owner,
                field: "unit_component",
            });
        }
    }
    let mut state = vec![0_u8; nodes.len()];
    let mut stack: Vec<(usize, usize)> = Vec::new();
    for start in 0..nodes.len() {
        if state.get(start) == Some(&2) {
            continue;
        }
        *state.get_mut(start).ok_or_else(internal)? = 1;
        stack.push((start, 0));
        while let Some((node, next)) = stack.last().copied() {
            checker.visit(
                checker
                    .record(nodes.get(node).ok_or_else(internal)?.record)?
                    .span,
            )?;
            if let Some(edge) = nodes.get(node).and_then(|n| n.edges.get(next)).cloned() {
                stack.last_mut().ok_or_else(internal)?.1 += 1;
                let (_, value) = checker.field(edge.owner, edge.field)?;
                checker.visit(value.span)?;
                let status = *state.get(edge.target).ok_or_else(internal)?;
                if status == 1 {
                    let record =
                        checker.record(nodes.get(edge.target).ok_or_else(internal)?.record)?;
                    let n916 = checker.schema()?.kind == "Ap203N916Source";
                    return Err(fail(
                        checker,
                        if n916 {
                            Status::Unsupported
                        } else {
                            Status::Rejected
                        },
                        edge.owner,
                        edge.field,
                        "SourceUnitDependencyCycle",
                        "finite dimensional dependency graph",
                        &record.id,
                        if n916 {
                            "project finite conversion dependency prerequisite"
                        } else {
                            "source dimensional dependency graph"
                        },
                    ));
                }
                if status == 0 {
                    *state.get_mut(edge.target).ok_or_else(internal)? = 1;
                    stack.push((edge.target, 0));
                }
                continue;
            }
            let dimensions = evaluate(checker, &nodes, node)?;
            nodes.get_mut(node).ok_or_else(internal)?.dimensions = dimensions;
            checker.outcome.unit_dependency_checks.unit_definitions += 1;
            *state.get_mut(node).ok_or_else(internal)? = 2;
            stack.pop();
        }
    }
    measures(checker, &nodes, &indices)?;
    // Derived observations are published only after every unit and measure
    // prerequisite succeeds; the owning snapshot waits for all requested checks.
    for node in nodes {
        checker.units.dimension_definitions.push(UnitDimension {
            record_index: node.record,
            nominal: nominal(checker, node.record).into(),
            dimensions: node.dimensions.map(|v| DimensionExponent {
                numerator: v.n.to_string(),
                denominator: v.d.to_string(),
            }),
        });
    }
    Ok(())
}
