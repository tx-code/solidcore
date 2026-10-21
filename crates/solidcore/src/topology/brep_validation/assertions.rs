// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Root consistency checks in the maintained native order.
//!
//! Upstream: `SmBrep.cpp::AssertValid` (34820--35214), `d8ed45a`;
//! `SmTopology.cpp::AssertValid`, `SmOwningTopology.cpp::AssertValid`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_valid`] | root `SmBrep::AssertValid` |
//! | [`AssertValidOptions`] | native level and walk arguments |
//! Deviations: arena/value invariants replace unrepresentable root peer/context
//! pointer failures. Missing geometric/local predicates remain explicit errors;
//! a completed root-only check is not descendant or import qualification.

use super::{
    bp, validate_and_update_tolerances, validate_pointers, Brep, CheckLevel, Object, Reporter,
    ValidationOptions,
};
use crate::error::{ignored, Result, SolidError};
use crate::geom::assert_array::{labels, AssertArray};
use crate::kernel::Kernel;
use crate::topology::{RegionId, ShellId};

/// Native root-validation arguments; default is level zero with descendant walk,
/// unlike standalone `ValidatePointers`' no-walk default. Full qualification
/// requires explicit level two plus a successfully completed descendant walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AssertValidOptions<'a> {
    /// Original Release test-level/request semantics.
    pub level: CheckLevel<'a>,
    /// `SM_WALK`: execute each local descendant validator once after root checks.
    pub walk: bool,
}
impl Default for AssertValidOptions<'_> {
    fn default() -> Self {
        Self {
            level: CheckLevel::Zero,
            walk: true,
        }
    }
}

fn missing(symbol: &'static str) -> SolidError {
    SolidError::unported(symbol, "fork.topology_validation")
}

/// Execute the actual root gate. Keep reports even when the returned boolean is
/// true: source pointer/tolerance routines may report without failing status,
/// and root test21 is warning-only. Validation may materialize caches/UV data
/// and alter native working tolerances; this is not a read-only repair policy.
#[allow(clippy::too_many_lines)]
pub fn assert_valid(
    k: &mut Kernel,
    b: &mut Brep,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    let mut report = Reporter { reports };
    let mut valid = true;
    if !matches!(options.level, CheckLevel::Given(_)) {
        valid &= root_base(b, &mut report)?;
    }
    // Native always invokes pointers at LEVEL0, not the root's requested level.
    match validate_pointers(
        k,
        b,
        ValidationOptions {
            level: CheckLevel::Zero,
            walk: options.walk,
        },
        report.reports.as_deref_mut(),
    ) {
        Ok(()) => {}
        Err(SolidError::Upstream { .. }) => return Ok(false),
        Err(error) => return Err(error),
    }
    // Native accumulates ordinary tolerance status failure and keeps checking.
    valid &= ignored(validate_and_update_tolerances(
        k,
        b,
        true,
        options.level,
        report.reports.as_deref_mut(),
    ))?
    .is_some();
    let regions = bp::regions(b, None)?;
    let shells = super::local::root_shells(k, b)?;
    let faces = bp::faces(b, None)?;
    let edges = bp::edges(b, None)?;
    let vertices = bp::vertices(b, None)?;
    if b.infinite_region().is_some() {
        valid &= root_pred(&mut report, 0, true)?;
    }
    // Global list-head/context infrastructure is intrinsic to this arena value.
    for index in [1, 2, 5] {
        valid &= root_pred(&mut report, index, true)?;
    }
    if !b.deleted_surfaces().is_empty() {
        valid &= root_pred(&mut report, 6, true)?;
    }
    if regions.len() > 1 {
        valid &= root_pred(&mut report, 7, !faces.is_empty())?;
    }

    let found = if let Some(found) = ignored(find_infinite_region(k, b, &regions))? {
        found
    } else {
        valid = false;
        None
    };
    valid &= root_pred(&mut report, 8, found == b.infinite_region())?;
    let first = regions.first().copied().ok_or_else(|| {
        SolidError::invalid_input(
            "topology::brep_validation::assert_valid",
            "native root would index an empty region array",
        )
    })?;
    valid &= root_pred(&mut report, 9, b.infinite_region() == Some(first))?;

    if !faces.is_empty() || !edges.is_empty() || !vertices.is_empty() {
        for (i, region) in regions.iter().copied().enumerate() {
            let children = b.region(region)?.shells.clone();
            valid &= root_pred(&mut report, 16, !children.is_empty() || regions.len() <= 1)?;
            if children.is_empty() && regions.len() > 1 {
                continue;
            }
            let outer = if i == 0 {
                None
            } else {
                children.first().copied()
            };
            if let Some(outer) = outer {
                valid &= root_pred(&mut report, 10, b.shell(outer)?.is_faceuse_shell())?;
                valid &= root_pred(&mut report, 11, b.shell(outer)?.is_closed(b, outer)?)?;
            }
            for inner in children.iter().skip(usize::from(i != 0)).copied() {
                let inside = match outer {
                    None => true,
                    Some(outer) => inner_shell_of(k, b, inner, outer)?,
                };
                valid &= root_pred(&mut report, 12, inside)?;
            }
        }
    }
    if regions.len() > 1 {
        let mut closed = false;
        for shell in b.region(first)?.shells.iter().copied() {
            if b.shell(shell)?.is_faceuse_shell() && b.shell(shell)?.is_closed(b, shell)? {
                closed = true;
                break;
            }
        }
        valid &= root_pred(&mut report, 17, closed)?;
    }
    let infinite = b
        .infinite_region()
        .ok_or_else(|| SolidError::null_pointer("root region traversal"))?;
    let mut connected = collect_regions(k, b, infinite)?;
    connected.push(infinite);
    valid &= root_pred(&mut report, 18, connected.len() == regions.len())?;
    let common = regions.iter().filter(|r| connected.contains(r)).count();
    valid &= root_pred(&mut report, 19, common == regions.len())?;

    // This explicit SM_ASSERT_TEST_NEED guard is important: Release's report
    // macro alone ignores Given requests. Do not run an expensive unselected query.
    if coincidence_selected(options.level) {
        let coincident = contains_coincident_topology(k, b)?;
        valid &= root_pred(&mut report, 15, !coincident)?;
    }
    for _ in &shells {
        valid &= root_pred(&mut report, 13, true)?;
    }
    for _ in &faces {
        valid &= root_pred(&mut report, 14, true)?;
    }
    let mut face_shell_count = 0;
    for shell in &shells {
        if b.shell(*shell)?.is_faceuse_shell() {
            face_shell_count += 1;
        }
    }
    // Source test21 intentionally does not modify bRtn.
    let manifold = crate::topology::brep::p2::is_manifold_solid(b)?;
    root_pred(
        &mut report,
        21,
        manifold || faces.len() != face_shell_count || faces.len() <= 1,
    )?;
    // Tests3/4 removed with composites,22 disabled;30 is under SM_NMTLIB_7166,
    // not the maintained default SDK profile. Do not invent them as active checks.
    if options.walk {
        valid &= assert_sub_topology(k, b, options.level, &mut report)?;
    }
    Ok(valid)
}

fn root_pred(report: &mut Reporter<'_>, test: u64, passed: bool) -> Result<bool> {
    report.list_value(
        0,
        test,
        Object::Brep,
        passed,
        crate::math::UNDEF_DOUBLE,
        crate::math::UNDEF_DOUBLE,
    )
}

fn root_base(b: &Brep, report: &mut Reporter<'_>) -> Result<bool> {
    let mut valid = true;
    // SmObject has no predicates. SmAObject's typed attribute representation and
    // common value context are intrinsic; emit successful tests to preserve the
    // native LastAdd sequence rather than pretending an alternate pointer graph.
    valid &= report.class_predicate(labels::A_OBJECT_TYPE, 0, Object::Brep, true)?;
    let attrs = b.notifications().attributes.attributes(b.key());
    if attrs.len() == 1 {
        valid &= report.class_predicate(labels::A_OBJECT_TYPE, 1, Object::Brep, true)?;
    } else {
        for _ in &attrs {
            valid &= report.class_predicate(labels::A_OBJECT_TYPE, 2, Object::Brep, true)?;
        }
    }
    // A Brep value is a root, never a peer in another owning list. Thus the
    // no-owner Topology tests5/6, the terminating ring test7 and count8 hold by
    // representation. Child list evidence comes from actual live region arrays.
    for test in [5, 6, 7, 8] {
        valid &= report.class_predicate(labels::TOPOLOGY_TYPE, test, Object::Brep, true)?;
    }
    let children = bp::regions(b, None)?;
    if children.is_empty() {
        valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 5, Object::Brep, true)?;
    } else {
        for (i, child) in children.iter().enumerate() {
            b.region(*child)?;
            // Vec's reciprocal peer order/count and attached in_brep membership
            // encode these native list invariants; no notification link fields are used.
            valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 1, Object::Brep, true)?;
            valid &= report.class_predicate(
                labels::OWNING_TOPOLOGY_TYPE,
                2,
                Object::Brep,
                b.region(*child)?.in_brep,
            )?;
            if i + 1 == children.len() {
                valid &=
                    report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 3, Object::Brep, true)?;
            }
            valid &= report.class_predicate(labels::OWNING_TOPOLOGY_TYPE, 4, Object::Brep, true)?;
        }
    }
    // SmSAGObject currently adds no predicates beyond this owning base chain.
    Ok(valid)
}

fn coincidence_selected(level: CheckLevel<'_>) -> bool {
    match level {
        CheckLevel::Two => true,
        CheckLevel::Given(requests) => requests.contains(&15),
        // SM_ASSERT_TEST_NEED excludes LEVEL_HEAL=3 as well as levels0/1.
        _ => false,
    }
}

fn find_infinite_region(_k: &Kernel, _b: &Brep, regions: &[RegionId]) -> Result<Option<RegionId>> {
    // Exact first branch of SmBrep.cpp::FindInfiniteRegion, before bbox/rays.
    if regions.len() == 1 {
        return Ok(regions.first().copied());
    }
    // UNPORTED(topology.brep_p9): multi-region FindInfiniteRegion ray selection.
    Err(SolidError::unported(
        "SmBrep::FindInfiniteRegion",
        "topology.brep_p9",
    ))
}
fn inner_shell_of(_k: &mut Kernel, _b: &Brep, _inner: ShellId, _outer: ShellId) -> Result<bool> {
    // UNPORTED(topology.shell): actual ray-based SmShell::IsInnerShellOf.
    Err(SolidError::unported(
        "SmShell::IsInnerShellOf",
        "topology.shell",
    ))
}
fn contains_coincident_topology(_k: &mut Kernel, _b: &mut Brep) -> Result<bool> {
    // UNPORTED(fork.topology_validation): exact LEVEL2 coincidence predicate.
    Err(missing("SmBrep::ContainsCoincidentTopology"))
}
fn assert_sub_topology(
    k: &mut Kernel,
    b: &mut Brep,
    level: CheckLevel<'_>,
    report: &mut Reporter<'_>,
) -> Result<bool> {
    super::local::walk_root(k, b, level, report)
}

fn adjacent_regions(k: &mut Kernel, b: &Brep, region: RegionId) -> Result<Vec<RegionId>> {
    let mut out = Vec::new();
    // SmRegion.cpp::GetAdjacentRegions: each faceuse shell's mated shells,
    // first-seen regions except self; its helper advances the actual context mark.
    for shell in b.region(region)?.shells.iter().copied() {
        if !b.shell(shell)?.is_faceuse_shell() {
            continue;
        }
        for mate in b.shell(shell)?.mated_shells(k, b)? {
            let id = b
                .shell(mate)?
                .region
                .ok_or_else(|| SolidError::null_pointer("adjacent region"))?;
            if id != region && !out.contains(&id) {
                out.push(id);
            }
        }
    }
    Ok(out)
}
fn collect_regions(k: &mut Kernel, b: &Brep, start: RegionId) -> Result<Vec<RegionId>> {
    let mut out = Vec::new();
    let mut stack = adjacent_regions(k, b, start)?;
    while let Some(current) = stack.pop() {
        if current == start {
            continue;
        }
        // Preserve source CollectRegions' shell->GetRegion lookup, not a more
        // convenient insertion of current. Detached/mismatched shells matter.
        for shell in b.region(current)?.shells.iter().copied() {
            let Some(mate) = b.shell(shell)?.region else {
                continue;
            };
            if mate == start || out.contains(&mate) {
                continue;
            }
            out.push(mate);
            for neighbor in adjacent_regions(k, b, mate)? {
                if neighbor != start && !stack.contains(&neighbor) {
                    stack.push(neighbor);
                }
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Brep {
        super::super::tests::fixture().0
    }

    #[test]
    fn native_root_boolean_can_be_true_while_pointer_reports_are_nonempty() {
        let mut b = fixture();
        b.set_tolerance(0.);
        let mut reports = AssertArray::new();
        let valid = assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions {
                level: CheckLevel::Zero,
                walk: false,
            },
            Some(&mut reports),
        )
        .unwrap();
        // Matches actual SDK Root AssertValid: it tests pointer status, not the
        // presence of pointer reports. Import acceptance needs its own report gate.
        assert!(valid);
        assert!(!reports.is_empty());
    }

    #[test]
    fn given_root_request_does_not_suppress_the_forced_pointer_prefix() {
        let mut b = fixture();
        let face = b.face_list()[0];
        b.face_mut(face).unwrap().surface = None;
        let mut reports = AssertArray::new();
        assert!(!assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions {
                level: CheckLevel::Given(&[8]),
                walk: false
            },
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(reports.reports()[0].test_index, 121);
    }

    #[test]
    fn default_walk_never_promotes_a_root_only_success() {
        let mut b = fixture();
        let error = assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions::default(),
            None,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            SolidError::Unported {
                upstream: "SmEdge::AssertValid",
                ..
            }
        ));
    }

    #[test]
    fn coincidence_query_runs_only_when_the_native_explicit_guard_selects_it() {
        let mut b = fixture();
        assert!(assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions {
                level: CheckLevel::Given(&[8]),
                walk: false
            },
            None
        )
        .unwrap());
        let error = assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions {
                level: CheckLevel::Given(&[15]),
                walk: false,
            },
            None,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            SolidError::Unported {
                upstream: "SmBrep::ContainsCoincidentTopology",
                ..
            }
        ));
    }

    #[test]
    fn damaged_owner_is_not_confused_with_a_legitimately_detached_region() {
        let mut b = fixture();
        let region = b.infinite_region().unwrap();
        b.region_mut(region).unwrap().in_brep = false;
        assert!(bp::regions(&b, None).unwrap().contains(&region));
        let mut reports = AssertArray::new();
        assert!(!assert_valid(
            &mut Kernel::default(),
            &mut b,
            AssertValidOptions {
                level: CheckLevel::Zero,
                walk: false
            },
            Some(&mut reports)
        )
        .unwrap());
        assert_eq!(
            reports.reports()[0].reporting_type,
            labels::OWNING_TOPOLOGY_TYPE
        );
        assert_eq!(reports.reports()[0].test_index, 2);
        b.detach_region_record(region).unwrap();
        b.set_region_list_head(None);
        assert!(bp::regions(&b, None).unwrap().is_empty());
        assert!(b.region(region).is_ok());
    }
}
