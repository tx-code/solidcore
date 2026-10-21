//! Shared native descendant assertion dispatch over the actual collected graph.
//!
//! Upstream: maintained `d8ed45a`, `SmBrep.cpp`35303--35495.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_subtopology`] | `SmBrep::AssertSubTopology` |
//! Deviations: checked arena/representation failures propagate; no recursion.
//! Summary-report and composite branches disabled in source stay disabled.

use super::{AssertValidOptions, CheckLevel};
use crate::topology::{
    topology_traverser::{get_subtopology, SubTopology, SubTopologyOptions},
    Brep,
};
use crate::{
    error::{EntityRef, Result},
    geom::assert_array::AssertArray,
    kernel::Kernel,
};

/// Collect every descendant array first, then assert Region, Shell, Face, Loop,
/// Edge and Vertex in native order. The receiver is excluded for non-root input.
/// Child calls receive the level, no Given requests and no walk. The source's
/// ordinary collector status is ignored; unavailable/checked failures propagate.
/// A successful selected walk does not establish whole-model/import acceptance.
pub fn assert_subtopology(
    k: &mut Kernel,
    b: &mut Brep,
    object: EntityRef,
    level: CheckLevel<'_>,
    mut reports: Option<&mut AssertArray>,
) -> Result<bool> {
    let mut out = SubTopology::all();
    super::local::ignore_native_status(get_subtopology(
        k,
        b,
        object,
        &mut out,
        SubTopologyOptions::default(),
    ))?;
    let options = AssertValidOptions {
        level: if matches!(level, CheckLevel::Given(_)) {
            CheckLevel::Given(&[])
        } else {
            level
        },
        walk: false,
    };
    let mut valid = true;
    for id in out.regions.into_iter().flatten() {
        valid &= super::assert_region(k, b, id, options, reports.as_deref_mut())?;
    }
    for id in out.shells.into_iter().flatten() {
        valid &= super::assert_shell(k, b, id, options, reports.as_deref_mut())?;
    }
    for id in out.faces.into_iter().flatten() {
        valid &= super::assert_face(k, b, id, options, reports.as_deref_mut())?;
    }
    for id in out.loops.into_iter().flatten() {
        valid &= super::assert_loop(k, b, id, options, reports.as_deref_mut())?;
    }
    for id in out.edges.into_iter().flatten() {
        valid &= super::assert_edge(k, b, id, options, reports.as_deref_mut())?;
    }
    for id in out.vertices.into_iter().flatten() {
        valid &= super::assert_vertex(k, b, id, options, reports.as_deref_mut())?;
    }
    Ok(valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ordinary_invalid_use_collector_status_is_ignored_without_summary_reports() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let mut reports = AssertArray::new();
        assert!(assert_subtopology(
            &mut Kernel::default(),
            &mut b,
            EntityRef::Edgeuse(use_),
            CheckLevel::Zero,
            Some(&mut reports)
        )
        .unwrap());
        assert!(reports.is_empty());
    }
}
