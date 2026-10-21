//! Batch input distribution and safe representation of native array aliases.
//!
//! Upstream: maintained `d8ed45a`, `SmTopologyTraverser.cpp`2094--2181.
//! | Rust | Upstream |
//! |------|----------|
//! | [`get_subtopologies`] | `SmTopologyTraverser::GetSubTopologies` |
//! Deviations: typed output-input selectors replace heterogeneous pointer-array
//! casts. Native reset-alias first-element dereferences return checked errors.
//! All collection/order/mark behavior delegates to the shared singular routine.
//! No recursion, geometry normalization or implicit input deduplication.

use super::{get_subtopology, validate, SubTopology, SubTopologyOptions};
use crate::{
    error::{EntityRef, Result, SolidError, SM_ERR_INVALID_INPUT},
    kernel::Kernel,
    topology::Brep,
};

const OP: &str = "topology::topology_traverser::get_subtopologies";

/// Input entries are scoped to the borrowed model. Output selectors represent
/// the source's actual shared-array semantics without unsafe Rust aliases.
/// They read the selected array after reset, not a clone of its original input.
#[derive(Debug, Clone, Copy)]
pub enum SubTopologyInput<'a> {
    /// A separate heterogeneous input array.
    Objects(&'a [EntityRef]),
    /// The region output array is also the input array.
    OutputRegions,
    /// The shell output array is also the input array.
    OutputShells,
    /// The face output array is also the input array.
    OutputFaces,
    /// The loop output array is also the input array.
    OutputLoops,
    /// The edge output array is also the input array.
    OutputEdges,
    /// The vertex output array is also the input array.
    OutputVertices,
}
fn missing() -> SolidError {
    SolidError::invalid_input(OP, "alias input requires an existing output array")
}
fn alias_len(input: SubTopologyInput<'_>, out: &SubTopology) -> Result<usize> {
    macro_rules! len {
        ($field:ident) => {
            out.$field.as_ref().map(Vec::len).ok_or_else(missing)
        };
    }
    match input {
        SubTopologyInput::Objects(v) => Ok(v.len()),
        SubTopologyInput::OutputRegions => len!(regions),
        SubTopologyInput::OutputShells => len!(shells),
        SubTopologyInput::OutputFaces => len!(faces),
        SubTopologyInput::OutputLoops => len!(loops),
        SubTopologyInput::OutputEdges => len!(edges),
        SubTopologyInput::OutputVertices => len!(vertices),
    }
}
fn alias_first(input: SubTopologyInput<'_>, out: &SubTopology) -> Result<EntityRef> {
    macro_rules! first {
        ($field:ident,$kind:ident) => {
            out.$field
                .as_ref()
                .and_then(|v| v.first())
                .copied()
                .map(EntityRef::$kind)
        };
    }
    let first = match input {
        SubTopologyInput::Objects(v) => v.first().copied(),
        SubTopologyInput::OutputRegions => first!(regions, Region),
        SubTopologyInput::OutputShells => first!(shells, Shell),
        SubTopologyInput::OutputFaces => first!(faces, Face),
        SubTopologyInput::OutputLoops => first!(loops, Loop),
        SubTopologyInput::OutputEdges => first!(edges, Edge),
        SubTopologyInput::OutputVertices => first!(vertices, Vertex),
    };
    first.ok_or_else(|| {
        SolidError::invalid_input(OP, "native reset-alias input has no first element")
    })
}

/// Native batch collection with reset/append and shared-array input semantics.
/// Empty input or no requested output returns before reset. Input duplicates
/// remain; the source code's uniqueness comment does not change its Add calls.
/// One Brep input overrides other roots; a second returns native status1007 and
/// retains earlier output writes. A reset alias retains its reset arrays, then
/// returns a checked error instead of dereferencing the emptied input.
#[allow(clippy::too_many_lines)]
pub fn get_subtopologies(
    k: &mut Kernel,
    b: &mut Brep,
    input: SubTopologyInput<'_>,
    out: &mut SubTopology,
    options: SubTopologyOptions,
) -> Result<()> {
    if alias_len(input, out)? == 0 {
        return Ok(());
    }
    if out.mask() == 0 {
        return Ok(());
    }
    if options.reset {
        out.clear();
    }
    // The native plural routine always supplies ALL six arrays to singular,
    // using local arrays for omitted outputs. Keep that behavior: it changes
    // traversal, marks and the safety of partial marked requests.
    let mut work = SubTopology::all();
    macro_rules! take {
        ($field:ident) => {
            if let Some(v) = &mut out.$field {
                work.$field = Some(std::mem::take(v));
            }
        };
    }
    take!(regions);
    take!(shells);
    take!(faces);
    take!(loops);
    take!(edges);
    take!(vertices);
    let result = (|| -> Result<()> {
        let mut root = None;
        if let SubTopologyInput::Objects(objects) = input {
            for object in objects.iter().copied() {
                validate(b, object)?;
                macro_rules! add {
                    ($field:ident,$id:expr) => {
                        work.$field.as_mut().ok_or_else(missing)?.push($id)
                    };
                }
                match object {
                    EntityRef::Brep(_) if root.is_some() => {
                        return Err(SolidError::Upstream {
                            operation: OP,
                            code: SM_ERR_INVALID_INPUT,
                            entity: None,
                        })
                    }
                    EntityRef::Brep(_) => root = Some(object),
                    EntityRef::Region(id) => add!(regions, id),
                    EntityRef::Shell(id) => add!(shells, id),
                    EntityRef::Face(id) => add!(faces, id),
                    EntityRef::Loop(id) => add!(loops, id),
                    EntityRef::Edge(id) => add!(edges, id),
                    EntityRef::Vertex(id) => add!(vertices, id),
                    // Source ignores invalid types while distributing. If one
                    // is first, singular will later reject it and reset arrays.
                    _ => {}
                }
            }
        }
        let root = if let Some(root) = root {
            root
        } else {
            alias_first(input, &work)?
        };
        get_subtopology(
            k,
            b,
            root,
            &mut work,
            SubTopologyOptions {
                reset: false,
                mark: options.mark,
            },
        )
    })();
    // Source requested outputs are directly mutated. Publish their exact partial
    // state on failure too; discard only the native temporary unrequested arrays.
    macro_rules! publish {
        ($field:ident) => {
            if let Some(v) = &mut out.$field {
                *v = work.$field.take().unwrap_or_default();
            }
        };
    }
    publish!(regions);
    publish!(shells);
    publish!(faces);
    publish!(loops);
    publish!(edges);
    publish!(vertices);
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topology::BrepId;
    fn fixture() -> (
        Brep,
        crate::topology::FaceId,
        crate::topology::LoopId,
        crate::topology::EdgeuseId,
    ) {
        crate::topology::brep_validation::tests::fixture()
    }

    #[test]
    fn empty_input_returns_before_reset_even_with_existing_output() {
        let (mut b, f, _, _) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology {
            faces: Some(vec![f]),
            ..SubTopology::default()
        };
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.faces, Some(vec![f]));
        out.edges = Some(Vec::new());
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::OutputEdges,
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.faces, Some(vec![f]));
    }

    #[test]
    fn duplicate_input_faces_and_input_vertices_are_retained() {
        let (mut b, f, _, use_) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology {
            faces: Some(vec![]),
            ..SubTopology::default()
        };
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[EntityRef::Face(f), EntityRef::Face(f)]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.faces, Some(vec![f, f]));
        let v = b
            .vertexuse(b.edgeuse(use_).unwrap().vertexuse.unwrap())
            .unwrap()
            .vertex
            .unwrap();
        let mut out = SubTopology {
            vertices: Some(vec![]),
            ..SubTopology::default()
        };
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[EntityRef::Vertex(v)]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.vertices, Some(vec![v]));
    }

    #[test]
    fn second_brep_returns_native_status_with_prior_distribution_retained() {
        let (mut b, f, _, _) = fixture();
        let mut k = Kernel::default();
        let root = EntityRef::Brep(BrepId::from_index(0));
        let mut out = SubTopology::all();
        let error = get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[EntityRef::Face(f), root, root]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap_err();
        assert!(matches!(error, SolidError::Upstream { code: 1007, .. }));
        assert_eq!(out.faces, Some(vec![f]));
        assert_eq!(out.edges, Some(vec![]));
    }

    #[test]
    fn one_brep_overrides_other_inputs_and_invalid_first_use_resets_all_work() {
        let (mut b, f, _, use_) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology::all();
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[
                EntityRef::Edgeuse(use_),
                EntityRef::Brep(BrepId::from_index(0)),
                EntityRef::Face(f),
            ]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.faces, Some(vec![f]));
        let error = get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::Objects(&[EntityRef::Edgeuse(use_), EntityRef::Face(f)]),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap_err();
        assert!(matches!(error, SolidError::Upstream { code: 1001, .. }));
        assert_eq!(out, SubTopology::all());
    }

    #[test]
    fn alias_append_uses_actual_output_and_reset_alias_keeps_the_clear_on_error() {
        let (mut b, _, _, _) = fixture();
        let mut k = Kernel::default();
        let e = b.edge_list()[0];
        let mut out = SubTopology {
            edges: Some(vec![e, e]),
            vertices: Some(vec![]),
            ..SubTopology::default()
        };
        get_subtopologies(
            &mut k,
            &mut b,
            SubTopologyInput::OutputEdges,
            &mut out,
            SubTopologyOptions {
                reset: false,
                ..SubTopologyOptions::default()
            },
        )
        .unwrap();
        assert_eq!(out.edges, Some(vec![e, e]));
        assert_eq!(out.vertices.as_ref().unwrap().len(), 2);
        assert!(matches!(
            get_subtopologies(
                &mut k,
                &mut b,
                SubTopologyInput::OutputEdges,
                &mut out,
                SubTopologyOptions::default()
            ),
            Err(SolidError::InvalidInput { .. })
        ));
        assert_eq!(out.edges, Some(vec![]));
        assert_eq!(out.vertices, Some(vec![]));
    }

    #[test]
    fn plural_partial_marked_request_supplies_missing_work_arrays() {
        let (mut b, f, _, _) = fixture();
        let mut k = Kernel::default();
        crate::geom::context::with_mark_lock(
            &mut k,
            crate::geom::context::MarkType::Mark2.code(),
            |k, _| {
                let mut out = SubTopology {
                    vertices: Some(vec![]),
                    ..SubTopology::default()
                };
                get_subtopologies(
                    k,
                    &mut b,
                    SubTopologyInput::Objects(&[EntityRef::Face(f)]),
                    &mut out,
                    SubTopologyOptions {
                        reset: true,
                        mark: crate::geom::context::MarkType::Mark2,
                    },
                )
                .unwrap();
                assert_eq!(out.vertices.as_ref().unwrap().len(), 3);
                assert!(out.faces.is_none());
            },
        )
        .unwrap();
    }
}
