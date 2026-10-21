//! Native optional-output subgraph collection, including append and target marks.
//!
//! Upstream: maintained `d8ed45a`, `SmTopologyTraverser.cpp`1667--2066.
//! | Rust | Upstream |
//! |------|----------|
//! | [`get_subtopology`] | `SmTopologyTraverser::GetSubTopology` |
//! Deviations: typed ids replace pointers; native partial-mark-output null
//! dereferences are checked errors. No recursion. Source Faceuse-marking and
//! duplicate-output behavior are retained; these queries do not repair graphs.

use super::Marks;
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::context::{with_mark_lock, MarkType, ALL_MARKS};
use crate::kernel::Kernel;
use crate::topology::{
    brep::p1 as bp, face::p2 as face, loopuse, shell, topology::Topology, Brep, EdgeId, FaceId,
    LoopId, RegionId, ShellId, VertexId,
};

const OP: &str = "topology::topology_traverser::get_subtopology";

/// Requested output arrays. None is a native NULL output pointer and stays None.
/// Existing array contents are retained for non-root append requests, including
/// duplicate inputs. Native output quirks can also retain duplicate faces.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct SubTopology {
    /// Region output.
    pub regions: Option<Vec<RegionId>>,
    /// Shell output.
    pub shells: Option<Vec<ShellId>>,
    /// Face output.
    pub faces: Option<Vec<FaceId>>,
    /// Loop output.
    pub loops: Option<Vec<LoopId>>,
    /// Edge output.
    pub edges: Option<Vec<EdgeId>>,
    /// Vertex output.
    pub vertices: Option<Vec<VertexId>>,
}
impl SubTopology {
    /// Request all six initially empty arrays.
    #[must_use]
    pub fn all() -> Self {
        Self {
            regions: Some(Vec::new()),
            shells: Some(Vec::new()),
            faces: Some(Vec::new()),
            loops: Some(Vec::new()),
            edges: Some(Vec::new()),
            vertices: Some(Vec::new()),
        }
    }
    fn mask(&self) -> u8 {
        u8::from(self.regions.is_some())
            | u8::from(self.shells.is_some()) << 1
            | u8::from(self.faces.is_some()) << 2
            | u8::from(self.loops.is_some()) << 3
            | u8::from(self.edges.is_some()) << 4
            | u8::from(self.vertices.is_some()) << 5
    }
    fn clear(&mut self) {
        if let Some(v) = &mut self.regions {
            v.clear();
        }
        if let Some(v) = &mut self.shells {
            v.clear();
        }
        if let Some(v) = &mut self.faces {
            v.clear();
        }
        if let Some(v) = &mut self.loops {
            v.clear();
        }
        if let Some(v) = &mut self.edges {
            v.clear();
        }
        if let Some(v) = &mut self.vertices {
            v.clear();
        }
    }
}

/// Source reset/mark arguments. A real target mark belongs to the caller: this
/// routine neither increments nor automatically locks it. Native nested getters
/// can reuse an unlocked target mark; the original resulting behavior is kept.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubTopologyOptions {
    /// Reset requested outputs first; root and invalid types always reset.
    pub reset: bool,
    /// Target mark used to filter output (`NoMark` disables target filtering).
    pub mark: MarkType,
}
impl Default for SubTopologyOptions {
    fn default() -> Self {
        Self {
            reset: true,
            mark: MarkType::NoMark,
        }
    }
}

#[derive(Default)]
struct Work {
    regions: Vec<RegionId>,
    shells: Vec<ShellId>,
    faces: Vec<FaceId>,
    loops: Vec<LoopId>,
    edges: Vec<EdgeId>,
    vertices: Vec<VertexId>,
}
impl Work {
    fn entities(&self) -> impl Iterator<Item = EntityRef> + '_ {
        self.regions
            .iter()
            .copied()
            .map(EntityRef::Region)
            .chain(self.shells.iter().copied().map(EntityRef::Shell))
            .chain(self.faces.iter().copied().map(EntityRef::Face))
            .chain(self.loops.iter().copied().map(EntityRef::Loop))
            .chain(self.edges.iter().copied().map(EntityRef::Edge))
            .chain(self.vertices.iter().copied().map(EntityRef::Vertex))
    }
    fn insert(&mut self, object: EntityRef) {
        match object {
            EntityRef::Region(v) => self.regions.insert(0, v),
            EntityRef::Shell(v) => self.shells.insert(0, v),
            EntityRef::Face(v) => self.faces.insert(0, v),
            EntityRef::Loop(v) => self.loops.insert(0, v),
            EntityRef::Edge(v) => self.edges.insert(0, v),
            EntityRef::Vertex(v) => self.vertices.insert(0, v),
            _ => {}
        }
    }
    fn remove(&mut self, object: EntityRef) {
        match object {
            EntityRef::Region(_) => {
                self.regions.remove(0);
            }
            EntityRef::Shell(_) => {
                self.shells.remove(0);
            }
            EntityRef::Face(_) => {
                self.faces.remove(0);
            }
            EntityRef::Loop(_) => {
                self.loops.remove(0);
            }
            EntityRef::Edge(_) => {
                self.edges.remove(0);
            }
            EntityRef::Vertex(_) => {
                self.vertices.remove(0);
            }
            _ => {}
        }
    }
}

fn validate(b: &Brep, object: EntityRef) -> Result<()> {
    match object {
        EntityRef::Brep(_) => {}
        EntityRef::Region(id) => {
            b.region(id)?;
        }
        EntityRef::Shell(id) => {
            b.shell(id)?;
        }
        EntityRef::Face(id) => {
            b.face(id)?;
        }
        EntityRef::Loop(id) => {
            b.loop_(id)?;
        }
        EntityRef::Edge(id) => {
            b.edge(id)?;
        }
        EntityRef::Vertex(id) => {
            b.vertex(id)?;
        }
        EntityRef::Faceuse(id) => {
            b.faceuse(id)?;
        }
        EntityRef::Loopuse(id) => {
            b.loopuse(id)?;
        }
        EntityRef::Edgeuse(id) => {
            b.edgeuse(id)?;
        }
        EntityRef::Vertexuse(id) => {
            b.vertexuse(id)?;
        }
    }
    Ok(())
}
fn marked(k: &Kernel, b: &Brep, entity: EntityRef, mark: MarkType) -> Result<bool> {
    let key = crate::topology::brep::p2::object_ref(b, entity)?.key;
    Ok(b.notifications()
        .topology
        .get(&key)
        .copied()
        .unwrap_or_else(Topology::default)
        .is_marked(&k.context, mark))
}
fn stamp(k: &Kernel, b: &mut Brep, entity: EntityRef, mark: MarkType) -> Result<()> {
    if mark != MarkType::NoMark {
        b.notification_topology_mut(entity)?
            .mark(Some(&k.context), mark);
    }
    Ok(())
}
fn shells(k: &mut Kernel, b: &mut Brep) -> Result<Vec<ShellId>> {
    let mut marks = Marks::from_brep(b)?;
    let result = bp::shells_with_marks(k, b, None, &mut marks);
    marks.store_in_brep(b)?;
    result
}
fn wires(k: &mut Kernel, b: &mut Brep, shell: ShellId) -> Result<Vec<EdgeId>> {
    let mut marks = Marks::from_brep(b)?;
    let result = shell::wire_edges_with_marks(k, b, shell, &mut marks);
    marks.store_in_brep(b)?;
    result
}

fn cull<T: Copy>(
    k: &Kernel,
    b: &mut Brep,
    values: &mut Vec<T>,
    entity: impl Fn(T) -> EntityRef,
    mark: MarkType,
) -> Result<()> {
    // Native culls in place. On a checked access error, retain all prior culls
    // and marks while keeping the unvisited suffix; never publish a fresh list.
    let mut i = 0;
    while i < values.len() {
        let v = values
            .get(i)
            .copied()
            .ok_or_else(|| SolidError::null_pointer(OP))?;
        if marked(k, b, entity(v), mark)? {
            values.remove(i);
        } else {
            stamp(k, b, entity(v), mark)?;
            i += 1;
        }
    }
    Ok(())
}
fn root(k: &mut Kernel, b: &mut Brep, out: &mut SubTopology, mark: MarkType) -> Result<()> {
    if let Some(v) = &mut out.regions {
        *v = bp::regions(b, None)?;
    }
    if let Some(v) = &mut out.shells {
        *v = shells(k, b)?;
    }
    let faces = if out.faces.is_some() || out.loops.is_some() {
        bp::faces(b, None)?
    } else {
        Vec::new()
    };
    if let Some(v) = &mut out.faces {
        v.clone_from(&faces);
    }
    if let Some(v) = &mut out.loops {
        for f in faces {
            v.extend(face::loops(b, f, false, None)?);
        }
    }
    if let Some(v) = &mut out.edges {
        *v = bp::edges(b, None)?;
    }
    if let Some(v) = &mut out.vertices {
        *v = bp::vertices(b, None)?;
    }
    if mark != MarkType::NoMark {
        if let Some(v) = &mut out.regions {
            cull(k, b, v, EntityRef::Region, mark)?;
        }
        if let Some(v) = &mut out.shells {
            cull(k, b, v, EntityRef::Shell, mark)?;
        }
        if let Some(v) = &mut out.faces {
            cull(k, b, v, EntityRef::Face, mark)?;
        }
        if let Some(v) = &mut out.loops {
            cull(k, b, v, EntityRef::Loop, mark)?;
        }
        if let Some(v) = &mut out.edges {
            cull(k, b, v, EntityRef::Edge, mark)?;
        }
        if let Some(v) = &mut out.vertices {
            cull(k, b, v, EntityRef::Vertex, mark)?;
        }
    }
    Ok(())
}

fn stages(
    k: &mut Kernel,
    b: &mut Brep,
    work: &mut Work,
    mut mask: u8,
    mark: MarkType,
) -> Result<()> {
    mask &= !1;
    if mask != 0 {
        for &region in &work.regions {
            for shell in b.region(region)?.shells.clone() {
                if !marked(k, b, EntityRef::Shell(shell), mark)? {
                    work.shells.push(shell);
                    stamp(k, b, EntityRef::Shell(shell), mark)?;
                }
            }
        }
    }
    mask &= !2;
    if mask != 0 {
        for &s in &work.shells {
            for use_ in b.shell(s)?.get_faceuses()? {
                if let Some(f) = b.faceuse(use_)?.face {
                    if !marked(k, b, EntityRef::Face(f), mark)? {
                        work.faces.push(f);
                        // Literal source line1911 marks the USE although the guard tests
                        // its Face. Duplicate face entries are observable and retained.
                        stamp(k, b, EntityRef::Faceuse(use_), mark)?;
                    }
                }
            }
            for edge in wires(k, b, s)? {
                if !marked(k, b, EntityRef::Edge(edge), mark)? {
                    work.edges.push(edge);
                    stamp(k, b, EntityRef::Edge(edge), mark)?;
                }
            }
            if let Some(v) = b.shell(s)?.vertex(b)? {
                if !marked(k, b, EntityRef::Vertex(v), mark)? {
                    work.vertices.push(v);
                    stamp(k, b, EntityRef::Vertex(v), mark)?;
                }
            }
        }
    }
    mask &= !4;
    if mask != 0 {
        for &f in &work.faces {
            for l in face::loops(b, f, false, None)? {
                if !marked(k, b, EntityRef::Loop(l), mark)? {
                    work.loops.push(l);
                    stamp(k, b, EntityRef::Loop(l), mark)?;
                }
            }
        }
    }
    mask &= !8;
    if mask != 0 {
        for &l in &work.loops {
            for e in loopuse::loop_edges(b, l)?.into_iter().flatten() {
                if !marked(k, b, EntityRef::Edge(e), mark)? {
                    work.edges.push(e);
                    stamp(k, b, EntityRef::Edge(e), mark)?;
                }
            }
            for v in loopuse::loop_vertices(b, l)?.into_iter().flatten() {
                if !marked(k, b, EntityRef::Vertex(v), mark)? {
                    work.vertices.push(v);
                    stamp(k, b, EntityRef::Vertex(v), mark)?;
                }
            }
        }
    }
    mask &= !16;
    if mask != 0 {
        for &e in &work.edges {
            for v in crate::topology::edge::vertices(b, e, None)?
                .into_iter()
                .flatten()
            {
                if !marked(k, b, EntityRef::Vertex(v), mark)? {
                    work.vertices.push(v);
                    stamp(k, b, EntityRef::Vertex(v), mark)?;
                }
            }
        }
    }
    Ok(())
}

fn append<T: Copy>(
    k: &Kernel,
    b: &mut Brep,
    src: &[T],
    dst: &mut Option<Vec<T>>,
    entity: impl Fn(T) -> EntityRef,
    mark: MarkType,
) -> Result<()> {
    if let Some(dst) = dst {
        for v in src {
            if !marked(k, b, entity(*v), mark)? {
                dst.push(*v);
                stamp(k, b, entity(*v), mark)?;
            }
        }
    }
    Ok(())
}
fn nonroot(
    k: &mut Kernel,
    b: &mut Brep,
    object: EntityRef,
    out: &mut SubTopology,
    mask: u8,
    target: MarkType,
    mark: MarkType,
) -> Result<()> {
    let marking = target != MarkType::NoMark;
    macro_rules! input {
        ($field:ident) => {
            if marking {
                out.$field.clone().unwrap_or_default()
            } else {
                out.$field.as_mut().map(std::mem::take).unwrap_or_default()
            }
        };
    }
    let mut w = Work {
        regions: input!(regions),
        shells: input!(shells),
        faces: input!(faces),
        loops: input!(loops),
        edges: input!(edges),
        vertices: input!(vertices),
    };
    let result = (|| -> Result<()> {
        if marking {
            for e in w.entities() {
                stamp(k, b, e, target)?;
            }
        }
        // Source leaves pMy* NULL for each unrequested marked output and then
        // dereferences all six in order. Preserve its defined marking prefix,
        // but return a checked error instead of following that undefined access.
        macro_rules! work_mark {
            ($field:ident,$kind:ident,$bit:expr) => {
                if marking && mask & $bit == 0 {
                    return Err(SolidError::invalid_input(
                        OP,
                        "native marked partial outputs have a NULL work array",
                    ));
                }
                for id in &w.$field {
                    stamp(k, b, EntityRef::$kind(*id), mark)?;
                }
            };
        }
        work_mark!(regions, Region, 1);
        work_mark!(shells, Shell, 2);
        work_mark!(faces, Face, 4);
        work_mark!(loops, Loop, 8);
        work_mark!(edges, Edge, 16);
        work_mark!(vertices, Vertex, 32);
        w.insert(object);
        stages(k, b, &mut w, mask, mark)?;
        w.remove(object);
        if marking {
            append(
                k,
                b,
                &w.regions,
                &mut out.regions,
                EntityRef::Region,
                target,
            )?;
            append(k, b, &w.shells, &mut out.shells, EntityRef::Shell, target)?;
            append(k, b, &w.faces, &mut out.faces, EntityRef::Face, target)?;
            append(k, b, &w.loops, &mut out.loops, EntityRef::Loop, target)?;
            append(k, b, &w.edges, &mut out.edges, EntityRef::Edge, target)?;
            append(
                k,
                b,
                &w.vertices,
                &mut out.vertices,
                EntityRef::Vertex,
                target,
            )?;
        }
        Ok(())
    })();
    if !marking {
        macro_rules! publish {
            ($field:ident) => {
                if let Some(v) = &mut out.$field {
                    *v = w.$field;
                }
            };
        }
        publish!(regions);
        publish!(shells);
        publish!(faces);
        publish!(loops);
        publish!(edges);
        publish!(vertices);
    }
    result
}

/// Collect the actual native subgraph (excluding a non-root receiver), preserving
/// optional outputs, reset/append, source order and mark effects. Use objects are
/// rejected after clearing requested arrays; an all-None request returns first.
/// Work-mark locks are released on failures, while already-written outputs/marks
/// remain visible. A root always replaces its outputs, even in append mode.
pub fn get_subtopology(
    k: &mut Kernel,
    b: &mut Brep,
    object: EntityRef,
    out: &mut SubTopology,
    options: SubTopologyOptions,
) -> Result<()> {
    let mask = out.mask();
    if mask == 0 {
        return Ok(());
    }
    validate(b, object)?;
    let valid = matches!(
        object,
        EntityRef::Brep(_)
            | EntityRef::Region(_)
            | EntityRef::Shell(_)
            | EntityRef::Face(_)
            | EntityRef::Loop(_)
            | EntityRef::Edge(_)
            | EntityRef::Vertex(_)
    );
    if options.reset || matches!(object, EntityRef::Brep(_)) || !valid {
        out.clear();
    }
    if !valid {
        return Err(SolidError::sm_err(OP));
    }
    if matches!(object, EntityRef::Brep(_)) {
        return root(k, b, out, options.mark);
    }
    with_mark_lock(k, ALL_MARKS & !options.mark.code(), |k, mark| {
        nonroot(k, b, object, out, mask, options.mark, mark)
    })?
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topology::{BrepId, EdgeuseId, VertexId};
    fn fixture() -> (Brep, FaceId, LoopId, EdgeuseId) {
        crate::topology::brep_validation::tests::fixture()
    }

    #[test]
    fn region_retains_faceuse_marking_quirk_but_loop_and_edge_order_are_stable() {
        let (mut b, face, loop_, _) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology::all();
        let region = b.infinite_region().unwrap();
        let edges = b.edge_list().to_vec();
        get_subtopology(
            &mut k,
            &mut b,
            EntityRef::Region(region),
            &mut out,
            SubTopologyOptions::default(),
        )
        .unwrap();
        assert_eq!(out.regions, Some(vec![]));
        assert_eq!(out.faces, Some(vec![face, face]));
        assert_eq!(out.loops, Some(vec![loop_]));
        assert_eq!(out.edges, Some(vec![edges[2], edges[0], edges[1]]));
        assert!(!marked(&k, &b, EntityRef::Face(face), MarkType::Mark).unwrap());
        let fu = b.face(face).unwrap().faceuse.unwrap();
        assert!(marked(&k, &b, EntityRef::Faceuse(fu), MarkType::Mark).unwrap());
    }

    #[test]
    fn append_traverses_seeded_higher_arrays_instead_of_only_the_receiver() {
        let (mut b, _, _, use_) = fixture();
        let mut k = Kernel::default();
        let edge = b.edgeuse(use_).unwrap().edge.unwrap();
        let mut out = SubTopology {
            regions: Some(vec![]),
            vertices: Some(vec![]),
            ..SubTopology::default()
        };
        let options = SubTopologyOptions {
            reset: false,
            ..SubTopologyOptions::default()
        };
        get_subtopology(&mut k, &mut b, EntityRef::Edge(edge), &mut out, options).unwrap();
        assert_eq!(out.vertices.as_ref().unwrap().len(), 2);
        out.vertices.as_mut().unwrap().clear();
        out.regions = Some(vec![b.infinite_region().unwrap()]);
        get_subtopology(&mut k, &mut b, EntityRef::Edge(edge), &mut out, options).unwrap();
        assert_eq!(out.vertices.as_ref().unwrap().len(), 3);
        assert!(out.edges.is_none());
    }

    #[test]
    fn root_append_is_still_reset_and_face_only_collection_keeps_other_outputs_absent() {
        let (mut b, face, _, _) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology {
            faces: Some(vec![face, face]),
            ..SubTopology::default()
        };
        get_subtopology(
            &mut k,
            &mut b,
            EntityRef::Brep(BrepId::from_index(0)),
            &mut out,
            SubTopologyOptions {
                reset: false,
                ..SubTopologyOptions::default()
            },
        )
        .unwrap();
        assert_eq!(out.faces, Some(vec![face]));
        assert!(out.loops.is_none());
    }

    #[test]
    fn invalid_use_resets_outputs_but_no_output_request_does_not_access_a_dangling_id() {
        let (mut b, face, _, use_) = fixture();
        let mut k = Kernel::default();
        let mut out = SubTopology {
            faces: Some(vec![face]),
            ..SubTopology::default()
        };
        assert!(matches!(
            get_subtopology(
                &mut k,
                &mut b,
                EntityRef::Edgeuse(use_),
                &mut out,
                SubTopologyOptions {
                    reset: false,
                    ..SubTopologyOptions::default()
                }
            ),
            Err(SolidError::Upstream { code: 1001, .. })
        ));
        assert_eq!(out.faces, Some(vec![]));
        get_subtopology(
            &mut k,
            &mut b,
            EntityRef::Vertex(VertexId::from_index(u32::MAX)),
            &mut SubTopology::default(),
            SubTopologyOptions::default(),
        )
        .unwrap();
    }

    #[test]
    fn marked_partial_output_retains_target_marking_prefix_and_releases_work_lock() {
        let (mut b, face, _, use_) = fixture();
        let mut k = Kernel::default();
        let edge = b.edgeuse(use_).unwrap().edge.unwrap();
        with_mark_lock(&mut k, MarkType::Mark2.code(), |k, _| {
            let mut out = SubTopology {
                edges: Some(vec![edge]),
                ..SubTopology::default()
            };
            assert!(matches!(
                get_subtopology(
                    k,
                    &mut b,
                    EntityRef::Face(face),
                    &mut out,
                    SubTopologyOptions {
                        reset: false,
                        mark: MarkType::Mark2
                    }
                ),
                Err(SolidError::InvalidInput { .. })
            ));
            assert_eq!(out.edges, Some(vec![edge]));
            assert!(marked(k, &b, EntityRef::Edge(edge), MarkType::Mark2).unwrap());
            with_mark_lock(k, ALL_MARKS, |_, mark| assert_eq!(mark, MarkType::Mark)).unwrap();
        })
        .unwrap();
    }

    #[test]
    fn checked_failure_publishes_the_native_partial_array_and_preserves_other_metadata() {
        let (mut b, _, loop_, _) = fixture();
        let mut k = Kernel::default();
        let lu = b.loop_(loop_).unwrap().loopuse.unwrap();
        b.loopuse_mut(lu).unwrap().start =
            crate::topology::LoopuseStart::Edgeuse(Some(EdgeuseId::from_index(u32::MAX)));
        b.notification_topology_mut(EntityRef::Loop(loop_))
            .unwrap()
            .set_all_flags(37);
        let mut out = SubTopology {
            loops: Some(vec![]),
            vertices: Some(vec![]),
            ..SubTopology::default()
        };
        assert!(get_subtopology(
            &mut k,
            &mut b,
            EntityRef::Loop(loop_),
            &mut out,
            SubTopologyOptions::default()
        )
        .is_err());
        assert_eq!(out.loops, Some(vec![loop_]));
        let key = crate::topology::brep::p2::object_ref(&b, EntityRef::Loop(loop_))
            .unwrap()
            .key;
        assert_eq!(
            b.notifications().topology.get(&key).unwrap().all_flags(),
            37
        );
        with_mark_lock(&mut k, ALL_MARKS, |_, mark| {
            assert_eq!(mark, MarkType::Mark);
        })
        .unwrap();
    }
}
