// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host-only raw subtopology observer; actual library collection owns behavior.
use serde_json::{json, Value};
use solidcore::{
    error::{EntityRef, Result, SolidError},
    geom::context::{with_mark_lock, MarkType},
    kernel::Kernel,
    topology::{
        brep::p2::object_ref,
        topology_traverser::{
            get_subtopologies, get_subtopology, SubTopology, SubTopologyInput, SubTopologyOptions,
        },
        Brep, EdgeId, EdgeuseId, FaceId, FaceuseId, LoopId, LoopuseId, RegionId, ShellId, VertexId,
        VertexuseId,
    },
};

pub struct Fixture {
    pub region: RegionId,
    pub shell: ShellId,
    pub face: FaceId,
    pub loop_: LoopId,
    pub edge: EdgeId,
    pub vertex: VertexId,
    pub fu: FaceuseId,
    pub lu: LoopuseId,
    pub eu: EdgeuseId,
    pub vu: VertexuseId,
}
fn invalid() -> SolidError {
    SolidError::invalid_input("subtopology observer", "invalid query configuration")
}
pub fn object(f: &Fixture, kind: &str) -> Result<EntityRef> {
    Ok(match kind {
        "brep" => EntityRef::Brep(solidcore::topology::BrepId::from_index(0)),
        "region" => EntityRef::Region(f.region),
        "shell" => EntityRef::Shell(f.shell),
        "face" => EntityRef::Face(f.face),
        "loop" => EntityRef::Loop(f.loop_),
        "edge" => EntityRef::Edge(f.edge),
        "vertex" => EntityRef::Vertex(f.vertex),
        "faceuse" => EntityRef::Faceuse(f.fu),
        "loopuse" => EntityRef::Loopuse(f.lu),
        "edgeuse" => EntityRef::Edgeuse(f.eu),
        "vertexuse" => EntityRef::Vertexuse(f.vu),
        _ => return Err(invalid()),
    })
}
fn batch_input(
    f: &Fixture,
    recipe: &str,
) -> Result<(Vec<EntityRef>, Option<SubTopologyInput<'static>>)> {
    let alias = match recipe {
        "aliasregions" => Some(SubTopologyInput::OutputRegions),
        "aliasshells" => Some(SubTopologyInput::OutputShells),
        "aliasfaces" => Some(SubTopologyInput::OutputFaces),
        "aliasloops" => Some(SubTopologyInput::OutputLoops),
        "aliasedges" => Some(SubTopologyInput::OutputEdges),
        "aliasvertices" => Some(SubTopologyInput::OutputVertices),
        _ => None,
    };
    if alias.is_some() {
        return Ok((Vec::new(), alias));
    }
    let kinds = match recipe {
        "empty" => vec![],
        "duplicate" => vec!["face", "face"],
        "mixed" => vec!["face", "edge", "vertex", "loop", "shell", "region"],
        "faceedge" => vec!["face", "edge"],
        "mixbrep" => vec!["face", "brep", "edge"],
        "twobrep" => vec!["brep", "brep"],
        "doublemixed" => vec!["face", "brep", "edge", "brep"],
        "firstuse" => vec!["edgeuse", "face"],
        "lateuse" => vec!["face", "edgeuse"],
        "alluses" => vec!["faceuse", "loopuse", "edgeuse", "vertexuse"],
        other => vec![other],
    };
    Ok((
        kinds
            .into_iter()
            .map(|kind| object(f, kind))
            .collect::<Result<Vec<_>>>()?,
        None,
    ))
}
fn entities(b: &Brep) -> Vec<EntityRef> {
    let mut v = vec![EntityRef::Brep(solidcore::topology::BrepId::from_index(0))];
    v.extend(b.region_list().iter().copied().map(EntityRef::Region));
    v.extend(b.shells().iter().map(|(id, _)| EntityRef::Shell(id)));
    v.extend(b.face_list().iter().copied().map(EntityRef::Face));
    v.extend(b.loops().iter().map(|(id, _)| EntityRef::Loop(id)));
    v.extend(b.edge_list().iter().copied().map(EntityRef::Edge));
    v.extend(b.vertex_list().iter().copied().map(EntityRef::Vertex));
    v.extend(b.faceuses().iter().map(|(id, _)| EntityRef::Faceuse(id)));
    v.extend(b.loopuses().iter().map(|(id, _)| EntityRef::Loopuse(id)));
    for (_, e) in b.edges().iter() {
        v.extend(e.edgeuses.iter().copied().map(EntityRef::Edgeuse));
    }
    for (_, vertex) in b.vertices().iter() {
        v.extend(vertex.vertexuses.iter().copied().map(EntityRef::Vertexuse));
    }
    v
}
pub struct Observed {
    pub status: i32,
    pub output: Value,
    pub advances: Value,
    pub marks: Value,
}
pub fn observe(
    k: &mut Kernel,
    b: &mut Brep,
    mode: &str,
    selection: &str,
    f: &Fixture,
) -> Result<Observed> {
    let config = selection
        .split(',')
        .map(str::parse::<i32>)
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| invalid())?;
    let [mask, reset, target, lock, repeats, seed, premark] = config.as_slice() else {
        return Err(invalid());
    };
    if !(0..=63).contains(mask) || !(1..=2).contains(repeats) || !(0..=2).contains(seed) {
        return Err(invalid());
    }
    let target = MarkType::from_code(*target).ok_or_else(invalid)?;
    let kind = mode
        .strip_prefix("sub_")
        .ok_or_else(invalid)?
        .split('_')
        .next()
        .ok_or_else(invalid)?;
    let batch = kind.strip_prefix("batch");
    let (objects, alias) = if let Some(recipe) = batch {
        batch_input(f, recipe)?
    } else {
        (Vec::new(), None)
    };
    let root = if batch.is_some() {
        if let Some(alias) = alias {
            Some(match alias {
                SubTopologyInput::OutputRegions => EntityRef::Region(f.region),
                SubTopologyInput::OutputShells => EntityRef::Shell(f.shell),
                SubTopologyInput::OutputFaces => EntityRef::Face(f.face),
                SubTopologyInput::OutputLoops => EntityRef::Loop(f.loop_),
                SubTopologyInput::OutputEdges => EntityRef::Edge(f.edge),
                SubTopologyInput::OutputVertices => EntityRef::Vertex(f.vertex),
                SubTopologyInput::Objects(_) => return Err(invalid()),
            })
        } else {
            objects.first().copied()
        }
    } else {
        Some(object(f, kind)?)
    };
    let mut run = |k: &mut Kernel| -> Result<Observed> {
        let mut out = SubTopology::default();
        macro_rules! input {
            ($field:ident,$flag:expr,$id:expr) => {
                if mask & $flag != 0 {
                    out.$field = Some(
                        std::iter::repeat_n($id, usize::try_from(*seed).map_err(|_| invalid())?)
                            .collect(),
                    );
                }
            };
        }
        input!(regions, 1, f.region);
        input!(shells, 2, f.shell);
        input!(faces, 4, f.face);
        input!(loops, 8, f.loop_);
        input!(edges, 16, f.edge);
        input!(vertices, 32, f.vertex);
        let pool = entities(b);
        if target != MarkType::NoMark {
            for entity in pool
                .iter()
                .copied()
                .filter(|e| *premark == 2 || (*premark == 1 && Some(*e) == root))
            {
                b.notification_topology_mut(entity)?
                    .mark(Some(&k.context), target);
            }
        }
        let types = [
            MarkType::Mark,
            MarkType::Mark2,
            MarkType::Mark3,
            MarkType::MarkIo,
            MarkType::MarkAssert,
        ];
        let before = types.map(|m| k.context.current_mark(m));
        let mut status = 1000;
        for _ in 0..*repeats {
            let options = SubTopologyOptions {
                reset: *reset != 0,
                mark: target,
            };
            let result = if batch.is_some() {
                get_subtopologies(
                    k,
                    b,
                    alias.unwrap_or(SubTopologyInput::Objects(&objects)),
                    &mut out,
                    options,
                )
            } else {
                get_subtopology(k, b, root.ok_or_else(invalid)?, &mut out, options)
            };
            match result {
                Ok(()) => status = 1000,
                Err(SolidError::Upstream { code, .. }) => status = code,
                Err(error) => return Err(error),
            }
        }
        macro_rules! ids {
            ($field:ident,$kind:ident) => {
                out.$field
                    .as_ref()
                    .map(|v| {
                        v.iter()
                            .copied()
                            .map(|id| object_ref(b, EntityRef::$kind(id)).map(|r| r.key.0))
                            .collect::<Result<Vec<_>>>()
                    })
                    .transpose()?
            };
        }
        let output = json!([
            ids!(regions, Region),
            ids!(shells, Shell),
            ids!(faces, Face),
            ids!(loops, Loop),
            ids!(edges, Edge),
            ids!(vertices, Vertex)
        ]);
        let mut marked = std::collections::BTreeMap::new();
        for e in pool {
            let key = object_ref(b, e)?.key;
            let record = b
                .notifications()
                .topology
                .get(&key)
                .copied()
                .unwrap_or_default();
            let display = if matches!(e, EntityRef::Brep(_)) {
                (solidcore::geom::assert_array::labels::BREP_TYPE as u64) << 32
            } else {
                key.0
            };
            marked.insert(
                display,
                types.map(|mark| record.is_marked(&k.context, mark)),
            );
        }
        Ok(Observed {
            status,
            output,
            advances: json!(types
                .into_iter()
                .zip(before)
                .map(|(m, before)| k.context.current_mark(m) - before)
                .collect::<Vec<_>>()),
            marks: json!(marked
                .into_iter()
                .map(|(key, marks)| json!([key, marks]))
                .collect::<Vec<_>>()),
        })
    };
    if *lock != 0 && target != MarkType::NoMark {
        with_mark_lock(k, target.code(), |k, _| run(k))?
    } else {
        run(k)
    }
}
