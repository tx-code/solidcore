// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Constructor edge-use insertion and identity-based loop closure.
//! Upstream: `SmBrepData.cpp::StartEdge/EndLoop` at `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`BrepConstructor::start_edge`] / [`BrepConstructor::end_edge`] | `StartEdge` / `EndEdge` |
//! | [`BrepConstructor::end_loop`] | `EndLoop` |
//!
//! Deviations: checked arena links and scoped temporary edge cleanup. No gluing
//! by coordinate; unresolved vertices are evaluated from the defining curve.
use super::{
    eu, invalid, BrepConstructor, EdgeId, EdgeuseId, Kernel, LoopuseStart, OrientType, Result,
    VertexId, Vertexuse, VertexuseOwner,
};
use crate::error::ignored;
use crate::topology::brep_cache::{arena_identity, CacheObject};
use crate::topology::{Edge, Edgeuse, EdgeuseOwner, LoopEdge};

impl BrepConstructor {
    /// Start one logical edge use and its mate. Existing edges share defining
    /// geometry/vertices; newly supplied endpoints are swapped exactly when the
    /// new primary-loop use runs opposite the underlying curve.
    pub fn start_edge(
        &mut self,
        k: &Kernel,
        orientation: OrientType,
        existing: Option<EdgeId>,
        start: Option<VertexId>,
        end: Option<VertexId>,
    ) -> Result<EdgeId> {
        let lu = self.primary_loop()?;
        let other = self
            .brep
            .loopuse(lu)?
            .mate
            .ok_or_else(|| invalid("missing loop mate"))?;
        // Existing-edge construction never reads the optional endpoint inputs:
        // its primary/mate uses alone supply endpoint identity. Do not resolve
        // irrelevant ids and introduce a rejection absent from StartEdge.
        if existing.is_none() {
            for vertex in [start, end].into_iter().flatten() {
                self.brep.vertex(vertex)?;
            }
        }
        let edge = if let Some(edge) = existing {
            self.brep.edge(edge)?;
            edge
        } else {
            let unresolved = VertexId::from_index(u32::MAX);
            let mut edge = Edge::new([unresolved; 2]);
            edge.tolerance = self.brep.tolerance();
            self.brep.add_edge_record(edge)?
        };
        let a = self.brep.add_edgeuse_record(Edgeuse {
            owner: EdgeuseOwner::Loopuse(Some(lu)),
            ..Edgeuse::default()
        })?;
        let b = self.brep.add_edgeuse_record(Edgeuse {
            owner: EdgeuseOwner::Loopuse(Some(other)),
            ..Edgeuse::default()
        })?;
        let va = self.brep.add_vertexuse_record(Vertexuse {
            owner: VertexuseOwner::Edgeuse(Some(a)),
            ..Vertexuse::default()
        })?;
        let vb = self.brep.add_vertexuse_record(Vertexuse {
            owner: VertexuseOwner::Edgeuse(Some(b)),
            ..Vertexuse::default()
        })?;
        self.brep.edgeuse_mut(a)?.vertexuse = Some(va);
        self.brep.edgeuse_mut(b)?.vertexuse = Some(vb);
        let same = orientation == self.loop_surface_orientation;
        self.brep.edgeuse_mut(a)?.orientation = if same {
            OrientType::Same
        } else {
            OrientType::Opposite
        };
        self.brep.edgeuse_mut(b)?.orientation = if same {
            OrientType::Opposite
        } else {
            OrientType::Same
        };
        let (start, end) = if same { (start, end) } else { (end, start) };
        // Native loop heads move to the latest pair; its circular links preserve
        // the separate input walking orientation and surface-side ordering.
        self.brep.loopuse_mut(lu)?.start = LoopuseStart::Edgeuse(Some(a));
        self.brep.loopuse_mut(other)?.start = LoopuseStart::Edgeuse(Some(b));
        if self.edge.is_none() {
            self.brep.edgeuse_mut(a)?.ccw = Some(a);
            self.brep.edgeuse_mut(a)?.cw = Some(a);
            self.brep.edgeuse_mut(b)?.ccw = Some(b);
            self.brep.edgeuse_mut(b)?.cw = Some(b);
        } else {
            let previous = Self::required_use(self.edgeuse)?;
            let mate = Self::required_use(eu::mate(&self.brep, previous)?)?;
            if self.loop_surface_orientation == OrientType::Same {
                self.splice_ccw(previous, a)?;
                self.splice_cw(mate, b)?;
            } else {
                self.splice_cw(previous, a)?;
                self.splice_ccw(mate, b)?;
            }
        }
        if existing.is_none() {
            if let Some(start) = start {
                self.attach_vertexuse(start, va)?;
            }
            if let Some(end) = end {
                self.attach_vertexuse(end, vb)?;
            }
            self.brep.edge_mut(edge)?.edgeuses = if same { vec![a, b] } else { vec![b, a] };
            self.brep.edgeuse_mut(a)?.edge = Some(edge);
            self.brep.edgeuse_mut(b)?.edge = Some(edge);
        } else {
            let primary = *self
                .brep
                .edge(edge)?
                .edgeuses
                .first()
                .ok_or_else(|| invalid("existing edge without primary use"))?;
            let mate = Self::required_use(eu::mate(&self.brep, primary)?)?;
            let (v1, v2) =
                if self.brep.edgeuse(primary)?.orientation == self.brep.edgeuse(a)?.orientation {
                    (va, vb)
                } else {
                    (vb, va)
                };
            if let Some(vertex) = self.eu_vertex(primary)? {
                self.attach_vertexuse(vertex, v1)?;
            }
            if let Some(vertex) = self.eu_vertex(mate)? {
                self.attach_vertexuse(vertex, v2)?;
            }
            let sector = if crate::topology::edge::is_lamina(&self.brep, edge)? {
                mate
            } else {
                self.manifold = false;
                self.radial_sector(k, edge, a, b)?.unwrap_or(mate)
            };
            // AddOrientedEUPair is the owning reciprocal operation; do not
            // rebuild mate/radial order from an imported face-number heuristic.
            crate::topology::edge::add_oriented_eu_pair(&mut self.brep, edge, a, b, Some(sector))?;
        }
        self.edge = Some(edge);
        self.edgeuse = Some(a);
        self.refresh_endpoints(edge)?;
        Ok(edge)
    }
    fn splice_ccw(&mut self, previous: EdgeuseId, new: EdgeuseId) -> Result<()> {
        let next = Self::required_use(self.brep.edgeuse(previous)?.ccw)?;
        self.brep.edgeuse_mut(new)?.ccw = Some(next);
        self.brep.edgeuse_mut(next)?.cw = Some(new);
        self.brep.edgeuse_mut(previous)?.ccw = Some(new);
        self.brep.edgeuse_mut(new)?.cw = Some(previous);
        Ok(())
    }
    fn splice_cw(&mut self, previous: EdgeuseId, new: EdgeuseId) -> Result<()> {
        let next = Self::required_use(self.brep.edgeuse(previous)?.cw)?;
        self.brep.edgeuse_mut(new)?.cw = Some(next);
        self.brep.edgeuse_mut(next)?.ccw = Some(new);
        self.brep.edgeuse_mut(previous)?.cw = Some(new);
        self.brep.edgeuse_mut(new)?.ccw = Some(previous);
        Ok(())
    }
    fn refresh_endpoints(&mut self, edge: EdgeId) -> Result<()> {
        let primary = *self
            .brep
            .edge(edge)?
            .edgeuses
            .first()
            .ok_or_else(|| invalid("edge without primary use"))?;
        let mate = Self::required_use(eu::mate(&self.brep, primary)?)?;
        if let (Some(a), Some(b)) = (self.eu_vertex(primary)?, self.eu_vertex(mate)?) {
            self.brep.edge_mut(edge)?.vertices = [a, b];
        }
        Ok(())
    }
    fn radial_sector(
        &mut self,
        k: &Kernel,
        edge: EdgeId,
        a: EdgeuseId,
        b: EdgeuseId,
    ) -> Result<Option<EdgeuseId>> {
        let vertices = self.brep.edge(edge)?.vertices;
        let interval = self.brep.edge(edge)?.interval;
        let mut curve = self
            .brep
            .edge(edge)?
            .curve
            .as_ref()
            .ok_or_else(|| invalid("nonmanifold edge without geometry"))?
            .clone();
        let mut temporary = Edge::new(vertices);
        temporary.interval = interval;
        temporary.tolerance = self.brep.tolerance();
        let same = self.brep.edgeuse(a)?.orientation == OrientType::Same;
        temporary.edgeuses = if same { vec![a, b] } else { vec![b, a] };
        let temporary_id = self.brep.add_edge_record(temporary)?;
        curve.set_owner(
            Some(arena_identity(CacheObject::Edge(temporary_id))),
            Some(arena_identity(CacheObject::Brep)),
        );
        self.brep.edge_mut(temporary_id)?.curve = Some(curve);
        self.brep.edgeuse_mut(a)?.edge = Some(temporary_id);
        self.brep.edgeuse_mut(b)?.edge = Some(temporary_id);
        let result = crate::topology::edge::find_radial_sector(
            k,
            &mut self.brep,
            edge,
            a,
            None,
            OrientType::Same,
            None,
        );
        // The temporary is always the final arena/list item. Undo it on BOTH
        // result paths before a callee error (including Unported) propagates.
        self.brep.edgeuse_mut(a)?.edge = None;
        self.brep.edgeuse_mut(b)?.edge = None;
        if self.brep.edge_list.last() != Some(&temporary_id) {
            return Err(invalid("temporary edge list changed"));
        }
        self.brep.edge_list.pop();
        self.brep.edges.items.pop();
        Ok(ignored(result)?.and_then(|answer| answer.radial_edgeuse))
    }
    /// Close a loop by vertex identity. Missing vertices are obtained from the
    /// defining 3D curve, preserving the original mate/radial preference order.
    /// Coincident coordinates are never an instruction to delete or weld vertices.
    /// `k` is the executing context for those lazy geometry queries; constructor
    /// state does not manufacture a replacement context or infer one from a curve.
    pub fn end_loop(&mut self, k: &Kernel) -> Result<()> {
        let lu = self.primary_loop()?;
        let vertexuses = crate::topology::loopuse::vertexuses(&self.brep, lu, None)?;
        for optional in vertexuses {
            let vu = Self::required_vertexuse(optional)?;
            let VertexuseOwner::Edgeuse(Some(eu_id)) = self.brep.vertexuse(vu)?.owner else {
                return Err(invalid("loop incidence is not an edge use"));
            };
            let edge = self
                .brep
                .edgeuse(eu_id)?
                .edge
                .ok_or_else(|| invalid("edge use without edge"))?;
            let mate = Self::required_use(eu::mate(&self.brep, eu_id)?)?;
            let mate_ccw = Self::required_use(self.brep.edgeuse(mate)?.ccw)?;
            let mate_vertexuse = Self::required_vertexuse(self.brep.edgeuse(mate_ccw)?.vertexuse)?;
            let radial = Self::required_use(eu::radial(&self.brep, eu_id)?)?;
            let radial_vertexuse = if radial == mate {
                None
            } else {
                let next = Self::required_use(self.brep.edgeuse(radial)?.ccw)?;
                Some(Self::required_vertexuse(
                    self.brep.edgeuse(next)?.vertexuse,
                )?)
            };
            let vertex = if let Some(vertex) = self.brep.vertexuse(vu)?.vertex {
                vertex
            } else {
                let mut found = self.brep.vertexuse(mate_vertexuse)?.vertex;
                if found.is_none() {
                    if let Some(vu) = radial_vertexuse {
                        found = self.brep.vertexuse(vu)?.vertex;
                    }
                }
                if found.is_none() {
                    if let Some(vertex) = self.eu_vertex(radial)? {
                        let record = self.brep.edge(edge)?;
                        let curve = record
                            .curve
                            .as_ref()
                            .ok_or_else(|| invalid("unresolved edge has no curve"))?;
                        if curve.is_closed(k, &record.interval, record.tolerance)? {
                            found = Some(vertex);
                        }
                    }
                }
                let vertex = if let Some(vertex) = found {
                    vertex
                } else {
                    let record = self.brep.edge(edge)?;
                    let curve = record
                        .curve
                        .as_ref()
                        .ok_or_else(|| invalid("unresolved vertex has no curve"))?;
                    let parameter = if self.brep.edgeuse(eu_id)?.orientation == OrientType::Opposite
                    {
                        record.interval.max()
                    } else {
                        record.interval.min()
                    };
                    let point = curve.evaluate_point(k, parameter)?;
                    self.start_vertex_of_loop(point)?
                };
                self.attach_vertexuse(vertex, vu)?;
                vertex
            };
            if self.brep.vertexuse(mate_vertexuse)?.vertex.is_none() {
                self.attach_vertexuse(vertex, mate_vertexuse)?;
            }
            let cw = Self::required_use(self.brep.edgeuse(eu_id)?.cw)?;
            if eu::mate(&self.brep, cw)? != Some(mate_ccw) {
                return Err(invalid("mate/loop reciprocity mismatch"));
            }
        }
        let loop_id = self
            .loop_id
            .ok_or_else(|| invalid("missing current loop"))?;
        let edgeuses = crate::topology::loopuse::loop_edgeuses(&self.brep, loop_id)?;
        let mut logical = Vec::new();
        for eu in edgeuses {
            let record = self.brep.edgeuse(eu)?;
            let edge = record.edge.ok_or_else(|| invalid("missing edge owner"))?;
            logical.push(LoopEdge {
                edge,
                forward: record.orientation == OrientType::Same,
            });
            self.refresh_endpoints(edge)?;
        }
        self.brep.loop_mut(loop_id)?.edges = logical;
        self.loop_id = None;
        self.edge = None;
        self.loop_surface_orientation = OrientType::Unknown;
        Ok(())
    }
}
