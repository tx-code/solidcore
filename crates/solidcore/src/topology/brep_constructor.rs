// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Owning top-down construction of runtime regions, shells, faces and use pairs.
//!
//! Upstream: `source/SMLib/inc/SmBrepData.h` (`SmBrepConstructor`, `SmVertexNode`),
//! `source/SMLib/src/SmBrepData.cpp` constructor methods, maintained fork `d8ed45a`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`BrepConstructor`] | `SmBrepConstructor` |
//! | [`VertexNode`] | `SmVertexNode` |
//! | [`BrepConstructor::new`] / [`BrepConstructor::finish`] | `StartBrep` / `EndBrep` |
//! | [`BrepConstructor::start_region`] / [`BrepConstructor::end_region`] | `StartRegion` / `EndRegion` |
//! | [`BrepConstructor::swap_regions`] | `SwapRegions` |
//! | [`BrepConstructor::start_shell`] / [`BrepConstructor::end_shell`] | `StartShell` / `EndShell` |
//! | [`BrepConstructor::start_face`] / [`BrepConstructor::end_face`] | `StartFace` / `EndFace` |
//! | [`BrepConstructor::start_loop`] / [`BrepConstructor::end_loop`] | `StartLoop` / `EndLoop` |
//! | [`BrepConstructor::start_vertex_of_loop`] / [`BrepConstructor::single_vertex_loop`] | `StartVertexOfLoop` / `StartAndEndSingleVertexLoop` |
//! | [`BrepConstructor::start_vertex`] / [`BrepConstructor::end_vertex`] | `StartVertex` / `EndVertex` |
//!
//! Deviations: an unpublished owned candidate is released on drop; finishing
//! consumes the builder, preventing stale constructor pointers after reuse.
//! Geometry arguments transfer Rust values instead of aliasing borrowed pointers.
//! Arena links may be incomplete between calls, just as the C++ constructor;
//! finish is not structural/geometric qualification. Unassigned convenience
//! endpoints use an invalid sentinel only until `EndLoop` resolves vertex uses.
//! Optional stitching runs the actual pointer validator before delegating; missing
//! geometry/notification callees remain explicit. See `docs/BREP_CONSTRUCTION.md`
//! for the separate full-validation and native-acceptance requirements.

mod edges;
mod geometry;
#[cfg(test)]
mod tests;

use super::edgeuse::p1 as eu;
use super::shell::{ShellMembers, ShellType};
use super::{
    Brep, EdgeId, EdgeuseId, Face, FaceId, Faceuse, FaceuseId, Loop, LoopId, Loopuse, LoopuseId,
    LoopuseStart, Region, RegionId, Shell, ShellId, Vertex, VertexId, Vertexuse, VertexuseId,
    VertexuseOwner,
};
use crate::error::{Result, SolidError};
use crate::kernel::Kernel;
use crate::math::{Point2, Point3};
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;

const OP: &str = "topology::brep_constructor";
fn invalid(reason: &'static str) -> SolidError {
    SolidError::invalid_input(OP, reason)
}

/// One original boundary spelling; UV points retain their original 2D embedding.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VertexNode {
    /// Explicit curve parameter, without a point drop or hidden snapping.
    Parameter(f64),
    /// A 3D point to drop under the original constructor policy.
    Point(Point3),
    /// A UV point embedded at z=0 for the curve's point-drop policy.
    Uv(Point2),
}

/// Stateful kernel constructor owning its unpublished candidate.
///
/// Methods establish reciprocal topology and geometry ownership, not acceptance
/// evidence. A caller must validate completed operations and their quality.
#[derive(Debug)]
pub struct BrepConstructor {
    brep: Brep,
    region: Option<RegionId>,
    outer_region: Option<RegionId>,
    outer_shell: Option<ShellId>,
    inner_shell: Option<ShellId>,
    face: Option<FaceId>,
    loop_id: Option<LoopId>,
    edge: Option<EdgeId>,
    edgeuse: Option<EdgeuseId>,
    vertex: Option<VertexId>,
    manifold: bool,
    loop_surface_orientation: OrientType,
}
impl BrepConstructor {
    /// Start a fresh `BRep` with the kernel's infinite void-region infrastructure.
    /// `SM_USE_OLDTOL` construction uses the existing `BRep` default; callers may
    /// explicitly set the candidate tolerance before creating contained objects.
    pub fn new() -> Result<Self> {
        Ok(Self {
            brep: Brep::new_with_infinite_region()?,
            region: None,
            outer_region: None,
            outer_shell: None,
            inner_shell: None,
            face: None,
            loop_id: None,
            edge: None,
            edgeuse: None,
            vertex: None,
            manifold: true,
            loop_surface_orientation: OrientType::Unknown,
        })
    }
    /// Inspect the unpublished candidate; its graph may be intermediate.
    pub fn brep(&self) -> &Brep {
        &self.brep
    }
    /// Kernel friend access to the unpublished candidate, for explicit tolerance,
    /// geometry and attribute settings. This does not publish a model.
    pub fn brep_mut(&mut self) -> &mut Brep {
        &mut self.brep
    }
    /// Current edge-use incidence, including its own surface side.
    pub const fn current_edgeuse(&self) -> Option<EdgeuseId> {
        self.edgeuse
    }
    /// Whether construction has needed a non-lamina radial-sector insertion.
    pub const fn is_manifold_construction(&self) -> bool {
        self.manifold
    }
    /// End construction only after every region/shell/face/loop is closed.
    /// Error drops the candidate; success is still subject to kernel validation.
    pub fn finish(self) -> Result<Brep> {
        if self.region.is_some()
            || self.outer_shell.is_some()
            || self.inner_shell.is_some()
            || self.face.is_some()
            || self.loop_id.is_some()
        {
            return Err(invalid("construction scopes remain open"));
        }
        Ok(self.brep)
    }
    /// Start a bounded region; the legacy solid hint does not set material/void
    /// semantics. Set the region flag explicitly, as in the original importer.
    pub fn start_region(&mut self, _solid_hint: bool, outer: Option<RegionId>) -> Result<RegionId> {
        if self.region.is_some() {
            return Err(invalid("region already active"));
        }
        if let Some(outer) = outer {
            self.brep.region(outer)?;
        }
        let region = self.brep.add_region_record(Region {
            in_brep: true,
            ..Region::default()
        })?;
        self.region = Some(region);
        self.outer_region = outer;
        Ok(region)
    }
    /// Close a region after its shell scopes have ended.
    pub fn end_region(&mut self) -> Result<()> {
        if self.region.is_none() || self.outer_shell.is_some() || self.inner_shell.is_some() {
            return Err(invalid("region/shell state mismatch"));
        }
        self.region = None;
        self.outer_region = None;
        Ok(())
    }
    /// Exchange the active inside/outside region identities (`SwapRegions`).
    /// This changes subsequent shell routing; it does not infer material roles
    /// or reverse existing face uses.
    pub fn swap_regions(&mut self) {
        std::mem::swap(&mut self.outer_region, &mut self.region);
    }
    /// Start paired boundary shells for a region, or one surface shell in the
    /// infinite region. Returned identities preserve outside/inside roles.
    pub fn start_shell(&mut self) -> Result<(ShellId, Option<ShellId>)> {
        if self.outer_shell.is_some() || self.inner_shell.is_some() {
            return Err(invalid("shell already active"));
        }
        let outside = self
            .outer_region
            .or(self.brep.infinite_region())
            .ok_or_else(|| invalid("missing infinite region"))?;
        self.outer_region = Some(outside);
        let outer = self.allocate_shell(outside)?;
        self.outer_shell = Some(outer);
        let inner = match self.region {
            Some(region) => Some(self.allocate_shell(region)?),
            None => None,
        };
        self.inner_shell = inner;
        Ok((outer, inner))
    }
    fn allocate_shell(&mut self, region: RegionId) -> Result<ShellId> {
        self.brep.region(region)?;
        let shell = self.brep.add_shell_record(Shell {
            region: Some(region),
            shell_type: ShellType::Faceuse,
            members: ShellMembers::Faceuses(Vec::new()),
            ..Shell::default()
        })?;
        self.brep.region_mut(region)?.shells.push(shell);
        Ok(shell)
    }
    /// Close a shell. Optional stitching delegates to the actual kernel operation;
    /// it never silently succeeds when a required callee is unported.
    pub fn end_shell(&mut self, k: &mut Kernel, stitch: bool) -> Result<()> {
        let shell = self.outer_shell.ok_or_else(|| invalid("no active shell"))?;
        if self.face.is_some() {
            return Err(invalid("face still active"));
        }
        if stitch {
            self.stitch_shell(k, shell)?;
        }
        self.outer_shell = None;
        self.inner_shell = None;
        Ok(())
    }
    /// Start a face and exactly two mated face uses. SAME remains the primary
    /// surface side; `outside` selects which shell receives that side.
    pub fn start_face(&mut self, outside: OrientType) -> Result<FaceId> {
        let outer = self
            .outer_shell
            .ok_or_else(|| invalid("face requires active shell"))?;
        let first_shell = if outside == OrientType::Same {
            outer
        } else {
            self.inner_shell.unwrap_or(outer)
        };
        let second_shell = if outside == OrientType::Same {
            self.inner_shell.unwrap_or(outer)
        } else {
            outer
        };
        let mut record = Face::new(first_shell);
        record.tolerance = self.brep.tolerance();
        let face = self.brep.add_face_record(record)?;
        let up = self.brep.add_faceuse_record(Faceuse {
            face: Some(face),
            orientation: OrientType::Same,
            ..Faceuse::default()
        })?;
        let down = self.brep.add_faceuse_record(Faceuse {
            face: Some(face),
            orientation: OrientType::Opposite,
            mate: Some(up),
            ..Faceuse::default()
        })?;
        self.brep.faceuse_mut(up)?.mate = Some(down);
        self.brep.face_mut(face)?.faceuse = Some(up);
        self.attach_faceuse(first_shell, up)?;
        self.attach_faceuse(second_shell, down)?;
        self.face = Some(face);
        Ok(face)
    }
    fn attach_faceuse(&mut self, shell: ShellId, fu: FaceuseId) -> Result<()> {
        let face = self
            .brep
            .faceuse(fu)?
            .face
            .ok_or_else(|| invalid("face use without face"))?;
        self.brep.faceuse_mut(fu)?.shell = Some(shell);
        let record = self.brep.shell_mut(shell)?;
        let ShellMembers::Faceuses(list) = &mut record.members else {
            return Err(invalid("non-surface shell"));
        };
        list.push(fu);
        if !record.faces.contains(&face) {
            record.faces.push(face);
        }
        Ok(())
    }
    /// Close the current face once no loop is active.
    pub fn end_face(&mut self) -> Result<()> {
        if self.face.is_none() || self.loop_id.is_some() {
            return Err(invalid("face/loop state mismatch"));
        }
        self.face = None;
        Ok(())
    }
    /// Start a logical loop and both surface-side loop uses. The first loop is
    /// outer by constructor convention; input walking orientation is independent.
    pub fn start_loop(&mut self, orientation: OrientType) -> Result<LoopId> {
        let face = self
            .face
            .ok_or_else(|| invalid("loop requires active face"))?;
        let up = self
            .brep
            .face(face)?
            .faceuse
            .ok_or_else(|| invalid("missing primary face use"))?;
        let down = self
            .brep
            .faceuse(up)?
            .mate
            .ok_or_else(|| invalid("missing face mate"))?;
        let loop_id = self.brep.add_loop_record(Loop {
            face,
            edges: Vec::new(),
            loopuse: None,
        })?;
        let outer = self.brep.faceuse(up)?.loopuses.is_empty();
        let ot = if outer {
            OrientType::Same
        } else {
            OrientType::Opposite
        };
        let lu = self.brep.add_loopuse_record(Loopuse {
            faceuse: Some(up),
            loop_: Some(loop_id),
            orientation: ot,
            ..Loopuse::default()
        })?;
        let mate = self.brep.add_loopuse_record(Loopuse {
            faceuse: Some(down),
            loop_: Some(loop_id),
            orientation: ot,
            mate: Some(lu),
            ..Loopuse::default()
        })?;
        self.brep.loopuse_mut(lu)?.mate = Some(mate);
        self.brep.loop_mut(loop_id)?.loopuse = Some(lu);
        self.brep.faceuse_mut(up)?.loopuses.push(lu);
        self.brep.faceuse_mut(down)?.loopuses.push(mate);
        self.brep.face_mut(face)?.loops.push(loop_id);
        self.edge = None;
        self.loop_id = Some(loop_id);
        self.loop_surface_orientation = orientation;
        Ok(loop_id)
    }
    /// Create a vertex without adding an incidence; it may be referenced by later
    /// edge construction. Coordinates do not imply topological welding.
    pub fn start_vertex_of_loop(&mut self, point: Point3) -> Result<VertexId> {
        self.brep.add_vertex_record(Vertex {
            point,
            tolerance: self.brep.tolerance(),
            ..Vertex::default()
        })
    }
    /// Create and mark a caller-tracked vertex. Existing vertex identities remain
    /// valid throughout construction; `EndLoop` never glues by coordinate.
    pub fn start_vertex(&mut self, point: Point3) -> Result<VertexId> {
        let vertex = self.start_vertex_of_loop(point)?;
        self.vertex = Some(vertex);
        Ok(vertex)
    }
    /// End the standalone vertex scope; as upstream, it is an idempotent marker reset.
    pub fn end_vertex(&mut self) {
        self.vertex = None;
    }
    /// Create an exact one-vertex loop with both mated loop-use incidences.
    pub fn single_vertex_loop(&mut self, point: Point3) -> Result<LoopId> {
        let loop_id = self.start_loop(OrientType::Same)?;
        let lu = self
            .brep
            .loop_(loop_id)?
            .loopuse
            .ok_or_else(|| invalid("missing primary loop"))?;
        let mate = self
            .brep
            .loopuse(lu)?
            .mate
            .ok_or_else(|| invalid("missing loop mate"))?;
        let v = self.start_vertex_of_loop(point)?;
        let a = self.brep.add_vertexuse_record(Vertexuse {
            owner: VertexuseOwner::Loopuse(Some(lu)),
            ..Vertexuse::default()
        })?;
        let b = self.brep.add_vertexuse_record(Vertexuse {
            owner: VertexuseOwner::Loopuse(Some(mate)),
            ..Vertexuse::default()
        })?;
        self.attach_vertexuse(v, a)?;
        self.attach_vertexuse(v, b)?;
        self.brep.loopuse_mut(lu)?.start = LoopuseStart::Vertexuse(Some(a));
        self.brep.loopuse_mut(mate)?.start = LoopuseStart::Vertexuse(Some(b));
        self.loop_id = None;
        Ok(loop_id)
    }
    fn attach_vertexuse(&mut self, v: VertexId, vu: VertexuseId) -> Result<()> {
        self.brep.vertex(v)?;
        self.brep.vertexuse(vu)?;
        if let Some(old) = self.brep.vertexuse(vu)?.vertex {
            if old != v {
                self.brep.vertex_mut(old)?.vertexuses.retain(|id| *id != vu);
            }
        }
        self.brep.vertexuse_mut(vu)?.vertex = Some(v);
        if !self.brep.vertex(v)?.vertexuses.contains(&vu) {
            self.brep.vertex_mut(v)?.vertexuses.push(vu);
        }
        Ok(())
    }
    /// Check the end-of-edge marker. It intentionally keeps the previous edge use
    /// active for insertion of the next edge, matching `EndEdge`.
    pub fn end_edge(&self) -> Result<()> {
        self.edge.ok_or_else(|| invalid("no active edge"))?;
        Ok(())
    }
    fn required_use(id: Option<EdgeuseId>) -> Result<EdgeuseId> {
        id.ok_or_else(|| invalid("missing required edge use"))
    }
    fn required_vertexuse(id: Option<VertexuseId>) -> Result<VertexuseId> {
        id.ok_or_else(|| invalid("missing required vertex use"))
    }
    fn primary_loop(&self) -> Result<LoopuseId> {
        let id = self.loop_id.ok_or_else(|| invalid("no active loop"))?;
        self.brep
            .loop_(id)?
            .loopuse
            .ok_or_else(|| invalid("missing loop use"))
    }
    fn eu_vertex(&self, id: EdgeuseId) -> Result<Option<VertexId>> {
        let vu = Self::required_vertexuse(self.brep.edgeuse(id)?.vertexuse)?;
        Ok(self.brep.vertexuse(vu)?.vertex)
    }
    fn stitch_shell(&mut self, k: &mut Kernel, shell: ShellId) -> Result<()> {
        use crate::heal::stitch::p1::{DefaultStitchCallback, Stitch};
        use crate::heal::stitch::p2::MinGapRequest;
        // EndShell(TRUE) first invokes the root validator even though C++ ignores
        // ordinary failure status. Use the actual pointer validator, including
        // geometry/cache side effects; unported callees still stop construction.
        crate::error::ignore_status(self.validate_before_stitch(k))?;
        let members = self.brep.shell(shell)?.get_faceuses()?;
        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        for fu in members {
            let face = self
                .brep
                .faceuse(fu)?
                .face
                .ok_or_else(|| invalid("shell face use missing face"))?;
            for loop_id in self.brep.face(face)?.loops.clone() {
                let lu = self
                    .brep
                    .loop_(loop_id)?
                    .loopuse
                    .ok_or_else(|| invalid("face loop missing loop use"))?;
                // SmFace::GetEdges/GetVertices append each LU's unique list;
                // repetitions between different loops/faces are retained.
                // Use the owning LU queries rather than deduplicating a face.
                for edge in crate::topology::loopuse::edges(&self.brep, lu, None)? {
                    edges.push(edge.ok_or_else(|| invalid("loop use without edge"))?);
                }
                for vertex in crate::topology::loopuse::vertices(&self.brep, lu, None)? {
                    vertices.push(vertex.ok_or_else(|| invalid("loop use without vertex"))?);
                }
            }
        }
        let mut callback = DefaultStitchCallback;
        let mut stitch = Stitch::new(&mut callback, self.brep.tolerance(), false, true);
        stitch.do_stitching(
            k,
            &mut self.brep,
            Some(&vertices),
            Some(&edges),
            MinGapRequest::default(),
        )?;
        Ok(())
    }
    fn validate_before_stitch(&mut self, k: &mut Kernel) -> Result<()> {
        super::brep_validation::validate_pointers(
            k,
            &mut self.brep,
            super::brep_validation::ValidationOptions::default(),
            None,
        )
    }
}
