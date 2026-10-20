// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Exact source edge incidence and identity-connected shell proof.
//! Upstream: `StepPlanarGeometry.cpp::Shell` at `d8ed45a`.
//! Deviations: safe maps/slices and iterative source walk; closure never promotes
//! a surface to material semantics or establishes shell nonintersection/volume.
use super::{metric, Checked, Failure, Validator, R};
use std::collections::{BTreeMap, BTreeSet};
impl Validator<'_, '_> {
    pub(super) fn shell(&mut self) -> Checked<()> {
        let shell = self.role("shell")?;
        let mut uses: BTreeMap<usize, Vec<(usize, i32)>> = BTreeMap::new();
        for face in &self.out.evidence.faces {
            for loop_e in &face.loops {
                if loop_e.edges.len() != loop_e.edge_orientations.len() {
                    return Err(Failure::Infrastructure);
                }
                for (edge, orientation) in loop_e.edges.iter().zip(&loop_e.edge_orientations) {
                    uses.entry(*edge).or_default().push((
                        face.face,
                        (if *orientation { 1 } else { -1 })
                            * (if loop_e.orientation { 1 } else { -1 }),
                    ));
                }
            }
        }
        let mut boundary = 0;
        let mut manifold = 0;
        let mut neighbours: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for (edge, list) in uses {
            self.tick(shell)?;
            match list.as_slice() {
                [_] => boundary += 1,
                [(a, da), (b, db)] if a != b => {
                    manifold += 1;
                    if da == db {
                        return Err(self.reject(
                            "FaceBoundaryOrShellOrientationMismatch",
                            "D4",
                            shell,
                            "cfs_faces",
                            "Shared source edge occurrences have the same effective direction",
                        ));
                    }
                    neighbours.entry(*a).or_default().insert(*b);
                    neighbours.entry(*b).or_default().insert(*a);
                }
                _ => {
                    return Err(self.reject(
                        "NonManifoldSourceEdge",
                        "D4",
                        edge,
                        "edge_start",
                        "A source edge has unsupported repeated or non-manifold face use",
                    ))
                }
            }
        }
        self.out.evidence.metrics.push(metric(
            "shell_incidence",
            shell,
            self.refs(shell, "cfs_faces")?,
            [
                ("boundary_edges", R::new(boundary, 1)?),
                ("paired_edges", R::new(manifold, 1)?),
            ],
        ));
        if self.is(shell, "CLOSED_SHELL") && boundary > 0 {
            return Err(self.reject(
                "ClaimedClosedShellHasBoundary",
                "D4",
                shell,
                "cfs_faces",
                "CLOSED_SHELL claim has free source edges; no filling, stitching or type promotion",
            ));
        }
        let faces = self.refs(shell, "cfs_faces")?;
        let mut reached = BTreeSet::new();
        let mut pending = vec![*faces.first().ok_or(Failure::Infrastructure)?];
        while let Some(face) = pending.pop() {
            if !reached.insert(face) {
                continue;
            }
            self.tick(face)?;
            if let Some(adjacency) = neighbours.get(&face) {
                pending.extend(adjacency.iter().copied());
            }
        }
        if reached.len() != faces.len() {
            return Err(self.reject(
                "DisconnectedSourceShell",
                "D4",
                shell,
                "cfs_faces",
                "CONNECTED_FACE_SET source claim is disconnected by shared edge identity",
            ));
        }
        Ok(())
    }
}
