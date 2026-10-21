// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Complete source-edge partitions for equivalent computational chart cuts.
//!
//! Project contract: native STEP source identity, defining geometry and parameter
//! preservation (`STEP_IMPORT/README.md` at fork d8ed45a). This is an explicit Rust
//! extension of `MissingSeam`, not a port of a C++ split that does not exist there.
//! Every piece owns a copy of the COMPLETE original curve; only its edge interval
//! changes. All adjacent faces reuse the same pieces and added vertices.

use super::{need, surface, GeneralDerivedVertex, SourceEdge, SourceUse, SourceVertex};
use crate::{Error, Execution, Result};
use solidcore::curve::curve::Curve;
use solidcore::kernel::Kernel;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::topology::{
    brep_constructor::{BrepConstructor, VertexNode},
    EdgeId, VertexId,
};
use std::collections::BTreeMap;

struct Piece {
    curve: Option<Curve>,
    interval: [f64; 2],
    vertices: [VertexId; 2],
    native: Option<EdgeId>,
}
/// Prepared once before any defining curve is transferred. Private source edge
/// indices address these groups; target identities are assigned on first use.
pub(super) struct Pieces {
    groups: Vec<Vec<Piece>>,
    seam_vertices: BTreeMap<usize, [VertexId; 2]>,
}
impl Pieces {
    pub(super) fn prepare(
        builder: &mut BrepConstructor,
        execution: &mut Execution,
        definitions: &mut [SourceEdge],
        source_vertices: &[SourceVertex],
        vertices: &BTreeMap<usize, VertexId>,
        plans: &BTreeMap<usize, surface::ImplicitRingPlan>,
        provenance: &mut Vec<GeneralDerivedVertex>,
        tolerance: f64,
    ) -> Result<Self> {
        let mut cuts: BTreeMap<usize, Vec<(f64, solidcore::math::Vec3)>> = BTreeMap::new();
        for plan in plans.values() {
            need(
                plan.ring_edges[1] == plan.split.edge && plan.ring_edges[0] != plan.ring_edges[1],
                "GeneralRingSplitPlan",
                plan.split.edge,
            )?;
            cuts.entry(plan.split.edge)
                .or_default()
                .push((plan.split.native_parameter, plan.split.point));
        }
        let mut groups = Vec::new();
        let mut cut_vertices: BTreeMap<usize, Vec<(f64, VertexId)>> = BTreeMap::new();
        for (index, edge) in definitions.iter_mut().enumerate() {
            let definition = edge
                .built
                .as_mut()
                .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?;
            let curve = definition
                .native
                .take()
                .ok_or(Error::Invalid("GeneralCurveAlreadyTransferred"))?;
            let original_vertex = |index: usize| -> Result<VertexId> {
                let source = source_vertices
                    .get(index)
                    .ok_or(Error::Invalid("GeneralVertexIndex"))?;
                vertices
                    .get(&source.record)
                    .copied()
                    .ok_or(Error::Invalid("GeneralVertexCorrespondence"))
            };
            let mut boundaries = vec![(
                definition.interval.min(),
                original_vertex(edge.vertices[0])?,
            )];
            if let Some(edge_cuts) = cuts.get_mut(&index) {
                edge_cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
                for &(parameter, point) in edge_cuts.iter() {
                    need(
                        parameter.is_finite()
                            && parameter > definition.interval.min()
                            && parameter < definition.interval.max(),
                        "GeneralRingSplitParameter",
                        edge.record,
                    )?;
                    if boundaries
                        .last()
                        .is_some_and(|(previous, _)| *previous == parameter)
                    {
                        continue;
                    }
                    execution.step()?;
                    let id = builder.start_vertex(point)?;
                    builder.brep_mut().vertex_mut(id)?.tolerance = tolerance;
                    execution.step()?;
                    builder.end_vertex();
                    boundaries.push((parameter, id));
                    cut_vertices.entry(index).or_default().push((parameter, id));
                    provenance.push(GeneralDerivedVertex {
                        source_edge: edge.record,
                        native_vertex: id,
                        native_parameter: parameter,
                    });
                }
            }
            boundaries.push((
                definition.interval.max(),
                original_vertex(edge.vertices[1])?,
            ));
            let mut pieces = Vec::new();
            // Exact adjacent endpoints are reused, so the intervals form an
            // ordered complete partition without a fitted or shortened basis.
            let mut original = Some(curve);
            let count = boundaries
                .len()
                .checked_sub(1)
                .ok_or(Error::Invalid("GeneralPieceBoundary"))?;
            for (index, pair) in boundaries.windows(2).enumerate() {
                let [(low, a), (high, b)] = pair else {
                    return Err(Error::Invalid("GeneralPieceBoundary"));
                };
                need(high > low, "GeneralRingSplitParameter", edge.record)?;
                let curve = if index + 1 == count {
                    original
                        .take()
                        .ok_or(Error::Invalid("GeneralCurveAlreadyTransferred"))?
                } else {
                    // One extra complete definition is unavoidable per split
                    // because native geometry is value-owned. Charge BEFORE
                    // copying; an unsplit source moves without any added copy.
                    execution.step()?;
                    original
                        .as_ref()
                        .ok_or(Error::Invalid("GeneralDefiningCurveMissing"))?
                        .clone()
                };
                pieces.push(Piece {
                    curve: Some(curve),
                    interval: [*low, *high],
                    vertices: [*a, *b],
                    native: None,
                });
            }
            groups.push(pieces);
        }
        let mut seam_vertices = BTreeMap::new();
        for (face, plan) in plans {
            let source = source_vertices
                .get(plan.source_vertices[0])
                .ok_or(Error::Invalid("GeneralSeamVertex"))?;
            let first = *vertices
                .get(&source.record)
                .ok_or(Error::Invalid("GeneralSeamVertex"))?;
            let second = cut_vertices
                .get(&plan.split.edge)
                .and_then(|items| {
                    items
                        .iter()
                        .find(|(t, _)| *t == plan.split.native_parameter)
                })
                .map(|(_, id)| *id)
                .ok_or(Error::Invalid("GeneralSeamVertex"))?;
            seam_vertices.insert(*face, [first, second]);
        }
        Ok(Self {
            groups,
            seam_vertices,
        })
    }
    pub(super) fn seam_vertices(&self, face: usize) -> Option<[VertexId; 2]> {
        self.seam_vertices.get(&face).copied()
    }
    pub(super) fn correspondence(
        &self,
        definitions: &[SourceEdge],
    ) -> Result<BTreeMap<usize, Vec<EdgeId>>> {
        definitions
            .iter()
            .zip(&self.groups)
            .map(|(source, pieces)| {
                let ids = pieces
                    .iter()
                    .map(|p| p.native.ok_or(Error::Invalid("GeneralMissingEdgePiece")))
                    .collect::<Result<Vec<_>>>()?;
                if ids.is_empty() {
                    return Err(Error::Invalid("GeneralMissingEdgePiece"));
                }
                Ok((source.record, ids))
            })
            .collect()
    }
    pub(super) fn append(
        &mut self,
        builder: &mut BrepConstructor,
        kernel: &Kernel,
        execution: &mut Execution,
        source_use: SourceUse,
        start: Option<VertexId>,
        tolerance: f64,
    ) -> Result<()> {
        let group = self
            .groups
            .get_mut(source_use.edge)
            .ok_or(Error::Invalid("GeneralEdgeIndex"))?;
        let mut order: Vec<_> = (0..group.len()).collect();
        if !source_use.forward {
            order.reverse();
        }
        if let Some(vertex) = start {
            // A split closed ring may start at the chart vertex. Reversing a use
            // reverses both piece order and orientation; rotation changes no
            // cyclic source traversal or physical boundary.
            let offset = order
                .iter()
                .position(|i| {
                    group.get(*i).is_some_and(|p| {
                        p.vertices.get(usize::from(!source_use.forward)) == Some(&vertex)
                    })
                })
                .ok_or(Error::Invalid("GeneralRingStartVertex"))?;
            order.rotate_left(offset);
        }
        for index in order {
            let piece = group
                .get_mut(index)
                .ok_or(Error::Invalid("GeneralEdgePieceIndex"))?;
            execution.step()?;
            let id = builder.start_edge(
                kernel,
                if source_use.forward {
                    OrientType::Same
                } else {
                    OrientType::Opposite
                },
                piece.native,
                Some(piece.vertices[0]),
                Some(piece.vertices[1]),
            )?;
            need(
                piece.native.is_none_or(|old| old == id),
                "GeneralSharedEdgeIdentity",
                source_use.record,
            )?;
            if piece.native.is_none() {
                builder.brep_mut().edge_mut(id)?.tolerance = tolerance;
                execution.step()?;
                builder.set_edge_curve(
                    kernel,
                    piece
                        .curve
                        .take()
                        .ok_or(Error::Invalid("GeneralCurveAlreadyTransferred"))?,
                    false,
                )?;
                execution.step()?;
                builder.set_limits(
                    kernel,
                    VertexNode::Parameter(piece.interval[0]),
                    VertexNode::Parameter(piece.interval[1]),
                )?;
                piece.native = Some(id);
            }
            execution.step()?;
            builder.end_edge()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::general::geometry::BuiltCurve;
    use solidcore::curve::{
        curve::CurveKind,
        ellipse::{Ellipse, EllipseOptions},
    };
    use solidcore::geom::{extent1d::Extent1d, extent2d::Extent2d};
    use solidcore::math::Vec3;
    use solidcore::nurbs::b_spline_curve::BoundingBoxRequest;
    use solidcore::surface::{plane::Plane, surface::Surface};
    use solidcore::topology::{loopuse, Brep, LoopId};

    const FORWARD_FACE: usize = 20;
    const REVERSE_FACE: usize = 10;

    struct Control {
        builder: BrepConstructor,
        definitions: Vec<SourceEdge>,
        source_vertices: Vec<SourceVertex>,
        vertices: BTreeMap<usize, VertexId>,
        plans: BTreeMap<usize, surface::ImplicitRingPlan>,
        original: Curve,
        parameters: [f64; 2],
    }

    fn definition(kernel: &Kernel, i: usize, height: f64, phase: f64) -> SourceEdge {
        let curve = Curve::from(
            Ellipse::new_circle(
                kernel,
                Vec3::new(0., 0., height),
                Vec3::X,
                Vec3::Y,
                Extent1d::from_min_max(phase, phase + 360.),
                3.,
                &EllipseOptions::default(),
            )
            .unwrap(),
        );
        let interval = curve.natural_interval();
        let bounds = curve
            .calculate_bounding_box(
                kernel,
                &interval,
                BoundingBoxRequest {
                    normal: true,
                    ..BoundingBoxRequest::default()
                },
                true,
            )
            .unwrap()
            .normal
            .unwrap();
        let endpoints = [
            curve.evaluate_point(kernel, interval.min()).unwrap(),
            curve.evaluate_point(kernel, interval.max()).unwrap(),
        ];
        SourceEdge {
            record: 100 + i,
            geometry: 200 + i,
            vertices: [i, i],
            same_sense: true,
            built: Some(BuiltCurve {
                native: Some(curve),
                interval,
                endpoints,
                bounds,
                mappings: Vec::new(),
                maximum_consistency_mm: 0.,
            }),
        }
    }

    // Independently authored R=3 circles: shared ring z=4 anchored at +X,
    // lower ring z=0 at +Y, upper ring z=7 at -X. Two source faces request
    // quarter- and half-turn cuts on that SAME original middle-ring curve.
    // This tests the already-prepared internal contract, not STEP qualification.
    fn control(kernel: &Kernel) -> Control {
        let mut builder = BrepConstructor::new().unwrap();
        let points = [
            Vec3::new(3., 0., 4.),
            Vec3::new(0., 3., 0.),
            Vec3::new(-3., 0., 7.),
        ];
        let mut source_vertices = Vec::new();
        let mut vertices = BTreeMap::new();
        for (i, point) in points.into_iter().enumerate() {
            source_vertices.push(SourceVertex {
                record: 300 + i,
                point,
            });
            let id = builder.start_vertex(point).unwrap();
            builder.end_vertex();
            vertices.insert(300 + i, id);
        }
        let definitions: Vec<_> = [(4., 0.), (0., -270.), (7., -180.)]
            .into_iter()
            .enumerate()
            .map(|(i, (height, phase))| definition(kernel, i, height, phase))
            .collect();
        let original = definitions[0]
            .built
            .as_ref()
            .unwrap()
            .native
            .as_ref()
            .unwrap()
            .clone();
        let CurveKind::Ellipse(circle) = original.kind() else {
            panic!("control circle")
        };
        let parameters = [
            circle.convert_t_from_step_to_nurbs(kernel, 90.).unwrap(),
            circle.convert_t_from_step_to_nurbs(kernel, 180.).unwrap(),
        ];
        let mut plans = BTreeMap::new();
        for (face, first, parameter, angle) in [
            (FORWARD_FACE, 1, parameters[0], std::f64::consts::FRAC_PI_2),
            (REVERSE_FACE, 2, parameters[1], std::f64::consts::PI),
        ] {
            let point = original.evaluate_point(kernel, parameter).unwrap();
            plans.insert(
                face,
                surface::ImplicitRingPlan {
                    ring_edges: [first, 0],
                    source_vertices: [first, 0],
                    seam_endpoints: [points[first], point],
                    seam_angle_rad: angle,
                    split: surface::RingSplit {
                        edge: 0,
                        native_parameter: parameter,
                        point,
                    },
                },
            );
        }
        Control {
            builder,
            definitions,
            source_vertices,
            vertices,
            plans,
            original,
            parameters,
        }
    }

    fn prepare(
        control: &mut Control,
        execution: &mut Execution,
    ) -> (Pieces, Vec<GeneralDerivedVertex>) {
        let mut provenance = Vec::new();
        let pieces = Pieces::prepare(
            &mut control.builder,
            execution,
            &mut control.definitions,
            &control.source_vertices,
            &control.vertices,
            &control.plans,
            &mut provenance,
            1e-6,
        )
        .unwrap();
        (pieces, provenance)
    }

    #[test]
    fn two_face_cut_union_is_one_complete_ordered_partition_with_original_bases() {
        let kernel = Kernel::default();
        let mut c = control(&kernel);
        let mut execution = Execution { used: 0, max: 6 };
        let (pieces, provenance) = prepare(&mut c, &mut execution);
        assert_eq!(execution.used, 6); // two vertices at two actions + two extra copies
        let group = &pieces.groups[0];
        assert_eq!(group.len(), 3);
        let interval = c.original.natural_interval();
        let expected = [
            [interval.min(), c.parameters[0]],
            [c.parameters[0], c.parameters[1]],
            [c.parameters[1], interval.max()],
        ];
        let CurveKind::Ellipse(original) = c.original.kind() else {
            panic!("control circle")
        };
        for (piece, ends) in group.iter().zip(expected) {
            assert_eq!(piece.interval, ends);
            let curve = piece.curve.as_ref().unwrap();
            assert_eq!(curve.natural_interval(), interval);
            let CurveKind::Ellipse(circle) = curve.kind() else {
                panic!("circle was replaced")
            };
            assert_eq!(circle.step_interval(), original.step_interval());
            assert_eq!(
                circle.base().knots(None).unwrap(),
                original.base().knots(None).unwrap()
            );
            assert_eq!(
                circle.base().gw_nurb().unwrap().cpts(),
                original.base().gw_nurb().unwrap().cpts()
            );
            for parameter in ends {
                let actual = curve.evaluate_point(&kernel, parameter).unwrap();
                let reference = c.original.evaluate_point(&kernel, parameter).unwrap();
                assert!((actual - reference).length() < 1e-12);
            }
        }
        assert_eq!(group[0].vertices[0], c.vertices[&300]);
        assert_eq!(group[0].vertices[1], group[1].vertices[0]);
        assert_eq!(group[1].vertices[1], group[2].vertices[0]);
        assert_eq!(group[2].vertices[1], c.vertices[&300]);
        assert_eq!(provenance.len(), 2);
        assert_eq!(
            provenance
                .iter()
                .map(|v| v.native_parameter)
                .collect::<Vec<_>>(),
            c.parameters
        );
        assert!(provenance.iter().all(|v| v.source_edge == 100));
        assert_eq!(c.vertices.len(), 3);
        assert_eq!(c.builder.brep().vertex_list().len(), 5);
        for (vertex, truth) in provenance
            .iter()
            .zip([Vec3::new(0., 3., 4.), Vec3::new(-3., 0., 4.)])
        {
            assert!(
                (c.builder.brep().vertex(vertex.native_vertex).unwrap().point - truth).length()
                    < 1e-12
            );
        }
        assert_eq!(
            pieces.seam_vertices(FORWARD_FACE),
            Some([c.vertices[&301], provenance[0].native_vertex])
        );
        assert_eq!(
            pieces.seam_vertices(REVERSE_FACE),
            Some([c.vertices[&302], provenance[1].native_vertex])
        );
        assert!(pieces.groups[1..].iter().all(|group| group.len() == 1));
    }

    #[test]
    fn additional_copies_are_precharged_and_unsplit_definitions_need_no_added_budget() {
        let kernel = Kernel::default();
        for maximum in 4..=6 {
            let mut c = control(&kernel);
            let mut execution = Execution {
                used: 0,
                max: maximum,
            };
            let mut provenance = Vec::new();
            let result = Pieces::prepare(
                &mut c.builder,
                &mut execution,
                &mut c.definitions,
                &c.source_vertices,
                &c.vertices,
                &c.plans,
                &mut provenance,
                1e-6,
            );
            assert_eq!(execution.used, maximum);
            assert_eq!(provenance.len(), 2); // both failures occur at the extra-copy boundary
            if maximum < 6 {
                assert!(matches!(result, Err(Error::ResourceLimit)));
            } else {
                assert_eq!(result.unwrap().groups[0].len(), 3);
            }
        }
        let mut c = control(&kernel);
        c.plans.clear();
        let mut execution = Execution { used: 0, max: 0 };
        let (pieces, provenance) = prepare(&mut c, &mut execution);
        assert_eq!(execution.used, 0);
        assert!(provenance.is_empty());
        assert!(pieces.groups.iter().all(|group| group.len() == 1));
        assert_eq!(c.builder.brep().vertex_list().len(), 3);
    }

    fn trace(brep: &Brep, loop_id: LoopId, start: VertexId) -> Vec<(EdgeId, OrientType, VertexId)> {
        let loop_use = brep.loop_(loop_id).unwrap().loopuse.unwrap();
        let mut trace: Vec<_> = loopuse::edgeuses(brep, loop_use, None, None)
            .unwrap()
            .into_iter()
            .map(|id| {
                let use_record = brep.edgeuse(id).unwrap();
                let vertex = brep
                    .vertexuse(use_record.vertexuse.unwrap())
                    .unwrap()
                    .vertex
                    .unwrap();
                (use_record.edge.unwrap(), use_record.orientation, vertex)
            })
            .collect();
        let index = trace
            .iter()
            .position(|(_, _, vertex)| *vertex == start)
            .unwrap();
        trace.rotate_left(index);
        trace
    }

    #[test]
    fn differently_anchored_reverse_uses_share_all_three_native_pieces() {
        let kernel = Kernel::default();
        let mut c = control(&kernel);
        let mut execution = Execution { used: 0, max: 100 };
        let (mut pieces, provenance) = prepare(&mut c, &mut execution);
        c.builder.start_region(true, None).unwrap();
        c.builder.start_shell().unwrap();
        let domain = Extent2d::from_bounds(-4., -4., 4., 4.);
        let mut loops = Vec::new();
        for (face, forward) in [(FORWARD_FACE, true), (REVERSE_FACE, false)] {
            c.builder.start_face(OrientType::Same).unwrap();
            c.builder
                .set_face_surface(
                    Surface::from(Plane::from_normal(Vec3::new(0., 0., 4.), Vec3::Z)),
                    domain,
                )
                .unwrap();
            let loop_id = c.builder.start_loop(OrientType::Same).unwrap();
            let start = pieces.seam_vertices(face).unwrap()[1];
            let before = execution.used;
            pieces
                .append(
                    &mut c.builder,
                    &kernel,
                    &mut execution,
                    SourceUse {
                        edge: 0,
                        record: 400 + face,
                        forward,
                    },
                    Some(start),
                    1e-6,
                )
                .unwrap();
            assert_eq!(execution.used - before, if forward { 12 } else { 6 });
            c.builder.end_loop(&kernel).unwrap();
            c.builder.end_face().unwrap();
            loops.push(loop_id);
        }
        // This is a partial two-face constructor control, not a fabricated solid
        // or a native validity claim. Only actual incidences and this interface's
        // sequence/ownership contract are examined; no finish/qualification is faked.
        let ids: Vec<_> = pieces.groups[0]
            .iter()
            .map(|piece| piece.native.unwrap())
            .collect();
        assert_eq!(c.builder.brep().edge_list().len(), 3);
        assert_eq!(c.builder.brep().vertex_list().len(), 5);
        let anchor = c.vertices[&300];
        let a = provenance[0].native_vertex;
        let b = provenance[1].native_vertex;
        assert_eq!(
            trace(c.builder.brep(), loops[0], a),
            vec![
                (ids[1], OrientType::Same, a),
                (ids[2], OrientType::Same, b),
                (ids[0], OrientType::Same, anchor)
            ]
        );
        assert_eq!(
            trace(c.builder.brep(), loops[1], b),
            vec![
                (ids[1], OrientType::Opposite, b),
                (ids[0], OrientType::Opposite, a),
                (ids[2], OrientType::Opposite, anchor)
            ]
        );
        for (piece, id) in pieces.groups[0].iter().zip(ids) {
            let native = c.builder.brep().edge(id).unwrap();
            assert_eq!(native.edgeuses.len(), 4);
            assert_eq!(
                [native.interval.min(), native.interval.max()],
                piece.interval
            );
            assert_eq!(
                native.curve.as_ref().unwrap().natural_interval(),
                c.original.natural_interval()
            );
            assert!(piece.curve.is_none());
        }
    }
}
