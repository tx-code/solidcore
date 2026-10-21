// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Continuous affine geometry certificates, independent of native validity.
//! Upstream: `StepImportNative.cpp::LineCoefficients/PlaneCoefficients/LiftBound/Verify`.
use super::{bounds as b, Candidate, Error, Result};
use solidcore::curve::{
    curve::{Curve, CurveKind},
    line::Line,
};
use solidcore::surface::surface::SurfaceKind;
use solidcore::topology::edgeuse::p1 as eu;
use solidcore_step::planar::Rational;

/// Explicit application acceptance selection; no production numbers are inferred.
#[derive(Clone, Debug)]
pub struct AcceptanceQuality {
    /// Caller-visible reusable configuration identity.
    pub id: String,
    /// Continuous defining-geometry conversion budget, in mm.
    pub definition_mm: Rational,
    /// Continuous derived UV lift budget, in mm.
    pub derived_uv_mm: Rational,
    /// Native endpoint/vertex/receiver consistency budget, in mm.
    pub consistency_mm: Rational,
    /// Maximum native working tolerance, in mm.
    pub maximum_work_tolerance_mm: Rational,
}
/// Completed affine certificate; values are conservative continuous bounds in mm.
#[derive(Clone, Copy, Debug, Default)]
pub struct Certificate {
    /// Greatest defining-geometry difference upper bound.
    pub definition_mm: f64,
    /// Greatest UV-to-native-curve difference upper bound.
    pub derived_uv_mm: f64,
    /// Greatest native endpoint/vertex/receiver difference upper bound.
    pub consistency_mm: f64,
}
/// Owning read-only result of the qualified initial open-planar profile.
/// This does not establish general `BRep`, closed/material-volume or query coverage.
pub struct Imported {
    candidate: Candidate,
    quality: AcceptanceQuality,
    certificate: Certificate,
}
impl Imported {
    /// Actual immutable Rust topology/geometry.
    pub fn brep(&self) -> &solidcore::topology::Brep {
        self.candidate.brep()
    }
    /// Original immutable bytes/units/roles and source semantics.
    pub fn source(&self) -> &solidcore_step::semantics::Snapshot {
        self.candidate.source()
    }
    /// Explicit quality identity and numerical budgets.
    pub fn quality(&self) -> &AcceptanceQuality {
        &self.quality
    }
    /// Completed continuous affine bounds.
    pub const fn certificate(&self) -> Certificate {
        self.certificate
    }
    /// Source-to-native vertex identities belonging to this result.
    pub fn vertices(&self) -> &std::collections::BTreeMap<usize, solidcore::topology::VertexId> {
        self.candidate.vertices()
    }
    /// Source-to-native defining edge identities belonging to this result.
    pub fn edges(&self) -> &std::collections::BTreeMap<usize, solidcore::topology::EdgeId> {
        self.candidate.edges()
    }
    /// Source-to-native face identities belonging to this result.
    pub fn faces(&self) -> &std::collections::BTreeMap<usize, solidcore::topology::FaceId> {
        self.candidate.faces()
    }
}
fn line(curve: &Curve) -> Result<&Line> {
    match curve.kind() {
        CurveKind::Line(l) => Ok(l),
        _ => Err(Error::Evidence("nonaffine native representation")),
    }
}
fn coefficients(line: &Line) -> Result<(b::V, b::V)> {
    if line.inside_out() || line.step_interval().min() != 0. || line.step_interval().max() != 1. {
        return Err(Error::Evidence("unexpected affine parameter mapping"));
    }
    Ok((
        b::native(line.line_point()),
        b::times(
            b::native(line.line_vector()),
            b::I::point(line.line_scale()),
        ),
    ))
}
fn polygon(line: &Line) -> Result<(b::V, b::V)> {
    let n = line
        .base()
        .gw_nurb()
        .ok_or(Error::Evidence("missing owned affine NURB"))?;
    if n.degree() != 1 || n.cpts().len() != 2 || n.is_rational() || n.knots() != [0., 0., 1., 1.] {
        return Err(Error::Evidence("nonaffine stored NURB"));
    }
    let p = |i: usize| -> Result<b::V> {
        let c = n.cpts().get(i).ok_or(Error::Evidence("control index"))?;
        Ok(b::native(solidcore::math::Vec3::new(c.x, c.y, c.z)))
    };
    Ok((p(0)?, p(1)?))
}
fn endpoints(line: &Line) -> Result<[(b::V, b::V); 2]> {
    let (o, d) = coefficients(line)?;
    Ok([(o, b::add(o, d)), polygon(line)?])
}
fn certify(bound: f64, budget: Rational, key: &'static str) -> Result<()> {
    if b::within(bound, budget) {
        Ok(())
    } else {
        Err(Error::Evidence(key))
    }
}

fn image(o: b::V, x: b::V, y: b::V, u: b::I, v: b::I) -> b::V {
    b::add(b::add(o, b::times(x, u)), b::times(y, v))
}

// A stored plane is not presumed equivalent merely because its analytic kind
// is Plane. A polynomial bilinear patch minus an affine plane is bilinear;
// its norm is separately convex in u and v, and the four control-corner
// differences bound the complete knot rectangle. No sampled fit is certified.
fn stored_plane_bound(
    plane: &solidcore::surface::plane::Plane,
    o: b::V,
    x: b::V,
    y: b::V,
    domain: solidcore::geom::extent2d::Extent2d,
) -> Result<f64> {
    let base = plane.base();
    let nurb = base
        .gw_nurb()
        .ok_or(Error::Evidence("missing owned plane NURB"))?;
    let net = base.control_point_net()?;
    if nurb.degrees() != (1, 1) || net.u_count != 2 || net.v_count != 2 || !net.weights.is_empty() {
        return Err(Error::Evidence("nonbilinear stored plane"));
    }
    let span = |knots: &[f64]| -> Result<[f64; 2]> {
        if let [a, b, c, d] = knots {
            if a.is_finite() && c.is_finite() && a == b && c == d && a < c {
                return Ok([*a, *c]);
            }
        }
        Err(Error::Evidence("unsupported plane knot span"))
    };
    let us = span(nurb.u_knots())?;
    let vs = span(nurb.v_knots())?;
    if domain.min().x < us[0]
        || domain.max().x > us[1]
        || domain.min().y < vs[0]
        || domain.max().y > vs[1]
    {
        return Err(Error::Evidence("face outside certified plane span"));
    }
    let mut bound: f64 = 0.;
    for (i, u) in us.into_iter().enumerate() {
        for (j, v) in vs.into_iter().enumerate() {
            let p = net
                .points
                .get(i * 2 + j)
                .ok_or(Error::Evidence("plane control index"))?;
            bound = bound.max(b::norm_upper(b::sub(
                b::native(*p),
                image(o, x, y, b::I::point(u), b::I::point(v)),
            )));
        }
    }
    Ok(bound)
}

fn correspondence(model: &Candidate, face: &solidcore_step::planar::FaceEvidence) -> Result<()> {
    let brep = &model.brep;
    if model.vertices.len() != brep.vertex_list().len()
        || model.edges.len() != brep.edge_list().len()
        || model.faces.len() != brep.face_list().len()
    {
        return Err(Error::Evidence("source/native cardinality"));
    }
    for r in brep.region_list() {
        if brep.region(*r)?.is_void == 0 {
            return Err(Error::Evidence("surface role changed to material"));
        }
    }
    let id = *model
        .faces
        .get(&face.face)
        .ok_or(Error::Evidence("source face correspondence"))?;
    let native = brep.face(id)?;
    let mut loops: Vec<_> = face.loops.iter().collect();
    loops.sort_by_key(|l| !l.outer);
    if loops.len() != native.loops.len() {
        return Err(Error::Evidence("source/native loop count"));
    }
    for (source, id) in loops.into_iter().zip(&native.loops) {
        let target = brep.loop_(*id)?;
        if target.face
            != *model
                .faces
                .get(&face.face)
                .ok_or(Error::Evidence("source face identity"))?
            || target.edges.len() != source.edges.len()
        {
            return Err(Error::Evidence("source/native loop incidence"));
        }
        let forward = source.orientation == face.same_sense;
        let first = source
            .edges
            .get(if forward {
                0
            } else {
                source
                    .edges
                    .len()
                    .checked_sub(1)
                    .ok_or(Error::Evidence("empty source loop"))?
            })
            .and_then(|r| model.edges.get(r))
            .ok_or(Error::Evidence("source loop head"))?;
        // Native EndLoop retains the last inserted edge as its list head.
        // A closed oriented cycle has no distinguished start; only rotation is
        // equivalent here. Reversal or any changed incidence still rejects.
        let head = target
            .edges
            .iter()
            .position(|e| e.edge == *first)
            .ok_or(Error::Evidence("native loop head"))?;
        for offset in 0..target.edges.len() {
            let edge = target
                .edges
                .get((head + offset) % target.edges.len())
                .ok_or(Error::Evidence("native cycle index"))?;
            let i = if forward {
                offset
            } else {
                source.edges.len() - 1 - offset
            };
            let record = *source
                .edges
                .get(i)
                .ok_or(Error::Evidence("source loop index"))?;
            let orientation = *source
                .edge_orientations
                .get(i)
                .ok_or(Error::Evidence("source loop direction"))?;
            let definition = model.planar_plan()?.edge(record)?;
            let expected = if forward {
                orientation == definition.same_sense
            } else {
                orientation != definition.same_sense
            };
            if Some(&edge.edge) != model.edges.get(&record) || edge.forward != expected {
                return Err(Error::Evidence("source/native directed use"));
            }
        }
    }
    for (record, id) in &model.edges {
        let definition = model.planar_plan()?.edge(*record)?;
        let edge = brep.edge(*id)?;
        let lo = *model
            .vertices
            .get(&definition.minimum_vertex)
            .ok_or(Error::Evidence("minimum vertex identity"))?;
        let hi = *model
            .vertices
            .get(&definition.maximum_vertex)
            .ok_or(Error::Evidence("maximum vertex identity"))?;
        if edge.vertices != [lo, hi] || edge.interval.min() != 0. || edge.interval.max() != 1. {
            return Err(Error::Evidence("source/native defining edge mapping"));
        }
    }
    Ok(())
}
impl Candidate {
    /// Qualify the first open planar profile using continuous affine bounds and
    /// actual native reports. On error the entire candidate is consumed/released.
    /// Closed/general shapes require their own additional source/native proofs.
    ///
    /// # Errors
    /// Unsupported profile, invalid budgets, unresolved native checks, or a
    /// continuous bound beyond its explicitly selected budget.
    #[allow(clippy::too_many_lines)]
    pub fn accept_open_planar(mut self, quality: AcceptanceQuality) -> Result<Imported> {
        if quality.id.is_empty()
            || quality.id.len() > 256
            || [
                quality.definition_mm,
                quality.derived_uv_mm,
                quality.consistency_mm,
                quality.maximum_work_tolerance_mm,
            ]
            .iter()
            .any(|v| v.numerator() <= 0)
        {
            return Err(Error::Invalid("explicit positive quality budgets required"));
        }
        if self.planar_plan()?.evidence().faces.len() != 1
            || self
                .source()
                .part_definition()
                .is_none_or(|p| p.shape_role != "OpenSurfaceModel")
        {
            return Err(Error::Unsupported(
                "closed/general acceptance needs its independent profile proofs",
            ));
        }
        // Materialize only the candidate's own receiver representation. This
        // is the native lazy GetOrCreate operation, not a fitted replacement.
        // Validate after it, so the returned certificate covers what is owned.
        self.charge()?;
        for id in self.faces.values() {
            let surface = self
                .brep
                .face_mut(*id)?
                .surface
                .as_mut()
                .ok_or(Error::Evidence("missing native receiver"))?;
            solidcore::surface::validity::materialize_bspline(&self.kernel, surface)?;
        }
        let mut model = self.validate_native()?;
        let mut cert = Certificate::default();
        let face = model
            .planar_plan()?
            .evidence()
            .faces
            .first()
            .ok_or(Error::Evidence("missing source region"))?
            .clone();
        correspondence(&model, &face)?;
        let face_id = *model
            .faces
            .get(&face.face)
            .ok_or(Error::Evidence("unmapped source face"))?;
        let surface = model
            .brep
            .face(face_id)?
            .surface
            .as_ref()
            .ok_or(Error::Evidence("missing native surface"))?;
        let SurfaceKind::Plane(plane) = surface.kind() else {
            return Err(Error::Evidence("native surface changed kind"));
        };
        let native_origin = b::native(plane.position().origin());
        let native_x = b::times(
            b::native(plane.position().x_axis()),
            b::I::point(plane.uv_scale().x),
        );
        let native_y = b::times(
            b::native(plane.position().y_axis()),
            b::I::point(plane.uv_scale().y),
        );
        let source_origin = b::exact(face.origin_mm);
        let source_x = b::times(
            b::exact(face.x),
            b::I::point(1.) / b::I::rational(face.x_norm_squared).sqrt(),
        );
        let source_y = b::times(
            b::exact(face.y),
            b::I::point(1.) / b::I::rational(face.y_norm_squared).sqrt(),
        );
        let domain = model.brep.face(face_id)?.uv_domain;
        if ![
            domain.min().x,
            domain.max().x,
            domain.min().y,
            domain.max().y,
        ]
        .iter()
        .all(|x| x.is_finite())
            || domain.min().x >= domain.max().x
            || domain.min().y >= domain.max().y
        {
            return Err(Error::Evidence("nonfinite or empty face domain"));
        }
        // The analytic evaluator clamps to its STEP domain unless its owning
        // context enables out-of-bounds evaluation (SmPlane::EvaluatePoint).
        // Our affine proof must stay inside that domain under either policy.
        let analytic_domain = plane.step_uv_domain();
        for (coordinate, minimum, maximum) in [
            (
                domain.min().x,
                analytic_domain.min().x,
                analytic_domain.max().x,
            ),
            (
                domain.max().x,
                analytic_domain.min().x,
                analytic_domain.max().x,
            ),
            (
                domain.min().y,
                analytic_domain.min().y,
                analytic_domain.max().y,
            ),
            (
                domain.max().y,
                analytic_domain.min().y,
                analytic_domain.max().y,
            ),
        ] {
            if !b::I::point(coordinate).within_range(minimum, maximum) {
                return Err(Error::Evidence("face outside analytic evaluator domain"));
            }
        }
        let plane_gap = stored_plane_bound(plane, native_origin, native_x, native_y, domain)?;
        let stored_definition =
            stored_plane_bound(plane, source_origin, source_x, source_y, domain)?;
        cert.definition_mm = cert.definition_mm.max(stored_definition);
        // Difference of two affine plane definitions is affine. The Euclidean
        // norm is convex, so maxima on this finite rectangle occur at corners.
        for u in [domain.min().x, domain.max().x] {
            for v in [domain.min().y, domain.max().y] {
                let a = image(
                    source_origin,
                    source_x,
                    source_y,
                    b::I::point(u),
                    b::I::point(v),
                );
                let c = image(
                    native_origin,
                    native_x,
                    native_y,
                    b::I::point(u),
                    b::I::point(v),
                );
                cert.definition_mm = cert.definition_mm.max(b::norm_upper(b::sub(a, c)));
            }
        }
        for (source_id, native_id) in &model.vertices {
            let exact = model.planar_plan()?.vertex(*source_id)?;
            cert.definition_mm = cert.definition_mm.max(b::norm_upper(b::sub(
                b::exact(exact),
                b::native(model.brep.vertex(*native_id)?.point),
            )));
            certify(
                model.brep.vertex(*native_id)?.tolerance,
                quality.maximum_work_tolerance_mm,
                "vertex working tolerance",
            )?;
        }
        for source_edge in &model.planar_plan()?.evidence().edges {
            let id = *model
                .edges
                .get(&source_edge.edge)
                .ok_or(Error::Evidence("unmapped source edge"))?;
            let e = model.brep.edge(id)?;
            let curve = line(
                e.curve
                    .as_ref()
                    .ok_or(Error::Evidence("missing defining curve"))?,
            )?;
            let definition = model.planar_plan()?.edge(source_edge.edge)?;
            let low = definition.minimum_vertex;
            let high = definition.maximum_vertex;
            let a = b::exact(definition.minimum_mm);
            let c = b::exact(definition.maximum_mm);
            let va = b::native(
                model
                    .brep
                    .vertex(
                        *model
                            .vertices
                            .get(&low)
                            .ok_or(Error::Evidence("minimum vertex"))?,
                    )?
                    .point,
            );
            let vc = b::native(
                model
                    .brep
                    .vertex(
                        *model
                            .vertices
                            .get(&high)
                            .ok_or(Error::Evidence("maximum vertex"))?,
                    )?
                    .point,
            );
            let mut use_id = None;
            for u in &e.edgeuses {
                if eu::face(&model.brep, *u)? == Some(face_id)
                    && eu::orientation(&model.brep, *u)?
                        == solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same
                {
                    use_id = Some(*u);
                }
            }
            let uv = line(
                model
                    .brep
                    .edgeuse(use_id.ok_or(Error::Evidence("missing canonical UV"))?)?
                    .uv_trim_curve
                    .as_ref()
                    .ok_or(Error::Evidence("missing UV curve"))?,
            )?;
            let uv_endpoints = endpoints(uv)?;
            for (a, b) in uv_endpoints {
                for parameter in [a, b] {
                    if !parameter[0].within_range(domain.min().x, domain.max().x)
                        || !parameter[1].within_range(domain.min().y, domain.max().y)
                    {
                        return Err(Error::Evidence("UV trim outside certified plane domain"));
                    }
                }
            }
            // Affine correspondence uses native t in [0,1]. Both analytic and
            // stored polynomial representations must independently satisfy it.
            for (p, q) in endpoints(curve)? {
                cert.definition_mm = cert
                    .definition_mm
                    .max(b::norm_upper(b::sub(p, a)))
                    .max(b::norm_upper(b::sub(q, c)));
                cert.consistency_mm = cert
                    .consistency_mm
                    .max(b::norm_upper(b::sub(p, va)))
                    .max(b::norm_upper(b::sub(q, vc)));
                for (ua, ub) in uv_endpoints {
                    for (point, parameter, vertex) in [(p, ua, va), (q, ub, vc)] {
                        let lift = image(
                            native_origin,
                            native_x,
                            native_y,
                            parameter[0],
                            parameter[1],
                        );
                        let affine_gap = b::norm_upper(b::sub(lift, point));
                        // Triangle inequality adds the stored-surface bound with
                        // outward software arithmetic, covering either native
                        // receiver representation over the entire affine trim.
                        let total = b::norm_upper([
                            b::I::point(affine_gap) + b::I::point(plane_gap),
                            b::I::point(0.),
                            b::I::point(0.),
                        ]);
                        cert.derived_uv_mm = cert.derived_uv_mm.max(total);
                        // The consistency contract checks the vertex directly
                        // against its receiver image as well. Passing separate
                        // curve/vertex and curve/face budgets cannot establish
                        // this same budget through an unchecked sum of two.
                        let vertex_gap = b::norm_upper(b::sub(lift, vertex));
                        let vertex_total = b::norm_upper([
                            b::I::point(vertex_gap) + b::I::point(plane_gap),
                            b::I::point(0.),
                            b::I::point(0.),
                        ]);
                        cert.consistency_mm = cert.consistency_mm.max(vertex_total);
                    }
                }
            }
            certify(
                e.tolerance,
                quality.maximum_work_tolerance_mm,
                "edge working tolerance",
            )?;
        }
        certify(
            model.brep.tolerance(),
            quality.maximum_work_tolerance_mm,
            "root working tolerance",
        )?;
        certify(
            model.brep.face(face_id)?.tolerance,
            quality.maximum_work_tolerance_mm,
            "face working tolerance",
        )?;
        certify(
            cert.definition_mm,
            quality.definition_mm,
            "continuous defining error",
        )?;
        certify(
            cert.derived_uv_mm,
            quality.derived_uv_mm,
            "continuous derived UV error",
        )?;
        certify(
            cert.consistency_mm,
            quality.consistency_mm,
            "native curve/vertex/receiver consistency",
        )?;
        // This first strict profile permits no validator-driven tolerance growth.
        if model.brep.tolerance() > model.initial_work_tolerance_mm
            || model.edges.values().any(|id| {
                model
                    .brep
                    .edge(*id)
                    .is_ok_and(|e| e.tolerance > model.initial_work_tolerance_mm)
            })
        {
            return Err(Error::Evidence(
                "unexpected native working tolerance growth",
            ));
        }
        if model.vertices.values().any(|id| {
            model
                .brep
                .vertex(*id)
                .is_ok_and(|v| v.tolerance > model.initial_work_tolerance_mm)
        }) || model.brep.face(face_id)?.tolerance > model.initial_work_tolerance_mm
        {
            return Err(Error::Evidence(
                "unexpected native working tolerance growth",
            ));
        }
        model.charge()?;
        Ok(Imported {
            candidate: model,
            quality,
            certificate: cert,
        })
    }
}
