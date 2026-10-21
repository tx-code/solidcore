// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native curved/planar interaction, with real ray confirmation and owner-local IDs.
use crate::{
    camera::Frame,
    scene::{Result, Scene},
};
use glam::{DVec2, DVec3};
use solidcore::{curve::curve::CurveKind, error::EntityRef, surface::surface::SurfaceKind};

#[derive(Clone, Copy)]
pub(crate) struct Hit {
    pub entity: EntityRef,
    pub point: DVec3,
}

#[derive(Default)]
pub(crate) struct Selection {
    pub hover: Option<Hit>,
    pub selected: Vec<EntityRef>,
}
impl Selection {
    pub fn hovered(&self) -> Option<EntityRef> {
        self.hover.map(|h| h.entity)
    }
    pub fn hover(&mut self, hit: Option<Hit>) -> bool {
        let changed = self.hovered() != hit.map(|h| h.entity);
        self.hover = hit;
        changed
    }
    pub fn click(&mut self, multiple: bool) {
        if !multiple {
            self.selected.clear();
        }
        if let Some(entity) = self.hovered() {
            if multiple && self.selected.contains(&entity) {
                self.selected.retain(|id| *id != entity);
            } else {
                if self.selected.len() >= 2 {
                    self.selected = self.selected.iter().skip(1).copied().collect();
                }
                self.selected.push(entity);
            }
        }
    }
}

fn point(p: solidcore::math::Vec3) -> DVec3 {
    DVec3::new(p.x, p.y, p.z)
}
fn native(p: DVec3) -> solidcore::math::Vec3 {
    solidcore::math::Vec3::new(p.x, p.y, p.z)
}
fn normalized(scene: &Scene, p: DVec3) -> DVec3 {
    (p - scene.center_mm) / scene.radius_mm
}

/// Application-selected query tolerances stay separate from display chord error.
fn query_tolerance(scene: &Scene) -> Result<f64> {
    let value = if let Some(report) = scene.owner.general_report() {
        report.quality.query_mm.numerator() as f64 / report.quality.query_mm.denominator() as f64
    } else {
        // Explicit P0 viewer fixture/application configuration, matching loading.rs.
        // This is not a system/native default or a display-derived acceptance value.
        1e-6
    };
    if value.is_finite() && value > 0. {
        Ok(value)
    } else {
        Err("Invalid selected native query tolerance".into())
    }
}

fn face_hit(scene: &mut Scene, frame: Frame, pixel: DVec2) -> Result<Option<Hit>> {
    let Some((origin, direction)) = frame.ray(pixel) else {
        return Ok(None);
    };
    let origin_mm = scene.center_mm + origin * scene.radius_mm;
    let tolerance = query_tolerance(scene)?;
    // Display triangles are never a replacement for trimmed native intersection.
    // Query the same owner even when a chord near a curved silhouette misses the
    // cursor; gating this ray on a coarse display triangle would hide true hits.
    let hit = scene
        .owner
        .ray_hit(native(origin_mm), native(direction), tolerance, None)
        .map_err(|e| format!("Native picking ray: {e}"))?;
    Ok(hit.map(|hit| Hit {
        entity: hit.entity,
        point: point(hit.point_mm),
    }))
}

fn visible(scene: &mut Scene, frame: Frame, position: DVec3) -> Result<bool> {
    let Some(projected) = frame.project(normalized(scene, position)) else {
        return Ok(false);
    };
    let Some(occluder) = face_hit(scene, frame, projected.pixel)? else {
        return Ok(true);
    };
    let Some(front) = frame.project(normalized(scene, occluder.point)) else {
        return Ok(false);
    };
    // This is float-depth display precision, not enlargement of native ray/query
    // budgets. Actual foreground geometry still occludes every candidate kind.
    Ok(projected.depth <= front.depth + 8. * f64::from(f32::EPSILON))
}

fn curve_candidate(
    scene: &Scene,
    frame: Frame,
    pixel: DVec2,
    id: solidcore::topology::EdgeId,
    low: f64,
    high: f64,
    initial: f64,
    evaluations: &mut usize,
) -> Result<Option<(f64, DVec3)>> {
    let edge = scene.owner.brep().edge(id).map_err(|e| e.to_string())?;
    let curve = edge.curve.as_ref().ok_or("Missing native defining curve")?;
    let mut score = |t: f64| -> Result<(f64, DVec3)> {
        *evaluations = evaluations
            .checked_add(1)
            .ok_or("Curve picking count overflow")?;
        if *evaluations > 16_384 {
            return Err("Curve picking evaluation budget exceeded".into());
        }
        let position = point(
            curve
                .evaluate_point(scene.owner.kernel(), t)
                .map_err(|e| e.to_string())?,
        );
        if !position.is_finite() {
            return Err("Nonfinite native curve picking point".into());
        }
        let distance = frame
            .project(normalized(scene, position))
            .map_or(f64::INFINITY, |p| p.pixel.distance_squared(pixel));
        Ok((distance, position))
    };
    let mut best = score(initial.clamp(low, high))?;
    for t in [low, high] {
        let next = score(t)?;
        if next.0 < best.0 {
            best = next;
        }
    }
    // The display chord supplies only a short original-parameter bracket. Refine
    // the cursor distance by evaluating the native basis inside that interval;
    // no mesh-edge coordinate or polyline length becomes a CAD measurement.
    let (mut a, mut b) = (low, high);
    let ratio = 0.618_033_988_749_894_9;
    let mut c = b - ratio * (b - a);
    let mut d = a + ratio * (b - a);
    let mut fc = score(c)?;
    let mut fd = score(d)?;
    for _ in 0..18 {
        if fc.0 < best.0 {
            best = fc;
        }
        if fd.0 < best.0 {
            best = fd;
        }
        if fc.0 < fd.0 {
            b = d;
            d = c;
            fd = fc;
            c = b - ratio * (b - a);
            fc = score(c)?;
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + ratio * (b - a);
            fd = score(d)?;
        }
    }
    Ok(best.0.is_finite().then_some((best.0.sqrt(), best.1)))
}

pub(crate) fn pick(scene: &mut Scene, frame: Frame, pixel: DVec2) -> Result<Option<Hit>> {
    if !frame.contains(pixel) {
        return Ok(None);
    }
    let radius = 6. * f64::from(frame.pixels_per_point);
    let mut vertices = Vec::new();
    for id in scene.owner.brep().vertex_list() {
        let position = point(
            scene
                .owner
                .brep()
                .vertex(*id)
                .map_err(|e| e.to_string())?
                .point,
        );
        if let Some(p) = frame.project(normalized(scene, position)) {
            let distance = p.pixel.distance(pixel);
            if distance <= radius {
                vertices.push((distance, *id, position));
            }
        }
    }
    vertices.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    for (_, id, position) in vertices {
        if visible(scene, frame, position)? {
            return Ok(Some(Hit {
                entity: EntityRef::Vertex(id),
                point: position,
            }));
        }
    }
    let mut edges = Vec::new();
    let mut evaluations = 0usize;
    // All native pieces and derived chart cuts are present in this owner-local
    // table. Two pieces of one source edge retain their distinct native IDs.
    for (id, samples) in &scene.edge_samples {
        for pair in samples.windows(2) {
            let [start, end] = pair else {
                return Err("Incomplete native curve candidate segment".into());
            };
            let Some((a, b, low, high)) = frame.segment(
                normalized(scene, start.point_mm),
                normalized(scene, end.point_mm),
            ) else {
                continue;
            };
            let delta = b.pixel - a.pixel;
            let fraction = if delta.length_squared() > 0. {
                ((pixel - a.pixel).dot(delta) / delta.length_squared()).clamp(0., 1.)
            } else {
                0.
            };
            let projected = a.pixel + fraction * delta;
            // The screen broad phase is deliberately wider than the hit radius.
            // Only a subsequent original-curve evaluation can produce an edge hit.
            let chord_pixels = scene.display_tolerance_mm / scene.radius_mm
                * f64::from(frame.height)
                / a.w.min(b.w).max(1e-9);
            if projected.distance(pixel) > radius + 4. + chord_pixels {
                continue;
            }
            let denominator = (1. - fraction) / a.w + fraction / b.w;
            if !denominator.is_finite() || denominator <= 0. {
                continue;
            }
            let fraction = low + (high - low) * (fraction / b.w) / denominator;
            let initial = start.parameter + (end.parameter - start.parameter) * fraction;
            if let Some((distance, position)) = curve_candidate(
                scene,
                frame,
                pixel,
                *id,
                start.parameter,
                end.parameter,
                initial,
                &mut evaluations,
            )? {
                if distance <= radius {
                    edges.push((distance, *id, position));
                }
            }
        }
    }
    edges.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut tried = std::collections::BTreeSet::new();
    for (_, id, position) in edges {
        if !tried.insert(id) {
            continue;
        }
        if visible(scene, frame, position)? {
            return Ok(Some(Hit {
                entity: EntityRef::Edge(id),
                point: position,
            }));
        }
    }
    face_hit(scene, frame, pixel)
}

pub(crate) fn label(scene: &Scene, entity: EntityRef) -> Result<String> {
    if let EntityRef::Vertex(id) = entity {
        if let Some(origin) = scene
            .owner
            .derived_vertices()
            .iter()
            .find(|origin| origin.native_vertex == id)
        {
            // A chart cut has no source VERTEX_POINT. Report its actual edge
            // origin and complete-basis parameter instead of inventing one.
            scene
                .owner
                .brep()
                .vertex(id)
                .map_err(|error| error.to_string())?;
            let source_edge = scene
                .owner
                .source()
                .source()
                .records()
                .get(origin.source_edge)
                .ok_or("Derived vertex source edge record missing")?;
            if !origin.native_parameter.is_finite() {
                return Err("Derived vertex has a nonfinite source-basis parameter".into());
            }
            return Ok(format!(
                "From edge #{} at t={} · native {entity}",
                source_edge.id, origin.native_parameter
            ));
        }
    }
    if let EntityRef::Edge(id) = entity {
        if let Some(report) = scene.owner.general_report() {
            if let Some((source_face, _)) = report.chart_cuts.iter().find(|(_, edge)| *edge == id) {
                scene.owner.brep().edge(id).map_err(|e| e.to_string())?;
                let source = scene
                    .owner
                    .source()
                    .source()
                    .records()
                    .get(*source_face)
                    .ok_or("Derived seam source face missing")?;
                return Ok(format!(
                    "Derived chart seam on face #{} · native {entity}",
                    source.id
                ));
            }
        }
    }
    let record = match entity {
        EntityRef::Face(id) => scene
            .owner
            .faces()
            .iter()
            .find(|(_, target)| **target == id)
            .map(|(record, _)| *record),
        EntityRef::Edge(id) => scene
            .owner
            .edges()
            .iter()
            .find(|(_, pieces)| pieces.contains(&id))
            .map(|(record, _)| *record),
        EntityRef::Vertex(id) => scene
            .owner
            .vertices()
            .iter()
            .find(|(_, target)| **target == id)
            .map(|(record, _)| *record),
        _ => None,
    }
    .ok_or("Selection does not belong to this source/native owner")?;
    let source = scene
        .owner
        .source()
        .source()
        .records()
        .get(record)
        .ok_or("Selection source record missing")?;
    Ok(format!("#{source} · native {entity}", source = source.id))
}

/// A completed native numerical receipt wins; analytic line/circle intervals are
/// evaluated directly, other definitions use the named native sampled query.
pub(crate) fn edge_length(scene: &Scene, id: solidcore::topology::EdgeId) -> Result<f64> {
    if let Some(measured) = scene
        .owner
        .general_report()
        .and_then(|report| report.material_measurements)
        .and_then(|m| m.edges.iter().find(|edge| edge.native_edge == id))
    {
        return Ok(measured.length_mm);
    }
    let edge = scene.owner.brep().edge(id).map_err(|e| e.to_string())?;
    let curve = edge.curve.as_ref().ok_or("Missing native edge curve")?;
    let length = match curve.kind() {
        CurveKind::Line(line) => line.length(&edge.interval),
        CurveKind::Ellipse(circle) if circle.is_circle_class() => {
            let first = circle
                .convert_t_from_nurbs_to_step(scene.owner.kernel(), edge.interval.min())
                .map_err(|e| e.to_string())?;
            let last = circle
                .convert_t_from_nurbs_to_step(scene.owner.kernel(), edge.interval.max())
                .map_err(|e| e.to_string())?;
            circle.radius() * (last - first).abs() * std::f64::consts::PI / 180.
        }
        _ => {
            curve
                .approximate_length(scene.owner.kernel(), &edge.interval, 4096, false)
                .map_err(|e| e.to_string())?
                .0
        }
    };
    if length.is_finite() && length >= 0. {
        Ok(length)
    } else {
        Err("Nonfinite native edge length".into())
    }
}

pub(crate) fn details(scene: &Scene, selection: &Selection) -> Result<Vec<String>> {
    let mut rows = Vec::new();
    if let Some(hit) = selection.hover {
        rows.push(format!("Hover: {}", label(scene, hit.entity)?));
    }
    for entity in &selection.selected {
        rows.push(format!("Selected: {}", label(scene, *entity)?));
        match *entity {
            EntityRef::Vertex(id) => {
                let p = scene
                    .owner
                    .brep()
                    .vertex(id)
                    .map_err(|e| e.to_string())?
                    .point;
                rows.push(format!("Vertex mm: {:.6}, {:.6}, {:.6}", p.x, p.y, p.z));
            }
            EntityRef::Edge(id) => {
                let edge = scene.owner.brep().edge(id).map_err(|e| e.to_string())?;
                let curve = edge
                    .curve
                    .as_ref()
                    .ok_or("Missing native defining edge curve")?;
                let sampled = !matches!(curve.kind(), CurveKind::Line(_))
                    && !matches!(curve.kind(),CurveKind::Ellipse(circle) if circle.is_circle_class());
                rows.push(format!(
                    "Native {}length: {:.6} mm",
                    if sampled {
                        "sampled curve "
                    } else {
                        "analytic interval "
                    },
                    edge_length(scene, id)?
                ));
            }
            EntityRef::Face(id) => {
                let face = scene.owner.brep().face(id).map_err(|e| e.to_string())?;
                let surface = face.surface.as_ref().ok_or("Missing face geometry")?;
                rows.push(format!("Surface: {}", surface.type_string()));
                if let Some(area) = scene
                    .owner
                    .general_report()
                    .and_then(|report| report.material_measurements)
                    .and_then(|m| m.faces.iter().find(|f| f.native_face == id))
                {
                    rows.push(format!("Native numerical area: {:.6} mm²", area.area_mm2));
                }
                match surface.kind() {
                    SurfaceKind::Plane(plane) => {
                        let normal = plane.evaluate_normal();
                        rows.push(format!(
                            "Plane normal: {:.6}, {:.6}, {:.6}",
                            normal.x, normal.y, normal.z
                        ));
                    }
                    SurfaceKind::Cone(cone) => {
                        if cone.is_cylinder() {
                            rows.push(format!("Cylinder radius: {:.6} mm", cone.bot_radius()));
                        } else {
                            rows.push(format!("Cone semi-angle: {:.6}°", cone.semi_angle_deg()));
                        }
                    }
                    _ => {}
                }
            }
            _ => return Err("Unsupported selection entity".into()),
        }
    }
    if let [EntityRef::Vertex(a), EntityRef::Vertex(b)] = selection.selected.as_slice() {
        let a = scene
            .owner
            .brep()
            .vertex(*a)
            .map_err(|e| e.to_string())?
            .point;
        let b = scene
            .owner
            .brep()
            .vertex(*b)
            .map_err(|e| e.to_string())?
            .point;
        rows.push(format!("Native vertex distance: {:.6} mm", a.distance(b)));
    }
    Ok(rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::Camera;
    use solidcore::intersect::point_class::PointClassType;
    use solidcore::topology::{EdgeId, FaceId, VertexId};
    use std::fmt::Write;
    const PLANE: &str = include_str!("../../solidcore-step/tests/fixtures/plane.step");
    fn load(text: &str) -> Scene {
        crate::scene::from_bytes(text.as_bytes().to_vec(), |_| Ok(())).unwrap()
    }
    fn frame(scale: f32) -> Frame {
        Camera::default().frame(
            egui::Rect::from_min_max(egui::pos2(300., 0.), egui::pos2(1180., 800.)),
            scale,
            [(1180. * scale) as u32, (800. * scale) as u32],
        )
    }
    fn record(scene: &Scene, id: &str) -> usize {
        scene
            .owner
            .source()
            .source()
            .records()
            .iter()
            .position(|r| r.id == id)
            .unwrap()
    }
    fn vertex(scene: &Scene, id: &str) -> VertexId {
        *scene.owner.vertices().get(&record(scene, id)).unwrap()
    }
    fn edge(scene: &Scene, id: &str) -> EdgeId {
        let pieces = scene.owner.edges().get(&record(scene, id)).unwrap();
        let [edge] = pieces.as_slice() else {
            panic!("planar fixture edge must have one native piece");
        };
        *edge
    }
    fn face(scene: &Scene, id: &str) -> FaceId {
        *scene.owner.faces().get(&record(scene, id)).unwrap()
    }
    fn pixel(scene: &Scene, frame: Frame, p: DVec3) -> DVec2 {
        frame.project(normalized(scene, p)).unwrap().pixel
    }
    fn vertex_point(scene: &Scene, id: VertexId) -> DVec3 {
        point(scene.owner.brep().vertex(id).unwrap().point)
    }

    #[test]
    fn real_pipeline_native_ids_face_edge_vertex_and_dpi_are_retained() {
        let mut scene = load(PLANE);
        let f = face(&scene, "302");
        let e = edge(&scene, "203");
        let v = vertex(&scene, "110");
        for scale in [1., 2.] {
            let view = frame(scale);
            let center = pixel(&scene, view, DVec3::new(3.4, 5.2, 5.));
            assert_eq!(
                pick(&mut scene, view, center).unwrap().unwrap().entity,
                EntityRef::Face(f)
            );
            let midpoint = pixel(&scene, view, DVec3::new(6.6, 2.8, 5.));
            assert_eq!(
                pick(&mut scene, view, midpoint).unwrap().unwrap().entity,
                EntityRef::Edge(e)
            );
            let near_vertex = pixel(&scene, view, vertex_point(&scene, v))
                + DVec2::new(4. * f64::from(scale), 0.);
            assert_eq!(
                pick(&mut scene, view, near_vertex).unwrap().unwrap().entity,
                EntityRef::Vertex(v)
            );
        }
        assert!(label(&scene, EntityRef::Edge(e)).unwrap().contains("#203"));
        assert_eq!(scene.owner.source().source().bytes(), PLANE.as_bytes());
        assert!((edge_length(&scene, e).unwrap() - 12.).abs() < 1e-10);
    }

    #[test]
    fn reversed_source_direction_centimeter_units_and_native_vertex_distance() {
        let text = PLANE
            .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)")
            .replace(
                "EDGE_CURVE('',#110,#111,#202,.T.)",
                "EDGE_CURVE('',#111,#110,#202,.F.)",
            )
            .replace(
                "ORIENTED_EDGE('',*,*,#203,.T.)",
                "ORIENTED_EDGE('',*,*,#203,.F.)",
            );
        let mut scene = load(&text);
        let view = frame(1.5);
        let e = edge(&scene, "203");
        let a = vertex(&scene, "110");
        let b = vertex(&scene, "111");
        let p = vertex_point(&scene, a);
        let q = vertex_point(&scene, b);
        let cursor = pixel(&scene, view, (p + q) * 0.5);
        assert_eq!(
            pick(&mut scene, view, cursor).unwrap().unwrap().entity,
            EntityRef::Edge(e)
        );
        assert!((edge_length(&scene, e).unwrap() - 120.).abs() < 1e-9);
        let mut selection = Selection {
            hover: Some(Hit {
                entity: EntityRef::Vertex(a),
                point: p,
            }),
            ..Default::default()
        };
        selection.click(false);
        selection.hover = Some(Hit {
            entity: EntityRef::Vertex(b),
            point: q,
        });
        selection.click(true);
        assert!(details(&scene, &selection)
            .unwrap()
            .iter()
            .any(|row| row == "Native vertex distance: 120.000000 mm"));
        assert_eq!(scene.owner.source().source().bytes(), text.as_bytes());
    }

    fn with_hole() -> String {
        let mut extra = String::new();
        for (i, ((x, y), (dx, dy))) in [
            ((3.8, 2.4), (-0.8, 0.6)),
            ((0.6, 4.8), (0.6, 0.8)),
            ((3.0, 8.0), (0.8, -0.6)),
            ((6.2, 5.6), (-0.6, -0.8)),
        ]
        .into_iter()
        .enumerate()
        {
            let p = 400 + i;
            let v = 410 + i;
            let next = 410 + (i + 1) % 4;
            let d = 420 + 10 * i;
            writeln!(
                extra,
                "#{p}=CARTESIAN_POINT('',({x:.1},{y:.1},5.));\n#{v}=VERTEX_POINT('',#{p});"
            )
            .unwrap();
            writeln!(extra,"#{d}=DIRECTION('',({dx:.1},{dy:.1},0.));\n#{}=VECTOR('',#{d},4.);\n#{}=LINE('',#{p},#{});\n#{}=EDGE_CURVE('',#{v},#{next},#{},.T.);\n#{}=ORIENTED_EDGE('',*,*,#{},.T.);",
                d+1,d+2,d+1,d+3,d+2,d+4,d+3).unwrap();
        }
        extra
            .push_str("#460=EDGE_LOOP('',(#424,#434,#444,#454));\n#461=FACE_BOUND('',#460,.T.);\n");
        PLANE.replace(
            "#302=ADVANCED_FACE('',(#301),#134,.T.);",
            &format!("#302=ADVANCED_FACE('',(#301,#461),#134,.T.);\n{extra}"),
        )
    }
    #[test]
    fn actual_native_trim_query_and_display_candidates_both_reject_the_hole() {
        let mut scene = load(&with_hole());
        // Independent planar truth: 12×8 outer rectangle minus a 4×4 hole.
        // This checks the displayed trim region as well as the native query.
        let (triangles, remainder) = scene.triangles.as_chunks::<3>();
        assert!(remainder.is_empty());
        let area = triangles
            .iter()
            .map(|[a, b, c]| {
                let p = |v: &crate::scene::Vertex| DVec3::from_array(v.position.map(f64::from));
                (p(b) - p(a)).cross(p(c) - p(a)).length() * 0.5 * scene.radius_mm * scene.radius_mm
            })
            .sum::<f64>();
        assert!((area - 80.).abs() < 1e-4);
        let view = frame(1.);
        let f = face(&scene, "302");
        let classification = scene
            .owner
            .classify_face_uv(f, solidcore::math::Vec2::new(6., 4.))
            .unwrap();
        assert!(!matches!(
            classification,
            PointClassType::Face | PointClassType::Edge | PointClassType::Vertex
        ));
        let cursor = pixel(&scene, view, DVec3::new(3.4, 5.2, 5.));
        assert!(pick(&mut scene, view, cursor).unwrap().is_none());
        let cursor = pixel(&scene, view, DVec3::new(2.8, -0.6, 5.));
        assert_eq!(
            pick(&mut scene, view, cursor).unwrap().unwrap().entity,
            EntityRef::Face(f)
        );
    }

    fn overlapping_faces() -> String {
        // A connected folded triangle shares the original edge #233, with
        // its new vertex behind the rectangular face. This stays one real
        // native shell rather than using an invalid disconnected test graph.
        let back="\n#600=CARTESIAN_POINT('',(1.6,2.8,4.));\n#610=VERTEX_POINT('',#600);
#630=DIRECTION('',(-0.6,-0.8,-3.));\n#631=DIRECTION('',(-0.8,0.6,0.));\n#632=AXIS2_PLACEMENT_3D('',#100,#630,#631);\n#634=PLANE('',#632);
#700=DIRECTION('',(5.,0.,-1.));\n#701=VECTOR('',#700,1.);\n#702=LINE('',#103,#701);\n#703=EDGE_CURVE('',#113,#610,#702,.T.);\n#704=ORIENTED_EDGE('',*,*,#703,.T.);
#710=DIRECTION('',(1.4,-4.8,1.));\n#711=VECTOR('',#710,1.);\n#712=LINE('',#600,#711);\n#713=EDGE_CURVE('',#610,#110,#712,.T.);\n#714=ORIENTED_EDGE('',*,*,#713,.T.);
#734=ORIENTED_EDGE('',*,*,#233,.F.);\n#800=EDGE_LOOP('',(#734,#704,#714));\n#801=FACE_OUTER_BOUND('',#800,.T.);\n#802=ADVANCED_FACE('',(#801),#634,.T.);\n";
        let original = PLANE.replace(
            "#22=OPEN_SHELL('',(#302));",
            "#22=OPEN_SHELL('',(#302,#802));",
        );
        let split = original.rfind("ENDSEC;").unwrap();
        format!(
            "{}{}{}",
            original.get(..split).unwrap(),
            back,
            original.get(split..).unwrap()
        )
    }
    #[test]
    fn real_front_face_occludes_native_back_vertices_and_edges() {
        let mut scene = load(&overlapping_faces());
        let view = frame(1.);
        let front = face(&scene, "302");
        let back = vertex(&scene, "610");
        let cursor = pixel(&scene, view, vertex_point(&scene, back));
        assert_eq!(
            pick(&mut scene, view, cursor).unwrap().unwrap().entity,
            EntityRef::Face(front)
        );
        let a = vertex_point(&scene, back);
        let b = vertex_point(&scene, vertex(&scene, "113"));
        let cursor = pixel(&scene, view, (a + b) * 0.5);
        assert_eq!(
            pick(&mut scene, view, cursor).unwrap().unwrap().entity,
            EntityRef::Face(front)
        );
    }

    #[test]
    fn real_curved_face_and_circle_picking_use_native_points_at_each_dpi() {
        use crate::scene::public_controls::{scene as curved_scene, CYLINDER};
        let mut scene = curved_scene(CYLINDER);
        let side = face(&scene, "182");
        let upper = scene
            .owner
            .edges()
            .get(&record(&scene, "131"))
            .unwrap()
            .clone();
        let coordinate = 3. / 2_f64.sqrt();
        for scale in [1., 2.] {
            let view = frame(scale);
            let cursor = pixel(&scene, view, DVec3::new(coordinate, coordinate, 3.5));
            let hit = pick(&mut scene, view, cursor).unwrap().unwrap();
            assert_eq!(hit.entity, EntityRef::Face(side));
            assert!((hit.point.x * hit.point.x + hit.point.y * hit.point.y - 9.).abs() < 1e-5);
            let cursor = pixel(&scene, view, DVec3::new(coordinate, coordinate, 7.));
            let hit = pick(&mut scene, view, cursor).unwrap().unwrap();
            let EntityRef::Edge(id) = hit.entity else {
                panic!("public circle pick returned {}", hit.entity)
            };
            assert!(upper.contains(&id));
            // The point comes from the original circle basis, not from the
            // display chord that supplied its original-parameter bracket.
            assert!((hit.point.x * hit.point.x + hit.point.y * hit.point.y - 9.).abs() < 1e-9);
            assert!((hit.point.z - 7.).abs() < 1e-9);
            assert!(label(&scene, hit.entity).unwrap().contains("#131"));
        }
        assert_eq!(scene.owner.source().source().bytes(), CYLINDER);
        // Native ray queries may mutate caches and therefore invalidate the old
        // numerical trim receipt; rendering alone must not be re-labelled as it.
        assert!(scene
            .owner
            .general_report()
            .unwrap()
            .final_trim_measurements
            .is_none());
    }

    #[test]
    fn curved_source_pieces_derived_chart_provenance_and_analytic_lengths_remain_distinct() {
        use crate::scene::public_controls::{scene as curved_scene, CYLINDER};
        let mut scene = curved_scene(CYLINDER);
        let upper = scene.owner.edges().get(&record(&scene, "131")).unwrap();
        assert_eq!(upper.len(), 2);
        let total = upper
            .iter()
            .map(|id| edge_length(&scene, *id).unwrap())
            .sum::<f64>();
        assert!((total - 6. * std::f64::consts::PI).abs() < 1e-10);
        let report = scene.owner.general_report().unwrap();
        let seam = report.chart_cuts[0].1;
        let origin = &report.derived_vertices[0];
        let vertex = origin.native_vertex;
        assert!((edge_length(&scene, seam).unwrap() - 7.).abs() < 1e-10);
        assert!(label(&scene, EntityRef::Edge(seam))
            .unwrap()
            .contains("Derived chart seam on face #182"));
        assert_eq!(
            label(&scene, EntityRef::Vertex(vertex)).unwrap(),
            format!(
                "From edge #131 at t={} · native {}",
                origin.native_parameter,
                EntityRef::Vertex(vertex)
            )
        );
        let rows = details(
            &scene,
            &Selection {
                hover: None,
                selected: vec![EntityRef::Face(face(&scene, "182"))],
            },
        )
        .unwrap();
        assert!(rows.iter().any(|row| row == "Cylinder radius: 3.000000 mm"));
        assert!(rows.iter().all(|row| !row.starts_with("Plane normal:")));

        // A display-buffer miss cannot suppress a true curved native ray hit.
        scene.triangles.clear();
        let view = frame(1.);
        let coordinate = 3. / 2_f64.sqrt();
        let cursor = pixel(&scene, view, DVec3::new(coordinate, coordinate, 3.5));
        assert_eq!(
            pick(&mut scene, view, cursor).unwrap().unwrap().entity,
            EntityRef::Face(face(&scene, "182"))
        );
    }
}
