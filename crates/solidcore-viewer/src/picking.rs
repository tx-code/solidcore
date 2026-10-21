// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! LINE/PLANE interaction, with native confirmation and owner-local identities.
use crate::{
    camera::Frame,
    scene::{Result, Scene},
};
use glam::{DVec2, DVec3};
use solidcore::{
    curve::curve::CurveKind, error::EntityRef, intersect::point_class::PointClassType,
    surface::surface::SurfaceKind,
};

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

fn triangle_candidate(origin: DVec3, direction: DVec3, a: DVec3, b: DVec3, c: DVec3) -> bool {
    let ab = b - a;
    let ac = c - a;
    let cross = direction.cross(ac);
    let determinant = ab.dot(cross);
    if determinant.abs() < 1e-14 {
        return false;
    }
    let offset = origin - a;
    let u = offset.dot(cross) / determinant;
    let q = offset.cross(ab);
    let v = direction.dot(q) / determinant;
    u >= -1e-7 && v >= -1e-7 && u + v <= 1. + 1e-7 && ac.dot(q) / determinant >= 0.
}

fn face_hit(scene: &mut Scene, frame: Frame, pixel: DVec2) -> Result<Option<Hit>> {
    let Some((origin, direction)) = frame.ray(pixel) else {
        return Ok(None);
    };
    // The f32 display triangles are a broad phase only. Every returned hit is
    // re-intersected with the native plane and checked by the native trim query.
    let mut candidates = Vec::new();
    for (id, range) in &scene.face_ranges {
        let mesh = scene
            .triangles
            .get(range.clone())
            .ok_or("Face display range changed")?;
        let (triangles, _) = mesh.as_chunks::<3>();
        if triangles.iter().any(|[a, b, c]| {
            triangle_candidate(
                origin,
                direction,
                DVec3::from_array(a.position.map(f64::from)),
                DVec3::from_array(b.position.map(f64::from)),
                DVec3::from_array(c.position.map(f64::from)),
            )
        }) {
            candidates.push(*id);
        }
    }
    let world_origin = scene.center_mm + origin * scene.radius_mm;
    let mut closest: Option<(f64, Hit)> = None;
    for id in candidates {
        let (distance, position, uv) = {
            let face = scene.owner.brep().face(id).map_err(|e| e.to_string())?;
            let SurfaceKind::Plane(plane) =
                face.surface.as_ref().ok_or("Missing native plane")?.kind()
            else {
                return Err("Picking supports native PLANE only".into());
            };
            let normal = point(plane.position().x_axis().cross(plane.position().y_axis()));
            let denominator = normal.dot(direction);
            if denominator.abs() < 1e-12 {
                continue;
            }
            let distance =
                normal.dot(point(plane.position().origin()) - world_origin) / denominator;
            if !distance.is_finite() || distance < 0. {
                continue;
            }
            let position = world_origin + distance * direction;
            (
                distance,
                position,
                plane.project_point_to_uv_domain(native(position)),
            )
        };
        let classification = scene
            .owner
            .classify_face_uv(id, uv)
            .map_err(|e| e.to_string())?;
        if matches!(
            classification,
            PointClassType::Face | PointClassType::Edge | PointClassType::Vertex
        ) && closest
            .as_ref()
            .is_none_or(|(previous, _)| distance < *previous)
        {
            closest = Some((
                distance,
                Hit {
                    entity: EntityRef::Face(id),
                    point: position,
                },
            ));
        }
    }
    Ok(closest.map(|(_, hit)| hit))
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
    // Match float-depth display precision, not an import/error budget. A
    // foreground native face still rejects hidden vertices and boundary lines.
    Ok(projected.depth <= front.depth + 8. * f64::from(f32::EPSILON))
}

pub(crate) fn pick(scene: &mut Scene, frame: Frame, pixel: DVec2) -> Result<Option<Hit>> {
    if !frame.contains(pixel) {
        return Ok(None);
    }
    let radius = 6. * f64::from(frame.pixels_per_point);
    let mut vertices = Vec::new();
    for id in scene.owner.vertices().values() {
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
    for id in scene.owner.edges().values() {
        let edge = scene.owner.brep().edge(*id).map_err(|e| e.to_string())?;
        let CurveKind::Line(line) = edge
            .curve
            .as_ref()
            .ok_or("Missing native defining curve")?
            .kind()
        else {
            return Err("Picking supports native LINE only".into());
        };
        let start = point(line.evaluate_point(edge.interval.min()));
        let end = point(line.evaluate_point(edge.interval.max()));
        let Some((a, b, low, high)) =
            frame.segment(normalized(scene, start), normalized(scene, end))
        else {
            continue;
        };
        let delta = b.pixel - a.pixel;
        let fraction = if delta.length_squared() > 0. {
            (pixel - a.pixel).dot(delta) / delta.length_squared()
        } else {
            0.
        }
        .clamp(0., 1.);
        // Perspective-correct interpolation recovers a parameter of the
        // ORIGINAL native LINE, rather than a mesh edge or screen-space chord.
        let denominator = (1. - fraction) / a.w + fraction / b.w;
        if !denominator.is_finite() || denominator <= 0. {
            continue;
        }
        let t = low + (high - low) * (fraction / b.w) / denominator;
        let position = point(line.evaluate_point(edge.interval.min() + t * edge.interval.length()));
        if let Some(p) = frame.project(normalized(scene, position)) {
            let distance = p.pixel.distance(pixel);
            if distance <= radius {
                edges.push((distance, *id, position));
            }
        }
    }
    edges.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    for (_, id, position) in edges {
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
            .find(|(_, target)| **target == id)
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

pub(crate) fn edge_length(scene: &Scene, id: solidcore::topology::EdgeId) -> Result<f64> {
    let edge = scene.owner.brep().edge(id).map_err(|e| e.to_string())?;
    let CurveKind::Line(line) = edge
        .curve
        .as_ref()
        .ok_or("Missing native edge curve")?
        .kind()
    else {
        return Err("Length measurement currently requires native LINE".into());
    };
    let length = line.length(&edge.interval);
    if length.is_finite() {
        Ok(length)
    } else {
        Err("Nonfinite native LINE length".into())
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
            EntityRef::Edge(id) => rows.push(format!(
                "Native LINE length: {:.6} mm",
                edge_length(scene, id)?
            )),
            EntityRef::Face(id) => {
                let face = scene.owner.brep().face(id).map_err(|e| e.to_string())?;
                let SurfaceKind::Plane(plane) =
                    face.surface.as_ref().ok_or("Missing face geometry")?.kind()
                else {
                    return Err("Face is not a native PLANE".into());
                };
                let n = plane.evaluate_normal();
                rows.push(format!(
                    "PLANE surface normal: {:.6}, {:.6}, {:.6}",
                    n.x, n.y, n.z
                ));
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
        *scene.owner.edges().get(&record(scene, id)).unwrap()
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
}
