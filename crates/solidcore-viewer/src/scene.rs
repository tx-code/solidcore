// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native planar trim rendering. This display triangulation grants no import acceptance.

use glam::DVec3;
use solidcore::curve::curve::CurveKind;
use solidcore::math::Vec2;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::surface::SurfaceKind;
use solidcore::topology::edgeuse::p1 as eu;
use solidcore::topology::{EdgeId, FaceId, VertexId};
use solidcore_step::{planar, semantics};
use solidcore_step_native::{Candidate, Options};
use std::io::Read;
use std::ops::Range;
use std::path::Path;

pub(crate) type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Copy)]
pub(crate) struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

pub(crate) struct Scene {
    // Preserve the original source, native definitions and their correspondence.
    // Drawing data is derived from this owner, never a substitute import result.
    pub owner: Candidate,
    pub triangles: Vec<Vertex>,
    pub lines: Vec<Vertex>,
    pub center_mm: DVec3,
    pub radius_mm: f64,
    pub role: String,
    pub face_ranges: Vec<(FaceId, Range<usize>)>,
    pub edge_ranges: Vec<(EdgeId, Range<usize>)>,
    pub vertices: Vec<(VertexId, Vertex)>,
}

fn native_point(value: solidcore::math::Vec3) -> Result<DVec3> {
    let point = DVec3::new(value.x, value.y, value.z);
    if point.is_finite() {
        Ok(point)
    } else {
        Err("Nonfinite native display coordinate".into())
    }
}

pub(crate) fn load(path: &Path, progress: impl Fn(&str) -> Result<()>) -> Result<Scene> {
    progress("1 / 6  Reading bounded STEP source")?;
    let limits = solidcore_step::Limits::planar_fixtures();
    let mut bytes = Vec::new();
    let input_limit = u64::try_from(limits.input_bytes).map_err(|e| e.to_string())?;
    std::fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(input_limit.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    from_bytes(bytes, progress)
}

pub(crate) fn from_bytes(bytes: Vec<u8>, progress: impl Fn(&str) -> Result<()>) -> Result<Scene> {
    let source = solidcore_step::inspect(bytes, solidcore_step::Limits::planar_fixtures());
    if source.status != solidcore_step::Status::Inspected {
        return Err(format!(
            "Source {:?}: {:?}",
            source.status,
            source.diagnostics.first()
        ));
    }
    progress("2 / 6  Checking source roles and units")?;
    let semantic = semantics::check(
        &source,
        semantics::Limits::fixtures(),
        &semantics::Options {
            check_part_and_closure: true,
            check_geometry_dependencies: true,
            ..Default::default()
        },
    );
    if semantic.status != semantics::Status::Checked {
        return Err(format!(
            "Semantics {:?}: {:?}",
            semantic.status,
            semantic.diagnostics.first()
        ));
    }
    progress("3 / 6  Checking the explicit P0 planar profile")?;
    let planar = planar::check(
        &semantic,
        planar::Quality::planar_fixture(),
        planar::Limits::fixtures(),
    );
    if planar.status != planar::Status::Checked {
        return Err(format!(
            "Planar {:?}: {:?}",
            planar.status,
            planar.diagnostics.first()
        ));
    }
    progress("4 / 6  Constructing the Rust BRep")?;
    let candidate = solidcore_step_native::construct_planar(
        &planar,
        Options {
            operations: 100_000,
            work_tolerance_mm: 1e-6,
        },
    )
    .map_err(|e| e.to_string())?;
    progress("5 / 6  Running native pointer and deep checks")?;
    let candidate = candidate.validate_native().map_err(|e| e.to_string())?;
    progress("6 / 6  Triangulating the native planar trims for display")?;
    from_candidate(candidate)
}

pub(crate) fn from_candidate(owner: Candidate) -> Result<Scene> {
    if owner.native_checks().is_none() {
        return Err("Native validation must finish before display extraction".into());
    }
    let role = owner
        .source()
        .part_definition()
        .map_or_else(|| "Unestablished".into(), |p| p.shape_role.clone());
    let brep = owner.brep();
    let mut triangles = Vec::new();
    let mut lines = Vec::new();
    let mut bounds = Vec::new();
    let mut face_ranges = Vec::new();
    let mut edge_ranges = Vec::new();
    for id in owner.edges().values() {
        let edge = brep.edge(*id).map_err(|e| e.to_string())?;
        let curve = edge.curve.as_ref().ok_or("Missing native defining curve")?;
        let CurveKind::Line(line) = curve.kind() else {
            return Err("Display currently requires native LINE edges".into());
        };
        let a = native_point(line.evaluate_point(edge.interval.min()))?;
        let b = native_point(line.evaluate_point(edge.interval.max()))?;
        let start = lines.len();
        lines.extend([a, b]);
        edge_ranges.push((*id, start..lines.len()));
        bounds.extend([a, b]);
    }
    for face_id in owner.faces().values() {
        let face = brep.face(*face_id).map_err(|e| e.to_string())?;
        let surface = face.surface.as_ref().ok_or("Missing native face surface")?;
        let SurfaceKind::Plane(plane) = surface.kind() else {
            return Err("Display currently requires native PLANE faces".into());
        };
        let mut coordinates = Vec::new();
        let mut points = Vec::new();
        let mut holes = Vec::new();
        for (loop_index, loop_id) in face.loops.iter().enumerate() {
            if loop_index > 0 {
                holes.push(points.len());
            }
            let loop_ = brep.loop_(*loop_id).map_err(|e| e.to_string())?;
            if loop_.edges.len() < 3 {
                return Err(
                    "Unsupported native planar boundary with fewer than three sides".into(),
                );
            }
            for occurrence in &loop_.edges {
                if points.len() >= 100_000 {
                    return Err("Display boundary vertex budget exceeded".into());
                }
                let edge = brep.edge(occurrence.edge).map_err(|e| e.to_string())?;
                let mut trim = None;
                for use_id in &edge.edgeuses {
                    if eu::face(brep, *use_id).map_err(|e| e.to_string())? == Some(*face_id)
                        && eu::orientation(brep, *use_id).map_err(|e| e.to_string())?
                            == OrientType::Same
                    {
                        if trim.is_some() {
                            return Err("Repeated face incidence needs seam rendering".into());
                        }
                        trim = brep
                            .edgeuse(*use_id)
                            .map_err(|e| e.to_string())?
                            .uv_trim_curve
                            .as_ref();
                    }
                }
                let CurveKind::Line(uv) = trim.ok_or("Missing actual native UV trim")?.kind()
                else {
                    return Err("Display currently requires affine native UV trims".into());
                };
                let parameter = if occurrence.forward {
                    edge.interval.min()
                } else {
                    edge.interval.max()
                };
                let value = uv.evaluate_point(parameter);
                if !value.x.is_finite() || !value.y.is_finite() {
                    return Err("Nonfinite native UV trim".into());
                }
                coordinates.extend([value.x, value.y]);
                // Fill the native receiver through its actual UV trims, rather
                // than source vertices or a fitted replacement for the face.
                let point = native_point(plane.evaluate_point(Vec2::new(value.x, value.y)))?;
                points.push(point);
                bounds.push(point);
            }
        }
        let indices = earcutr::earcut(&coordinates, &holes, 2)
            .map_err(|e| format!("Display triangulation: {e:?}"))?;
        if indices.is_empty() || indices.len() % 3 != 0 {
            return Err("Native planar face produced no complete display triangles".into());
        }
        let normal = native_point(plane.position().x_axis().cross(plane.position().y_axis()))?
            .normalize_or_zero();
        let light = normal.dot(DVec3::new(0.3, -0.4, 0.85).normalize()).abs() as f32;
        let color = [
            0.23 + 0.18 * light,
            0.49 + 0.22 * light,
            0.68 + 0.22 * light,
        ];
        let start = triangles.len();
        for index in indices {
            triangles.push((
                *points
                    .get(index)
                    .ok_or("Display triangle index out of range")?,
                color,
            ));
        }
        face_ranges.push((*face_id, start..triangles.len()));
    }
    let first = *bounds
        .first()
        .ok_or("Native candidate contains no drawable geometry")?;
    let (minimum, maximum) = bounds
        .iter()
        .fold((first, first), |(lo, hi), p| (lo.min(*p), hi.max(*p)));
    // Center and normalize in f64 before GPU narrowing. Source/native geometry
    // and physical units remain unchanged in the retained owner.
    let center_mm = minimum * 0.5 + maximum * 0.5;
    let radius_mm = bounds
        .iter()
        .map(|p| (*p - center_mm).length())
        .fold(0.0_f64, f64::max);
    if !radius_mm.is_finite() || radius_mm <= 0. {
        return Err("Degenerate or overflowing native display bounds".into());
    }
    let vertex = |position: DVec3, color: [f32; 3]| Vertex {
        position: ((position - center_mm) / radius_mm).as_vec3().to_array(),
        color,
    };
    let mut vertices = Vec::new();
    for id in owner.vertices().values() {
        let point = native_point(owner.brep().vertex(*id).map_err(|e| e.to_string())?.point)?;
        vertices.push((*id, vertex(point, [0.8, 0.8, 0.8])));
    }
    Ok(Scene {
        owner,
        center_mm,
        radius_mm,
        role,
        face_ranges,
        edge_ranges,
        vertices,
        triangles: triangles.into_iter().map(|(p, c)| vertex(p, c)).collect(),
        lines: lines
            .into_iter()
            .map(|p| vertex(p, [0.004, 0.008, 0.014]))
            .collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn public_plane_uses_real_native_owner_and_derived_geometry() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../solidcore-step/tests/fixtures/plane.step");
        let scene = load(&path, |_| Ok(())).unwrap();
        assert_eq!(scene.role, "OpenSurfaceModel");
        assert_eq!(scene.owner.faces().len(), 1);
        assert_eq!(scene.triangles.len(), 6);
        assert_eq!(scene.lines.len(), scene.owner.edges().len() * 2);
        assert!(scene
            .triangles
            .iter()
            .flat_map(|v| v.position)
            .all(f32::is_finite));
        // Public analytic truth: the rotated rectangle has 12 mm and 8 mm
        // sides. This checks displayed area, including physical normalization,
        // with a display-f32 allowance; it is never an acceptance budget.
        let mut area = 0.;
        let (triangles, _) = scene.triangles.as_chunks::<3>();
        for [a, b, c] in triangles {
            let point = |v: &Vertex| DVec3::from_array(v.position.map(f64::from));
            area += (point(b) - point(a)).cross(point(c) - point(a)).length()
                * 0.5
                * scene.radius_mm
                * scene.radius_mm;
        }
        assert!((area - 96.).abs() < 1e-4);
    }
    #[test]
    fn file_io_failure_never_creates_a_scene() {
        assert!(load(Path::new("absent-public-synthetic.step"), |_| Ok(())).is_err());
    }
    #[test]
    fn cancellation_before_the_next_stage_never_returns_partial_geometry() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../solidcore-step/tests/fixtures/plane.step");
        assert!(load(&path, |_| Err("cancelled".into())).is_err());
    }
}
