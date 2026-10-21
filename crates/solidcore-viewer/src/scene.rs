// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native owner plus bounded derived display geometry; no import acceptance claim.
mod mesh;

use glam::DVec3;
use solidcore::topology::{EdgeId, FaceId, VertexId};
use solidcore_step_native::Candidate;
use std::ops::Range;
#[cfg(test)]
use std::path::Path;

pub(crate) type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

/// Display allowances are separate from all import/repair/query budgets. Samples
/// screen chord and normal variation but do not establish continuous CAD error.
#[derive(Clone, Copy, Debug)]
pub(crate) struct DisplayOptions {
    pub chord_mm: Option<f64>,
    pub normal_angle_radians: f64,
    pub uv_chord_fraction: f64,
    pub max_depth: u32,
    pub max_vertices: usize,
    pub max_triangles: usize,
    pub max_evaluations: usize,
}
impl Default for DisplayOptions {
    fn default() -> Self {
        Self {
            chord_mm: None,
            normal_angle_radians: 0.2,
            uv_chord_fraction: 0.001,
            max_depth: 8,
            max_vertices: 750_000,
            max_triangles: 250_000,
            max_evaluations: 5_000_000,
        }
    }
}
impl DisplayOptions {
    fn validate(self) -> Result<()> {
        if self.chord_mm.is_some_and(|v| !v.is_finite() || v <= 0.)
            || !self.normal_angle_radians.is_finite()
            || self.normal_angle_radians <= 0.
            || self.normal_angle_radians >= std::f64::consts::PI
            || !self.uv_chord_fraction.is_finite()
            || !(0. ..0.1).contains(&self.uv_chord_fraction)
            || self.max_depth > 12
            || self.max_vertices < 3
            || self.max_triangles == 0
            || self.max_evaluations == 0
        {
            return Err("Invalid or exhausted display options".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct EdgeSample {
    pub parameter: f64,
    pub point_mm: DVec3,
}

pub(crate) struct Scene {
    // This owner, source identities and exact definitions survive all display work.
    pub owner: Candidate,
    pub triangles: Vec<Vertex>,
    pub lines: Vec<Vertex>,
    pub center_mm: DVec3,
    pub radius_mm: f64,
    pub role: String,
    pub face_ranges: Vec<(FaceId, Range<usize>)>,
    pub edge_ranges: Vec<(EdgeId, Range<usize>)>,
    pub vertices: Vec<(VertexId, Vertex)>,
    pub edge_samples: Vec<(EdgeId, Vec<EdgeSample>)>,
    pub display_tolerance_mm: f64,
}

fn native_point(value: solidcore::math::Vec3) -> Result<DVec3> {
    let point = DVec3::new(value.x, value.y, value.z);
    if point.is_finite() {
        Ok(point)
    } else {
        Err("Nonfinite native display coordinate".into())
    }
}

#[cfg(test)]
pub(crate) fn load(path: &Path, progress: impl Fn(&str) -> Result<()>) -> Result<Scene> {
    crate::loading::load(path, &crate::loading::LoadOptions::default(), progress)
}
#[cfg(test)]
pub(crate) fn from_bytes(bytes: Vec<u8>, progress: impl Fn(&str) -> Result<()>) -> Result<Scene> {
    crate::loading::from_bytes(bytes, &crate::loading::LoadOptions::default(), progress)
}
#[cfg(test)]
pub(crate) fn from_candidate(owner: Candidate) -> Result<Scene> {
    from_candidate_with_options(owner, DisplayOptions::default(), |_| Ok(()))
}

pub(crate) fn from_candidate_with_options(
    owner: Candidate,
    options: DisplayOptions,
    progress: impl Fn(&str) -> Result<()>,
) -> Result<Scene> {
    if owner.native_checks().is_none() {
        return Err("Native validation must finish before display extraction".into());
    }
    let role = owner
        .source()
        .part_definition()
        .map_or_else(|| "Unestablished".into(), |p| p.shape_role.clone());
    let extracted = mesh::extract(&owner, options, &progress)?;
    let triangle_vertices = extracted.triangles.iter().try_fold(0usize, |sum, mesh| {
        sum.checked_add(mesh.vertices.len())
            .ok_or("Display output count overflow")
    })?;
    let line_vertices = extracted
        .edges
        .iter()
        .try_fold(0usize, |sum, (_, samples)| {
            samples
                .len()
                .saturating_sub(1)
                .checked_mul(2)
                .and_then(|count| sum.checked_add(count))
                .ok_or("Display output count overflow")
        })?;
    let output_count = triangle_vertices
        .checked_add(line_vertices)
        .and_then(|count| count.checked_add(owner.brep().vertex_list().len()))
        .ok_or("Display output count overflow")?;
    // Preflight before expanding GPU/bounds buffers. This is a display-element
    // ceiling, not a claimed RSS bound: the native owner and bounded extraction
    // scratch coexist until publication, and cancellation still drops all of it.
    if output_count > options.max_vertices {
        return Err("Display output vertex budget exceeded".into());
    }
    let mut bounds = Vec::new();
    for mesh in &extracted.triangles {
        bounds.extend(mesh.vertices.iter().map(|(point, _)| *point));
    }
    for (_, edge) in &extracted.edges {
        bounds.extend(edge.iter().map(|sample| sample.point_mm));
    }
    let first = *bounds
        .first()
        .ok_or("Native candidate contains no drawable geometry")?;
    let (minimum, maximum) = bounds
        .iter()
        .fold((first, first), |(lo, hi), p| (lo.min(*p), hi.max(*p)));
    let center_mm = minimum * 0.5 + maximum * 0.5;
    let radius_mm = bounds
        .iter()
        .map(|p| (*p - center_mm).length())
        .fold(0_f64, f64::max);
    if !center_mm.is_finite() || !radius_mm.is_finite() || radius_mm <= 0. {
        return Err("Degenerate or overflowing native display bounds".into());
    }
    let vertex = |position: DVec3, color: [f32; 3]| -> Result<Vertex> {
        let normalized = ((position - center_mm) / radius_mm).as_vec3();
        if !normalized.is_finite() {
            return Err("Nonfinite normalized display vertex".into());
        }
        Ok(Vertex {
            position: normalized.to_array(),
            color,
        })
    };
    let mut triangles = Vec::new();
    let mut lines = Vec::new();
    let mut face_ranges = Vec::new();
    let mut edge_ranges = Vec::new();
    for mesh in extracted.triangles {
        let begin = triangles.len();
        for (point, color) in mesh.vertices {
            triangles.push(vertex(point, color)?);
        }
        face_ranges.push((mesh.face, begin..triangles.len()));
    }
    for (id, samples) in &extracted.edges {
        let begin = lines.len();
        for pair in samples.windows(2) {
            let [a, b] = pair else {
                return Err("Incomplete display edge segment".into());
            };
            lines.extend([
                vertex(a.point_mm, [0.004, 0.008, 0.014])?,
                vertex(b.point_mm, [0.004, 0.008, 0.014])?,
            ]);
        }
        edge_ranges.push((*id, begin..lines.len()));
    }
    let mut vertices = Vec::new();
    for id in owner.brep().vertex_list() {
        let point = native_point(owner.brep().vertex(*id).map_err(|e| e.to_string())?.point)?;
        vertices.push((*id, vertex(point, [0.8, 0.8, 0.8])?));
    }
    let count = triangles
        .len()
        .checked_add(lines.len())
        .and_then(|n| n.checked_add(vertices.len()))
        .ok_or("Display output count overflow")?;
    if count > options.max_vertices {
        return Err("Display output vertex budget exceeded".into());
    }
    // A cancelled extraction cannot publish even after its last geometry callback.
    progress("6 / 6  Native display buffers ready")?;
    Ok(Scene {
        owner,
        triangles,
        lines,
        center_mm,
        radius_mm,
        role,
        face_ranges,
        edge_ranges,
        vertices,
        edge_samples: extracted.edges,
        display_tolerance_mm: extracted.chord_mm,
    })
}

#[cfg(test)]
pub(crate) mod public_controls {
    use super::*;
    use solidcore_step::{inspect, planar::Rational, semantics as s, Limits};
    use solidcore_step_native::{construct_general, GeneralQuality, Options};

    pub const CYLINDER: &[u8] =
        include_bytes!("../../solidcore-step-native/tests/fixtures/cylinder-phase-rings.step");
    pub const CONE: &[u8] = include_bytes!("../../solidcore-step/tests/fixtures/cone-pole.step");

    pub fn candidate(bytes: &[u8]) -> Candidate {
        let raw = inspect(bytes.to_vec(), Limits::planar_fixtures());
        let quality = GeneralQuality {
            id: "Q-Viewer-public-analytic".into(),
            definition_mm: Rational::decimal("0.0000001").unwrap(),
            consistency_mm: Rational::decimal("0.000001").unwrap(),
            derived_uv_mm: Rational::decimal("0.000001").unwrap(),
            query_mm: Rational::decimal("0.000001").unwrap(),
            maximum_work_tolerance_mm: Rational::decimal("0.00001").unwrap(),
            maximum_tolerance_growth_mm: Rational::ZERO,
        };
        construct_general(
            &raw,
            s::Limits::fixtures(),
            &s::Options::default(),
            Options { operations: 100_000, work_tolerance_mm: 1e-6 },
            quality,
        )
        .unwrap()
        .validate_native_detailed()
        .unwrap_or_else(|failure| {
            panic!("public viewer native validation: stage={:?}, completed={}, error={:?}, reports={:#?}", failure.stage, failure.completed, failure.error, failure.reports)
        })
        .measure_general_trims()
        .unwrap()
    }

    pub fn scene(bytes: &[u8]) -> Scene {
        from_candidate(candidate(bytes)).unwrap()
    }

    pub fn world(scene: &Scene, vertex: &Vertex) -> DVec3 {
        scene.center_mm + DVec3::from_array(vertex.position.map(f64::from)) * scene.radius_mm
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use public_controls::{candidate, scene, world, CONE, CYLINDER};
    use std::collections::BTreeMap;

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

    // Vertex-bit equality deliberately checks the published GPU buffer: matching
    // analytic positions within epsilon would conceal an actual display crack.
    fn closed_conforming(scene: &Scene) {
        let mut edges = BTreeMap::<([u32; 3], [u32; 3]), usize>::new();
        let (triangles, remainder) = scene.triangles.as_chunks::<3>();
        assert!(remainder.is_empty());
        for (index, [a, b, c]) in triangles.iter().enumerate() {
            let bits = |v: &Vertex| v.position.map(|x| if x == 0. { 0 } else { x.to_bits() });
            let [a, b, c] = [bits(a), bits(b), bits(c)];
            assert!(
                a != b && b != c && c != a,
                "collapsed display triangle index={index}, bits={:?}, face={:?}",
                [a, b, c],
                scene
                    .face_ranges
                    .iter()
                    .find(|(_, range)| range.contains(&(3 * index)))
            );
            for (a, b) in [(a, b), (b, c), (c, a)] {
                *edges
                    .entry(if a < b { (a, b) } else { (b, a) })
                    .or_default() += 1;
            }
        }
        assert!(!edges.is_empty());
        let bad = edges
            .iter()
            .filter(|(_, count)| **count != 2)
            .collect::<Vec<_>>();
        assert!(
            bad.is_empty(),
            "nonconforming display mesh: {} edges, all={:?}",
            bad.len(),
            bad
        );
    }

    fn bounds(scene: &Scene) -> (DVec3, DVec3) {
        scene.triangles.iter().map(|v| world(scene, v)).fold(
            (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY)),
            |(lo, hi), p| (lo.min(p), hi.max(p)),
        )
    }
    #[test]
    fn public_plane_uses_real_native_owner_and_derived_geometry() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../solidcore-step/tests/fixtures/plane.step");
        let scene = load(&path, |_| Ok(())).unwrap();
        assert_eq!(scene.role, "OpenSurfaceModel");
        assert_eq!(scene.owner.faces().len(), 1);
        // Exact native trim endpoints can retain additional collinear boundary
        // nodes. Their triangulation count is not the geometric contract.
        assert!(scene.triangles.len() >= 6 && scene.triangles.len().is_multiple_of(3));
        assert!(scene.lines.len() >= scene.owner.brep().edge_list().len() * 2);
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

    #[test]
    fn public_periodic_cylinder_is_curved_conforming_and_preserves_every_native_piece() {
        let scene = scene(CYLINDER);
        assert_eq!(scene.owner.source().source().bytes(), CYLINDER);
        assert_eq!(scene.role, "MaterialSolid");
        assert_eq!(
            (
                scene.vertices.len(),
                scene.edge_ranges.len(),
                scene.face_ranges.len()
            ),
            (3, 4, 3)
        );
        let upper = scene.owner.edges().get(&record(&scene, "131")).unwrap();
        assert_eq!(upper.len(), 2);
        assert!(upper.iter().all(|piece| scene
            .edge_ranges
            .iter()
            .any(|(id, range)| id == piece && !range.is_empty())));
        let report = scene.owner.general_report().unwrap();
        assert_eq!(report.final_trim_measurements.unwrap().edge_uses, 8);
        assert!(report.material_measurements.is_none());
        assert_eq!(report.chart_cuts.len(), 1);
        assert!(scene
            .edge_ranges
            .iter()
            .any(|(id, range)| *id == report.chart_cuts[0].1 && !range.is_empty()));
        let side = scene.owner.faces().get(&record(&scene, "182")).unwrap();
        let range = &scene
            .face_ranges
            .iter()
            .find(|(id, _)| id == side)
            .unwrap()
            .1;
        assert!(
            range.len() > 30,
            "a cylinder requires interior surface subdivision"
        );
        for v in scene.triangles.get(range.clone()).unwrap() {
            let p = world(&scene, v);
            assert!((p.x * p.x + p.y * p.y - 9.).abs() < 2e-5);
            assert!((-1e-5..=7. + 1e-5).contains(&p.z));
        }
        let (lo, hi) = bounds(&scene);
        assert!(lo.distance(DVec3::new(-3., -3., 0.)) < 1e-5);
        assert!(hi.distance(DVec3::new(3., 3., 7.)) < 1e-5);
        closed_conforming(&scene);
    }

    #[test]
    fn public_cone_pole_uses_exact_surface_positions_and_a_closed_display_chart() {
        let scene = scene(CONE);
        assert_eq!(scene.owner.source().source().bytes(), CONE);
        assert_eq!(
            (
                scene.vertices.len(),
                scene.edge_ranges.len(),
                scene.face_ranges.len()
            ),
            (2, 2, 2)
        );
        assert_eq!(
            scene
                .owner
                .general_report()
                .unwrap()
                .final_trim_measurements
                .unwrap()
                .edge_uses,
            4
        );
        let side = scene.owner.faces().get(&record(&scene, "182")).unwrap();
        let range = &scene
            .face_ranges
            .iter()
            .find(|(id, _)| id == side)
            .unwrap()
            .1;
        assert!(range.len() > 30);
        for v in scene.triangles.get(range.clone()).unwrap() {
            let p = world(&scene, v);
            assert!((p.x * p.x + p.y * p.y - p.z * p.z).abs() < 2e-5);
            assert!(v.color.iter().all(|c| c.is_finite()));
        }
        closed_conforming(&scene);
    }

    #[test]
    fn rotated_translated_cylinder_and_set_order_keep_analytic_display_truth() {
        let text = std::str::from_utf8(CYLINDER)
            .unwrap()
            .replace(
                "#100=CARTESIAN_POINT('',(0.,0.,0.));",
                "#100=CARTESIAN_POINT('',(11.,13.,17.));",
            )
            .replace(
                "#101=CARTESIAN_POINT('',(0.,0.,7.));",
                "#101=CARTESIAN_POINT('',(11.,6.,17.));",
            )
            .replace(
                "#102=CARTESIAN_POINT('',(0.,3.,0.));",
                "#102=CARTESIAN_POINT('',(11.,13.,20.));",
            )
            .replace(
                "#103=CARTESIAN_POINT('',(3.,0.,7.));",
                "#103=CARTESIAN_POINT('',(14.,6.,17.));",
            )
            .replace(
                "#104=DIRECTION('',(0.,0.,1.));",
                "#104=DIRECTION('',(0.,-1.,0.));",
            )
            .replace(
                "#182=ADVANCED_FACE('',(#172,#173),#142,.T.);",
                "#182=ADVANCED_FACE('',(#173,#172),#142,.T.);",
            );
        let scene = scene(text.as_bytes());
        assert_eq!(scene.owner.source().source().bytes(), text.as_bytes());
        let (lo, hi) = bounds(&scene);
        assert!(lo.distance(DVec3::new(8., 6., 14.)) < 1e-5, "{lo:?}");
        assert!(hi.distance(DVec3::new(14., 13., 20.)) < 1e-5, "{hi:?}");
        closed_conforming(&scene);
    }

    #[test]
    fn display_limits_and_cancellation_fail_without_a_published_partial_scene() {
        for options in [
            DisplayOptions {
                max_evaluations: 1,
                ..Default::default()
            },
            DisplayOptions {
                max_triangles: 1,
                ..Default::default()
            },
            DisplayOptions {
                max_vertices: 3,
                ..Default::default()
            },
            DisplayOptions {
                max_depth: 0,
                ..Default::default()
            },
            DisplayOptions {
                chord_mm: Some(f64::NAN),
                ..Default::default()
            },
        ] {
            assert!(from_candidate_with_options(candidate(CONE), options, |_| Ok(())).is_err());
        }
        let cancelled = std::cell::Cell::new(false);
        let result =
            from_candidate_with_options(candidate(CONE), DisplayOptions::default(), |stage| {
                if stage.contains("Refining conforming") {
                    cancelled.set(true);
                    Err("public display cancellation".into())
                } else {
                    Ok(())
                }
            });
        assert!(cancelled.get());
        assert_eq!(result.err().unwrap(), "public display cancellation");
        let replay = scene(CONE);
        assert_eq!(replay.owner.source().source().bytes(), CONE);
        assert_eq!(
            replay
                .owner
                .general_report()
                .unwrap()
                .final_trim_measurements
                .unwrap()
                .edge_uses,
            4
        );
    }
}
