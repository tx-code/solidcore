// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Camera in normalized display coordinates; never edits the native geometry.
use glam::{
    camera::rh::{proj::directx, view},
    DMat4, DVec2, DVec3, DVec4, Vec3,
};

#[derive(Clone, Copy)]
pub(crate) struct Frame {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
    pub pixels_per_point: f32,
    pub matrix: [f32; 16],
}
#[derive(Clone, Copy)]
pub(crate) struct Projection {
    pub pixel: DVec2,
    pub depth: f64,
    pub w: f64,
}

impl Frame {
    fn transform(self) -> DMat4 {
        DMat4::from_cols_array(&self.matrix.map(f64::from))
    }
    pub fn contains(self, p: DVec2) -> bool {
        p.x >= f64::from(self.x)
            && p.x <= f64::from(self.x + self.width)
            && p.y >= f64::from(self.y)
            && p.y <= f64::from(self.y + self.height)
    }
    fn projected(self, p: DVec4) -> Option<Projection> {
        if !p.is_finite() || p.w <= 0. {
            return None;
        }
        let ndc = p.truncate() / p.w;
        // This is only a display-coordinate allowance after homogeneous
        // clipping, never a source/native acceptance or working tolerance.
        if !ndc.is_finite() || !(-1e-10..=1. + 1e-10).contains(&ndc.z) {
            return None;
        }
        Some(Projection {
            pixel: DVec2::new(
                f64::from(self.x) + (ndc.x + 1.) * 0.5 * f64::from(self.width),
                f64::from(self.y) + (1. - ndc.y) * 0.5 * f64::from(self.height),
            ),
            depth: ndc.z.clamp(0., 1.),
            w: p.w,
        })
    }
    pub fn project(self, normalized: DVec3) -> Option<Projection> {
        self.projected(self.transform() * normalized.extend(1.))
    }
    pub fn ray(self, p: DVec2) -> Option<(DVec3, DVec3)> {
        if !self.contains(p) {
            return None;
        }
        let inverse = self.transform().inverse();
        let x = 2. * (p.x - f64::from(self.x)) / f64::from(self.width) - 1.;
        let y = 1. - 2. * (p.y - f64::from(self.y)) / f64::from(self.height);
        let a = inverse * DVec4::new(x, y, 0., 1.);
        let b = inverse * DVec4::new(x, y, 1., 1.);
        if !a.is_finite() || !b.is_finite() || a.w == 0. || b.w == 0. {
            return None;
        }
        let origin = a.truncate() / a.w;
        let end = b.truncate() / b.w;
        let direction = (end - origin).normalize_or_zero();
        if direction.length_squared() == 0. {
            None
        } else {
            Some((origin, direction))
        }
    }
    pub fn segment(self, a: DVec3, b: DVec3) -> Option<(Projection, Projection, f64, f64)> {
        let a = self.transform() * a.extend(1.);
        let b = self.transform() * b.extend(1.);
        let mut low = 0.0_f64;
        let mut high = 1.0_f64;
        // Clip the original native LINE interval, retaining its parameter
        // fractions for perspective-correct native curve evaluation.
        for (x, y) in [
            (a.w + a.x, b.w + b.x),
            (a.w - a.x, b.w - b.x),
            (a.w + a.y, b.w + b.y),
            (a.w - a.y, b.w - b.y),
            (a.z, b.z),
            (a.w - a.z, b.w - b.z),
        ] {
            if x < 0. && y < 0. {
                return None;
            }
            if x < 0. {
                low = low.max(x / (x - y));
            }
            if y < 0. {
                high = high.min(x / (x - y));
            }
        }
        if low > high {
            return None;
        }
        Some((
            self.projected(a.lerp(b, low))?,
            self.projected(a.lerp(b, high))?,
            low,
            high,
        ))
    }
}

pub(crate) struct Camera {
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: 0.8,
            // A lower oblique start keeps tapered sidewalls visible rather
            // than looking almost along a common 45-degree cone generator.
            pitch: 0.45,
            distance: 3.8,
        }
    }
}

impl Camera {
    pub fn frame(&self, viewport: egui::Rect, pixels_per_point: f32, size: [u32; 2]) -> Frame {
        let [width, height] = size;
        let width = width.max(1) as f32;
        let height = height.max(1) as f32;
        let x = (viewport.left() * pixels_per_point).clamp(0., width - 1.);
        let y = (viewport.top() * pixels_per_point).clamp(0., height - 1.);
        let width = (viewport.width() * pixels_per_point).clamp(1., width - x);
        let height = (viewport.height() * pixels_per_point).clamp(1., height - y);
        Frame {
            x,
            y,
            width,
            height,
            pixels_per_point,
            matrix: self.matrix(width / height),
        }
    }
    pub fn fit(&mut self) {
        *self = Self::default();
    }
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw = (self.yaw - dx * 0.008).rem_euclid(std::f32::consts::TAU);
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.52, 1.52);
    }
    pub fn zoom(&mut self, steps: f32) {
        self.distance = (self.distance * (-steps.clamp(-20., 20.) * 0.12).exp()).clamp(0.02, 1000.);
    }
    pub fn matrix(&self, aspect: f32) -> [f32; 16] {
        let eye = Vec3::new(
            self.yaw.cos() * self.pitch.cos(),
            self.yaw.sin() * self.pitch.cos(),
            self.pitch.sin(),
        ) * self.distance;
        // Unit-radius display bounds permit scale-independent clipping. Refit
        // near/far after every zoom, preserving useful depth resolution.
        let near = (self.distance - 1.5)
            .max(self.distance * 0.001)
            .max(0.00001);
        let far = (self.distance + 2.).max(near * 2.);
        (directx::perspective(45_f32.to_radians(), aspect.max(0.01), near, far)
            * view::look_at_mat4(eye, Vec3::ZERO, Vec3::Z))
        .to_cols_array()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn camera_extreme_navigation_keeps_finite_projection() {
        let mut camera = Camera::default();
        for delta in [-100_000., 100_000., 0.] {
            camera.zoom(delta);
            camera.orbit(delta, delta);
            assert!(camera.matrix(0.2).into_iter().all(f32::is_finite));
        }
        camera.fit();
        assert!(camera.matrix(2.).into_iter().all(f32::is_finite));
    }
    #[test]
    fn render_frame_projection_dpi_ray_and_near_clip_agree() {
        for scale in [1., 1.5, 2.] {
            let frame = Camera::default().frame(
                egui::Rect::from_min_max(egui::pos2(300., 0.), egui::pos2(1180., 800.)),
                scale,
                [(1180. * scale) as u32, (800. * scale) as u32],
            );
            let center = frame.project(DVec3::ZERO).unwrap();
            let (origin, direction) = frame.ray(center.pixel).unwrap();
            assert!((origin - direction * origin.dot(direction)).length() < 1e-10);
            let (a, b, low, high) = frame
                .segment(origin - direction, origin + direction)
                .unwrap();
            assert!(low > 0. && low < 1. && high <= 1.);
            assert!(a.depth >= 0. && b.depth <= 1.);
            assert!(a.pixel.distance(center.pixel) < 1e-8);
            assert!(frame.ray(DVec2::new(0., 0.)).is_none());
        }
    }

    #[test]
    fn default_and_fit_show_public_conical_band_thickness_and_its_actual_native_side() {
        use crate::scene::public_controls::scene;
        use solidcore::{math::Vec3 as NativePoint, EntityRef};

        const BAND: &[u8] =
            include_bytes!("../../solidcore-step-native/tests/fixtures/cone-band.step");
        let mut scene = scene(BAND);
        let side_record = scene
            .owner
            .source()
            .source()
            .records()
            .iter()
            .position(|record| record.id == "182")
            .unwrap();
        let side = *scene.owner.faces().get(&side_record).unwrap();
        let mut camera = Camera::default();
        // Independent source truth is r(z)=2+z for 0<=z<=4 mm. On its
        // near-side 45-degree meridian, these are the bottom/top boundaries
        // and an interior point, not samples copied from the display mesh.
        let sqrt_two = 2_f64.sqrt();
        let bottom = DVec3::new(sqrt_two, sqrt_two, 0.);
        let top = DVec3::new(3. * sqrt_two, 3. * sqrt_two, 4.);
        let interior = DVec3::new(2. * sqrt_two, 2. * sqrt_two, 2.);
        for after_fit in [false, true] {
            if after_fit {
                camera.orbit(110., -60.);
                camera.zoom(4.);
                camera.fit();
            }
            for scale in [1., 2.] {
                let frame = camera.frame(
                    egui::Rect::from_min_max(egui::pos2(300., 0.), egui::pos2(1180., 800.)),
                    scale,
                    [(1180. * scale) as u32, (800. * scale) as u32],
                );
                let normalized = |point| (point - scene.center_mm) / scene.radius_mm;
                let bottom_pixel = frame.project(normalized(bottom)).unwrap().pixel;
                let top_pixel = frame.project(normalized(top)).unwrap().pixel;
                assert!(frame.contains(bottom_pixel) && frame.contains(top_pixel));
                // This is a visibility criterion in screen space, independent
                // of native geometric precision. The old near-generator view
                // collapsed the entire side to less than this 2% height.
                assert!(bottom_pixel.y - top_pixel.y >= 0.02 * f64::from(frame.height));

                let pixel = frame.project(normalized(interior)).unwrap().pixel;
                let (origin, direction) = frame.ray(pixel).unwrap();
                let origin_mm = scene.center_mm + origin * scene.radius_mm;
                let native = |point: DVec3| NativePoint::new(point.x, point.y, point.z);
                let hit = scene
                    .owner
                    .ray_hit(native(origin_mm), native(direction), 1e-6, None)
                    .unwrap()
                    .unwrap();
                // A cap occluding this pixel would fail the face and point
                // truth, even if a misleading projected outline looked wide.
                assert_eq!(hit.entity, EntityRef::Face(side));
                assert!(hit.point_mm.distance(native(interior)) < 1e-6);
            }
        }
        assert_eq!(scene.owner.source().source().bytes(), BAND);
    }
}
