// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Camera in normalized display coordinates; never edits the native geometry.
use glam::{
    camera::rh::{proj::directx, view},
    Vec3,
};

pub(crate) struct Camera {
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: 0.8,
            pitch: 0.65,
            distance: 3.8,
        }
    }
}

impl Camera {
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
}
