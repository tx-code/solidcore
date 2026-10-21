// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! wgpu 30 rendering and egui overlay, using only its safe surface interface.
use crate::{
    camera::Camera,
    capture,
    picking::Selection,
    scene::{Result, Scene, Vertex},
    UserEvent,
};
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::{event_loop::EventLoopProxy, window::Window};

pub(crate) struct Gpu {
    instance: wgpu::Instance,
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth: wgpu::TextureView,
    faces: wgpu::RenderPipeline,
    edges: wgpu::RenderPipeline,
    points: wgpu::RenderPipeline,
    uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    triangles: Option<wgpu::Buffer>,
    lines: Option<wgpu::Buffer>,
    markers: Option<wgpu::Buffer>,
    triangle_count: u32,
    line_count: u32,
    marker_count: u32,
    gui: egui_wgpu::Renderer,
    repaint: EventLoopProxy<UserEvent>,
    screenshot: Option<capture::Request>,
    readback: Option<(u64, std::thread::JoinHandle<Result<std::path::PathBuf>>)>,
    pub adapter: String,
}

impl Gpu {
    pub async fn new(window: Arc<Window>, proxy: EventLoopProxy<UserEvent>) -> Result<Self> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_with_display_handle(
            Box::new(window.clone()),
        ));
        let surface = instance
            .create_surface(window.clone())
            .map_err(|e| e.to_string())?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
                apply_limit_buckets: false,
            })
            .await
            .map_err(|e| e.to_string())?;
        let info = adapter.get_info();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("solidcore viewer"),
                required_limits: wgpu::Limits::downlevel_defaults(),
                ..Default::default()
            })
            .await
            .map_err(|e| e.to_string())?;
        let error_proxy = proxy.clone();
        device.on_uncaptured_error(Arc::new(move |error: wgpu::Error| {
            let _ = error_proxy.send_event(UserEvent::GpuError(error.to_string()));
        }));
        let size = window.inner_size();
        let mut config = surface
            .get_default_config(&adapter, size.width.max(1), size.height.max(1))
            .ok_or("No compatible presentation configuration")?;
        config.present_mode = wgpu::PresentMode::AutoVsync;
        if surface
            .get_capabilities(&adapter)
            .usages
            .contains(wgpu::TextureUsages::COPY_SRC)
        {
            config.usage |= wgpu::TextureUsages::COPY_SRC;
        }
        surface.configure(&device, &config);
        let depth = depth(&device, &config);
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("native planar display"),
            source: wgpu::ShaderSource::Wgsl(include_str!("geometry.wgsl").into()),
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("camera"),
            size: 80,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bindings = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera"),
            layout: &bindings,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("native display layout"),
            bind_group_layouts: &[Some(&bindings)],
            immediate_size: 0,
        });
        let faces = pipeline(&device, &shader, &layout, config.format, Pass::Faces);
        let edges = pipeline(&device, &shader, &layout, config.format, Pass::Edges);
        let points = pipeline(&device, &shader, &layout, config.format, Pass::Points);
        let gui = egui_wgpu::Renderer::new(
            &device,
            config.format,
            egui_wgpu::RendererOptions::default(),
        );
        Ok(Self {
            instance,
            window,
            surface,
            device,
            queue,
            config,
            depth,
            faces,
            edges,
            points,
            uniform,
            bind_group,
            triangles: None,
            lines: None,
            markers: None,
            triangle_count: 0,
            line_count: 0,
            marker_count: 0,
            gui,
            repaint: proxy,
            screenshot: None,
            readback: None,
            adapter: format!("{} · {:?}", info.name, info.backend),
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
        self.depth = depth(&self.device, &self.config);
    }

    pub fn clear(&mut self) {
        self.triangles = None;
        self.lines = None;
        self.triangle_count = 0;
        self.line_count = 0;
        self.markers = None;
        self.marker_count = 0;
        self.cancel_screenshot();
    }

    pub fn request_screenshot(&mut self, request: capture::Request) -> Result<()> {
        if !self.config.usage.contains(wgpu::TextureUsages::COPY_SRC) {
            return Err(
                "GPU screenshot unsupported: the presentation surface lacks COPY_SRC capability"
                    .into(),
            );
        }
        if !capture::supported(self.config.format) {
            return Err(format!(
                "GPU screenshot unsupported for surface format {:?}",
                self.config.format
            ));
        }
        request.check()?;
        self.screenshot = Some(request);
        Ok(())
    }

    pub fn cancel_screenshot(&mut self) {
        if let Some(request) = self.screenshot.take() {
            request.cancel();
        }
    }

    pub fn take_capture_completion(&mut self) -> Option<(u64, Result<std::path::PathBuf>)> {
        if !self
            .readback
            .as_ref()
            .is_some_and(|(_, worker)| worker.is_finished())
        {
            return None;
        }
        let (generation, worker) = self.readback.take()?;
        // Never join a running thread on the UI. Joining a completed worker
        // retains its real failure status, including an unexpected thread exit,
        // instead of relying on an unacknowledged detached completion message.
        Some((
            generation,
            worker
                .join()
                .unwrap_or_else(|_| Err("Screenshot worker terminated unexpectedly".into())),
        ))
    }

    pub fn upload(&mut self, scene: &Scene) -> Result<()> {
        let triangles = encode(&scene.triangles);
        let lines = encode_edges(&scene.lines)?;
        if u64::try_from(triangles.len()).map_err(|e| e.to_string())?
            > self.device.limits().max_buffer_size
            || u64::try_from(lines.len()).map_err(|e| e.to_string())?
                > self.device.limits().max_buffer_size
        {
            return Err("Native display geometry exceeds GPU buffer capacity".into());
        }
        self.triangle_count = u32::try_from(scene.triangles.len()).map_err(|e| e.to_string())?;
        self.line_count = u32::try_from(scene.lines.len() / 2).map_err(|e| e.to_string())?;
        self.triangles = Some(
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("native faces"),
                    contents: &triangles,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                }),
        );
        self.lines = Some(
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("native defining edges"),
                    contents: &lines,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                }),
        );
        self.markers = None;
        self.marker_count = 0;
        Ok(())
    }

    pub fn highlight(&mut self, scene: &Scene, selection: &Selection) -> Result<()> {
        use solidcore::error::EntityRef;
        let color = |entity: EntityRef| {
            if selection.selected.contains(&entity) {
                Some([0.95, 0.46, 0.055])
            } else if selection.hovered() == Some(entity) {
                Some([0.16, 0.8, 0.52])
            } else {
                None
            }
        };
        let mut triangles = scene.triangles.clone();
        let mut lines = scene.lines.clone();
        for (id, range) in &scene.face_ranges {
            if let Some(color) = color(EntityRef::Face(*id)) {
                for vertex in triangles
                    .get_mut(range.clone())
                    .ok_or("Face highlight range")?
                {
                    vertex.color = color;
                }
            }
        }
        for (id, range) in &scene.edge_ranges {
            if let Some(color) = color(EntityRef::Edge(*id)) {
                for vertex in lines.get_mut(range.clone()).ok_or("Edge highlight range")? {
                    vertex.color = color;
                }
            }
        }
        if let Some(buffer) = &self.triangles {
            self.queue.write_buffer(buffer, 0, &encode(&triangles));
        }
        if let Some(buffer) = &self.lines {
            self.queue.write_buffer(buffer, 0, &encode_edges(&lines)?);
        }
        let markers: Vec<_> = scene
            .vertices
            .iter()
            .filter_map(|(id, vertex)| {
                color(EntityRef::Vertex(*id)).map(|color| Vertex {
                    position: vertex.position,
                    color,
                })
            })
            .collect();
        self.marker_count = u32::try_from(markers.len()).map_err(|e| e.to_string())?;
        self.markers = if markers.is_empty() {
            None
        } else {
            Some(
                self.device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("native selected vertices"),
                        contents: &encode(&markers),
                        usage: wgpu::BufferUsages::VERTEX,
                    }),
            )
        };
        Ok(())
    }

    pub fn render(
        &mut self,
        camera: &Camera,
        viewport: egui::Rect,
        pixels_per_point: f32,
        jobs: &[egui::ClippedPrimitive],
        textures: &mut egui::TexturesDelta,
    ) -> Result<()> {
        // egui deltas are consumed exactly once, even if presentation is
        // occluded or the surface must be recreated. Dropping a font upload
        // on a skipped frame would leave later text jobs without that atlas.
        // epaint 0.37's TexturesDelta::Drop asserts that handled commands have
        // been removed. Borrowed iteration applies the GPU updates but leaves
        // their ownership live, causing a first-frame debug panic on drop.
        for (id, deltas) in textures.set.drain() {
            for delta in deltas {
                self.gui
                    .update_texture(&self.device, &self.queue, id, &delta);
            }
        }
        let result = self.render_frame(camera, viewport, pixels_per_point, jobs);
        // Do this after every presentation outcome, including skipped or
        // failed frames. Once render_frame returns no pass still references
        // the commands, and epaint will not repeat them in the next output.
        for id in textures.free.drain() {
            self.gui.free_texture(&id);
        }
        result
    }

    fn render_frame(
        &mut self,
        camera: &Camera,
        viewport: egui::Rect,
        pixels_per_point: f32,
        jobs: &[egui::ClippedPrimitive],
    ) -> Result<()> {
        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Timeout => {
                let _ = self
                    .repaint
                    .send_event(UserEvent::Repaint(std::time::Duration::from_millis(100)));
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Occluded => return Ok(()),
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.resize(self.config.width, self.config.height);
                self.window.request_redraw();
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Lost => {
                self.surface = self
                    .instance
                    .create_surface(self.window.clone())
                    .map_err(|e| e.to_string())?;
                self.resize(self.config.width, self.config.height);
                self.window.request_redraw();
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Validation => {
                return Err("GPU surface validation failed".into())
            }
        };
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("viewer frame"),
            });
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [self.config.width, self.config.height],
            pixels_per_point,
        };
        let commands =
            self.gui
                .update_buffers(&self.device, &self.queue, &mut encoder, jobs, &screen);
        let frame_view = camera.frame(
            viewport,
            pixels_per_point,
            [self.config.width, self.config.height],
        );
        let uniform: Vec<u8> = frame_view
            .matrix
            .into_iter()
            .chain([
                frame_view.width,
                frame_view.height,
                (2. * pixels_per_point).clamp(2., 4.),
                0.,
            ])
            .flat_map(f32::to_ne_bytes)
            .collect();
        self.queue.write_buffer(&self.uniform, 0, &uniform);
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("native faces and edges"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.055,
                            g: 0.068,
                            b: 0.085,
                            a: 1.,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_viewport(
                frame_view.x,
                frame_view.y,
                frame_view.width,
                frame_view.height,
                0.,
                1.,
            );
            pass.set_bind_group(0, &self.bind_group, &[]);
            if let Some(buffer) = &self.triangles {
                pass.set_pipeline(&self.faces);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..self.triangle_count, 0..1);
            }
            if let Some(buffer) = &self.lines {
                pass.set_pipeline(&self.edges);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..6, 0..self.line_count);
            }
            if let Some(buffer) = &self.markers {
                pass.set_pipeline(&self.points);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..6, 0..self.marker_count);
            }
        }
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("readable HUD"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                ..Default::default()
            });
            // egui requires a detached pass lifetime. The pass is dropped before
            // finishing the encoder, as required by wgpu's safe dynamic checks.
            self.gui.render(&mut pass.forget_lifetime(), jobs, &screen);
        }
        let capture = if let Some(request) = self.screenshot.take() {
            request.check()?;
            Some((
                capture::copy(
                    &self.device,
                    &mut encoder,
                    &frame.texture,
                    self.config.width,
                    self.config.height,
                    self.config.format,
                )?,
                request,
            ))
        } else {
            None
        };
        let submission = self.queue.submit(
            commands
                .into_iter()
                .chain(std::iter::once(encoder.finish())),
        );
        self.window.pre_present_notify();
        self.queue.present(frame);
        if let Some((capture, request)) = capture {
            let device = self.device.clone();
            let proxy = self.repaint.clone();
            request.mark_started();
            let started = request.clone();
            let generation = request.generation;
            let spawned = std::thread::Builder::new()
                .name("viewer-frame-readback".into())
                .spawn(move || {
                    // This worker owns the submission and readback buffer;
                    // finish borrows them until its map/file cleanup completes.
                    let result = capture::finish(&device, &submission, &capture, &request);
                    let _ = proxy.send_event(UserEvent::Repaint(std::time::Duration::ZERO));
                    result
                });
            match spawned {
                Ok(worker) => self.readback = Some((generation, worker)),
                Err(error) => {
                    started.mark_not_started();
                    return Err(format!("Screenshot worker: {error}"));
                }
            }
        }
        Ok(())
    }
}

fn encode(vertices: &[Vertex]) -> Vec<u8> {
    vertices
        .iter()
        .flat_map(|v| v.position.into_iter().chain(v.color))
        .flat_map(f32::to_ne_bytes)
        .collect()
}

fn encode_edges(vertices: &[Vertex]) -> Result<Vec<u8>> {
    let (segments, remainder) = vertices.as_chunks::<2>();
    if !remainder.is_empty() {
        return Err("Incomplete native boundary display segment".into());
    }
    let mut bytes = Vec::new();
    for [a, b] in segments {
        for value in a.position.into_iter().chain(b.position).chain(a.color) {
            bytes.extend(value.to_ne_bytes());
        }
    }
    Ok(bytes)
}

fn depth(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("viewer depth"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

#[derive(Clone, Copy)]
enum Pass {
    Faces,
    Edges,
    Points,
}
fn pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    format: wgpu::TextureFormat,
    kind: Pass,
) -> wgpu::RenderPipeline {
    let edges = matches!(kind, Pass::Edges);
    let points = matches!(kind, Pass::Points);
    let face_attributes = wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3];
    let edge_attributes = wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32x3];
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(if points {
            "native vertices"
        } else if edges {
            "native edges"
        } else {
            "native faces"
        }),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(if points {
                "point"
            } else if edges {
                "edge"
            } else {
                "vertex"
            }),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[Some(wgpu::VertexBufferLayout {
                array_stride: if edges { 36 } else { 24 },
                step_mode: if edges || points {
                    wgpu::VertexStepMode::Instance
                } else {
                    wgpu::VertexStepMode::Vertex
                },
                attributes: if edges {
                    edge_attributes.as_slice()
                } else {
                    face_attributes.as_slice()
                },
            })],
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(if points { "point_fragment" } else { "fragment" }),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: Some(!edges && !points),
            depth_compare: Some(wgpu::CompareFunction::LessEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState {
                constant: if edges || points { 0 } else { 2 },
                // Leave edge depth unchanged. A face-only, pixel-scale bias
                // prevents its own boundary from z-fighting, while LessEqual
                // continues to reject segments behind foreground geometry.
                slope_scale: if edges || points { 0. } else { 2. },
                clamp: 0.,
            },
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview_mask: None,
        cache: None,
    })
}
