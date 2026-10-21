// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Standalone viewer of Rust native construction candidates, with explicit qualification.
#![forbid(unsafe_code)]

mod camera;
mod capture;
mod fonts;
mod gpu;
mod picking;
mod scene;

use camera::Camera;
use glam::DVec2;
use scene::{Result, Scene};
use std::path::PathBuf;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};
use winit::keyboard::{KeyCode, ModifiersState, PhysicalKey};
use winit::window::{Window, WindowId};

struct Cli {
    path: PathBuf,
    fixture: bool,
    check: bool,
    screenshot: Option<PathBuf>,
    exit_after_screenshot: bool,
}

fn arguments() -> Result<Option<Cli>> {
    let mut path = None;
    let mut fixture = false;
    let mut check = false;
    let mut screenshot = None;
    let mut exit_after_screenshot = false;
    let mut arguments = std::env::args_os().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.to_str() {
            Some("--help" | "-h") => {
                println!("solidcore-viewer FILE.step [--fixture-limits] [--check]\n                    [--screenshot FILE.png] [--exit-after-screenshot]\n\nNative Rust LINE/PLANE development viewer.\nWith FILE alone, select the development preset in the window.\n--fixture-limits explicitly chooses synthetic development budgets and starts loading.\n--check loads and derives display geometry without a GPU (requires --fixture-limits).\n--screenshot copies this viewer's complete loaded GPU frame, including HUD, to PNG.\n--exit-after-screenshot exits after the PNG is written; capture/load errors exit nonzero.\n\nDrag left: orbit · wheel: zoom · F/Home: fit · Esc: close.\nNative construction and validation do not establish accepted import.\nGeneral STEP, hover, selection and measurement remain separate work.");
                return Ok(None);
            }
            Some("--fixture-limits") => fixture = true,
            Some("--check") => check = true,
            Some("--screenshot") => {
                screenshot = Some(PathBuf::from(
                    arguments.next().ok_or("--screenshot requires a PNG path")?,
                ));
            }
            Some("--exit-after-screenshot") => exit_after_screenshot = true,
            Some(value) if value.starts_with('-') => {
                return Err(format!("Unknown option {value}; use --help"))
            }
            _ => {
                if path.replace(PathBuf::from(argument)).is_some() {
                    return Err("Supply one STEP file; use --help".into());
                }
            }
        }
    }
    if check && !fixture {
        return Err("--check requires the explicit --fixture-limits development preset".into());
    }
    if check && screenshot.is_some() {
        return Err(
            "--screenshot requires window/GPU rendering and cannot be combined with --check".into(),
        );
    }
    if exit_after_screenshot && screenshot.is_none() {
        return Err("--exit-after-screenshot requires --screenshot FILE.png".into());
    }
    let path = path.ok_or("Supply a STEP file; use --help")?;
    if let Some(output) = &screenshot {
        screenshot = Some(capture::destination(&path, output)?);
    }
    Ok(Some(Cli {
        path,
        fixture,
        check,
        screenshot,
        exit_after_screenshot,
    }))
}

enum UserEvent {
    Progress(String),
    Loaded(Result<Box<Scene>>),
    GpuError(String),
    Repaint(Duration),
    Screenshot {
        generation: u64,
        result: Result<PathBuf>,
    },
}

struct App {
    cli: Cli,
    proxy: EventLoopProxy<UserEvent>,
    window: Option<Arc<Window>>,
    gpu: Option<gpu::Gpu>,
    ctx: egui::Context,
    input: Option<egui_winit::State>,
    scene: Option<Box<Scene>>,
    camera: Camera,
    progress: String,
    error: Option<String>,
    fatal: Option<String>,
    loading: bool,
    cancelled: Arc<AtomicBool>,
    dragging: bool,
    cursor: Option<(f64, f64)>,
    viewport: egui::Rect,
    repaint_at: Option<Instant>,
    captures: capture::State,
    closing: bool,
    selection: picking::Selection,
    pick_error: Option<String>,
    press: Option<DVec2>,
    modifiers: ModifiersState,
}

impl App {
    fn new(cli: Cli, proxy: EventLoopProxy<UserEvent>) -> Self {
        Self {
            cli,
            proxy,
            window: None,
            gpu: None,
            ctx: egui::Context::default(),
            input: None,
            scene: None,
            camera: Camera::default(),
            progress: "Choose the explicit development preset to load this file.".into(),
            error: None,
            fatal: None,
            loading: false,
            cancelled: Arc::new(AtomicBool::new(false)),
            dragging: false,
            cursor: None,
            viewport: egui::Rect::NOTHING,
            repaint_at: None,
            captures: capture::State::default(),
            closing: false,
            selection: picking::Selection::default(),
            pick_error: None,
            press: None,
            modifiers: ModifiersState::empty(),
        }
    }

    fn start_load(&mut self) {
        if self.loading || self.captures.request().is_some() || self.closing {
            return;
        }
        self.scene = None;
        self.selection = picking::Selection::default();
        self.pick_error = None;
        if let Some(gpu) = &mut self.gpu {
            gpu.clear();
        }
        self.error = None;
        self.loading = true;
        self.progress = "Starting native load…".into();
        let path = self.cli.path.clone();
        let proxy = self.proxy.clone();
        let cancelled = self.cancelled.clone();
        let error_proxy = self.proxy.clone();
        if let Err(error) = std::thread::Builder::new()
            .name("step-native-load".into())
            .spawn(move || {
                let result = scene::load(&path, |message| {
                    if cancelled.load(Ordering::Relaxed) {
                        return Err("Loading cancelled".into());
                    }
                    proxy
                        .send_event(UserEvent::Progress(message.into()))
                        .map_err(|e| e.to_string())
                })
                .map(Box::new);
                if !cancelled.load(Ordering::Relaxed) {
                    let _ = proxy.send_event(UserEvent::Loaded(result));
                }
            })
        {
            let _ = error_proxy.send_event(UserEvent::Loaded(Err(error.to_string())));
        }
    }

    fn redraw(&mut self) {
        let Some(window) = &self.window else {
            return;
        };
        let size = window.inner_size();
        if size.width == 0 || size.height == 0 {
            return;
        }
        let Some(input) = &mut self.input else {
            return;
        };
        let raw = input.take_egui_input(window);
        let mut start = false;
        let mut fit = false;
        let mut clear_selection = false;
        let inspection = self
            .scene
            .as_ref()
            .map(|scene| picking::details(scene, &self.selection))
            .transpose();
        let mut output=self.ctx.run_ui(raw,|ui| {
            egui::Panel::left("model-hud").resizable(false).exact_size(300.).show(ui,|ui| {
                egui::ScrollArea::vertical().auto_shrink([false,false]).show(ui,|ui| {
                ui.add_space(8.);ui.heading("solidcore");ui.label("Native STEP viewer");ui.separator();
                ui.label(egui::RichText::new(self.cli.path.file_name().map_or_else(||"STEP file".into(),|p|p.to_string_lossy())).strong());
                ui.small(self.cli.path.to_string_lossy());ui.add_space(8.);
                if self.loading{ui.horizontal(|ui|{ui.spinner();ui.label(&self.progress);});}
                else{ui.label(&self.progress);}
                if let Some(error)=&self.error {
                    ui.add_space(8.);ui.colored_label(egui::Color32::from_rgb(255,160,140),"Load / rendering failed");
                    egui::ScrollArea::vertical().max_height(230.).show(ui,|ui|{ui.label(error);});
                }
                if let Some(scene)=&self.scene {
                    ui.add_space(8.);ui.label(format!("Source role: {}",scene.role));
                    ui.label(format!("{} faces · {} edges · {} vertices",scene.owner.brep().face_list().len(),scene.owner.brep().edge_list().len(),scene.owner.brep().vertex_list().len()));
                    ui.label(format!("{} display triangles",scene.triangles.len()/3));
                    ui.label(format!("Radius: {:.6} mm",scene.radius_mm));
                    ui.small(format!("Center mm: {:.5}, {:.5}, {:.5}",scene.center_mm.x,scene.center_mm.y,scene.center_mm.z));
                    ui.add_space(6.);ui.colored_label(egui::Color32::from_rgb(242,202,128),"Native candidate · not accepted import");
                    ui.small("Pointer and level-two native checks passed. Continuous source/native acceptance and publication are not selected here.");
                }
                ui.add_space(10.);
                if !self.loading {
                    if self.scene.is_none(){start=ui.button("Load with development fixture preset").clicked();}
                    else{ui.horizontal(|ui|{fit=ui.button("Fit model  [F]").clicked();start=ui.add_enabled(self.captures.request().is_none(),egui::Button::new("Reload")).clicked();});}
                }
                ui.small("Explicit synthetic limits / quality / working tolerance. No production preset is inferred.");
                ui.separator();ui.label("Drag left: orbit\nWheel: zoom\nF / Home: fit\nEsc: close");
                ui.add_space(6.);ui.small("LINE / PLANE trims only. Shading and native boundary edges are derived display output.");
                ui.small("Hover to inspect · Click to select · Ctrl-click two vertices to measure");
                if let Ok(Some(rows))=&inspection {for row in rows {ui.label(row);}}
                if let Err(error)=&inspection {ui.colored_label(egui::Color32::LIGHT_RED,error);}
                if let Some(error)=&self.pick_error {ui.colored_label(egui::Color32::LIGHT_RED,format!("Native query unavailable: {error}"));}
                if !self.selection.selected.is_empty(){clear_selection=ui.button("Clear selection").clicked();}
                if let Some(gpu)=&self.gpu{ui.add_space(8.);ui.small(&gpu.adapter);}
                });
            });
            self.viewport=ui.available_rect_before_wrap();
        });
        input.handle_platform_output(window, output.platform_output);
        let jobs = self.ctx.tessellate(output.shapes, output.pixels_per_point);
        if let Some(gpu) = &mut self.gpu {
            if let Err(error) = gpu.render(
                &self.camera,
                self.viewport,
                output.pixels_per_point,
                &jobs,
                &mut output.textures_delta,
            ) {
                if let Some(request) = self.captures.request() {
                    request.cancel();
                    if !request.started() {
                        let _ = self.proxy.send_event(UserEvent::Screenshot {
                            generation: request.generation,
                            result: Err(error.clone()),
                        });
                    }
                }
                let _ = self.proxy.send_event(UserEvent::GpuError(error.clone()));
                self.error = Some(error);
                self.scene = None;
                self.selection = picking::Selection::default();
                gpu.clear();
            }
        } else {
            // No painter owns these commands, so explicitly discard this
            // frame rather than dropping a live epaint TexturesDelta. All
            // GPU-present paths drain it inside render, including errors.
            output.textures_delta.clear();
        }
        if fit {
            self.camera.fit();
            window.request_redraw();
        }
        if fit {
            self.update_hover();
        }
        if clear_selection {
            self.selection.selected.clear();
            self.refresh_highlights();
        }
        if start {
            self.start_load();
        }
    }

    fn refresh_highlights(&mut self) {
        if let (Some(scene), Some(gpu)) = (&self.scene, &mut self.gpu) {
            if let Err(error) = gpu.highlight(scene, &self.selection) {
                self.pick_error = Some(error);
            }
        }
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn update_hover(&mut self) {
        let result = if let (Some(scene), Some(window), Some((x, y))) =
            (&mut self.scene, &self.window, self.cursor)
        {
            let size = window.inner_size();
            if size.width == 0 || size.height == 0 {
                Ok(None)
            } else {
                let frame = self.camera.frame(
                    self.viewport,
                    window.scale_factor() as f32,
                    [size.width, size.height],
                );
                picking::pick(scene, frame, DVec2::new(x, y))
            }
        } else {
            Ok(None)
        };
        let changed = match result {
            Ok(hit) => {
                self.pick_error = None;
                self.selection.hover(hit)
            }
            Err(error) => {
                self.pick_error = Some(error);
                self.selection.hover(None)
            }
        };
        if changed {
            self.refresh_highlights();
        }
    }

    fn request_close(&mut self, event_loop: &ActiveEventLoop) {
        self.closing = true;
        self.cancelled.store(true, Ordering::Relaxed);
        self.captures.cancel();
        if let Some(gpu) = &mut self.gpu {
            gpu.cancel_screenshot();
        }
        if self
            .captures
            .request()
            .is_some_and(capture::Request::started)
        {
            // Keep dispatching until the worker has unmapped its buffer and
            // removed any unpublished temporary PNG. Never kill a live write.
            self.progress = "Cancelling GPU capture and cleaning temporary output…".into();
            if let Some(window) = &self.window {
                window.request_redraw();
            }
        } else {
            if let Some(request) = self.captures.request() {
                let generation = request.generation;
                self.captures.complete(generation);
            }
            event_loop.exit();
        }
    }
}

impl ApplicationHandler<UserEvent> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attributes = Window::default_attributes()
            .with_title("solidcore · Native STEP viewer")
            .with_inner_size(LogicalSize::new(1180., 800.))
            .with_min_inner_size(LogicalSize::new(660., 480.))
            .with_visible(false);
        let window = match event_loop.create_window(attributes) {
            Ok(window) => Arc::new(window),
            Err(error) => {
                self.fatal = Some(error.to_string());
                event_loop.exit();
                return;
            }
        };
        let gpu = match pollster::block_on(gpu::Gpu::new(window.clone(), self.proxy.clone())) {
            Ok(gpu) => gpu,
            Err(error) => {
                self.fatal = Some(format!("GPU initialization: {error}"));
                event_loop.exit();
                return;
            }
        };
        self.ctx.set_visuals(egui::Visuals::dark());
        match fonts::definitions() {
            Ok(fonts) => self.ctx.set_fonts(fonts),
            Err(error) => {
                self.fatal = Some(error);
                event_loop.exit();
                return;
            }
        }
        self.input = Some(egui_winit::State::new(
            self.ctx.clone(),
            egui::ViewportId::ROOT,
            event_loop,
            Some(window.scale_factor() as f32),
            window.theme(),
            None,
        ));
        let repaint_proxy = self.proxy.clone();
        self.ctx.set_request_repaint_callback(move |request| {
            let _ = repaint_proxy.send_event(UserEvent::Repaint(request.delay));
        });
        self.gpu = Some(gpu);
        self.window = Some(window.clone());
        window.set_visible(true);
        window.request_redraw();
        if self.cli.fixture {
            self.start_load();
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::Progress(progress) => self.progress = progress,
            UserEvent::Loaded(result) => {
                self.loading = false;
                match result {
                    Ok(scene) => {
                        let upload = self
                            .gpu
                            .as_mut()
                            .ok_or_else(|| "GPU unavailable".to_string())
                            .and_then(|gpu| gpu.upload(&scene));
                        match upload {
                            Ok(()) => {
                                self.scene = Some(scene);
                                self.selection = picking::Selection::default();
                                self.pick_error = None;
                                self.camera.fit();
                                self.progress = "Native geometry ready for inspection".into();
                                if let Some(path) = self.cli.screenshot.clone() {
                                    match self.captures.begin(
                                        &self.cli.path,
                                        &path,
                                        Duration::from_secs(20),
                                    ) {
                                        Ok(request) => {
                                            let generation = request.generation;
                                            let queued = self
                                                .gpu
                                                .as_mut()
                                                .ok_or_else(|| "GPU unavailable".to_string())
                                                .and_then(|gpu| gpu.request_screenshot(request));
                                            if let Err(error) = queued {
                                                self.captures.complete(generation);
                                                self.error = Some(error);
                                            }
                                        }
                                        Err(error) => self.error = Some(error),
                                    }
                                }
                            }
                            Err(error) => self.error = Some(error),
                        }
                    }
                    Err(error) => {
                        self.error = Some(error);
                        self.progress = "No geometry published to the viewer".into();
                    }
                }
            }
            UserEvent::GpuError(error) => {
                self.error = Some(format!("GPU: {error}"));
                self.scene = None;
                self.selection = picking::Selection::default();
                if let Some(gpu) = &mut self.gpu {
                    gpu.clear();
                }
                if let Some(request) = self.captures.request().cloned() {
                    request.cancel();
                    if !request.started() {
                        self.captures.complete(request.generation);
                    }
                }
            }
            UserEvent::Screenshot { generation, result } => {
                // Late events cannot complete a later request or publish UI
                // success for a reloaded owner. State owns this single seam.
                if !self.captures.complete(generation) {
                    return;
                }
                if self.closing {
                    event_loop.exit();
                    return;
                }
                match result {
                    Ok(path) => {
                        self.progress = format!("GPU frame saved: {}", path.display());
                        eprintln!("{}", self.progress);
                        if self.cli.exit_after_screenshot {
                            self.request_close(event_loop);
                            return;
                        }
                    }
                    Err(error) => self.error = Some(format!("GPU screenshot: {error}")),
                }
            }
            UserEvent::Repaint(delay) => {
                if let Some(when) = Instant::now().checked_add(delay) {
                    if self.repaint_at.is_none_or(|previous| when < previous) {
                        self.repaint_at = Some(when);
                    }
                }
                // about_to_wait schedules delayed animation without turning
                // egui's future repaint request into an immediate busy loop.
                return;
            }
        }
        if self.cli.exit_after_screenshot {
            if let Some(error) = &self.error {
                self.fatal = Some(error.clone());
                self.request_close(event_loop);
                return;
            }
        }
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        let Some(window) = self.window.clone() else {
            return;
        };
        if id != window.id() {
            return;
        }
        let mut consumed = false;
        if let Some(input) = &mut self.input {
            let response = input.on_window_event(&window, &event);
            consumed = response.consumed;
            if response.repaint {
                window.request_redraw();
            }
        }
        match event {
            WindowEvent::CloseRequested => {
                if self.cli.exit_after_screenshot && !self.captures.finished() {
                    self.fatal = Some("Viewer closed before GPU screenshot completed".into());
                }
                self.request_close(event_loop);
            }
            WindowEvent::RedrawRequested => self.redraw(),
            WindowEvent::Resized(size) => {
                if let Some(gpu) = &mut self.gpu {
                    gpu.resize(size.width, size.height);
                }
                window.request_redraw();
            }
            WindowEvent::ScaleFactorChanged { .. } => {
                let size = window.inner_size();
                if let Some(gpu) = &mut self.gpu {
                    gpu.resize(size.width, size.height);
                }
                window.request_redraw();
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                if state == ElementState::Released {
                    let click = self.press.take().is_some() && !self.dragging && !consumed;
                    self.dragging = false;
                    self.update_hover();
                    if click {
                        self.selection.click(self.modifiers.control_key());
                        self.refresh_highlights();
                    }
                } else if !consumed {
                    self.dragging = false;
                    self.press = self.cursor.map(|(x, y)| DVec2::new(x, y));
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let cursor = DVec2::new(position.x, position.y);
                if self
                    .press
                    .is_some_and(|p| p.distance(cursor) > 4. * window.scale_factor())
                {
                    self.dragging = true;
                }
                if self.dragging {
                    if let Some((x, y)) = self.cursor {
                        self.camera
                            .orbit((position.x - x) as f32, (position.y - y) as f32);
                        window.request_redraw();
                    }
                }
                self.cursor = Some((position.x, position.y));
                if self.dragging {
                    if self.selection.hover(None) {
                        self.refresh_highlights();
                    }
                } else {
                    self.update_hover();
                }
            }
            WindowEvent::MouseWheel { delta, .. } if !consumed => {
                let steps = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.02,
                };
                self.camera.zoom(steps);
                self.update_hover();
                window.request_redraw();
            }
            WindowEvent::KeyboardInput { event, .. } if event.state == ElementState::Pressed => {
                match event.physical_key {
                    PhysicalKey::Code(KeyCode::Escape) => {
                        if self.cli.exit_after_screenshot && !self.captures.finished() {
                            self.fatal =
                                Some("Viewer closed before GPU screenshot completed".into());
                        }
                        self.request_close(event_loop);
                    }
                    PhysicalKey::Code(KeyCode::KeyF | KeyCode::Home) => {
                        self.camera.fit();
                        self.update_hover();
                        window.request_redraw();
                    }
                    _ => {}
                }
            }
            WindowEvent::Focused(false) => {
                self.dragging = false;
                self.press = None;
                if self.selection.hover(None) {
                    self.refresh_highlights();
                }
            }
            WindowEvent::ModifiersChanged(modifiers) => self.modifiers = modifiers.state(),
            WindowEvent::CursorLeft { .. } => {
                if self.selection.hover(None) {
                    self.refresh_highlights();
                }
            }
            WindowEvent::Occluded(false) => window.request_redraw(),
            _ => {
                if consumed {
                    window.request_redraw();
                }
            }
        }
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        self.cancelled.store(true, Ordering::Relaxed);
        self.captures.cancel();
        self.scene = None;
        self.gpu = None;
        self.input = None;
        self.window = None;
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if let Some((generation, result)) = self
            .gpu
            .as_mut()
            .and_then(gpu::Gpu::take_capture_completion)
        {
            self.user_event(event_loop, UserEvent::Screenshot { generation, result });
        }
        if self
            .captures
            .deadline()
            .is_some_and(|when| when <= Instant::now())
        {
            if let Some(request) = self.captures.request().cloned() {
                request.cancel();
                if let Some(gpu) = &mut self.gpu {
                    gpu.cancel_screenshot();
                }
                self.error = Some("Screenshot request deadline exceeded".into());
                self.progress = "Cancelling GPU screenshot and cleaning temporary output…".into();
                if !request.started() {
                    self.captures.complete(request.generation);
                }
                if self.cli.exit_after_screenshot {
                    self.fatal = self.error.clone();
                    self.request_close(event_loop);
                } else if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
        }
        if self.repaint_at.is_some_and(|when| when <= Instant::now()) {
            self.repaint_at = None;
            if let Some(window) = &self.window {
                window.request_redraw();
            }
        }
        let worker_poll = self
            .captures
            .request()
            .filter(|r| r.started())
            .and_then(|_| Instant::now().checked_add(Duration::from_millis(100)));
        let next = self
            .repaint_at
            .into_iter()
            .chain(self.captures.deadline())
            .chain(worker_poll)
            .min();
        event_loop.set_control_flow(next.map_or(ControlFlow::Wait, ControlFlow::WaitUntil));
    }
}

fn run() -> Result<()> {
    let Some(cli) = arguments()? else {
        return Ok(());
    };
    if cli.check {
        let scene = scene::load(&cli.path, |progress| {
            eprintln!("{progress}");
            Ok(())
        })?;
        println!("NativeConstructed; accepted_import=false; source_role={}; faces={}; edges={}; vertices={}; display_triangles={}; display_edges={}",
            scene.role,scene.owner.brep().face_list().len(),scene.owner.brep().edge_list().len(),scene.owner.brep().vertex_list().len(),scene.triangles.len()/3,scene.lines.len()/2);
        return Ok(());
    }
    let event_loop = EventLoop::<UserEvent>::with_user_event()
        .build()
        .map_err(|e| e.to_string())?;
    event_loop.set_control_flow(ControlFlow::Wait);
    let mut app = App::new(cli, event_loop.create_proxy());
    event_loop.run_app(&mut app).map_err(|e| e.to_string())?;
    if let Some(error) = app.fatal {
        return Err(error);
    }
    Ok(())
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            std::process::ExitCode::FAILURE
        }
    }
}
