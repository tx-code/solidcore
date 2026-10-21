// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Read back this viewer's completed GPU frame, never the desktop or another window.
use crate::scene::Result;
use std::{
    io::{BufWriter, ErrorKind},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    time::{Duration, Instant},
};

/// Resolve aliases before opening a window and reject every existing destination.
/// Publication also uses atomic no-replace creation, closing the later race.
pub(crate) fn destination(input: &Path, output: &Path) -> Result<PathBuf> {
    let input = input
        .canonicalize()
        .map_err(|e| format!("Screenshot input: {e}"))?;
    if let Ok(target) = output.canonicalize() {
        if target == input {
            return Err("Screenshot output is the input STEP file".into());
        }
    }
    match output.symlink_metadata() {
        Ok(_) => return Err("Screenshot output already exists; choose a new PNG path".into()),
        Err(error) if error.kind() == ErrorKind::NotFound => {}
        Err(error) => return Err(format!("Screenshot destination: {error}")),
    }
    let name = output
        .file_name()
        .ok_or("Screenshot output needs a filename")?;
    let parent = output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let target = parent
        .canonicalize()
        .map_err(|e| format!("Screenshot output directory: {e}"))?
        .join(name);
    if target == input {
        return Err("Screenshot output is the input STEP file".into());
    }
    Ok(target)
}

struct Status {
    cancelled: AtomicBool,
    started: AtomicBool,
    // Cancellation and final no-replace publication share this very short
    // critical section: cancellation acknowledged first prevents publication.
    publication: Mutex<()>,
}

#[derive(Clone)]
pub(crate) struct Request {
    pub generation: u64,
    target: PathBuf,
    deadline: Instant,
    status: Arc<Status>,
}

impl Request {
    pub fn check(&self) -> Result<()> {
        if self.status.cancelled.load(Ordering::Acquire) {
            return Err("Screenshot cancelled".into());
        }
        if Instant::now() >= self.deadline {
            return Err("Screenshot request deadline exceeded".into());
        }
        Ok(())
    }
    pub fn cancel(&self) {
        // A poisoned lock still fails closed. No error path may resurrect this
        // request after closing or its overall deadline has fired.
        let _publication = self.status.publication.lock();
        self.status.cancelled.store(true, Ordering::Release);
    }
    pub fn started(&self) -> bool {
        self.status.started.load(Ordering::Acquire)
    }
    pub fn mark_started(&self) {
        self.status.started.store(true, Ordering::Release);
    }
    pub fn mark_not_started(&self) {
        self.status.started.store(false, Ordering::Release);
    }
    fn remaining(&self) -> Result<Duration> {
        self.check()?;
        Ok(self.deadline.saturating_duration_since(Instant::now()))
    }
}

#[derive(Default)]
pub(crate) struct State {
    next_generation: u64,
    in_flight: Option<Request>,
    finished: bool,
}

impl State {
    pub fn begin(&mut self, input: &Path, output: &Path, timeout: Duration) -> Result<Request> {
        if self.in_flight.is_some() {
            return Err("A GPU screenshot is already in progress".into());
        }
        let target = destination(input, output)?;
        self.next_generation = self
            .next_generation
            .checked_add(1)
            .ok_or("Screenshot generation capacity exceeded")?;
        let deadline = Instant::now()
            .checked_add(timeout)
            .ok_or("Screenshot deadline capacity exceeded")?;
        let request = Request {
            generation: self.next_generation,
            target,
            deadline,
            status: Arc::new(Status {
                cancelled: AtomicBool::new(false),
                started: AtomicBool::new(false),
                publication: Mutex::new(()),
            }),
        };
        self.in_flight = Some(request.clone());
        self.finished = false;
        Ok(request)
    }
    pub fn request(&self) -> Option<&Request> {
        self.in_flight.as_ref()
    }
    pub fn finished(&self) -> bool {
        self.finished
    }
    pub fn deadline(&self) -> Option<Instant> {
        self.in_flight
            .as_ref()
            .filter(|r| !r.status.cancelled.load(Ordering::Acquire))
            .map(|r| r.deadline)
    }
    pub fn complete(&mut self, generation: u64) -> bool {
        if self
            .in_flight
            .as_ref()
            .is_none_or(|r| r.generation != generation)
        {
            return false;
        }
        self.in_flight = None;
        self.finished = true;
        true
    }
    pub fn cancel(&self) {
        if let Some(request) = &self.in_flight {
            request.cancel();
        }
    }
}

pub(crate) struct Capture {
    buffer: wgpu::Buffer,
    width: u32,
    height: u32,
    row_bytes: u32,
    padded_row_bytes: u32,
    bgra: bool,
}

pub(crate) fn supported(format: wgpu::TextureFormat) -> bool {
    matches!(
        format,
        wgpu::TextureFormat::Rgba8Unorm
            | wgpu::TextureFormat::Rgba8UnormSrgb
            | wgpu::TextureFormat::Bgra8Unorm
            | wgpu::TextureFormat::Bgra8UnormSrgb
    )
}

pub(crate) fn copy(
    device: &wgpu::Device,
    encoder: &mut wgpu::CommandEncoder,
    texture: &wgpu::Texture,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
) -> Result<Capture> {
    if width == 0 || height == 0 || !supported(format) {
        return Err("Screenshot requires a nonempty RGBA8/BGRA8 surface".into());
    }
    let row_bytes = width
        .checked_mul(4)
        .ok_or("Screenshot row capacity exceeded")?;
    let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_row_bytes = row_bytes
        .checked_add(alignment - 1)
        .map(|n| n / alignment * alignment)
        .ok_or("Screenshot stride capacity exceeded")?;
    let size = u64::from(padded_row_bytes) * u64::from(height);
    if size > device.limits().max_buffer_size || size > 256 * 1024 * 1024 {
        return Err("Screenshot exceeds the 256 MiB/device readback capacity".into());
    }
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("complete viewer frame readback"),
        size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    // Encoded after the geometry AND HUD render passes in the same encoder.
    // A surface COPY_SRC capability is checked before requesting capture.
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_row_bytes),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    Ok(Capture {
        buffer,
        width,
        height,
        row_bytes,
        padded_row_bytes,
        bgra: matches!(
            format,
            wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
        ),
    })
}

pub(crate) fn finish(
    device: &wgpu::Device,
    submission: &wgpu::SubmissionIndex,
    capture: &Capture,
    request: &Request,
) -> Result<PathBuf> {
    request.check()?;
    let (sender, receiver) = mpsc::sync_channel(1);
    let mapping = Mapping(&capture.buffer);
    capture
        .buffer
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result.map_err(|e| e.to_string()));
        });
    // Poll the exact submission on the worker. Short finite waits acknowledge
    // cancellation promptly; the request deadline also covers waiting for a
    // surface and PNG encoding, not merely this post-submit phase.
    let map_deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let remaining = request.remaining()?;
        if Instant::now() >= map_deadline {
            return Err("Screenshot GPU mapping timed out".into());
        }
        let wait = remaining
            .min(map_deadline.saturating_duration_since(Instant::now()))
            .min(Duration::from_millis(100));
        match device.poll(wgpu::PollType::Wait {
            submission_index: Some(submission.clone()),
            timeout: Some(wait),
        }) {
            Ok(_) | Err(wgpu::PollError::Timeout) => {}
            Err(error) => return Err(format!("Screenshot GPU completion: {error}")),
        }
        match receiver.recv_timeout(Duration::from_millis(1)) {
            Ok(result) => {
                result?;
                break;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(error) => return Err(format!("Screenshot map callback: {error}")),
        }
    }
    let rgba = match capture.buffer.slice(..).get_mapped_range() {
        Ok(mapped) => rgba_rows(
            &mapped,
            capture.row_bytes,
            capture.padded_row_bytes,
            capture.height,
            capture.bgra,
        ),
        Err(error) => Err(format!("Screenshot map range: {error}")),
    };
    // Mapping is cancelled/unmapped on every return, even timeout/map failure.
    // All mapped views are already dropped before this guard is released.
    drop(mapping);
    let rgba = rgba?;
    write_png(request, capture.width, capture.height, &rgba)
}

struct Mapping<'a>(&'a wgpu::Buffer);
impl Drop for Mapping<'_> {
    fn drop(&mut self) {
        self.0.unmap();
    }
}

struct TemporaryOutput {
    path: PathBuf,
}
impl TemporaryOutput {
    fn create(request: &Request) -> Result<(Self, std::fs::File)> {
        request.check()?;
        let name = request
            .target
            .file_name()
            .ok_or("Screenshot output filename missing")?
            .to_string_lossy();
        let path = request.target.with_file_name(format!(
            ".{name}.{}.{}.part",
            std::process::id(),
            request.generation
        ));
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| format!("Screenshot temporary output: {e}"))?;
        Ok((Self { path }, file))
    }
    fn publish(&self, request: &Request) -> Result<PathBuf> {
        let _publication = request
            .status
            .publication
            .lock()
            .map_err(|_| "Screenshot publication lock poisoned")?;
        request.check()?;
        // Both paths share a directory/filesystem. Creating the final hard
        // link is atomic and NEVER replaces an existing file (unlike rename on
        // Unix), even if another process creates it after initial validation.
        std::fs::hard_link(&self.path, &request.target)
            .map_err(|e| format!("Screenshot no-replace publication: {e}"))?;
        Ok(request.target.clone())
    }
}
impl Drop for TemporaryOutput {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

fn write_png(request: &Request, width: u32, height: u32, rgba: &[u8]) -> Result<PathBuf> {
    let (temporary, file) = TemporaryOutput::create(request)?;
    // The writer closes before the temporary-file guard runs, on success and
    // every encoder/I/O failure. No partial PNG is published at the target.
    let mut encoder = png::Encoder::new(BufWriter::new(file), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| e.to_string())?;
    writer.write_image_data(rgba).map_err(|e| e.to_string())?;
    writer.finish().map_err(|e| e.to_string())?;
    let published = temporary.publish(request)?;
    std::fs::remove_file(&temporary.path)
        .map_err(|e| format!("Screenshot temporary cleanup: {e}"))?;
    Ok(published)
}

fn rgba_rows(
    bytes: &[u8],
    row_bytes: u32,
    padded_row_bytes: u32,
    height: u32,
    bgra: bool,
) -> Result<Vec<u8>> {
    let stride = usize::try_from(padded_row_bytes).map_err(|e| e.to_string())?;
    let row_len = usize::try_from(row_bytes).map_err(|e| e.to_string())?;
    let height = usize::try_from(height).map_err(|e| e.to_string())?;
    if stride == 0
        || row_len == 0
        || row_len > stride
        || row_len % 4 != 0
        || stride.checked_mul(height) != Some(bytes.len())
    {
        return Err("Invalid screenshot row layout".into());
    }
    let mut rgba = Vec::new();
    // GPU copies are top-to-bottom. Remove per-row padding before encoding,
    // and swizzle BGRA surfaces without changing row order or alpha.
    for padded in bytes.chunks_exact(stride) {
        let row = padded.get(..row_len).ok_or("Short screenshot row")?;
        if bgra {
            let (pixels, _) = row.as_chunks::<4>();
            for [b, g, r, a] in pixels {
                rgba.extend([*r, *g, *b, *a]);
            }
        } else {
            rgba.extend_from_slice(row);
        }
    }
    Ok(rgba)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU64;

    struct Directory(PathBuf);
    impl Directory {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let path = std::env::temp_dir().join(format!(
                "solidcore-capture-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&path).unwrap();
            Self(path)
        }
        fn source(&self) -> PathBuf {
            let path = self.0.join("original.step");
            std::fs::write(&path, b"original STEP bytes").unwrap();
            path
        }
        fn request(&self) -> Request {
            State::default()
                .begin(
                    &self.source(),
                    &self.0.join("capture.png"),
                    Duration::from_secs(20),
                )
                .unwrap()
        }
        fn no_temporary_files(&self) -> bool {
            std::fs::read_dir(&self.0).unwrap().all(|entry| {
                !entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .ends_with(".part")
            })
        }
    }
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn source_alias_and_existing_output_are_refused_without_modification() {
        let directory = Directory::new();
        let source = directory.source();
        assert!(destination(&source, &source).is_err());
        std::fs::create_dir(directory.0.join("sub")).unwrap();
        assert!(destination(&source, &directory.0.join("sub/../original.step")).is_err());
        let output = directory.0.join("existing.png");
        std::fs::write(&output, b"previous image").unwrap();
        assert!(destination(&source, &output).is_err());
        assert_eq!(std::fs::read(&source).unwrap(), b"original STEP bytes");
        assert_eq!(std::fs::read(&output).unwrap(), b"previous image");
    }

    #[test]
    fn encoder_failure_and_destination_race_leave_no_partial_output() {
        let directory = Directory::new();
        let request = directory.request();
        assert!(write_png(&request, 2, 2, &[0]).is_err());
        assert!(!request.target.exists());
        assert!(directory.no_temporary_files());
        std::fs::write(&request.target, b"concurrently created file").unwrap();
        assert!(write_png(&request, 1, 1, &[1, 2, 3, 255]).is_err());
        assert_eq!(
            std::fs::read(&request.target).unwrap(),
            b"concurrently created file"
        );
        assert!(directory.no_temporary_files());
    }

    #[test]
    fn cancelled_write_cleans_temporary_and_completed_png_is_published_once() {
        let directory = Directory::new();
        let request = directory.request();
        {
            let (temporary, file) = TemporaryOutput::create(&request).unwrap();
            drop(file);
            request.cancel();
            assert!(temporary.publish(&request).is_err());
        }
        assert!(!request.target.exists());
        assert!(directory.no_temporary_files());
        let request = directory.request();
        let output = write_png(&request, 1, 1, &[1, 2, 3, 255]).unwrap();
        assert_eq!(
            std::fs::read(&output).unwrap().get(..8),
            Some(b"\x89PNG\r\n\x1a\n".as_slice())
        );
        assert!(directory.no_temporary_files());
        assert!(write_png(&request, 1, 1, &[9, 8, 7, 255]).is_err());
        assert!(directory.no_temporary_files());
    }

    #[test]
    fn old_generation_cannot_complete_new_request_and_deadline_cancels_publication() {
        let directory = Directory::new();
        let source = directory.source();
        let output = directory.0.join("capture.png");
        let mut state = State::default();
        let old = state
            .begin(&source, &output, Duration::from_secs(20))
            .unwrap();
        assert!(state
            .begin(&source, &output, Duration::from_secs(20))
            .is_err());
        assert!(state.complete(old.generation));
        let current = state.begin(&source, &output, Duration::ZERO).unwrap();
        assert!(!state.complete(old.generation));
        assert_eq!(state.request().unwrap().generation, current.generation);
        assert!(current.check().is_err());
        assert!(write_png(&current, 1, 1, &[1, 2, 3, 255]).is_err());
        state.cancel();
        assert!(state.deadline().is_none());
        assert!(state.complete(current.generation));
        assert!(state.request().is_none());
        assert!(!output.exists());
        assert!(directory.no_temporary_files());
    }
    #[test]
    fn padded_bgra_rows_keep_top_to_bottom_pixels_and_alpha() {
        let mut data = vec![0; 512];
        data[..8].copy_from_slice(&[3, 2, 1, 255, 6, 5, 4, 128]);
        data[256..264].copy_from_slice(&[9, 8, 7, 64, 12, 11, 10, 0]);
        assert_eq!(
            rgba_rows(&data, 8, 256, 2, true).unwrap(),
            vec![1, 2, 3, 255, 4, 5, 6, 128, 7, 8, 9, 64, 10, 11, 12, 0]
        );
        assert_eq!(
            rgba_rows(&[1, 2, 3, 4], 4, 4, 1, false).unwrap(),
            vec![1, 2, 3, 4]
        );
        assert!(rgba_rows(&data, 8, 0, 2, true).is_err());
        assert!(rgba_rows(&data, 8, 256, 3, true).is_err());
    }
}
