# Standalone native STEP viewer

This is a wgpu/winit desktop executable over the actual Rust `solidcore-step-native`
construction pipeline. The scene retains its owning `Candidate`, immutable source,
source role and native correspondence. It is labeled **Native candidate — not
accepted import**. Native pointer/deep validation is required before scene extraction;
failure produces a readable error and an empty viewport. There is no fallback reader,
foreign kernel, synthetic box or source-only geometry substituted on failure.

```text
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/plane.step --fixture-limits
cargo run -p solidcore-viewer -- path/to/file.step
cargo run -p solidcore-viewer -- --help
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/plane.step --fixture-limits --check
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/plane.step --fixture-limits --screenshot plane.png --exit-after-screenshot
```

Launching with a file alone opens configuration selection: the user must explicitly
choose the development fixture preset in the HUD before loading. `--fixture-limits`
chooses it on the command line. `--check` exercises native construction, validation
and derived scene extraction without a window or GPU. The preset uses the source,
semantic and P0 planar fixture budgets, 100,000 construction operations and 1e-6 mm
native working tolerance. These are development values, **not production defaults**.
General-purpose application configuration and broader native import are future work.

The viewport starts with shaded faces and dark native defining edges. Left drag
orbits, the wheel zooms, F/Home fits and Escape closes. A click selects a native
face, edge or vertex; Ctrl-click keeps up to two selections. Two selected native
vertices show their distance in mm. A selected LINE shows its original native curve
interval length, a vertex shows its native coordinates, and a face shows its PLANE
surface normal (not an inferred material/outward normal). Area/volume measurements
are not provided. The Clear selection button clears selections. The 300-point HUD occupies
its own area and reports original roles, executed checks, loading stages and errors.
Resize and display scaling rebuild the viewport/depth buffer. Camera near/far planes
follow zoom; f64 centering and normalization precede GPU f32 narrowing, preserving
the original geometry and mm coordinates in the native owner.

The HUD explicitly loads locally installed fonts: Segoe UI on Windows with an
available Microsoft Yahei fallback, or a common installed Linux/macOS UI font.
No font assets are copied into this repository or redistributed. Missing UI fonts
produce a clear startup error. `--help` and `--check` do not initialize fonts or a GPU.
This is required because egui 0.37's default feature set includes no fonts and its
empty font families otherwise render even ASCII as synthetic tofu boxes. The bundled
default-font feature is deliberately unused to preserve current dependency licenses.

Native edges use 2–4 physical-pixel, near-black quads with their actual projected
depth. The edge pass keeps depth testing enabled and only the face pass has a small
depth bias; hidden edges are not displayed as an x-ray overlay.

The current display adapter accepts native PLANE receivers and LINE/affine UV trims.
It walks complete native loops (including holes), evaluates the actual plane at native
trim endpoints and triangulates that region using earcutr. This is explicitly derived
display output, not the missing complete kernel BRep tessellation driver and not a
continuous source/native acceptance certificate. Unsupported geometry is refused.
The original native LINE endpoints supply edge rendering independently of face fill.
Only a successful complete extraction is transferred to the GPU.

`--screenshot FILE.png` copies the loaded scene's completed presentation texture
after the same shaded-face, edge and HUD passes used by the window. It never reads
the desktop or another application's window. This requires surface `COPY_SRC`
capability and an RGBA8/BGRA8 format; unsupported surfaces report an explicit error.
GPU rows are padded to 256 bytes for copying, then depadded and BGRA-swizzled as
needed. A readback worker maps the buffer and polls the exact submission with a
15-second GPU map limit before PNG encoding. Each capture also has a 20-second
request deadline, scheduled through the event loop even while the surface is occluded
or repeatedly times out. `--exit-after-screenshot` closes normally only after the PNG
finishes; load/capture failure or premature closing exits nonzero.
This image characterizes actual display output and grants no import acceptance.

The canonical input/output paths must differ. Existing output files are refused.
PNG encoding writes a new temporary file in the output directory; only a completed
PNG is published by atomic no-replace hard-link creation, then the temporary name is
removed. The filesystem must support hard links. Output races cannot truncate the
STEP input or another existing file. Failed/cancelled unpublished files are cleaned.
Reload stays disabled during capture; completion events carry a checked generation.
Closing cancels readback and waits for the worker's buffer/file cleanup before exit.
GPU waits check cancellation at 100 ms intervals. A blocked operating-system file
write cannot be interrupted safely; cancellation prevents later publication and the
window waits for that I/O to return so cleanup can finish. Forced process termination
can leave an unpublished `.part` file, but does not replace an existing destination.

Loading runs on one worker while the UI remains responsive; closing suppresses later
stage delivery and drops unpublished results. The worker checks cancellation between
stages. This does not promise interruption inside a native kernel operation or a hard
CPU/memory limit. No private file, inventory or derived data is uploaded.

Interaction is limited to the same LINE/PLANE profile. Display triangles only
nominate a face; a native plane intersection and owner-bound trimmed-face query
confirm it. Original LINE/vertex positions confirm features, with foreground native
faces rejecting occluded selections. Missing kernel queries are shown as unavailable
and never replaced with mesh classification. Pixel and float-depth allowances are
interaction settings, not import acceptance budgets. Rendering and picking share
one viewport/matrix, including DPI and near clipping. Queries never use a default
Kernel, cleared geometry owner or cloned model.

This increment does **not** complete general STEP/native import, 100 real models,
general curved picking or general area/volume measurement. Native identities and ownership stay
available in `Scene`. A viewer screenshot does not establish
source equivalence, material-solid semantics, accepted import or query coverage.

Versions were checked against the published crate sources: [wgpu 30.0.1](https://docs.rs/wgpu/30.0.1/wgpu/),
[winit 0.30.13](https://docs.rs/winit/0.30.13/winit/) and egui/egui-wgpu/egui-winit 0.37.0.
egui-wgpu 0.37 requires wgpu 30 and winit 0.30.13. This executable alone requires Rust
1.95; kernel/workspace MSRV is unchanged. These dependencies use permissive licenses;
earcutr is ISC and the GUI/GPU dependencies are MIT/Apache-2.0.

Focused validation:

```text
cargo test -p solidcore-viewer
cargo clippy -p solidcore-viewer --all-targets -- -D warnings
```

The public plane fixture tests actual reader-to-native scene extraction and retained
role/ownership; camera tests cover extreme zoom/orbit matrices. A real GPU/window
smoke test is additionally necessary for text layout, drawing and interaction evidence.
