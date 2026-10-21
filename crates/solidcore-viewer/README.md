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
cargo run -p solidcore-viewer -- crates/solidcore-step-native/tests/fixtures/cylinder-phase-rings.step --fixture-limits --general-development --quality quality.json
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/cone-pole.step --fixture-limits --general-development --quality quality.json --check
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/plane.step --fixture-limits --check
cargo run -p solidcore-viewer -- crates/solidcore-step/tests/fixtures/plane.step --fixture-limits --screenshot plane.png --exit-after-screenshot
```

Launching with a file alone opens configuration selection: the user must explicitly
choose the development resource preset in the HUD before loading. `--fixture-limits`
chooses it on the command line. The default route remains the strict P0 planar path;
a General failure never falls back to another reader or to source-only display.

General loading requires both `--general-development` and `--quality FILE.json`.
The quality file is read with a 64-KiB ceiling and parsed by the same pure exact
`parse_general_quality_json` API used by `step-native-import`. It contains exactly
these eight **string** fields; no missing budget or production precision is inferred:

```json
{
  "id": "Q-Viewer-public-controls",
  "definition_mm": "0.0000001",
  "consistency_mm": "0.000001",
  "derived_uv_mm": "0.000001",
  "query_mm": "0.000001",
  "initial_work_tolerance_mm": "0.000001",
  "maximum_work_tolerance_mm": "0.00001",
  "maximum_tolerance_growth_mm": "0"
}
```

This example is a **public synthetic development selection**, not a production
preset. The P0 route keeps its existing fixture quality and 1e-6 mm working tolerance.
Both modes use the explicit source/semantic resource preset and 100,000 charged
native operations across construction and requested checkpoints. General calls `construct_general`, which owns the complete source
interpretation/semantics, followed by detailed pointer/level-two native validation and
final UV correspondence measurements. The viewer does not build a second source
interpretation. `--check` runs the same entire Scene extraction as windowed loading,
including curved display derivation, without fonts, a window or a GPU; it requires
`--fixture-limits`.

General source-policy switches match the native CLI. `--interop` selects the proven
nongeometric policy. `--ap214-unregistered-as-2010` additionally requires `--interop`.
`--opaque-header-name`, `--opaque-data-strings` and `--level1-as-corrected-encoding`
require explicit General plus interoperability mode; interop enables none of them
implicitly. The separate `--development-ap214-angle-dimensions` option requires
General and an explicit quality file, but does not require or enlarge interop. It
continues only after the full source-owned dimensional/conversion proof. The HUD
retains the distinction between a requested option and an applied receipt; applied
unit compatibility is labeled **source syntax unverified**, with original source
bytes and diagnostics retained by the Candidate.

`--measure-material-accuracy VALUE` optionally requests numerical material
observations after the required native/trim checkpoints. It requires General mode,
quality, and a finite dimensionless control within `[1e-8,1e-4]`. Nothing selects an
accuracy silently. These observations do not grant import acceptance or a continuous
error bound. Native queries that invalidate a prior observation are reflected by the
HUD instead of continuing to claim that receipt is current.

The viewport starts with shaded faces and dark native defining edges. Left drag
orbits, the wheel zooms, F/Home fits and Escape closes. A click selects a native
face, edge or vertex; Ctrl-click keeps up to two selections. Two selected native
vertices show their distance in mm. A selected LINE shows its original native curve
interval length, a vertex shows its native coordinates, and a face's local surface
information is queried from its actual native receiver. Query capabilities that are
not available remain explicit; no mesh-derived measurement is substituted. Optional
material observations are separate from feature picking and require the accuracy
selection described above. The Clear selection button clears selections. The 300-point HUD occupies
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

The display adapter derives its output from actual native surfaces, defining
curves and incidence-specific UV trims. Its first curved controls are the public
phase-split cylinder and cone with a source pole boundary. Computational seams,
source edge pieces and pole/derived vertices retain their native identities and
source correspondence. Display subdivisions do not introduce native/source edges
or replace the stored analytic/NURBS definitions. Unsupported extraction remains an
error with no partial Scene transferred to the GPU.

Triangle shading and boundary polylines are approximate display output. Their
subdivision settings are independent of source acceptance, native working tolerance,
repair and optional material integration controls. This viewer adapter is not a
claim that the complete kernel BRep tessellation driver or arbitrary general STEP
support has been finished.

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

Display triangles nominate native face candidates; actual owning native queries
must confirm a returned hit. Face/edge/vertex selections and measurements retain
original native identity, including source edges split into several pieces and
explicitly derived chart geometry. Missing queries are shown as unavailable, never
replaced with mesh classification. Pixel and float-depth allowances are interaction
settings, not import acceptance budgets. Rendering and picking share one
viewport/matrix, including DPI and near clipping. Queries must use the actual
owning Candidate and its Kernel; no cleared-owner copy or default context is a
valid replacement.

This increment does **not** complete general STEP import or a real-model acceptance
target. The HUD continues to identify the result as an unpublished native candidate.
Source assumptions, current numerical observations and derived display output stay
separate. A screenshot does not establish source equivalence, material-solid
semantics, accepted import or unexecuted query coverage.

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

Public controls cover default P0 loading, explicit General loading of the cylinder
and cone, quality/policy gates, cancellation before delivery, and source-owned unit
compatibility receipts. Camera tests cover extreme zoom/orbit matrices. A real GPU/window
smoke test is additionally necessary for text layout, drawing and interaction evidence.
