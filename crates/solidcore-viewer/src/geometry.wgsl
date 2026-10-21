struct Camera {
    view_projection: mat4x4<f32>,
    viewport: vec2<f32>,
    line_width: f32,
    padding: f32,
};
@group(0) @binding(0) var<uniform> camera: Camera;
struct Output {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec3<f32>,
};
@vertex fn vertex(@location(0) position: vec3<f32>, @location(1) color: vec3<f32>) -> Output {
    var output: Output;
    output.position = camera.view_projection * vec4<f32>(position, 1.0);
    output.color = color;
    return output;
}
@fragment fn fragment(input: Output) -> @location(0) vec4<f32> {
    return vec4<f32>(input.color, 1.0);
}

// Rasterize a native boundary segment as a constant-pixel-width quad. Retain
// its actual clip-space depth: the edge pass still tests against face depth,
// so hidden boundaries are not promoted into an x-ray overlay.
@vertex fn edge(@builtin(vertex_index) corner: u32,
    @location(0) start: vec3<f32>, @location(1) end: vec3<f32>, @location(2) color: vec3<f32>) -> Output {
    var a = camera.view_projection * vec4<f32>(start, 1.0);
    var b = camera.view_projection * vec4<f32>(end, 1.0);
    var output: Output;
    output.color = color;
    if a.z < 0.0 && b.z < 0.0 {
        output.position = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        return output;
    }
    // Clip to WebGPU's z=0 near plane before perspective division.
    if a.z < 0.0 { a = mix(a, b, -a.z / (b.z - a.z)); }
    if b.z < 0.0 { b = mix(b, a, -b.z / (a.z - b.z)); }
    let delta = (b.xy / b.w - a.xy / a.w) * camera.viewport;
    let segment_length = length(delta);
    if segment_length <= 0.0001 || a.w <= 0.0 || b.w <= 0.0 {
        output.position = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        return output;
    }
    let perpendicular = vec2<f32>(-delta.y, delta.x) / segment_length;
    var at_end = false;
    var side = -1.0;
    switch corner {
        case 1u, 4u: { at_end = true; }
        case 2u, 3u: { side = 1.0; }
        case 5u: { at_end = true; side = 1.0; }
        default: {}
    }
    var point = select(a, b, at_end);
    point.x += perpendicular.x * side * camera.line_width / camera.viewport.x * point.w;
    point.y += perpendicular.y * side * camera.line_width / camera.viewport.y * point.w;
    output.position = point;
    return output;
}

struct PointOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) uv: vec2<f32>,
};
@vertex fn point(@builtin(vertex_index) corner: u32,
    @location(0) position: vec3<f32>, @location(1) color: vec3<f32>) -> PointOutput {
    var output: PointOutput;
    output.color = color;
    var uv = vec2<f32>(-1.0,-1.0);
    switch corner {
        case 1u,4u: {uv = vec2<f32>(1.0,-1.0);}
        case 2u,3u: {uv = vec2<f32>(-1.0,1.0);}
        case 5u: {uv = vec2<f32>(1.0,1.0);}
        default: {}
    }
    var clip = camera.view_projection * vec4<f32>(position,1.0);
    output.uv = uv;
    clip.x += uv.x * max(camera.line_width * 3.0,6.0) / camera.viewport.x * clip.w;
    clip.y += uv.y * max(camera.line_width * 3.0,6.0) / camera.viewport.y * clip.w;
    output.position = clip;
    return output;
}
@fragment fn point_fragment(input: PointOutput) -> @location(0) vec4<f32> {
    if dot(input.uv,input.uv) > 1.0 {discard;}
    return vec4<f32>(input.color,1.0);
}
