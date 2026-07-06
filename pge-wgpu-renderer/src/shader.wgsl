struct CameraUniform {
    view_proj: mat4x4<f32>,
    light_dir: vec4<f32>,
};

struct ObjectUniform {
    model: mat4x4<f32>,
    color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> object: ObjectUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_position = object.model * vec4<f32>(input.position, 1.0);
    out.clip_position = camera.view_proj * world_position;
    out.normal = normalize((object.model * vec4<f32>(input.normal, 0.0)).xyz);
    return out;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(input.normal);
    let l = normalize(camera.light_dir.xyz);
    let diffuse = max(dot(n, -l), 0.0);
    let ambient = 0.35;
    let lit = ambient + diffuse * 0.65;
    return vec4<f32>(object.color.rgb * lit, object.color.a);
}
