struct InstanceInput {
    @location(5) model_matrix_0: vec4<f32>,
    @location(6) model_matrix_1: vec4<f32>,
    @location(7) model_matrix_2: vec4<f32>,
    @location(8) model_matrix_3: vec4<f32>,
};

struct VertexInput {
    @location(0) position: vec3<f32>,
	@location(1) normal: vec3<f32>,
	@location(2) tex_coords: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
	@location(1) world_position: vec3<f32>,
    @location(2) normal: vec3<f32>,
	@location(3) tex_coords: vec2<f32>,
};

struct Camera {
    model: mat4x4<f32>,
    position: vec3<f32>,
    _padding: f32,
}
@group(0) @binding(0)
var<storage, read> camera: Camera;

const MAX_POINT_LIGHTS: u32 = 16u;

struct PointLight {
	color_intensity: vec4<f32>,
	position: vec4<f32>,
};

struct PointLightBuffer {
	count: u32,
	_padding: array<u32, 3>,
	lights: array<PointLight, MAX_POINT_LIGHTS>,
};

struct Material {
	base_color_factor: vec4<f32>,
	metallic_factor: f32,
	roughness_factor: f32,
	emissive_factor: vec3<f32>,
};

@group(1) @binding(0)
var<storage, read> point_lights: PointLightBuffer;

@vertex
fn vs_main(input: VertexInput, instance: InstanceInput) -> VertexOutput {
	let instance_model = mat4x4<f32>(
        instance.model_matrix_0,
        instance.model_matrix_1,
        instance.model_matrix_2,
        instance.model_matrix_3,
    );
	let c0 = instance.model_matrix_0.xyz;
	let c1 = instance.model_matrix_1.xyz;
	let c2 = instance.model_matrix_2.xyz;
	let normal_matrix = mat3x3<f32>(
		cross(c1, c2),
		cross(c2, c0),
		cross(c0, c1),
	);

    var out: VertexOutput;
    let world_position = (instance_model * vec4<f32>(input.position, 1.0)).xyz;
    out.clip_position = camera.model * vec4<f32>(world_position, 1.0);
    out.color = vec3(1.0, 0.0, 0.0); // Placeholder for color, to be modified by lighting calculation
    out.world_position = world_position;
	let normal = normalize(normal_matrix * input.normal);
	out.normal = normal;
	out.tex_coords = input.tex_coords;
    return out;
}

@group(2) @binding(0)
var base_color_texture: texture_2d<f32>;
@group(2) @binding(1)
var base_color_sampler: sampler;

@group(3) @binding(0)
var metallic_roughness_texture: texture_2d<f32>;
@group(3) @binding(1)
var metallic_roughness_sampler: sampler;

@group(4) @binding(0)
var normal_texture: texture_2d<f32>;
@group(4) @binding(1)
var normal_sampler: sampler;

@group(5) @binding(0)
var occlusion_texture: texture_2d<f32>;
@group(5) @binding(1)
var occlusion_sampler: sampler;

@group(6) @binding(0)
var emissive_texture: texture_2d<f32>;
@group(6) @binding(1)
var emissive_sampler: sampler;

@group(7) @binding(0)
var<storage, read> material: Material;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let view_dir = normalize(camera.position - in.world_position);
    let normal = normalize(in.normal);
    let ambient = 0.05;
    var diffuse = vec3<f32>(ambient, ambient, ambient);
    var specular = vec3<f32>(0.0, 0.0, 0.0);

    let texture_color = textureSample(base_color_texture, base_color_sampler, in.tex_coords);
    let base_color = texture_color.rgb * material.base_color_factor.rgb;
    let roughness = clamp(material.roughness_factor, 0.04, 1.0);
    let metallic = material.metallic_factor;

	let point_light_count = min(point_lights.count, MAX_POINT_LIGHTS);
	for (var i = 0u; i < point_light_count; i = i + 1u) {
		let point_light = point_lights.lights[i];
		let light_position = point_light.position.xyz;
		let light_color = point_light.color_intensity.xyz;
		let light_intensity = point_light.color_intensity.w;
		let light_delta = light_position - in.world_position;
		let light_dir = normalize(light_delta);
        let distance_sq = max(dot(light_delta, light_delta), 0.01);
		let light_radiance = light_color * light_intensity * (1.0 / distance_sq);
        let halfway_dir = normalize(light_dir + view_dir);

        // Diffuse
        let ndotl = max(dot(normal, light_dir), 0.0);
        diffuse += ndotl * light_radiance;

        // Blinn-Phong
        let ndoth = max(dot(normal, halfway_dir), 0.0);
        let spec = pow(ndoth, (1.0 - roughness) * 128.0); // Higher exponent for smoother surfaces
        specular += spec * light_radiance;
    }

    // **Combine Diffuse and Specular with Material Properties**
    // Adjust specular intensity based on metallic factor
    let final_color = (diffuse * base_color) + (specular * mix(vec3<f32>(0.04), base_color, metallic));
    // Incorporate the alpha component from base_color_factor
    return vec4<f32>(final_color, material.base_color_factor.a);
}
