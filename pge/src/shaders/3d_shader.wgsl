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
}
@group(0) @binding(0)
var<storage, read> camera: Camera;

struct PointLight {
	color: vec3<f32>,
	// Padding to align to 16 bytes
	_padding: f32, 
	intensity: f32,
	position: vec3<f32>,
	// Padding to align to 16 bytes
	_padding2: f32,
};

struct Material {
	base_color_factor: vec4<f32>,
	metallic_factor: f32,
	roughness_factor: f32,
	emissive_factor: vec3<f32>,
};

@group(1) @binding(0)
var<storage, read> point_lights: array<PointLight>;

@vertex
fn vs_main(input: VertexInput, instance: InstanceInput) -> VertexOutput {
	let instance_model = mat4x4<f32>(
        instance.model_matrix_0,
        instance.model_matrix_1,
        instance.model_matrix_2,
        instance.model_matrix_3,
    );

    var out: VertexOutput;
    let world_position = (instance_model * vec4<f32>(input.position, 1.0)).xyz;
    out.clip_position = camera.model * vec4<f32>(world_position, 1.0);
    out.color = vec3(1.0, 0.0, 0.0); // Placeholder for color, to be modified by lighting calculation
    out.world_position = world_position;
	let normal = input.normal;
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
    let light_color = vec3<f32>(1.0, 1.0, 1.0);
    let view_dir = normalize(camera.model[3].xyz - in.world_position);
    var diffuse = vec3<f32>(0.0, 0.0, 0.0);
    var specular = vec3<f32>(0.0, 0.0, 0.0);

    let texture_color = textureSample(base_color_texture, base_color_sampler, in.tex_coords);
    let base_color = texture_color.rgb * material.base_color_factor.rgb;
    let roughness = material.roughness_factor;
    let metallic = material.metallic_factor;

    for (var i = 0u; i < 2; i = i + 1u) {
        let point_light = point_lights[i];
        let light_position = point_light.position;
        let light_dir = normalize(light_position - in.world_position);
        let halfway_dir = normalize(light_dir + view_dir);

        // Diffuse
        let ndotl = max(dot(in.normal, light_dir), 0.0);
        diffuse += ndotl * light_color;

        // Blinn-Phong
        let ndoth = max(dot(in.normal, halfway_dir), 0.0);
        let spec = pow(ndoth, (1.0 - roughness) * 128.0); // Higher exponent for smoother surfaces
        specular += spec * light_color;
    }

    // **Combine Diffuse and Specular with Material Properties**
    // Adjust specular intensity based on metallic factor
    let final_color = (diffuse * base_color) + (specular * mix(vec3<f32>(0.04), base_color, metallic));
    // Incorporate the alpha component from base_color_factor
    return vec4<f32>(final_color, material.base_color_factor.a);
}