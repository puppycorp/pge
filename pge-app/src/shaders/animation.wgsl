struct Keyframe {
    value: mat4x4<f32>,
    is_running: u32,
	node_inx: u32
};

struct NodeTransform {
    model: mat4x4<f32>,
    parent_index: i32
};

struct NodeChange {
    model: mat4x4<f32>,
    waiting: u32
}

@group(0) @binding(0) 
var<storage, read> keyframes: array<Keyframe>;
// @group(0) @binding(1)
// var<uniform> current_time: f32;
@group(1) @binding(0)
var<storage, read_write> node_transforms: array<NodeTransform>;
@group(2) @binding(0)
var<storage, read_write> node_changes: array<NodeChange>;

// Linear Interpolation Function for mat4x4
fn mat4_lerp(a: mat4x4<f32>, b: mat4x4<f32>, t: f32) -> mat4x4<f32> {
    return mat4x4<f32>(
        mix(a[0], b[0], t),
        mix(a[1], b[1], t),
        mix(a[2], b[2], t),
        mix(a[3], b[3], t)
    );
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = id.x;

    if index >= arrayLength(&node_transforms) {
        return;
    }

	// var i = 0u;
	// while (i < arrayLength(&keyframes) - 1u && keyframes[i + 1u].start_time <= current_time) {
	// 	i = i + 1u;
	// }

	var node_change = node_changes[index];
    if node_change.waiting == 1u {
        node_changes[index].waiting = 0u;
		node_transforms[index].model = node_change.model * node_transforms[index].model;
    }

	// let keyframe = keyframes[0];

	// if keyframe.is_running == 0u {
	// 	return;
	// }

	// if keyframe.node_inx != index {
	// 	return;
	// }

	// let sensitivity = 10.0;
	// let scaling_factor = sensitivity * current_time;

	// let scaling_matrix: mat4x4<f32> = mat4x4<f32>(
	// 	vec4<f32>(1.0, 0.0, 0.0, 0.0),
	// 	vec4<f32>(0.0, 1.0, 0.0, 0.0),
	// 	vec4<f32>(0.0, 0.0, 1.0, 0.0),
	// 	vec4<f32>(0.0, 0.0, 0.0, 1.0)
	// );

	// var scaled_transform: mat4x4<f32> = keyframe.value * scaling_matrix;
	// scaled_transform[3][0] = keyframe.value[3][0] * scaling_factor;
	// scaled_transform[3][1] = keyframe.value[3][1] * scaling_factor;
	// scaled_transform[3][2] = keyframe.value[3][2] * scaling_factor;
	
    // node_transforms[index].model = scaled_transform * node_transforms[index].model;
}


    // var i = 0u;
    // while (i < arrayLength(&keyframes) - 1u && keyframes[i + 1u].start_time <= current_time) {
    //     i = i + 1u;
    // }

    // let kf1 = keyframes[i];
    // let kf2 = keyframes[i + 1u];

    // if kf1.is_running == 0u || kf1.node_id != index {
    //     // If the animation is not running or the keyframe does not belong to this node, skip
    //     return;
    // }

    // // Adjust current time relative to the start time of the animation
    // let relative_time = current_time - kf1.start_time;

    // // Handle repeating logic
    // var adjusted_time = relative_time;
    // if (kf1.repeat == 1u) {
    //     let duration = kf2.time - kf1.time;
    //     adjusted_time = relative_time % duration;
    // }

    // if (relative_time < 0.0) {
    //     // Animation hasn't started yet
    //     return;
    // }

    // // Calculate the interpolation factor
    // let t = (adjusted_time - kf1.time) / (kf2.time - kf1.time);

    // // Interpolate between the keyframes
    // let interpolated_value = mat4_lerp(kf1.value, kf2.value, t);

    // // Apply the interpolated value to the node transformation
    // node_transforms[index].model = interpolated_value;