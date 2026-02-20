use crate::types::*;
use crate::ArenaId;
use crate::State;
use glam::*;

pub fn orbit_offset(yaw: f32, pitch: f32, radius: f32) -> Vec3 {
	let cy = yaw.cos();
	let sy = yaw.sin();
	let cp = pitch.cos();
	let sp = pitch.sin();

	Vec3::new(radius * sy * cp, radius * sp, radius * cy * cp)
}

pub fn orbit_state_from_offset(offset: Vec3) -> (f32, f32, f32) {
	let radius = offset.length();
	if radius == 0.0 {
		return (0.0, 0.0, 0.0);
	}

	let yaw = offset.x.atan2(offset.z);
	let pitch = (offset.y / radius).clamp(-1.0, 1.0).asin();
	(yaw, pitch, radius)
}

#[derive(Debug, Clone)]
pub struct OrbitController {
	pub target: Vec3,
	pub rotation: Quat,
	pub distance: f32,
	pub rot_speed: f32,
	pub pan_speed: f32,
	pub zoom_speed: f32,
	pub min_dist: f32,
	pub max_dist: f32,
	orbit_delta: Vec2,
	pan_delta: Vec2,
	zoom_delta: f32,
}

impl Default for OrbitController {
	fn default() -> Self {
		Self {
			target: Vec3::ZERO,
			rotation: Quat::IDENTITY,
			distance: 3.0,
			rot_speed: 0.01,
			pan_speed: 0.002,
			zoom_speed: 0.12,
			min_dist: 0.05,
			max_dist: 200.0,
			orbit_delta: Vec2::ZERO,
			pan_delta: Vec2::ZERO,
			zoom_delta: 0.0,
		}
	}
}

impl OrbitController {
	pub fn set_from_target_and_position(&mut self, target: Vec3, position: Vec3) {
		self.target = target;
		let offset = position - target;
		self.distance = offset.length().max(self.min_dist);

		let forward = (target - position).normalize_or_zero();
		let world_up = Vec3::Y;

		let mut right = world_up.cross(forward);
		if right.length_squared() < 1e-8 {
			right = Vec3::X;
		} else {
			right = right.normalize();
		}
		let up = forward.cross(right).normalize_or_zero();

		let m = Mat3::from_cols(right, up, forward);
		self.rotation = Quat::from_mat3(&m).normalize();
	}

	pub fn orbit(&mut self, mouse_delta: Vec2) {
		self.orbit_delta += mouse_delta;
	}

	pub fn pan(&mut self, mouse_delta: Vec2) {
		self.pan_delta += mouse_delta;
	}

	pub fn zoom(&mut self, scroll_y: f32) {
		self.zoom_delta += scroll_y;
	}

	pub fn process(&mut self, state: &mut State, camera_node_id: ArenaId<Node>, _dt: f32) {
		if self.orbit_delta != Vec2::ZERO {
			let yaw_delta = self.orbit_delta.x * self.rot_speed;
			let pitch_delta = self.orbit_delta.y * self.rot_speed;

			let right = (self.rotation * Vec3::X).normalize_or_zero();
			let up = (self.rotation * Vec3::Y).normalize_or_zero();

			let q_yaw = Quat::from_axis_angle(up, yaw_delta);
			let q_pitch = Quat::from_axis_angle(right, pitch_delta);
			self.rotation = (q_pitch * q_yaw * self.rotation).normalize();

			crate::log2!("Orbit: delta {:?}", self.orbit_delta);
			self.orbit_delta = Vec2::ZERO;
		}

		if self.pan_delta != Vec2::ZERO {
			let right = self.rotation * Vec3::X;
			let up = self.rotation * Vec3::Y;
			let scale = self.pan_speed * self.distance;
			self.target += (-self.pan_delta.x * right + self.pan_delta.y * up) * scale;
			crate::log2!("Pan: target {:?} delta {:?}", self.target, self.pan_delta);
			self.pan_delta = Vec2::ZERO;
		}

		if self.zoom_delta != 0.0 {
			self.distance *= (-self.zoom_delta * self.zoom_speed).exp();
			self.distance = self.distance.clamp(self.min_dist, self.max_dist);
			crate::log2!("Zoom: distance {:.3} delta {:.3}", self.distance, self.zoom_delta);
			self.zoom_delta = 0.0;
		}

		let forward = (self.rotation * Vec3::Z).normalize_or_zero();
		let offset = -forward * self.distance;
		if let Some(camera_node) = state.nodes.get_mut(&camera_node_id) {
			let prev_translation = camera_node.translation;
			let prev_rotation = camera_node.rotation;
			camera_node.translation = self.target + offset;
			camera_node.rotation = self.rotation;
			if prev_translation != camera_node.translation {
				crate::log2!(
					"Node translation: {:?} -> {:?}",
					prev_translation,
					camera_node.translation
				);
			}
			if prev_rotation != camera_node.rotation {
				crate::log2!(
					"Node rotation: {:?} -> {:?}",
					prev_rotation,
					camera_node.rotation
				);
			}
		}
	}

	fn basis(&self) -> (Vec3, Vec3, Vec3) {
		let forward = (self.rotation * Vec3::Z).normalize_or_zero();
		let right = (self.rotation * Vec3::X).normalize_or_zero();
		let up = forward.cross(right).normalize_or_zero();
		(right, up, forward)
	}
}

fn orbit_rotation(yaw: f32, pitch: f32) -> Quat {
	let forward = orbit_forward(yaw, pitch);
	let (right, up) = orbit_axes(yaw, forward);
	let rotation_matrix = Mat3::from_cols(right, up, forward);
	Quat::from_mat3(&rotation_matrix)
}

fn orbit_forward(yaw: f32, pitch: f32) -> Vec3 {
	let (sy, cy) = yaw.sin_cos();
	let (sp, cp) = pitch.sin_cos();
	Vec3::new(-sy * cp, -sp, -cy * cp)
}

fn orbit_axes(yaw: f32, forward: Vec3) -> (Vec3, Vec3) {
	let (sy, cy) = yaw.sin_cos();
	let right = Vec3::new(-cy, 0.0, sy);
	let up = forward.cross(right).normalize_or_zero();
	(right, up)
}

#[cfg(test)]
mod tests {
	use super::*;
	use rand::rngs::StdRng;
	use rand::{Rng, SeedableRng};

	fn assert_approx_eq(a: f32, b: f32, eps: f32) {
		assert!((a - b).abs() <= eps, "expected {a} ~= {b}");
	}

	fn assert_vec3_approx_eq(a: Vec3, b: Vec3, eps: f32) {
		assert!((a - b).length() <= eps, "expected {a:?} ~= {b:?}");
	}

	#[test]
	fn orbit_offset_round_trip() {
		let yaw = 1.1;
		let pitch = -0.6;
		let radius = 3.5;

		let offset = orbit_offset(yaw, pitch, radius);
		let (round_yaw, round_pitch, round_radius) = orbit_state_from_offset(offset);

		assert_approx_eq(round_yaw, yaw, 1e-4);
		assert_approx_eq(round_pitch, pitch, 1e-4);
		assert_approx_eq(round_radius, radius, 1e-4);
	}

	#[test]
	fn orbit_state_handles_zero_offset() {
		let (yaw, pitch, radius) = orbit_state_from_offset(Vec3::ZERO);
		assert_eq!(yaw, 0.0);
		assert_eq!(pitch, 0.0);
		assert_eq!(radius, 0.0);
	}

	#[test]
	fn orbit_controller_updates_node() {
		let mut state = State::default();
		let camera_node_id = state.nodes.insert(Node::default());
		let mut controller = OrbitController::default();
		controller.target = Vec3::new(1.0, 2.0, 3.0);
		controller.distance = 4.0;
		controller.rotation = orbit_rotation(0.7, -0.2);

		controller.process(&mut state, camera_node_id, 0.0);

		let camera_node = state.nodes.get(&camera_node_id).unwrap();
		let expected_pos = controller.target - (controller.rotation * Vec3::Z) * controller.distance;
		assert_vec3_approx_eq(camera_node.translation, expected_pos, 1e-4);

		let expected_forward = (controller.target - camera_node.translation).normalize_or_zero();
		let node_forward = camera_node.rotation * Vec3::Z;
		assert_vec3_approx_eq(node_forward, expected_forward, 1e-4);
	}

	#[test]
	fn orbit_vertical_then_horizontal_stays_valid() {
		let mut state = State::default();
		let camera_node_id = state.nodes.insert(Node::default());
		let mut controller = OrbitController::default();
		controller.target = Vec3::ZERO;
		controller.distance = 5.0;
		controller.rot_speed = 0.01;

		controller.orbit(Vec2::new(0.0, 200.0));
		controller.process(&mut state, camera_node_id, 0.0);

		controller.orbit(Vec2::new(300.0, 0.0));
		controller.process(&mut state, camera_node_id, 0.0);

		let camera_node = state.nodes.get(&camera_node_id).unwrap();
		println!("Camera node: {:#?}", camera_node);
		let forward = camera_node.rotation * Vec3::Z;
		assert!(forward.is_finite());
		assert_approx_eq(forward.length(), 1.0, 1e-3);
		assert!(camera_node.translation.is_finite());
	}

	#[test]
	fn orbit_randomized_steps_stay_valid() {
		let mut state = State::default();
		let camera_node_id = state.nodes.insert(Node::default());
		let mut controller = OrbitController::default();
		controller.target = Vec3::ZERO;
		controller.distance = 5.0;

		let mut rng = StdRng::seed_from_u64(0x5eeda1u64);
		for _ in 0..1000 {
			if rng.gen_bool(0.7) {
				let delta = Vec2::new(rng.gen_range(-8.0..8.0), rng.gen_range(-8.0..8.0));
				controller.orbit(delta);
			}
			if rng.gen_bool(0.6) {
				let delta = Vec2::new(rng.gen_range(-6.0..6.0), rng.gen_range(-6.0..6.0));
				controller.pan(delta);
			}
			if rng.gen_bool(0.6) {
				controller.zoom(rng.gen_range(-2.0..2.0));
			}

			controller.process(&mut state, camera_node_id, 0.016);

			let camera_node = state.nodes.get(&camera_node_id).unwrap();
			let forward = camera_node.rotation * Vec3::Z;
			assert!(forward.is_finite());
			assert_approx_eq(forward.length(), 1.0, 1e-3);
			assert!(camera_node.translation.is_finite());
			assert!(controller.distance >= controller.min_dist - 1e-4);
			assert!(controller.distance <= controller.max_dist + 1e-4);
		}
	}
}
