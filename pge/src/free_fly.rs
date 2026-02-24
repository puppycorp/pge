use crate::ArenaId;
use crate::Node;
use crate::State;
use glam::EulerRot;
use glam::Quat;
use glam::Vec2;
use glam::Vec3;

#[derive(Debug, Clone, Copy, Default)]
pub struct FreeFlyMoveInput {
	pub right: f32,
	pub up: f32,
	pub forward: f32,
	pub fast: bool,
}

#[derive(Debug, Clone)]
pub struct FreeFlyController {
	pub position: Vec3,
	pub yaw: f32,
	pub pitch: f32,
	pub move_speed: f32,
	pub fast_multiplier: f32,
	pub keyboard_look_speed: f32,
	pub mouse_look_sensitivity: f32,
	pub min_pitch: f32,
	pub max_pitch: f32,
}

impl Default for FreeFlyController {
	fn default() -> Self {
		Self {
			position: Vec3::ZERO,
			yaw: 0.0,
			pitch: 0.0,
			move_speed: 2.0,
			fast_multiplier: 3.0,
			keyboard_look_speed: 1.4,
			mouse_look_sensitivity: 0.01,
			min_pitch: -1.553343, // ~ -89 deg
			max_pitch: 1.553343,  // ~ 89 deg
		}
	}
}

impl FreeFlyController {
	pub fn set_from_transform(&mut self, position: Vec3, rotation: Quat) {
		self.position = position;
		let forward = (rotation * Vec3::Z).normalize_or_zero();
		self.yaw = forward.x.atan2(forward.z);
		self.pitch = forward.y.clamp(-1.0, 1.0).asin();
	}

	pub fn set_from_target_and_position(&mut self, target: Vec3, position: Vec3) {
		self.position = position;
		let mut forward = (target - position).normalize_or_zero();
		if forward.length_squared() <= f32::EPSILON {
			forward = Vec3::Z;
		}
		self.yaw = forward.x.atan2(forward.z);
		self.pitch = forward.y.clamp(-1.0, 1.0).asin();
	}

	pub fn rotation(&self) -> Quat {
		Quat::from_euler(EulerRot::YXZ, self.yaw, self.pitch, 0.0)
	}

	pub fn look_mouse(&mut self, mouse_delta: Vec2) {
		self.yaw += mouse_delta.x * self.mouse_look_sensitivity;
		self.pitch -= mouse_delta.y * self.mouse_look_sensitivity;
		self.pitch = self.pitch.clamp(self.min_pitch, self.max_pitch);
	}

	pub fn look_keyboard(&mut self, yaw_dir: f32, pitch_dir: f32, dt: f32) {
		if yaw_dir == 0.0 && pitch_dir == 0.0 {
			return;
		}

		self.yaw += yaw_dir * self.keyboard_look_speed * dt;
		self.pitch += pitch_dir * self.keyboard_look_speed * dt;
		self.pitch = self.pitch.clamp(self.min_pitch, self.max_pitch);
	}

	pub fn move_local(&mut self, input: FreeFlyMoveInput, dt: f32) {
		let mut direction = Vec3::ZERO;
		if input.right != 0.0 || input.forward != 0.0 {
			let rotation = self.rotation();
			let right = rotation * Vec3::X;
			let forward = rotation * Vec3::Z;
			direction += right * input.right + forward * input.forward;
		}
		direction += Vec3::Y * input.up;

		if direction.length_squared() <= f32::EPSILON {
			return;
		}

		let mut speed = self.move_speed;
		if input.fast {
			speed *= self.fast_multiplier;
		}
		self.position += direction.normalize() * speed * dt;
	}

	pub fn apply_to_node(&self, state: &mut State, camera_node_id: ArenaId<Node>) {
		if let Some(camera_node) = state.nodes.get_mut(&camera_node_id) {
			camera_node.translation = self.position;
			camera_node.rotation = self.rotation();
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn assert_approx(a: f32, b: f32, eps: f32) {
		assert!((a - b).abs() <= eps, "expected {a} ~= {b}");
	}

	#[test]
	fn forward_movement_uses_camera_forward() {
		let mut controller = FreeFlyController::default();
		controller.yaw = 0.0;
		controller.pitch = 0.0;
		controller.move_local(
			FreeFlyMoveInput {
				forward: 1.0,
				..Default::default()
			},
			1.0,
		);
		assert!(controller.position.z > 1.9);
		assert_approx(controller.position.x, 0.0, 1e-5);
	}

	#[test]
	fn fast_movement_scales_speed() {
		let mut slow = FreeFlyController::default();
		let mut fast = FreeFlyController::default();

		slow.move_local(
			FreeFlyMoveInput {
				forward: 1.0,
				fast: false,
				..Default::default()
			},
			1.0,
		);
		fast.move_local(
			FreeFlyMoveInput {
				forward: 1.0,
				fast: true,
				..Default::default()
			},
			1.0,
		);

		assert!(fast.position.length() > slow.position.length());
	}
}
