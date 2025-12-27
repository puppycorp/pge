use std::path::PathBuf;

use pge::*;
use pge::OrbitController;

#[derive(Default)]
struct PuppyArmExample {
	scene_id: Option<ArenaId<Scene>>,
	camera_node_id: Option<ArenaId<Node>>,
	light_node_id: Option<ArenaId<Node>>,
	framed: bool,
	orbit_controller: OrbitController,
	right_button_down: bool,
	middle_button_down: bool,
}

impl pge::App for PuppyArmExample {
	fn on_create(&mut self, state: &mut pge::State) {
		self.orbit_controller.rot_speed = 0.0005;

		let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.join("../../assets/puppyarm/puppyarm.urdf");
		let scene_id = state.load_urdf(&urdf_path);
		self.scene_id = Some(scene_id);

		let mut light_node = Node::new();
		light_node.name = Some("Light".to_string());
		light_node.set_translation(2.0, 2.0, 2.0);
		light_node.parent = NodeParent::Scene(scene_id);
		let light_node_id = state.nodes.insert(light_node);
		self.light_node_id = Some(light_node_id);
		let mut light = PointLight::new();
		light.node_id = Some(light_node_id);
		state.point_lights.insert(light);

		let mut camera_node = Node::new();
		camera_node.name = Some("Camera".to_string());
		camera_node.set_translation(0.0, 0.5, 1.5);
		camera_node.looking_at(0.0, 0.0, 0.0);
		camera_node.parent = NodeParent::Scene(scene_id);
		let camera_node_id = state.nodes.insert(camera_node);
		self.camera_node_id = Some(camera_node_id);

		let mut camera = Camera::new();
		camera.zfar = 100.0;
		camera.node_id = Some(camera_node_id);
		let camera_id = state.cameras.insert(camera);

		let gui_id = state.guis.insert(camera_view(camera_id));
		state.windows.insert(window().title("Puppyarm").ui(gui_id));

		state.print_state();
	}

	fn on_process(&mut self, state: &mut pge::State, _dt: f32) {
		if !self.framed {
			let scene_id = match self.scene_id {
				Some(id) => id,
				None => return,
			};
			let aabb = state.get_scene_bounding_box(scene_id);
			if aabb.min == aabb.max {
				return;
			}

			let center = (aabb.min + aabb.max) * 0.5;
			let extents = aabb.max - aabb.min;
			let radius = extents.length() * 0.5;
			let distance = (radius * 3.0).max(0.3);
			let cam_pos = center + Vec3::new(distance, distance * 0.5, distance);

			if let Some(camera_node_id) = self.camera_node_id {
				if let Some(camera_node) = state.nodes.get_mut(&camera_node_id) {
					camera_node.translation = cam_pos;
					camera_node.looking_at(center.x, center.y, center.z);
				}
			}
			if let Some(light_node_id) = self.light_node_id {
				if let Some(light_node) = state.nodes.get_mut(&light_node_id) {
					light_node.translation = center + Vec3::new(distance, distance, distance);
				}
			}

			self.orbit_controller.set_from_target_and_position(center, cam_pos);
			self.framed = true;
		}

		if let Some(camera_node_id) = self.camera_node_id {
			self.orbit_controller.process(state, camera_node_id, _dt);
		}
	}

	fn on_mouse_input(
		&mut self,
		_window_id: ArenaId<Window>,
		event: MouseEvent,
		_state: &mut pge::State,
	) {
		match event {
			MouseEvent::Moved { dx, dy } => {
				println!("Mouse moved: dx {}, dy {}", dx, dy);
				let delta = Vec2::new(dx, dy);
				if self.right_button_down {
					self.orbit_controller.orbit(delta);
				} else if self.middle_button_down {
					self.orbit_controller.pan(delta);
				}
			}
			MouseEvent::Pressed { button } => {
				if let MouseButton::Right = button {
					self.right_button_down = true;
				}
				if let MouseButton::Middle = button {
					self.middle_button_down = true;
				}
			}
			MouseEvent::Released { button } => {
				if let MouseButton::Right = button {
					self.right_button_down = false;
				}
				if let MouseButton::Middle = button {
					self.middle_button_down = false;
				}
			}
			MouseEvent::Wheel { dx: _, dy } => {
				self.orbit_controller.zoom(dy);
			}
		}
	}
}

fn main() {
	pge::init_logging();
	pge::run(PuppyArmExample::default()).unwrap();
}
