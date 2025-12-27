use std::collections::HashSet;
use std::time::Duration;

use args::Command;
use clap::Parser;
use pge::*;
mod args;

struct SceneViewer {
	window_id: ArenaId<Window>,
	scene_id: ArenaId<Scene>,
	camera_node_id: ArenaId<Node>,
	orbit_center: Vec3,
	right_button_down: bool,
	last_cursor_offset: Option<Vec2>,
}

impl SceneViewer {
	fn new(state: &mut State, scene_id: ArenaId<Scene>) -> Self {
		let scene = state.scenes.get_mut(&scene_id).unwrap();
		scene.scale = Vec3::new(10.0, 10.0, 10.0);
		let name = scene.name.clone().unwrap_or_default();
		log::info!("Scene added: {:?}", scene_id);
		log::info!("scene bounding box: {:?}", state.get_scene_bounding_box(scene_id));
		for (_, node) in state.nodes.iter_mut().filter(|(_, node)| node.parent == NodeParent::Scene(scene_id)) {
			node.scale = Vec3::new(10.0, 10.0, 10.0);
		}

		let mut light_node = Node::new();
		light_node.parent = NodeParent::Scene(scene_id);
		light_node.translation = Vec3::new(0.0, 5.0,-5.0);
		let light_node_id = state.nodes.insert(light_node);
		let mut light = PointLight::new();
		light.node_id = Some(light_node_id);
		state.point_lights.insert(light);

		let scene_bounding_box = state.get_scene_bounding_box(scene_id);
        let center = (scene_bounding_box.min + scene_bounding_box.max) * 0.5;
        let size = scene_bounding_box.max - scene_bounding_box.min;
        let max_size = size.x.max(size.y).max(size.z);

		// Define camera FOV (in degrees)
		let fov_degrees = 60.0_f32;
		let fov_radians = fov_degrees.to_radians();
		let distance = (max_size / 2.0) / fov_radians.tan();
		log::info!("distance: {}", distance);

		let mut camera_node = Node::new();
		camera_node.translation = Vec3::new(0.0, 0.0, 3.0);
		camera_node.looking_at(0.0, 0.0, 0.0);
		camera_node.parent = NodeParent::Scene(scene_id);
		let camera_transform = camera_node.matrix();
		let camera_node_id = state.nodes.insert(camera_node);

		let mut camera = Camera::new();
		camera.fovy = fov_radians;
		camera.node_id = Some(camera_node_id);
		let view_rect = camera.view_rect(camera_transform);
		log::info!("view rect: {:?}", view_rect);
		let camera_id = state.cameras.insert(camera);

		let ui = camera_view(camera_id);
		let ui_id = state.guis.insert(ui);

		let window = Window::new().title(&name).ui(ui_id);
		let window_id = state.windows.insert(window);
		Self {
			window_id,
			scene_id,
			camera_node_id,
			orbit_center: center,
			right_button_down: false,
			last_cursor_offset: None,
		}
	}

	fn on_mouse_input(&mut self, event: MouseEvent, state: &mut State) {
		match event {
			MouseEvent::Moved { dx, dy } => {
				let current_offset = Vec2::new(dx, dy);
				if self.right_button_down {
					if let Some(prev_offset) = self.last_cursor_offset {
						let delta = current_offset - prev_offset;
						let sensitivity = 0.005;
						let yaw = Quat::from_axis_angle(Vec3::Y, -delta.x * sensitivity);
						let camera_node = state.nodes.get_mut(&self.camera_node_id).unwrap();
						let mut offset = camera_node.translation - self.orbit_center;
						let forward = (-offset).normalize_or_zero();
						let mut right = forward.cross(Vec3::Y).normalize_or_zero();
						if right.length_squared() == 0.0 {
							right = Vec3::X;
						}
						let pitch = Quat::from_axis_angle(right, -delta.y * sensitivity);
						offset = (yaw * pitch) * offset;
						camera_node.translation = self.orbit_center + offset;
						camera_node.looking_at(
							self.orbit_center.x,
							self.orbit_center.y,
							self.orbit_center.z,
						);
					}
					self.last_cursor_offset = Some(current_offset);
				} else {
					self.last_cursor_offset = Some(current_offset);
				}
			},
			MouseEvent::Pressed { button } => {
				if let MouseButton::Right = button {
					self.right_button_down = true;
					self.last_cursor_offset = None;
				}
			}
			MouseEvent::Released { button } => {
				if let MouseButton::Right = button {
					self.right_button_down = false;
					self.last_cursor_offset = None;
				}
			}
			MouseEvent::Wheel { dx, dy } => {
				log::info!("scroll delta: {:?}", (dx, dy));
				let camera_node = state.nodes.get_mut(&self.camera_node_id).unwrap();
				camera_node.translation += Vec3::new(0.0, 0.0, dy * 0.005);
			}
		}
	}
}

struct PgeEditor {
	asset_path: Option<String>,
	windows: Vec<ArenaId<Window>>,
	scenes: HashSet<ArenaId<Scene>>,
	scene_viewers: Vec<SceneViewer>,
}

impl PgeEditor {
	fn new() -> Self {
		Self {
			asset_path: None,
			windows: Vec::new(),
			scenes: HashSet::new(),
			scene_viewers: Vec::new(),
		}
	}

	pub fn set_inspect_path(&mut self, path: String) {
		self.asset_path = Some(path);
	}
}

impl pge::App for PgeEditor {
	fn on_create(&mut self, state: &mut State) {
		if let Some(path) = &self.asset_path {
			state.load_3d_model(path);
		}
	}

	fn on_process(&mut self, state: &mut State, delta: f32) {
		let mut new_scene_ids = Vec::new();
		for (scene_id,_) in state.scenes.iter() {
			if self.scenes.contains(&scene_id) {
				continue;
			}
			let node_count = state.nodes.iter().filter(|(_, node)| node.scene_id == Some(scene_id)).count();
			if node_count == 0 {
				continue;
			}
			log::info!("scene {:?} has {} nodes", scene_id, node_count);
			new_scene_ids.push(scene_id);
			self.scenes.insert(scene_id);
		}
		for scene_id in new_scene_ids {
			let scene_viewer = SceneViewer::new(state, scene_id);
			self.scene_viewers.push(scene_viewer);
		}
	}

	fn on_mouse_input(&mut self, window_id: ArenaId<Window>, event: MouseEvent, state: &mut State) {
		let scene_viewer = match self.scene_viewers.iter_mut().find(|v| v.window_id == window_id) {
			Some(v) => v,
			None => return,
		};
		scene_viewer.on_mouse_input(event, state);
	}
}

fn main() {
    pge::init_logging();

	let mut editor = PgeEditor::new();

	let args = args::Args::parse();

	if let Some(command) = args.command {
		match command {
			Command::Inspect { path } => {
				editor.set_inspect_path(path);
			}
		}
	}

	pge::run(editor).unwrap();
}
