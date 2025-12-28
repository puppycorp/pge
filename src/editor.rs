use std::collections::HashSet;
use std::path::Path;

use crate::*;

#[derive(Debug, Clone, Default)]
pub struct EditorSettings {
	pub add_light: bool,
	pub scene_scale: Option<Vec3>,
}

struct SceneViewer {
	window_id: ArenaId<Window>,
	camera_node_id: ArenaId<Node>,
	orbit_controller: OrbitController,
	right_button_down: bool,
	middle_button_down: bool,
}

impl SceneViewer {
	fn new(state: &mut State, scene_id: ArenaId<Scene>, settings: &EditorSettings) -> Self {
		if let Some(scale) = settings.scene_scale {
			if let Some(scene) = state.scenes.get_mut(&scene_id) {
				scene.scale = scale;
			}
		}

		if settings.add_light {
			let mut light_node = Node::new();
			light_node.parent = NodeParent::Scene(scene_id);
			light_node.translation = Vec3::new(0.0, 5.0, -5.0);
			let light_node_id = state.nodes.insert(light_node);
			let mut light = PointLight::new();
			light.node_id = Some(light_node_id);
			state.point_lights.insert(light);
		}

		let scene_bounding_box = state.get_scene_bounding_box(scene_id);
		let center = (scene_bounding_box.min + scene_bounding_box.max) * 0.5;
		let size = scene_bounding_box.max - scene_bounding_box.min;
		let max_size = size.x.max(size.y).max(size.z);

		let fov_degrees = 60.0_f32;
		let fov_radians = fov_degrees.to_radians();
		let distance = if max_size > 0.0 {
			(max_size / 2.0) / fov_radians.tan()
		} else {
			3.0
		};

		let camera_pos = center + Vec3::new(0.0, 0.0, distance.max(0.1));
		let mut camera_node = Node::new();
		camera_node.translation = camera_pos;
		camera_node.looking_at(center.x, center.y, center.z);
		camera_node.parent = NodeParent::Scene(scene_id);
		let camera_node_id = state.nodes.insert(camera_node);

		let mut camera = Camera::new();
		camera.fovy = fov_radians;
		camera.node_id = Some(camera_node_id);
		let camera_id = state.cameras.insert(camera);

		let ui = camera_view(camera_id);
		let ui_id = state.guis.insert(ui);

		let scene = state.scenes.get(&scene_id).unwrap();
		let name = scene.name.clone().unwrap_or_default();
		let window = Window::new().title(&name).ui(ui_id);
		let window_id = state.windows.insert(window);

		let mut orbit_controller = OrbitController::default();
		orbit_controller.set_from_target_and_position(center, camera_pos);

		Self {
			window_id,
			camera_node_id,
			orbit_controller,
			right_button_down: false,
			middle_button_down: false,
		}
	}

	fn on_process(&mut self, state: &mut State, dt: f32) {
		self.orbit_controller
			.process(state, self.camera_node_id, dt);
	}

	fn on_mouse_input(&mut self, event: MouseEvent) {
		match event {
			MouseEvent::Moved { dx, dy } => {
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

pub struct EditorPlugin {
	asset_path: Option<String>,
	scenes: HashSet<ArenaId<Scene>>,
	scene_viewers: Vec<SceneViewer>,
	pub settings: EditorSettings,
}

impl EditorPlugin {
	pub fn new() -> Self {
		Self {
			asset_path: None,
			scenes: HashSet::new(),
			scene_viewers: Vec::new(),
			settings: EditorSettings::default(),
		}
	}

	pub fn set_inspect_path<P: AsRef<Path>>(&mut self, path: P) {
		self.asset_path = Some(path.as_ref().to_string_lossy().to_string());
	}

	pub fn on_create(&mut self, state: &mut State) {
		if let Some(path) = &self.asset_path {
			let ext = Path::new(path)
				.extension()
				.and_then(|ext| ext.to_str())
				.unwrap_or_default();
			if ext.eq_ignore_ascii_case("urdf") {
				state.load_urdf(path);
			} else {
				state.load_3d_model(path);
			}
		}
	}

	pub fn on_process(&mut self, state: &mut State, dt: f32) {
		let mut new_scene_ids = Vec::new();
		for (scene_id, _) in state.scenes.iter() {
			if self.scenes.contains(&scene_id) {
				continue;
			}
			let node_count = state
				.nodes
				.iter()
				.filter(|(_, node)| node.scene_id == Some(scene_id))
				.count();
			if node_count == 0 {
				continue;
			}
			new_scene_ids.push(scene_id);
			self.scenes.insert(scene_id);
		}
		for scene_id in new_scene_ids {
			let scene_viewer = SceneViewer::new(state, scene_id, &self.settings);
			self.scene_viewers.push(scene_viewer);
		}
		for scene_viewer in &mut self.scene_viewers {
			scene_viewer.on_process(state, dt);
		}
	}

	pub fn on_mouse_input(
		&mut self,
		window_id: ArenaId<Window>,
		event: MouseEvent,
	) {
		let scene_viewer = match self
			.scene_viewers
			.iter_mut()
			.find(|v| v.window_id == window_id)
		{
			Some(v) => v,
			None => return,
		};
		scene_viewer.on_mouse_input(event);
	}
}

impl Default for EditorPlugin {
	fn default() -> Self {
		Self::new()
	}
}

pub struct EditorApp<T: App> {
	app: T,
	editor: EditorPlugin,
}

impl<T: App> EditorApp<T> {
	pub fn new(app: T) -> Self {
		Self {
			app,
			editor: EditorPlugin::new(),
		}
	}

	pub fn editor_mut(&mut self) -> &mut EditorPlugin {
		&mut self.editor
	}
}

impl<T: App> App for EditorApp<T> {
	fn on_create(&mut self, state: &mut State) {
		self.app.on_create(state);
		self.editor.on_create(state);
	}

	fn on_keyboard_input(
		&mut self,
		window_id: ArenaId<Window>,
		key: KeyboardKey,
		action: KeyAction,
		state: &mut State,
	) {
		self.app.on_keyboard_input(window_id, key, action, state);
	}

	fn on_mouse_input(&mut self, window_id: ArenaId<Window>, event: MouseEvent, state: &mut State) {
		self.app.on_mouse_input(window_id, event.clone(), state);
		self.editor.on_mouse_input(window_id, event);
	}

	fn on_process(&mut self, state: &mut State, delta: f32) {
		self.app.on_process(state, delta);
		self.editor.on_process(state, delta);
	}

	fn on_phycis_update(&mut self, state: &mut State, delta: f32) {
		self.app.on_phycis_update(state, delta);
	}
}

pub fn with_editor<T: App>(app: T) -> EditorApp<T> {
	EditorApp::new(app)
}
