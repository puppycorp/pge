use std::collections::HashSet;
use std::fs;
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
			let asset_path = Path::new(path);
			if Self::is_urdf_asset(asset_path) {
				state.load_urdf(path);
			} else {
				state.load_3d_model(path);
			}
		}
		self.create_scene_viewers_for_new_scenes(state);
	}

	fn is_urdf_asset(path: &Path) -> bool {
		let ext = path
			.extension()
			.and_then(|ext| ext.to_str())
			.unwrap_or_default();
		if ext.eq_ignore_ascii_case("urdf") {
			return true;
		}
		if !ext.eq_ignore_ascii_case("xml") {
			return false;
		}

		let content = match fs::read_to_string(path) {
			Ok(content) => content,
			Err(_) => return false,
		};
		content.contains("<robot")
	}

	fn discover_new_scenes(&mut self, state: &State) -> Vec<ArenaId<Scene>> {
		let mut new_scene_ids = Vec::new();
		for (scene_id, _) in state.scenes.iter() {
			if self.scenes.contains(&scene_id) {
				continue;
			}
			let has_nodes = state
				.nodes
				.iter()
				.any(|(_, node)| node.parent == NodeParent::Scene(scene_id) || node.scene_id == Some(scene_id));
			if !has_nodes {
				continue;
			}
			new_scene_ids.push(scene_id);
			self.scenes.insert(scene_id);
		}
		new_scene_ids
	}

	fn create_scene_viewers_for_new_scenes(&mut self, state: &mut State) {
		let new_scene_ids = self.discover_new_scenes(state);
		for scene_id in new_scene_ids {
			let scene_viewer = SceneViewer::new(state, scene_id, &self.settings);
			self.scene_viewers.push(scene_viewer);
		}
	}

	pub fn on_process(&mut self, state: &mut State, dt: f32) {
		self.create_scene_viewers_for_new_scenes(state);
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

#[cfg(test)]
mod tests {
	use std::fs;
	use std::path::PathBuf;
	use std::time::{SystemTime, UNIX_EPOCH};

	use super::*;

	fn write_temp_urdf() -> PathBuf {
		let nanos = SystemTime::now()
			.duration_since(UNIX_EPOCH)
			.unwrap()
			.as_nanos();
		let path = std::env::temp_dir().join(format!("pge_editor_{}.urdf", nanos));
		let urdf = r#"<robot name="editor_test_robot">
  <link name="base_link"/>
</robot>"#;
		fs::write(&path, urdf).expect("Failed to write temporary URDF");
		path
	}

	#[test]
	fn editor_loads_urdf_from_inspect_path() {
		let urdf_path = write_temp_urdf();

		let mut state = State::default();
		let mut editor = EditorPlugin::new();
		editor.set_inspect_path(&urdf_path);
		editor.on_create(&mut state);

		assert_eq!(state.scenes.len(), 1);
		let (_, scene) = state.scenes.iter().next().expect("Missing scene");
		assert_eq!(scene.name.as_deref(), Some("editor_test_robot"));

		let _ = fs::remove_file(urdf_path);
	}

	#[test]
	fn editor_loads_urdf_from_xml_path() {
		let nanos = SystemTime::now()
			.duration_since(UNIX_EPOCH)
			.unwrap()
			.as_nanos();
		let xml_path = std::env::temp_dir().join(format!("pge_editor_{}.xml", nanos));
		let urdf = r#"<robot name="editor_xml_robot">
# comment line used by some urdf exporters
  <link name="base_link"/>
</robot>"#;
		fs::write(&xml_path, urdf).expect("Failed to write temporary URDF xml");

		let mut state = State::default();
		let mut editor = EditorPlugin::new();
		editor.set_inspect_path(&xml_path);
		editor.on_create(&mut state);

		assert_eq!(state.scenes.len(), 1);
		let (_, scene) = state.scenes.iter().next().expect("Missing scene");
		assert_eq!(scene.name.as_deref(), Some("editor_xml_robot"));

		let _ = fs::remove_file(xml_path);
	}
}
