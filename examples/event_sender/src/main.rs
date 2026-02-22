use std::thread;
use std::time::Duration;

use pge::*;

#[derive(Debug)]
enum ControlEvent {
	AddCube,
	TogglePause,
	Exit,
}

struct EventDemo {
	scene_id: Option<ArenaId<Scene>>,
	cube_mesh: Option<ArenaId<Mesh>>,
	cube_nodes: Vec<ArenaId<Node>>,
	cube_index: usize,
	paused: bool,
	should_exit: bool,
	time: f32,
}

impl EventDemo {
	fn spawn_cube(&mut self, state: &mut State) {
		let scene_id = match self.scene_id {
			Some(scene_id) => scene_id,
			None => return,
		};
		let mesh = match self.cube_mesh {
			Some(mesh) => mesh,
			None => return,
		};
		let x_offset = (self.cube_index as f32 - 1.5) * 2.0;
		let mut cube = Node::new();
		cube.parent = NodeParent::Scene(scene_id);
		cube.mesh = Some(mesh);
		cube.translation = Vec3::new(x_offset, 2.0, 5.0);
		let cube_id = state.nodes.insert(cube);
		self.cube_nodes.push(cube_id);
		self.cube_index += 1;
	}

	fn request_exit(&mut self) {
		self.should_exit = true;
	}
}

impl Default for EventDemo {
	fn default() -> Self {
		Self {
			scene_id: None,
			cube_mesh: None,
			cube_nodes: Vec::new(),
			cube_index: 0,
			paused: false,
			should_exit: false,
			time: 0.0,
		}
	}
}

impl App<ControlEvent> for EventDemo {
	fn on_create(&mut self, state: &mut State) {
		let scene_id = state.scenes.insert(Scene::new());
		self.scene_id = Some(scene_id);
		self.cube_mesh = Some(state.meshes.insert(cube(0.5)));

		let mut player = Node::new();
		player.parent = NodeParent::Scene(scene_id);
		player.translation = Vec3::new(0.0, 2.0, 0.0);
		player.looking_at(0.0, 2.0, 5.0);
		let player_node_id = state.nodes.insert(player);

		let mut camera = Camera::new();
		camera.node_id = Some(player_node_id);
		let camera_id = state.cameras.insert(camera);
		let gui_id = state.guis.insert(camera_view(camera_id));
		state.windows.insert(window().title("Event Sender Demo").ui(gui_id).width(900).height(600));

		let mut light_node = Node::new();
		light_node.parent = NodeParent::Scene(scene_id);
		light_node.translation = Vec3::new(0.0, 8.0, 5.0);
		let light_node_id = state.nodes.insert(light_node);
		let mut light = PointLight::new();
		light.node_id = Some(light_node_id);
		state.point_lights.insert(light);

		// Keep initial scene interesting with one cube.
		self.spawn_cube(state);
	}

	fn on_event(&mut self, event: ControlEvent, state: &mut State) {
		match event {
			ControlEvent::AddCube => self.spawn_cube(state),
			ControlEvent::TogglePause => self.paused = !self.paused,
			ControlEvent::Exit => self.request_exit(),
		}
	}

	fn on_process(&mut self, state: &mut State, dt: f32) {
		if self.should_exit {
			std::process::exit(0);
		}
		if self.paused {
			return;
		}
		self.time += dt;
		for node_id in self.cube_nodes.iter().copied() {
			if let Some(node) = state.nodes.get_mut(&node_id) {
				node.translation.y = 2.0 + self.time.sin();
			}
		}
	}
}

fn main() {
	pge::init_logging();
	pge::run_with_event_sender(EventDemo::default(), |sender| {
		thread::spawn(move || {
			thread::sleep(Duration::from_secs(2));
			let _ = sender.send(ControlEvent::AddCube);
			thread::sleep(Duration::from_secs(1));
			let _ = sender.send(ControlEvent::AddCube);
			thread::sleep(Duration::from_secs(2));
			let _ = sender.send(ControlEvent::Exit);
		});
	}).unwrap();
}
