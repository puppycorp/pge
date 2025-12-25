use std::default;
use std::f32::consts::PI;

use pge::*;
use rand::Rng;

#[derive(Default)]
struct JustCube {
	cube_node_id: Option<ArenaId<Node>>,
}

impl pge::App for JustCube {
	fn on_create(&mut self, state: &mut pge::State) {
		let scene = Scene::new();
		let scene_id = state.scenes.insert(scene);
		let cube_mesh = state.meshes.insert(cube(0.5));

		let mut light_node = Node::new();
		light_node.name = Some("Light".to_string());
		light_node.set_translation(10.0, 10.0, 0.0);
		light_node.parent = NodeParent::Scene(scene_id);
		let light_node_id = state.nodes.insert(light_node);
		let mut light = PointLight::new();
		light.node_id = Some(light_node_id);
		state.point_lights.insert(light);

		let mut player = Node::new();
		player.name = Some("Player".to_string());
		player.set_translation(0.0, 10.0, 0.0);
		player.parent = NodeParent::Scene(scene_id);
		let player_id = state.nodes.insert(player);

		let mut cube_node = Node::new();
		cube_node.name = Some("Cube".to_string());
		cube_node.set_translation(0.0, 10.0, 3.0);
		cube_node.mesh = Some(cube_mesh);
		cube_node.physics.typ = PhycisObjectType::Dynamic;
		cube_node.physics.mass = 10.0;
		cube_node.collision_shape = Some(CollisionShape::new(Vec3::new(1.0, 1.0, 1.0)));
		cube_node.parent = NodeParent::Scene(scene_id);
		self.cube_node_id = Some(state.nodes.insert(cube_node));

		let mut camera = Camera::new();
		camera.zfar = 1000.0;
		camera.node_id = Some(player_id);
		let camera_id = state.cameras.insert(camera);

		let gui_id = state.guis.insert(camera_view(camera_id));
		state.windows.insert(window().title("JUST A CUBE!!").ui(gui_id));
	}

	fn on_process(&mut self, state: &mut pge::State, dt: f32) {
		let cube_node = state.nodes.get_mut(&self.cube_node_id.unwrap()).unwrap();
		cube_node.translation.y += 1.0 * dt;
	}
}

fn main() {
	pge::init_logging();
	pge::run(JustCube::default()).unwrap()
}
