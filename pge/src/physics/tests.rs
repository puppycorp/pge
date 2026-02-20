use super::*;
use crate::CollisionShape;
use crate::Plugin;

#[test]
fn player_does_not_fall_through_floor() {
	run_floor_test(Box::new(PhysicsWorld::new()));
}

fn run_floor_test(mut physics: Box<dyn Plugin>) {
	let mut state = State::default();
	let scene_id = state.scenes.insert(Scene::new());

	let mut floor = Node::new();
	floor.physics.typ = PhycisObjectType::Static;
	floor.collision_shape = Some(CollisionShape::new(glam::Vec3::new(50.0, 0.1, 50.0)));
	floor.translation = glam::Vec3::new(0.0, 0.0, 0.0);
	floor.scene_id = Some(scene_id);
	state.nodes.insert(floor);

	let mut player = Node::new();
	player.physics.typ = PhycisObjectType::Dynamic;
	player.physics.mass = 70.0;
	player.collision_shape = Some(CollisionShape::new(glam::Vec3::new(1.0, 2.0, 1.0)));
	player.translation = glam::Vec3::new(0.0, 5.0, 0.0);
	player.scene_id = Some(scene_id);
	let player_id = state.nodes.insert(player);

	let dt = 0.016;
	for _ in 0..600 {
		physics.process(&mut state, dt);
	}

	let player = state.nodes.get(&player_id).expect("player missing");
	assert!(
		player.translation.y >= -0.01,
		"player fell through floor: y={}",
		player.translation.y
	);
}
