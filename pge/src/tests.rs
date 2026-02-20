
#[cfg(test)]
mod tests {
	use std::time::Duration;
use std::time::Instant;

use engine::Engine;
	use mock_hardware::MockHardware;
	use crate::*;

	#[test]
	fn object_does_not_fall_through_floor() {
		init_logging();
		#[derive(Default)]
		struct TestApp {
			pub dynamic_node_id: Option<ArenaId<Node>>,
		}

		impl App for TestApp {
			fn on_create(&mut self, state: &mut crate::State) {
				let scene = Scene::new();
				let scene_id = state.scenes.insert(scene);

				// Create a static floor node
				let floor_node = Node {
					physics: PhysicsProps {
						typ: PhycisObjectType::Static,
						stationary: true,
						..Default::default()
					},
					translation: Vec3::new(0.0, 1.0, 0.0),
					collision_shape: Some(CollisionShape::new(Vec3::new(10.0, 1.0, 10.0))),
					parent: NodeParent::Scene(scene_id),
					..Default::default()
				};
				let floor_id = state.nodes.insert(floor_node);
				
				// Create a dynamic object above the floor
				let dynamic_node = Node {
					physics: PhysicsProps {
						typ: PhycisObjectType::Dynamic,
						mass: 1.0,
						stationary: false,
						..Default::default()
					},
					lock_rotation: true,
					translation: Vec3::new(0.0, 10.0, 0.0),
					collision_shape: Some(CollisionShape::new(Vec3::new(1.0, 1.0, 1.0))),
					parent: NodeParent::Scene(scene_id),
					..Default::default()
				};
				self.dynamic_node_id = Some(state.nodes.insert(dynamic_node));
			}
		}

		let hardware = MockHardware::new();

		let mut engine = Engine::new(TestApp::default(), hardware);

		let timer = Instant::now();
		let dt = 0.016;
		for _ in 0..2000 {
			engine.render(dt);
		}
		let duration = timer.elapsed();
		let fps = 600.0 / duration.as_secs_f32();
		println!("duration: {:?}", duration);
		println!("fps: {:?}", fps);

		let dynamic_node = engine.state.nodes.get(&engine.app.dynamic_node_id.unwrap()).unwrap();
		println!("dynamic_node.translation: {:?}", dynamic_node.translation);

		assert!(dynamic_node.translation.y >= 0.0, "Dynamic object fell through the floor");
	}

	#[test]
	fn fast_object_does_not_fall_through_floor() {
		#[derive(Default)]
		struct TestApp {
			pub dynamic_node_id: Option<ArenaId<Node>>,
		}

		impl App for TestApp {
			fn on_create(&mut self, state: &mut crate::State) {
				let scene = Scene::new();
				let scene_id = state.scenes.insert(scene);

				// Create a static floor node
				let floor_node = Node {
					physics: PhysicsProps {
						typ: PhycisObjectType::Static,
						stationary: true,
						..Default::default()
					},
					translation: Vec3::new(0.0, 1.0, 0.0),
					collision_shape: Some(CollisionShape::new(Vec3::new(10.0, 1.0, 10.0))),
					parent: NodeParent::Scene(scene_id),
					..Default::default()
				};
				let floor_id = state.nodes.insert(floor_node);
				
				// Create a dynamic object above the floor
				let dynamic_node = Node {
					physics: PhysicsProps {
						typ: PhycisObjectType::Dynamic,
						mass: 1.0,
						stationary: false,
						velocity: Vec3::new(0.0, -500.0, 0.0),
						..Default::default()
					},
					translation: Vec3::new(0.0, 10.0, 0.0),
					collision_shape: Some(CollisionShape::new(Vec3::new(1.0, 1.0, 1.0))),
					parent: NodeParent::Scene(scene_id),
					..Default::default()
				};
				self.dynamic_node_id = Some(state.nodes.insert(dynamic_node));
			}
		}

		let hardware = MockHardware::new();

		let mut engine = Engine::new(TestApp::default(), hardware);

		let timer = Instant::now();
		let dt = 0.016;
		for _ in 0..3000 {
			engine.render(dt);
		}
		let duration = timer.elapsed();
		println!("duration: {:?}", duration);
		println!("per frame: {:?} micros", duration.as_micros() / 600);

		let dynamic_node = engine.state.nodes.get(&engine.app.dynamic_node_id.unwrap()).unwrap();
		println!("dynamic_node.translation: {:?}", dynamic_node.translation);

		assert!(dynamic_node.translation.y >= 0.0, "Fast object fell through the floor");
	}
}
