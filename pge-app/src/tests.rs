#[cfg(test)]
mod tests {
    use std::time::Duration;
    use std::time::Instant;

    use crate::*;
    use engine::Engine;
    use mock_hardware::MockHardware;

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

        let dynamic_node = engine
            .state
            .nodes
            .get(&engine.app.dynamic_node_id.unwrap())
            .unwrap();
        println!("dynamic_node.translation: {:?}", dynamic_node.translation);

        assert!(
            dynamic_node.translation.y >= 0.0,
            "Dynamic object fell through the floor"
        );
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

        let dynamic_node = engine
            .state
            .nodes
            .get(&engine.app.dynamic_node_id.unwrap())
            .unwrap();
        println!("dynamic_node.translation: {:?}", dynamic_node.translation);

        assert!(
            dynamic_node.translation.y >= 0.0,
            "Fast object fell through the floor"
        );
    }

    #[test]
    fn window_overlay_lines_are_bounded() {
        let overlay = WindowOverlayLines::default();
        overlay.set(vec![
            "A".repeat(40),
            "SECOND".to_string(),
            "THIRD".to_string(),
            "FOURTH".to_string(),
            "FIFTH".to_string(),
        ]);

        let lines = overlay.snapshot();
        assert_eq!(lines.len(), 4);
        assert_eq!(lines[0].chars().count(), 32);
        assert_eq!(lines[3], "FOURTH");
    }

    #[test]
    fn additional_window_overlay_line_renders_below_fps() {
        let fps_only = fps_overlay_vertices("120 FPS", &[], [960, 540]);
        let with_ups = fps_overlay_vertices("120 FPS", &["SIM 5.0 UPS".to_string()], [960, 540]);
        let lowest_y = |vertices: &[OverlayVertex]| {
            vertices
                .iter()
                .map(|vertex| vertex.position[1])
                .fold(f32::INFINITY, f32::min)
        };

        assert!(with_ups.len() > fps_only.len());
        assert!(lowest_y(&with_ups) < lowest_y(&fps_only));
    }
}
