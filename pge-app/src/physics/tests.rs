use super::*;
use crate::{CollisionShape, Node, PhycisObjectType, Plugin, Scene, State};

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
    let floor_id = state.nodes.insert(floor);

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
        (2.0..=2.2).contains(&player.translation.y),
        "player did not settle on the Y-up floor: y={}",
        player.translation.y
    );
    assert!(
        player
            .contacts
            .iter()
            .any(|contact| contact.node_id == floor_id),
        "settled player is missing its floor contact"
    );
}

#[test]
fn raycast_reports_stable_node_ids_in_distance_order() {
    let mut physics = PhysicsWorld::new();
    let mut state = State::default();
    let scene_id = state.scenes.insert(Scene::new());

    let mut source = Node::new();
    source.physics.typ = PhycisObjectType::Static;
    source.collision_shape = Some(CollisionShape::new(glam::Vec3::splat(0.25)));
    source.scene_id = Some(scene_id);
    let source_id = state.nodes.insert(source);
    let ray_id = state.raycasts.insert(crate::RayCast::new(source_id, 10.0));

    let mut near_target = Node::new();
    near_target.physics.typ = PhycisObjectType::Static;
    near_target.collision_shape = Some(CollisionShape::new(glam::Vec3::splat(0.5)));
    near_target.translation = glam::Vec3::new(0.0, 0.0, 3.0);
    near_target.scene_id = Some(scene_id);
    let near_id = state.nodes.insert(near_target);

    let mut far_target = Node::new();
    far_target.physics.typ = PhycisObjectType::Static;
    far_target.collision_shape = Some(CollisionShape::new(glam::Vec3::splat(0.5)));
    far_target.translation = glam::Vec3::new(0.0, 0.0, 6.0);
    far_target.scene_id = Some(scene_id);
    let far_id = state.nodes.insert(far_target);

    physics.process(&mut state, 0.016);

    let hits = &state
        .raycasts
        .get(&ray_id)
        .expect("forward ray missing")
        .intersects;
    assert_eq!(hits, &vec![near_id, far_id]);
}
