use std::collections::HashMap;

use pge_core::{ArenaId, BodyKind, Collider, Node, PhysicsBody, Transform, WorldState};
pub use rapier3d;
use rapier3d::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsStep {
    pub dt_sec: f32,
}

pub struct RapierPhysicsWorld {
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
}

impl RapierPhysicsWorld {
    pub fn new() -> Self {
        Self {
            gravity: vector![0.0, 0.0, -9.81],
            integration_parameters: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
        }
    }

    pub fn set_gravity(&mut self, gravity: Vector<Real>) {
        self.gravity = gravity;
    }

    pub fn set_time_step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
    }

    pub fn step(&mut self) {
        if self.bodies.is_empty()
            && self.colliders.is_empty()
            && self.impulse_joints.is_empty()
            && self.multibody_joints.iter().next().is_none()
        {
            return;
        }

        let physics_hooks = ();
        let event_handler = ();
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &physics_hooks,
            &event_handler,
        );
    }

    pub fn bodies(&self) -> &RigidBodySet {
        &self.bodies
    }

    pub fn colliders(&self) -> &ColliderSet {
        &self.colliders
    }

    pub fn body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle)
    }

    pub fn bodies_mut(&mut self) -> &mut RigidBodySet {
        &mut self.bodies
    }

    pub fn colliders_mut(&mut self) -> &mut ColliderSet {
        &mut self.colliders
    }

    pub fn impulse_joints_mut(&mut self) -> &mut ImpulseJointSet {
        &mut self.impulse_joints
    }

    pub fn multibody_joints_mut(&mut self) -> &mut MultibodyJointSet {
        &mut self.multibody_joints
    }

    pub fn query_pipeline(&self) -> &QueryPipeline {
        &self.query_pipeline
    }
}

impl Default for RapierPhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug, Default)]
pub struct PhysicsSystem {
    pub gravity_mps2: [f32; 3],
}

impl PhysicsSystem {
    pub fn new() -> Self {
        Self {
            gravity_mps2: [0.0, 0.0, -9.81],
        }
    }

    pub fn step(&mut self, world: &mut WorldState, step: PhysicsStep) {
        self.step_world(world, step);
    }

    pub fn step_world(&mut self, world: &mut WorldState, step: PhysicsStep) {
        if step.dt_sec <= 0.0 {
            return;
        }

        let mut rapier = RapierPhysicsWorld::new();
        rapier.set_gravity(vector![
            self.gravity_mps2[0],
            self.gravity_mps2[1],
            self.gravity_mps2[2]
        ]);
        rapier.set_time_step(step.dt_sec);

        let mut handles = HashMap::<ArenaId<Node>, RigidBodyHandle>::new();
        for (node_id, node) in world.nodes.iter() {
            let Some(body) = node.body else {
                continue;
            };
            if body.kind == BodyKind::None {
                continue;
            }
            let rigid_body = rigid_body_builder(node.transform, body).build();
            let handle = rapier.bodies_mut().insert(rigid_body);
            if let Some(collider) = node.collider.as_ref().and_then(collider_builder) {
                rapier
                    .colliders
                    .insert_with_parent(collider.build(), handle, &mut rapier.bodies);
            }
            handles.insert(node_id, handle);
        }

        rapier.step();

        for (node_id, handle) in handles {
            let Some(rigid_body) = rapier.body(handle) else {
                continue;
            };
            let Some(node) = world.nodes.get_mut(&node_id) else {
                continue;
            };
            node.transform.translation = [
                rigid_body.translation().x,
                rigid_body.translation().y,
                rigid_body.translation().z,
            ];
            if let Some(mut body) = node.body {
                body.velocity_mps = [
                    rigid_body.linvel().x,
                    rigid_body.linvel().y,
                    rigid_body.linvel().z,
                ];
                body.angular_velocity_rps = [
                    rigid_body.angvel().x,
                    rigid_body.angvel().y,
                    rigid_body.angvel().z,
                ];
                node.body = Some(body);
            }
        }
    }
}

fn rigid_body_builder(transform: Transform, body: PhysicsBody) -> RigidBodyBuilder {
    let translation = vector![
        transform.translation[0],
        transform.translation[1],
        transform.translation[2]
    ];
    let builder = match body.kind {
        BodyKind::Static => RigidBodyBuilder::fixed(),
        BodyKind::Dynamic => RigidBodyBuilder::dynamic(),
        BodyKind::Kinematic => RigidBodyBuilder::kinematic_position_based(),
        BodyKind::None => RigidBodyBuilder::fixed(),
    };
    builder
        .translation(translation)
        .linvel(vector![
            body.velocity_mps[0],
            body.velocity_mps[1],
            body.velocity_mps[2]
        ])
        .angvel(vector![
            body.angular_velocity_rps[0],
            body.angular_velocity_rps[1],
            body.angular_velocity_rps[2]
        ])
        .linear_damping(body.linear_damping)
        .angular_damping(body.angular_damping)
}

fn collider_builder(collider: &Collider) -> Option<ColliderBuilder> {
    match collider {
        Collider::Box { size } | Collider::MeshBounds { size } => Some(ColliderBuilder::cuboid(
            size[0] * 0.5,
            size[1] * 0.5,
            size[2] * 0.5,
        )),
        Collider::Sphere { radius } => Some(ColliderBuilder::ball(*radius)),
        Collider::Cylinder { radius, height } => {
            Some(ColliderBuilder::cylinder(height * 0.5, *radius))
        }
    }
}

#[cfg(test)]
mod tests {
    use pge_core::{BodyKind, Collider, EntityId, Node, PhysicsBody, Transform, WorldState};

    use super::rapier3d::prelude::*;
    use super::{PhysicsStep, PhysicsSystem, RapierPhysicsWorld};

    #[test]
    fn read_only_accessors_inspect_inserted_body() {
        let mut world = RapierPhysicsWorld::new();
        let body = RigidBodyBuilder::fixed()
            .translation(vector![1.0, 2.0, 3.0])
            .build();
        let handle = world.bodies_mut().insert(body);
        world
            .colliders_mut()
            .insert(ColliderBuilder::ball(0.1).build());

        assert_eq!(world.bodies().len(), 1);
        assert_eq!(world.colliders().len(), 1);
        let inserted = world.body(handle).expect("inserted body");
        assert_eq!(inserted.translation().x, 1.0);
        assert_eq!(inserted.translation().y, 2.0);
        assert_eq!(inserted.translation().z, 3.0);
    }

    #[test]
    fn dynamic_body_falls_in_z() {
        let mut world = WorldState::new();
        let mut node = Node::new("falling");
        node.transform = Transform::translated([0.0, 0.0, 1.0]);
        node.body = Some(PhysicsBody {
            kind: BodyKind::Dynamic,
            mass_kg: 1.0,
            ..PhysicsBody::default()
        });
        node.collider = Some(Collider::Sphere { radius: 0.05 });
        world.push_entity(pge_core::EntityMetadata {
            id: EntityId("falling".to_string()),
            name: "Falling".to_string(),
            kind: "body".to_string(),
            robot_id: None,
            link_name: None,
        });
        let node_id = world.nodes.insert(node);

        PhysicsSystem::new().step_world(&mut world, PhysicsStep { dt_sec: 0.1 });

        let node = world.nodes.get(&node_id).expect("node");
        assert!(node.transform.translation[2] < 1.0);
    }

    #[test]
    fn dynamic_cube_stays_above_static_floor() {
        let mut world = WorldState::new();
        let mut floor = Node::new("floor");
        floor.transform = Transform::translated([0.0, 0.0, -0.05]);
        floor.body = Some(PhysicsBody {
            kind: BodyKind::Static,
            ..PhysicsBody::default()
        });
        floor.collider = Some(Collider::Box {
            size: [4.0, 4.0, 0.1],
        });
        world.nodes.insert(floor);

        let mut cube = Node::new("cube");
        cube.transform = Transform::translated([0.0, 0.0, 0.5]);
        cube.body = Some(PhysicsBody {
            kind: BodyKind::Dynamic,
            mass_kg: 1.0,
            ..PhysicsBody::default()
        });
        cube.collider = Some(Collider::Box {
            size: [0.2, 0.2, 0.2],
        });
        let cube_id = world.nodes.insert(cube);

        let mut physics = PhysicsSystem::new();
        for _ in 0..120 {
            physics.step_world(&mut world, PhysicsStep { dt_sec: 1.0 / 60.0 });
        }

        let cube = world.nodes.get(&cube_id).expect("cube");
        assert!(cube.transform.translation[2] >= 0.09);
    }
}
