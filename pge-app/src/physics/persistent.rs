use std::collections::{HashMap, HashSet};

use pge_physics::{
    BodyDesc, BodyId, BodyMode, ColliderDesc, ColliderId, ColliderMaterial, ColliderShape,
    MassPropertiesDesc, PhysicsConfig, PhysicsQueryFilter, PhysicsWorld as PersistentWorld, Pose,
};

use crate::{ArenaId, ColliderType, ContactInfo, Node, PhycisObjectType, Plugin, Scene, State};

const FIXED_DT_SEC: f32 = 1.0 / 120.0;
const MAX_ACCUMULATED_TIME_SEC: f32 = 0.25;
const MAX_FRAME_FORCE_TIME_SEC: f32 = 1.0 / 60.0;

struct SceneRuntime {
    world: PersistentWorld,
    accumulator_sec: f32,
}

impl SceneRuntime {
    fn new() -> Self {
        Self {
            world: PersistentWorld::new(PhysicsConfig {
                // The legacy app and FPS example are explicitly Y-up.
                gravity_mps2: [0.0, -10.0, 0.0],
                fixed_dt_sec: FIXED_DT_SEC,
                substeps: 1,
            })
            .expect("the pge-app compatibility physics configuration is valid"),
            accumulator_sec: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct StructuralSignature {
    mode: BodyMode,
    mass: Option<MassPropertiesDesc>,
    linear_damping: f32,
    angular_damping: f32,
    lock_rotation: [bool; 3],
    collider: ColliderDesc,
}

#[derive(Clone)]
struct DesiredNode {
    node_id: ArenaId<Node>,
    scene_id: ArenaId<Scene>,
    pose: Pose,
    linear_velocity_mps: [f32; 3],
    angular_velocity_rps: [f32; 3],
    force_n: [f32; 3],
    torque_nm: [f32; 3],
    signature: StructuralSignature,
}

#[derive(Clone)]
struct NodeBinding {
    scene_id: ArenaId<Scene>,
    body_id: BodyId,
    collider_id: ColliderId,
    signature: StructuralSignature,
    last_pose: Pose,
    last_linear_velocity_mps: [f32; 3],
    last_angular_velocity_rps: [f32; 3],
}

/// Compatibility adapter from the legacy `pge_app::State` model to the
/// canonical persistent PGE physics runtime.
pub struct PhysicsWorld {
    scenes: HashMap<ArenaId<Scene>, SceneRuntime>,
    bindings: HashMap<ArenaId<Node>, NodeBinding>,
    next_stable_id: u64,
}

impl PhysicsWorld {
    pub fn new() -> Self {
        Self {
            scenes: HashMap::new(),
            bindings: HashMap::new(),
            next_stable_id: 0,
        }
    }

    pub fn process(&mut self, state: &mut State, dt_sec: f32) {
        let desired = state
            .nodes
            .iter()
            .filter_map(|(node_id, node)| desired_node(node_id, node))
            .map(|node| (node.node_id, node))
            .collect::<HashMap<_, _>>();

        self.remove_stale_bindings(&desired);
        for node in desired.values() {
            self.reconcile_node(node);
        }

        let dt_sec = if dt_sec.is_finite() && dt_sec > 0.0 {
            dt_sec.min(MAX_ACCUMULATED_TIME_SEC)
        } else {
            0.0
        };
        self.step_scenes(&desired, dt_sec);
        self.write_back(state, dt_sec);
        self.write_contacts(state);
        self.write_raycasts(state);
    }

    fn remove_stale_bindings(&mut self, desired: &HashMap<ArenaId<Node>, DesiredNode>) {
        let removed = self
            .bindings
            .iter()
            .filter(|(node_id, binding)| {
                desired.get(node_id).is_none_or(|node| {
                    node.scene_id != binding.scene_id || node.signature != binding.signature
                })
            })
            .map(|(node_id, _)| *node_id)
            .collect::<Vec<_>>();
        for node_id in removed {
            if let Some(binding) = self.bindings.remove(&node_id) {
                if let Some(scene) = self.scenes.get_mut(&binding.scene_id) {
                    let _ = scene.world.remove_body(&binding.body_id);
                }
            }
        }
    }

    fn reconcile_node(&mut self, desired: &DesiredNode) {
        if !self.bindings.contains_key(&desired.node_id) {
            self.create_binding(desired);
        }
        let Some(binding) = self.bindings.get(&desired.node_id).cloned() else {
            return;
        };
        let Some(scene) = self.scenes.get_mut(&binding.scene_id) else {
            return;
        };
        if desired.pose != binding.last_pose {
            let _ = scene
                .world
                .set_body_pose(&binding.body_id, desired.pose, true);
        }
        if desired.linear_velocity_mps != binding.last_linear_velocity_mps
            || desired.angular_velocity_rps != binding.last_angular_velocity_rps
        {
            let _ = scene.world.set_body_velocity(
                &binding.body_id,
                desired.linear_velocity_mps,
                desired.angular_velocity_rps,
                true,
            );
        }
        if let Some(binding) = self.bindings.get_mut(&desired.node_id) {
            binding.last_pose = desired.pose;
            binding.last_linear_velocity_mps = desired.linear_velocity_mps;
            binding.last_angular_velocity_rps = desired.angular_velocity_rps;
        }
    }

    fn create_binding(&mut self, desired: &DesiredNode) {
        let stable = self.next_stable_id;
        self.next_stable_id = self.next_stable_id.saturating_add(1);
        let body_id = BodyId::new(format!("pge-app:body:{stable}"));
        let collider_id = ColliderId::new(format!("pge-app:collider:{stable}"));
        let scene = self
            .scenes
            .entry(desired.scene_id)
            .or_insert_with(SceneRuntime::new);
        let body = BodyDesc {
            mode: desired.signature.mode,
            pose: desired.pose,
            linear_velocity_mps: desired.linear_velocity_mps,
            angular_velocity_rps: desired.angular_velocity_rps,
            linear_damping: desired.signature.linear_damping,
            angular_damping: desired.signature.angular_damping,
            mass: desired.signature.mass,
            ccd_enabled: desired.signature.mode == BodyMode::Dynamic,
            lock_rotation: desired.signature.lock_rotation,
            ..BodyDesc::default()
        };
        if scene.world.create_body(body_id.clone(), body).is_err() {
            return;
        }
        if scene
            .world
            .create_collider(
                collider_id.clone(),
                &body_id,
                desired.signature.collider.clone(),
            )
            .is_err()
        {
            let _ = scene.world.remove_body(&body_id);
            return;
        }
        self.bindings.insert(
            desired.node_id,
            NodeBinding {
                scene_id: desired.scene_id,
                body_id,
                collider_id,
                signature: desired.signature.clone(),
                last_pose: desired.pose,
                last_linear_velocity_mps: desired.linear_velocity_mps,
                last_angular_velocity_rps: desired.angular_velocity_rps,
            },
        );
    }

    fn step_scenes(&mut self, desired: &HashMap<ArenaId<Node>, DesiredNode>, dt_sec: f32) {
        for (scene_id, scene) in &mut self.scenes {
            scene.accumulator_sec = (scene.accumulator_sec + dt_sec).min(MAX_ACCUMULATED_TIME_SEC);
            if scene.accumulator_sec + f32::EPSILON >= FIXED_DT_SEC {
                // Legacy pge-app forces are authored once per app frame. Consume
                // them once as a frame impulse instead of replaying the same
                // feedback-controller output on every fixed catch-up step.
                let force_time_sec = dt_sec.min(MAX_FRAME_FORCE_TIME_SEC);
                for (node_id, binding) in &self.bindings {
                    if binding.scene_id != *scene_id {
                        continue;
                    }
                    let Some(node) = desired.get(node_id) else {
                        continue;
                    };
                    if node.signature.mode != BodyMode::Dynamic {
                        continue;
                    }
                    let impulse = node.force_n.map(|force| force * force_time_sec);
                    let torque_impulse = node.torque_nm.map(|torque| torque * force_time_sec);
                    let _ = scene.world.apply_impulse(&binding.body_id, impulse, true);
                    let _ =
                        scene
                            .world
                            .apply_torque_impulse(&binding.body_id, torque_impulse, true);
                }
            }
            while scene.accumulator_sec + f32::EPSILON >= FIXED_DT_SEC {
                scene.world.step();
                scene.accumulator_sec -= FIXED_DT_SEC;
            }
        }
    }

    fn write_back(&mut self, state: &mut State, dt_sec: f32) {
        for (node_id, binding) in &mut self.bindings {
            let Some(scene) = self.scenes.get(&binding.scene_id) else {
                continue;
            };
            let Ok(snapshot) = scene.world.body_snapshot(&binding.body_id) else {
                continue;
            };
            let Some(node) = state.nodes.get_mut(node_id) else {
                continue;
            };
            let previous_linear = node.physics.velocity;
            let previous_angular = node.physics.angular_velocity;
            node.translation = vec3(snapshot.pose.translation);
            node.rotation = quat(snapshot.pose.rotation_xyzw);
            node.physics.velocity = vec3(snapshot.linear_velocity_mps);
            node.physics.angular_velocity = vec3(snapshot.angular_velocity_rps);
            if dt_sec > 0.0 {
                node.physics.acceleration = (node.physics.velocity - previous_linear) / dt_sec;
                node.physics.angular_acceleration =
                    (node.physics.angular_velocity - previous_angular) / dt_sec;
            }
            binding.last_pose = snapshot.pose;
            binding.last_linear_velocity_mps = snapshot.linear_velocity_mps;
            binding.last_angular_velocity_rps = snapshot.angular_velocity_rps;
        }
    }

    fn write_contacts(&self, state: &mut State) {
        for (_, node) in &mut state.nodes {
            node.contacts.clear();
        }
        for (scene_id, scene) in &self.scenes {
            let collider_nodes = self
                .bindings
                .iter()
                .filter(|(_, binding)| binding.scene_id == *scene_id)
                .map(|(node_id, binding)| (binding.collider_id.clone(), *node_id))
                .collect::<HashMap<_, _>>();
            for contact in scene.world.contacts() {
                if contact.sensor {
                    continue;
                }
                let (Some(node1), Some(node2)) = (
                    collider_nodes.get(&contact.collider1).copied(),
                    collider_nodes.get(&contact.collider2).copied(),
                ) else {
                    continue;
                };
                let point = contact
                    .manifolds
                    .first()
                    .and_then(|manifold| manifold.points.first())
                    .map(|point| point.point1_m)
                    .unwrap_or([0.0; 3]);
                let normal = vec3(contact.normal);
                if state
                    .nodes
                    .get(&node1)
                    .is_some_and(|node| node.physics.typ == PhycisObjectType::Dynamic)
                {
                    state
                        .nodes
                        .get_mut(&node1)
                        .expect("contact node exists")
                        .contacts
                        .push(ContactInfo {
                            normal,
                            point: vec3(point),
                            node_id: node2,
                        });
                }
                if state
                    .nodes
                    .get(&node2)
                    .is_some_and(|node| node.physics.typ == PhycisObjectType::Dynamic)
                {
                    state
                        .nodes
                        .get_mut(&node2)
                        .expect("contact node exists")
                        .contacts
                        .push(ContactInfo {
                            normal: -normal,
                            point: vec3(point),
                            node_id: node1,
                        });
                }
            }
        }
    }

    fn write_raycasts(&self, state: &mut State) {
        let casts = state
            .raycasts
            .iter()
            .map(|(id, ray)| (id, ray.node_id, ray.len))
            .collect::<Vec<_>>();
        for (ray_id, source_node, length) in casts {
            let intersections = self.raycast_nodes(state, source_node, length);
            if let Some(ray) = state.raycasts.get_mut(&ray_id) {
                ray.intersects = intersections;
            }
        }
    }

    fn raycast_nodes(
        &self,
        state: &State,
        source_node: ArenaId<Node>,
        length: f32,
    ) -> Vec<ArenaId<Node>> {
        if !length.is_finite() || length <= 0.0 {
            return Vec::new();
        }
        let Some(source) = state.nodes.get(&source_node) else {
            return Vec::new();
        };
        let Some(binding) = self.bindings.get(&source_node) else {
            return Vec::new();
        };
        let Some(scene) = self.scenes.get(&binding.scene_id) else {
            return Vec::new();
        };
        let direction = source.rotation * glam::Vec3::Z;
        if direction.length_squared() <= f32::EPSILON {
            return Vec::new();
        }
        let collider_nodes = self
            .bindings
            .iter()
            .filter(|(_, candidate)| candidate.scene_id == binding.scene_id)
            .map(|(node_id, candidate)| (candidate.collider_id.clone(), *node_id))
            .collect::<HashMap<_, _>>();
        let mut filter = PhysicsQueryFilter {
            excluded_bodies: vec![binding.body_id.clone()],
            ..PhysicsQueryFilter::default()
        };
        let mut nodes = Vec::new();
        let mut seen = HashSet::new();
        loop {
            let Ok(Some(hit)) = scene.world.cast_ray_filtered(
                source.translation.to_array(),
                direction.to_array(),
                length,
                true,
                &filter,
            ) else {
                break;
            };
            filter.excluded_colliders.push(hit.collider.clone());
            if let Some(node_id) = collider_nodes.get(&hit.collider).copied() {
                if seen.insert(node_id) {
                    nodes.push(node_id);
                }
            }
            if filter.excluded_colliders.len() >= collider_nodes.len() {
                break;
            }
        }
        nodes
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl Plugin for PhysicsWorld {
    fn process(&mut self, state: &mut State, dt: f32) {
        PhysicsWorld::process(self, state, dt);
    }
}

fn desired_node(node_id: ArenaId<Node>, node: &Node) -> Option<DesiredNode> {
    let scene_id = node.scene_id?;
    let shape = collider_shape(node.collision_shape.as_ref()?)?;
    let mode = match node.physics.typ {
        PhycisObjectType::Static => BodyMode::Static,
        PhycisObjectType::Dynamic if node.physics.stationary => BodyMode::Static,
        PhycisObjectType::Dynamic => BodyMode::Dynamic,
        PhycisObjectType::None => return None,
    };
    let body_pose = pose(node.translation, node.rotation);
    let collider_pose = pose(
        node.collision_shape.as_ref()?.position_offset,
        node.collision_shape.as_ref()?.rotation_offset,
    );
    let collider = ColliderDesc {
        pose: collider_pose,
        shape,
        material: ColliderMaterial {
            friction: node.physics.friction.max(0.0),
            restitution: node.physics.restitution.clamp(0.0, 1.0),
            density_kg_m3: 1.0,
            contact_skin_m: 0.0,
        },
        sensor: node.physics.is_sensor,
        collision_memberships: node.physics.collision_group,
        collision_filter: node.physics.collision_mask,
    };
    let mass = explicit_mass(node, mode);
    Some(DesiredNode {
        node_id,
        scene_id,
        pose: body_pose,
        linear_velocity_mps: node.physics.velocity.to_array(),
        angular_velocity_rps: node.physics.angular_velocity.to_array(),
        force_n: node.physics.force.to_array(),
        torque_nm: node.physics.torque.to_array(),
        signature: StructuralSignature {
            mode,
            mass,
            linear_damping: node.physics.linear_damping.max(0.0),
            angular_damping: node.physics.angular_damping.max(0.0),
            lock_rotation: [node.lock_rotation; 3],
            collider,
        },
    })
}

fn collider_shape(shape: &crate::CollisionShape) -> Option<ColliderShape> {
    match &shape.shape {
        ColliderType::Cuboid { size } => Some(ColliderShape::Box {
            size: (*size * 2.0).to_array(),
        }),
        ColliderType::Sphere { radius } => Some(ColliderShape::Sphere { radius: *radius }),
        ColliderType::Capsule {
            half_height,
            radius,
        } => Some(ColliderShape::CapsuleY {
            half_height: *half_height,
            radius: *radius,
        }),
        ColliderType::Cylinder {
            half_height,
            radius,
        } => Some(ColliderShape::CylinderY {
            half_height: *half_height,
            radius: *radius,
        }),
        ColliderType::TriMesh { .. } => None,
    }
}

fn explicit_mass(node: &Node, mode: BodyMode) -> Option<MassPropertiesDesc> {
    if mode != BodyMode::Dynamic || !node.physics.mass.is_finite() || node.physics.mass <= 0.0 {
        return None;
    }
    let inertia = node.inertia_tensor().to_cols_array_2d();
    let mut principal = [inertia[0][0], inertia[1][1], inertia[2][2]];
    for value in &mut principal {
        if !value.is_finite() || *value <= f32::EPSILON {
            *value = 1.0;
        }
    }
    Some(MassPropertiesDesc {
        mass_kg: node.physics.mass,
        center_of_mass_m: node
            .collision_shape
            .as_ref()
            .map_or([0.0; 3], |shape| shape.position_offset.to_array()),
        principal_inertia_kg_m2: principal,
        ..MassPropertiesDesc::default()
    })
}

fn pose(translation: glam::Vec3, rotation: glam::Quat) -> Pose {
    Pose {
        translation: translation.to_array(),
        rotation_xyzw: [rotation.x, rotation.y, rotation.z, rotation.w],
    }
}

fn vec3(value: [f32; 3]) -> glam::Vec3 {
    glam::Vec3::from_array(value)
}

fn quat(value: [f32; 4]) -> glam::Quat {
    glam::Quat::from_xyzw(value[0], value[1], value[2], value[3])
}
