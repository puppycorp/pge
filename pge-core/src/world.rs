use serde::{Deserialize, Serialize};

use crate::{Arena, ArenaId};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub translation: [f32; 3],
    pub rotation: [f32; 3],
    pub rotation_matrix: Option<[[f32; 3]; 3]>,
}

impl Transform {
    pub fn translated(translation: [f32; 3]) -> Self {
        Self {
            translation,
            rotation: [0.0, 0.0, 0.0],
            rotation_matrix: None,
        }
    }

    pub fn matrix(translation: [f32; 3], rotation_matrix: [[f32; 3]; 3]) -> Self {
        Self {
            translation,
            rotation: matrix_to_rpy(rotation_matrix),
            rotation_matrix: Some(rotation_matrix),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct EntityId(pub String);

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EntityMetadata {
    pub id: EntityId,
    pub name: String,
    pub kind: String,
    pub robot_id: Option<String>,
    pub link_name: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TextLabel {
    pub entity: EntityId,
    pub text: String,
    pub position: [f32; 3],
    pub color: [f32; 4],
    pub background_color: [f32; 4],
    pub font_size_px: f32,
    pub billboard: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum BodyKind {
    Static,
    Dynamic,
    Kinematic,
    None,
}

impl Default for BodyKind {
    fn default() -> Self {
        Self::None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsBody {
    pub kind: BodyKind,
    pub mass_kg: f32,
    pub velocity_mps: [f32; 3],
    pub angular_velocity_rps: [f32; 3],
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl Default for PhysicsBody {
    fn default() -> Self {
        Self {
            kind: BodyKind::None,
            mass_kg: 0.0,
            velocity_mps: [0.0, 0.0, 0.0],
            angular_velocity_rps: [0.0, 0.0, 0.0],
            linear_damping: 0.0,
            angular_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Collider {
    Box { size: [f32; 3] },
    Sphere { radius: f32 },
    Cylinder { radius: f32, height: f32 },
    MeshBounds { size: [f32; 3] },
}

/// A primitive that can be drawn by a collider-debug renderer.
///
/// This is deliberately separate from [`Collider`]. It describes diagnostic
/// geometry only and is never consumed by PGE physics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum ColliderWireframeShape {
    Box {
        size: [f32; 3],
    },
    Sphere {
        radius: f32,
    },
    Cylinder {
        radius: f32,
        height: f32,
    },
    MeshBounds {
        size: [f32; 3],
    },
    Compound {
        children: Vec<ColliderWireframeChild>,
    },
}

/// One link-local member of a compound debug collider.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ColliderWireframeChild {
    pub transform: Transform,
    pub shape: ColliderWireframeShape,
}

/// A render-only collider diagnostic in world coordinates.
///
/// `id` is intended to be stable across frames, while `category` lets a
/// product distinguish scene, vehicle, link, or backend-only colliders. The
/// colour is linear RGBA and belongs to the diagnostic, not its source body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ColliderWireframe {
    pub id: String,
    pub category: String,
    pub color: [f32; 4],
    pub transform: Transform,
    pub shape: ColliderWireframeShape,
}

impl ColliderWireframe {
    pub fn new(
        id: impl Into<String>,
        category: impl Into<String>,
        transform: Transform,
        shape: ColliderWireframeShape,
    ) -> Self {
        Self {
            id: id.into(),
            category: category.into(),
            color: [1.0, 0.0, 1.0, 1.0],
            transform,
            shape,
        }
    }
}

/// Format-neutral, render-only collider diagnostics.
///
/// A consumer may add colliders that live in a backend outside PGE's native
/// `Node::collider` field (for example reviewed robot-link profiles). These
/// entries remain intentionally outside `nodes`, `meshes`, and `PhysicsBody`,
/// so enabling the overlay cannot affect stepping or camera fitting.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ColliderDebugOverlay {
    pub enabled: bool,
    pub wireframes: Vec<ColliderWireframe>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum NodeParent {
    Node(ArenaId<Node>),
    Scene(ArenaId<Scene>),
    Orphan,
}

impl Default for NodeParent {
    fn default() -> Self {
        Self::Orphan
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Node {
    pub entity: EntityId,
    pub name: Option<String>,
    pub parent: NodeParent,
    pub transform: Transform,
    pub mesh: Option<ArenaId<Mesh>>,
    pub camera: Option<ArenaId<Camera>>,
    pub light: Option<ArenaId<Light>>,
    pub body: Option<PhysicsBody>,
    pub collider: Option<Collider>,
}

impl Node {
    pub fn new(entity: impl Into<String>) -> Self {
        let entity = EntityId(entity.into());
        Self {
            name: Some(entity.0.clone()),
            entity,
            parent: NodeParent::Orphan,
            transform: Transform::default(),
            mesh: None,
            camera: None,
            light: None,
            body: None,
            collider: None,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Scene {
    pub name: Option<String>,
    pub gravity_mps2: [f32; 3],
    pub physics_enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryBounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MeshSource {
    Procedural(Geometry),
    Asset {
        path: String,
        scale: [f32; 3],
        bounds: Option<GeometryBounds>,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Geometry {
    Box { size: [f32; 3] },
    Sphere { radius: f32 },
    Cylinder { radius: f32, height: f32 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mesh {
    pub name: Option<String>,
    pub source: MeshSource,
    pub material: Option<ArenaId<Material>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TextureSource {
    File(String),
    Buffer {
        data: Vec<u8>,
        width: u32,
        height: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Texture {
    pub name: String,
    pub source: TextureSource,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub name: Option<String>,
    pub base_color_factor: [f32; 4],
    pub base_color_texture: Option<ArenaId<Texture>>,
    pub metallic_factor: f32,
    pub roughness_factor: f32,
    pub emissive_factor: [f32; 3],
}

impl Default for Material {
    fn default() -> Self {
        Self {
            name: None,
            base_color_factor: [0.7, 0.75, 0.8, 1.0],
            base_color_texture: None,
            metallic_factor: 0.0,
            roughness_factor: 0.7,
            emissive_factor: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum CameraProjection {
    Perspective,
    Orthographic { size_m: f32 },
}

impl Default for CameraProjection {
    fn default() -> Self {
        Self::Perspective
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub skew: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraDistortion {
    pub k1: f32,
    pub k2: f32,
    pub p1: f32,
    pub p2: f32,
    pub k3: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraSensorEffects {
    pub exposure: f32,
    pub gamma: f32,
    pub rgb_noise_stddev: f32,
    pub depth_noise_stddev_m: f32,
    pub depth_quantization_m: f32,
    pub noise_seed: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Camera {
    pub name: Option<String>,
    pub fov_deg: f32,
    pub projection: CameraProjection,
    pub resolution: [u32; 2],
    pub intrinsics: Option<CameraIntrinsics>,
    pub distortion: Option<CameraDistortion>,
    pub depth_range_m: Option<[f32; 2]>,
    pub sensor_effects: Option<CameraSensorEffects>,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            name: None,
            fov_deg: 58.0,
            projection: CameraProjection::Perspective,
            resolution: [640, 480],
            intrinsics: None,
            distortion: None,
            depth_range_m: None,
            sensor_effects: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum LightKind {
    Directional {
        direction: [f32; 3],
        angular_radius_deg: f32,
    },
    Point {
        range_m: Option<f32>,
    },
    Spot {
        direction: [f32; 3],
        inner_cone_deg: f32,
        outer_cone_deg: f32,
        range_m: Option<f32>,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Light {
    pub name: Option<String>,
    pub kind: LightKind,
    pub color_rgb: [u8; 3],
    pub intensity: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    Fixed,
    Revolute {
        axis: [f32; 3],
        limits_rad: Option<[f32; 2]>,
    },
    Prismatic {
        axis: [f32; 3],
        limits_m: Option<[f32; 2]>,
    },
    Ball,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: Option<String>,
    pub parent: ArenaId<Node>,
    pub child: ArenaId<Node>,
    pub kind: JointKind,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WorldState {
    pub scenes: Arena<Scene>,
    pub nodes: Arena<Node>,
    pub meshes: Arena<Mesh>,
    pub materials: Arena<Material>,
    pub textures: Arena<Texture>,
    pub cameras: Arena<Camera>,
    pub lights: Arena<Light>,
    pub joints: Arena<Joint>,
    pub entities: Vec<EntityMetadata>,
    pub text_labels: Vec<TextLabel>,
    #[serde(default)]
    pub collider_debug: ColliderDebugOverlay,
}

impl WorldState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn entity(&self, id: &EntityId) -> Option<&EntityMetadata> {
        self.entities.iter().find(|entity| &entity.id == id)
    }

    pub fn push_entity(&mut self, metadata: EntityMetadata) {
        if let Some(existing) = self
            .entities
            .iter_mut()
            .find(|entity| entity.id == metadata.id)
        {
            *existing = metadata;
        } else {
            self.entities.push(metadata);
        }
    }

    /// Returns the complete renderer-facing collider overlay for this world.
    ///
    /// Native PGE scene colliders are derived every call so their current node
    /// poses remain accurate. `collider_debug.wireframes` supplies additional
    /// colliders owned by importers or an external physics backend. No entry
    /// returned here is a world node or a physics shape.
    pub fn collider_wireframes(&self) -> Vec<ColliderWireframe> {
        if !self.collider_debug.enabled {
            return Vec::new();
        }

        let mut wireframes = self.collider_debug.wireframes.clone();
        for (node_id, node) in self.nodes.iter() {
            let Some(collider) = &node.collider else {
                continue;
            };
            wireframes.push(ColliderWireframe {
                id: format!("pge.scene-collider:{}", node.entity.0),
                category: "sceneCollider".to_string(),
                color: [1.0, 0.0, 1.0, 1.0],
                transform: self.node_world_transform(node_id),
                shape: collider_wireframe_shape(collider),
            });
        }
        wireframes.sort_by(|left, right| left.id.cmp(&right.id));
        wireframes
    }

    /// Adds a non-physical collider diagnostic. Callers should update the
    /// transform every simulation frame for backend-owned dynamic colliders.
    pub fn push_collider_wireframe(&mut self, wireframe: ColliderWireframe) {
        self.collider_debug.wireframes.push(wireframe);
    }

    fn node_world_transform(&self, node_id: ArenaId<Node>) -> Transform {
        let Some(node) = self.nodes.get(&node_id) else {
            return Transform::default();
        };
        match node.parent {
            NodeParent::Node(parent) => {
                compose_transforms(self.node_world_transform(parent), node.transform)
            }
            NodeParent::Scene(_) | NodeParent::Orphan => node.transform,
        }
    }
}

fn collider_wireframe_shape(collider: &Collider) -> ColliderWireframeShape {
    match collider {
        Collider::Box { size } => ColliderWireframeShape::Box { size: *size },
        Collider::Sphere { radius } => ColliderWireframeShape::Sphere { radius: *radius },
        Collider::Cylinder { radius, height } => ColliderWireframeShape::Cylinder {
            radius: *radius,
            height: *height,
        },
        Collider::MeshBounds { size } => ColliderWireframeShape::MeshBounds { size: *size },
    }
}

fn compose_transforms(parent: Transform, child: Transform) -> Transform {
    let parent_rotation = transform_rotation_matrix(parent);
    let child_rotation = transform_rotation_matrix(child);
    let rotated_translation = multiply_matrix_vector(parent_rotation, child.translation);
    Transform::matrix(
        [
            parent.translation[0] + rotated_translation[0],
            parent.translation[1] + rotated_translation[1],
            parent.translation[2] + rotated_translation[2],
        ],
        multiply_matrices(parent_rotation, child_rotation),
    )
}

fn transform_rotation_matrix(transform: Transform) -> [[f32; 3]; 3] {
    transform
        .rotation_matrix
        .unwrap_or_else(|| rpy_to_matrix(transform.rotation))
}

fn rpy_to_matrix([roll, pitch, yaw]: [f32; 3]) -> [[f32; 3]; 3] {
    let (sin_roll, cos_roll) = roll.sin_cos();
    let (sin_pitch, cos_pitch) = pitch.sin_cos();
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    [
        [
            cos_yaw * cos_pitch,
            cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
            cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
        ],
        [
            sin_yaw * cos_pitch,
            sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
            sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
        ],
        [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll],
    ]
}

fn multiply_matrices(left: [[f32; 3]; 3], right: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            result[row][column] = (0..3)
                .map(|index| left[row][index] * right[index][column])
                .sum();
        }
    }
    result
}

fn multiply_matrix_vector(matrix: [[f32; 3]; 3], vector: [f32; 3]) -> [f32; 3] {
    [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]
}

fn matrix_to_rpy(matrix: [[f32; 3]; 3]) -> [f32; 3] {
    let pitch = (-matrix[2][0]).asin();
    let cos_pitch = pitch.cos();
    if cos_pitch.abs() > 1.0e-6 {
        [
            matrix[2][1].atan2(matrix[2][2]),
            pitch,
            matrix[1][0].atan2(matrix[0][0]),
        ]
    } else {
        [0.0, pitch, (-matrix[0][1]).atan2(matrix[1][1])]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collider_wireframes_are_disabled_until_explicitly_enabled() {
        let mut world = WorldState::new();
        let mut node = Node::new("floor");
        node.collider = Some(Collider::Box {
            size: [2.0, 3.0, 0.1],
        });
        world.nodes.insert(node);

        assert!(world.collider_wireframes().is_empty());
    }

    #[test]
    fn collider_wireframes_merge_native_and_backend_entries_in_stable_order() {
        let mut world = WorldState::new();
        world.collider_debug.enabled = true;
        world.push_collider_wireframe(ColliderWireframe::new(
            "robot-link:shoulder",
            "robotLink",
            Transform::translated([0.0, 0.0, 1.0]),
            ColliderWireframeShape::Compound {
                children: vec![ColliderWireframeChild {
                    transform: Transform::translated([0.0, 0.1, 0.0]),
                    shape: ColliderWireframeShape::Cylinder {
                        radius: 0.02,
                        height: 0.15,
                    },
                }],
            },
        ));
        let mut node = Node::new("floor");
        node.collider = Some(Collider::Box {
            size: [2.0, 3.0, 0.1],
        });
        world.nodes.insert(node);

        let wireframes = world.collider_wireframes();

        assert_eq!(wireframes.len(), 2);
        assert_eq!(wireframes[0].id, "pge.scene-collider:floor");
        assert_eq!(wireframes[0].category, "sceneCollider");
        assert_eq!(wireframes[1].id, "robot-link:shoulder");
        assert_eq!(wireframes[1].category, "robotLink");
        assert!(matches!(
            wireframes[1].shape,
            ColliderWireframeShape::Compound { .. }
        ));
    }

    #[test]
    fn native_collider_uses_composed_parent_pose_without_creating_a_node() {
        let mut world = WorldState::new();
        world.collider_debug.enabled = true;
        let mut parent = Node::new("parent");
        parent.transform = Transform {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, std::f32::consts::FRAC_PI_2],
            rotation_matrix: None,
        };
        let parent_id = world.nodes.insert(parent);
        let mut child = Node::new("child");
        child.parent = NodeParent::Node(parent_id);
        child.transform = Transform::translated([1.0, 0.0, 0.0]);
        child.collider = Some(Collider::Sphere { radius: 0.05 });
        world.nodes.insert(child);

        let wireframes = world.collider_wireframes();
        let child = wireframes
            .iter()
            .find(|wireframe| wireframe.id == "pge.scene-collider:child")
            .expect("child collider wireframe");

        assert!((child.transform.translation[0] - 1.0).abs() < 1.0e-5);
        assert!((child.transform.translation[1] - 1.0).abs() < 1.0e-5);
        assert_eq!(world.nodes.len(), 2);
    }
}
