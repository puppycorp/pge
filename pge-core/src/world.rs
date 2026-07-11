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
