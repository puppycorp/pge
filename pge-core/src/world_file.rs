//! Hand-editable JSON interchange for [`WorldState`].
//!
//! [`WorldDocument`] mirrors the runtime world as plain collections keyed by
//! stable string identifiers, so a project file stays reviewable and diffable
//! without exposing [`Arena`] internals or integer [`ArenaId`] handles.
//!
//! Cross-references between nodes, meshes, materials, textures, cameras,
//! lights, scenes, and joints are expressed by stable string IDs. Asset
//! meshes and textures reference external files by path; geometry is never
//! embedded inline.

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::{
    Arena, ArenaId, Camera, CameraDistortion, CameraIntrinsics, CameraProjection,
    CameraSensorEffects, Collider, ColliderDebugOverlay, EntityId, EntityMetadata, Joint,
    JointKind, Light, LightKind, Material, Mesh, MeshSource, Node, NodeParent, PhysicsBody, Scene,
    TextLabel, Texture, TextureSource, Transform, WorldState,
};

/// Stable on-disk schema version. Bumping this resets migration expectations.
pub const WORLD_FILE_FORMAT_VERSION: u32 = 1;

/// JSON document representing a complete [`WorldState`] in a hand-editable form.
///
/// Each collection is an array of records whose `id` field is the stable string
/// other records reference. On load, the IDs are translated back to runtime
/// [`ArenaId`] handles, so callers never see integer indices in the file.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WorldDocument {
    pub version: u32,
    #[serde(default)]
    pub scenes: Vec<SceneRecord>,
    #[serde(default)]
    pub nodes: Vec<NodeRecord>,
    #[serde(default)]
    pub meshes: Vec<MeshRecord>,
    #[serde(default)]
    pub materials: Vec<MaterialRecord>,
    #[serde(default)]
    pub textures: Vec<TextureRecord>,
    #[serde(default)]
    pub cameras: Vec<CameraRecord>,
    #[serde(default)]
    pub lights: Vec<LightRecord>,
    #[serde(default)]
    pub joints: Vec<JointRecord>,
    #[serde(default)]
    pub entities: Vec<EntityMetadata>,
    #[serde(default)]
    pub text_labels: Vec<TextLabel>,
    #[serde(default)]
    pub collider_debug: ColliderDebugOverlay,
}

impl WorldDocument {
    /// Serializes the document as pretty-printed JSON.
    pub fn to_json_pretty(&self) -> Result<String, WorldFileError> {
        serde_json::to_string_pretty(self).map_err(|error| WorldFileError::Serialize {
            message: error.to_string(),
        })
    }

    /// Parses a document from a JSON string.
    pub fn from_json_str(json: &str) -> Result<Self, WorldFileError> {
        serde_json::from_str(json).map_err(|error| WorldFileError::Parse {
            message: error.to_string(),
        })
    }
}

/// A scene node's parent expressed as a stable string reference.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DocumentNodeParent {
    Node(String),
    Scene(String),
    Orphan,
}

impl Default for DocumentNodeParent {
    fn default() -> Self {
        Self::Orphan
    }
}

/// One scene entry in a [`WorldDocument`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SceneRecord {
    pub id: String,
    pub name: Option<String>,
    pub gravity_mps2: [f32; 3],
    pub physics_enabled: bool,
}

impl Default for SceneRecord {
    fn default() -> Self {
        let scene = Scene::default();
        Self {
            id: String::new(),
            name: scene.name,
            gravity_mps2: scene.gravity_mps2,
            physics_enabled: scene.physics_enabled,
        }
    }
}

/// One node entry in a [`WorldDocument`]. `mesh`, `camera`, and `light` are
/// stable string IDs referring to records in their respective collections.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeRecord {
    pub id: String,
    pub entity: EntityId,
    pub name: Option<String>,
    #[serde(default)]
    pub parent: DocumentNodeParent,
    #[serde(default)]
    pub transform: Transform,
    pub mesh: Option<String>,
    pub camera: Option<String>,
    pub light: Option<String>,
    pub body: Option<PhysicsBody>,
    pub collider: Option<Collider>,
}

/// One mesh entry in a [`WorldDocument`]. `material` is a stable string ID
/// referring to a [`MaterialRecord`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshRecord {
    pub id: String,
    pub name: Option<String>,
    pub source: MeshSource,
    pub material: Option<String>,
}

/// One material entry in a [`WorldDocument`]. `base_color_texture` is a stable
/// string ID referring to a [`TextureRecord`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MaterialRecord {
    pub id: String,
    pub name: Option<String>,
    pub base_color_factor: [f32; 4],
    pub base_color_texture: Option<String>,
    pub metallic_factor: f32,
    pub roughness_factor: f32,
    pub emissive_factor: [f32; 3],
}

impl Default for MaterialRecord {
    fn default() -> Self {
        let material = Material::default();
        Self {
            id: String::new(),
            name: material.name,
            base_color_factor: material.base_color_factor,
            base_color_texture: None,
            metallic_factor: material.metallic_factor,
            roughness_factor: material.roughness_factor,
            emissive_factor: material.emissive_factor,
        }
    }
}

/// One texture entry in a [`WorldDocument`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TextureRecord {
    pub id: String,
    pub name: String,
    pub source: TextureSource,
}

/// One camera entry in a [`WorldDocument`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CameraRecord {
    pub id: String,
    pub name: Option<String>,
    pub fov_deg: f32,
    pub projection: CameraProjection,
    pub resolution: [u32; 2],
    pub intrinsics: Option<CameraIntrinsics>,
    pub distortion: Option<CameraDistortion>,
    pub depth_range_m: Option<[f32; 2]>,
    pub sensor_effects: Option<CameraSensorEffects>,
}

impl Default for CameraRecord {
    fn default() -> Self {
        let camera = Camera::default();
        Self {
            id: String::new(),
            name: camera.name,
            fov_deg: camera.fov_deg,
            projection: camera.projection,
            resolution: camera.resolution,
            intrinsics: camera.intrinsics,
            distortion: camera.distortion,
            depth_range_m: camera.depth_range_m,
            sensor_effects: camera.sensor_effects,
        }
    }
}

/// One light entry in a [`WorldDocument`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LightRecord {
    pub id: String,
    pub name: Option<String>,
    pub kind: LightKind,
    #[serde(default = "default_light_color")]
    pub color_rgb: [u8; 3],
    #[serde(default = "default_light_intensity")]
    pub intensity: f32,
}

fn default_light_color() -> [u8; 3] {
    [255, 255, 255]
}

fn default_light_intensity() -> f32 {
    1.0
}

/// One joint entry in a [`WorldDocument`]. `parent` and `child` are stable
/// string IDs referring to [`NodeRecord`] entries.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointRecord {
    pub id: String,
    pub name: Option<String>,
    pub parent: String,
    pub child: String,
    pub kind: JointKind,
}

/// Why a world file could not be loaded or written.
#[derive(Clone, Debug, PartialEq)]
pub enum WorldFileError {
    UnsupportedVersion {
        file_version: u32,
        expected: u32,
    },
    DuplicateId {
        collection: &'static str,
        id: String,
    },
    UnknownId {
        collection: &'static str,
        id: String,
    },
    Parse {
        message: String,
    },
    Serialize {
        message: String,
    },
    Read {
        path: PathBuf,
        message: String,
    },
    Write {
        path: PathBuf,
        message: String,
    },
}

impl fmt::Display for WorldFileError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedVersion {
                file_version,
                expected,
            } => write!(
                formatter,
                "unsupported world file version {file_version}, expected {expected}"
            ),
            Self::DuplicateId { collection, id } => {
                write!(formatter, "duplicate {collection} id '{id}' in world file")
            }
            Self::UnknownId { collection, id } => write!(
                formatter,
                "{collection} id '{id}' referenced in world file does not exist"
            ),
            Self::Parse { message } => write!(formatter, "could not parse world file: {message}"),
            Self::Serialize { message } => {
                write!(formatter, "could not serialize world file: {message}")
            }
            Self::Read { path, message } => write!(
                formatter,
                "could not read world file '{}': {message}",
                path.display()
            ),
            Self::Write { path, message } => write!(
                formatter,
                "could not write world file '{}': {message}",
                path.display()
            ),
        }
    }
}

impl std::error::Error for WorldFileError {}

impl WorldState {
    /// Builds a [`WorldDocument`] snapshot of this world with stable string IDs.
    ///
    /// IDs are derived from each item's natural name (entity ID for nodes,
    /// `name` for meshes/materials/etc.) with `#2`, `#3`, … suffixes used to
    /// disambiguate duplicates within a collection.
    pub fn to_document(&self) -> WorldDocument {
        let scene_ids = assign_ids(&self.scenes, default_scene_id);
        let mesh_ids = assign_ids(&self.meshes, default_mesh_id);
        let material_ids = assign_ids(&self.materials, default_material_id);
        let texture_ids = assign_ids(&self.textures, default_texture_id);
        let camera_ids = assign_ids(&self.cameras, default_camera_id);
        let light_ids = assign_ids(&self.lights, default_light_id);
        let node_ids = assign_ids(&self.nodes, default_node_id);
        let joint_ids = assign_ids(&self.joints, default_joint_id);

        let scenes = self
            .scenes
            .iter()
            .map(|(id, scene)| SceneRecord {
                id: scene_ids[&id.index()].clone(),
                name: scene.name.clone(),
                gravity_mps2: scene.gravity_mps2,
                physics_enabled: scene.physics_enabled,
            })
            .collect();
        let nodes = self
            .nodes
            .iter()
            .map(|(id, node)| NodeRecord {
                id: node_ids[&id.index()].clone(),
                entity: node.entity.clone(),
                name: node.name.clone(),
                parent: document_parent(&node.parent, &scene_ids, &node_ids),
                transform: node.transform,
                mesh: node.mesh.map(|handle| mesh_ids[&handle.index()].clone()),
                camera: node
                    .camera
                    .map(|handle| camera_ids[&handle.index()].clone()),
                light: node.light.map(|handle| light_ids[&handle.index()].clone()),
                body: node.body,
                collider: node.collider.clone(),
            })
            .collect();
        let meshes = self
            .meshes
            .iter()
            .map(|(id, mesh)| MeshRecord {
                id: mesh_ids[&id.index()].clone(),
                name: mesh.name.clone(),
                source: mesh.source.clone(),
                material: mesh
                    .material
                    .map(|handle| material_ids[&handle.index()].clone()),
            })
            .collect();
        let materials = self
            .materials
            .iter()
            .map(|(id, material)| MaterialRecord {
                id: material_ids[&id.index()].clone(),
                name: material.name.clone(),
                base_color_factor: material.base_color_factor,
                base_color_texture: material
                    .base_color_texture
                    .map(|handle| texture_ids[&handle.index()].clone()),
                metallic_factor: material.metallic_factor,
                roughness_factor: material.roughness_factor,
                emissive_factor: material.emissive_factor,
            })
            .collect();
        let textures = self
            .textures
            .iter()
            .map(|(id, texture)| TextureRecord {
                id: texture_ids[&id.index()].clone(),
                name: texture.name.clone(),
                source: texture.source.clone(),
            })
            .collect();
        let cameras = self
            .cameras
            .iter()
            .map(|(id, camera)| CameraRecord {
                id: camera_ids[&id.index()].clone(),
                name: camera.name.clone(),
                fov_deg: camera.fov_deg,
                projection: camera.projection,
                resolution: camera.resolution,
                intrinsics: camera.intrinsics,
                distortion: camera.distortion,
                depth_range_m: camera.depth_range_m,
                sensor_effects: camera.sensor_effects,
            })
            .collect();
        let lights = self
            .lights
            .iter()
            .map(|(id, light)| LightRecord {
                id: light_ids[&id.index()].clone(),
                name: light.name.clone(),
                kind: light.kind,
                color_rgb: light.color_rgb,
                intensity: light.intensity,
            })
            .collect();
        let joints = self
            .joints
            .iter()
            .map(|(id, joint)| JointRecord {
                id: joint_ids[&id.index()].clone(),
                name: joint.name.clone(),
                parent: node_ids[&joint.parent.index()].clone(),
                child: node_ids[&joint.child.index()].clone(),
                kind: joint.kind,
            })
            .collect();

        WorldDocument {
            version: WORLD_FILE_FORMAT_VERSION,
            scenes,
            nodes,
            meshes,
            materials,
            textures,
            cameras,
            lights,
            joints,
            entities: self.entities.clone(),
            text_labels: self.text_labels.clone(),
            collider_debug: self.collider_debug.clone(),
        }
    }

    /// Rebuilds a [`WorldState`] from a [`WorldDocument`], translating stable
    /// string IDs back into runtime [`ArenaId`] handles.
    pub fn from_document(document: &WorldDocument) -> Result<Self, WorldFileError> {
        if document.version != WORLD_FILE_FORMAT_VERSION {
            return Err(WorldFileError::UnsupportedVersion {
                file_version: document.version,
                expected: WORLD_FILE_FORMAT_VERSION,
            });
        }

        let mut world = WorldState::new();
        let mut scene_ids = HashMap::<String, ArenaId<Scene>>::new();
        let mut mesh_ids = HashMap::<String, ArenaId<Mesh>>::new();
        let mut material_ids = HashMap::<String, ArenaId<Material>>::new();
        let mut texture_ids = HashMap::<String, ArenaId<Texture>>::new();
        let mut camera_ids = HashMap::<String, ArenaId<Camera>>::new();
        let mut light_ids = HashMap::<String, ArenaId<Light>>::new();
        let mut node_ids = HashMap::<String, ArenaId<Node>>::new();

        for record in &document.textures {
            ensure_unique_id(&texture_ids, &record.id, "texture")?;
            let handle = world.textures.insert(Texture {
                name: record.name.clone(),
                source: record.source.clone(),
            });
            texture_ids.insert(record.id.clone(), handle);
        }
        for record in &document.materials {
            ensure_unique_id(&material_ids, &record.id, "material")?;
            let base_color_texture = record
                .base_color_texture
                .as_ref()
                .map(|id| resolve_id(&texture_ids, id, "texture"))
                .transpose()?;
            let handle = world.materials.insert(Material {
                name: record.name.clone(),
                base_color_factor: record.base_color_factor,
                base_color_texture,
                metallic_factor: record.metallic_factor,
                roughness_factor: record.roughness_factor,
                emissive_factor: record.emissive_factor,
            });
            material_ids.insert(record.id.clone(), handle);
        }
        for record in &document.meshes {
            ensure_unique_id(&mesh_ids, &record.id, "mesh")?;
            let material = record
                .material
                .as_ref()
                .map(|id| resolve_id(&material_ids, id, "material"))
                .transpose()?;
            let handle = world.meshes.insert(Mesh {
                name: record.name.clone(),
                source: record.source.clone(),
                material,
            });
            mesh_ids.insert(record.id.clone(), handle);
        }
        for record in &document.cameras {
            ensure_unique_id(&camera_ids, &record.id, "camera")?;
            let handle = world.cameras.insert(Camera {
                name: record.name.clone(),
                fov_deg: record.fov_deg,
                projection: record.projection,
                resolution: record.resolution,
                intrinsics: record.intrinsics,
                distortion: record.distortion,
                depth_range_m: record.depth_range_m,
                sensor_effects: record.sensor_effects,
            });
            camera_ids.insert(record.id.clone(), handle);
        }
        for record in &document.lights {
            ensure_unique_id(&light_ids, &record.id, "light")?;
            let handle = world.lights.insert(Light {
                name: record.name.clone(),
                kind: record.kind,
                color_rgb: record.color_rgb,
                intensity: record.intensity,
            });
            light_ids.insert(record.id.clone(), handle);
        }
        for record in &document.scenes {
            ensure_unique_id(&scene_ids, &record.id, "scene")?;
            let handle = world.scenes.insert(Scene {
                name: record.name.clone(),
                gravity_mps2: record.gravity_mps2,
                physics_enabled: record.physics_enabled,
            });
            scene_ids.insert(record.id.clone(), handle);
        }
        for record in &document.nodes {
            ensure_unique_id(&node_ids, &record.id, "node")?;
            let mesh = record
                .mesh
                .as_ref()
                .map(|id| resolve_id(&mesh_ids, id, "mesh"))
                .transpose()?;
            let camera = record
                .camera
                .as_ref()
                .map(|id| resolve_id(&camera_ids, id, "camera"))
                .transpose()?;
            let light = record
                .light
                .as_ref()
                .map(|id| resolve_id(&light_ids, id, "light"))
                .transpose()?;
            let parent = match &record.parent {
                DocumentNodeParent::Scene(id) => {
                    NodeParent::Scene(resolve_id(&scene_ids, id, "scene")?)
                }
                DocumentNodeParent::Node(_) => NodeParent::Orphan,
                DocumentNodeParent::Orphan => NodeParent::Orphan,
            };
            let handle = world.nodes.insert(Node {
                entity: record.entity.clone(),
                name: record.name.clone(),
                parent,
                transform: record.transform,
                mesh,
                camera,
                light,
                body: record.body,
                collider: record.collider.clone(),
            });
            node_ids.insert(record.id.clone(), handle);
        }
        for record in &document.nodes {
            if let DocumentNodeParent::Node(parent_str) = &record.parent {
                let child_handle = node_ids
                    .get(&record.id)
                    .copied()
                    .expect("node inserted in previous pass");
                let parent_handle = resolve_id(&node_ids, parent_str, "node")?;
                let node = world
                    .nodes
                    .get_mut(&child_handle)
                    .expect("node inserted in previous pass");
                node.parent = NodeParent::Node(parent_handle);
            }
        }
        for record in &document.joints {
            let parent = resolve_id(&node_ids, &record.parent, "node")?;
            let child = resolve_id(&node_ids, &record.child, "node")?;
            world.joints.insert(Joint {
                name: record.name.clone(),
                parent,
                child,
                kind: record.kind,
            });
        }

        world.entities = document.entities.clone();
        world.text_labels = document.text_labels.clone();
        world.collider_debug = document.collider_debug.clone();
        Ok(world)
    }

    /// Serializes this world as a pretty-printed JSON string.
    pub fn to_json_string(&self) -> Result<String, WorldFileError> {
        self.to_document().to_json_pretty()
    }

    /// Parses a world from a JSON string.
    pub fn from_json_str(json: &str) -> Result<Self, WorldFileError> {
        let document = WorldDocument::from_json_str(json)?;
        Self::from_document(&document)
    }

    /// Writes this world to a JSON file at `path`.
    pub fn save_json(&self, path: impl AsRef<Path>) -> Result<(), WorldFileError> {
        let path = path.as_ref().to_path_buf();
        let json = self.to_json_string()?;
        std::fs::write(&path, json).map_err(|error| WorldFileError::Write {
            path,
            message: error.to_string(),
        })
    }

    /// Loads a world from a JSON file at `path`.
    pub fn load_json(path: impl AsRef<Path>) -> Result<Self, WorldFileError> {
        let path = path.as_ref().to_path_buf();
        let contents = std::fs::read_to_string(&path).map_err(|error| WorldFileError::Read {
            path: path.clone(),
            message: error.to_string(),
        })?;
        Self::from_json_str(&contents)
    }
}

fn assign_ids<T, F>(arena: &Arena<T>, make_default: F) -> HashMap<usize, String>
where
    F: Fn(&T, usize) -> String,
{
    let mut taken: HashSet<String> = HashSet::new();
    let mut out = HashMap::new();
    for (position, (handle, item)) in arena.iter().enumerate() {
        let base = make_default(item, position);
        let mut candidate = base.clone();
        let mut suffix = 2;
        while taken.contains(&candidate) {
            candidate = format!("{base}#{suffix}");
            suffix += 1;
        }
        taken.insert(candidate.clone());
        out.insert(handle.index(), candidate);
    }
    out
}

fn default_scene_id(scene: &Scene, position: usize) -> String {
    scene
        .name
        .clone()
        .unwrap_or_else(|| format!("scene-{position}"))
}

fn default_mesh_id(mesh: &Mesh, position: usize) -> String {
    mesh.name
        .clone()
        .unwrap_or_else(|| format!("mesh-{position}"))
}

fn default_material_id(material: &Material, position: usize) -> String {
    material
        .name
        .clone()
        .unwrap_or_else(|| format!("material-{position}"))
}

fn default_texture_id(texture: &Texture, position: usize) -> String {
    if texture.name.is_empty() {
        format!("texture-{position}")
    } else {
        texture.name.clone()
    }
}

fn default_camera_id(camera: &Camera, position: usize) -> String {
    camera
        .name
        .clone()
        .unwrap_or_else(|| format!("camera-{position}"))
}

fn default_light_id(light: &Light, position: usize) -> String {
    light
        .name
        .clone()
        .unwrap_or_else(|| format!("light-{position}"))
}

fn default_node_id(node: &Node, _position: usize) -> String {
    node.entity.0.clone()
}

fn default_joint_id(joint: &Joint, position: usize) -> String {
    joint
        .name
        .clone()
        .unwrap_or_else(|| format!("joint-{position}"))
}

fn document_parent(
    parent: &NodeParent,
    scene_ids: &HashMap<usize, String>,
    node_ids: &HashMap<usize, String>,
) -> DocumentNodeParent {
    match parent {
        NodeParent::Node(handle) => {
            DocumentNodeParent::Node(node_ids.get(&handle.index()).cloned().unwrap_or_default())
        }
        NodeParent::Scene(handle) => {
            DocumentNodeParent::Scene(scene_ids.get(&handle.index()).cloned().unwrap_or_default())
        }
        NodeParent::Orphan => DocumentNodeParent::Orphan,
    }
}

fn ensure_unique_id<T>(
    map: &HashMap<String, ArenaId<T>>,
    id: &str,
    collection: &'static str,
) -> Result<(), WorldFileError> {
    if id.trim().is_empty() {
        return Err(WorldFileError::DuplicateId {
            collection,
            id: id.to_string(),
        });
    }
    if map.contains_key(id) {
        return Err(WorldFileError::DuplicateId {
            collection,
            id: id.to_string(),
        });
    }
    Ok(())
}

fn resolve_id<T>(
    map: &HashMap<String, ArenaId<T>>,
    id: &str,
    collection: &'static str,
) -> Result<ArenaId<T>, WorldFileError>
where
    ArenaId<T>: Copy,
{
    map.get(id)
        .copied()
        .ok_or_else(|| WorldFileError::UnknownId {
            collection,
            id: id.to_string(),
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Collider, ColliderWireframe, ColliderWireframeShape, Geometry, GeometryBounds, MeshSource,
        Node, NodeParent, PhysicsBody, Transform, WorldState,
    };

    #[test]
    fn empty_world_round_trips() {
        let world = WorldState::new();
        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");
        assert_eq!(world, restored);
    }

    #[test]
    fn json_output_uses_stable_string_ids_and_hides_arena_internals() {
        let mut world = WorldState::new();
        world.scenes.insert(Scene {
            name: Some("main".to_string()),
            gravity_mps2: [0.0, 0.0, -9.81],
            physics_enabled: true,
        });
        world.nodes.insert(Node::new("floor"));

        let json = world.to_json_string().expect("serialize");

        assert!(json.contains("\"version\""));
        assert!(json.contains("\"id\": \"main\""));
        assert!(json.contains("\"id\": \"floor\""));
        assert!(!json.contains("\"free_slots\""));
        assert!(!json.contains("\"items\""));
    }

    #[test]
    fn node_with_mesh_material_and_texture_round_trips() {
        let mut world = WorldState::new();
        let texture_id = world.textures.insert(Texture {
            name: "checker".to_string(),
            source: TextureSource::File("./assets/checker.png".to_string()),
        });
        let material_id = world.materials.insert(Material {
            name: Some("floor_mat".to_string()),
            base_color_texture: Some(texture_id),
            ..Material::default()
        });
        let mesh_id = world.meshes.insert(Mesh {
            name: Some("floor_mesh".to_string()),
            source: MeshSource::Procedural(Geometry::Box {
                size: [4.0, 4.0, 0.1],
            }),
            material: Some(material_id),
        });
        let scene_id = world.scenes.insert(Scene {
            name: Some("main".to_string()),
            gravity_mps2: [0.0, 0.0, -9.81],
            physics_enabled: true,
        });
        let mut node = Node::new("floor");
        node.parent = NodeParent::Scene(scene_id);
        node.mesh = Some(mesh_id);
        world.nodes.insert(node);

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
        assert_eq!(restored.nodes.len(), 1);
        assert_eq!(restored.meshes.len(), 1);
        assert_eq!(restored.materials.len(), 1);
        assert_eq!(restored.textures.len(), 1);
    }

    #[test]
    fn node_parent_chain_round_trips() {
        let mut world = WorldState::new();
        let parent_id = world.nodes.insert(Node::new("parent"));
        let mut child = Node::new("child");
        child.parent = NodeParent::Node(parent_id);
        world.nodes.insert(child);

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
        let restored_child = restored
            .nodes
            .iter()
            .find(|(_, node)| node.entity.0 == "child")
            .map(|(_, node)| node.clone())
            .expect("child present");
        assert!(matches!(restored_child.parent, NodeParent::Node(_)));
    }

    #[test]
    fn duplicate_node_entity_ids_are_disambiguated_and_round_trip() {
        let mut world = WorldState::new();
        world.nodes.insert(Node::new("link"));
        world.nodes.insert(Node::new("link"));

        let json = world.to_json_string().expect("serialize");
        assert!(json.contains("\"id\": \"link\""));
        assert!(json.contains("\"id\": \"link#2\""));

        let restored = WorldState::from_json_str(&json).expect("deserialize");
        assert_eq!(world, restored);
        assert_eq!(restored.nodes.len(), 2);
    }

    #[test]
    fn joints_round_trip_through_node_parent_child_refs() {
        let mut world = WorldState::new();
        let parent_id = world.nodes.insert(Node::new("parent"));
        let child_id = world.nodes.insert(Node::new("child"));
        world.joints.insert(Joint {
            name: Some("shoulder".to_string()),
            parent: parent_id,
            child: child_id,
            kind: JointKind::Revolute {
                axis: [0.0, 0.0, 1.0],
                limits_rad: Some([-1.5, 1.5]),
            },
        });

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
        assert_eq!(restored.joints.len(), 1);
    }

    #[test]
    fn collider_debug_overlay_round_trips() {
        let mut world = WorldState::new();
        world.collider_debug.enabled = true;
        world.push_collider_wireframe(ColliderWireframe::new(
            "robot-link:shoulder",
            "robotLink",
            Transform::translated([0.0, 0.0, 1.0]),
            ColliderWireframeShape::Cylinder {
                radius: 0.02,
                height: 0.15,
            },
        ));

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
        assert!(restored.collider_debug.enabled);
        assert_eq!(restored.collider_debug.wireframes.len(), 1);
    }

    #[test]
    fn procedural_and_asset_mesh_sources_round_trip() {
        let mut world = WorldState::new();
        world.meshes.insert(Mesh {
            name: Some("box".to_string()),
            source: MeshSource::Procedural(Geometry::Box {
                size: [1.0, 2.0, 3.0],
            }),
            material: None,
        });
        world.meshes.insert(Mesh {
            name: Some("orkki".to_string()),
            source: MeshSource::Asset {
                path: "./assets/orkki.glb".to_string(),
                scale: [2.0; 3],
                bounds: Some(GeometryBounds {
                    min: [-1.0; 3],
                    max: [1.0; 3],
                }),
            },
            material: None,
        });

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
        assert_eq!(restored.meshes.len(), 2);
    }

    #[test]
    fn physics_body_and_collider_round_trip() {
        let mut world = WorldState::new();
        let mut node = Node::new("dynamic_box");
        node.body = Some(PhysicsBody {
            mass_kg: 1.5,
            friction: 0.4,
            restitution: 0.2,
            ..PhysicsBody::default()
        });
        node.collider = Some(Collider::Box {
            size: [0.2, 0.2, 0.2],
        });
        world.nodes.insert(node);

        let json = world.to_json_string().expect("serialize");
        let restored = WorldState::from_json_str(&json).expect("deserialize");

        assert_eq!(world, restored);
    }

    #[test]
    fn missing_node_reference_returns_unknown_id_error() {
        let json = r#"{
            "version": 1,
            "nodes": [
                {"id": "child", "entity": "child", "parent": {"node": "missing"}}
            ]
        }"#;
        let error = WorldState::from_json_str(json).expect_err("unknown node");
        assert!(matches!(
            error,
            WorldFileError::UnknownId {
                collection: "node",
                ..
            }
        ));
    }

    #[test]
    fn duplicate_ids_in_document_are_reported() {
        let json = r#"{
            "version": 1,
            "nodes": [
                {"id": "dup", "entity": "a"},
                {"id": "dup", "entity": "b"}
            ]
        }"#;
        let error = WorldState::from_json_str(json).expect_err("duplicate id");
        assert!(matches!(
            error,
            WorldFileError::DuplicateId {
                collection: "node",
                ..
            }
        ));
    }

    #[test]
    fn empty_id_is_rejected() {
        let json = r#"{
            "version": 1,
            "nodes": [
                {"id": "", "entity": "a"}
            ]
        }"#;
        let error = WorldState::from_json_str(json).expect_err("empty id");
        assert!(matches!(
            error,
            WorldFileError::DuplicateId {
                collection: "node",
                ..
            }
        ));
    }

    #[test]
    fn unsupported_version_is_rejected() {
        let json = r#"{"version": 999}"#;
        let error = WorldState::from_json_str(json).expect_err("version");
        assert!(matches!(error, WorldFileError::UnsupportedVersion { .. }));
    }

    #[test]
    fn empty_json_document_yields_empty_world() {
        let json = r#"{"version": 1}"#;
        let world = WorldState::from_json_str(json).expect("empty world");
        assert_eq!(world, WorldState::new());
    }

    #[test]
    fn save_and_load_json_round_trips_through_file() {
        let mut world = WorldState::new();
        world.scenes.insert(Scene {
            name: Some("scene".to_string()),
            gravity_mps2: [0.0, 0.0, -9.81],
            physics_enabled: true,
        });
        world.nodes.insert(Node::new("floor"));

        let path = std::env::temp_dir().join(format!(
            "pge-world-file-{}.json",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        world.save_json(&path).expect("save");
        let restored = WorldState::load_json(&path).expect("load");
        let _ = std::fs::remove_file(&path);
        assert_eq!(world, restored);
    }
}
