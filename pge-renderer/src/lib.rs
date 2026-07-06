use pge_core::{CameraDistortion, CameraIntrinsics, CameraProjection, CameraSensorEffects, EntityId, Transform, WorldState};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RenderView {
    Rgb,
    Depth,
    Segmentation,
    Normal,
    Albedo,
    MaterialProperties,
    WorldPosition,
    State,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FrameKind {
    Rgb,
    Depth,
    Segmentation,
    Normal,
    Albedo,
    MaterialProperties,
    WorldPosition,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrameBuffer {
    pub kind: FrameKind,
    pub width: u32,
    pub height: u32,
    pub bytes: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RenderRequest {
    pub camera_id: Option<EntityId>,
    pub views: Vec<RenderView>,
    pub resolution: [u32; 2],
    pub settings: Option<RenderSettings>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RenderOutput {
    pub metadata: RenderMetadata,
    pub frames: Vec<FrameBuffer>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RenderMetadata {
    pub timestamp_sec: f64,
    pub camera_id: Option<EntityId>,
    pub camera_pose: Option<Transform>,
    pub camera_projection: Option<CameraProjection>,
    pub camera_intrinsics: Option<CameraIntrinsics>,
    pub camera_distortion: Option<CameraDistortion>,
    pub sensor_effects: Option<CameraSensorEffects>,
    pub resolution: [u32; 2],
    pub views: Vec<RenderView>,
    pub settings: Option<RenderSettings>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ToneMapping {
    Linear,
    Reinhard,
    Aces,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EnvironmentSettings {
    pub sky_top_rgb: [u8; 3],
    pub sky_horizon_rgb: [u8; 3],
    pub ground_rgb: [u8; 3],
    pub map: Option<String>,
    pub map_rotation_deg: f32,
    pub intensity: f32,
    pub ambient_intensity: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReflectionProbeSettings {
    pub map: String,
    pub rotation_deg: f32,
    pub intensity: f32,
    pub ambient_intensity: f32,
    pub position: Option<[f32; 3]>,
    pub box_size_m: Option<[f32; 3]>,
    pub influence_radius_m: Option<f32>,
    pub falloff_power: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RenderSettings {
    pub background_rgb: [u8; 3],
    pub ambient_rgb: [u8; 3],
    pub ambient_intensity: f32,
    pub tone_mapping: ToneMapping,
    pub tone_exposure: f32,
    pub environment: Option<EnvironmentSettings>,
    pub reflection_probes: Vec<ReflectionProbeSettings>,
    pub debug_rgb_samples_per_pixel: u32,
    pub soft_shadow_samples: u32,
    pub area_light_samples: u32,
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            background_rgb: [22, 35, 49],
            ambient_rgb: [255, 255, 255],
            ambient_intensity: 0.35,
            tone_mapping: ToneMapping::Linear,
            tone_exposure: 1.0,
            environment: None,
            reflection_probes: Vec::new(),
            debug_rgb_samples_per_pixel: 1,
            soft_shadow_samples: 1,
            area_light_samples: 1,
        }
    }
}

pub trait Renderer {
    fn render(&mut self, world: &WorldState, request: &RenderRequest) -> Result<RenderOutput, RenderError>;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RenderError {
    pub message: String,
}

impl RenderError {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl std::fmt::Display for RenderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for RenderError {}

#[derive(Clone, Debug, Default)]
pub struct NullRenderer;

impl Renderer for NullRenderer {
    fn render(&mut self, _world: &WorldState, request: &RenderRequest) -> Result<RenderOutput, RenderError> {
        Ok(RenderOutput {
            metadata: RenderMetadata {
                timestamp_sec: 0.0,
                camera_id: request.camera_id.clone(),
                camera_pose: None,
                camera_projection: None,
                camera_intrinsics: None,
                camera_distortion: None,
                sensor_effects: None,
                resolution: request.resolution,
                views: request.views.clone(),
                settings: request.settings.clone(),
            },
            frames: Vec::new(),
        })
    }
}
