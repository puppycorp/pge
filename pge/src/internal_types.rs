use image::codecs::qoi;

use crate::ArenaId;
use crate::Camera;
use crate::Texture;

pub struct WriteCommand {
    pub start: usize,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub enum EngineEvent {
    ImageLoaded {
        texture_id: ArenaId<Texture>,
        width: u32,
        height: u32,
        data: Vec<u8>,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CamView {
    pub camera_id: ArenaId<Camera>,
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct RawCamera {
    pub model: [[f32; 4]; 4],
    pub position: [f32; 3],
    pub _padding: f32,
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable, Debug)]
pub struct RawInstance {
    pub model: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct RawPointLight {
    pub color_intensity: [f32; 4],
    pub position: [f32; 4],
}

impl RawPointLight {
    pub fn new(color: [f32; 3], intensity: f32, position: [f32; 3]) -> Self {
        Self {
            color_intensity: [color[0], color[1], color[2], intensity],
            position: [position[0], position[1], position[2], 0.0],
        }
    }
}

pub const MAX_POINT_LIGHTS: usize = 16;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct RawPointLightBuffer {
    pub count: u32,
    pub _padding: [u32; 3],
    pub lights: [RawPointLight; MAX_POINT_LIGHTS],
}

impl Default for RawPointLightBuffer {
    fn default() -> Self {
        Self {
            count: 0,
            _padding: [0; 3],
            lights: [RawPointLight::new([0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0]); MAX_POINT_LIGHTS],
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct RawMaterial {
    pub base_color_factor: [f32; 4], // 16 bytes
    pub metallic_factor: f32,        // 4 bytes
    pub roughness_factor: f32,       // 4 bytes
    pub normal_texture_scale: f32,   // 4 bytes
    pub occlusion_strength: f32,     // 4 bytes
    pub emissive_factor: [f32; 3],   // 12 bytes
    pub _padding: f32,               // 4 bytes to align to 16 bytes
}

impl Default for RawMaterial {
    fn default() -> Self {
        Self {
            base_color_factor: [0.8, 0.8, 0.8, 1.0], // Light gray
            metallic_factor: 0.0,
            roughness_factor: 0.5,
            normal_texture_scale: 1.0,
            occlusion_strength: 1.0,
            emissive_factor: [0.0, 0.0, 0.0],
            _padding: 0.0,
        }
    }
}
