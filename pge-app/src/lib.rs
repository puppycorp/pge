use std::sync::{Arc, Mutex};

use bytemuck::{Pod, Zeroable};

mod arena;
mod buffer;
mod collision_detection;
mod compositor;
pub mod core;
mod debug;
pub mod editor;
pub mod engine;
pub mod free_fly;
mod gltf;
pub mod gui;
mod hardware;
mod internal_types;
#[path = "wgpu/mod.rs"]
mod legacy_wgpu;
mod log;
mod mock_hardware;
pub mod orbit;
pub mod physics;
pub mod shapes;
mod spatial_grid;
mod state;
#[cfg(test)]
mod tests;
pub mod text;
pub mod types;
mod urdf;
pub mod utility;

pub use arena::*;
pub use editor::{with_editor, EditorApp, EditorPlugin, EditorSettings};
pub use free_fly::*;
pub use glam::*;
pub use gltf::load_gltf;
pub use gui::*;
pub use legacy_wgpu::{run, run_with_event_sender};
pub use log::*;
pub use orbit::*;
pub use shapes::*;
pub use state::*;
pub use types::*;
pub use urdf::load_urdf;

use pge_core::{Arena as CoreArena, ArenaId as CoreArenaId, WorldState};
use pge_renderer::{RenderError, RenderRequest};
use pge_wgpu_renderer::WgpuRenderer;
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::{ElementState, MouseButton as WinitMouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window as WinitWindow, WindowAttributes};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AppWindow {
    pub title: String,
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AppGuiElement {
    pub name: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct InputState {
    pub focused_window: Option<CoreArenaId<AppWindow>>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AppState {
    pub windows: CoreArena<AppWindow>,
    pub guis: CoreArena<AppGuiElement>,
    pub input: InputState,
    pub screenshot_request: Option<(CoreArenaId<AppWindow>, String)>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct EngineState {
    pub world: WorldState,
    pub app: AppState,
}

#[derive(Clone, Debug)]
pub struct WindowRenderConfig {
    pub title: String,
    pub resolution: [u32; 2],
}

impl Default for WindowRenderConfig {
    fn default() -> Self {
        Self {
            title: "PGE WGPU Renderer".to_string(),
            resolution: [640, 360],
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WindowOverlayLines {
    lines: Arc<Mutex<Vec<String>>>,
}

impl WindowOverlayLines {
    pub fn set(&self, lines: Vec<String>) {
        if let Ok(mut current) = self.lines.lock() {
            *current = lines
                .into_iter()
                .take(4)
                .map(|line| line.chars().take(32).collect())
                .collect();
        }
    }

    fn snapshot(&self) -> Vec<String> {
        self.lines
            .lock()
            .map(|lines| lines.clone())
            .unwrap_or_default()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WindowFrameContext {
    pub frame_index: u64,
    pub elapsed_sec: f64,
    pub input: WindowInputState,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WindowInputState {
    pub cursor_position_px: Option<[f32; 2]>,
    pub left_drag_delta_px: [f32; 2],
    pub middle_drag_delta_px: [f32; 2],
    pub right_drag_delta_px: [f32; 2],
    pub scroll_delta_lines: f32,
    pub left_drag_active: bool,
    pub middle_drag_active: bool,
    pub right_drag_active: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct OverlayVertex {
    position: [f32; 2],
    color: [f32; 4],
}

impl OverlayVertex {
    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<OverlayVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 2]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
    }
}

struct FpsOverlayRenderer {
    pipeline: wgpu::RenderPipeline,
}

impl FpsOverlayRenderer {
    fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("pge fps overlay shader"),
            source: wgpu::ShaderSource::Wgsl(FPS_OVERLAY_SHADER.into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pge fps overlay pipeline layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("pge fps overlay pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[OverlayVertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });
        Self { pipeline }
    }

    fn render(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        view: &wgpu::TextureView,
        resolution: [u32; 2],
        fps: f32,
        additional_lines: &[String],
    ) {
        let text = if fps > 0.0 {
            format!("{fps:.0} FPS")
        } else {
            "-- FPS".to_string()
        };
        let vertices = fps_overlay_vertices(&text, additional_lines, resolution);
        if vertices.is_empty() {
            return;
        }
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("pge fps overlay vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("pge fps overlay encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pge fps overlay pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            pass.draw(0..vertices.len() as u32, 0..1);
        }
        queue.submit(std::iter::once(encoder.finish()));
    }
}

struct TextOverlayRenderer {
    pipeline: wgpu::RenderPipeline,
    cached_labels: Vec<pge_core::TextLabel>,
    cached_resolution: [u32; 2],
    vertex_buffer: Option<wgpu::Buffer>,
    vertex_count: u32,
}

/// Opt-in timing for a native WGPU window frame.  The renderer profile covers
/// offscreen work only; this additionally exposes surface acquisition,
/// presentation, and PGE overlays when diagnosing a live desktop preview.
struct WindowProfile {
    started_at: std::time::Instant,
    frames: u64,
    update: std::time::Duration,
    acquire: std::time::Duration,
    scene: std::time::Duration,
    fps_overlay: std::time::Duration,
    text_overlay: std::time::Duration,
    present: std::time::Duration,
    redraw: std::time::Duration,
}

impl WindowProfile {
    fn from_environment() -> Option<Self> {
        std::env::var_os("PGE_WINDOW_PROFILE").map(|_| Self {
            started_at: std::time::Instant::now(),
            frames: 0,
            update: std::time::Duration::ZERO,
            acquire: std::time::Duration::ZERO,
            scene: std::time::Duration::ZERO,
            fps_overlay: std::time::Duration::ZERO,
            text_overlay: std::time::Duration::ZERO,
            present: std::time::Duration::ZERO,
            redraw: std::time::Duration::ZERO,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn record(
        &mut self,
        update: std::time::Duration,
        acquire: std::time::Duration,
        scene: std::time::Duration,
        fps_overlay: std::time::Duration,
        text_overlay: std::time::Duration,
        present: std::time::Duration,
        redraw: std::time::Duration,
    ) {
        self.frames += 1;
        self.update += update;
        self.acquire += acquire;
        self.scene += scene;
        self.fps_overlay += fps_overlay;
        self.text_overlay += text_overlay;
        self.present += present;
        self.redraw += redraw;

        let elapsed = self.started_at.elapsed();
        if elapsed < std::time::Duration::from_secs(1) {
            return;
        }
        let frames = self.frames.max(1) as f64;
        let avg_ms = |duration: std::time::Duration| duration.as_secs_f64() * 1_000.0 / frames;
        eprintln!(
            "PGE window profile: {:.1} fps; avg ms update={:.3} acquire={:.3} scene={:.3} fps-overlay={:.3} text-overlay={:.3} present={:.3} redraw={:.3}; frames={}",
            frames / elapsed.as_secs_f64(),
            avg_ms(self.update),
            avg_ms(self.acquire),
            avg_ms(self.scene),
            avg_ms(self.fps_overlay),
            avg_ms(self.text_overlay),
            avg_ms(self.present),
            avg_ms(self.redraw),
            self.frames,
        );
        self.started_at = std::time::Instant::now();
        self.frames = 0;
        self.update = std::time::Duration::ZERO;
        self.acquire = std::time::Duration::ZERO;
        self.scene = std::time::Duration::ZERO;
        self.fps_overlay = std::time::Duration::ZERO;
        self.text_overlay = std::time::Duration::ZERO;
        self.present = std::time::Duration::ZERO;
        self.redraw = std::time::Duration::ZERO;
    }
}

impl TextOverlayRenderer {
    fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        Self {
            pipeline: FpsOverlayRenderer::new(device, target_format).pipeline,
            cached_labels: Vec::new(),
            cached_resolution: [0, 0],
            vertex_buffer: None,
            vertex_count: 0,
        }
    }

    fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        view: &wgpu::TextureView,
        resolution: [u32; 2],
        labels: &[pge_core::TextLabel],
    ) {
        if self.cached_resolution != resolution || self.cached_labels != labels {
            let vertices = text_overlay_vertices(labels, resolution);
            self.vertex_count = vertices.len() as u32;
            self.vertex_buffer = (!vertices.is_empty()).then(|| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("pge text overlay vertices"),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            self.cached_labels = labels.to_vec();
            self.cached_resolution = resolution;
        }
        let Some(vertex_buffer) = &self.vertex_buffer else {
            return;
        };
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("pge text overlay encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pge text overlay pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            pass.draw(0..self.vertex_count, 0..1);
        }
        queue.submit(std::iter::once(encoder.finish()));
    }
}

const FPS_OVERLAY_SHADER: &str = r#"
struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(@location(0) position: vec2<f32>, @location(1) color: vec4<f32>) -> VertexOut {
    var out: VertexOut;
    out.position = vec4<f32>(position, 0.0, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

fn fps_overlay_vertices(
    fps_text: &str,
    additional_lines: &[String],
    resolution: [u32; 2],
) -> Vec<OverlayVertex> {
    let width = resolution[0].max(1) as f32;
    let height = resolution[1].max(1) as f32;
    let scale = 3.0_f32;
    let glyph_w = 5.0 * scale;
    let glyph_h = 7.0 * scale;
    let spacing = scale;
    let margin = 12.0_f32;
    let padding = 6.0_f32;
    let mut vertices = Vec::new();
    for (line_index, text) in std::iter::once(fps_text)
        .chain(additional_lines.iter().map(String::as_str))
        .enumerate()
    {
        let glyph_count = text.chars().filter_map(glyph_rows).count();
        if glyph_count == 0 {
            continue;
        }
        let text_width =
            glyph_count.saturating_sub(1) as f32 * spacing + glyph_count as f32 * glyph_w;
        let x = (width - margin - text_width).max(margin);
        let y = margin + line_index as f32 * (glyph_h + padding * 2.0 + 4.0);
        push_overlay_rect(
            &mut vertices,
            [width, height],
            x - padding,
            y - padding,
            text_width + padding * 2.0,
            glyph_h + padding * 2.0,
            [0.0, 0.0, 0.0, 0.48],
        );
        let mut cursor_x = x;
        for ch in text.chars() {
            let Some(rows) = glyph_rows(ch) else {
                continue;
            };
            for (row_index, row_bits) in rows.iter().enumerate() {
                for col in 0..5 {
                    let bit = 1 << (4 - col);
                    if row_bits & bit == 0 {
                        continue;
                    }
                    push_overlay_rect(
                        &mut vertices,
                        [width, height],
                        cursor_x + col as f32 * scale,
                        y + row_index as f32 * scale,
                        scale,
                        scale,
                        [0.82, 0.95, 1.0, 1.0],
                    );
                }
            }
            cursor_x += glyph_w + spacing;
        }
    }
    vertices
}

fn text_overlay_vertices(
    labels: &[pge_core::TextLabel],
    resolution: [u32; 2],
) -> Vec<OverlayVertex> {
    let width = resolution[0].max(1) as f32;
    let height = resolution[1].max(1) as f32;
    let mut vertices = Vec::new();
    let margin = 12.0_f32;
    let mut y = margin;
    for label in labels.iter().take(24) {
        let scale = (label.font_size_px.max(8.0) / 7.0).clamp(1.0, 5.0);
        let glyph_w = 5.0 * scale;
        let glyph_h = 7.0 * scale;
        let spacing = scale;
        let padding = 4.0_f32;
        let text = label.text.to_uppercase();
        let glyph_count = text.chars().filter_map(glyph_rows).count();
        if glyph_count == 0 {
            continue;
        }
        let text_width =
            glyph_count.saturating_sub(1) as f32 * spacing + glyph_count as f32 * glyph_w;
        push_overlay_rect(
            &mut vertices,
            [width, height],
            margin - padding,
            y - padding,
            text_width + padding * 2.0,
            glyph_h + padding * 2.0,
            label.background_color,
        );
        let mut cursor_x = margin;
        for ch in text.chars() {
            let Some(rows) = glyph_rows(ch) else {
                continue;
            };
            for (row_index, row_bits) in rows.iter().enumerate() {
                for col in 0..5 {
                    let bit = 1 << (4 - col);
                    if row_bits & bit == 0 {
                        continue;
                    }
                    push_overlay_rect(
                        &mut vertices,
                        [width, height],
                        cursor_x + col as f32 * scale,
                        y + row_index as f32 * scale,
                        scale,
                        scale,
                        label.color,
                    );
                }
            }
            cursor_x += glyph_w + spacing;
        }
        y += glyph_h + padding * 2.0 + 4.0;
        if y > height - glyph_h {
            break;
        }
    }
    vertices
}

fn glyph_rows(ch: char) -> Option<[u8; 7]> {
    match ch {
        '0' => Some([
            0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110,
        ]),
        '1' => Some([
            0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110,
        ]),
        '2' => Some([
            0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111,
        ]),
        '3' => Some([
            0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110,
        ]),
        '4' => Some([
            0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010,
        ]),
        '5' => Some([
            0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001, 0b11110,
        ]),
        '6' => Some([
            0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110,
        ]),
        '7' => Some([
            0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000,
        ]),
        '8' => Some([
            0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110,
        ]),
        '9' => Some([
            0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100,
        ]),
        'F' => Some([
            0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000,
        ]),
        'P' => Some([
            0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000,
        ]),
        'S' => Some([
            0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110,
        ]),
        'A' => Some([
            0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001,
        ]),
        'B' => Some([
            0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110,
        ]),
        'C' => Some([
            0b01111, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b01111,
        ]),
        'D' => Some([
            0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110,
        ]),
        'E' => Some([
            0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111,
        ]),
        'G' => Some([
            0b01111, 0b10000, 0b10000, 0b10011, 0b10001, 0b10001, 0b01111,
        ]),
        'H' => Some([
            0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001,
        ]),
        'I' => Some([
            0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111,
        ]),
        'J' => Some([
            0b00111, 0b00010, 0b00010, 0b00010, 0b10010, 0b10010, 0b01100,
        ]),
        'K' => Some([
            0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001,
        ]),
        'L' => Some([
            0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111,
        ]),
        'M' => Some([
            0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001,
        ]),
        'N' => Some([
            0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001,
        ]),
        'O' => Some([
            0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
        ]),
        'Q' => Some([
            0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101,
        ]),
        'R' => Some([
            0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001,
        ]),
        'T' => Some([
            0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
        ]),
        'U' => Some([
            0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
        ]),
        'V' => Some([
            0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b01010, 0b00100,
        ]),
        'W' => Some([
            0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001,
        ]),
        'X' => Some([
            0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b01010, 0b10001,
        ]),
        'Y' => Some([
            0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
        ]),
        'Z' => Some([
            0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111,
        ]),
        ':' => Some([
            0b00000, 0b00100, 0b00100, 0b00000, 0b00100, 0b00100, 0b00000,
        ]),
        '.' => Some([
            0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100,
        ]),
        '/' => Some([
            0b00001, 0b00010, 0b00010, 0b00100, 0b01000, 0b01000, 0b10000,
        ]),
        '+' => Some([
            0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000,
        ]),
        '-' => Some([
            0b00000, 0b00000, 0b00000, 0b11110, 0b00000, 0b00000, 0b00000,
        ]),
        ' ' => Some([0; 7]),
        _ => None,
    }
}

fn push_overlay_rect(
    vertices: &mut Vec<OverlayVertex>,
    resolution: [f32; 2],
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    color: [f32; 4],
) {
    let x0 = x;
    let y0 = y;
    let x1 = x + width;
    let y1 = y + height;
    let p0 = pixel_to_ndc([x0, y0], resolution);
    let p1 = pixel_to_ndc([x1, y0], resolution);
    let p2 = pixel_to_ndc([x1, y1], resolution);
    let p3 = pixel_to_ndc([x0, y1], resolution);
    vertices.extend_from_slice(&[
        OverlayVertex {
            position: p0,
            color,
        },
        OverlayVertex {
            position: p1,
            color,
        },
        OverlayVertex {
            position: p2,
            color,
        },
        OverlayVertex {
            position: p0,
            color,
        },
        OverlayVertex {
            position: p2,
            color,
        },
        OverlayVertex {
            position: p3,
            color,
        },
    ]);
}

fn pixel_to_ndc(position: [f32; 2], resolution: [f32; 2]) -> [f32; 2] {
    [
        position[0] / resolution[0] * 2.0 - 1.0,
        1.0 - position[1] / resolution[1] * 2.0,
    ]
}

pub fn run_windowed<F>(
    world: WorldState,
    request: RenderRequest,
    config: WindowRenderConfig,
    update: F,
) -> Result<(), RenderError>
where
    F: FnMut(&mut WorldState, WindowFrameContext) -> Result<bool, RenderError> + 'static,
{
    run_windowed_with_overlay(
        world,
        request,
        config,
        WindowOverlayLines::default(),
        update,
    )
}

pub fn run_windowed_with_overlay<F>(
    world: WorldState,
    request: RenderRequest,
    config: WindowRenderConfig,
    overlay_lines: WindowOverlayLines,
    update: F,
) -> Result<(), RenderError>
where
    F: FnMut(&mut WorldState, WindowFrameContext) -> Result<bool, RenderError> + 'static,
{
    let event_loop = EventLoop::new()
        .map_err(|err| RenderError::new(format!("create window event loop: {err}")))?;
    let mut app = WindowRendererApp::new(world, request, config, overlay_lines, update);
    event_loop
        .run_app(&mut app)
        .map_err(|err| RenderError::new(format!("run window event loop: {err}")))?;
    if let Some(err) = app.last_error {
        return Err(err);
    }
    Ok(())
}

struct WindowRendererApp<F>
where
    F: FnMut(&mut WorldState, WindowFrameContext) -> Result<bool, RenderError>,
{
    world: WorldState,
    request: RenderRequest,
    config: WindowRenderConfig,
    update: F,
    window: Option<Arc<WinitWindow>>,
    surface: Option<wgpu::Surface<'static>>,
    surface_config: Option<wgpu::SurfaceConfiguration>,
    depth_view: Option<wgpu::TextureView>,
    renderer: Option<WgpuRenderer>,
    fps_overlay: Option<FpsOverlayRenderer>,
    text_overlay: Option<TextOverlayRenderer>,
    frame_index: u64,
    start: std::time::Instant,
    last_frame_instant: Option<std::time::Instant>,
    smoothed_fps: f32,
    overlay_lines: WindowOverlayLines,
    window_profile: Option<WindowProfile>,
    last_error: Option<RenderError>,
    input: WindowInputState,
    last_cursor_position_px: Option<[f32; 2]>,
}

impl<F> WindowRendererApp<F>
where
    F: FnMut(&mut WorldState, WindowFrameContext) -> Result<bool, RenderError>,
{
    fn new(
        world: WorldState,
        request: RenderRequest,
        config: WindowRenderConfig,
        overlay_lines: WindowOverlayLines,
        update: F,
    ) -> Self {
        Self {
            world,
            request,
            config,
            update,
            window: None,
            surface: None,
            surface_config: None,
            depth_view: None,
            renderer: None,
            fps_overlay: None,
            text_overlay: None,
            frame_index: 0,
            start: std::time::Instant::now(),
            last_frame_instant: None,
            smoothed_fps: 0.0,
            overlay_lines,
            window_profile: WindowProfile::from_environment(),
            last_error: None,
            input: WindowInputState::default(),
            last_cursor_position_px: None,
        }
    }

    fn initialize(&mut self, event_loop: &ActiveEventLoop) -> Result<(), RenderError> {
        if self.window.is_some() {
            return Ok(());
        }
        let resolution = self.config.resolution;
        let attributes = WindowAttributes::default()
            .with_title(self.config.title.clone())
            .with_inner_size(PhysicalSize::new(
                resolution[0].max(1),
                resolution[1].max(1),
            ));
        let window = Arc::new(
            event_loop
                .create_window(attributes)
                .map_err(|err| RenderError::new(format!("create window: {err}")))?,
        );
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
        let surface = instance
            .create_surface(window.clone())
            .map_err(|err| RenderError::new(format!("create WGPU surface: {err}")))?;
        let adapter_name_override = std::env::var_os("WGPU_ADAPTER_NAME");
        let power_preference_override = std::env::var_os("WGPU_POWER_PREF");
        let adapter = if adapter_name_override.is_some() || power_preference_override.is_some() {
            // WGPU's helper enumerates adapters when WGPU_ADAPTER_NAME is
            // supplied, so a desktop test can explicitly select the NVIDIA
            // adapter while still requiring compatibility with this surface.
            pollster::block_on(wgpu::util::initialize_adapter_from_env_or_default(
                &instance,
                Some(&surface),
            ))
        } else {
            // Keep the established production default. The WGPU helper's
            // fallback uses its own default power preference, which need not
            // be HighPerformance.
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            }))
        }
        .ok_or_else(|| RenderError::new("no WGPU adapter available for window surface"))?;
        let adapter_info = adapter.get_info();
        eprintln!(
            "PGE window WGPU adapter: name={} type={:?} backend={:?} driver={} ({})",
            adapter_info.name,
            adapter_info.device_type,
            adapter_info.backend,
            adapter_info.driver,
            adapter_info.driver_info,
        );
        if let Some(name) = adapter_name_override {
            eprintln!(
                "PGE window WGPU adapter selected with WGPU_ADAPTER_NAME={:?}",
                name
            );
        }
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|format| *format == wgpu::TextureFormat::Rgba8UnormSrgb)
            .or_else(|| {
                surface_caps
                    .formats
                    .iter()
                    .copied()
                    .find(|format| *format == wgpu::TextureFormat::Bgra8UnormSrgb)
            })
            .or_else(|| {
                surface_caps
                    .formats
                    .iter()
                    .copied()
                    .find(|format| format.is_srgb())
            })
            .unwrap_or(surface_caps.formats[0]);
        let present_mode = surface_caps
            .present_modes
            .iter()
            .copied()
            .find(|mode| *mode == wgpu::PresentMode::Fifo)
            .unwrap_or(surface_caps.present_modes[0]);
        let alpha_mode = surface_caps.alpha_modes[0];
        let (device, queue) =
            pollster::block_on(adapter.request_device(&default_device_descriptor(), None))
                .map_err(|err| RenderError::new(format!("create WGPU device: {err}")))?;
        let width = size.width.max(1);
        let height = size.height.max(1);
        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width,
            height,
            present_mode,
            alpha_mode,
            view_formats: vec![],
            desired_maximum_frame_latency: 1,
        };
        surface.configure(&device, &surface_config);
        let renderer = WgpuRenderer::from_device(device, queue, surface_format);
        let fps_overlay = FpsOverlayRenderer::new(renderer.device(), surface_format);
        let text_overlay = TextOverlayRenderer::new(renderer.device(), surface_format);
        let depth_view = create_depth_view(renderer.device(), [width, height]);
        self.request.resolution = [width, height];
        self.window = Some(window);
        self.surface = Some(surface);
        self.surface_config = Some(surface_config);
        self.depth_view = Some(depth_view);
        self.renderer = Some(renderer);
        self.fps_overlay = Some(fps_overlay);
        self.text_overlay = Some(text_overlay);
        Ok(())
    }

    fn resize(&mut self, size: PhysicalSize<u32>) {
        if size.width == 0 || size.height == 0 {
            return;
        }
        let (Some(surface), Some(surface_config), Some(renderer)) = (
            self.surface.as_ref(),
            self.surface_config.as_mut(),
            self.renderer.as_ref(),
        ) else {
            return;
        };
        surface_config.width = size.width.max(1);
        surface_config.height = size.height.max(1);
        surface.configure(renderer.device(), surface_config);
        self.depth_view = Some(create_depth_view(
            renderer.device(),
            [surface_config.width, surface_config.height],
        ));
        self.request.resolution = [surface_config.width, surface_config.height];
    }

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let redraw_start = std::time::Instant::now();
        let now = std::time::Instant::now();
        if let Some(previous) = self.last_frame_instant.replace(now) {
            let dt = now.duration_since(previous).as_secs_f32();
            if dt > 0.0 {
                let fps = 1.0 / dt;
                self.smoothed_fps = if self.smoothed_fps > 0.0 {
                    self.smoothed_fps * 0.9 + fps * 0.1
                } else {
                    fps
                };
            }
        }
        let context = WindowFrameContext {
            frame_index: self.frame_index,
            elapsed_sec: self.start.elapsed().as_secs_f64(),
            input: self.input,
        };
        let update_start = std::time::Instant::now();
        match (self.update)(&mut self.world, context) {
            Ok(true) => {}
            Ok(false) => {
                event_loop.exit();
                return;
            }
            Err(err) => {
                self.last_error = Some(err);
                event_loop.exit();
                return;
            }
        }
        let update_elapsed = update_start.elapsed();

        let (Some(surface), Some(config), Some(depth_view), Some(renderer)) = (
            self.surface.as_ref(),
            self.surface_config.as_ref(),
            self.depth_view.as_ref(),
            self.renderer.as_mut(),
        ) else {
            return;
        };
        let acquire_start = std::time::Instant::now();
        let output = match surface.get_current_texture() {
            Ok(output) => output,
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                surface.configure(renderer.device(), config);
                return;
            }
            Err(wgpu::SurfaceError::Timeout) => return,
            Err(err) => {
                self.last_error = Some(RenderError::new(format!("acquire surface texture: {err}")));
                event_loop.exit();
                return;
            }
        };
        let acquire_elapsed = acquire_start.elapsed();
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let scene_start = std::time::Instant::now();
        if let Err(err) = renderer.render_to_view(&self.world, &self.request, &view, depth_view) {
            self.last_error = Some(err);
            event_loop.exit();
            return;
        }
        let scene_elapsed = scene_start.elapsed();
        let fps_overlay_start = std::time::Instant::now();
        if let Some(fps_overlay) = self.fps_overlay.as_ref() {
            let additional_lines = self.overlay_lines.snapshot();
            fps_overlay.render(
                renderer.device(),
                renderer.queue(),
                &view,
                [config.width, config.height],
                self.smoothed_fps,
                &additional_lines,
            );
        }
        let fps_overlay_elapsed = fps_overlay_start.elapsed();
        let text_overlay_start = std::time::Instant::now();
        if let Some(text_overlay) = self.text_overlay.as_mut() {
            text_overlay.render(
                renderer.device(),
                renderer.queue(),
                &view,
                [config.width, config.height],
                &self.world.text_labels,
            );
        }
        let text_overlay_elapsed = text_overlay_start.elapsed();
        self.input.left_drag_delta_px = [0.0, 0.0];
        self.input.middle_drag_delta_px = [0.0, 0.0];
        self.input.right_drag_delta_px = [0.0, 0.0];
        self.input.scroll_delta_lines = 0.0;
        let present_start = std::time::Instant::now();
        output.present();
        let present_elapsed = present_start.elapsed();
        if let Some(profile) = self.window_profile.as_mut() {
            profile.record(
                update_elapsed,
                acquire_elapsed,
                scene_elapsed,
                fps_overlay_elapsed,
                text_overlay_elapsed,
                present_elapsed,
                redraw_start.elapsed(),
            );
        }
        self.frame_index += 1;
    }

    fn update_cursor_position(&mut self, x: f64, y: f64) {
        let position = [x as f32, y as f32];
        if let Some(previous) = self.last_cursor_position_px {
            let delta = [position[0] - previous[0], position[1] - previous[1]];
            if self.input.left_drag_active {
                self.input.left_drag_delta_px[0] += delta[0];
                self.input.left_drag_delta_px[1] += delta[1];
            }
            if self.input.middle_drag_active {
                self.input.middle_drag_delta_px[0] += delta[0];
                self.input.middle_drag_delta_px[1] += delta[1];
            }
            if self.input.right_drag_active {
                self.input.right_drag_delta_px[0] += delta[0];
                self.input.right_drag_delta_px[1] += delta[1];
            }
        }
        self.input.cursor_position_px = Some(position);
        self.last_cursor_position_px = Some(position);
    }

    fn update_mouse_button(&mut self, button: WinitMouseButton, state: ElementState) {
        let active = state == ElementState::Pressed;
        match button {
            WinitMouseButton::Left => {
                self.input.left_drag_active = active;
                if active {
                    self.input.left_drag_delta_px = [0.0, 0.0];
                }
            }
            WinitMouseButton::Middle => {
                self.input.middle_drag_active = active;
                if active {
                    self.input.middle_drag_delta_px = [0.0, 0.0];
                }
            }
            WinitMouseButton::Right => {
                self.input.right_drag_active = active;
                if active {
                    self.input.right_drag_delta_px = [0.0, 0.0];
                }
            }
            _ => {}
        }
    }

    fn update_mouse_wheel(&mut self, delta: MouseScrollDelta) {
        self.input.scroll_delta_lines += match delta {
            MouseScrollDelta::LineDelta(_, dy) => dy,
            MouseScrollDelta::PixelDelta(position) => position.y as f32 / 100.0,
        };
    }

    fn clear_pointer_state(&mut self) {
        self.input.left_drag_active = false;
        self.input.middle_drag_active = false;
        self.input.right_drag_active = false;
        self.last_cursor_position_px = None;
    }
}

impl<F> ApplicationHandler for WindowRendererApp<F>
where
    F: FnMut(&mut WorldState, WindowFrameContext) -> Result<bool, RenderError>,
{
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        event_loop.set_control_flow(ControlFlow::Poll);
        if let Err(err) = self.initialize(event_loop) {
            self.last_error = Some(err);
            event_loop.exit();
            return;
        }
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => self.resize(size),
            WindowEvent::RedrawRequested => self.redraw(event_loop),
            WindowEvent::CursorMoved { position, .. } => {
                self.update_cursor_position(position.x, position.y);
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                self.update_mouse_button(button, state);
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                self.update_mouse_wheel(delta);
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::CursorLeft { .. } | WindowEvent::Focused(false) => {
                self.clear_pointer_state();
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

fn default_device_descriptor() -> wgpu::DeviceDescriptor<'static> {
    wgpu::DeviceDescriptor {
        label: Some("pge app window renderer device"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
    }
}

fn create_depth_view(device: &wgpu::Device, resolution: [u32; 2]) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("pge app window depth texture"),
        size: wgpu::Extent3d {
            width: resolution[0].max(1),
            height: resolution[1].max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth24Plus,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}
