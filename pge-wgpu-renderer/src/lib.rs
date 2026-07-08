use std::collections::HashMap;
use std::num::NonZeroU64;
use std::path::{Path, PathBuf};
use std::time::Duration;

use bytemuck::{Pod, Zeroable};
use glam::{Mat3, Mat4, Vec3};
use pge_core::{
    ArenaId, Camera, CameraProjection, Geometry, Mesh, MeshSource, Node, NodeParent, Transform,
    WorldState,
};
use pge_renderer::{
    FrameBuffer, FrameKind, OffscreenRenderer, ProfiledRenderer, RenderError, RenderMetadata,
    RenderOutput, RenderPerformanceProfile, RenderRequest, RenderView, Renderer, RgbaFrame,
};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

impl Vertex {
    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct ObjectUniform {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

#[derive(Clone, Debug)]
struct MeshData {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    color: [f32; 4],
}

struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    color: [f32; 4],
}

struct RenderObject {
    mesh_id: ArenaId<Mesh>,
    transform: Mat4,
    color: [f32; 4],
}

pub type WgpuRgbaFrame = RgbaFrame;

struct DrawItem {
    mesh_key: String,
    mesh_index: usize,
    dynamic_offset: u32,
}

struct RenderTarget {
    color_texture: wgpu::Texture,
    color_view: wgpu::TextureView,
    _depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    readback_buffer: wgpu::Buffer,
    unpadded_bytes_per_row: u32,
    padded_bytes_per_row: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WgpuRenderTimings {
    pub total: Duration,
    pub camera: Duration,
    pub collect_objects: Duration,
    pub mesh_keys: Duration,
    pub ensure_gpu_meshes: Duration,
    pub object_uniforms: Duration,
    pub render_submit: Duration,
    pub object_count: usize,
    pub draw_item_count: usize,
}

impl From<WgpuRenderTimings> for RenderPerformanceProfile {
    fn from(timings: WgpuRenderTimings) -> Self {
        let mut profile = RenderPerformanceProfile::single_frame("wgpu-renderer");
        profile.add_timing("total", timings.total);
        profile.add_timing("camera", timings.camera);
        profile.add_timing("collectObjects", timings.collect_objects);
        profile.add_timing("meshKeys", timings.mesh_keys);
        profile.add_timing("ensureGpuMeshes", timings.ensure_gpu_meshes);
        profile.add_timing("objectUniforms", timings.object_uniforms);
        profile.add_timing("renderSubmit", timings.render_submit);
        profile.set_counter("objectCount", timings.object_count as u64);
        profile.set_counter("drawItemCount", timings.draw_item_count as u64);
        profile
    }
}

pub struct WgpuRenderer {
    pub(crate) device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    object_bind_group_layout: wgpu::BindGroupLayout,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    object_buffer: wgpu::Buffer,
    object_buffer_capacity: usize,
    object_bind_group: wgpu::BindGroup,
    mesh_cache: HashMap<String, Vec<MeshData>>,
    gpu_cache: HashMap<String, Vec<GpuMesh>>,
    render_targets: HashMap<[u32; 2], RenderTarget>,
}

impl WgpuRenderer {
    pub fn new() -> Result<Self, RenderError> {
        pollster::block_on(Self::new_async())
    }

    async fn new_async() -> Result<Self, RenderError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| RenderError::new("no WGPU adapter available"))?;
        let (device, queue) = adapter
            .request_device(&default_device_descriptor(), None)
            .await
            .map_err(|err| RenderError::new(format!("create WGPU device: {err}")))?;
        Ok(Self::from_device(
            device,
            queue,
            wgpu::TextureFormat::Rgba8UnormSrgb,
        ))
    }

    pub fn from_device(
        device: wgpu::Device,
        queue: wgpu::Queue,
        target_format: wgpu::TextureFormat,
    ) -> Self {
        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("pge camera bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let object_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("pge object bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pge wgpu pipeline layout"),
            bind_group_layouts: &[&camera_bind_group_layout, &object_bind_group_layout],
            push_constant_ranges: &[],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("pge wgpu shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("pge wgpu renderer pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::layout()],
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
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });
        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pge camera uniform"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("pge camera bind group"),
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });
        let object_buffer_capacity = 256_usize;
        let object_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pge object uniform buffer"),
            size: object_buffer_capacity as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let object_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("pge object bind group"),
            layout: &object_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &object_buffer,
                    offset: 0,
                    size: NonZeroU64::new(std::mem::size_of::<ObjectUniform>() as u64),
                }),
            }],
        });

        Self {
            device,
            queue,
            pipeline,
            object_bind_group_layout,
            camera_buffer,
            camera_bind_group,
            object_buffer,
            object_buffer_capacity,
            object_bind_group,
            mesh_cache: HashMap::new(),
            gpu_cache: HashMap::new(),
            render_targets: HashMap::new(),
        }
    }

    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    pub fn render_to_view(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
        color_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
    ) -> Result<(), RenderError> {
        let resolution = request.resolution;
        let (camera_node, camera) = select_camera(world, request)?;
        let camera_transform = world_transform(world, camera_node)?;
        let view_proj = camera_view_projection(camera, camera_transform, resolution)?;
        let camera_uniform = CameraUniform {
            view_proj: view_proj.to_cols_array_2d(),
            light_dir: [0.35, 0.45, -0.82, 0.0],
        };
        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));

        let render_objects = collect_render_objects(world)?;
        let mesh_keys: Vec<String> = render_objects
            .iter()
            .map(|object| mesh_key(world, object.mesh_id))
            .collect::<Result<_, _>>()?;
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            self.ensure_gpu_meshes(world, object.mesh_id, mesh_key)?;
        }
        let object_uniform_stride = 256_usize;
        let mut object_uniform_bytes = Vec::new();
        let mut draw_items = Vec::new();
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            if let Some(meshes) = self.gpu_cache.get(mesh_key) {
                for (mesh_index, mesh) in meshes.iter().enumerate() {
                    let dynamic_offset = object_uniform_bytes.len() as u32;
                    object_uniform_bytes
                        .resize(object_uniform_bytes.len() + object_uniform_stride, 0);
                    let uniform = ObjectUniform {
                        model: object.transform.to_cols_array_2d(),
                        color: multiply_color(object.color, mesh.color),
                    };
                    let uniform_bytes = bytemuck::bytes_of(&uniform);
                    object_uniform_bytes
                        [dynamic_offset as usize..dynamic_offset as usize + uniform_bytes.len()]
                        .copy_from_slice(uniform_bytes);
                    draw_items.push(DrawItem {
                        mesh_key: mesh_key.clone(),
                        mesh_index,
                        dynamic_offset,
                    });
                }
            }
        }
        if object_uniform_bytes.is_empty() {
            object_uniform_bytes.resize(object_uniform_stride, 0);
        }
        self.ensure_object_buffer(object_uniform_bytes.len());
        self.queue
            .write_buffer(&self.object_buffer, 0, &object_uniform_bytes);

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("pge surface render encoder"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pge surface render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.05,
                            g: 0.09,
                            b: 0.13,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.camera_bind_group, &[]);
            for item in &draw_items {
                if let Some(meshes) = self.gpu_cache.get(&item.mesh_key) {
                    if let Some(mesh) = meshes.get(item.mesh_index) {
                        pass.set_bind_group(1, &self.object_bind_group, &[item.dynamic_offset]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(
                            mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        pass.draw_indexed(0..mesh.index_count, 0, 0..1);
                    }
                }
            }
        }
        self.queue.submit(std::iter::once(encoder.finish()));
        Ok(())
    }

    pub fn render_rgba(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<WgpuRgbaFrame, RenderError> {
        let resolution = request.resolution;
        let (camera_node, camera) = select_camera(world, request)?;
        let camera_transform = world_transform(world, camera_node)?;
        let view_proj = camera_view_projection(camera, camera_transform, resolution)?;
        let camera_uniform = CameraUniform {
            view_proj: view_proj.to_cols_array_2d(),
            light_dir: [0.35, 0.45, -0.82, 0.0],
        };
        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));

        let render_objects = collect_render_objects(world)?;
        let mesh_keys: Vec<String> = render_objects
            .iter()
            .map(|object| mesh_key(world, object.mesh_id))
            .collect::<Result<_, _>>()?;
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            self.ensure_gpu_meshes(world, object.mesh_id, mesh_key)?;
        }
        let object_uniform_stride = 256_usize;
        let mut object_uniform_bytes = Vec::new();
        let mut draw_items = Vec::new();
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            if let Some(meshes) = self.gpu_cache.get(mesh_key) {
                for (mesh_index, mesh) in meshes.iter().enumerate() {
                    let dynamic_offset = object_uniform_bytes.len() as u32;
                    object_uniform_bytes
                        .resize(object_uniform_bytes.len() + object_uniform_stride, 0);
                    let uniform = ObjectUniform {
                        model: object.transform.to_cols_array_2d(),
                        color: multiply_color(object.color, mesh.color),
                    };
                    let uniform_bytes = bytemuck::bytes_of(&uniform);
                    object_uniform_bytes
                        [dynamic_offset as usize..dynamic_offset as usize + uniform_bytes.len()]
                        .copy_from_slice(uniform_bytes);
                    draw_items.push(DrawItem {
                        mesh_key: mesh_key.clone(),
                        mesh_index,
                        dynamic_offset,
                    });
                }
            }
        }
        if object_uniform_bytes.is_empty() {
            object_uniform_bytes.resize(object_uniform_stride, 0);
        }
        self.ensure_object_buffer(object_uniform_bytes.len());
        self.queue
            .write_buffer(&self.object_buffer, 0, &object_uniform_bytes);
        self.ensure_render_target(resolution);
        let target = self
            .render_targets
            .get(&resolution)
            .expect("render target exists");

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("pge render encoder"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pge render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.05,
                            g: 0.09,
                            b: 0.13,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.camera_bind_group, &[]);
            for item in &draw_items {
                if let Some(meshes) = self.gpu_cache.get(&item.mesh_key) {
                    if let Some(mesh) = meshes.get(item.mesh_index) {
                        pass.set_bind_group(1, &self.object_bind_group, &[item.dynamic_offset]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(
                            mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        pass.draw_indexed(0..mesh.index_count, 0, 0..1);
                    }
                }
            }
        }

        self.copy_target_to_readback(&mut encoder, target, resolution);
        self.queue.submit(std::iter::once(encoder.finish()));
        self.device.poll(wgpu::Maintain::Wait);
        let rgba = self.map_target_rgba(target, resolution)?;

        Ok(RgbaFrame {
            width: resolution[0],
            height: resolution[1],
            bytes: rgba,
        })
    }

    pub fn render_profile(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<WgpuRenderTimings, RenderError> {
        let total_start = std::time::Instant::now();
        let resolution = request.resolution;

        let camera_start = std::time::Instant::now();
        let (camera_node, camera) = select_camera(world, request)?;
        let camera_transform = world_transform(world, camera_node)?;
        let view_proj = camera_view_projection(camera, camera_transform, resolution)?;
        let camera_uniform = CameraUniform {
            view_proj: view_proj.to_cols_array_2d(),
            light_dir: [0.35, 0.45, -0.82, 0.0],
        };
        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));
        let camera_elapsed = camera_start.elapsed();

        let collect_start = std::time::Instant::now();
        let render_objects = collect_render_objects(world)?;
        let collect_elapsed = collect_start.elapsed();

        let mesh_keys_start = std::time::Instant::now();
        let mesh_keys: Vec<String> = render_objects
            .iter()
            .map(|object| mesh_key(world, object.mesh_id))
            .collect::<Result<_, _>>()?;
        let mesh_keys_elapsed = mesh_keys_start.elapsed();

        let ensure_start = std::time::Instant::now();
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            self.ensure_gpu_meshes(world, object.mesh_id, mesh_key)?;
        }
        let ensure_elapsed = ensure_start.elapsed();

        let uniforms_start = std::time::Instant::now();
        let object_uniform_stride = 256_usize;
        let mut object_uniform_bytes = Vec::new();
        let mut draw_items = Vec::new();
        for (object, mesh_key) in render_objects.iter().zip(mesh_keys.iter()) {
            if let Some(meshes) = self.gpu_cache.get(mesh_key) {
                for (mesh_index, mesh) in meshes.iter().enumerate() {
                    let dynamic_offset = object_uniform_bytes.len() as u32;
                    object_uniform_bytes
                        .resize(object_uniform_bytes.len() + object_uniform_stride, 0);
                    let uniform = ObjectUniform {
                        model: object.transform.to_cols_array_2d(),
                        color: multiply_color(object.color, mesh.color),
                    };
                    let uniform_bytes = bytemuck::bytes_of(&uniform);
                    object_uniform_bytes
                        [dynamic_offset as usize..dynamic_offset as usize + uniform_bytes.len()]
                        .copy_from_slice(uniform_bytes);
                    draw_items.push(DrawItem {
                        mesh_key: mesh_key.clone(),
                        mesh_index,
                        dynamic_offset,
                    });
                }
            }
        }
        if object_uniform_bytes.is_empty() {
            object_uniform_bytes.resize(object_uniform_stride, 0);
        }
        self.ensure_object_buffer(object_uniform_bytes.len());
        self.queue
            .write_buffer(&self.object_buffer, 0, &object_uniform_bytes);
        let uniforms_elapsed = uniforms_start.elapsed();

        let render_submit_start = std::time::Instant::now();
        self.ensure_render_target(resolution);
        let target = self
            .render_targets
            .get(&resolution)
            .expect("render target exists");
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("pge profile render encoder"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pge profile render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.05,
                            g: 0.09,
                            b: 0.13,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.camera_bind_group, &[]);
            for item in &draw_items {
                if let Some(meshes) = self.gpu_cache.get(&item.mesh_key) {
                    if let Some(mesh) = meshes.get(item.mesh_index) {
                        pass.set_bind_group(1, &self.object_bind_group, &[item.dynamic_offset]);
                        pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        pass.set_index_buffer(
                            mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        pass.draw_indexed(0..mesh.index_count, 0, 0..1);
                    }
                }
            }
        }
        self.queue.submit(std::iter::once(encoder.finish()));
        let render_submit_elapsed = render_submit_start.elapsed();

        Ok(WgpuRenderTimings {
            total: total_start.elapsed(),
            camera: camera_elapsed,
            collect_objects: collect_elapsed,
            mesh_keys: mesh_keys_elapsed,
            ensure_gpu_meshes: ensure_elapsed,
            object_uniforms: uniforms_elapsed,
            render_submit: render_submit_elapsed,
            object_count: render_objects.len(),
            draw_item_count: draw_items.len(),
        })
    }

    fn render_rgb(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<FrameBuffer, RenderError> {
        let rgba = self.render_rgba(world, request)?;
        Ok(FrameBuffer {
            kind: FrameKind::Rgb,
            width: rgba.width,
            height: rgba.height,
            bytes: encode_png_rgba([rgba.width, rgba.height], &rgba.bytes)?,
        })
    }

    fn copy_target_to_readback(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        target: &RenderTarget,
        resolution: [u32; 2],
    ) {
        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &target.color_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &target.readback_buffer,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(target.padded_bytes_per_row),
                    rows_per_image: Some(resolution[1]),
                },
            },
            wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: 1,
            },
        );
    }

    fn map_target_rgba(
        &self,
        target: &RenderTarget,
        resolution: [u32; 2],
    ) -> Result<Vec<u8>, RenderError> {
        let bytes_per_pixel = 4;
        let buffer_slice = target.readback_buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        self.device.poll(wgpu::Maintain::Wait);
        receiver
            .recv()
            .map_err(|err| RenderError::new(format!("readback channel failed: {err}")))?
            .map_err(|err| RenderError::new(format!("readback map failed: {err}")))?;

        let mapped = buffer_slice.get_mapped_range();
        let mut rgba = vec![0_u8; (resolution[0] * resolution[1] * bytes_per_pixel) as usize];
        for y in 0..resolution[1] as usize {
            let src = y * target.padded_bytes_per_row as usize;
            let dst = y * target.unpadded_bytes_per_row as usize;
            rgba[dst..dst + target.unpadded_bytes_per_row as usize]
                .copy_from_slice(&mapped[src..src + target.unpadded_bytes_per_row as usize]);
        }
        drop(mapped);
        target.readback_buffer.unmap();
        Ok(rgba)
    }

    fn ensure_object_buffer(&mut self, required_capacity: usize) {
        if required_capacity <= self.object_buffer_capacity {
            return;
        }
        let mut capacity = self.object_buffer_capacity.max(256);
        while capacity < required_capacity {
            capacity *= 2;
        }
        let object_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pge object uniform buffer"),
            size: capacity as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let object_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("pge object bind group"),
            layout: &self.object_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &object_buffer,
                    offset: 0,
                    size: NonZeroU64::new(std::mem::size_of::<ObjectUniform>() as u64),
                }),
            }],
        });
        self.object_buffer = object_buffer;
        self.object_bind_group = object_bind_group;
        self.object_buffer_capacity = capacity;
    }

    fn ensure_render_target(&mut self, resolution: [u32; 2]) {
        if self.render_targets.contains_key(&resolution) {
            return;
        }
        let bytes_per_pixel = 4;
        let unpadded_bytes_per_row = resolution[0] * bytes_per_pixel;
        let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let padded_bytes_per_row = unpadded_bytes_per_row.div_ceil(align) * align;
        let output_buffer_size = padded_bytes_per_row as u64 * resolution[1] as u64;
        let color_texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("pge render color texture"),
            size: wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("pge render depth texture"),
            size: wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let readback_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pge render readback buffer"),
            size: output_buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        self.render_targets.insert(
            resolution,
            RenderTarget {
                color_texture,
                color_view,
                _depth_texture: depth_texture,
                depth_view,
                readback_buffer,
                unpadded_bytes_per_row,
                padded_bytes_per_row,
            },
        );
    }

    fn ensure_gpu_meshes(
        &mut self,
        world: &WorldState,
        mesh_id: ArenaId<Mesh>,
        key: &str,
    ) -> Result<(), RenderError> {
        if self.gpu_cache.contains_key(key) {
            return Ok(());
        }
        let mesh_data = self.mesh_data(world, mesh_id, key)?.to_vec();
        let gpu_meshes = mesh_data
            .iter()
            .filter(|mesh| !mesh.vertices.is_empty() && !mesh.indices.is_empty())
            .map(|mesh| GpuMesh {
                vertex_buffer: self
                    .device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("pge mesh vertices"),
                        contents: bytemuck::cast_slice(&mesh.vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    }),
                index_buffer: self
                    .device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("pge mesh indices"),
                        contents: bytemuck::cast_slice(&mesh.indices),
                        usage: wgpu::BufferUsages::INDEX,
                    }),
                index_count: mesh.indices.len() as u32,
                color: mesh.color,
            })
            .collect();
        self.gpu_cache.insert(key.to_string(), gpu_meshes);
        Ok(())
    }

    fn mesh_data(
        &mut self,
        world: &WorldState,
        mesh_id: ArenaId<Mesh>,
        key: &str,
    ) -> Result<&[MeshData], RenderError> {
        if !self.mesh_cache.contains_key(key) {
            let mesh = world
                .meshes
                .get(&mesh_id)
                .ok_or_else(|| RenderError::new(format!("mesh {mesh_id} missing")))?;
            let fallback_color = mesh
                .material
                .and_then(|id| world.materials.get(&id))
                .map(|material| material.base_color_factor)
                .unwrap_or([0.7, 0.75, 0.8, 1.0]);
            let data = match &mesh.source {
                MeshSource::Procedural(geometry) => procedural_mesh(geometry, fallback_color),
                MeshSource::Asset { path, scale, .. } => {
                    load_gltf_meshes(path, *scale, fallback_color)?
                }
            };
            self.mesh_cache.insert(key.to_string(), data);
        }
        Ok(self.mesh_cache.get(key).expect("mesh cache populated"))
    }
}

impl OffscreenRenderer for WgpuRenderer {
    fn render_rgba(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<RgbaFrame, RenderError> {
        WgpuRenderer::render_rgba(self, world, request)
    }
}

impl ProfiledRenderer for WgpuRenderer {
    fn profile_render(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<RenderPerformanceProfile, RenderError> {
        self.render_profile(world, request).map(Into::into)
    }
}

impl Renderer for WgpuRenderer {
    fn render(
        &mut self,
        world: &WorldState,
        request: &RenderRequest,
    ) -> Result<RenderOutput, RenderError> {
        let mut frames = Vec::new();
        if request.views.contains(&RenderView::Rgb) {
            frames.push(self.render_rgb(world, request)?);
        }
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
            frames,
        })
    }
}

fn default_device_descriptor() -> wgpu::DeviceDescriptor<'static> {
    wgpu::DeviceDescriptor {
        label: Some("pge wgpu renderer device"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
    }
}

fn collect_render_objects(world: &WorldState) -> Result<Vec<RenderObject>, RenderError> {
    let mut objects = Vec::new();
    for (node_id, node) in world.nodes.iter() {
        let Some(mesh_id) = node.mesh else {
            continue;
        };
        let mesh = world
            .meshes
            .get(&mesh_id)
            .ok_or_else(|| RenderError::new(format!("mesh {mesh_id} missing")))?;
        let color = mesh
            .material
            .and_then(|id| world.materials.get(&id))
            .map(|material| material.base_color_factor)
            .unwrap_or([0.7, 0.75, 0.8, 1.0]);
        objects.push(RenderObject {
            mesh_id,
            transform: world_transform(world, node_id)?,
            color,
        });
    }
    Ok(objects)
}

fn select_camera<'a>(
    world: &'a WorldState,
    request: &RenderRequest,
) -> Result<(ArenaId<Node>, &'a Camera), RenderError> {
    for (node_id, node) in world.nodes.iter() {
        let Some(camera_id) = node.camera else {
            continue;
        };
        if let Some(requested) = &request.camera_id {
            if &node.entity != requested {
                continue;
            }
        }
        let camera = world
            .cameras
            .get(&camera_id)
            .ok_or_else(|| RenderError::new(format!("camera {camera_id} missing")))?;
        return Ok((node_id, camera));
    }
    Err(RenderError::new("no matching camera in world state"))
}

fn world_transform(world: &WorldState, node_id: ArenaId<Node>) -> Result<Mat4, RenderError> {
    let node = world
        .nodes
        .get(&node_id)
        .ok_or_else(|| RenderError::new(format!("node {node_id} missing")))?;
    let local = transform_matrix(node.transform);
    match node.parent {
        NodeParent::Node(parent) => Ok(world_transform(world, parent)? * local),
        NodeParent::Scene(_) | NodeParent::Orphan => Ok(local),
    }
}

fn transform_matrix(transform: Transform) -> Mat4 {
    let rotation = transform
        .rotation_matrix
        .map(mat3_from_cols_array)
        .unwrap_or_else(|| {
            Mat3::from_euler(
                glam::EulerRot::XYZ,
                transform.rotation[0],
                transform.rotation[1],
                transform.rotation[2],
            )
        });
    Mat4::from_translation(Vec3::from_array(transform.translation)) * Mat4::from_mat3(rotation)
}

fn mat3_from_cols_array(matrix: [[f32; 3]; 3]) -> Mat3 {
    Mat3::from_cols_array(&[
        matrix[0][0],
        matrix[1][0],
        matrix[2][0],
        matrix[0][1],
        matrix[1][1],
        matrix[2][1],
        matrix[0][2],
        matrix[1][2],
        matrix[2][2],
    ])
}

fn camera_view_projection(
    camera: &Camera,
    camera_transform: Mat4,
    resolution: [u32; 2],
) -> Result<Mat4, RenderError> {
    let eye = camera_transform.transform_point3(Vec3::ZERO);
    let forward = camera_transform
        .transform_vector3(Vec3::X)
        .normalize_or_zero();
    let up = camera_transform
        .transform_vector3(Vec3::Z)
        .normalize_or_zero();
    let aspect = resolution[0] as f32 / resolution[1].max(1) as f32;
    let projection = match camera.projection {
        CameraProjection::Perspective => {
            Mat4::perspective_rh(camera.fov_deg.to_radians(), aspect, 0.01, 100.0)
        }
        CameraProjection::Orthographic { size_m } => {
            let half_h = size_m * 0.5;
            let half_w = half_h * aspect;
            Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, 0.01, 100.0)
        }
    };
    if forward.length_squared() <= f32::EPSILON || up.length_squared() <= f32::EPSILON {
        return Err(RenderError::new("camera transform has invalid basis"));
    }
    Ok(projection * Mat4::look_to_rh(eye, forward, up))
}

fn mesh_key(world: &WorldState, mesh_id: ArenaId<Mesh>) -> Result<String, RenderError> {
    let mesh = world
        .meshes
        .get(&mesh_id)
        .ok_or_else(|| RenderError::new(format!("mesh {mesh_id} missing")))?;
    Ok(match &mesh.source {
        MeshSource::Procedural(geometry) => format!("procedural:{geometry:?}:{:?}", mesh.material),
        MeshSource::Asset { path, scale, .. } => format!("asset:{path}:{scale:?}"),
    })
}

fn multiply_color(left: [f32; 4], right: [f32; 4]) -> [f32; 4] {
    [
        left[0] * right[0],
        left[1] * right[1],
        left[2] * right[2],
        left[3] * right[3],
    ]
}

fn procedural_mesh(geometry: &Geometry, color: [f32; 4]) -> Vec<MeshData> {
    match geometry {
        Geometry::Box { size } => vec![box_mesh(*size, color)],
        Geometry::Sphere { radius } => vec![sphere_mesh(*radius, color)],
        Geometry::Cylinder { radius, height } => vec![cylinder_mesh(*radius, *height, color)],
    }
}

fn box_mesh(size: [f32; 3], color: [f32; 4]) -> MeshData {
    let half = [size[0] * 0.5, size[1] * 0.5, size[2] * 0.5];
    let positions = [
        [-half[0], -half[1], -half[2]],
        [half[0], -half[1], -half[2]],
        [half[0], half[1], -half[2]],
        [-half[0], half[1], -half[2]],
        [-half[0], -half[1], half[2]],
        [half[0], -half[1], half[2]],
        [half[0], half[1], half[2]],
        [-half[0], half[1], half[2]],
    ];
    let indices = vec![
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6,
        3, 0, 4, 3, 4, 7,
    ];
    let vertices = positions
        .iter()
        .map(|position| Vertex {
            position: *position,
            normal: Vec3::from_array(*position).normalize_or_zero().to_array(),
        })
        .collect();
    MeshData {
        vertices,
        indices,
        color,
    }
}

fn sphere_mesh(radius: f32, color: [f32; 4]) -> MeshData {
    let segments = 16;
    let rings = 8;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for ring in 0..=rings {
        let v = ring as f32 / rings as f32;
        let theta = v * std::f32::consts::PI;
        for segment in 0..=segments {
            let u = segment as f32 / segments as f32;
            let phi = u * std::f32::consts::TAU;
            let normal = Vec3::new(
                phi.cos() * theta.sin(),
                phi.sin() * theta.sin(),
                theta.cos(),
            );
            vertices.push(Vertex {
                position: (normal * radius).to_array(),
                normal: normal.to_array(),
            });
        }
    }
    for ring in 0..rings {
        for segment in 0..segments {
            let a = ring * (segments + 1) + segment;
            let b = a + segments + 1;
            indices.extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
        }
    }
    MeshData {
        vertices,
        indices: indices.into_iter().map(|index| index as u32).collect(),
        color,
    }
}

fn cylinder_mesh(radius: f32, height: f32, color: [f32; 4]) -> MeshData {
    let segments = 20;
    let half_h = height * 0.5;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for segment in 0..segments {
        let u = segment as f32 / segments as f32;
        let phi = u * std::f32::consts::TAU;
        let normal = Vec3::new(phi.cos(), phi.sin(), 0.0);
        vertices.push(Vertex {
            position: [normal.x * radius, normal.y * radius, -half_h],
            normal: normal.to_array(),
        });
        vertices.push(Vertex {
            position: [normal.x * radius, normal.y * radius, half_h],
            normal: normal.to_array(),
        });
    }
    for segment in 0..segments {
        let next = (segment + 1) % segments;
        let a = segment * 2;
        let b = next * 2;
        indices.extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
    }
    MeshData {
        vertices,
        indices: indices.into_iter().map(|index| index as u32).collect(),
        color,
    }
}

fn load_gltf_meshes(
    path: &str,
    scale: [f32; 3],
    fallback_color: [f32; 4],
) -> Result<Vec<MeshData>, RenderError> {
    let path = PathBuf::from(path);
    let (document, buffers, _) = gltf::import(&path)
        .map_err(|err| RenderError::new(format!("load GLTF {}: {err}", path.display())))?;
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    let mut meshes = Vec::new();
    for gltf_mesh in document.meshes() {
        for primitive in gltf_mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()].0));
            let Some(positions) = reader.read_positions() else {
                continue;
            };
            let positions: Vec<_> = positions
                .map(|position| {
                    [
                        position[0] * scale[0],
                        position[1] * scale[1],
                        position[2] * scale[2],
                    ]
                })
                .collect();
            let normals: Vec<_> = reader
                .read_normals()
                .map(|normals| normals.collect())
                .unwrap_or_else(|| vec![[0.0, 0.0, 1.0]; positions.len()]);
            let indices: Vec<u32> = reader
                .read_indices()
                .map(|indices| indices.into_u32().collect())
                .unwrap_or_else(|| (0..positions.len() as u32).collect());
            let pbr = primitive.material().pbr_metallic_roughness();
            let base = pbr.base_color_factor();
            let color = if primitive.material().index().is_some() {
                [base[0], base[1], base[2], base[3]]
            } else {
                fallback_color
            };
            let vertices = positions
                .into_iter()
                .zip(normals)
                .map(|(position, normal)| Vertex { position, normal })
                .collect();
            meshes.push(MeshData {
                vertices,
                indices,
                color,
            });
        }
    }
    if meshes.is_empty() {
        Err(RenderError::new(format!(
            "GLTF {} has no renderable triangle meshes",
            base_dir.display()
        )))
    } else {
        Ok(meshes)
    }
}

fn encode_png_rgba(resolution: [u32; 2], rgba: &[u8]) -> Result<Vec<u8>, RenderError> {
    let mut bytes = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut bytes, resolution[0], resolution[1]);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder
            .write_header()
            .map_err(|err| RenderError::new(format!("write PNG header: {err}")))?;
        writer
            .write_image_data(rgba)
            .map_err(|err| RenderError::new(format!("write PNG data: {err}")))?;
    }
    Ok(bytes)
}
