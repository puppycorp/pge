use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::process;
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;
use std::env;
use futures::executor::block_on;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalPosition;
use winit::event::MouseScrollDelta;
use winit::event::WindowEvent;
use winit::event_loop::ControlFlow;
use winit::event_loop::EventLoop;
use winit::keyboard::KeyCode;

use crate::engine::Engine;
use crate::hardware::BufferHandle;
use crate::hardware::Hardware;
use crate::hardware::PipelineHandle;
use crate::hardware::RenderEncoder;
use crate::hardware::TextureHandle;
use crate::hardware::WindowHandle;
use crate::mock_hardware::MockHardware;
use crate::KeyAction;
use crate::MouseEvent;
use super::wgpu_types::*;
use crate::App;
use crate::KeyboardKey;
use crate::MouseButton;
use crate::Window;

struct Size {
	width: u32,
	height: u32,
}

struct CreateWindow {
	window_id: u32,
	name: String,
	size: Option<Size>,
	fullscreen: bool,
	lock_cursor: bool,
}

enum UserEvent{
	CreateWindow(CreateWindow),
	DestroyWindow {
		window_id: u32,
	},
	CreatePipeline {
		window: WindowHandle,
		name: String,
		pipeline_id: u32
	},
	CreateBuffer {
		buffer_id: u32,
		size: u64,
		name: String,
	},
	DestroyBuffer {
		buffer_id: u32,
	},
	CreateTexture {
		texture_id: u32,
		name: String,
		data: Vec<u8>,
		width: u32,
		height: u32,
	},
	WriteBuffer {
		buffer: BufferHandle,
		data: Vec<u8>,
	},
	Render {
		window: WindowHandle,
		encoder: RenderEncoder,
	},
	SaveScreenshot {
		window: WindowHandle,
		path: String,
	},
}

struct WindowContext<'a> {
	winit_id: winit::window::WindowId,
	wininit_window: Arc<winit::window::Window>,
	window_id: u32,
	surface: Arc<wgpu::Surface<'a>>,
	lock_cursor: bool,
	last_cursor_pos: Option<PhysicalPosition<f64>>,
	surface_format: Option<wgpu::TextureFormat>,
}

struct PipelineContext {
	id: u32,
	pipeline: Arc<wgpu::RenderPipeline>,
	depth_texture_view: Option<Arc<wgpu::TextureView>>,
	uses_depth: bool,
}

struct PgeWininitHandler<'a, A, H> {
	engine: Engine<A, H>,
	start_time: Instant,
	last_on_process_time: Instant,
	max_iterations: Option<u64>,
	iterations: u64,
	progress_interval: u64,
	windows: Vec<WindowContext<'a>>,
	adapter: Arc<wgpu::Adapter>,
	device: Arc<wgpu::Device>,
	queue: Arc<wgpu::Queue>,
	instance: Arc<wgpu::Instance>,
	pipelines: Vec<PipelineContext>,
	buffers: Vec<BufferContext>,
	textures: Vec<TextureContext>,
	pending_screenshots: HashMap<u32, String>,
	screenshot_dir: Option<PathBuf>,
	screenshot_counter: u64,
	screenshot_interval: u64,
}

struct HeadlessWindowContext {
	window_id: u32,
	size: wgpu::Extent3d,
	color_texture: wgpu::Texture,
	color_view: wgpu::TextureView,
}

struct HeadlessWgpuHardware {
	device: Arc<wgpu::Device>,
	queue: Arc<wgpu::Queue>,
	pipeline_id: u32,
	buffer_id: u32,
	texture_id: u32,
	window_id: u32,
	windows: Vec<HeadlessWindowContext>,
	pipelines: Vec<PipelineContext>,
	buffers: Vec<BufferContext>,
	textures: Vec<TextureContext>,
	pending_screenshots: HashMap<u32, String>,
	screenshot_dir: Option<PathBuf>,
	screenshot_counter: u64,
	screenshot_interval: u64,
}

impl<'a, A, H> PgeWininitHandler<'a, A, H> {
	fn new(engine: Engine<A, H>, adapter: Arc<wgpu::Adapter>, device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>, instance: Arc<wgpu::Instance>, max_iterations: Option<u64>) -> Self {
		let progress_interval = max_iterations
			.map(iteration_log_interval)
			.unwrap_or(0);
		let screenshot_dir = screenshot_dir_from_env();
		let screenshot_interval = screenshot_interval_from_env();
		Self {
			engine,
			start_time: Instant::now(),
			last_on_process_time: Instant::now(),
			max_iterations,
			iterations: 0,
			progress_interval,
			windows: Vec::new(),
			adapter,
			device,
			queue,
			instance,
			pipelines: Vec::new(),
			buffers: Vec::new(),
			textures: Vec::new(),
			pending_screenshots: HashMap::new(),
			screenshot_dir,
			screenshot_counter: 0,
			screenshot_interval,
		}
	}

	fn log_exit_stats(&self) {
		let runtime = self.start_time.elapsed();
		let runtime_secs = runtime.as_secs_f64();
		let avg_fps = if runtime_secs > 0.0 {
			self.iterations as f64 / runtime_secs
		} else {
			0.0
		};
		crate::log1!(
			"Iterations: {} Average FPS: {:.2}, runtime: {:.2}s",
			self.iterations,
			avg_fps,
			runtime_secs
		);
	}
}

impl<'a, A, H> ApplicationHandler<UserEvent> for PgeWininitHandler<'a, A, H> 
where
	A: App,
	H: Hardware,
{
	fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
		event_loop.set_control_flow(ControlFlow::Poll);
	}

	fn user_event(&mut self, event_loop: &winit::event_loop::ActiveEventLoop, event: UserEvent) {
		match event {
			UserEvent::CreateWindow(args) => {
				let mut window_attributes = winit::window::Window::default_attributes().with_title(&args.name);
				if screenshot_enabled() {
					window_attributes = window_attributes.with_visible(false);
				}
				if args.fullscreen {
					window_attributes.fullscreen = Some(winit::window::Fullscreen::Borderless(None));
				}
				if let Some(size) = args.size {
					window_attributes.inner_size = Some(winit::dpi::Size::Physical(winit::dpi::PhysicalSize::new(size.width, size.height)));
				}
				let wininit_window = event_loop.create_window(window_attributes).unwrap();
				let wininit_window = Arc::new(wininit_window);
				let surface = Arc::new(self.instance.create_surface(wininit_window.clone()).unwrap());
				let wininit_window_id = wininit_window.id();
				let window_ctx = WindowContext {
					winit_id: wininit_window_id,
					surface,
					window_id: args.window_id,
					wininit_window,
					lock_cursor: args.lock_cursor,
					last_cursor_pos: None,
					surface_format: None,
				};
				self.windows.push(window_ctx);
			}
			UserEvent::DestroyWindow {
				window_id,
			} => {

			}
			UserEvent::CreatePipeline {
				window,
				name,
				pipeline_id,
			} => {
				let window_ctx = match self.windows.iter_mut().find(|w| w.window_id == window.id) {
					Some(window) => window,
					None => {
						log::error!("Window not found: {:?}", window);
						return;
					}
				};
				let surface_caps = window_ctx.surface.get_capabilities(&self.adapter);
				let surface_format = surface_caps
					.formats
					.iter()
					.copied()
					.find(|f| f.is_srgb())
					.unwrap_or(surface_caps.formats[0]);
				window_ctx.surface_format = Some(surface_format);

				let size = window_ctx.wininit_window.inner_size();
		
				let config = wgpu::SurfaceConfiguration {
					usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
					format: surface_format,
					width: size.width,
					height: size.height,
					present_mode: surface_caps.present_modes[0],
					alpha_mode: surface_caps.alpha_modes[0],
					view_formats: vec![],
					desired_maximum_frame_latency: 1,
				};
		
				window_ctx.surface.configure(&self.device, &config);
				let camera_bind_group_layout = RawCamera::create_bind_group_layout(&self.device);
				let point_light_bind_group_layout = RawPointLight::create_bind_group_layout(&self.device);
				let base_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
				let metallic_roughness_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
				let normal_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
				let occlusion_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
				let emissive_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
				let material_bind_group_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
					label: Some("Material Bind Group Layout"),
					entries: &[wgpu::BindGroupLayoutEntry {
						binding: 0,
						visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
						ty: wgpu::BindingType::Buffer {
							ty: wgpu::BufferBindingType::Storage { read_only: true },
							has_dynamic_offset: false,
							min_binding_size: None,
						},
						count: None,
					}],
				});
			
				let (render_pipeline, depth_texture_view, uses_depth) = if name == "gui" {
					let shader_source =
						wgpu::ShaderSource::Wgsl(include_str!("../shaders/gui_shader.wgsl").into());
					let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
						label: Some("Gui Shader"),
						source: shader_source,
					});

					let render_pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
						label: Some("Gui Pipeline Layout"),
						bind_group_layouts: &[],
						push_constant_ranges: &[],
					});

					let position_layout = wgpu::VertexBufferLayout {
						array_stride: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
						step_mode: wgpu::VertexStepMode::Vertex,
						attributes: &[wgpu::VertexAttribute {
							offset: 0,
							format: wgpu::VertexFormat::Float32x3,
							shader_location: 0,
						}],
					};
					let color_layout = wgpu::VertexBufferLayout {
						array_stride: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
						step_mode: wgpu::VertexStepMode::Vertex,
						attributes: &[wgpu::VertexAttribute {
							offset: 0,
							format: wgpu::VertexFormat::Float32x3,
							shader_location: 1,
						}],
					};

					let render_pipeline = self.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
						label: Some("Gui Pipeline"),
						layout: Some(&render_pipeline_layout),
						vertex: wgpu::VertexState {
							module: &shader,
							entry_point: "vs_main",
							buffers: &[position_layout, color_layout],
							compilation_options: Default::default(),
						},
						fragment: Some(wgpu::FragmentState {
							module: &shader,
							entry_point: "fs_main",
							targets: &[Some(wgpu::ColorTargetState {
								format: wgpu::TextureFormat::Bgra8UnormSrgb,
								blend: Some(wgpu::BlendState {
									color: wgpu::BlendComponent::REPLACE,
									alpha: wgpu::BlendComponent::REPLACE,
								}),
								write_mask: wgpu::ColorWrites::ALL,
							})],
							compilation_options: Default::default(),
						}),
						primitive: wgpu::PrimitiveState {
							topology: wgpu::PrimitiveTopology::TriangleList,
							strip_index_format: None,
							front_face: wgpu::FrontFace::Ccw,
							cull_mode: None,
							polygon_mode: wgpu::PolygonMode::Fill,
							unclipped_depth: false,
							conservative: false,
						},
						depth_stencil: None,
						multisample: wgpu::MultisampleState {
							count: 1,
							mask: !0,
							alpha_to_coverage_enabled: false,
						},
						multiview: None,
					});

					(render_pipeline, None, false)
				} else {
					let tex_coords_layout = wgpu::VertexBufferLayout {
						array_stride: std::mem::size_of::<TexCoords>() as wgpu::BufferAddress,
						step_mode: wgpu::VertexStepMode::Vertex,
						attributes: &[wgpu::VertexAttribute {
							offset: 0,
							format: wgpu::VertexFormat::Float32x2,
							shader_location: 2,
						}],
					};
				
					let layouts = &[
						&camera_bind_group_layout, 
						&point_light_bind_group_layout, 
						&base_texture_bind_group_layout,
						&metallic_roughness_texture_bind_group_layout,
						&normal_texture_bind_group_layout,
						&occlusion_texture_bind_group_layout,
						&emissive_texture_bind_group_layout,
						&material_bind_group_layout,
					];
					let buffers = &[Vertices::desc(), RawInstance::desc(), Normals::desc(), tex_coords_layout];
					let shader_source = wgpu::ShaderSource::Wgsl(include_str!("../shaders/3d_shader.wgsl").into());
				
					let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
						label: Some("Shader"),
						source: shader_source
					});
				
					let render_pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
						label: Some("Render Pipeline Layout"),
						bind_group_layouts: layouts,
						push_constant_ranges: &[],
					});
				
					let depth_stencil_state = wgpu::DepthStencilState {
						format: wgpu::TextureFormat::Depth24PlusStencil8,
						depth_write_enabled: true,
						depth_compare: wgpu::CompareFunction::Less,
						stencil: wgpu::StencilState::default(),
						bias: wgpu::DepthBiasState::default(),
					};
				
					let render_pipeline = self.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
						label: Some("Render Pipeline"),
						layout: Some(&render_pipeline_layout),
						vertex: wgpu::VertexState {
							module: &shader,
							entry_point: "vs_main",
							buffers,
							compilation_options: Default::default(),
						},
						fragment: Some(wgpu::FragmentState {
							module: &shader,
							entry_point: "fs_main",
							targets: &[Some(wgpu::ColorTargetState {
								format: wgpu::TextureFormat::Bgra8UnormSrgb,
								blend: Some(wgpu::BlendState {
									color: wgpu::BlendComponent::REPLACE,
									alpha: wgpu::BlendComponent::REPLACE,
								}),
								write_mask: wgpu::ColorWrites::ALL,
							})],
							compilation_options: Default::default(),
						}),
						primitive: wgpu::PrimitiveState {
							topology: wgpu::PrimitiveTopology::TriangleList,
							strip_index_format: None,
							front_face: wgpu::FrontFace::Ccw,
							cull_mode: None,
							polygon_mode: wgpu::PolygonMode::Fill,
							unclipped_depth: false,
							conservative: false,
						},
						depth_stencil: Some(depth_stencil_state),
						multisample: wgpu::MultisampleState {
							count: 1,
							mask: !0,
							alpha_to_coverage_enabled: false,
						},
						multiview: None,
					});

					let depth_texture = self.device.create_texture(&wgpu::TextureDescriptor {
						label: Some("Depth Texture"),
						size: wgpu::Extent3d {
							width: size.width,
							height: size.height,
							depth_or_array_layers: 1,
						},
						mip_level_count: 1,
						sample_count: 1,
						dimension: wgpu::TextureDimension::D2,
						format: wgpu::TextureFormat::Depth24PlusStencil8,
						usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
						view_formats: Default::default(),
					});
					let depth_texture_view =
						Arc::new(depth_texture.create_view(&wgpu::TextureViewDescriptor::default()));

					(render_pipeline, Some(depth_texture_view), true)
				};

				let pipeline_ctx = PipelineContext {
					id: pipeline_id,
					pipeline: Arc::new(render_pipeline),
					depth_texture_view,
					uses_depth,
				};
				self.pipelines.push(pipeline_ctx);
			}
			UserEvent::Render {
				window,
				encoder,
			} => {
				let mut screenshot_path = self.pending_screenshots.remove(&window.id);
				let frame_index = self.screenshot_counter;
				self.screenshot_counter += 1;
				if screenshot_path.is_none() {
					if let Some(dir) = &self.screenshot_dir {
						if self.screenshot_interval > 0 && frame_index % self.screenshot_interval == 0 {
							let filename = format!("screenshot_{}_{}.png", window.id, frame_index);
							let path = dir.join(filename);
							screenshot_path = Some(path.to_string_lossy().to_string());
						}
					}
				}
				let window_ctx = match self.windows.iter().find(|window_ctx| window_ctx.window_id == window.id) {
					Some(window) => window,
					None => {
						log::error!("Window not found: {:?} => RETURN", window);
						return;
					}
				};
				let output = window_ctx.surface.get_current_texture().unwrap();
				let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());
				let mut wgpu_encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
					label: Some("Render Encoder"),
				});
				let mut screenshot_buffer = None;
				let mut screenshot_info = None;
				for (pass_index, pass) in encoder.passes.into_iter().enumerate() {
					let pipeline = pass.pipeline.unwrap();
					let pipeline_ctx = match self.pipelines.iter().find(|pipeline_ctx| pipeline_ctx.id == pipeline.id) {
						Some(pipeline_ctx) => pipeline_ctx,
						None => {
							log::error!("Pipeline not found: {:?} => RETURN", pipeline);
							return;
						}
					};
					let load_op = if pass_index == 0 {
						wgpu::LoadOp::Clear(wgpu::Color {
							r: 0.1,
							g: 0.2,
							b: 0.3,
							a: 1.0,
						})
					} else {
						wgpu::LoadOp::Load
					};
					let depth_stencil_attachment = if pipeline_ctx.uses_depth {
						Some(wgpu::RenderPassDepthStencilAttachment {
							view: pipeline_ctx
								.depth_texture_view
								.as_ref()
								.expect("missing depth texture view"),
							depth_ops: Some(wgpu::Operations {
								load: if pass_index == 0 {
									wgpu::LoadOp::Clear(1.0)
								} else {
									wgpu::LoadOp::Load
								},
								store: wgpu::StoreOp::Store,
							}),
							stencil_ops: None,
						})
					} else {
						None
					};
					let mut wgpu_pass = wgpu_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
						label: Some("Render Pass"),
						color_attachments: &[Some(wgpu::RenderPassColorAttachment {
							view: &view,
							resolve_target: None,
							ops: wgpu::Operations {
								load: load_op,
								store: wgpu::StoreOp::Store,
							},
						})],
						depth_stencil_attachment,
						..Default::default()
					});
		
					wgpu_pass.set_pipeline(&pipeline_ctx.pipeline);
					for subpass in &pass.subpasses {
						for (slot, texture) in &subpass.textures {
							let texture_ctx = match self.textures.iter().find(|t| t.id == texture.id) {
								Some(texture) => texture, 
								None => {
									log::error!("Texture not found: {:?} => RETURN", texture);
									return;
								}
							};
							wgpu_pass.set_bind_group(*slot, &texture_ctx.bind_group, &[]);
						}
						for (slot, buffer) in &subpass.buffers {
							let buffer_ctx = match self.buffers.iter().find(|b| b.id == buffer.id) {
								Some(buffer) => buffer,
								None => {
									log::error!("Buffer not found: {:?} => RETURN", buffer);
									return;
								}
							};
							if !buffer_ctx.written {
								log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", buffer);
								return;
							}
							wgpu_pass.set_bind_group(*slot, &buffer_ctx.bind_group, &[]);
						}
						for (slot, buffer) in &subpass.vertex_buffers {
							let buffer_ctx = match self.buffers.iter().find(|b| b.id == buffer.handle.id) {
								Some(buffer) => buffer,
								None => {
									log::error!("Buffer not found: {:?} => RETURN", buffer);
									return;
								}
							};
							if !buffer_ctx.written {
								log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", buffer);
								return;
							}
							if buffer.range.start == buffer.range.end {
								log::error!("BUFFER RANGE IS ZERO: {:?} => RETURN", buffer);
								continue;
							}
							wgpu_pass.set_vertex_buffer(*slot, buffer_ctx.buffer.slice(buffer.range.clone()));
						}
						if let Some(slice) = &subpass.index_buffer {
							let buffer_ctx = match self.buffers.iter().find(|b| b.id == slice.handle.id) {
								Some(buffer) => buffer,
								None => {
									log::error!("Buffer not found: {:?} => RETURN", slice.handle);
									return;
								}
							};
							if !buffer_ctx.written {
								log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", slice.handle);
								return;
							}
							if slice.range.start == slice.range.end {
								log::error!("BUFFER RANGE IS ZERO: {:?} => RETURN", slice.handle);
								continue;
							}
							wgpu_pass.set_index_buffer(buffer_ctx.buffer.slice(slice.range.clone()), wgpu::IndexFormat::Uint16);
						}
						let indices = subpass.indices.clone().unwrap();
						let instances = subpass.instances.clone().unwrap();
						wgpu_pass.draw_indexed(indices.clone(), 0, instances.clone());
					}
				}
				if screenshot_path.is_some() {
					let size = window_ctx.wininit_window.inner_size();
					let bytes_per_pixel = 4;
					let unpadded_bytes_per_row = size.width * bytes_per_pixel;
					let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
					let padded_bytes_per_row_padding = (align - unpadded_bytes_per_row % align) % align;
					let padded_bytes_per_row = unpadded_bytes_per_row + padded_bytes_per_row_padding;
					let output_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
						label: Some("Screenshot Buffer"),
						size: padded_bytes_per_row as u64 * size.height as u64,
						usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
						mapped_at_creation: false,
					});
					let texture_size = wgpu::Extent3d {
						width: size.width,
						height: size.height,
						depth_or_array_layers: 1,
					};
					wgpu_encoder.copy_texture_to_buffer(
						wgpu::ImageCopyTexture {
							texture: &output.texture,
							mip_level: 0,
							origin: wgpu::Origin3d::ZERO,
							aspect: wgpu::TextureAspect::All,
						},
						wgpu::ImageCopyBuffer {
							buffer: &output_buffer,
							layout: wgpu::ImageDataLayout {
								offset: 0,
								bytes_per_row: Some(padded_bytes_per_row),
								rows_per_image: Some(size.height),
							},
						},
						texture_size,
					);
					screenshot_buffer = Some(output_buffer);
					screenshot_info = Some((size.width, size.height, unpadded_bytes_per_row, padded_bytes_per_row));
				}
				self.queue.submit(std::iter::once(wgpu_encoder.finish()));
				output.present();
				if let (Some(path), Some(buffer), Some((width, height, unpadded_bytes_per_row, padded_bytes_per_row))) =
					(screenshot_path, screenshot_buffer, screenshot_info)
				{
					let buffer_slice = buffer.slice(..);
					let (tx, rx) = std::sync::mpsc::channel();
					buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
						let _ = tx.send(result);
					});
					self.device.poll(wgpu::Maintain::Wait);
					match rx.recv() {
						Ok(Ok(())) => {}
						Ok(Err(err)) => {
							log::error!("Failed to map screenshot buffer: {:?}", err);
							buffer.unmap();
							return;
						}
						Err(err) => {
							log::error!("Failed to receive screenshot buffer map result: {:?}", err);
							buffer.unmap();
							return;
						}
					}
					let data = buffer_slice.get_mapped_range();
					let mut pixels: Vec<u8> = Vec::with_capacity((width * height * 4) as usize);
					for chunk in data.chunks(padded_bytes_per_row as usize) {
						pixels.extend_from_slice(&chunk[..unpadded_bytes_per_row as usize]);
					}
					drop(data);
					buffer.unmap();
					if matches!(
						window_ctx.surface_format,
						Some(wgpu::TextureFormat::Bgra8Unorm) | Some(wgpu::TextureFormat::Bgra8UnormSrgb)
					) {
						for chunk in pixels.chunks_exact_mut(4) {
							chunk.swap(0, 2);
						}
					}
					match image::save_buffer(
						&path,
						&pixels,
						width,
						height,
						image::ColorType::Rgba8,
					) {
						Ok(_) => crate::log1!("Screenshot saved to {}", path),
						Err(err) => log::error!("Failed to save screenshot: {:?}", err),
					}
				}
			},
			UserEvent::CreateBuffer {
				buffer_id,
				name,
				size,
			} => {
				crate::log2!("new buffer id: {:?} name: {:?} size: {:?}", buffer_id, name, size);
				let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
					label: Some(&name),
					size,
					usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::INDEX | wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::STORAGE,
					mapped_at_creation: false,
				});
				let layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
					label: None,
					entries: &[wgpu::BindGroupLayoutEntry {
						binding: 0,
						visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
						ty: wgpu::BindingType::Buffer {
							ty: wgpu::BufferBindingType::Storage { read_only: true },
							has_dynamic_offset: false,
							min_binding_size: None,
						},
						count: None,
					}],
				});
				let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
					layout: &layout,
					entries: &[wgpu::BindGroupEntry {
						binding: 0,
						resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
							buffer: &buffer,
							offset: 0,
							size: None,
						}),
					}],
					label: Some("Buffer Bind Group"),
				});
		
				self.buffers.push(BufferContext {
					id: buffer_id,
					name,
					buffer,
					bind_group,
					written: false,
				});
			}
			UserEvent::DestroyBuffer {
				buffer_id,
			} => {
				let buffer_ctx = match self.buffers.iter_mut().find(|b| b.id == buffer_id) {
					Some(b) => b,
					None => {
						log::error!("Buffer not found: {:?}", buffer_id);
						return;
					}
				};
				buffer_ctx.buffer.destroy();
				self.buffers.retain(|b| b.id != buffer_id);
			}
			UserEvent::CreateTexture {
				texture_id,
				name,
				data,
				width,
				height,
			} => {
				let size = wgpu::Extent3d {
					width,
					height,
					depth_or_array_layers: 1,
				};
				let texture = self.device.create_texture(&wgpu::TextureDescriptor {
					label: Some(&name),
					size,
					mip_level_count: 1,
					sample_count: 1,
					dimension: wgpu::TextureDimension::D2,
					format: wgpu::TextureFormat::Rgba8Unorm,
					usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
					view_formats: Default::default(),
				});
				self.queue.write_texture(
					wgpu::ImageCopyTexture {
						texture: &texture,
						mip_level: 0,
						origin: wgpu::Origin3d::ZERO,
						aspect: wgpu::TextureAspect::All,
					},
					&data,
					wgpu::ImageDataLayout {
						offset: 0,
						bytes_per_row: Some(4 * width),
						rows_per_image: Some(height),
					},
					size
				);
			
				let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());
				let sampler = self.device.create_sampler(&wgpu::SamplerDescriptor {
					address_mode_u: wgpu::AddressMode::Repeat,
					address_mode_v: wgpu::AddressMode::Repeat,
					address_mode_w: wgpu::AddressMode::Repeat,
					mag_filter: wgpu::FilterMode::Linear,
					min_filter: wgpu::FilterMode::Linear,
					mipmap_filter: wgpu::FilterMode::Nearest,
					..Default::default()
				});
			
				let texture_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
					layout: &TextureBuffer::create_bind_group_layout(&self.device),
					entries: &[
						wgpu::BindGroupEntry {
							binding: 0,
							resource: wgpu::BindingResource::TextureView(&texture_view),
						},
						wgpu::BindGroupEntry {
							binding: 1,
							resource: wgpu::BindingResource::Sampler(&sampler),
						},
					],
					label: Some("texture_bind_group"),
				});
				self.textures.push(TextureContext {
					id: texture_id,
					texture,
					bind_group: texture_bind_group,
				});
			}
			UserEvent::WriteBuffer {
				buffer,
				data,
			} => {
				let buffer_ctx = match self.buffers.iter_mut().find(|b| b.id == buffer.id) {
					Some(b) => b,
					None => {
						log::error!("Buffer not found: {:?}", buffer);
						return;
					}
				};
				if data.len() == 0 {
					buffer_ctx.written = false;
					return;
				}
				buffer_ctx.written = true;
				self.queue.write_buffer(&buffer_ctx.buffer, 0, &data);
			}
			UserEvent::SaveScreenshot {
				window,
				path,
			} => {
				self.pending_screenshots.insert(window.id, path);
			}
		}
	}

	/*fn about_to_wait(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
		event_loop.set_control_flow(ControlFlow::WaitUntil(
			Instant::now() + Duration::from_millis(3000),
		));
		//sleep(Duration::from_millis(3000));
		let dt = self.last_on_process_time.elapsed().as_secs_f32();
		self.last_on_process_time = Instant::now();
		let timer = Instant::now();
		self.engine.render(dt);
	}*/

	fn about_to_wait(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
		let dt = self.last_on_process_time.elapsed().as_secs_f32();
		if dt < 0.016 {
			return
		}
		self.last_on_process_time = Instant::now();
		self.engine.render(dt);
		self.iterations += 1;
		if let Some(max) = self.max_iterations {
			if self.progress_interval > 0 && (self.iterations % self.progress_interval == 0 || self.iterations == max) {
				let elapsed = self.start_time.elapsed().as_secs_f64();
				let rate = if elapsed > 0.0 {
					self.iterations as f64 / elapsed
				} else {
					0.0
				};
				crate::log1!("Iterations: {}/{} ({:.2} it/s)", self.iterations, max, rate);
			}
		}
		if let Some(max) = self.max_iterations {
			if self.iterations >= max {
				crate::log1!("Exiting: ITERATIONS limit reached ({}).", max);
				self.log_exit_stats();
				event_loop.exit();
			}
		}
	}

	fn window_event(
		&mut self,
		event_loop: &winit::event_loop::ActiveEventLoop,
		window_id: winit::window::WindowId,
		event: winit::event::WindowEvent,
	) {
		let window_ctx = match self.windows.iter_mut().find(|window| window.winit_id == window_id) {
			Some(window) => window,
			None => {
				log::error!("Window not found: {:?}", window_id);
				return;
			}
		};

		match event {
			WindowEvent::CloseRequested => {
				crate::log1!("Exiting: window close requested.");
				self.log_exit_stats();
				event_loop.exit();
			}
			WindowEvent::RedrawRequested => {
				println!("redraw requested for window {:?}", window_id);
			}
			WindowEvent::CursorMoved {
				device_id,
				position,
			} => {
				let size = &window_ctx.wininit_window.inner_size();
				let middle_x = size.width as f64 / 2.0;
				let middle_y = size.height as f64 / 2.0;
				if window_ctx.lock_cursor {
					let dx = (position.x - middle_x) as f32;
					let dy = (position.y - middle_y) as f32;
					if dx != 0.0 || dy != 0.0 {
						self.engine.on_mouse_input(
							WindowHandle { id: window_ctx.window_id },
							MouseEvent::Moved { dx, dy },
						);
					}
					window_ctx
						.wininit_window
						.set_cursor_position(PhysicalPosition::new(middle_x, middle_y))
						.unwrap();
					window_ctx.wininit_window.set_cursor_visible(false);
					window_ctx.last_cursor_pos = Some(PhysicalPosition::new(middle_x, middle_y));
				} else if let Some(prev) = window_ctx.last_cursor_pos {
					let dx = (position.x - prev.x) as f32;
					let dy = (position.y - prev.y) as f32;
					if dx != 0.0 || dy != 0.0 {
						self.engine.on_mouse_input(
							WindowHandle { id: window_ctx.window_id },
							MouseEvent::Moved { dx, dy },
						);
					}
					window_ctx.last_cursor_pos = Some(position);
				} else {
					window_ctx.last_cursor_pos = Some(position);
				}
			}
			WindowEvent::MouseInput {
				device_id,
				state,
				button,
			} => {
				let button = match button {
					winit::event::MouseButton::Left => MouseButton::Left,
					winit::event::MouseButton::Right => MouseButton::Right,
					winit::event::MouseButton::Middle => MouseButton::Middle,
					_ => return,
				};

				let event = if state.is_pressed() {
					MouseEvent::Pressed {
						button,
					}
				} else {
					MouseEvent::Released {
						button,
					}
				};

				self.engine.on_mouse_input(WindowHandle {
					id: window_ctx.window_id,
				}, event);
			},
			WindowEvent::KeyboardInput {
				device_id,
				event,
				is_synthetic,
			} => match event {
				winit::event::KeyEvent {
					state,
					location,
					physical_key,
					repeat,
					..
				} => {
					if !repeat {
						match physical_key {
							winit::keyboard::PhysicalKey::Code(code) => {
								let key = match code {
									KeyCode::ArrowUp => KeyboardKey::Up,
									KeyCode::ArrowDown => KeyboardKey::Down,
									KeyCode::ArrowLeft => KeyboardKey::Left,
									KeyCode::ArrowRight => KeyboardKey::Right,
									KeyCode::KeyW => KeyboardKey::W,
									KeyCode::KeyA => KeyboardKey::A,
									KeyCode::KeyS => KeyboardKey::S,
									KeyCode::KeyD => KeyboardKey::D,
									KeyCode::KeyF => KeyboardKey::F,
									KeyCode::KeyG => KeyboardKey::G,
									KeyCode::KeyR => KeyboardKey::R,
									KeyCode::KeyE => KeyboardKey::E,
									KeyCode::ControlLeft => KeyboardKey::ControlLeft,
									KeyCode::Space => KeyboardKey::Space,
									KeyCode::ShiftLeft => KeyboardKey::ShiftLeft,
									KeyCode::Digit1 => KeyboardKey::Digit1,
									KeyCode::Digit2 => KeyboardKey::Digit2,
									KeyCode::Digit3 => KeyboardKey::Digit3,
									KeyCode::Digit4 => KeyboardKey::Digit4,
									KeyCode::Digit5 => KeyboardKey::Digit5,
									KeyCode::Digit6 => KeyboardKey::Digit6,
									KeyCode::Escape => {
										process::exit(0);
									}
									_ => return,
								};
								let action = if state.is_pressed() {
									KeyAction::Pressed
								} else {
									KeyAction::Released
								};
								self.engine.on_keyboard_input(WindowHandle {
									id: window_ctx.window_id,
								}, key, action);
							}
							winit::keyboard::PhysicalKey::Unidentified(_) => {}
						}
					}
				}
			},
			WindowEvent::MouseWheel {
				device_id,
				delta,
				phase,
			} => {
				crate::log2!("scroll delta: {:?}", delta);
				match delta {
					MouseScrollDelta::LineDelta(dx, dy) => {
						let event = MouseEvent::Wheel {
							dx,
							dy,
						};
						self.engine.on_mouse_input(WindowHandle {
							id: window_ctx.window_id,
						}, event);
					}
					_ => {}
				}
			}
			_ => {}
		}
	}
}

fn run_with_winit(app: impl App, max_iterations: Option<u64>) -> anyhow::Result<()> {
	let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
	let adapters = instance.enumerate_adapters(wgpu::Backends::all());
	for adapter in adapters {
		println!("Adapter: {:?}", adapter.get_info());
	}
	let mut adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()));
	if adapter.is_none() {
		let fallback = wgpu::RequestAdapterOptions {
			force_fallback_adapter: true,
			..Default::default()
		};
		adapter = block_on(instance.request_adapter(&fallback));
	}
	let adapter = adapter.expect("Failed to find an appropriate adapter");
	let (device, queue) = block_on(adapter
		.request_device(
			&wgpu::DeviceDescriptor {
				required_features: wgpu::Features::VERTEX_WRITABLE_STORAGE,
				required_limits: wgpu::Limits {
					max_uniform_buffer_binding_size: 20_000_000,
					max_buffer_size: 100_000_000,
					max_bind_groups: 8,
					..Default::default()
				},
			..Default::default()
		},
		None,
		))
		.expect("Failed to create device");

	let device = Arc::new(device);
	let queue = Arc::new(queue);
	let adapter = Arc::new(adapter);
	let instance = Arc::new(instance);

	let event_loop = EventLoop::<UserEvent>::with_user_event().build()?;
	let proxy = event_loop.create_proxy();
	let hardware = WgpuHardware::new(proxy, instance.clone(), adapter.clone(), device.clone(), queue.clone());
	let engine = Engine::new(app, hardware);
	let mut handler = PgeWininitHandler::new(engine, adapter, device, queue, instance, max_iterations);
	Ok(event_loop.run_app(&mut handler)?)
}

pub fn run(app: impl App) -> anyhow::Result<()> {
	if is_headless() {
		return run_headless(app);
	}
	run_with_winit(app, read_iterations())
}

fn flag_enabled(name: &str) -> bool {
	if matches!(env::var(name).as_deref(), Ok("1")) {
		return true;
	}
	let arg = format!("{name}=1");
	env::args().any(|value| value == arg)
}

fn arg_value(name: &str) -> Option<String> {
	let prefix = format!("{name}=");
	env::args().find_map(|value| value.strip_prefix(&prefix).map(str::to_string))
}

fn is_headless() -> bool {
	flag_enabled("HEADLESS") || flag_enabled("SCREENSHOT")
}

fn screenshot_enabled() -> bool {
	flag_enabled("SCREENSHOT")
}

fn screenshot_interval_from_env() -> u64 {
	match env::var("SCREENSHOT_INTERVAL").ok().and_then(|value| value.parse::<u64>().ok()) {
		Some(0) | None => 1,
		Some(value) => value,
	}
}

fn screenshot_dir_from_env() -> Option<PathBuf> {
	if !screenshot_enabled() {
		return None;
	}
	let dir = match env::current_dir() {
		Ok(dir) => dir.join("workdir").join("screenshots"),
		Err(err) => {
			log::error!("Failed to read current dir for screenshots: {:?}", err);
			PathBuf::from("workdir").join("screenshots")
		}
	};
	if let Err(err) = fs::create_dir_all(&dir) {
		log::error!("Failed to create screenshot directory {:?}: {:?}", dir, err);
	}
	Some(dir)
}

fn run_headless_loop<A, H, F>(app: A, hardware: H, mut tick: F) -> anyhow::Result<()>
where
	A: App,
	H: Hardware,
	F: FnMut(&mut Engine<A, H>, f32),
{
	let mut engine = Engine::new(app, hardware);
	let mut last_tick = Instant::now();
	let start_time = Instant::now();
	let target_dt = Duration::from_millis(16);
	let max_iterations = read_iterations();
	let progress_interval = max_iterations
		.map(iteration_log_interval)
		.unwrap_or(0);
	let mut iterations = 0u64;

	loop {
		if let Some(max) = max_iterations {
			if iterations >= max {
				crate::log1!("Headless exiting: ITERATIONS limit reached ({}).", max);
				log_exit_stats(iterations, start_time);
				break;
			}
		}
		let elapsed = last_tick.elapsed();
		if elapsed < target_dt {
			sleep(target_dt - elapsed);
			continue;
		}
		let dt = elapsed.as_secs_f32();
		last_tick = Instant::now();
		tick(&mut engine, dt);
		iterations += 1;
		if let Some(max) = max_iterations {
			if progress_interval > 0 && (iterations % progress_interval == 0 || iterations == max) {
				let elapsed = start_time.elapsed().as_secs_f64();
				let rate = if elapsed > 0.0 {
					iterations as f64 / elapsed
				} else {
					0.0
				};
				crate::log1!("Headless iterations: {}/{} ({:.2} it/s)", iterations, max, rate);
			}
		}
	}
	Ok(())
}

fn run_headless(app: impl App) -> anyhow::Result<()> {
	if screenshot_enabled() {
		return run_headless_with_wgpu(app);
	}
	run_headless_loop(app, MockHardware::new(), |engine, dt| engine.tick_headless(dt))
}

fn run_headless_with_wgpu(app: impl App) -> anyhow::Result<()> {
	let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
	let mut adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()));
	if adapter.is_none() {
		let fallback = wgpu::RequestAdapterOptions {
			force_fallback_adapter: true,
			..Default::default()
		};
		adapter = block_on(instance.request_adapter(&fallback));
	}
	let adapter = match adapter {
		Some(adapter) => adapter,
		None => {
			log::error!("Failed to find an adapter for headless screenshots; falling back to windowed screenshots.");
			return run_with_winit(app, read_iterations());
		}
	};
	let (device, queue) = block_on(adapter
		.request_device(
			&wgpu::DeviceDescriptor {
				required_features: wgpu::Features::VERTEX_WRITABLE_STORAGE,
				required_limits: wgpu::Limits {
					max_uniform_buffer_binding_size: 20_000_000,
					max_buffer_size: 100_000_000,
					max_bind_groups: 8,
					..Default::default()
				},
			..Default::default()
		},
		None,
		))
		.expect("Failed to create device");

	let device = Arc::new(device);
	let queue = Arc::new(queue);
	let hardware = HeadlessWgpuHardware::new(device, queue);
	run_headless_loop(app, hardware, |engine, dt| engine.render(dt))
}

fn read_iterations() -> Option<u64> {
	match env::var("ITERATIONS") {
		Ok(value) => value.parse::<u64>().ok(),
		Err(_) => arg_value("ITERATIONS").and_then(|value| value.parse::<u64>().ok()),
	}
}

fn iteration_log_interval(max: u64) -> u64 {
	if max <= 10 {
		1
	} else if max <= 100 {
		10
	} else if max <= 1000 {
		100
	} else if max <= 10_000 {
		1000
	} else {
		10_000
	}
}

fn log_exit_stats(iterations: u64, start_time: Instant) {
	let runtime = start_time.elapsed();
	let runtime_secs = runtime.as_secs_f64();
	let avg_fps = if runtime_secs > 0.0 {
		iterations as f64 / runtime_secs
	} else {
		0.0
	};
	crate::log1!(
		"Iterations: {} Average FPS: {:.2}, runtime: {:.2}s",
		iterations,
		avg_fps,
		runtime_secs
	);
}

struct BufferContext {
	id: u32,
	name: String,
	buffer: wgpu::Buffer,
	bind_group: wgpu::BindGroup,
	written: bool,
}

struct TextureContext {
	id: u32,
	texture: wgpu::Texture,
	bind_group: wgpu::BindGroup,
}

pub struct WgpuHardware {
	device: Arc<wgpu::Device>,
	queue: Arc<wgpu::Queue>,
	instance: Arc<wgpu::Instance>,
	adapter: Arc<wgpu::Adapter>,
	proxy: winit::event_loop::EventLoopProxy<UserEvent>,
	pipeline_id: u32,
	buffer_id: u32,
	texture_id: u32,
	window_id: u32,
}

impl WgpuHardware {
	pub fn new(proxy: winit::event_loop::EventLoopProxy<UserEvent>, instance: Arc<wgpu::Instance>, adapter: Arc<wgpu::Adapter>, device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
		Self {
			instance,
			device,
			queue,
			adapter,
			proxy,
			pipeline_id: 1,
			buffer_id: 1,
			texture_id: 1,
			window_id: 1,
		}
	}
}

impl Hardware for WgpuHardware {
	fn create_buffer(&mut self, name: &str, size: u64) -> BufferHandle {
		let buffer_id = self.buffer_id;
		self.proxy.send_event(UserEvent::CreateBuffer {
			name: name.to_string(),
			buffer_id,
			size,
		});
		self.buffer_id += 1;
		BufferHandle {
			id: buffer_id,
			size,
		}
	}

	fn destroy_buffer(&mut self, handle: BufferHandle) {
		self.proxy.send_event(UserEvent::DestroyBuffer {
			buffer_id: handle.id,
		});
	}

	fn create_texture(&mut self, name: &str, data: &[u8], width: u32, height: u32) -> TextureHandle {
		let texture_id = self.texture_id;
		self.proxy.send_event(UserEvent::CreateTexture {
			texture_id,
			name: name.to_string(),
			data: data.to_vec(),
			width,
			height,
		});
		self.texture_id += 1;
		TextureHandle {
			id: texture_id,
		}
	}

	fn create_window(&mut self, window: &Window) -> WindowHandle {
		let window_id = self.window_id;
		let args = CreateWindow {
			window_id,
			name: window.title.clone(),
			size: Some(Size {
				height: window.height,
				width: window.width,
			}),
			fullscreen: window.fullscreen,
			lock_cursor: window.lock_cursor,
		};
		self.proxy.send_event(UserEvent::CreateWindow(args));
		self.window_id += 1;
		WindowHandle {
			id: window_id,
		}
	}
	
	fn destroy_window(&mut self, handle: WindowHandle) {
		self.proxy.send_event(UserEvent::DestroyWindow {
			window_id: handle.id,
		});
	}

	fn create_pipeline(&mut self, name: &str, window: WindowHandle) -> PipelineHandle {
		let pipeline_id = self.pipeline_id;
		self.proxy.send_event(UserEvent::CreatePipeline {
			window,
			name: name.to_string(),
			pipeline_id,
		});
		self.pipeline_id += 1;
		PipelineHandle {
			id: pipeline_id,
		}
	}

	fn write_buffer(&mut self, buffer: BufferHandle, data: &[u8]) {
		self.proxy.send_event(UserEvent::WriteBuffer {
			buffer,
			data: data.to_vec(),
		});
	}

	fn render(&mut self, encoder: RenderEncoder, window: WindowHandle) {
		self.proxy.send_event(UserEvent::Render {
			window,
			encoder,
		});
	}

	fn save_screenshot(&mut self, window: WindowHandle, path: &str) {
		self.proxy.send_event(UserEvent::SaveScreenshot {
			window,
			path: path.to_string(),
		});
	}
}

impl HeadlessWgpuHardware {
	fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
		let screenshot_dir = screenshot_dir_from_env();
		let screenshot_interval = screenshot_interval_from_env();
		Self {
			device,
			queue,
			pipeline_id: 1,
			buffer_id: 1,
			texture_id: 1,
			window_id: 1,
			windows: Vec::new(),
			pipelines: Vec::new(),
			buffers: Vec::new(),
			textures: Vec::new(),
			pending_screenshots: HashMap::new(),
			screenshot_dir,
			screenshot_counter: 0,
			screenshot_interval,
		}
	}
}

impl Hardware for HeadlessWgpuHardware {
	fn create_buffer(&mut self, name: &str, size: u64) -> BufferHandle {
		let buffer_id = self.buffer_id;
		self.buffer_id += 1;
		let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(name),
			size,
			usage: wgpu::BufferUsages::VERTEX
				| wgpu::BufferUsages::COPY_DST
				| wgpu::BufferUsages::COPY_SRC
				| wgpu::BufferUsages::INDEX
				| wgpu::BufferUsages::UNIFORM
				| wgpu::BufferUsages::STORAGE,
			mapped_at_creation: false,
		});
		let layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: None,
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
		});
		let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &buffer,
					offset: 0,
					size: None,
				}),
			}],
			label: Some("Buffer Bind Group"),
		});
		self.buffers.push(BufferContext {
			id: buffer_id,
			name: name.to_string(),
			buffer,
			bind_group,
			written: false,
		});
		BufferHandle { id: buffer_id, size }
	}

	fn destroy_buffer(&mut self, handle: BufferHandle) {
		if let Some(buffer_ctx) = self.buffers.iter_mut().find(|b| b.id == handle.id) {
			buffer_ctx.buffer.destroy();
		}
		self.buffers.retain(|b| b.id != handle.id);
	}

	fn create_texture(&mut self, name: &str, data: &[u8], width: u32, height: u32) -> TextureHandle {
		let texture_id = self.texture_id;
		self.texture_id += 1;
		let size = wgpu::Extent3d {
			width,
			height,
			depth_or_array_layers: 1,
		};
		let texture = self.device.create_texture(&wgpu::TextureDescriptor {
			label: Some(name),
			size,
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::Rgba8Unorm,
			usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
			view_formats: Default::default(),
		});
		self.queue.write_texture(
			wgpu::ImageCopyTexture {
				texture: &texture,
				mip_level: 0,
				origin: wgpu::Origin3d::ZERO,
				aspect: wgpu::TextureAspect::All,
			},
			data,
			wgpu::ImageDataLayout {
				offset: 0,
				bytes_per_row: Some(4 * width),
				rows_per_image: Some(height),
			},
			size
		);
		let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());
		let sampler = self.device.create_sampler(&wgpu::SamplerDescriptor {
			address_mode_u: wgpu::AddressMode::Repeat,
			address_mode_v: wgpu::AddressMode::Repeat,
			address_mode_w: wgpu::AddressMode::Repeat,
			mag_filter: wgpu::FilterMode::Linear,
			min_filter: wgpu::FilterMode::Linear,
			mipmap_filter: wgpu::FilterMode::Nearest,
			..Default::default()
		});
		let texture_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &TextureBuffer::create_bind_group_layout(&self.device),
			entries: &[
				wgpu::BindGroupEntry {
					binding: 0,
					resource: wgpu::BindingResource::TextureView(&texture_view),
				},
				wgpu::BindGroupEntry {
					binding: 1,
					resource: wgpu::BindingResource::Sampler(&sampler),
				},
			],
			label: Some("texture_bind_group"),
		});
		self.textures.push(TextureContext {
			id: texture_id,
			texture,
			bind_group: texture_bind_group,
		});
		TextureHandle { id: texture_id }
	}

	fn create_pipeline(&mut self, name: &str, window: WindowHandle) -> PipelineHandle {
		let pipeline_id = self.pipeline_id;
		self.pipeline_id += 1;
		let window_ctx = match self.windows.iter().find(|window_ctx| window_ctx.window_id == window.id) {
			Some(window_ctx) => window_ctx,
			None => {
				log::error!("Window not found: {:?}", window);
				return PipelineHandle { id: pipeline_id };
			}
		};
		let (render_pipeline, depth_texture_view, uses_depth) = if name == "gui" {
			let shader_source =
				wgpu::ShaderSource::Wgsl(include_str!("../shaders/gui_shader.wgsl").into());
			let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Gui Shader"),
				source: shader_source,
			});

			let render_pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Gui Pipeline Layout"),
				bind_group_layouts: &[],
				push_constant_ranges: &[],
			});

			let position_layout = wgpu::VertexBufferLayout {
				array_stride: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
				step_mode: wgpu::VertexStepMode::Vertex,
				attributes: &[wgpu::VertexAttribute {
					offset: 0,
					format: wgpu::VertexFormat::Float32x3,
					shader_location: 0,
				}],
			};
			let color_layout = wgpu::VertexBufferLayout {
				array_stride: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
				step_mode: wgpu::VertexStepMode::Vertex,
				attributes: &[wgpu::VertexAttribute {
					offset: 0,
					format: wgpu::VertexFormat::Float32x3,
					shader_location: 1,
				}],
			};

			let render_pipeline = self.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
				label: Some("Gui Pipeline"),
				layout: Some(&render_pipeline_layout),
				vertex: wgpu::VertexState {
					module: &shader,
					entry_point: "vs_main",
					buffers: &[position_layout, color_layout],
					compilation_options: Default::default(),
				},
				fragment: Some(wgpu::FragmentState {
					module: &shader,
					entry_point: "fs_main",
					targets: &[Some(wgpu::ColorTargetState {
						format: wgpu::TextureFormat::Bgra8UnormSrgb,
						blend: Some(wgpu::BlendState {
							color: wgpu::BlendComponent::REPLACE,
							alpha: wgpu::BlendComponent::REPLACE,
						}),
						write_mask: wgpu::ColorWrites::ALL,
					})],
					compilation_options: Default::default(),
				}),
				primitive: wgpu::PrimitiveState {
					topology: wgpu::PrimitiveTopology::TriangleList,
					strip_index_format: None,
					front_face: wgpu::FrontFace::Ccw,
					cull_mode: None,
					polygon_mode: wgpu::PolygonMode::Fill,
					unclipped_depth: false,
					conservative: false,
				},
				depth_stencil: None,
				multisample: wgpu::MultisampleState {
					count: 1,
					mask: !0,
					alpha_to_coverage_enabled: false,
				},
				multiview: None,
			});

			(render_pipeline, None, false)
		} else {
			let depth_texture = self.device.create_texture(&wgpu::TextureDescriptor {
				label: None,
				size: wgpu::Extent3d {
					width: window_ctx.size.width,
					height: window_ctx.size.height,
					depth_or_array_layers: 1,
				},
				mip_level_count: 1,
				sample_count: 1,
				dimension: wgpu::TextureDimension::D2,
				format: wgpu::TextureFormat::Depth24PlusStencil8,
				usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
				view_formats: Default::default(),
			});
			let depth_texture_view =
				Arc::new(depth_texture.create_view(&wgpu::TextureViewDescriptor::default()));
			let camera_bind_group_layout = RawCamera::create_bind_group_layout(&self.device);
			let point_light_bind_group_layout = RawPointLight::create_bind_group_layout(&self.device);
			let base_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
			let metallic_roughness_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
			let normal_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
			let occlusion_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
			let emissive_texture_bind_group_layout = TextureBuffer::create_bind_group_layout(&self.device);
			let material_bind_group_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				label: Some("Material Bind Group Layout"),
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Storage { read_only: true },
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				}],
			});
			let tex_coords_layout = wgpu::VertexBufferLayout {
				array_stride: std::mem::size_of::<TexCoords>() as wgpu::BufferAddress,
				step_mode: wgpu::VertexStepMode::Vertex,
				attributes: &[wgpu::VertexAttribute {
					offset: 0,
					format: wgpu::VertexFormat::Float32x2,
					shader_location: 2,
				}],
			};
			let layouts = &[
				&camera_bind_group_layout, 
				&point_light_bind_group_layout, 
				&base_texture_bind_group_layout,
				&metallic_roughness_texture_bind_group_layout,
				&normal_texture_bind_group_layout,
				&occlusion_texture_bind_group_layout,
				&emissive_texture_bind_group_layout,
				&material_bind_group_layout,
			];
			let buffers = &[Vertices::desc(), RawInstance::desc(), Normals::desc(), tex_coords_layout];
			let shader_source = wgpu::ShaderSource::Wgsl(include_str!("../shaders/3d_shader.wgsl").into());
			let shader = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
				label: Some("Shader"),
				source: shader_source
			});
			let render_pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
				label: Some("Render Pipeline Layout"),
				bind_group_layouts: layouts,
				push_constant_ranges: &[],
			});
			let depth_stencil_state = wgpu::DepthStencilState {
				format: wgpu::TextureFormat::Depth24PlusStencil8,
				depth_write_enabled: true,
				depth_compare: wgpu::CompareFunction::Less,
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			};
			let render_pipeline = self.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
				label: Some("Render Pipeline"),
				layout: Some(&render_pipeline_layout),
				vertex: wgpu::VertexState {
					module: &shader,
					entry_point: "vs_main",
					buffers,
					compilation_options: Default::default(),
				},
				fragment: Some(wgpu::FragmentState {
					module: &shader,
					entry_point: "fs_main",
					targets: &[Some(wgpu::ColorTargetState {
						format: wgpu::TextureFormat::Bgra8UnormSrgb,
						blend: Some(wgpu::BlendState {
							color: wgpu::BlendComponent::REPLACE,
							alpha: wgpu::BlendComponent::REPLACE,
						}),
						write_mask: wgpu::ColorWrites::ALL,
					})],
					compilation_options: Default::default(),
				}),
				primitive: wgpu::PrimitiveState {
					topology: wgpu::PrimitiveTopology::TriangleList,
					strip_index_format: None,
					front_face: wgpu::FrontFace::Ccw,
					cull_mode: None,
					polygon_mode: wgpu::PolygonMode::Fill,
					unclipped_depth: false,
					conservative: false,
				},
				depth_stencil: Some(depth_stencil_state),
				multisample: wgpu::MultisampleState {
					count: 1,
					mask: !0,
					alpha_to_coverage_enabled: false,
				},
				multiview: None,
			});

			(render_pipeline, Some(depth_texture_view), true)
		};
		self.pipelines.push(PipelineContext {
			id: pipeline_id,
			pipeline: Arc::new(render_pipeline),
			depth_texture_view,
			uses_depth,
		});
		PipelineHandle { id: pipeline_id }
	}

	fn render(&mut self, encoder: RenderEncoder, window: WindowHandle) {
		let mut screenshot_path = self.pending_screenshots.remove(&window.id);
		let frame_index = self.screenshot_counter;
		self.screenshot_counter += 1;
		if screenshot_path.is_none() {
			if let Some(dir) = &self.screenshot_dir {
				if self.screenshot_interval > 0 && frame_index % self.screenshot_interval == 0 {
					let filename = format!("screenshot_{}_{}.png", window.id, frame_index);
					let path = dir.join(filename);
					screenshot_path = Some(path.to_string_lossy().to_string());
				}
			}
		}
		let window_ctx = match self.windows.iter().find(|window_ctx| window_ctx.window_id == window.id) {
			Some(window_ctx) => window_ctx,
			None => {
				log::error!("Window not found: {:?} => RETURN", window);
				return;
			}
		};
		let mut wgpu_encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
			label: Some("Render Encoder"),
		});
		let mut screenshot_buffer = None;
		let mut screenshot_info = None;
		for (pass_index, pass) in encoder.passes.into_iter().enumerate() {
			let pipeline = pass.pipeline.unwrap();
			let pipeline_ctx = match self.pipelines.iter().find(|pipeline_ctx| pipeline_ctx.id == pipeline.id) {
				Some(pipeline_ctx) => pipeline_ctx,
				None => {
					log::error!("Pipeline not found: {:?} => RETURN", pipeline);
					return;
				}
			};
			let load_op = if pass_index == 0 {
				wgpu::LoadOp::Clear(wgpu::Color {
					r: 0.1,
					g: 0.2,
					b: 0.3,
					a: 1.0,
				})
			} else {
				wgpu::LoadOp::Load
			};
			let depth_stencil_attachment = if pipeline_ctx.uses_depth {
				Some(wgpu::RenderPassDepthStencilAttachment {
					view: pipeline_ctx
						.depth_texture_view
						.as_ref()
						.expect("missing depth texture view"),
					depth_ops: Some(wgpu::Operations {
						load: if pass_index == 0 {
							wgpu::LoadOp::Clear(1.0)
						} else {
							wgpu::LoadOp::Load
						},
						store: wgpu::StoreOp::Store,
					}),
					stencil_ops: None,
				})
			} else {
				None
			};
			let mut wgpu_pass = wgpu_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("Render Pass"),
				color_attachments: &[Some(wgpu::RenderPassColorAttachment {
					view: &window_ctx.color_view,
					resolve_target: None,
					ops: wgpu::Operations {
						load: load_op,
						store: wgpu::StoreOp::Store,
					},
				})],
				depth_stencil_attachment,
				..Default::default()
			});
			wgpu_pass.set_pipeline(&pipeline_ctx.pipeline);
			for subpass in &pass.subpasses {
				for (slot, texture) in &subpass.textures {
					let texture_ctx = match self.textures.iter().find(|t| t.id == texture.id) {
						Some(texture) => texture, 
						None => {
							log::error!("Texture not found: {:?} => RETURN", texture);
							return;
						}
					};
					wgpu_pass.set_bind_group(*slot, &texture_ctx.bind_group, &[]);
				}
				for (slot, buffer) in &subpass.buffers {
					let buffer_ctx = match self.buffers.iter().find(|b| b.id == buffer.id) {
						Some(buffer) => buffer,
						None => {
							log::error!("Buffer not found: {:?} => RETURN", buffer);
							return;
						}
					};
					if !buffer_ctx.written {
						log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", buffer);
						return;
					}
					wgpu_pass.set_bind_group(*slot, &buffer_ctx.bind_group, &[]);
				}
				for (slot, buffer) in &subpass.vertex_buffers {
					let buffer_ctx = match self.buffers.iter().find(|b| b.id == buffer.handle.id) {
						Some(buffer) => buffer,
						None => {
							log::error!("Buffer not found: {:?} => RETURN", buffer);
							return;
						}
					};
					if !buffer_ctx.written {
						log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", buffer);
						return;
					}
					if buffer.range.start == buffer.range.end {
						log::error!("BUFFER RANGE IS ZERO: {:?} => RETURN", buffer);
						continue;
					}
					wgpu_pass.set_vertex_buffer(*slot, buffer_ctx.buffer.slice(buffer.range.clone()));
				}
				if let Some(slice) = &subpass.index_buffer {
					let buffer_ctx = match self.buffers.iter().find(|b| b.id == slice.handle.id) {
						Some(buffer) => buffer,
						None => {
							log::error!("Buffer not found: {:?} => RETURN", slice.handle);
							return;
						}
					};
					if !buffer_ctx.written {
						log::error!("BUFFER NOT WRITTEN: {:?} => RETURN", slice.handle);
						return;
					}
					if slice.range.start == slice.range.end {
						log::error!("BUFFER RANGE IS ZERO: {:?} => RETURN", slice.handle);
						continue;
					}
					wgpu_pass.set_index_buffer(buffer_ctx.buffer.slice(slice.range.clone()), wgpu::IndexFormat::Uint16);
				}
				let indices = subpass.indices.clone().unwrap();
				let instances = subpass.instances.clone().unwrap();
				wgpu_pass.draw_indexed(indices.clone(), 0, instances.clone());
			}
		}
		if screenshot_path.is_some() {
			let bytes_per_pixel = 4;
			let unpadded_bytes_per_row = window_ctx.size.width * bytes_per_pixel;
			let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
			let padded_bytes_per_row_padding = (align - unpadded_bytes_per_row % align) % align;
			let padded_bytes_per_row = unpadded_bytes_per_row + padded_bytes_per_row_padding;
			let output_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("Screenshot Buffer"),
				size: padded_bytes_per_row as u64 * window_ctx.size.height as u64,
				usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			});
			wgpu_encoder.copy_texture_to_buffer(
				wgpu::ImageCopyTexture {
					texture: &window_ctx.color_texture,
					mip_level: 0,
					origin: wgpu::Origin3d::ZERO,
					aspect: wgpu::TextureAspect::All,
				},
				wgpu::ImageCopyBuffer {
					buffer: &output_buffer,
					layout: wgpu::ImageDataLayout {
						offset: 0,
						bytes_per_row: Some(padded_bytes_per_row),
						rows_per_image: Some(window_ctx.size.height),
					},
				},
				window_ctx.size,
			);
			screenshot_buffer = Some(output_buffer);
			screenshot_info = Some((window_ctx.size.width, window_ctx.size.height, unpadded_bytes_per_row, padded_bytes_per_row));
		}
		self.queue.submit(std::iter::once(wgpu_encoder.finish()));
		if let (Some(path), Some(buffer), Some((width, height, unpadded_bytes_per_row, padded_bytes_per_row))) =
			(screenshot_path, screenshot_buffer, screenshot_info)
		{
			let buffer_slice = buffer.slice(..);
			let (tx, rx) = std::sync::mpsc::channel();
			buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
				let _ = tx.send(result);
			});
			self.device.poll(wgpu::Maintain::Wait);
			match rx.recv() {
				Ok(Ok(())) => {}
				Ok(Err(err)) => {
					log::error!("Failed to map screenshot buffer: {:?}", err);
					buffer.unmap();
					return;
				}
				Err(err) => {
					log::error!("Failed to receive screenshot buffer map result: {:?}", err);
					buffer.unmap();
					return;
				}
			}
			let data = buffer_slice.get_mapped_range();
			let mut pixels: Vec<u8> = Vec::with_capacity((width * height * 4) as usize);
			for chunk in data.chunks(padded_bytes_per_row as usize) {
				pixels.extend_from_slice(&chunk[..unpadded_bytes_per_row as usize]);
			}
			drop(data);
			buffer.unmap();
			for chunk in pixels.chunks_exact_mut(4) {
				chunk.swap(0, 2);
			}
			match image::save_buffer(
				&path,
				&pixels,
				width,
				height,
				image::ColorType::Rgba8,
			) {
				Ok(_) => crate::log1!("Screenshot saved to {}", path),
				Err(err) => log::error!("Failed to save screenshot: {:?}", err),
			}
		}
	}

	fn create_window(&mut self, window: &Window) -> WindowHandle {
		let window_id = self.window_id;
		self.window_id += 1;
		let size = wgpu::Extent3d {
			width: window.width,
			height: window.height,
			depth_or_array_layers: 1,
		};
		let color_texture = self.device.create_texture(&wgpu::TextureDescriptor {
			label: Some("Headless Color Texture"),
			size,
			mip_level_count: 1,
			sample_count: 1,
			dimension: wgpu::TextureDimension::D2,
			format: wgpu::TextureFormat::Bgra8UnormSrgb,
			usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
			view_formats: Default::default(),
		});
		let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());
		self.windows.push(HeadlessWindowContext {
			window_id,
			size,
			color_texture,
			color_view,
		});
		WindowHandle { id: window_id }
	}

	fn destroy_window(&mut self, handle: WindowHandle) {
		self.windows.retain(|window_ctx| window_ctx.window_id != handle.id);
	}

	fn write_buffer(&mut self, buffer: BufferHandle, data: &[u8]) {
		let buffer_ctx = match self.buffers.iter_mut().find(|b| b.id == buffer.id) {
			Some(b) => b,
			None => {
				log::error!("Buffer not found: {:?}", buffer);
				return;
			}
		};
		if data.is_empty() {
			buffer_ctx.written = false;
			return;
		}
		buffer_ctx.written = true;
		self.queue.write_buffer(&buffer_ctx.buffer, 0, data);
	}

	fn save_screenshot(&mut self, window: WindowHandle, path: &str) {
		self.pending_screenshots.insert(window.id, path.to_string());
	}
}

/*#[derive(Debug, Clone)]
pub struct Texture {
	texture: Arc<wgpu::Texture>,
	queue: Arc<wgpu::Queue>,
	bind_group: Arc<wgpu::BindGroup>,
}

impl Texture {
	pub fn bind_group(&self) -> &wgpu::BindGroup {
		&self.bind_group
	}
}*/
