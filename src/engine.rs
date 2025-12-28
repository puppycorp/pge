use glam::Mat4;
use ttf_parser::Transform;

use crate::buffer::Buffer;
use crate::compositor::Compositor;
use crate::hardware;
use crate::hardware::BufferHandle;
use crate::hardware::Hardware;
use crate::hardware::PipelineHandle;
use crate::hardware::RenderEncoder;
use crate::hardware::TextureHandle;
use crate::hardware::WindowHandle;
use crate::internal_types::*;
use crate::state::State;
use crate::types::*;
use crate::utility::topo_sort_nodes;
use crate::ArenaId;
use crate::GUIElement;
use crate::Window;
use std::collections::HashMap;
use std::ops::Range;
use std::time::Duration;
use std::time::Instant;

#[derive(Debug, Clone)]
pub struct DrawCall {
	pub material: Option<ArenaId<Material>>,
	pub vertices: Range<u64>,
	pub indices: Range<u64>,
	pub normals: Range<u64>,
	pub tex_coords: Range<u64>,
	pub instances: Range<u32>,
	pub indices_range: Range<u32>,
}

#[derive(Debug, Clone)]
pub struct View {
	pub camview: CamView,
	pub scene_id: ArenaId<Scene>,
}

#[derive(Debug, Clone)]
pub struct UIRenderArgs {
	pub ui: ArenaId<GUIElement>,
	pub views: Vec<View>,
}

struct GuiBuffers {
    vertices_buffer: Buffer,
    index_buffer: Buffer,
    color_buffer: Buffer,
    position_range: Range<u64>,
    index_range: Range<u64>,
    colors_range: Range<u64>,
    indices_range: Range<u32>,
}

impl GuiBuffers {
    pub fn new(hardware: &mut impl Hardware) -> Self {
        let vertices_buffer = Buffer::new(hardware.create_buffer("gui_vertices", 1000));
		let index_buffer = Buffer::new(hardware.create_buffer("gui_indices", 1000));
		let color_buffer = Buffer::new(hardware.create_buffer("gui_colors", 1000));
        Self {
            vertices_buffer,
            index_buffer,
            color_buffer,
            position_range: 0..0,
            index_range: 0..0,
            colors_range: 0..0,
            indices_range: 0..0,
        }
    }
}

struct WindowContext {
	window_id: ArenaId<Window>,
	window: WindowHandle,
	pipeline: PipelineHandle,
}

struct NodeComputedMetadata {
	model: glam::Mat4,
	scene_id: ArenaId<Scene>,
}

pub struct Engine<A, H> {
    pub app: A,
    pub state: State,
    hardware: H,
    vertices_buffer: Buffer,
    tex_coords_buffer: Buffer,
    normal_buffer: Buffer,	
    index_buffer: Buffer,
    point_light_buffers: HashMap<ArenaId<Scene>, Buffer>,
    gui_buffers: HashMap<ArenaId<GUIElement>, GuiBuffers>,
    camera_buffers: HashMap<ArenaId<Camera>, Buffer>,
    default_texture: TextureHandle,
	default_point_lights: Buffer,
	default_material: BufferHandle,
    scene_instance_buffers: HashMap<ArenaId<Scene>, Buffer>,
    scene_draw_calls: HashMap<ArenaId<Scene>, Vec<DrawCall>>,
	textures: HashMap<ArenaId<Texture>, TextureHandle>,
	materials: HashMap<ArenaId<Material>, BufferHandle>,
    ui_compositors: HashMap<ArenaId<GUIElement>, Compositor>,
    ui_render_args: HashMap<ArenaId<GUIElement>, UIRenderArgs>,
	windows: Vec<WindowContext>,
	//nodes: HashMap<ArenaId<Node>, NodeComputedMetadata>,
	mesh_nodes: HashMap<ArenaId<Mesh>, Vec<ArenaId<Node>>>,
	topo_sorted_nodes: Vec<ArenaId<Node>>,
	fps: u32
}

impl<A, H> Engine<A, H>
where
    A: App,
    H: Hardware,
{
    pub fn new(mut app: A, mut hardware: H) -> Self {
        let data: [u8; 4] = [255, 255, 255, 255]; // white
		//let data = [0, 0, 0, 0];
        let default_texture = hardware.create_texture("default_texture", &data, 1, 1);

        let vertices_buffer = Buffer::new(hardware.create_buffer("vertices", 1000));
        let tex_coords_buffer = Buffer::new(hardware.create_buffer("tex_coords", 1000));
        let normal_buffer = Buffer::new(hardware.create_buffer("normals", 1000));
        let index_buffer = Buffer::new(hardware.create_buffer("indices", 1000));

		let default_point_lights = Buffer::new(hardware.create_buffer("default_point_lights", 1000));
        
        let default_material_data = RawMaterial::default();
        let default_material = hardware.create_buffer("default_material", 1000);
        hardware.write_buffer(default_material, bytemuck::cast_slice(&[default_material_data]));

        let mut state = State::default();
        app.on_create(&mut state);

		state.print_state();

        Self {
            app,
            state,
            hardware,
            vertices_buffer,
            tex_coords_buffer,
            normal_buffer,
            index_buffer,
            point_light_buffers: HashMap::new(),
            gui_buffers: HashMap::new(),
            camera_buffers: HashMap::new(),
            default_texture,
            scene_instance_buffers: HashMap::new(),
			default_point_lights,
			default_material,
			textures: HashMap::new(),
			materials: HashMap::new(),
            ui_compositors: HashMap::new(),
            scene_draw_calls: HashMap::new(),
            ui_render_args: HashMap::new(),
			windows: Vec::new(),
			//nodes: HashMap::new(),
			mesh_nodes: HashMap::new(),
			fps: 0,
			topo_sorted_nodes: Vec::new(),
        }
    }

	fn process_nodes(&mut self) {
		let timer = Instant::now();
		for (_, nodes) in &mut self.mesh_nodes {
			nodes.clear();
		}

		self.topo_sorted_nodes.clear();
		let sort_timer = Instant::now();
		topo_sort_nodes(&self.state.nodes, &mut self.topo_sorted_nodes);
		if sort_timer.elapsed() > Duration::from_millis(10) {
			crate::log3!("Topo sort {} nodes took {:?}", self.topo_sorted_nodes.len(), sort_timer.elapsed());
		}

		for node_id in &self.topo_sorted_nodes {
			let mut scene_id: Option<ArenaId<Scene>> = None;
			let transform = {
				let node = match self.state.nodes.get(node_id) {
					Some(node) => node,
					None => continue,
				};
				match node.parent {
					NodeParent::Scene(id) => {
						scene_id = Some(id);
						let scene = self.state.scenes.get(&id).unwrap();
						node.matrix() * glam::Mat4::from_scale(scene.scale)
					}
					NodeParent::Orphan => node.matrix(),
					NodeParent::Node(parent_node_id) => {
						let parent = match self.state.nodes.get(&parent_node_id) {
							Some(parent) => parent,
							None => continue,
						};
						scene_id = parent.scene_id;
						parent.global_transform * node.matrix()
					}
				}
			};

			let node = self.state.nodes.get_mut(node_id).unwrap();
			node.global_transform = transform;
			node.scene_id = scene_id;

			if let Some(mesh_id) = node.mesh {
				self.mesh_nodes
					.entry(mesh_id)
					.or_insert(Vec::new())
					.push(*node_id);
			}
		}

		let elapsed = timer.elapsed();
		if elapsed > Duration::from_millis(5) {
			crate::log3!("Node processing took {:?}", elapsed);
		}
	}

	fn process_textures(&mut self) {
		for (texture_id, texture) in &self.state.textures {
			if self.textures.contains_key(&texture_id) {
				continue;
			}
			let mut data = vec![255, 0, 0, 255];
			let mut width = 1;
			let mut height = 1;

			match &texture.source {
				TextureSource::File(path) => {
					crate::log2!("Loading texture from file: {:?}", path);
					match image::open(&path) {
						Ok(img) => {
							let img: image::ImageBuffer<image::Rgba<u8>, Vec<u8>> = img.to_rgba8();
							let dim = img.dimensions();
							data = img.into_raw();
							crate::log2!("Image loaded: {}x{}", dim.0, dim.1);
							width = dim.0 as u32;
							height = dim.1 as u32;
						}
						Err(e) => {
							log::error!("Failed to load image: {:?}", e);
						}
					}
				}
				TextureSource::Buffer { data: d, width: w, height: h } => {
					data = d.clone();
					width = *w;
					height = *h;
				}
				TextureSource::None => {
					log::warn!("TextureSource::None encountered - using red texture");
				}
 			};
			let handle = self.hardware.create_texture(&texture.name, &data, width, height);
			self.textures.insert(texture_id.clone(), handle);
		}
	}

	fn process_materials(&mut self) {
		for (material_id, material) in &self.state.materials {
			if self.materials.contains_key(&material_id) {
				continue;
			}

			let raw_material = RawMaterial {
				base_color_factor: material.base_color_factor,
				metallic_factor: material.metallic_factor,
				roughness_factor: material.roughness_factor,
				normal_texture_scale: material.normal_texture_scale,
				occlusion_strength: material.occlusion_strength,
				emissive_factor: material.emissive_factor,
				_padding: 0.0,
			};
			crate::log4!("new material: {:?}", raw_material);

			let buffer = self.hardware.create_buffer(&format!("material_buffer_{:?}", material_id.index()), 1000);
			self.hardware.write_buffer(buffer, bytemuck::bytes_of(&raw_material));
			self.materials.insert(material_id.clone(), buffer);
		}
	}

    fn process_meshes(&mut self) {
		let timer = Instant::now();
		for (_, s) in &mut self.scene_draw_calls {
			s.clear();
		}
        
		for (mesh_id, mesh) in &self.state.meshes {
			for primitive in &mesh.primitives {
				if primitive.topology == PrimitiveTopology::TriangleList {
					if primitive.vertices.len() == 0 || primitive.indices.len() == 0 {
						continue;
					}

					let vertices_start = self.vertices_buffer.len();
                    self.vertices_buffer.write(bytemuck::cast_slice(&primitive.vertices));
					let vertices_end = self.vertices_buffer.len();

					let normals_start = self.normal_buffer.len();
					self.normal_buffer.write(bytemuck::cast_slice(&primitive.normals));
					let normals_end = self.normal_buffer.len();

					let indices_start = self.index_buffer.len();
					self.index_buffer.write(bytemuck::cast_slice(&primitive.indices));
					let indices_end = self.index_buffer.len();

					let tex_coords_start = self.tex_coords_buffer.len();
					if primitive.tex_coords.len() > 0 {
                        self.tex_coords_buffer.write(bytemuck::cast_slice(&primitive.tex_coords));
					} else {
						let tex_coords = vec![[0.0, 0.0]; primitive.vertices.len()];
						self.tex_coords_buffer.write(bytemuck::cast_slice(&tex_coords));
					}
					let tex_coords_end = self.tex_coords_buffer.len();
					let node_ids = match self.mesh_nodes.get(&mesh_id) {
						Some(ids) => ids,
						None => continue,
					};

					let mut checkpoints: HashMap<ArenaId<Scene>, Range<u32>> = HashMap::new();

					for node_id in node_ids {
						let node = match self.state.nodes.get(node_id) {
							Some(node) => node,
							None => continue,
						};
						let scene_id = match node.scene_id {
							Some(id) => id,
							None => continue,
						};
						let instance = RawInstance {
							model: node.global_transform.to_cols_array_2d(),
						};
						let buffer = self.scene_instance_buffers.entry(scene_id)
							.or_insert_with(|| Buffer::new(self.hardware.create_buffer(&format!("instances_{:?}", scene_id.index()), 1000)));

						let instance_start = buffer.len() as u32 / std::mem::size_of::<RawInstance>() as u32;
						buffer.write(bytemuck::bytes_of(&instance));
						let instance_end = buffer.len() as u32 / std::mem::size_of::<RawInstance>() as u32;

						let checkpoint = checkpoints
							.entry(scene_id)
							.or_insert(instance_start..instance_end);
						checkpoint.end = instance_end;
					}

					for (scene_id, instances) in checkpoints {
						let draw_calls = self.scene_draw_calls.entry(scene_id).or_insert(Vec::new());
						draw_calls.push(DrawCall {
							material: primitive.material,
							vertices: vertices_start..vertices_end,
							indices: indices_start..indices_end,
							normals: normals_start..normals_end,
							tex_coords: tex_coords_start..tex_coords_end,
							instances,
							indices_range: 0..primitive.indices.len() as u32,
						});
					}
				}
			}
		}
		let flush_timer = Instant::now();
		self.vertices_buffer.flush(&mut self.hardware);
		self.tex_coords_buffer.flush(&mut self.hardware);
		self.normal_buffer.flush(&mut self.hardware);
		self.index_buffer.flush(&mut self.hardware);
		for (_, buffer) in &mut self.scene_instance_buffers {
			buffer.flush(&mut self.hardware);
		}
		if flush_timer.elapsed() > Duration::from_millis(10) {
			crate::log3!("Flushing buffers took {:?}", flush_timer.elapsed());
		}

		if timer.elapsed() > Duration::from_millis(10) {
			crate::log3!("Mesh processing took {:?}", timer.elapsed());
		}
    }

    fn process_cameras(&mut self) {
		for (cam_id, cam) in &self.state.cameras {
			let node_id = match cam.node_id {
				Some(id) => id,
				None => continue,
			};
			let node = match self.state.nodes.get(&node_id) {
				Some(node) => node,
				None => continue,
			};
			let model = glam::Mat4::perspective_lh(cam.fovy, cam.aspect, cam.znear, cam.zfar)
				* node.global_transform.inverse();

			let cam = RawCamera {
				model: model.to_cols_array_2d(),
			};
			let buffer = self
				.camera_buffers
				.entry(cam_id)
				.or_insert_with(|| Buffer::new(self.hardware.create_buffer(&format!("camera_buffer_{:?}", cam_id.index()), 1000)));
			buffer.write(bytemuck::bytes_of(&cam));
		}
		for (_, buffer) in &mut self.camera_buffers {
			buffer.flush(&mut self.hardware);
		}
	}

    fn process_point_lights(&mut self) {
		for (_, light) in &self.state.point_lights {
            let node_id = match light.node_id {
                Some(id) => id,
                None => continue,
            };
			let node = match self.state.nodes.get(&node_id) {
				Some(node) => node,
				None => continue,
			};
			let scene_id = match node.scene_id {
				Some(id) => id,
				None => continue,
			};
			let model = node.global_transform;
			let pos = model.w_axis.truncate().into();
			let light = RawPointLight::new(light.color, light.intensity, pos);

			self.point_light_buffers.entry(scene_id).or_insert_with(|| {
				crate::log2!("Creating new point light buffer for scene ID: {:?}", scene_id);
				Buffer::new(self.hardware.create_buffer("pointlight", 1000))
			}).write(bytemuck::bytes_of(&light));
		}
		for (_, buffer) in &mut self.point_light_buffers {
			buffer.flush(&mut self.hardware);
		}
	}

    fn process_ui(&mut self) {
		for (ui_id, gui) in &self.state.guis {
			let c: &mut Compositor = self
				.ui_compositors
				.entry(ui_id.clone())
				.or_insert_with(|| {
					crate::log2!("Creating new Compositor for UI ID: {:?}", ui_id); // Debug print
					Compositor::new()
				});
			c.process(gui);
	
			let buffers = self
				.gui_buffers
				.entry(ui_id)
				.or_insert_with(|| {
					crate::log2!("Creating new GuiBuffers for UI ID: {:?}", ui_id); // Debug print
					GuiBuffers::new(&mut self.hardware)
				});

            if c.positions.len() > 0 {
                let positions_data = bytemuck::cast_slice(&c.positions);
                let positions_data_len = positions_data.len() as u64;
                buffers.vertices_buffer.write(positions_data);
                buffers.position_range = 0..positions_data_len;
            }

            if c.indices.len() > 0 {
                let indices_data = bytemuck::cast_slice(&c.indices);
                let indices_data_len = indices_data.len() as u64;
                buffers.index_buffer.write(indices_data);
                buffers.index_range = 0..indices_data_len;
                buffers.indices_range = 0..c.indices.len() as u32;
            }

            if c.colors.len() > 0 {
                let colors_data = bytemuck::cast_slice(&c.colors);
                let colors_data_len = colors_data.len() as u64;
                buffers.color_buffer.write(colors_data);
                buffers.colors_range = 0..colors_data_len;
            }
			let render_args = self.ui_render_args.entry(ui_id.clone()).or_insert(UIRenderArgs {
				ui: ui_id.clone(),
				views: Vec::new(),
			});
			render_args.views.clear();
			for view in &c.views {
				let camera = match self.state.cameras.get(&view.camera_id) {
					Some(camera) => camera,
					None => continue,
				};
				let node_id = match &camera.node_id {
					Some(node_id) => node_id,
					None => continue,
				};
				let node = match self.state.nodes.get(node_id) {
					Some(node) => node,
					None => continue,
				};
				let scene_id = match node.scene_id {
					Some(id) => id,
					None => continue,
				};
				render_args.views.push(View {
					camview: view.clone(),
					scene_id: scene_id,
				});
			}
		}

		for (_, buffer) in &mut self.gui_buffers {
			buffer.vertices_buffer.flush(&mut self.hardware);
			buffer.index_buffer.flush(&mut self.hardware);
			buffer.color_buffer.flush(&mut self.hardware);
		}
	}

    fn get_window_render_args(&self, window_id: ArenaId<Window>) -> Option<&UIRenderArgs> {
		let window = match self.state.windows.get(&window_id) {
			Some(window) => window,
			None => return None,
		};

		let ui_id = match &window.ui {
			Some(ui_id) => ui_id,
			None => return None,
		};

		self.ui_render_args.get(&ui_id)
	}

	fn get_camera_draw_calls(&self, camera_id: ArenaId<Camera>) -> Option<&Vec<DrawCall>> {
		let camera = self.state.cameras.get(&camera_id)?;
		let scene_id = match &camera.node_id {
			Some(node_id) => {
				let node = match self.state.nodes.get(node_id) {
					Some(node) => node,
					None => return None,
				};
				match node.scene_id {
					Some(id) => id,
					None => return None,
				}
			}
			None => return None,
		};
		self.scene_draw_calls.get(&scene_id)
	}

    fn update_windows(&mut self) {
        for (window_id, window) in self.state.windows.iter_mut() {
			if self.windows.iter().any(|w| w.window_id == window_id) {
				continue;
			}
			crate::log1!("Creating window: {:?}", window_id);
            let handle = self.hardware.create_window(&window);
			let pipeline = self.hardware.create_pipeline("pipeline", handle);
			self.windows.push(WindowContext {
				window_id,
				window: handle,
				pipeline,
			});
        }

        /*for (window_id, _) in self.prev_state.windows.iter() {
            if !self.state.windows.contains(&window_id) {
                //self.hardware.destroy_window(window_id);
            }
        }*/
    }


    pub fn on_mouse_input(&mut self, window: WindowHandle, event: MouseEvent) {
		let window_ctx = match self.windows.iter().find(|w| w.window == window) {
			Some(w) => w,
			None => return,
		};
		self.app.on_mouse_input(window_ctx.window_id, event, &mut self.state);
    }

	pub fn on_keyboard_input(&mut self, window: WindowHandle, key: KeyboardKey, action: KeyAction) {
		let window_ctx = match self.windows.iter().find(|w| w.window == window) {
			Some(w) => w,
			None => return,
		};
		self.app.on_keyboard_input(window_ctx.window_id, key, action, &mut self.state);
    }

	pub fn tick_headless(&mut self, dt: f32) {
		self.process_nodes();
		self.app.on_process(&mut self.state, dt);
	}

    pub fn render(&mut self, dt: f32) {
		let fps = (1.0 / dt) as u32;
		if (fps as i32 - self.fps as i32).abs() > 10 {
			crate::log1!("FPS: {}", fps);
		}
		self.fps = fps;

		if crate::debug_level() >= 3 {
			let frame_start = Instant::now();
			let timer = Instant::now();
			self.process_materials();
			let materials_time = timer.elapsed();
			let timer = Instant::now();
			self.process_textures();
			let textures_time = timer.elapsed();
			let timer = Instant::now();
			self.process_nodes();
			let nodes_time = timer.elapsed();
			let timer = Instant::now();
			self.process_meshes();
			let meshes_time = timer.elapsed();
			let timer = Instant::now();
			self.process_cameras();
			let cameras_time = timer.elapsed();
			let timer = Instant::now();
			self.process_point_lights();
			let lights_time = timer.elapsed();
			let timer = Instant::now();
			self.process_ui();
			let ui_time = timer.elapsed();
			let timer = Instant::now();
			self.update_windows();
			let windows_time = timer.elapsed();
			let timer = Instant::now();
			self.app.on_process(&mut self.state, dt);
			let app_time = timer.elapsed();
			let total_time = frame_start.elapsed();

			crate::log3!(
				"Frame timings: materials={:?}, textures={:?}, nodes={:?}, meshes={:?}, cameras={:?}, lights={:?}, ui={:?}, windows={:?}, app={:?}, total={:?}",
				materials_time,
				textures_time,
				nodes_time,
				meshes_time,
				cameras_time,
				lights_time,
				ui_time,
				windows_time,
				app_time,
				total_time
			);
		} else {
			self.process_materials();
			self.process_textures();
			self.process_nodes();
			self.process_meshes();
			self.process_cameras();
			self.process_point_lights();
			self.process_ui();
			self.update_windows();
			self.app.on_process(&mut self.state, dt);
		}
		if let Some((window_id, path)) = self.state.screenshot_request.take() {
			let ctx = match self.windows.iter().find(|w| w.window_id == window_id) {
				Some(ctx) => ctx,
				None => {
					log::error!("Window context not found for screenshot: {:?}", window_id);
					return;
				}
			};
			self.hardware.save_screenshot(ctx.window, &path);
		}
        for (window_id, _) in &self.state.windows {
			let ctx = match self.windows.iter().find(|w| w.window_id == window_id) {
				Some(ctx) => ctx,
				None => {
					log::error!("Window context not found: {:?}", window_id);
					continue;
				}
			};
            let mut encoder = RenderEncoder::new();
            let args = match self.get_window_render_args(window_id) {
                Some(a) => a,
                None => {
                    panic!("Window render args not found");
                }
            };

            let pass = encoder.begin_render_pass();
			pass.set_pipeline(ctx.pipeline);
            for v in &args.views {
                let camera_buffer = match self.camera_buffers.get(&v.camview.camera_id) {
                    Some(b) => b,
                    None => {
                        panic!("Camera buffer not found");
                    }
                };

                let calls = match self.get_camera_draw_calls(v.camview.camera_id) {
                    Some(c) => c,
                    None => {
                        //panic!("Draw calls not found");
						continue;
                    }
                };

                let instance_buffer = match self.scene_instance_buffers.get(&v.scene_id) {
                    Some(b) => b,
                    None => {
                        panic!("Instance buffer not found");
                    }
                };

                let point_light_buffer = match self.point_light_buffers.get(&v.scene_id) {
                    Some(b) => b,
                    None => {
						&self.default_point_lights
					}
                };
                
				pass.bind_buffer(0, camera_buffer.handle);
                pass.bind_buffer(1, point_light_buffer.handle);

                for call in calls {
					let mut base_color_texture = self.default_texture;
					let mut metallic_roughness_texture = self.default_texture;
					let mut normal_texture = self.default_texture;
					let mut occlusion_texture = self.default_texture;
					let mut emissive_texture = self.default_texture;
					let mut material = self.default_material;
					
					if let Some(material_id) = call.material {
						if let Some(material_buffer) = self.state.materials.get(&material_id) {
							if let Some(base_color_texture_id) = material_buffer.base_color_texture {
								if let Some(t) = self.textures.get(&base_color_texture_id) {
									base_color_texture = *t;
								}
							}
							if let Some(metallic_roughness_texture_id) = material_buffer.metallic_roughness_texture {
								if let Some(t) = self.textures.get(&metallic_roughness_texture_id) {
									metallic_roughness_texture = *t;
								}
							}
							if let Some(normal_texture_id) = material_buffer.normal_texture {
								if let Some(t) = self.textures.get(&normal_texture_id) {
									normal_texture = *t;
								}
							}
							if let Some(occlusion_texture_id) = material_buffer.occlusion_texture {
								if let Some(t) = self.textures.get(&occlusion_texture_id) {
									occlusion_texture = *t;
								}
							}
							if let Some(emissive_texture_id) = material_buffer.emissive_texture {
								if let Some(t) = self.textures.get(&emissive_texture_id) {
									emissive_texture = *t;
								}
							}
						}

						if let Some(m) = self.materials.get(&material_id) {
							material = *m;
						}
					}

					pass.bind_texture(2, base_color_texture);
					pass.bind_texture(3, metallic_roughness_texture);
					pass.bind_texture(4, normal_texture);
					pass.bind_texture(5, occlusion_texture);
					pass.bind_texture(6, emissive_texture);
					pass.bind_buffer(7, material);
                    pass.set_vertex_buffer(0, self.vertices_buffer.slice(call.vertices.clone()));
                    pass.set_vertex_buffer(1, instance_buffer.full());
                    pass.set_vertex_buffer(2, self.normal_buffer.slice(call.normals.clone()));
                    pass.set_vertex_buffer(3, self.tex_coords_buffer.slice(call.tex_coords.clone()));
                    pass.set_index_buffer(self.index_buffer.slice(call.indices.clone()));
                    let indices = call.indices.clone();
                    let instances = call.instances.clone();
                    pass.draw_indexed(call.indices_range.clone(), instances.start as u32..instances.end as u32);
                }
            }
            self.hardware.render(encoder, ctx.window);
		}
	}
}
