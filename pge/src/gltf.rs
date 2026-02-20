use std::collections::HashMap;
use std::path::Path;
use glam::Quat;
use glam::Vec3;
use gltf::animation::util::MorphTargetWeights;
use gltf::animation::util::ReadOutputs;
use gltf::animation::util::Rotations;
use gltf::buffer::Data;
use gltf::image::Source;
use crate::state::State;
use crate::Animation;
use crate::AnimationOutput;
use crate::ArenaId;
use crate::Material;
use crate::Mesh;
use crate::Model3D;
use crate::Node;
use crate::NodeParent;
use crate::Primitive;
use crate::PrimitiveTopology;
use crate::Scene;
use crate::AnimationChannel;
use crate::AnimationSampler;
use crate::AnimationTarget;
use crate::AnimationTargetPath;
use crate::Interpolation;
use crate::Texture;
use crate::TextureSource;

struct ParserState {
	node_map: HashMap<usize, ArenaId<Node>>,
	texture_map: HashMap<usize, ArenaId<Texture>>,
	material_map: HashMap<usize, ArenaId<Material>>,
}

impl ParserState {
	fn new() -> Self {
		ParserState {
			node_map: HashMap::new(),
			texture_map: HashMap::new(),
			material_map: HashMap::new(),
		}
	}
}

pub fn load_node(n: &gltf::Node, buffers: &[Data], state: &mut State, parser_state: &mut ParserState, parent: NodeParent) {
	crate::log2!("Loading node: {}", n.name().unwrap_or("Unnamed"));

	let mut node = Node {
		name: Some(n.name().unwrap_or_default().to_string()),
		parent,
		..Default::default()
	};

	// Set the node's transform
	let (translation, rotation, scale) = n.transform().decomposed();
	node.translation = translation.into();
	node.rotation = Quat::from_array(rotation);
	node.scale = scale.into();

	match n.mesh() {
		Some(gltf_mesh) => {
			crate::log2!("Mesh: {}", gltf_mesh.name().unwrap_or("Unnamed"));
			let mut mesh = Mesh::new();
			for p in gltf_mesh.primitives() {
				let mut primitive = Primitive::new(PrimitiveTopology::from_mode(p.mode()));

				crate::log2!("- Primitive #{}", p.index());

				let reader = p.reader(|buffer| {
					let buffer_data = &buffers[buffer.index()];
					Some(&buffer_data.0[..])
				});
				if let Some(iter) = reader.read_positions() {
					for vertex_position in iter {
						primitive.vertices.push([vertex_position[0], vertex_position[1], vertex_position[2]]);
					}
				} else {
					log::warn!("Primitive #{} is missing position data", p.index());
				}

				if let Some(iter) = reader.read_indices() {
					for index in iter.into_u32() {
						primitive.indices.push(index as u16);
					}
				} else {
					log::warn!("Primitive #{} is missing index data", p.index());
				}

				if let Some(iter) = reader.read_normals() {
					for normal in iter {
						primitive.normals.push([normal[0], normal[1], normal[2]]);
					}
				} else {
					log::warn!("Primitive #{} is missing normal data", p.index());
				}

				if let Some(iter) = reader.read_tex_coords(0) {
					for tex_coord in iter.into_f32() {
						primitive.tex_coords.push([tex_coord[0], tex_coord[1]]);
					}
				} else {
					log::warn!("Primitive #{} is missing texture coordinate data", p.index());
				}

				if reader.read_colors(0).is_none() {
					log::warn!("Primitive #{} is missing color data", p.index());
				}

				if reader.read_tangents().is_none() {
					log::warn!("Primitive #{} is missing tangent data", p.index());
				}

				if let Some(material_index) = p.material().index() {
					if let Some(material_id) = parser_state.material_map.get(&material_index) {
						primitive.material = Some(*material_id);
					}
				}

				mesh.primitives.push(primitive);
			}

			let mesh_id = state.meshes.insert(mesh);
			node.mesh = Some(mesh_id);
		},
		None => {
			crate::log2!("Node does not contain a mesh");
		}
	}
	
	let node_id = state.nodes.insert(node);
	parser_state.node_map.insert(n.index(), node_id); // Store the mapping

	for child in n.children() {
		load_node(&child, buffers, state, parser_state, NodeParent::Node(node_id));
	}
}

pub fn load_scene(s: &gltf::Scene, buffers: &[Data], state: &mut State, parser_state: &mut ParserState) -> ArenaId<Scene> {
	let scene = Scene {
		name: Some(s.name().unwrap_or_default().to_string()),
		..Default::default()
	};

	let scene_id = state.scenes.insert(scene);
	let parent = NodeParent::Scene(scene_id);

	for node in s.nodes() {
		load_node(&node, buffers, state, parser_state, parent);
	}

	scene_id
}

pub fn load_animation(anim: &gltf::Animation, buffers: &[Data], state: &mut State, parser_state: &mut ParserState) {
	crate::log2!("Loading animation: {}", anim.name().unwrap_or("Unnamed"));

	let mut animation = Animation::new();

	for channel in anim.channels() {
		let target_node = channel.target().node();


		let target_node_id = match parser_state.node_map.get(&target_node.index()) { // Use parser_state
			Some(id) => id,
			None => {
				log::warn!("Animation target node not found: {}", target_node.name().unwrap_or_default().to_string());
				continue;
			}
		};

		let target_path = match channel.target().property() {
			gltf::animation::Property::Translation => AnimationTargetPath::Translation,
			gltf::animation::Property::Rotation => AnimationTargetPath::Rotation,
			gltf::animation::Property::Scale => AnimationTargetPath::Scale,
			gltf::animation::Property::MorphTargetWeights => AnimationTargetPath::Weights,
		};

		let target = AnimationTarget {
			node_id: target_node_id.clone(),
			path: target_path,
		};

		let sampler = channel.sampler();
		let sampler_index = animation.samplers.len();

		let interpolation = match sampler.interpolation() {
			gltf::animation::Interpolation::Linear => Interpolation::Linear,
			gltf::animation::Interpolation::Step => Interpolation::Stepm,
			gltf::animation::Interpolation::CubicSpline => Interpolation::Cubicspline,
		};

		let reader = channel.reader(|buffer| Some(&buffers[buffer.index()]));

		let input: Vec<f32> = reader.read_inputs().unwrap().collect();
		let output = match reader.read_outputs().unwrap() {
			ReadOutputs::Translations(output) => {
				//let m = output.map(|o| Vec3::from(o));
				//let m = m.collect::<Vec<Vec3>>();
				AnimationOutput::Translation(output.map(|p| Vec3::from(p)).collect())
			},
			ReadOutputs::Rotations(output) => {
				AnimationOutput::Rotation(match output {
					Rotations::I8(d) => d.map(|o| Quat::from_array([
						o[0] as f32 / 127.0,
						o[1] as f32 / 127.0,
						o[2] as f32 / 127.0,
						o[3] as f32 / 127.0
					])).collect(),
					Rotations::U8(d) => d.map(|o| Quat::from_array([
						o[0] as f32 / 255.0,
						o[1] as f32 / 255.0,
						o[2] as f32 / 255.0,
						o[3] as f32 / 255.0
					])).collect(),
					Rotations::I16(d) => d.map(|o| Quat::from_array([
						o[0] as f32 / 32767.0,
						o[1] as f32 / 32767.0,
						o[2] as f32 / 32767.0,
						o[3] as f32 / 32767.0
					])).collect(),
					Rotations::U16(d) => d.map(|o| Quat::from_array([
						o[0] as f32 / 65535.0,
						o[1] as f32 / 65535.0,
						o[2] as f32 / 65535.0,
						o[3] as f32 / 65535.0
					])).collect(),
					Rotations::F32(d) => d.map(|o| Quat::from_array(o)).collect(),
				})
			},
			ReadOutputs::Scales(output) => {
				AnimationOutput::Scale(output.map(|p| Vec3::from(p)).collect())
			},
			ReadOutputs::MorphTargetWeights(output) => {
				AnimationOutput::MorphWeights(match output {
					MorphTargetWeights::I8(d) => crate::types::WorphTargetWeight::I8(d.collect()),
					MorphTargetWeights::U8(d) => crate::types::WorphTargetWeight::U8(d.collect()),
					MorphTargetWeights::I16(d) => crate::types::WorphTargetWeight::I16(d.collect()),
					MorphTargetWeights::U16(d) => crate::types::WorphTargetWeight::U16(d.collect()),
					MorphTargetWeights::F32(d) => crate::types::WorphTargetWeight::F32(d.collect()),
				})
			}
		};

		animation.samplers.push(AnimationSampler {
			input,
			output,
			interpolation,
		});

		animation.channels.push(AnimationChannel {
			sampler: sampler_index,
			target,
		});
	}

	state.animations.insert(animation);
}

pub fn load_gltf<P: AsRef<Path>>(p: P, state: &mut State) -> Model3D {
	let mut model = Model3D::default();

	let mut parser_state = ParserState::new();

	let p = p.as_ref();
	crate::log2!("loading {:?}", p.to_string_lossy());

	let (document, buffers, images) = match gltf::import(p) {
		Ok(r) => r,
		Err(e) => {
			log::error!("Failed to load gltf file: {:?}", e);
			return model;
		},
	};

	for image in images {
		crate::log2!("Image: {}x{}", image.width, image.height);
	}

	for animation in document.animations() {
		load_animation(&animation, &buffers, state, &mut parser_state);
	}

	for texture in document.textures() {
		crate::log2!("Texture: {}", texture.name().unwrap_or("Unnamed"));
		let s = texture.source();
		let source = s.source();
		let texture_id = match source {
			Source::View { view, mime_type } => {
				crate::log2!("mime_type: {}", mime_type);
				// Retrieve the buffer associated with the view
				let buffer = view.buffer();
				let buffer_index = buffer.index();
			
				// Access the buffer data using the buffer index
				let buffer_data = &buffers[buffer_index].0;
			
				// Calculate the start and end positions using byte_offset and byte_length
				let start = view.offset() as usize;
				let length = view.length() as usize;
				let end = start + length;
			
				// Extract the image data slice from the buffer
				let image_data = &buffer_data[start..end];
				let image = image::load_from_memory_with_format(image_data, image::ImageFormat::from_mime_type(mime_type).unwrap()).unwrap();
				let image = image.to_rgba8();
				let dim = image.dimensions();
				let data = image.into_raw();
				state.textures.insert(Texture {
					name: texture.name().unwrap_or_default().to_string(),
					source: TextureSource::Buffer { data, width: dim.0 as u32, height: dim.1 as u32 },
					..Default::default()
				})
			},
			Source::Uri { uri, mime_type } => {
				crate::log2!("uri: {}", uri);
				crate::log2!("mime_type: {}", mime_type.unwrap_or("None"));
				todo!()
			}
		};

		parser_state.texture_map.insert(texture.index(), texture_id);
	}

	for gltf_material in document.materials() {
		crate::log2!("Material: {}", gltf_material.name().unwrap_or("Unnamed"));
		let material_index = match gltf_material.index() {
			Some(index) => index,
			None => {
				log::warn!("Material has no index");
				continue;
			}
		};
		let pbr = gltf_material.pbr_metallic_roughness();

		let mut material = Material {
			name: gltf_material.name().map(|p| p.to_string()),
			..Default::default()
		};

		if let Some(base_color_texture) = pbr.base_color_texture() {
			crate::log2!("base_color_texture: {}", base_color_texture.texture().index());
			let pbr_texture_id = match parser_state.texture_map.get(&base_color_texture.texture().index()) {
				Some(id) => id,
				None => {
					continue;
				}
			};

			material.base_color_texture = Some(*pbr_texture_id);
		}
		material.base_color_factor = pbr.base_color_factor();

		if let Some(metallic_roughness_texture) = pbr.metallic_roughness_texture() {
			crate::log2!("metallic_roughness_texture: {}", metallic_roughness_texture.texture().index());
			let texture_id = match parser_state.texture_map.get(&metallic_roughness_texture.texture().index()) {
				Some(id) => id,
				None => {
					continue;
				}
			};
			material.metallic_roughness_texture = Some(*texture_id);
		}
		material.metallic_factor = pbr.metallic_factor();
		material.roughness_factor = pbr.roughness_factor();

		if let Some(normal_texture) = gltf_material.normal_texture() {
			crate::log2!("normal_texture: {}", normal_texture.texture().index());
			// normal_texture.tex_coord() TODO how to handle ?
			let texture_id = match parser_state.texture_map.get(&normal_texture.texture().index()) {
				Some(id) => id,
				None => {
					continue;
				}
			};
			material.normal_texture = Some(*texture_id);
			material.normal_texture_scale = normal_texture.scale();
		}

		if let Some(occlusion_texture) = gltf_material.occlusion_texture() {
			crate::log2!("occlusion_texture: {}", occlusion_texture.texture().index());
			let texture_id = match parser_state.texture_map.get(&occlusion_texture.texture().index()) {
				Some(id) => id,
				None => {
					continue;
				}
			};
			material.occlusion_texture = Some(*texture_id);
			material.occlusion_strength = occlusion_texture.strength();
		}

		if let Some(emissive_texture) = gltf_material.emissive_texture() {
			crate::log2!("emissive_texture: {}", emissive_texture.texture().index());
			let texture_id = match parser_state.texture_map.get(&emissive_texture.texture().index()) {
				Some(id) => id,
				None => {
					continue;
				}
			};
			material.emissive_texture = Some(*texture_id);
		}
		material.emissive_factor = gltf_material.emissive_factor();
		let material_id = state.materials.insert(material);
		parser_state.material_map.insert(material_index, material_id);
	}

	for skin in document.skins() {
		crate::log2!("Skin: {}", skin.name().unwrap_or("Unnamed"));
	}

	if let Some(s) = document.default_scene() {
		crate::log2!("Default scene: {}", s.name().unwrap_or("Unnamed"));
		let scene_id = load_scene(&s, &buffers, state, &mut parser_state);
		model.default_scene = Some(scene_id);
	}

	for s in document.scenes() {
		crate::log2!("Scene: {}", s.name().unwrap_or("Unnamed"));
		let scene_id = load_scene(&s, &buffers, state, &mut parser_state);
		model.scenes.push(scene_id);
	}

	model
}


#[cfg(test)]
mod tests {
	use crate::init_logging;
	use super::*;

	#[test]
	fn test_load_gltf() {
		let mut state = State::default();
		load_gltf("./assets/orkki.glb", &mut state);
		state.print_state();

	crate::log4!("materials: {:#?}", state.materials);
	}
}
