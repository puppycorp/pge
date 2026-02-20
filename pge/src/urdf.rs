use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::path::{Path, PathBuf};

use urdf_rs::{read_file, Geometry, JointType as UrdfJointType, Pose, Vec3};

use crate::types::*;
use crate::{ArenaId, State};
use crate::cube;

fn resolve_mesh_path(urdf_path: &Path, mesh_filename: &str) -> Option<PathBuf> {
	let mesh_path = if let Some(stripped) = mesh_filename.strip_prefix("package://") {
		let mut parts = stripped.splitn(2, '/');
		let _pkg = parts.next();
		PathBuf::from(parts.next().unwrap_or(stripped))
	} else if let Some(stripped) = mesh_filename.strip_prefix("file://") {
		PathBuf::from(stripped)
	} else {
		PathBuf::from(mesh_filename)
	};
	if mesh_path.is_absolute() {
		return mesh_path.exists().then(|| mesh_path);
	}

	let base_dir = urdf_path.parent().unwrap_or_else(|| Path::new("."));
	let candidate = base_dir.join(&mesh_path);
	if candidate.exists() {
		return Some(candidate);
	}

	if let Some(file_name) = mesh_path.file_name() {
		let fallback = base_dir.join("meshes").join(file_name);
		if fallback.exists() {
			return Some(fallback);
		}
	}

	None
}

fn load_stl_mesh(path: &Path) -> Mesh {
	let mut file = File::open(path).unwrap_or_else(|e| {
		panic!("Failed to open STL mesh {}: {}", path.display(), e);
	});
	let stl = stl_io::read_stl(&mut file).unwrap_or_else(|e| {
		panic!("Failed to read STL mesh {}: {}", path.display(), e);
	});

	let mut prim = Primitive::new(crate::PrimitiveTopology::TriangleList);
	prim.vertices = stl
		.vertices
		.iter()
		.map(|v| [v[0], v[1], v[2]])
		.collect();
	prim.indices = stl
		.faces
		.iter()
		.flat_map(|f| f.vertices.iter().copied())
		.map(|i| i as u16)
		.collect();

	let mut normals = vec![[0.0, 0.0, 0.0]; stl.vertices.len()];
	for face in &stl.faces {
		let ia = face.vertices[0] as usize;
		let ib = face.vertices[1] as usize;
		let ic = face.vertices[2] as usize;
		let a = glam::Vec3::from_array(stl.vertices[ia].into());
		let b = glam::Vec3::from_array(stl.vertices[ib].into());
		let c = glam::Vec3::from_array(stl.vertices[ic].into());
		let n = (b - a).cross(c - a);
		if n.length_squared() > 0.0 {
			let n = n.normalize();
			for i in [ia, ib, ic] {
				normals[i][0] += n.x;
				normals[i][1] += n.y;
				normals[i][2] += n.z;
			}
		}
	}
	for n in normals.iter_mut() {
		let v = glam::Vec3::new(n[0], n[1], n[2]);
		if v.length_squared() > 0.0 {
			let v = v.normalize();
			n[0] = v.x;
			n[1] = v.y;
			n[2] = v.z;
		} else {
			n[1] = 1.0;
		}
	}
	prim.normals = normals;

	let mut mesh = Mesh::new();
	mesh.name = Some(path.file_name().unwrap_or_default().to_string_lossy().to_string());
	mesh.primitives.push(prim);
	mesh
}

fn vec3_from_urdf(v: &Vec3) -> glam::Vec3 {
	glam::Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

fn quat_from_rpy(rpy: &Vec3) -> glam::Quat {
	glam::Quat::from_euler(
		glam::EulerRot::ZYX,
		rpy[2] as f32,
		rpy[1] as f32,
		rpy[0] as f32,
	)
}

fn origin_offsets(origin: &Pose) -> (glam::Vec3, glam::Quat) {
	(vec3_from_urdf(&origin.xyz), quat_from_rpy(&origin.rpy))
}

fn collision_shape_from_geometry(geometry: &Geometry, origin: &Pose) -> Option<CollisionShape> {
	let (position_offset, rotation_offset) = origin_offsets(origin);
	let shape = match geometry {
		Geometry::Box { size } => {
			let extents = vec3_from_urdf(size) * 0.5;
			ColliderType::Cuboid { size: extents }
		}
		Geometry::Sphere { radius } => ColliderType::Sphere {
			radius: *radius as f32,
		},
		Geometry::Cylinder { radius, length } => ColliderType::Cylinder {
			radius: *radius as f32,
			half_height: (*length as f32) * 0.5,
		},
		Geometry::Capsule { radius, length } => ColliderType::Capsule {
			radius: *radius as f32,
			half_height: (*length as f32) * 0.5,
		},
		Geometry::Mesh { .. } => {
			return None;
		}
	};

	Some(CollisionShape {
		shape,
		position_offset,
		rotation_offset,
	})
}

fn joint_axis_or_default(joint_axis: &Vec3) -> glam::Vec3 {
	let axis = vec3_from_urdf(joint_axis);
	if axis.length_squared() > 0.0 {
		axis.normalize()
	} else {
		glam::Vec3::X
	}
}

pub fn load_urdf<P: AsRef<Path>>(p: P, state: &mut State) -> ArenaId<Scene> {
	let urdf_path = p.as_ref();
	let robot = read_file(urdf_path).expect("Failed to read URDF");
	let scene_id = state.scenes.insert(Scene::new());

	let unit_cube = state.meshes.insert(cube(0.5));
	let mut mesh_by_path = HashMap::new();
	let mut link_nodes = HashMap::new();
	for link in &robot.links {
		let mut node = Node::new();
		node.name = Some(link.name.clone());
		node.parent = NodeParent::Orphan;
		node.physics.typ = PhycisObjectType::Static;

		let mass = link.inertial.mass.value as f32;
		if mass > 0.0 {
			node.physics.typ = PhycisObjectType::Dynamic;
			node.physics.mass = mass;
		}

		if let Some(collision) = link.collision.first() {
			node.collision_shape =
				collision_shape_from_geometry(&collision.geometry, &collision.origin);
		} else if let Some(visual) = link.visual.first() {
			node.collision_shape =
				collision_shape_from_geometry(&visual.geometry, &visual.origin);
		}

		let node_id = state.nodes.insert(node);
		link_nodes.insert(link.name.clone(), node_id);

		let mut mesh_id: Option<ArenaId<Mesh>> = None;
		let mut mesh_scale: Option<glam::Vec3> = None;
		if let Some(visual) = link.visual.first() {
			if let Geometry::Mesh { filename, scale } = &visual.geometry {
				if let Some(mesh_path) = resolve_mesh_path(urdf_path, filename) {
					let key = mesh_path.to_string_lossy().to_string();
					let id = *mesh_by_path.entry(key).or_insert_with(|| {
						let mesh = load_stl_mesh(&mesh_path);
						state.meshes.insert(mesh)
					});
					mesh_id = Some(id);
					if let Some(scale) = scale {
						mesh_scale = Some(vec3_from_urdf(scale));
					}
				}
			}
		}
		if mesh_id.is_none() {
			if let Some(collision) = link.collision.first() {
				if let Geometry::Mesh { filename, scale } = &collision.geometry {
					if let Some(mesh_path) = resolve_mesh_path(urdf_path, filename) {
						let key = mesh_path.to_string_lossy().to_string();
						let id = *mesh_by_path.entry(key).or_insert_with(|| {
							let mesh = load_stl_mesh(&mesh_path);
							state.meshes.insert(mesh)
						});
						mesh_id = Some(id);
						if let Some(scale) = scale {
							mesh_scale = Some(vec3_from_urdf(scale));
						}
					}
				}
			}
		}
		if let Some(mesh_id) = mesh_id {
			let mut visual_node = Node::new();
			visual_node.name = Some(format!("{}_visual", link.name));
			visual_node.parent = NodeParent::Node(node_id);
			if let Some(visual) = link.visual.first() {
				let (translation, rotation) = origin_offsets(&visual.origin);
				visual_node.translation = translation;
				visual_node.rotation = rotation;
			}
			visual_node.mesh = Some(mesh_id);
			if let Some(scale) = mesh_scale {
				visual_node.scale = scale;
			}
			state.nodes.insert(visual_node);
		} else if let Some(shape) = &state.nodes.get(&node_id).unwrap().collision_shape {
			let mut visual_node = Node::new();
			visual_node.name = Some(format!("{}_visual", link.name));
			visual_node.parent = NodeParent::Node(node_id);
			let aabb = shape.aabb(glam::Vec3::ZERO);
			let size = aabb.max - aabb.min;
			visual_node.mesh = Some(unit_cube);
			visual_node.scale = size;
			state.nodes.insert(visual_node);
		} else {
			let mut visual_node = Node::new();
			visual_node.name = Some(format!("{}_visual", link.name));
			visual_node.parent = NodeParent::Node(node_id);
			visual_node.mesh = Some(unit_cube);
			visual_node.scale = glam::Vec3::splat(0.05);
			state.nodes.insert(visual_node);
		}
	}

	let mut child_links = HashSet::new();
	for joint in &robot.joints {
		let parent_id = link_nodes.get(&joint.parent.link).copied();
		let child_id = match link_nodes.get(&joint.child.link) {
			Some(id) => *id,
			None => continue,
		};

		child_links.insert(joint.child.link.clone());

		let (translation, rotation) = origin_offsets(&joint.origin);
		if let Some(child_node) = state.nodes.get_mut(&child_id) {
			child_node.translation = translation;
			child_node.rotation = rotation;
			if let Some(parent_id) = parent_id {
				child_node.parent = NodeParent::Node(parent_id);
			} else if joint.parent.link == "world" {
				child_node.parent = NodeParent::Scene(scene_id);
			}
		}

		let parent_id = match parent_id {
			Some(id) => id,
			None => continue,
		};

		let joint_type = match joint.joint_type {
			UrdfJointType::Fixed => JointType::Fixed,
			UrdfJointType::Revolute => JointType::Revolute {
				axis: joint_axis_or_default(&joint.axis.xyz),
				limits: Some((joint.limit.lower as f32, joint.limit.upper as f32)),
			},
			UrdfJointType::Continuous => JointType::Revolute {
				axis: joint_axis_or_default(&joint.axis.xyz),
				limits: None,
			},
			UrdfJointType::Prismatic => JointType::Prismatic {
				axis: joint_axis_or_default(&joint.axis.xyz),
				limits: Some((joint.limit.lower as f32, joint.limit.upper as f32)),
			},
			UrdfJointType::Floating => JointType::Ball,
			UrdfJointType::Planar => JointType::Ball,
			UrdfJointType::Spherical => JointType::Ball,
		};

		let (anchor, _) = origin_offsets(&joint.origin);
		let damping = joint
			.dynamics
			.as_ref()
			.map(|d| d.damping as f32)
			.unwrap_or(0.0);

		let joint = Joint {
			name: Some(joint.name.clone()),
			body_a: parent_id,
			body_b: child_id,
			local_anchor_a: anchor,
			local_anchor_b: glam::Vec3::ZERO,
			joint_type,
			compliance: 0.0,
			damping,
		};
		state.joints.insert(joint);
	}

	for (link_name, node_id) in &link_nodes {
		if !child_links.contains(link_name) {
			if let Some(node) = state.nodes.get_mut(node_id) {
				node.parent = NodeParent::Scene(scene_id);
			}
		}
	}

	scene_id
}
