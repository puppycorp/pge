use crate::types::*;
use crate::Arena;
use crate::ArenaId;
use crate::State;
use glam::*;

pub fn topo_sort_nodes(nodes: &Arena<Node>, sorted_nodes: &mut Vec<ArenaId<Node>>) {
    // Pre-build child lookup map to avoid repeated filtering
    let mut children: std::collections::HashMap<ArenaId<Node>, Vec<ArenaId<Node>>> = std::collections::HashMap::new();
    for (id, node) in nodes.iter() {
        if let NodeParent::Node(parent_id) = node.parent {
            children.entry(parent_id).or_default().push(id);
        }
    }

    // Find root nodes (we don't care about Orphan's)
    let mut stack = Vec::new();
    for (id, node) in nodes.iter() {
        match node.parent {
            NodeParent::Scene(_) => stack.push(id),
            _ => {}
        }
    }

    // Process nodes depth-first
    while let Some(node_id) = stack.pop() {
        sorted_nodes.push(node_id);
        if let Some(child_ids) = children.get(&node_id) {
            stack.extend(child_ids);
        }
    }
}

#[cfg(test)]
mod topo_sort_tests {
    use crate::Arena;
	use super::*;

	#[test]
	pub fn topo_sort_nodes_test() {
		let mut scenes = Arena::new();
		let scene_id = scenes.insert(Scene::default());
		let mut nodes = Arena::new();

		let parent1 = nodes.insert(Node {
			parent: NodeParent::Orphan,
			..Default::default()
		});
		let parent2 = nodes.insert(Node {
			parent: NodeParent::Scene(scene_id),
			..Default::default()
		});
		let parent3 = nodes.insert(Node {
			parent: NodeParent::Scene(scene_id),
			..Default::default()
		});
		let child1 = nodes.insert(Node {
			parent: NodeParent::Node(parent1),
			..Default::default()
		});
		let child2 = nodes.insert(Node {
			parent: NodeParent::Node(parent2),
			..Default::default()
		});
		let child3 = nodes.insert(Node {
			parent: NodeParent::Node(child1),
			..Default::default()
		});
		let child4 = nodes.insert(Node {
			parent: NodeParent::Node(parent3),
			..Default::default()
		});
		let mut sorted_nodes = Vec::new();
		topo_sort_nodes(&nodes, &mut sorted_nodes);
		assert_eq!(sorted_nodes, vec![parent3, child4, parent2, child2]);

	}


}

pub fn get_scene_bounding_box(scene_id: ArenaId<Scene>, state: &State) -> AABB {
    let mut aabb: Option<AABB> = None;
    
    // Find all nodes that belong to this scene
    for (_, node) in state.nodes.iter().filter(|(_, node)| node.scene_id == Some(scene_id)) {
        let mesh_id = match node.mesh {
			Some(id) => id,
			None => continue,
		};
		let mesh = match state.meshes.get(&mesh_id) {
			Some(mesh) => mesh,
			None => continue,
		};

		for primitive in mesh.primitives.iter() {
			for vertice in primitive.vertices.iter() {
				let vertice = (node.global_transform * Vec4::new(vertice[0], vertice[1], vertice[2], 1.0)).xyz();
				match aabb {
					None => aabb = Some(AABB { min: vertice, max: vertice }),
					Some(ref mut aabb) => {
						if aabb.min.x > vertice.x { aabb.min.x = vertice.x; }
						if aabb.min.y > vertice.y { aabb.min.y = vertice.y; }
						if aabb.min.z > vertice.z { aabb.min.z = vertice.z; }
						if aabb.max.x < vertice.x { aabb.max.x = vertice.x; }
						if aabb.max.y < vertice.y { aabb.max.y = vertice.y; }
						if aabb.max.z < vertice.z { aabb.max.z = vertice.z; }
					}
				}
			}
		}
	}
    match aabb.take() {
		Some(aabb) => aabb,
		None => AABB::empty(),
	}
}

#[cfg(test)]
mod get_scene_bounding_box_tests {
    use super::*;
    use crate::CollisionShape;
    use crate::Primitive;
    use crate::PrimitiveTopology;
    use crate::Mesh;

    #[test]
    pub fn get_scene_bounding_box_test() {
        let mut state = State::default();
        
        // Create a scene
        let scene_id = state.scenes.insert(Scene::default());

        // Create a mesh with known vertices
        let mut mesh = Mesh::new();
        let mut primitive = Primitive::new(PrimitiveTopology::TriangleList);
        primitive.vertices = vec![
            [-1.0, -1.0, -1.0], // min point
            [2.0, 3.0, 4.0],    // max point
            [0.0, 0.0, 0.0],    // middle point
        ];
        mesh.primitives.push(primitive);
        let mesh_id = state.meshes.insert(mesh);

        // Create a node with the mesh
        let node = Node {
            mesh: Some(mesh_id),
            scene_id: Some(scene_id),
            global_transform: Mat4::IDENTITY, // No transformation
            ..Default::default()
        };
        state.nodes.insert(node);

        // Get bounding box
        let bbox = get_scene_bounding_box(scene_id, &state);

        // Check the bounds match our vertices
        assert_eq!(bbox.min, Vec3::new(-1.0, -1.0, -1.0));
        assert_eq!(bbox.max, Vec3::new(2.0, 3.0, 4.0));
    }

    #[test]
    pub fn get_scene_bounding_box_with_transform_test() {
        let mut state = State::default();
        let scene_id = state.scenes.insert(Scene::default());

        // Create a mesh with a simple cube
        let mut mesh = Mesh::new();
        let mut primitive = Primitive::new(PrimitiveTopology::TriangleList);
        primitive.vertices = vec![
            [-1.0, -1.0, -1.0],
            [1.0, 1.0, 1.0],
        ];
        mesh.primitives.push(primitive);
        let mesh_id = state.meshes.insert(mesh);

        // Create a node with translation
        let translation = Vec3::new(5.0, 0.0, 0.0); // Move 5 units in x direction
        let node = Node {
            mesh: Some(mesh_id),
            scene_id: Some(scene_id),
            translation,
            global_transform: Mat4::from_translation(translation),
            ..Default::default()
        };
        state.nodes.insert(node);

        let bbox = get_scene_bounding_box(scene_id, &state);

        // Check bounds are translated
        assert_eq!(bbox.min, Vec3::new(4.0, -1.0, -1.0)); // -1 + 5 = 4
        assert_eq!(bbox.max, Vec3::new(6.0, 1.0, 1.0));   // 1 + 5 = 6
    }
}
