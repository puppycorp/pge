use std::collections::HashMap;
use std::path::Path;
use crate::load_gltf;
use crate::arena::*;
use crate::types::*;
use crate::utility::get_scene_bounding_box;
use crate::GUIElement;
use crate::Window;

#[derive(Debug, Clone, Default)]
pub struct State {
    pub scenes: Arena<Scene>,
    pub meshes: Arena<Mesh>,
    pub nodes: Arena<Node>,
    pub cameras: Arena<Camera>,
    pub windows: Arena<Window>,
    pub guis: Arena<GUIElement>,
    pub point_lights: Arena<PointLight>,
    pub textures: Arena<Texture>,
    pub raycasts: Arena<RayCast>,
    pub models: Arena<Model3D>,
    pub animations: Arena<Animation>,
    pub materials: Arena<Material>,
    pub joints: Arena<Joint>,
    pub keyboard: Option<Keyboard>,
}

impl State {
    pub fn load_3d_model<P: AsRef<Path> + Clone>(&mut self, path: P) -> ArenaId<Model3D> {
        let model = load_gltf(path, self);
        self.models.insert(model)
    }

    /// Deep clones node and it's children
    pub fn clone_node(&mut self, node_id: ArenaId<Node>) -> ArenaId<Node> {
        let node = self.nodes.get(&node_id).expect("Node not found");
        let mut new_node = node.clone();
        new_node.parent = NodeParent::Orphan;
        let new_node_id = self.nodes.insert(new_node);
        let mut stack = vec![(node_id, new_node_id.clone())];
        while let Some((orig_id, new_parent_id)) = stack.pop() {
            let children: Vec<_> = self.nodes.iter()
                .filter_map(|(id, n)| if n.parent == NodeParent::Node(orig_id) { Some(id) } else { None })
                .collect();
            for child_id in children {
                let child = self.nodes.get(&child_id).expect("Child node not found");
                let mut new_child = child.clone();
                new_child.parent = NodeParent::Node(new_parent_id);
                let new_child_id = self.nodes.insert(new_child);
                stack.push((child_id, new_child_id));
            }
        }
    
        new_node_id
    }

    pub fn mem_size(&self) -> usize {
        self.scenes.mem_size()
            + self.meshes.mem_size()
            + self.nodes.mem_size()
            + self.cameras.mem_size()
            + self.windows.mem_size()
            + self.guis.mem_size()
            + self.point_lights.mem_size()
            + self.textures.mem_size()
            + self.raycasts.mem_size()
            + self.joints.mem_size()
    }

    pub fn print_state(&self) {
        crate::log2!("scene count: {:?}", self.scenes.len());
        crate::log2!("mesh count: {:?}", self.meshes.len());
        crate::log2!("node count: {:?}", self.nodes.len());
        crate::log2!("camera count: {:?}", self.cameras.len());
        crate::log2!("window count: {:?}", self.windows.len());
        crate::log2!("gui count: {:?}", self.guis.len());
        crate::log2!("point light count: {:?}", self.point_lights.len());
        crate::log2!("texture count: {:?}", self.textures.len());
        crate::log2!("raycast count: {:?}", self.raycasts.len());
        crate::log2!("joint count: {:?}", self.joints.len());
    }

	pub fn get_scene_bounding_box(&self, scene_id: ArenaId<Scene>) -> AABB {
		get_scene_bounding_box(scene_id, self)
	}
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn test_load_3d_model() {
        let mut state = State::default();
        let model_id = state.load_3d_model("test_model.gltf");
        assert!(state.models.contains(&model_id));
    }

    #[test]
    fn test_clone_node() {
        let mut state = State::default();
        let original_node = Node::new();
        let original_id = state.nodes.insert(original_node);
        
        let cloned_id = state.clone_node(original_id);
        
        assert_ne!(original_id, cloned_id);
        assert!(state.nodes.contains(&cloned_id));
    }

    #[test]
    fn test_mem_size() {
        let state = State::default();
        assert!(state.mem_size() > 0);
    }
}
