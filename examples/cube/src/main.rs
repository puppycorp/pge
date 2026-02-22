use pge::*;

#[derive(Default)]
struct JustCube {
    cube_node_id: Option<ArenaId<Node>>,
    angle: f32,
}

impl pge::App for JustCube {
    fn on_create(&mut self, state: &mut pge::State) {
        let mut scene = Scene::new();
        scene.name = Some("Cube Scene".to_string());
        let scene_id = state.scenes.insert(scene);
        let cube_mesh = state.meshes.insert(cube(0.5));

        let floor_material = Material {
            name: Some("Floor Dark Blue".to_string()),
            base_color_factor: [0.05, 0.12, 0.35, 1.0],
            roughness_factor: 0.2,
            ..Default::default()
        };
        let floor_material_id = state.materials.insert(floor_material);
        let mut floor_mesh_data = plane(1.0, 1.0);
        floor_mesh_data.primitives[0].material = Some(floor_material_id);
        let floor_mesh = state.meshes.insert(floor_mesh_data);

        let mut light_node = Node::new();
        light_node.name = Some("Light".to_string());
        light_node.set_translation(0.0, 2.5, 0.0);
        light_node.parent = NodeParent::Scene(scene_id);
        let light_node_id = state.nodes.insert(light_node);
        let mut light = PointLight::new();
        light.color = [1.0, 1.0, 1.0];
        light.intensity = 30.0;
        light.node_id = Some(light_node_id);
        state.point_lights.insert(light);

        let mut cube_node = Node::new();
        cube_node.name = Some("Cube".to_string());
        cube_node.set_translation(0.0, 1.2, 0.0);
        cube_node.mesh = Some(cube_mesh);
        cube_node.parent = NodeParent::Scene(scene_id);
        self.cube_node_id = Some(state.nodes.insert(cube_node));

        let mut floor_node = Node::new();
        floor_node.name = Some("Floor".to_string());
        floor_node.set_translation(0.0, -0.5, 0.0);
        floor_node.scale = Vec3::new(8.0, 1.0, 8.0);
        floor_node.mesh = Some(floor_mesh);
        floor_node.parent = NodeParent::Scene(scene_id);
        state.nodes.insert(floor_node);
    }

    fn on_process(&mut self, state: &mut pge::State, dt: f32) {
        self.angle += dt;
        if let Some(cube_node_id) = self.cube_node_id {
            let cube_node = state.nodes.get_mut(&cube_node_id).unwrap();
            cube_node.rotation = Quat::from_rotation_y(self.angle);
        }
    }
}

fn main() {
    pge::init_logging();
    let mut app = pge::editor::with_editor(JustCube::default());
    app.editor_mut().settings.add_light = false;
    pge::run(app).unwrap()
}
