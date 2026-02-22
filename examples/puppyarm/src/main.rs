use std::path::PathBuf;

use pge::*;

#[derive(Default)]
struct PuppyArmExample {
    editor: pge::editor::EditorPlugin,
}

impl pge::App for PuppyArmExample {
    fn on_create(&mut self, state: &mut pge::State) {
        let urdf_path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/puppyarm/puppyarm.urdf");
        self.editor.settings.add_light = false;
        self.editor.set_inspect_path(urdf_path);
        self.editor.on_create(state);

        let scene_ids: Vec<_> = state.scenes.iter().map(|(scene_id, _)| scene_id).collect();
        for scene_id in scene_ids {
            let mut light_node = Node::new();
            light_node.name = Some("PuppyArmLight".to_string());
            light_node.set_translation(10.0, 12.0, 10.0);
            light_node.parent = NodeParent::Scene(scene_id);
            let light_node_id = state.nodes.insert(light_node);

            let mut light = PointLight::new();
            light.intensity = 6.0;
            light.node_id = Some(light_node_id);
            state.point_lights.insert(light);
        }
    }

    fn on_process(&mut self, state: &mut pge::State, _dt: f32) {
        self.editor.on_process(state, _dt);
    }

    fn on_mouse_input(
        &mut self,
        _window_id: ArenaId<Window>,
        event: MouseEvent,
        _state: &mut pge::State,
    ) {
        self.editor.on_mouse_input(_window_id, event.clone());
    }

    fn on_keyboard_input(
        &mut self,
        window_id: ArenaId<Window>,
        key: KeyboardKey,
        action: KeyAction,
        _state: &mut pge::State,
    ) {
        self.editor.on_keyboard_input(window_id, key, action);
    }
}

fn main() {
    pge::init_logging();
    pge::run(PuppyArmExample::default()).unwrap();
}
