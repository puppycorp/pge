use std::path::PathBuf;

use pge::*;

#[derive(Default)]
struct PuppyArmExample {
	editor: pge::editor::EditorPlugin,
}

impl pge::App for PuppyArmExample {
	fn on_create(&mut self, state: &mut pge::State) {
		let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.join("../../assets/puppyarm/puppyarm.urdf");
		self.editor.settings.add_light = true;
		self.editor.set_inspect_path(urdf_path);
		self.editor.on_create(state);
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
}

fn main() {
	pge::init_logging();
	pge::run(PuppyArmExample::default()).unwrap();
}
