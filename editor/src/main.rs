use args::Command;
use clap::Parser;
use pge::*;

mod args;

#[derive(Default)]
struct EditorCliApp {
    editor: pge::editor::EditorPlugin,
}

impl App for EditorCliApp {
    fn on_create(&mut self, state: &mut State) {
        self.editor.on_create(state);
    }

    fn on_process(&mut self, state: &mut State, dt: f32) {
        self.editor.on_process(state, dt);
    }

    fn on_mouse_input(
        &mut self,
        window_id: ArenaId<Window>,
        event: MouseEvent,
        _state: &mut State,
    ) {
        self.editor.on_mouse_input(window_id, event);
    }

    fn on_keyboard_input(
        &mut self,
        window_id: ArenaId<Window>,
        key: KeyboardKey,
        action: KeyAction,
        _state: &mut State,
    ) {
        self.editor.on_keyboard_input(window_id, key, action);
    }
}

fn main() {
    pge::init_logging();

    let mut app = EditorCliApp::default();
    app.editor.settings.add_light = true;

    let args = args::Args::parse();

    if let Some(command) = args.command {
        match command {
            Command::Inspect { path } => {
                app.editor.set_inspect_path(path);
            }
        }
    }

    pge::run(app).unwrap();
}
