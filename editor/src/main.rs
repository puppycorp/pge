use args::Command;
use clap::Parser;
use pge::editor::with_editor;
use pge::App;

mod args;

#[derive(Default)]
struct EmptyApp;

impl App for EmptyApp {}

fn main() {
	pge::init_logging();

	let mut app = with_editor(EmptyApp::default());
	app.editor_mut().settings.add_light = true;

	let args = args::Args::parse();

	if let Some(command) = args.command {
		match command {
			Command::Inspect { path } => {
				app.editor_mut().set_inspect_path(path);
			}
		}
	}

	pge::run(app).unwrap();
}
