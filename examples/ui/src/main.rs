use pge::*;

#[derive(Default)]
struct UiExample;

impl App for UiExample {
	fn on_create(&mut self, state: &mut State) {
		let mut title = text("PGE UI Example");
		title.font_size = 28;
		title.font_color = [1.0, 1.0, 1.0, 1.0];
		let title = title.margin(0.05);

		let mut subtitle = text("Layout + text smoke test");
		subtitle.font_size = 16;
		subtitle.font_color = [0.9, 0.9, 0.9, 1.0];
		let subtitle = subtitle.margin(0.05);

		let ui = stack(&[
			rect().background_color(Color::DARK_GRAY),
			column(&[
				title,
				subtitle,
				row(&[
					rect().background_color(Color::RED).grow(1),
					rect().background_color(Color::GREEN).grow(1),
					rect().background_color(Color::BLUE).grow(1),
				])
				.grow(2),
			]),
		]);
		let gui_id = state.guis.insert(ui);
		state
			.windows
			.insert(window().title("UI Example").ui(gui_id).width(900).height(600));
	}
}

fn main() {
	pge::init_logging();
	pge::run(UiExample::default()).unwrap();
}
