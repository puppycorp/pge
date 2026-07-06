use pge_core::{Arena, ArenaId, WorldState};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Window {
    pub title: String,
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct GuiElement {
    pub name: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct InputState {
    pub focused_window: Option<ArenaId<Window>>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AppState {
    pub windows: Arena<Window>,
    pub guis: Arena<GuiElement>,
    pub input: InputState,
    pub screenshot_request: Option<(ArenaId<Window>, String)>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct EngineState {
    pub world: WorldState,
    pub app: AppState,
}
