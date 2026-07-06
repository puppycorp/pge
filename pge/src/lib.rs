pub mod engine;
pub mod types;
pub mod shapes;
pub mod gui;
mod buffer;
mod internal_types;
mod tests;
mod compositor;
pub mod physics;
mod spatial_grid;
//mod engine_state;
mod debug;
//mod texture;
mod gltf;
mod urdf;
mod arena;
mod log;
mod hardware;
mod state;
#[cfg(feature = "wgpu_winit")]
mod wgpu;
mod mock_hardware;
mod collision_detection;
pub mod utility;
pub mod orbit;
pub mod free_fly;
pub mod core;
pub mod text;
pub mod editor;
pub use pge_app as app;
pub use pge_core as world;
pub use pge_physics as shared_physics;
pub use pge_renderer as render;
pub use pge_app::{AppState, EngineState, InputState};
pub use pge_core::{EntityId, EntityMetadata, WorldState};
pub use pge_physics::{PhysicsStep, PhysicsSystem as SharedPhysicsSystem};
pub use pge_renderer::{FrameBuffer, FrameKind, RenderMetadata, RenderOutput, RenderRequest, RenderView, Renderer};
pub use types::*;
pub use shapes::*;
pub use gui::*;
pub use arena::*;
pub use glam::*;
pub use orbit::*;
pub use free_fly::*;
pub use log::*;
pub use state::*;
pub use gltf::load_gltf;
pub use urdf::load_urdf;
pub use editor::{EditorApp, EditorPlugin, EditorSettings, with_editor};

#[cfg(not(feature = "wgpu_winit"))]
pub fn run<T>(app: T) -> anyhow::Result<()>
where
    T: App,
{
    todo!()
}

#[cfg(feature = "wgpu_winit")]
pub use crate::wgpu::{run, run_with_event_sender};
