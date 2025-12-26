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
pub mod text;
pub use types::*;
pub use shapes::*;
pub use gui::*;
pub use arena::*;
pub use glam::*;
pub use log::*;
pub use state::*;
pub use gltf::load_gltf;
pub use urdf::load_urdf;

#[cfg(not(feature = "wgpu_winit"))]
pub fn run<T>(app: T) -> anyhow::Result<()>
where
    T: App,
{
    todo!()
}

#[cfg(feature = "wgpu_winit")]
pub use crate::wgpu::run;
