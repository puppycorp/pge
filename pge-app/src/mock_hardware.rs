use std::collections::HashMap;

use crate::hardware::*;
use crate::*;

#[derive(Clone, Copy)]
struct MockWindow {
    width: u32,
    height: u32,
}

struct MockBuffer {
    name: String,
    size: u64,
    id: u32,
}

pub struct MockHardware {
    buffers: Vec<MockBuffer>,
    next_buffer_id: u32,
    windows: HashMap<u32, MockWindow>,
    next_window_id: u32,
}

impl MockHardware {
    pub fn new() -> Self {
        Self {
            buffers: Vec::new(),
            next_buffer_id: 0,
            windows: HashMap::new(),
            next_window_id: 0,
        }
    }
}

impl Hardware for MockHardware {
    fn create_buffer(&mut self, name: &str, size: u64) -> BufferHandle {
        let id = self.next_buffer_id;
        self.next_buffer_id += 1;
        self.buffers.push(MockBuffer {
            name: name.to_string(),
            size,
            id,
        });
        BufferHandle { id, size }
    }

    fn destroy_buffer(&mut self, _handle: BufferHandle) {
        // No-op for mock
    }

    fn create_texture(
        &mut self,
        _name: &str,
        _data: &[u8],
        _width: u32,
        _height: u32,
    ) -> TextureHandle {
        TextureHandle { id: 0 }
    }

    fn create_pipeline(&mut self, _name: &str, _window: WindowHandle) -> PipelineHandle {
        PipelineHandle { id: 0 }
    }

    fn render(&mut self, _encoder: RenderEncoder, _window: WindowHandle) {
        // No-op for mock
    }

    fn create_window(&mut self, _window: &Window) -> WindowHandle {
        let id = self.next_window_id;
        self.next_window_id += 1;
        self.windows.insert(
            id,
            MockWindow {
                width: _window.width.max(1),
                height: _window.height.max(1),
            },
        );
        WindowHandle { id }
    }

    fn destroy_window(&mut self, _handle: WindowHandle) {
        // No-op for mock
    }

    fn write_buffer(&mut self, _buffer: BufferHandle, _data: &[u8]) {
        // No-op for mock
    }

    fn save_screenshot(&mut self, _window: WindowHandle, _path: &str) {
        let window = self
            .windows
            .get(&_window.id)
            .copied()
            .unwrap_or(MockWindow {
                width: 800,
                height: 600,
            });
        let mut pixels = Vec::new();
        let stride = window.width as usize * 4;
        pixels.reserve_exact(stride.saturating_mul(window.height as usize));
        for y in 0..window.height {
            for x in 0..window.width {
                let x_block = (x / 16) % 2;
                let y_block = (y / 16) % 2;
                let checker = (x_block + y_block) % 2;
                let mut r = if checker == 0 { 40 } else { 220 };
                let mut g = if checker == 0 { 180 } else { 40 };
                let mut b = if checker == 0 { 230 } else { 20 };
                if x < 8
                    || y < 8
                    || x > window.width.saturating_sub(8)
                    || y > window.height.saturating_sub(8)
                {
                    r = 255;
                    g = 60;
                    b = 60;
                }
                let a = 255;
                pixels.extend_from_slice(&[r, g, b, a]);
            }
        }
        if let Err(err) = image::save_buffer(
            _path,
            &pixels,
            window.width,
            window.height,
            image::ColorType::Rgba8,
        ) {
            eprintln!(
                "mock screenshot save failed for {path}: {err}",
                path = _path
            );
        }
    }
}
