use crate::hardware::*;
use crate::*;

struct MockBuffer {
	name: String,
	size: u64,
	id: u32,
}

pub struct MockHardware {
	buffers: Vec<MockBuffer>,
}

impl MockHardware {
	pub fn new() -> Self {
		Self { buffers: Vec::new() }
	}
}

impl Hardware for MockHardware {
    fn create_buffer(&mut self, name: &str, size: u64) -> BufferHandle {
        let id = self.buffers.len() as u32;
        self.buffers.push(MockBuffer { name: name.to_string(), size, id });
        BufferHandle { id, size }
    }

    fn destroy_buffer(&mut self, _handle: BufferHandle) {
        // No-op for mock
    }

    fn create_texture(&mut self, _name: &str, _data: &[u8], _width: u32, _height: u32) -> TextureHandle {
        TextureHandle { id: 0 }
    }

    fn create_pipeline(&mut self, _name: &str, _window: WindowHandle) -> PipelineHandle {
        PipelineHandle { id: 0 }
    }

    fn render(&mut self, _encoder: RenderEncoder, _window: WindowHandle) {
        // No-op for mock
    }

    fn create_window(&mut self, _window: &Window) -> WindowHandle {
        WindowHandle { id: 0 }
    }

    fn destroy_window(&mut self, _handle: WindowHandle) {
        // No-op for mock
    }

    fn write_buffer(&mut self, _buffer: BufferHandle, _data: &[u8]) {
        // No-op for mock
    }

	fn save_screenshot(&mut self, _window: WindowHandle, _path: &str) {
		// No-op for mock
	}
}
