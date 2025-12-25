use std::collections::HashMap;


#[derive(Debug, Default, Clone)]
pub struct ChangePrinter {
    prev: HashMap<u32, String>
}

impl ChangePrinter {
    pub fn new() -> Self {
        Self {
            prev: HashMap::new()
        }
    }

    pub fn print(&mut self, slot: u32, new: String) {
        if self.prev.contains_key(&(slot as u32)) {
			let prev = self.prev.get(&(slot as u32)).unwrap();
			if prev != &new {
				crate::log2!("{}", new);
				self.prev.insert(slot as u32, new);
			}
		} else {
			crate::log2!("{}", new);
			self.prev.insert(slot as u32, new);
		}
    }

	pub fn remove(&mut self, slot: u32) {
		self.prev.remove(&(slot as u32));
	}
}