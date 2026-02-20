use std::collections::HashMap;
use std::collections::HashSet;
use crate::debug::ChangePrinter;
use crate::ArenaId;
use crate::Node;
use crate::AABB;

#[derive(Debug, Clone)]
struct NodeMetadata {
	rect: AABB,
	cells: Vec<CellCoord>,
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct CellCoord {
	x: i32,
	y: i32,
	z: i32,
}

#[derive(Debug, Clone)]
pub struct SpatialGrid {
	cell_size: f32,
	pub cells: HashMap<CellCoord, Vec<ArenaId<Node>>>,
	nodes: HashMap<ArenaId<Node>, NodeMetadata>,
}

impl SpatialGrid {
	pub fn new(cell_size: f32) -> Self {
		Self {
			cell_size,
			cells: HashMap::new(),
			nodes: HashMap::new(),
		}
	}

	pub fn get_node_rect(&self, node: ArenaId<Node>) -> Option<&AABB> {
		match self.nodes.get(&node) {
			Some(n) => Some(&n.rect),
			None => None,
		}
	}

	pub fn retain_nodes(&mut self, f: impl Fn(&ArenaId<Node>) -> bool) {
		let mut to_remove = HashSet::new();
		for (node_id, _) in &self.nodes {
			if !f(node_id) {
				to_remove.insert(*node_id);
			}
		}

		self.rem_nodes(&to_remove);
	}

	pub fn set_node(&mut self, node_id: ArenaId<Node>, rect: AABB) {
		match self.nodes.contains_key(&node_id) {
			true => {
				self.rem_node(node_id);
				self.add_node(node_id, rect)
			},
			false => {
				self.add_node(node_id, rect)
			},
		}
	}

	pub fn add_node(&mut self, node: ArenaId<Node>, rect: AABB) {
		let min_x = (rect.min.x / self.cell_size).floor() as i32;
		let max_x = (rect.max.x / self.cell_size).ceil() as i32;
		let min_y = (rect.min.y / self.cell_size).floor() as i32;
		let max_y = (rect.max.y / self.cell_size).ceil() as i32;
		let min_z = (rect.min.z / self.cell_size).floor() as i32;
		let max_z = (rect.max.z / self.cell_size).ceil() as i32;
		let mut node_cells = Vec::new();

		for x in min_x..max_x {
			for y in min_y..max_y {
				for z in min_z..max_z {
					let coord = CellCoord { x, y, z };
					node_cells.push(coord);
					let cell = self.cells.entry(coord).or_insert(Vec::new());
					cell.push(node);
				}
			}
		}

		self.nodes.insert(node, NodeMetadata {
			rect,
			cells: node_cells,
		});
	}

	pub fn get_cell(&self, x: i32, y: i32, z: i32) -> &[ArenaId<Node>] {
		let coord = CellCoord { x, y, z };
		self.cells.get(&coord).map(|v| v.as_slice()).unwrap_or(&[])
	}

	pub fn rem_node(&mut self, node_id: ArenaId<Node>) {
		let node = match self.nodes.remove(&node_id) {
			Some(n) => n,
			None => return,
		};

		for cell in node.cells {
			let cell = match self.cells.get_mut(&cell) {
				Some(c) => c,
				None => continue,
			};
			cell.retain(|&inx| inx != node_id);
		}
	}

	pub fn rem_nodes(&mut self, nodes: &HashSet<ArenaId<Node>>) {
		for node_inx in nodes {
			let node = match self.nodes.get(node_inx) {
				Some(n) => n,
				None => continue,
			};

			for cell_inx in &node.cells {
				let cell = match self.cells.get_mut(cell_inx) {
					Some(c) => c,
					None => continue,
				};
				cell.retain(|&inx| inx != *node_inx);
			}
		}
	}

	pub fn get_line_ray_nodes(&self, start: glam::Vec3, end: glam::Vec3) -> HashSet<ArenaId<Node>> {
		let mut nodes = HashSet::new();
		let mut current = start;
		let direction = (end - start).normalize();
		let distance = (end - start).length();
		let mut t = 0.0;
		let mut last_cell = CellCoord { x: 0, y: 0, z: 0 };

		while t < distance {
			let next = current + direction * t;
			let cell = CellCoord {
				x: (next.x / self.cell_size).floor() as i32,
				y: (next.y / self.cell_size).floor() as i32,
				z: (next.z / self.cell_size).floor() as i32,
			};

			if cell != last_cell {
				last_cell = cell;
				if let Some(cell_nodes) = self.cells.get(&cell) {
					for node in cell_nodes {
						nodes.insert(*node);
					}
				}
			}

			t += self.cell_size;
		}

		nodes
	}
}

#[cfg(test)]
mod tests {
	use crate::Arena;
	use super::*;

	#[test]
	fn test_add_node() {
		let mut arena = Arena::new();
		let id = arena.insert(Node::new());
		let mut grid = SpatialGrid::new(1.0);
		let rect = AABB::new(glam::Vec3::new(-1.0, -1.0, -1.0), glam::Vec3::new(1.0, 1.0, 1.0));
		grid.add_node(id, rect);
		assert_eq!(grid.cells.len(), 8);
		let cell = grid.get_cell(-1, -1, -1);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(-1, -1, 0);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(-1, 0, -1);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(-1, 0, 0);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(0, -1, -1);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(0,  -1, 0);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(0, 0, -1);
		assert_eq!(cell.contains(&id), true);
		let cell = grid.get_cell(0, 0, 0);
		assert_eq!(cell.contains(&id), true);
	}

	#[test]
	fn test_considers_grid_cell_size() {
		let mut arena = Arena::new();
		let id = arena.insert(Node::new());
		let mut grid = SpatialGrid::new(2.0);
		let rect = AABB::new(glam::Vec3::new(-1.0, -1.0, -2.0), glam::Vec3::new(0.0, 0.0, -1.0));
		grid.add_node(id, rect);
		assert_eq!(grid.cells.len(), 1);
		let cell = grid.get_cell(-1, -1, -1);
		assert_eq!(cell.contains(&id), true);
	}
}