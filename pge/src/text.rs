use std::collections::HashMap;
use std::error;

use lyon::math::point;
use lyon::path::builder::NoAttributes;
use lyon::path::traits::PathBuilder;
use lyon::path::BuilderImpl;
use lyon::path::Path;
use lyon::tessellation::BuffersBuilder;
use lyon::tessellation::FillOptions;
use lyon::tessellation::FillTessellator;
use lyon::tessellation::FillVertex;
use lyon::tessellation::VertexBuffers;
use ttf_parser::GlyphId;
use ttf_parser::OutlineBuilder;
use ttf_parser::Rect;

use crate::Mesh;
use crate::Primitive;
use crate::PrimitiveTopology;

// pub struct Mesh {
//     pub id: usize,
//     pub material: Option<Material>,
//     pub positions: Vec<[f32; 3]>,
//     pub normals: Vec<[f32; 3]>,
//     pub text_coords: Vec<[f32; 2]>,
//     pub indices: Vec<u16>,
// }

// pub struct Mesh {
//     pub id: usize,
//     pub material: Option<Material>,
//     pub positions: Vec<[f32; 3]>,
//     pub normals: Vec<[f32; 3]>,
//     pub text_coords: Vec<[f32; 2]>,
//     pub indices: Vec<u16>,
// }

pub struct GlyphMeshBuilder {
    builder: NoAttributes<BuilderImpl>
}

impl GlyphMeshBuilder {
    pub fn new() -> Self {
		let builder = Path::builder();

        GlyphMeshBuilder {
            builder
        }
    }

    // Helper method to add vertex
    // fn add_vertex(&mut self, x: f32, y: f32) -> u16 {
    //     let z = 0.0; // for 2D paths, z is always 0
    //     self.mesh.positions.push([x, y, z]);
    //     (self.mesh.positions.len() - 1) as u16
    // }

    pub fn build_mesh(mut self, rect: Rect) -> Mesh {
        let path = self.builder.build();
		// Let's use our own custom vertex type instead of the default one.
		#[derive(Copy, Clone, Debug)]
		struct MyVertex { position: [f32; 2] }
		// Will contain the result of the tessellation.
		let mut geometry: VertexBuffers<MyVertex, u16> = VertexBuffers::new();
		let mut tessellator = FillTessellator::new();
		{
			// Compute the tessellation.
			tessellator.tessellate_path(
				&path,
				&FillOptions::default(),
				&mut BuffersBuilder::new(&mut geometry, |vertex: FillVertex| {
					MyVertex {
						position: vertex.position().to_array(),
					}
				}),
			).unwrap();
		}

		let mut mesh = Mesh::new();
		let mut p = Primitive::new(PrimitiveTopology::TriangleList);

		p.vertices = geometry.vertices.iter().map(|v| [v.position[0], v.position[1], 0.0]).collect();
		p.indices = geometry.indices.chunks(3).flat_map(|chunk| chunk.iter().rev()).map(|i| *i as u16).collect();
		p.normals = vec![[0.0, 0.0, 1.0]; p.vertices.len()];
		normalize(&mut p.vertices, &rect);
		mesh
    }
}

impl OutlineBuilder for GlyphMeshBuilder {
    fn move_to(&mut self, x: f32, y: f32) {
		println!("move_to: {}, {}", x, y);
        self.builder.begin(point(x, y));
    }

    fn line_to(&mut self, x: f32, y: f32) {
		println!("line_to: {}, {}", x, y);
        self.builder.line_to(point(x, y));
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
		println!("quad_to: {}, {}, {}, {}", x1, y1, x, y);
        self.builder.quadratic_bezier_to(point(x1, y1), point(x, y));
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
		println!("curve_to: {}, {}, {}, {}, {}, {}", x1, y1, x2, y2, x, y);
        self.builder.cubic_bezier_to(point(x1, y1), point(x2, y2), point(x, y));
    }

    fn close(&mut self) {
		println!("close");
        self.builder.end(true);
    }
}

// Tesselation functions
fn tessellate_quad_bezier(x0: f32, y0: f32, x1: f32, y1: f32, x2: f32, y2: f32) -> Vec<[f32; 2]> {
    let mut points = Vec::new();
    for t in 0..=10 {
        let t = t as f32 / 10.0;
        let x = (1.0 - t) * (1.0 - t) * x0 + 2.0 * (1.0 - t) * t * x1 + t * t * x2;
        let y = (1.0 - t) * (1.0 - t) * y0 + 2.0 * (1.0 - t) * t * y1 + t * t * y2;
        points.push([x, y]);
    }
    points
}

fn tessellate_cubic_bezier(x0: f32, y0: f32, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) -> Vec<[f32; 2]> {
    let mut points = Vec::new();
    for t in 0..=10 {
        let t = t as f32 / 10.0;
        let x = (1.0 - t).powi(3) * x0
            + 3.0 * (1.0 - t).powi(2) * t * x1
            + 3.0 * (1.0 - t) * t * t * x2
            + t * t * t * x3;
        let y = (1.0 - t).powi(3) * y0
            + 3.0 * (1.0 - t).powi(2) * t * y1
            + 3.0 * (1.0 - t) * t * t * y2
            + t * t * t * y3;
        points.push([x, y]);
    }
    points
}

const chars : [char; 95] = [
	' ', '!', '"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
	'@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
	'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', '\\', ']', '^', '_',
	'`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
	'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~'
];

pub struct FontMesh {
    map: HashMap<char, Mesh>,
}

impl FontMesh {
    pub fn load(path: &str) -> anyhow::Result<Self> {
        let font_data = std::fs::read(path)?;
        let face = ttf_parser::Face::parse(&font_data, 0)?;

        let mut map = HashMap::new();

        let glyphs_count = face.number_of_glyphs();

		for char in chars.iter() {
			let gid = match face.glyph_index(*char) {
				Some(gid) => gid,
				None => {
					println!("no glyph for char: {:?}", char);
					continue;
				}
			};
			
			let mut b = GlyphMeshBuilder::new();
            let outline = match face.outline_glyph(gid, &mut b) {
                Some(outline) => outline,
                None => {
                    println!("no outline for glyph: {:?}", gid);
                    continue;
                }
            };
			let mesh = b.build_mesh(outline);
			map.insert(*char, mesh);
		}

        // for id in 0..glyphs_count {
        //     let gid = GlyphId(id);
		// 	face.
        //     let mut b = GlyphMeshBuilder::new();
        //     let outline = match face.outline_glyph(gid, &mut b) {
        //         Some(outline) => outline,
        //         None => {
        //             println!("no outline for glyph: {:?}", gid);
        //             continue;
        //         }
        //     };
		// 	let mesh = b.build_mesh(outline);
        //     // let mut builder = Builder {};

        //     // outline.build_outline(&mut builder);

        //     // map.insert(glyph.character(), Mesh {});
        // }

        Ok(FontMesh { map })
    }

    pub fn get_mesh(&self, char: char) -> Option<Mesh> {
		self.map.get(&char).cloned()
    }
}

fn normalize(points: &mut Vec<[f32; 3]>, rect: &Rect) {
	let width = rect.width() as f32;
	let height = rect.height() as f32;
	let size = width.max(height);
	let width_ratio = width / size;
	let height_ratio = height / size;

    // Compute the center of the rectangle
    let center_x = (rect.x_min + rect.x_max) as f32 / 2.0;
    let center_y = (rect.y_min + rect.y_max) as f32 / 2.0;

    // Compute the half width and half height of the rectangle
    let half_width = (rect.x_max - rect.x_min) as f32 / 2.0;
    let half_height = (rect.y_max - rect.y_min) as f32 / 2.0;

    // Normalize each point
    for point in points.iter_mut() {
        point[0] = (point[0] - center_x) / half_width * width_ratio;
        point[1] = (point[1] - center_y) / half_height * height_ratio;
    }
}

pub enum WhiteSpace {
	Normal,
	Nowrap
}

pub struct TextRenderer {
	font: FontMesh,
	font_size: u32,
	white_space: WhiteSpace,
	
}

impl TextRenderer {
	pub fn render(&self, text: &str) -> Mesh {
		let mut mesh = Mesh::new();
		// let p = Primitive::new(PrimitiveTopology::TriangleList);
		// for c in text.chars() {
		// 	println!("char: {:?}", c);
		// 	match self.font.get_mesh(c) {
		// 		Some(char_mesh) => {
		// 			mesh.add_mesh(char_mesh);
		// 		}
		// 		None => {}
		// 	}
		// }
		// mesh
		mesh
	}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_font() {
        let font = FontMesh::load("./fonts/Roboto-Regular.ttf").unwrap();
        let mesh = font.get_mesh('I');
		println!("mesh: {:?}", mesh);
    }

	#[test]
	fn test_rect_normalize() {
		let mut v = vec![[200.0, 200.0, 0.0], [100.0, 200.0, 0.0], [100.0, 100.0, 0.0], [200.0, 100.0, 0.0]];
		let rect = Rect {
			x_min: 100,
			y_min: 100,
			x_max: 200,
			y_max: 200,
		};
		normalize(&mut v, &rect);
		assert_eq!(v, vec![[1.0, 1.0, 0.0], [-1.0, 1.0, 0.0], [-1.0, -1.0, 0.0], [1.0, -1.0, 0.0]]);
	}

	#[test]
	fn test_rect2_normalize() {
		let mut v = vec![[400.0, 500.0, 0.0], [200.0, 500.0, 0.0], [200.0, 100.0, 0.0], [400.0, 100.0, 0.0]];
		let rect = Rect {
			x_min: 200,
			y_min: 100,
			x_max: 400,
			y_max: 500,
		};
		normalize(&mut v, &rect);
		assert_eq!(v, vec![[0.5, 1.0, 0.0], [-0.5, 1.0, 0.0], [-0.5, -1.0, 0.0], [0.5, -1.0, 0.0]]);
	}

	#[test]
	fn build_simple_i_outline() {
		let mut b = GlyphMeshBuilder::new();

		b.move_to(375.0, 0.0);
	}
}
