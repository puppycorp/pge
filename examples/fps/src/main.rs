use std::collections::HashSet;
use std::f32::consts::PI;
use std::time::Instant;
use std::env;

use pge::*;
use pge::physics::PhysicsWorld;
use rand::Rng;

#[derive(Debug, Clone)]
struct PressedKeys {
	forward: bool,
	backward: bool,
	left: bool,
	right: bool,
}

impl PressedKeys {
	pub fn new() -> Self {
		Self {
			forward: false,
			backward: false,
			left: false,
			right: false,
		}
	}

	pub fn to_vec3(&self) -> glam::Vec3 {
        let mut direction = glam::Vec3::ZERO;

        if self.forward {
            direction += glam::Vec3::Z;
        }
        if self.backward {
            direction -= glam::Vec3::Z;
        }
        if self.left {
            direction -= glam::Vec3::X;
        }
        if self.right {
            direction += glam::Vec3::X;
        }

        if direction.length_squared() > 0.0 {
            direction = direction.normalize();
        }

        direction
    }

	pub fn any_pressed(&self) -> bool {
		self.forward || self.backward || self.left || self.right
	}
}

struct Orc {
	node: ArenaId<Node>
}

impl Orc {
	pub fn new(node: ArenaId<Node>) -> Self {
		Self {
			node,
		}
	}

	// pub fn on_create(&mut self, state: &mut State) {
	// 	Model3D::from_path("./assets/orkki.glb");

		

	// 	let mut orc_node = Node::new();
	// 	orc_node.translation = self.initial_pos;
	// 	// orc_node.mesh = Some(asset_id);
	// 	orc_node.physics.typ = PhycisObjectType::Dynamic;
	// 	orc_node.physics.mass = 10.0;
	// 	orc_node.collision_shape = Some(CollisionShape::new(glam::Vec3::new(1.0, 2.0, 1.0)));
	// 	let node_id = state.nodes.insert(orc_node);
	// 	self.node_id = Some(node_id);
	// }

	pub fn on_process(&mut self, enemy: ArenaId<Node>, state: &mut State) {
		let translation = state.nodes.get(&enemy).unwrap().translation;
		let player_node = state.nodes.get_mut(&self.node).unwrap();
		player_node.looking_at(translation.x, translation.y, translation.z);
		let dir = translation - player_node.translation;
		player_node.translation += dir.normalize_or_zero() * 0.01;
	}
}

struct Bullet {
	spawned: Instant,
	node_id: ArenaId<Node>,
}

pub struct FpsShooter {
	physics: PhysicsWorld,
	sensitivity: f32,
	player_id: Option<ArenaId<Node>>,
	light_inx: Option<ArenaId<Node>>,
	light_circle_i: f32,
	pressed_keys: PressedKeys,
	yaw: f32,
	pitch: f32,
	speed: f32,
	dashing: bool,
	movement_force: f32,
	player_ray: Option<ArenaId<RayCast>>,
    gripping: bool,
	gripping_node: Option<ArenaId<Node>>,
	rng: rand::rngs::ThreadRng,
	orcs: Vec<Orc>,
	shooting: bool,
	firing_rate: Instant,
	bullet_mesh: Option<ArenaId<Mesh>>,
	main_scene: Option<ArenaId<Scene>>,
	move_force: Vec3,
	recoil_force: Vec3,
	bullets: Vec<Bullet>,
	world_bounds: f32,
	hit_log: HashSet<(ArenaId<Node>, ArenaId<Node>)>,
	autoplay: bool,
	autoplay_next_pick: Instant,
	floor_id: Option<ArenaId<Node>>,
	player_floor_violation_frames: u32,
}

impl FpsShooter {
	pub fn new() -> Self {
		let mut rng = rand::thread_rng();

		Self {
			physics: PhysicsWorld::new(),
			player_id: None,
			light_inx: None,
			sensitivity: 0.001,
			pressed_keys: PressedKeys::new(),
			yaw: 0.0,
			pitch: 0.0,
			speed: 10.0,
			light_circle_i: 0.0,
			movement_force: 4000.0,
			dashing: false,
			player_ray: None,
			gripping: false,
			gripping_node: None,
			rng,
			orcs: Vec::new(),
			shooting: false,
			firing_rate: Instant::now(),
			bullet_mesh: None,
			main_scene: None,
			move_force: Vec3::ZERO,
			recoil_force: Vec3::ZERO,
			bullets: Vec::new(),
			world_bounds: 0.0,
			hit_log: HashSet::new(),
			autoplay: false,
			autoplay_next_pick: Instant::now(),
			floor_id: None,
			player_floor_violation_frames: 0,
		}
	}
}

impl FpsShooter {
	pub fn rotate_player(&mut self, dx: f32, dy: f32) {
		self.yaw += dx * self.sensitivity;
		self.pitch += dy * self.sensitivity;

		if self.pitch > PI / 2.0 {
			self.pitch = PI / 2.0;
		} else if self.pitch < -PI / 2.0 {
			self.pitch = -PI / 2.0;
		}
	}

	fn handle_rays(&mut self, state: &mut State) {
		if let Some(player_ray_inx) = self.player_ray {
			if let Some(player_ray) = state.raycasts.get_mut(&player_ray_inx) {
				if player_ray.intersects.len() > 0 {
					if !self.gripping {
						return;
					}
	
					// pge::log2!("player ray intersects: {:?}", player_ray.intersects);
	
					let translation = {
						let player_inx = match self.player_id {
							Some(index) => index,
							None => return,
						};
	
						let player = match state.nodes.get_mut(&player_inx) {
							Some(node) => node,
							None => return,
						};
	
						let dir = player.rotation * Vec3::new(0.0, 0.0, 1.0);
						player.translation + dir * 5.0
					};
	
					let first_node = match player_ray.intersects.first() {
						Some(inx) => {
							self.gripping_node = Some(*inx);
							match state.nodes.get_mut(inx) {
								Some(node) => node,
								None => return,
							}
						},
						None => return,
					};
	
					if first_node.physics.typ != PhycisObjectType::Dynamic {
						return;
					}
	
					first_node.translation = translation;
				}
			}
		}
	}

	fn handle_dashing(&mut self, state: &mut State) {
		if self.dashing {
			let player_inx = match self.player_id {
				Some(index) => index,
				None => return,
			};
			let player = match state.nodes.get_mut(&player_inx) {
				Some(node) => node,
				None => return,
			};
			let dir = player.rotation * Vec3::new(0.0, 0.0, 1.0);
			player.physics.velocity = dir * 100.0;
		}
	}


	fn handle_shooting(&mut self, state: &mut State) {
		if self.firing_rate.elapsed().as_secs_f32() < 0.1 {
			return;
		}
		self.firing_rate = Instant::now();

		if !self.shooting {
			self.recoil_force = Vec3::ZERO;
			return;
		}

		let player_inx = match self.player_id {
			Some(index) => index,
			None => return,
		};



		if let Some(bullet_mesh_id) = self.bullet_mesh {
			pge::log2!("spawn bullet");
			let mut bullet = Node::new();
			bullet.mesh = Some(bullet_mesh_id);
			bullet.physics.typ = PhycisObjectType::Dynamic;
			bullet.physics.mass = 1.0;
			bullet.collision_shape = Some(CollisionShape::new(glam::Vec3::new(0.3, 0.3, 0.3)));
			bullet.parent = NodeParent::Scene(self.main_scene.unwrap());
			let rotation = state.nodes.get(&player_inx).unwrap().rotation;
			let mut translation = state.nodes.get(&player_inx).unwrap().translation;
			// location in fron of player
			translation += rotation * Vec3::new(0.0, 0.0, 3.0);
			bullet.translation = translation;
			
			let dir = rotation * Vec3::new(0.0, 0.0, 1.0);
			bullet.physics.velocity = dir * 50.0;
			let bullet_id = state.nodes.insert(bullet);
			self.bullets.push(Bullet {
				spawned: Instant::now(),
				node_id: bullet_id,
			});
		}

		let player = match state.nodes.get_mut(&player_inx) {
			Some(node) => node,
			None => return,
		};

		let dir = player.rotation * Vec3::new(0.0, 0.0, 0.3);
		self.recoil_force = dir * -100.0;

		// rotate comera up
		self.pitch -= 0.05;
		// let rot = glam::Quat::from_euler(glam::EulerRot::YXZ, 0.0, 0.3, 0.0);
		// player.rotation = rot * player.rotation;
	}

	fn handle_player_move(&mut self, state: &mut State) {
		let player_id = match self.player_id {
			Some(index) => index,
			None => return,
		};

		if let Some(player) = state.nodes.get_mut(&player_id) {
			let current_speed = player.physics.velocity.length();
			if self.pressed_keys.any_pressed() {
				let dir = self.pressed_keys.to_vec3();
				let mut force = player.rotation * dir;

				if force.x > 0.0 && player.physics.velocity.x < 0.0 {
					force.x += -player.physics.velocity.x * self.movement_force;
				} else if force.x < 0.0 && player.physics.velocity.x > 0.0 {
					force.x += -player.physics.velocity.x * self.movement_force;
				} else if current_speed < 25.0 {
					force.x *= self.movement_force;
				} 

				if force.z > 0.0 && player.physics.velocity.z < 0.0 {
					force.z += -player.physics.velocity.z * self.movement_force;
				} else if force.z < 0.0 && player.physics.velocity.z > 0.0 {
					force.z += -player.physics.velocity.z * self.movement_force;
				} else if current_speed < 25.0 {
					force.z *= self.movement_force;
				}

				force.y = 0.0;

				self.move_force = force;
				// pge::log2!("force: {:?}", player.physics.force);
			} else {
				// We calculate force opposite of momevement to slow down the player
				let force = -player.physics.velocity.xz() * self.movement_force;
				self.move_force = glam::Vec3::new(force.x, 0.0, force.y);
				//player.physics.force = glam::Vec3::ZERO;
			}
		}
	}
}

impl pge::App for FpsShooter {
	fn on_create(&mut self, state: &mut State) {
		self.autoplay = matches!(env::var("AUTOPLAY").as_deref(), Ok("1"));

		let scene = Scene::new();
		let main_scene_id = state.scenes.insert(scene);
		self.main_scene = Some(main_scene_id);

		let model_id = state.load_3d_model("./assets/orkki.glb");
		let ork_scene_id = {
			let model = state.models.get(&model_id).unwrap();
			match model.default_scene {
				Some(id) => id,
				None => model.scenes[0],
			}
		};
		let mut orc_base_node = Node::new();
		orc_base_node.parent = NodeParent::Orphan;
		let orc_base_node_id = state.nodes.insert(orc_base_node);
		
		for (node_id, node) in &mut state.nodes.iter_mut() {
			if node.parent == NodeParent::Scene(ork_scene_id) {
				node.parent = NodeParent::Node(orc_base_node_id);
			}
		} 

		let bullet_mesh = cube(0.3);
		let bullet_mesh_id = state.meshes.insert(bullet_mesh);
		self.bullet_mesh = Some(bullet_mesh_id);

		// pge::log2!("continue");

		let mut rng = rand::thread_rng();

		for _ in 0..10 {
			let node_id = state.clone_node(orc_base_node_id);
			let node = state.nodes.get_mut(&node_id).unwrap();
			node.parent = NodeParent::Scene(main_scene_id);
			node.physics.typ = PhycisObjectType::Dynamic;
			node.physics.mass = 10.0;
			node.lock_rotation = true;
			node.collision_shape = Some(CollisionShape::new(glam::Vec3::new(1.0, 3.0, 1.0)));
			let x = rng.gen_range(-20.0..20.0);
			let z = rng.gen_range(-20.0..20.0);
			let pos = Vec3::new(x, 10.0, z);
			node.translation = pos;

			let orc = Orc::new(node_id);
			self.orcs.push(orc);
		}

		pge::log2!("continue2");

		let texture = Texture::new("./assets/gandalf.jpg");
	 	let texture_id = state.textures.insert(texture);
		let material = Material {
			name: Some("GANDALF".to_string()),
			base_color_texture: Some(texture_id),
			..Default::default()
		};
		let material_id = state.materials.insert(material);
		let mut cube_mesh = cube(0.5);
		cube_mesh.primitives[0].material = Some(material_id);
		let mut plane_mesh = plane(1.0, 1.0);
		plane_mesh.primitives[0].material = Some(material_id);

		let cube_mesh = state.meshes.insert(cube_mesh);
		let plane_mesh = state.meshes.insert(plane_mesh);

		let plane_size = 1000.0;
		self.world_bounds = plane_size * 0.5;

		let mut light_node = Node::new();
		light_node.name = Some("Light".to_string());
		light_node.set_translation(10.0, 10.0, 0.0);
		light_node.parent = NodeParent::Scene(main_scene_id);
		let light_node_id = state.nodes.insert(light_node);
		self.light_inx = Some(light_node_id);
		let mut light = PointLight::new();
		light.node_id = Some(light_node_id);
		state.point_lights.insert(light);

		let mut plane_node = Node::new();
		plane_node.name = Some("Floor".to_string());
		plane_node.set_translation(0.0, 1.0, 0.0);
		plane_node.mesh = Some(plane_mesh);
		plane_node.physics.typ = PhycisObjectType::Static;
		plane_node.scale = glam::Vec3::new(plane_size, 1.0, plane_size);
		plane_node.collision_shape = Some(CollisionShape::new(glam::Vec3::new(plane_size, 0.1, plane_size)));
		plane_node.parent = NodeParent::Scene(main_scene_id);
		let plane_node_id = state.nodes.insert(plane_node);
		self.floor_id = Some(plane_node_id);

		let mut player = Node::new();
		player.name = Some("Player".to_string());
		player.set_translation(0.0, 30.0, 0.0);
		player.physics.typ = PhycisObjectType::Dynamic;
		player.physics.mass = 70.0;
		//player.looking_at(0.0, 0.0, 0.0);
		player.collision_shape = Some(CollisionShape::new(glam::Vec3::new(1.0, 2.0, 1.0)));
		player.parent = NodeParent::Scene(main_scene_id);
		player.lock_rotation = true;
		let player_id = state.nodes.insert(player);

		{
			let mut node = Node::new();
			node.parent = NodeParent::Node(player_id);
			node.translation = glam::Vec3::new(0.3, -1.0, 1.0);
			// rotate 180 degrees
			node.rotation = glam::Quat::from_euler(glam::EulerRot::YXZ, PI * 1.5, 0.0, 0.0);
			let node_id = state.nodes.insert(node);
			let ak47_model_id = state.load_3d_model("./assets/ak47.glb");
			let model = state.models.get(&ak47_model_id).unwrap();
			let ak47_scene_id = match model.default_scene {
				Some(id) => id,
				None => model.scenes[0],
			};
			
			for (_, node) in &mut state.nodes.iter_mut() {
				if node.parent == NodeParent::Scene(ak47_scene_id) {
					node.parent = NodeParent::Node(node_id);
					node.scale = glam::Vec3::new(60.0, 60.0, 60.0);
				}
			}
		}
		

		let raycast = RayCast::new(player_id, 10.0);
		let player_ray_inx = state.raycasts.insert(raycast);
		self.player_ray = Some(player_ray_inx);

		//Spawn random cubes
		let mut rng = rand::thread_rng();
		let cube_count = 0;
		for i in 0..cube_count {
			let x = rng.gen_range(-20.0..20.0);
			let z = rng.gen_range(-20.0..20.0);
			let mut cube_node = Node::new();
			cube_node.name = Some(format!("Cube{}", i));
			cube_node.set_translation(x, 10.0, z);
			cube_node.mesh = Some(cube_mesh);
			cube_node.physics.typ = PhycisObjectType::Dynamic;
			cube_node.physics.mass = 10.0;
			cube_node.collision_shape = Some(CollisionShape::new(glam::Vec3::new(1.0, 1.0, 1.0)));
			cube_node.parent = NodeParent::Scene(main_scene_id);
			let node_id = state.nodes.insert(cube_node);
		}

		let mut camera = Camera::new();
		camera.zfar = 1000.0;
		camera.node_id = Some(player_id);
		let camera_id = state.cameras.insert(camera);
		self.player_id = Some(player_id);

		let gui = stack(&[
			camera_view(camera_id),
			rect().background_color(Color::GREEN).height(0.1).anchor_bottom()
		]);
		let gui_id = state.guis.insert(gui);

		let window = window().title("FPS Shooter1").ui(gui_id).lock_cursor(true).width(1024).height(768);
		state.windows.insert(window);
	}

	fn on_keyboard_input(&mut self, window_id: ArenaId<Window>, key: KeyboardKey, action: KeyAction, state: &mut State) {
		match action {
			KeyAction::Pressed => {
				match key {
					KeyboardKey::W => self.pressed_keys.forward = true,
					KeyboardKey::S => self.pressed_keys.backward = true,
					KeyboardKey::A => self.pressed_keys.left = true,
					KeyboardKey::D => self.pressed_keys.right = true,
					KeyboardKey::Space => {
						let player_inx = match self.player_id {
							Some(index) => index,
							None => return,
						};
						let player = state.nodes.get_mut(&player_inx).unwrap();
						player.physics.velocity.y = 10.0;
					},
					KeyboardKey::ShiftLeft => {
						self.dashing = true;
					},
					KeyboardKey::G => self.gripping = true,
					_ => {}
				}
			},
			KeyAction::Released => {
				match key {
					KeyboardKey::W => self.pressed_keys.forward = false,
					KeyboardKey::S => self.pressed_keys.backward = false,
					KeyboardKey::A => self.pressed_keys.left = false,
					KeyboardKey::D => self.pressed_keys.right = false,
					KeyboardKey::ShiftLeft => {
						self.dashing = false;
					},
					KeyboardKey::G => self.gripping = false,
					_ => {}
				}
			},
		};

		let player = match self.player_id {
			Some(index) => match state.nodes.get_mut(&index) {
				Some(node) => node,
				None => return,
			},
			None => return,
		};

		// let dir = self.pressed_keys.to_vec3();
		// player.physics.force = player.rotation * dir * 300.0;
	}

	fn on_mouse_input(&mut self, window_id: ArenaId<Window>, event: MouseEvent, state: &mut State) {
		match event {
			MouseEvent::Moved { dx, dy } => {
				let player_inx = match self.player_id {
					Some(index) => index,
					None => {
						log::error!("Player not found");
						return;
					},
				};
				self.rotate_player(dx, dy);
				let player = state.nodes.get_mut(&player_inx).unwrap();
				player.rotation = glam::Quat::from_euler(glam::EulerRot::YXZ, self.yaw, self.pitch, 0.0);
			},
			MouseEvent::Pressed { button } => {
				match button {
					MouseButton::Left => {
						if let Some(gripping_node) = self.gripping_node.take() {
							self.gripping = false;

							let push_vel = {
								let player_inx = match self.player_id {
									Some(index) => index,
									None => return,
								};

								let player = match state.nodes.get_mut(&player_inx) {
									Some(node) => node,
									None => return,
								};

								let dir = player.rotation * Vec3::new(0.0, 0.0, 1.0);
								dir * 100.0
							};

							if let Some(node) = state.nodes.get_mut(&gripping_node) {
								node.physics.velocity = push_vel;
							}
						} else {
							self.shooting = true;
						}
					},
					_ => {}
				}
			},
			MouseEvent::Released { button } => {
				match button {
					MouseButton::Left => {
						self.shooting = false;
					},
					_ => {}
				}
			},
			_ => {}
		}
	}

	fn on_process(&mut self, state: &mut State, delta: f32) {
		self.physics.process(state, delta);

		if let (Some(player_id), Some(floor_id)) = (self.player_id, self.floor_id) {
			let player = state.nodes.get(&player_id);
			let floor = state.nodes.get(&floor_id);
			if let (Some(player), Some(floor)) = (player, floor) {
				let player_half_y = player
					.collision_shape
					.as_ref()
					.map(|shape| shape.aabb(player.translation).max[1] - player.translation.y)
					.unwrap_or(0.0);
				let floor_half_y = floor
					.collision_shape
					.as_ref()
					.map(|shape| floor.translation.y - shape.aabb(floor.translation).min[1])
					.unwrap_or(0.0);
				let floor_min_y = floor.translation.y - floor_half_y;
				let player_min_y = player.translation.y - player_half_y;
				let allowed_penetration = player_half_y * 0.25;
				let touching_floor = player
					.contacts
					.iter()
					.any(|contact| contact.node_id == floor_id);
				if player_min_y < floor_min_y {
					if let Some(player) = state.nodes.get_mut(&player_id) {
						let correction = floor_min_y - player_min_y;
						player.translation.y += correction;
						player.physics.velocity.y = 0.0;
					}
					self.player_floor_violation_frames = 0;
				} else if player_min_y < floor_min_y - allowed_penetration && !touching_floor {
					self.player_floor_violation_frames += 1;
					if self.player_floor_violation_frames > 10 {
						panic!(
							"Player fell through floor: player_id={:?}, y_min={:.3}, floor_min={:.3}, allowed={:.3}, frames={}",
							player_id,
							player_min_y,
							floor_min_y,
							allowed_penetration,
							self.player_floor_violation_frames
						);
					}
				} else {
					self.player_floor_violation_frames = 0;
				}
			}
		}

		if self.autoplay {
			self.shooting = true;
			let player_id = match self.player_id {
				Some(id) => id,
				None => return,
			};
			let player_pos = match state.nodes.get(&player_id) {
				Some(node) => node.translation,
				None => return,
			};
			if self.autoplay_next_pick.elapsed().as_secs_f32() > 0.5 {
				let mut candidates = Vec::new();
				for orc in &self.orcs {
					let node = match state.nodes.get(&orc.node) {
						Some(node) => node,
						None => continue,
					};
					let dist = node.translation.distance(player_pos);
					if dist < 50.0 {
						candidates.push(orc.node);
					}
				}
				if !candidates.is_empty() {
					let idx = self.rng.gen_range(0..candidates.len());
					let target_id = candidates[idx];
					let target_pos = match state.nodes.get(&target_id) {
						Some(target) => target.translation,
						None => return,
					};
					if let Some(player) = state.nodes.get_mut(&player_id) {
						player.looking_at(target_pos.x, target_pos.y, target_pos.z);
					}
				}
				self.autoplay_next_pick = Instant::now();
			}
		}

		let bullet_ids: HashSet<ArenaId<Node>> = self.bullets.iter().map(|b| b.node_id).collect();
		self.hit_log.retain(|(_, bullet_id)| bullet_ids.contains(bullet_id));
		for orc in &self.orcs {
			let node = match state.nodes.get(&orc.node) {
				Some(node) => node,
				None => continue,
			};
			for contact in &node.contacts {
				if bullet_ids.contains(&contact.node_id) {
					let key = (orc.node, contact.node_id);
					if self.hit_log.insert(key) {
						pge::log2!("orc hit: orc_id={} bullet_id={}", orc.node, contact.node_id);
					}
				}
			}
		}

		for orc in &mut self.orcs {
			orc.on_process(self.player_id.unwrap(), state);
		}
		let bounds = self.world_bounds;
		for orc in &self.orcs {
			let node = match state.nodes.get(&orc.node) {
				Some(node) => node,
				None => continue,
			};
			let pos = node.translation;
			let mut clamped = pos;
			let mut out = false;
			if pos.y < 0.0 {
				clamped.y = 0.0;
				out = true;
			} else if pos.y > 200.0 {
				clamped.y = 200.0;
				out = true;
			}
			if pos.x.abs() > bounds {
				clamped.x = bounds * pos.x.signum();
				out = true;
			}
			if pos.z.abs() > bounds {
				clamped.z = bounds * pos.z.signum();
				out = true;
			}
			if out {
				if let Some(node) = state.nodes.get_mut(&orc.node) {
					node.translation = clamped;
					node.physics.velocity = Vec3::ZERO;
				}
			}
			if clamped.y < 0.0 || clamped.y > 200.0 || clamped.x.abs() > bounds || clamped.z.abs() > bounds {
				panic!(
					"Orc out of bounds: id={:?}, pos={:?}, bounds=±{}, y=[0,200]",
					orc.node,
					clamped,
					bounds
				);
			}
		}

		if let Some(index) = self.light_inx {
			let light = state.nodes.get_mut(&index).unwrap();
			self.light_circle_i += delta;
			let x = 10.0 * self.light_circle_i.cos();
			let z = 10.0 * self.light_circle_i.sin();
			light.set_translation(x, 10.0, z);
		}

		self.handle_player_move(state);
		self.handle_dashing(state);
		self.handle_rays(state);
		self.handle_shooting(state);

		if let Some(player_id) = self.player_id {
			let player = state.nodes.get_mut(&player_id).unwrap();
			player.physics.force = self.move_force;
			player.physics.force += self.recoil_force;
		}

		self.bullets.retain(|bullet| {
			if bullet.spawned.elapsed().as_secs_f32() > 5.0 {
				pge::log2!("depspawn bullet {}", bullet.node_id);
				state.nodes.remove(&bullet.node_id);
				false
			} else {
				true
			}
		});
	}
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
	pge::init_logging();
	Ok(pge::run(FpsShooter::new())?)
}
