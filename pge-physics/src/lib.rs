use pge_core::{BodyKind, WorldState};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsStep {
    pub dt_sec: f32,
}

#[derive(Clone, Debug, Default)]
pub struct PhysicsSystem {
    pub gravity_mps2: [f32; 3],
}

impl PhysicsSystem {
    pub fn new() -> Self {
        Self {
            gravity_mps2: [0.0, 0.0, -9.81],
        }
    }

    pub fn step(&mut self, world: &mut WorldState, step: PhysicsStep) {
        for (_, node) in world.nodes.iter_mut() {
            let Some(mut body) = node.body else {
                continue;
            };
            if body.kind != BodyKind::Dynamic {
                continue;
            }
            for axis in 0..3 {
                body.velocity_mps[axis] += self.gravity_mps2[axis] * step.dt_sec;
                body.velocity_mps[axis] *= 1.0 - body.linear_damping.clamp(0.0, 1.0) * step.dt_sec;
                node.transform.translation[axis] += body.velocity_mps[axis] * step.dt_sec;
            }
            node.body = Some(body);
        }
    }
}
