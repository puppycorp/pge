const std = @import("std");

fn expectApproxEq(comptime T: type, got: T, expected: T, tol: T) !void {
    try std.testing.expect(@abs(got - expected) <= tol);
}

const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn add(self: Vec3, other: Vec3) Vec3 {
        return Vec3{
            .x = self.x + other.x,
            .y = self.y + other.y,
            .z = self.z + other.z,
        };
    }
    pub fn sub(self: Vec3, other: Vec3) Vec3 {
        return Vec3{
            .x = self.x - other.x,
            .y = self.y - other.y,
            .z = self.z - other.z,
        };
    }
    pub fn scale(self: Vec3, scalar: f32) Vec3 {
        return Vec3{
            .x = self.x * scalar,
            .y = self.y * scalar,
            .z = self.z * scalar,
        };
    }
    pub fn dot(self: Vec3, other: Vec3) f32 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }
    pub fn cross(self: Vec3, other: Vec3) Vec3 {
        return Vec3{
            .x = self.y * other.z - self.z * other.y,
            .y = self.z * other.x - self.x * other.z,
            .z = self.x * other.y - self.y * other.x,
        };
    }
    pub fn length(self: Vec3) f32 {
        return std.math.sqrt(self.dot(self));
    }

    pub fn normalized(self: Vec3) Vec3 {
        const len = self.length();
        if (len == 0.0) return self;
        return self.scale(1.0 / len);
    }
};

pub const Quaternion = struct {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
    pub fn identity() Quaternion {
        return Quaternion{ .w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0 };
    }
    /// Scales the quaternion by a scalar.
    pub fn scale(self: Quaternion, scalar: f32) Quaternion {
        return Quaternion{
            .w = self.w * scalar,
            .x = self.x * scalar,
            .y = self.y * scalar,
            .z = self.z * scalar,
        };
    }
    /// Normalizes the quaternion.
    pub fn normalized(self: Quaternion) Quaternion {
        const mag = std.math.sqrt(self.w * self.w + self.x * self.x +
            self.y * self.y + self.z * self.z);
        return Quaternion{
            .w = self.w / mag,
            .x = self.x / mag,
            .y = self.y / mag,
            .z = self.z / mag,
        };
    }
    /// Multiplies two quaternions.
    pub fn mul(self: Quaternion, other: Quaternion) Quaternion {
        return Quaternion{
            .w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            .x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            .y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            .z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        };
    }
    /// Integrates the quaternion over time using the angular velocity.
    /// This uses the formula: q_new = q + dt * 0.5 * omega_quat * q
    pub fn integrate(self: Quaternion, angularVelocity: Vec3, dt: f32) Quaternion {
        const omega = Quaternion{
            .w = 0.0,
            .x = angularVelocity.x,
            .y = angularVelocity.y,
            .z = angularVelocity.z,
        };
        const qdot = omega.mul(self).scale(0.5);
        return (Quaternion{
            .w = self.w + qdot.w * dt,
            .x = self.x + qdot.x * dt,
            .y = self.y + qdot.y * dt,
            .z = self.z + qdot.z * dt,
        }).normalized();
    }
};

pub const Joint = struct {
    bodyA: *RigidBody,
    bodyB: *RigidBody,
    anchorA: Vec3,
    anchorB: Vec3,
    distance: f32,
    pub fn solve(self: *Joint) void {
        const rA = self.anchorA;
        const rB = self.anchorB;
        const pA = self.bodyA.position.add(rA);
        const pB = self.bodyB.position.add(rB);
        const n = pB.sub(pA).normalized();
        const c = pB.sub(pA).length() - self.distance;
        const invMassSum = 1.0 / self.bodyA.mass + 1.0 / self.bodyB.mass;
        const j = n.scale(-c / invMassSum);
        self.bodyA.velocity = self.bodyA.velocity.sub(j.scale(1.0 / self.bodyA.mass));
        self.bodyB.velocity = self.bodyB.velocity.add(j.scale(1.0 / self.bodyB.mass));
    }
};

pub const RigidBody = struct {
    position: Vec3,
    velocity: Vec3,
    mass: f32,
    restitution: f32,
    rot: Quaternion,
    avel: Vec3,
    inertia: f32,
    pub fn integrate(self: *RigidBody, dt: f32) void {
        self.position = self.position.add(self.velocity.scale(dt));
    }
};

pub const Scene = struct {
    bodies: []RigidBody,
    gravity: Vec3,
    pub fn update(self: *Scene, dt: f32) void {
        for (self.bodies) |*body| {
            body.velocity = body.velocity.add(self.gravity.scale(dt));
            body.integrate(dt);
            body.rot = body.rot.integrate(body.avel, dt);
        }
    }
};

test "Vec3 operations" {
    var v1 = Vec3{ .x = 1.0, .y = 2.0, .z = 3.0 };
    var v2 = Vec3{ .x = 4.0, .y = 5.0, .z = 6.0 };

    // Test add
    const vadd = v1.add(v2);
    try expectApproxEq(f32, vadd.x, 5.0, 0.001);
    try expectApproxEq(f32, vadd.y, 7.0, 0.001);
    try expectApproxEq(f32, vadd.z, 9.0, 0.001);

    // Test sub
    const vsub = v2.sub(v1);
    try expectApproxEq(f32, vsub.x, 3.0, 0.001);
    try expectApproxEq(f32, vsub.y, 3.0, 0.001);
    try expectApproxEq(f32, vsub.z, 3.0, 0.001);

    // Test scale
    const vscale = v1.scale(2.0);
    try expectApproxEq(f32, vscale.x, 2.0, 0.001);
    try expectApproxEq(f32, vscale.y, 4.0, 0.001);
    try expectApproxEq(f32, vscale.z, 6.0, 0.001);

    // Test dot
    const dot = v1.dot(v2);
    try expectApproxEq(f32, dot, 32.0, 0.001);

    // Test cross
    const vcross = v1.cross(v2);
    try expectApproxEq(f32, vcross.x, -3.0, 0.001);
    try expectApproxEq(f32, vcross.y, 6.0, 0.001);
    try expectApproxEq(f32, vcross.z, -3.0, 0.001);

    // Test length and normalized (non-zero)
    //const len = v1.length();
    const norm = v1.normalized();
    try expectApproxEq(f32, norm.length(), 1.0, 0.001);
}

test "Quaternion integration" {
    var q = Quaternion.identity();
    const angularVel = Vec3{ .x = 0.0, .y = 1.0, .z = 0.0 };
    const dt = 0.016; // approximate frame time

    const qNew = q.integrate(angularVel, dt);
    // Check that the quaternion changed away from identity.
    try std.testing.expect(qNew.w != q.w or qNew.x != q.x or qNew.y != q.y or qNew.z != q.z);
}

test "RigidBody integration" {
    var body = RigidBody{
        .position = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .velocity = Vec3{ .x = 1.0, .y = 2.0, .z = 3.0 },
        .mass = 1.0,
        .restitution = 0.5,
        .rot = Quaternion.identity(),
        .avel = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .inertia = 1.0,
    };
    const dt = 1.0;
    body.integrate(dt);
    try expectApproxEq(f32, body.position.x, 1.0, 0.001);
    try expectApproxEq(f32, body.position.y, 2.0, 0.001);
    try expectApproxEq(f32, body.position.z, 3.0, 0.001);
}

test "Joint.solve" {
    // Setup two rigid bodies
    var bodyA = RigidBody{
        .position = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .velocity = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .mass = 1.0,
        .restitution = 0.5,
        .rot = Quaternion.identity(),
        .avel = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .inertia = 1.0,
    };

    var bodyB = RigidBody{
        .position = Vec3{ .x = 2.0, .y = 0.0, .z = 0.0 },
        .velocity = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .mass = 1.0,
        .restitution = 0.5,
        .rot = Quaternion.identity(),
        .avel = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .inertia = 1.0,
    };

    // Setup joint with anchors at zero offset and desired distance 1.0.
    // Current distance = 2.0, so constraint is violated.
    var joint = Joint{
        .bodyA = &bodyA,
        .bodyB = &bodyB,
        .anchorA = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .anchorB = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .distance = 1.0,
    };

    // Solve the joint constraint.
    joint.solve();

    // Expected impulse calculation:
    // n = (bodyB.position - bodyA.position).normalized() = (1, 0, 0)
    // c = current_distance - desired_distance = 2 - 1 = 1
    // invMassSum = 1/1 + 1/1 = 2
    // impulse = n.scale(-c / invMassSum) = (1,0,0) * (-0.5) = (-0.5, 0, 0)
    //
    // The velocities are updated as:
    // bodyA.velocity: 0 - (-0.5 / mass) = (0.5, 0, 0)
    // bodyB.velocity: 0 + (-0.5 / mass) = (-0.5, 0, 0)
    try expectApproxEq(f32, bodyA.velocity.x, 0.5, 0.001);
    try expectApproxEq(f32, bodyA.velocity.y, 0.0, 0.001);
    try expectApproxEq(f32, bodyA.velocity.z, 0.0, 0.001);

    try expectApproxEq(f32, bodyB.velocity.x, -0.5, 0.001);
    try expectApproxEq(f32, bodyB.velocity.y, 0.0, 0.001);
    try expectApproxEq(f32, bodyB.velocity.z, 0.0, 0.001);
}
test "Scene.update integrates bodies with gravity and angular velocity" {
    var bodies: [1]RigidBody = .{RigidBody{
        .position = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 },
        .velocity = Vec3{ .x = 1.0, .y = 0.0, .z = 0.0 },
        .mass = 1.0,
        .restitution = 0.5,
        .rot = Quaternion.identity(),
        .avel = Vec3{ .x = 0.0, .y = 1.0, .z = 0.0 },
        .inertia = 1.0,
    }};

    var scene = Scene{
        .bodies = bodies[0..],
        .gravity = Vec3{ .x = 0.0, .y = -9.81, .z = 0.0 },
    };

    const dt = 1.0;
    scene.update(dt);

    // Expected new velocity: (1.0, -9.81, 0.0)
    try expectApproxEq(f32, bodies[0].velocity.x, 1.0, 0.001);
    try expectApproxEq(f32, bodies[0].velocity.y, -9.81, 0.001);
    try expectApproxEq(f32, bodies[0].velocity.z, 0.0, 0.001);

    // Expected new position: (1.0, -9.81, 0.0)
    try expectApproxEq(f32, bodies[0].position.x, 1.0, 0.001);
    try expectApproxEq(f32, bodies[0].position.y, -9.81, 0.001);
    try expectApproxEq(f32, bodies[0].position.z, 0.0, 0.001);

    // Check that the rotation has been updated.
    try std.testing.expect(bodies[0].rot.w != 1.0 or
        bodies[0].rot.x != 0.0 or
        bodies[0].rot.y != 0.0 or
        bodies[0].rot.z != 0.0);
}
