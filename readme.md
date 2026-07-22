# PGE

PuppyCorp's reusable world, rendering, application, video, and physics engine.

## Physics ownership

PGE owns the complete generic physical world and keeps the selected solver
backend private. Consumers create persistent `pge_physics::PhysicsWorld`
instances and address bodies, colliders, joints, and sensors with stable PGE
IDs. The contract covers fixed stepping, forces and impulses, body-mode and
kinematic commands, queries, ordered contact/sensor events, atomic snapshots,
checkpoints, and backend-neutral diagnostics.

The dependency direction is:

```text
product configuration -> simulation semantics -> PGE physics -> private solver
```

Robot and product meaning remains outside PGE. PGE does not interpret URDF,
model virtual devices, decide grasp or attachment policy, run task triggers, or
contain PuppyBot calibration and scenarios. RobotDreams translates those
robotics concepts into PGE physics commands and observations.

The persistent API is currently experimental (`PHYSICS_API_VERSION == 0`)
while its first stable-version review is completed. The solver dependency is a
private `pge-physics` implementation detail: PGE has no public backend reexport,
handle API, rebuild-per-step compatibility system, or second app solver.

Physics uses metres, kilograms, seconds, Newtons, and radians. Poses use a
right-handed coordinate system and quaternions in `[x, y, z, w]` order;
cylinders and capsules extend along local Y. `PhysicsConfig::fixed_dt_sec` is
the public step duration and each step is divided into its configured number of
equal substeps.

`StepInput` is the deterministic mutation boundary. Commands are applied
atomically in vector order immediately before a fixed step; a failed command
restores the complete pre-batch checkpoint and no time advances. Bounded
kinematic targets are then sampled before every substep in stable body-ID
order. Snapshots sort bodies, colliders, joints, contacts, and sensor pairs by
their stable public IDs. Same-build checkpoint continuation is tested for exact
PGE snapshot/event equality; cross-platform bitwise floating-point identity is
not promised and consumers should compare continuous values with declared
tolerances.

Every post-step snapshot also carries the authoritative collider debug
geometry and contact inspection state. Contact pairs contain deterministically
ordered manifolds and points with world witnesses, oriented normals,
penetration/distance, relative velocity, and finite solver impulses when the
backend provides them. Events include step, substep, and sequence indices.
Diagnostics report body activity, sleeping and CCD counts, pair/manifold/point
counts, resources, and total step time; backend phase timings are optional
rather than fabricated when the backend does not expose them.

Generic joints may use either independent impulse constraints or private
multibody articulation mechanics; both retain stable `JointId` ownership and
participate in snapshots, checkpoints, diagnostics, and lifecycle removal.
Joint friction is an explicit bounded Coulomb policy, not a damping alias:
`set_joint_friction` accepts a maximum resisting force in N for prismatic
joints or torque in N*m for revolute/spherical joints. Before each substep PGE
applies a sign-opposing impulse capped by both `maximum_effort * dt` and the
impulse needed to reach zero relative speed, so it cannot accelerate a joint
through rest. Break thresholds compare deterministic PGE-observed constraint
impulse/effort magnitudes after each substep, remove exceeded joints before the
next substep, and report stable-ID `JointBreakEvent`s in joint-ID order.

## ENVS

When you test the program there are some envs which you can use to affect how the PGE library behaves

### HEADLESS (1 | 0)

It will run just without graphics and input processing.

### ITERATIONS (number)

Limits the number of app ticks before exiting (headless and normal). Logs progress and exit stats.

### DEBUG (0 | 1 | 2 | 3 | 4)

- not set or 0: no logs printed
- 1: minimal logs (FPS + select initialization/exit logs)
- 2: standard debug logs
- 3: detailed timing breakdowns
- 4: verbose object dumps

### SCREENSHOT (1 | 0)

When set to 1, saves rendered frames to `./workdir/screenshots` as PNG files. Works in normal mode and in headless offscreen mode.

If no GPU adapter is available, SCREENSHOT/HEADLESS rendering will fall back to a software mock renderer so screenshot capture still succeeds (with a generated fallback image) instead of panicking on adapter creation.

### SCREENSHOT_INTERVAL (number)

When SCREENSHOT is 1, save a frame every N renders (default 1).

## Collider wireframe overlay

PGE owns a format-neutral, render-only collider overlay in `pge_core`. Enable
it on a `WorldState` and the WGPU renderer draws every native `Node::collider`
plus any backend-owned collider diagnostics supplied by the caller:

```rust
use pge_core::{
    ColliderWireframe, ColliderWireframeShape, Transform,
};

world.collider_debug.enabled = true;
world.push_collider_wireframe(ColliderWireframe::new(
    "robot-link:shoulder",
    "robotLink",
    Transform::translated([0.0, 0.0, 0.25]),
    ColliderWireframeShape::Cylinder {
        radius: 0.03,
        height: 0.18,
    },
));
```

By default the overlay also draws every native `Node::collider`. A caller
that provides authoritative live physics wireframes can prevent duplicate
diagnostics from render-scene colliders:

```rust
world.collider_debug.include_native_node_colliders = false;
```

`ColliderWireframe` carries a stable ID, category, RGBA colour, world pose,
and a box, sphere, cylinder, mesh-bounds, or recursively compound shape. The
renderer obtains the complete list with `WorldState::collider_wireframes()`.
For moving articulated physics, publish the shape layout once and update only
the compact pose frame each tick:

```rust
use pge_core::ColliderWireframePose;

world.set_collider_wireframe_pose_frame(vec![ColliderWireframePose::new(
    "robot-link:shoulder",
    current_shoulder_pose,
    [1.0, 0.25, 0.77, 1.0],
)]);
```

Increment `ColliderWireframe::shape_layout_revision` (or use
`with_shape_layout_revision`) when replacing the shape for an existing stable
ID. The WGPU renderer retains local line topology until this layout changes,
then uploads only transforms and colours on ordinary frames. Existing callers
that replace complete `ColliderWireframe` records every frame remain supported.
Cylinder heights run along local Y, matching PGE physics and Rapier's cylinder
convention.
PGE-native `Node::collider` wireframes and newly constructed generic backend
entries use yellow by default. A product may set an explicit semantic colour;
for example, RobotDreams reserves magenta for its reviewed robot-link
envelopes.
Products can update the supplied entries with their current backend poses each
frame. The overlay is not a `Node`, `Mesh`, or `PhysicsBody`; it is excluded
from physics stepping and camera-fitting by construction.
