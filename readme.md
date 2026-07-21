# PGE

Game engine

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

`ColliderWireframe` carries a stable ID, category, RGBA colour, world pose,
and a box, sphere, cylinder, mesh-bounds, or recursively compound shape. The
renderer obtains the complete list with `WorldState::collider_wireframes()`.
Products can update the supplied entries with their current backend poses each
frame. The overlay is not a `Node`, `Mesh`, or `PhysicsBody`; it is excluded
from physics stepping and camera-fitting by construction.
