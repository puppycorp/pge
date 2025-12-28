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

### SCREENSHOT_INTERVAL (number)

When SCREENSHOT is 1, save a frame every N renders (default 1).
