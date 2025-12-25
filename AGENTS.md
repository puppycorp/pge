# PGE

PGE is game engine build with rust.

## General rules

- when you make changes always try to minimize no useless whitespace changes and 
  always try to keep similar code formating and other coding practices that the code has.
  You should see yourself from other files.
- after changes run
	- cargo build --workspace
	- cargo test --workspace
	- cargo clippy --workspace
- When making changes always consider if you need to update existing tests or create new ones.

## ENVS

When you test the program there are some envs which you can use to affect how the PGE library behaves

### HEADLESS (1 | 0)

It will run just without graphics and input processing. You should always use this since you cannot access graphics or input devices.

### ITERATIONS (number)

Limits the number of app ticks before exiting (headless and normal). Logs progress and exit stats.

### DEBUG (0 | 1 | 2 | 3 | 4)

- not set or 0: no logs printed
- 1: minimal logs (FPS + select initialization/exit logs)
- 2: standard debug logs
- 3: detailed timing breakdowns
- 4: verbose object dumps
