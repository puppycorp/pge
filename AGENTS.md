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
- See readme how to use the library
- You should use HEADLESS when testing since you dont have access top graphics pipeline or input devices.


## Testing

Like told before you should always run cargo test but after you have
made sure build and unit tests pass you should run
- HEADLESS=1 AUTOPLAY=1 ITERATIONS=1000 cargo run -p fps

which should run without crashing.

when you are testing the code you can use the DEBUG env to modify 
how much is logged see readme for descriptions. Also if you 
can temporarily add logging if you think that is nessesary for testing.
Agents can also use SCREENSHOT=1 to save rendered frames to ./workdir/screenshots for debugging. Use SCREENSHOT_INTERVAL to control how often frames are saved.
