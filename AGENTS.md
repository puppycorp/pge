when you make changes always try to minimize no useless whitespace changes and 
always try to keep similar code formating and other coding practices that the code has.
You should see yourself from other files.
after changes run
- cargo build --workspace
- cargo test --workspace
- cargo clippy --workspace

When making changes always consider if you need to update existing tests or create new ones.