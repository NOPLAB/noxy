# Suggested Commands for noxy Development

## Build & Run Commands
- `cargo build` - Build the project
- `cargo run` - Build and run the application
- `cargo test` - Run tests
- `cargo clean` - Clean build artifacts

## Code Quality Commands
- `cargo fmt` - Format code according to Rust style guidelines
- `cargo clippy` - Run Rust linter for additional warnings and suggestions
- `cargo check` - Fast compilation check without producing binaries

## Development Commands
- `cargo build --release` - Build optimized release version
- `cargo run --release` - Run optimized release version
- `RUST_LOG=debug cargo run` - Run with debug logging
- `RUST_LOG=trace cargo run` - Run with verbose logging

## Git Commands
- `git status` - Check repository status
- `git add .` - Stage all changes
- `git commit -m "message"` - Commit changes
- `git push` - Push to remote repository

## System Commands (Linux)
- `ls` - List directory contents
- `cd <directory>` - Change directory
- `find . -name "*.rs"` - Find Rust source files
- `grep -r "pattern" src/` - Search for patterns in source code
- `tree` - Display directory structure (if installed)