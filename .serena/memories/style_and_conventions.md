# Code Style and Conventions for noxy

## Rust Conventions
- **Edition**: Rust 2021
- **Formatting**: Use `cargo fmt` for consistent formatting
- **Linting**: Use `cargo clippy` for additional code quality checks
- **Naming**: Follow Rust naming conventions:
  - `snake_case` for functions, variables, modules
  - `PascalCase` for types, structs, enums, traits
  - `SCREAMING_SNAKE_CASE` for constants

## Project-Specific Patterns
- **Error Handling**: Use `anyhow::Result<()>` for error propagation
- **Logging**: Use `log` crate with `env_logger` for runtime logging
- **GPU Data**: Use `bytemuck` for safe memory layout with GPU
- **Architecture**: 
  - Trait-based design for extensibility (`ApplicationWindow` trait)
  - Modular structure with separate renderer components
  - Clear separation between app logic and rendering

## Code Organization
- **Modules**: Clear module hierarchy in `src/app/` for renderer components
- **Traits**: Use traits for abstractions (e.g., `ApplicationWindow`)
- **Error Handling**: Consistent use of `?` operator with `anyhow`
- **Resource Management**: RAII principles for GPU resources

## Documentation
- Follow standard Rust doc comments with `///`
- Include examples in documentation where appropriate
- Use `CLAUDE.md` for project-specific development instructions