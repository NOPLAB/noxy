# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust graphics application using wgpu for GPU-accelerated rendering. The project appears to be named "pasta" (in Cargo.toml) but located in a directory called "mol". It implements a basic 3D renderer with camera controls, following patterns from the [Learn Wgpu](https://sotrh.github.io/learn-wgpu/) tutorial.

## Common Commands

- **Build**: `cargo build`
- **Run**: `cargo run`
- **Test**: `cargo test`
- **Format**: `cargo fmt`
- **Lint**: `cargo clippy`
- **Clean**: `cargo clean`

## Architecture

### Core Structure
- **main.rs**: Entry point that initializes logging and creates the winit event loop
- **app.rs**: Main application logic with multi-window support architecture
- **app/renderer.rs**: Core rendering system using wgpu
- **app/renderer/camera.rs**: Camera system with 3D controls (WASD + Space/Shift)
- **app/renderer/types.rs**: Vertex definitions and geometry data
- **app/shader.wgsl**: WGSL shaders for vertex and fragment processing

### Key Components

**Application**: Multi-window application handler that manages windows and their renderer instances. Uses a trait-based approach with `ApplicationWindow` for extensibility.

**Renderer**: Complete wgpu rendering pipeline including:
- Surface configuration and device initialization
- Vertex/index buffer management
- Camera uniform buffer and bind groups
- Render pipeline with WGSL shaders
- Frame rendering with proper error handling

**Camera System**: 3D camera with:
- Perspective projection using glam matrices
- WASD movement controls + Space/Shift for up/down
- View-projection matrix calculation with OpenGL-to-wgpu conversion

### Dependencies
- **wgpu 25.0**: Graphics API abstraction
- **winit 0.30**: Window management and event handling
- **glam 0.30**: Mathematics library for 3D transformations
- **bytemuck**: Safe transmutation for GPU data
- **anyhow**: Error handling
- **env_logger/log**: Logging infrastructure

### Controls
- WASD or Arrow Keys: Camera movement
- Space: Move camera up
- Left Shift: Move camera down

## Notes

- The project uses wgpu v25 which may have breaking changes from earlier versions
- Shader compilation uses WGSL (WebGPU Shading Language)
- The renderer implements proper surface error handling for lost/outdated surfaces
- Camera movement maintains constant distance from target when rotating