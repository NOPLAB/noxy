# Project Overview: noxy

## Purpose
This is a Rust graphics application using wgpu for GPU-accelerated rendering. The project implements a basic 3D renderer with camera controls, following patterns from the [Learn Wgpu](https://sotrh.github.io/learn-wgpu/) tutorial. The project seems to be set up as a foundation for more advanced graphics programming.

## Tech Stack
- **Language**: Rust (Edition 2021)
- **Graphics API**: wgpu 25.0 (WebGPU API abstraction)
- **Window Management**: winit 0.30
- **Mathematics**: glam 0.30.3 (3D transformations and matrices)
- **Error Handling**: anyhow 1
- **Async Runtime**: pollster 0.4.0
- **Memory Layout**: bytemuck 1.23.0 (safe transmutation for GPU data)
- **Logging**: env_logger 0.11 + log 0.4

## Project Structure
```
noxy/
├── src/
│   ├── main.rs                    # Entry point with event loop
│   ├── app.rs                     # Application handler with multi-window support
│   └── app/
│       ├── renderer.rs            # Core wgpu rendering system
│       ├── shader.wgsl            # WGSL shaders
│       └── renderer/
│           ├── camera.rs          # 3D camera system with controls
│           └── types.rs           # Vertex definitions and geometry
├── assets/                        # Asset directory (currently empty)
├── Cargo.toml                     # Package configuration
└── CLAUDE.md                      # Development instructions for Claude
```

## Architecture
- **Multi-window Application**: Uses trait-based approach with `ApplicationWindow` for extensibility
- **Modular Renderer**: Complete wgpu pipeline with proper error handling
- **3D Camera System**: Perspective projection with WASD + Space/Shift controls
- **GPU-focused**: Built for GPU-accelerated graphics with potential for compute shaders