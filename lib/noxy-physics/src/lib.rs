//! # Noxy Physics Library
//!
//! High-performance GPU physics simulation library with multi-solver support.
//! 
//! ## Features
//! 
//! - **Multi-solver support**: Verlet, PBD, XPBD solvers
//! - **CPU/GPU backends**: Unified interface with automatic fallback
//! - **Robotics integration**: URDF support, joint mechanisms
//! - **Real-time simulation**: Optimized for interactive applications
//! 
//! ## Architecture
//! 
//! ```text
//! ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
//! │    Solvers      │    │    Backends     │    │   Applications  │
//! │                 │    │                 │    │                 │
//! │ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
//! │ │    PBD      │ │    │ │     CPU     │ │    │ │   Rust API  │ │
//! │ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
//! │ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
//! │ │   XPBD      │ │────│ │     GPU     │ │────│ │ Python API  │ │
//! │ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
//! │ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
//! │ │   Verlet    │ │    │ │    Auto     │ │    │ │    C API    │ │
//! │ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
//! └─────────────────┘    └─────────────────┘    └─────────────────┘
//! ```
//! 
//! ## Quick Start
//! 
//! ```rust
//! use noxy_physics::prelude::*;
//! 
//! // Create simulation with PBD solver and auto backend selection
//! let mut sim = SimulationBuilder::new()
//!     .solver(SolverType::PBD)
//!     .backend(BackendType::Auto)
//!     .build()?;
//! 
//! // Add rigid bodies
//! let sphere = sim.add_rigidbody(
//!     Shape::Sphere { radius: 1.0 },
//!     Transform::from_translation(Vec3::new(0.0, 5.0, 0.0)),
//!     Material::default(),
//! )?;
//! 
//! // Run simulation
//! for _ in 0..100 {
//!     sim.step(0.016)?; // 60 FPS
//! }
//! 
//! // Get results
//! let position = sim.get_position(sphere)?;
//! println!("Final position: {:?}", position);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

// Core physics modules
pub mod core;
pub mod solvers;
pub mod backends;
pub mod scene;
pub mod utils;

#[cfg(feature = "robotics")]
pub mod robotics;

// Main simulation API
mod simulation;
pub use simulation::*;

// Convenience exports
pub mod prelude {
    pub use crate::{
        core::{
            rigidbody::{RigidBody, RigidBodyHandle},
            shapes::Shape,
            forces::Force,
            materials::Material,
        },
        solvers::{SolverType, SolverConfig},
        backends::{BackendType, BackendConfig},
        simulation::{Simulation, SimulationBuilder, SimulationConfig},
        utils::math::Transform,
    };
    
    pub use glam::{Vec3, Quat, Mat4};
    pub use anyhow::{Result, Error};
}

// Re-exports for compatibility
pub use glam;
pub use nalgebra;

#[cfg(test)]
mod integration_tests {
    use super::prelude::*;

    #[test]
    fn test_basic_simulation() {
        let mut sim = SimulationBuilder::new()
            .solver(SolverType::PBD)
            .build()
            .unwrap();
        
        let sphere = sim.add_rigidbody(
            Shape::Sphere { radius: 1.0 },
            Transform::from_translation(Vec3::new(0.0, 5.0, 0.0)),
            Material::default(),
        ).unwrap();
        
        // Apply gravity
        sim.apply_force(sphere, Force::Gravity(Vec3::new(0.0, -9.81, 0.0))).unwrap();
        
        // Step simulation
        for _ in 0..10 {
            sim.step(0.016).unwrap();
        }
        
        // Sphere should have fallen
        let final_pos = sim.get_position(sphere).unwrap();
        assert!(final_pos.y < 5.0);
    }

    #[test]
    fn test_solver_switching() {
        // Test that different solvers can be created
        for solver_type in [SolverType::Implicit, SolverType::PBD, SolverType::XPBD] {
            let sim = SimulationBuilder::new()
                .solver(solver_type)
                .build();
            assert!(sim.is_ok(), "Failed to create simulation with solver: {:?}", solver_type);
        }
    }
}