// Noxy Physics Simulation Library
// GPU-accelerated rigid body physics with Rust + WGPU

pub mod app;
pub mod physics;
pub mod utils;

// Re-export commonly used types
pub use glam::{Mat3, Mat4, Quat, Vec3};

// Library version and metadata
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const NAME: &str = env!("CARGO_PKG_NAME");
