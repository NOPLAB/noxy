//! # Physics Backends
//! 
//! Abstraction layer for different computation backends (CPU, GPU).

pub mod traits;
pub mod cpu;
pub mod gpu;

pub use traits::*;

/// Available backend types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BackendType {
    /// CPU computation
    CPU,
    /// GPU computation
    GPU,
    /// Automatically select best available backend
    Auto,
}

impl Default for BackendType {
    fn default() -> Self {
        BackendType::Auto
    }
}

/// Configuration for physics backends
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BackendConfig {
    /// Which backend to use
    pub backend_type: BackendType,
    /// Number of worker threads for CPU backend
    pub cpu_threads: Option<usize>,
    /// GPU device selection (None for default)
    pub gpu_device: Option<u32>,
}

impl Default for BackendConfig {
    fn default() -> Self {
        Self {
            backend_type: BackendType::Auto,
            cpu_threads: None,
            gpu_device: None,
        }
    }
}