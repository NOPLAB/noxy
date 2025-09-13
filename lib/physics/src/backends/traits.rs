//! # Backend Traits
//! 
//! Unified interface for physics computation backends.

use crate::core::rigidbody::{RigidBody, RigidBodyHandle};
use anyhow::Result;

/// Information about a backend
#[derive(Debug, Clone)]
pub struct BackendInfo {
    /// Human-readable name
    pub name: String,
    /// Backend type
    pub backend_type: super::BackendType,
    /// Maximum number of bodies it can handle efficiently
    pub max_bodies: usize,
    /// Whether it supports specific features
    pub features: BackendFeatures,
}

/// Features supported by a backend
#[derive(Debug, Clone)]
pub struct BackendFeatures {
    /// Supports parallel computation
    pub parallel: bool,
    /// Supports GPU acceleration
    pub gpu_acceleration: bool,
    /// Supports double precision
    pub double_precision: bool,
}

/// Trait for physics computation backends
pub trait PhysicsBackend: Send + Sync {
    /// Get information about this backend
    fn info(&self) -> BackendInfo;
    
    /// Initialize the backend
    fn initialize(&mut self) -> Result<()>;
    
    /// Add a rigid body to the simulation
    fn add_rigidbody(&mut self, body: RigidBody) -> Result<RigidBodyHandle>;
    
    /// Remove a rigid body from the simulation
    fn remove_rigidbody(&mut self, handle: RigidBodyHandle) -> Result<()>;
    
    /// Update rigid body data
    fn update_rigidbody(&mut self, handle: RigidBodyHandle, body: &RigidBody) -> Result<()>;
    
    /// Get rigid body data
    fn get_rigidbody(&self, handle: RigidBodyHandle) -> Result<RigidBody>;
    
    /// Perform one physics step
    fn step(&mut self, dt: f32) -> Result<()>;
    
    /// Synchronize data between backend and host
    fn synchronize(&mut self) -> Result<()>;
}