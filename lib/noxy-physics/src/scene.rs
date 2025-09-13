//! # Scene Management
//! 
//! Scene configuration and loading functionality.

use crate::core::{rigidbody::RigidBody, shapes::Shape, materials::Material};
use glam::Vec3;

/// Scene configuration
#[derive(Debug, Clone)]
pub struct Scene {
    /// Scene name
    pub name: String,
    /// Initial rigid bodies
    pub rigid_bodies: Vec<RigidBodyConfig>,
    /// Global gravity
    pub gravity: Vec3,
}

/// Configuration for a rigid body in a scene
#[derive(Debug, Clone)]
pub struct RigidBodyConfig {
    /// Body position
    pub position: Vec3,
    /// Body shape
    pub shape: Shape,
    /// Body material
    pub material: Material,
    /// Body mass (None for auto-calculation from density)
    pub mass: Option<f32>,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            name: "Default Scene".to_string(),
            rigid_bodies: Vec::new(),
            gravity: Vec3::new(0.0, -9.81, 0.0),
        }
    }
}