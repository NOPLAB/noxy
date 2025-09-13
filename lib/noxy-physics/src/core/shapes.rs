//! # Shape Definitions
//! 
//! Geometric shapes used in physics simulation.

use glam::Vec3;
use serde::{Deserialize, Serialize};

/// Shape types supported by the physics engine
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Shape {
    /// Sphere with given radius
    Sphere { radius: f32 },
    /// Box with dimensions (width, height, depth)
    Box { width: f32, height: f32, depth: f32 },
    /// Cylinder with radius and height
    Cylinder { radius: f32, height: f32 },
    /// Plane (infinite plane)
    Plane { normal: Vec3 },
}

impl Default for Shape {
    fn default() -> Self {
        Shape::Sphere { radius: 1.0 }
    }
}

impl Shape {
    /// Calculate volume of the shape
    pub fn volume(&self) -> f32 {
        match self {
            Shape::Sphere { radius } => (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3),
            Shape::Box { width, height, depth } => width * height * depth,
            Shape::Cylinder { radius, height } => std::f32::consts::PI * radius.powi(2) * height,
            Shape::Plane { .. } => f32::INFINITY,
        }
    }
    
    /// Calculate moment of inertia (simplified scalar version)
    pub fn moment_of_inertia(&self, mass: f32) -> f32 {
        match self {
            Shape::Sphere { radius } => (2.0 / 5.0) * mass * radius.powi(2),
            Shape::Box { width, height, depth } => {
                (1.0 / 12.0) * mass * (width.powi(2) + height.powi(2) + depth.powi(2))
            },
            Shape::Cylinder { radius, height } => {
                0.5 * mass * radius.powi(2) + (1.0 / 12.0) * mass * height.powi(2)
            },
            Shape::Plane { .. } => f32::INFINITY,
        }
    }
}