//! # Material Properties
//! 
//! Physical material definitions for simulation.

use serde::{Deserialize, Serialize};

/// Material properties for physics simulation
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Material {
    /// Coefficient of restitution (bounciness: 0.0 = no bounce, 1.0 = perfect bounce)
    pub restitution: f32,
    /// Static friction coefficient
    pub static_friction: f32,
    /// Dynamic friction coefficient
    pub dynamic_friction: f32,
    /// Density (kg/m³)
    pub density: f32,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            restitution: 0.5,
            static_friction: 0.6,
            dynamic_friction: 0.4,
            density: 1000.0, // Water density
        }
    }
}