// Mathematical utility functions

use glam::Vec3;

/// Mathematical constants
pub const EPSILON: f32 = 1e-6;

/// Check if two floating point values are approximately equal
pub fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() < epsilon
}

/// Check if two Vec3 are approximately equal
pub fn vec3_approx_eq(a: Vec3, b: Vec3, epsilon: f32) -> bool {
    (a - b).length() < epsilon
}
