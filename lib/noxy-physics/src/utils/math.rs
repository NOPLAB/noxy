//! # Mathematical Utilities
//! 
//! Common mathematical functions and data structures.

use glam::{Vec3, Quat, Mat4};
use serde::{Deserialize, Serialize};

/// 3D transformation (position + rotation)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Transform {
    /// Translation component
    pub translation: Vec3,
    /// Rotation component (quaternion)
    pub rotation: Quat,
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }
    }
}

impl Transform {
    /// Create a new transform
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self { translation, rotation }
    }
    
    /// Create a transform from translation only
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation,
            rotation: Quat::IDENTITY,
        }
    }
    
    /// Create a transform from rotation only
    pub fn from_rotation(rotation: Quat) -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation,
        }
    }
    
    /// Convert to a 4x4 transformation matrix
    pub fn to_matrix(&self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation, self.translation)
    }
    
    /// Apply this transform to a point
    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.rotation * point + self.translation
    }
    
    /// Apply this transform to a vector (rotation only)
    pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
        self.rotation * vector
    }
    
    /// Combine two transforms
    pub fn combine(&self, other: &Transform) -> Transform {
        Transform {
            translation: self.translation + self.rotation * other.translation,
            rotation: self.rotation * other.rotation,
        }
    }
    
    /// Get the inverse transform
    pub fn inverse(&self) -> Transform {
        let inv_rotation = self.rotation.conjugate();
        Transform {
            translation: inv_rotation * (-self.translation),
            rotation: inv_rotation,
        }
    }
}

/// Linear interpolation between two values
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Clamp a value between min and max
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Check if two floating point values are approximately equal
pub fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() < epsilon
}

/// Check if two Vec3 values are approximately equal
pub fn vec3_approx_eq(a: Vec3, b: Vec3, epsilon: f32) -> bool {
    (a - b).length() < epsilon
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform_creation() {
        let t = Transform::from_translation(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(t.translation, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(t.rotation, Quat::IDENTITY);
    }

    #[test]
    fn test_transform_point() {
        let t = Transform::from_translation(Vec3::new(1.0, 0.0, 0.0));
        let point = Vec3::new(0.0, 0.0, 0.0);
        let transformed = t.transform_point(point);
        assert_eq!(transformed, Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn test_transform_combine() {
        let t1 = Transform::from_translation(Vec3::new(1.0, 0.0, 0.0));
        let t2 = Transform::from_translation(Vec3::new(0.0, 1.0, 0.0));
        let combined = t1.combine(&t2);
        assert_eq!(combined.translation, Vec3::new(1.0, 1.0, 0.0));
    }

    #[test]
    fn test_approx_eq() {
        assert!(approx_eq(1.0, 1.0001, 0.001));
        assert!(!approx_eq(1.0, 1.1, 0.001));
    }
}