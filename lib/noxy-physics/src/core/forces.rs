//! # Force System
//! 
//! Force definitions and force application.

use glam::Vec3;
use serde::{Deserialize, Serialize};

/// Force types that can be applied to rigid bodies
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Force {
    /// Linear force
    Linear(Vec3),
    /// Torque (angular force)
    Torque(Vec3),
    /// Gravity force
    Gravity(Vec3),
}

impl Force {
    /// Calculate total force from a collection of forces
    pub fn total_linear(forces: &[Force]) -> Vec3 {
        forces.iter().fold(Vec3::ZERO, |acc, f| match f {
            Force::Linear(v) | Force::Gravity(v) => acc + *v,
            Force::Torque(_) => acc,
        })
    }

    /// Calculate total torque from a collection of forces
    pub fn total_torque(forces: &[Force]) -> Vec3 {
        forces.iter().fold(Vec3::ZERO, |acc, f| match f {
            Force::Torque(t) => acc + *t,
            _ => acc,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn test_force_creation() {
        let linear = Force::Linear(Vec3::new(1.0, 2.0, 3.0));
        let torque = Force::Torque(Vec3::new(0.1, 0.2, 0.3));
        let gravity = Force::Gravity(Vec3::new(0.0, -9.81, 0.0));

        assert_eq!(linear, Force::Linear(Vec3::new(1.0, 2.0, 3.0)));
        assert_eq!(torque, Force::Torque(Vec3::new(0.1, 0.2, 0.3)));
        assert_eq!(gravity, Force::Gravity(Vec3::new(0.0, -9.81, 0.0)));
    }

    #[test]
    fn test_total_linear_force() {
        let forces = vec![
            Force::Linear(Vec3::new(1.0, 0.0, 0.0)),
            Force::Linear(Vec3::new(0.0, 1.0, 0.0)),
            Force::Gravity(Vec3::new(0.0, -9.81, 0.0)),
            Force::Torque(Vec3::new(0.0, 0.0, 1.0)), // Should be ignored
        ];

        let total = Force::total_linear(&forces);
        assert_eq!(total, Vec3::new(1.0, 1.0 - 9.81, 0.0));
    }

    #[test]
    fn test_total_torque() {
        let forces = vec![
            Force::Torque(Vec3::new(1.0, 0.0, 0.0)),
            Force::Torque(Vec3::new(0.0, 1.0, 0.0)),
            Force::Linear(Vec3::new(10.0, 10.0, 10.0)), // Should be ignored
            Force::Gravity(Vec3::new(0.0, -9.81, 0.0)), // Should be ignored
        ];

        let total = Force::total_torque(&forces);
        assert_eq!(total, Vec3::new(1.0, 1.0, 0.0));
    }

    #[test]
    fn test_empty_forces() {
        let forces: Vec<Force> = vec![];
        assert_eq!(Force::total_linear(&forces), Vec3::ZERO);
        assert_eq!(Force::total_torque(&forces), Vec3::ZERO);
    }
}