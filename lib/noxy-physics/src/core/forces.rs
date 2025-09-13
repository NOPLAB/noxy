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