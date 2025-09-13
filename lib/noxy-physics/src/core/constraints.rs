//! # Constraint System
//! 
//! Constraint definitions for physics simulation.

use glam::Vec3;
use serde::{Deserialize, Serialize};
use crate::core::rigidbody::RigidBodyHandle;

/// Constraint types for physics simulation
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Constraint {
    /// Distance constraint between two bodies
    Distance {
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        distance: f32,
    },
    /// Position constraint for a single body
    Position {
        body: RigidBodyHandle,
        target_position: Vec3,
    },
    /// Joint constraint (various joint types)
    Joint {
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        joint_type: JointType,
    },
}

/// Types of joints supported
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum JointType {
    /// Fixed joint (no relative motion)
    Fixed,
    /// Revolute joint (rotation around axis)
    Revolute { axis: Vec3 },
    /// Prismatic joint (sliding along axis)
    Prismatic { axis: Vec3 },
}