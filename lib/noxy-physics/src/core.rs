//! # Core Physics Components
//! 
//! Fundamental physics data structures and algorithms.

pub mod rigidbody;
pub mod shapes;
pub mod forces;
pub mod materials;
pub mod constraints;
pub mod collision;
pub mod integration;

// Re-exports for convenience
pub use rigidbody::{RigidBody, RigidBodyHandle};
pub use shapes::Shape;
pub use forces::Force;
pub use materials::Material;
pub use constraints::Constraint;