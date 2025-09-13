//! # Numerical Integration
//! 
//! Integration schemes for physics simulation.

use glam::{Vec3, Quat};

/// Verlet integration for position and velocity
pub fn verlet_integrate(
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    dt: f32,
) -> (Vec3, Vec3) {
    let new_position = position + velocity * dt + 0.5 * acceleration * dt * dt;
    let new_velocity = velocity + acceleration * dt;
    (new_position, new_velocity)
}

/// Integrate rotation using quaternions
pub fn integrate_rotation(
    rotation: Quat,
    angular_velocity: Vec3,
    angular_acceleration: Vec3,
    dt: f32,
) -> (Quat, Vec3) {
    let new_angular_velocity = angular_velocity + angular_acceleration * dt;
    
    // Simple quaternion integration
    let angular_velocity_quat = Quat::from_xyzw(
        angular_velocity.x * 0.5,
        angular_velocity.y * 0.5,
        angular_velocity.z * 0.5,
        0.0,
    );
    
    let new_rotation = (rotation + angular_velocity_quat * rotation * dt).normalize();
    
    (new_rotation, new_angular_velocity)
}