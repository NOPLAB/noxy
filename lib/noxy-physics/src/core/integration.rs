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

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Vec3, Quat};

    const EPSILON: f32 = 1e-6;

    #[test]
    fn test_verlet_integrate_zero_acceleration() {
        let pos = Vec3::new(1.0, 2.0, 3.0);
        let vel = Vec3::new(0.5, -0.5, 1.0);
        let acc = Vec3::ZERO;
        let dt = 0.016; // 60 FPS

        let (new_pos, new_vel) = verlet_integrate(pos, vel, acc, dt);
        
        // With zero acceleration, velocity should remain constant
        assert_eq!(new_vel, vel);
        // Position should change by velocity * dt
        assert!((new_pos - (pos + vel * dt)).length() < EPSILON);
    }

    #[test]
    fn test_verlet_integrate_with_gravity() {
        let pos = Vec3::new(0.0, 10.0, 0.0);
        let vel = Vec3::new(0.0, 0.0, 0.0);
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 0.1;

        let (new_pos, new_vel) = verlet_integrate(pos, vel, gravity, dt);
        
        // Velocity should increase due to gravity
        assert!((new_vel - Vec3::new(0.0, -0.981, 0.0)).length() < EPSILON);
        // Position should follow kinematic equation
        let expected_pos = pos + vel * dt + 0.5 * gravity * dt * dt;
        assert!((new_pos - expected_pos).length() < EPSILON);
    }

    #[test]
    fn test_verlet_integrate_projectile_motion() {
        let pos = Vec3::ZERO;
        let vel = Vec3::new(10.0, 10.0, 0.0); // 45 degree launch
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 0.01;

        let (new_pos, new_vel) = verlet_integrate(pos, vel, gravity, dt);
        
        // Horizontal velocity should remain constant
        assert!((new_vel.x - vel.x).abs() < EPSILON);
        // Vertical velocity should decrease
        assert!(new_vel.y < vel.y);
        // Z velocity should remain zero
        assert!(new_vel.z.abs() < EPSILON);
    }

    #[test]
    fn test_integrate_rotation_no_angular_velocity() {
        let rotation = Quat::IDENTITY;
        let angular_vel = Vec3::ZERO;
        let angular_acc = Vec3::ZERO;
        let dt = 0.016;

        let (new_rot, new_angular_vel) = integrate_rotation(rotation, angular_vel, angular_acc, dt);
        
        // Rotation should remain unchanged
        assert!((new_rot.x - rotation.x).abs() < EPSILON);
        assert!((new_rot.y - rotation.y).abs() < EPSILON);
        assert!((new_rot.z - rotation.z).abs() < EPSILON);
        assert!((new_rot.w - rotation.w).abs() < EPSILON);
        // Angular velocity should remain zero
        assert_eq!(new_angular_vel, Vec3::ZERO);
    }

    #[test]
    fn test_integrate_rotation_constant_angular_velocity() {
        let rotation = Quat::IDENTITY;
        let angular_vel = Vec3::new(0.0, 1.0, 0.0); // 1 rad/s around Y axis
        let angular_acc = Vec3::ZERO;
        let dt = 0.1;

        let (new_rot, new_angular_vel) = integrate_rotation(rotation, angular_vel, angular_acc, dt);
        
        // Angular velocity should remain constant
        assert_eq!(new_angular_vel, angular_vel);
        // Rotation should be normalized
        assert!((new_rot.length() - 1.0).abs() < EPSILON);
        // Should have rotated around Y axis
        assert!(new_rot.y.abs() > EPSILON);
    }

    #[test]
    fn test_integrate_rotation_with_acceleration() {
        let rotation = Quat::IDENTITY;
        let angular_vel = Vec3::new(0.0, 0.0, 0.0);
        let angular_acc = Vec3::new(0.0, 0.0, 2.0); // Accelerating around Z
        let dt = 0.5;

        let (new_rot, new_angular_vel) = integrate_rotation(rotation, angular_vel, angular_acc, dt);
        
        // Angular velocity should increase
        assert!((new_angular_vel - Vec3::new(0.0, 0.0, 1.0)).length() < EPSILON);
        // Rotation should be normalized
        assert!((new_rot.length() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_verlet_integrate_small_timestep_stability() {
        let pos = Vec3::new(0.0, 100.0, 0.0);
        let vel = Vec3::new(50.0, 0.0, 0.0);
        let acc = Vec3::new(0.0, -9.81, 0.0);
        let dt = 0.0001; // Very small timestep

        let (new_pos, new_vel) = verlet_integrate(pos, vel, acc, dt);
        
        // Should produce stable results even with small dt
        assert!(new_pos.is_finite());
        assert!(new_vel.is_finite());
        // Changes should be proportional to dt
        assert!((new_pos - pos).length() < 1.0);
        assert!((new_vel - vel).length() < 1.0);
    }
}