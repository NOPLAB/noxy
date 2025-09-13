//! # Rigid Body Physics Implementation
//!
//! This module contains the core rigid body data structures and physics calculations
//! for 3D rigid body dynamics simulation.

use glam::{Mat3, Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Handle for referencing rigid bodies in the simulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RigidBodyHandle(pub u64);

/// Rigid body representation for 3D physics simulation
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RigidBody {
    /// Position in 3D space
    pub position: Vec3,
    /// Orientation as quaternion
    pub rotation: Quat,
    /// Linear velocity
    pub velocity: Vec3,
    /// Angular velocity
    pub angular_velocity: Vec3,
    
    /// Mass of the body
    pub mass: f32,
    /// Inverse mass for efficient calculations (0.0 for infinite mass)
    pub inverse_mass: f32,
    /// Moment of inertia in body space
    pub inertia: f32, // Simplified scalar inertia for now
    /// Inverse inertia for efficient calculations
    pub inverse_inertia: f32,
    
    /// Body shape type
    pub shape: crate::core::shapes::Shape,
    /// Material properties
    pub material: crate::core::materials::Material,
    
    /// Whether the body is active (participates in simulation)
    pub is_active: bool,
    /// Whether the body is sleeping (optimization for resting bodies)
    pub is_sleeping: bool,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inverse_mass: 1.0,
            inertia: 1.0,
            inverse_inertia: 1.0,
            shape: crate::core::shapes::Shape::Sphere { radius: 1.0 },
            material: crate::core::materials::Material::default(),
            is_active: true,
            is_sleeping: false,
        }
    }
}

impl RigidBody {
    /// Create a new rigid body with given properties
    pub fn new(
        position: Vec3,
        rotation: Quat,
        mass: f32,
        inertia: f32,
        shape: crate::core::shapes::Shape,
        material: crate::core::materials::Material,
    ) -> Self {
        let inverse_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let inverse_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
        
        Self {
            position,
            rotation,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            shape,
            material,
            is_active: true,
            is_sleeping: false,
        }
    }
    
    /// Set the mass and update inverse mass
    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.inverse_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
    }
    
    /// Set the inertia and update inverse inertia
    pub fn set_inertia(&mut self, inertia: f32) {
        self.inertia = inertia;
        self.inverse_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
    }
    
    /// Check if the body has infinite mass (static body)
    pub fn is_static(&self) -> bool {
        self.mass <= 0.0 || self.inverse_mass == 0.0
    }
    
    /// Calculate kinetic energy of the rigid body
    pub fn kinetic_energy(&self) -> f32 {
        let linear_ke = 0.5 * self.mass * self.velocity.length_squared();
        let angular_ke = 0.5 * self.inertia * self.angular_velocity.length_squared();
        linear_ke + angular_ke
    }
    
    /// Apply an impulse to the rigid body
    pub fn apply_impulse(&mut self, impulse: Vec3, point: Vec3) {
        if self.is_static() {
            return;
        }
        
        // Apply linear impulse
        self.velocity += impulse * self.inverse_mass;
        
        // Apply angular impulse
        let r = point - self.position;
        let angular_impulse = r.cross(impulse);
        self.angular_velocity += angular_impulse * self.inverse_inertia;
    }
    
    /// Get velocity at a point on the rigid body
    pub fn velocity_at_point(&self, point: Vec3) -> Vec3 {
        let r = point - self.position;
        self.velocity + self.angular_velocity.cross(r)
    }
}

/// Calculate inertia tensor for a solid sphere
/// Formula: I = (2/5) * m * r²
pub fn sphere_inertia_tensor(mass: f32, radius: f32) -> Mat3 {
    let inertia = (2.0 / 5.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::splat(inertia))
}

/// Calculate inertia tensor for a box
/// Formula: Ixx = (1/12)*m*(h²+d²), Iyy = (1/12)*m*(w²+d²), Izz = (1/12)*m*(w²+h²)
pub fn box_inertia_tensor(mass: f32, dimensions: Vec3) -> Mat3 {
    let w = dimensions.x;
    let h = dimensions.y;
    let d = dimensions.z;

    let ixx = (1.0 / 12.0) * mass * (h * h + d * d);
    let iyy = (1.0 / 12.0) * mass * (w * w + d * d);
    let izz = (1.0 / 12.0) * mass * (w * w + h * h);

    Mat3::from_diagonal(Vec3::new(ixx, iyy, izz))
}

/// Calculate kinetic energy: KE = 0.5*m*v² + 0.5*ω·I·ω
pub fn calculate_kinetic_energy(
    mass: f32,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    inertia_tensor: Mat3,
) -> f32 {
    let linear_ke = 0.5 * mass * linear_velocity.length_squared();
    let angular_ke = 0.5 * angular_velocity.dot(inertia_tensor * angular_velocity);
    linear_ke + angular_ke
}

/// Convert force to linear acceleration: a = F/m
pub fn force_to_linear_acceleration(force: Vec3, mass: f32) -> Vec3 {
    force / mass
}

/// Convert torque to angular acceleration: α = I⁻¹ * τ
pub fn torque_to_angular_acceleration(torque: Vec3, inertia_tensor: Mat3) -> Vec3 {
    inertia_tensor.inverse() * torque
}

/// Transform inertia tensor from body space to world space
/// I_world = R * I_body * R^T
pub fn transform_inertia_tensor(inertia_tensor: Mat3, orientation: Quat) -> Mat3 {
    let rotation_matrix = Mat3::from_quat(orientation);
    rotation_matrix * inertia_tensor * rotation_matrix.transpose()
}

/// Integration step for rigid body rotation
/// Returns (new_orientation, new_angular_velocity)
pub fn integrate_rotation(
    orientation: Quat,
    angular_velocity: Vec3,
    inertia_tensor: Mat3,
    torque: Vec3,
    dt: f32,
) -> (Quat, Vec3) {
    // Calculate angular acceleration
    let angular_acceleration = torque_to_angular_acceleration(torque, inertia_tensor);

    // Update angular velocity
    let new_angular_velocity = angular_velocity + angular_acceleration * dt;

    // Update orientation using quaternion integration
    let angular_velocity_quat = Quat::from_xyzw(
        angular_velocity.x * 0.5,
        angular_velocity.y * 0.5,
        angular_velocity.z * 0.5,
        0.0,
    );

    let new_orientation = (orientation + angular_velocity_quat * orientation * dt).normalize();

    (new_orientation, new_angular_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() < epsilon
    }

    fn vec3_approx_eq(a: Vec3, b: Vec3, epsilon: f32) -> bool {
        (a - b).length() < epsilon
    }

    #[test]
    fn test_moment_of_inertia_sphere() {
        let mass = 10.0;
        let radius = 2.0;

        let expected_inertia = (2.0 / 5.0) * mass * radius * radius;
        let expected_tensor = Mat3::from_diagonal(Vec3::splat(expected_inertia));
        let calculated_tensor = sphere_inertia_tensor(mass, radius);

        for i in 0..3 {
            for j in 0..3 {
                assert!(approx_eq(
                    calculated_tensor.col(j)[i],
                    expected_tensor.col(j)[i],
                    1e-6
                ));
            }
        }
    }

    #[test]
    fn test_moment_of_inertia_box() {
        let mass = 8.0;
        let dimensions = Vec3::new(2.0, 4.0, 6.0);

        let ixx = (1.0 / 12.0) * mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z);
        let iyy = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z);
        let izz = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y);

        let expected_tensor = Mat3::from_diagonal(Vec3::new(ixx, iyy, izz));
        let calculated_tensor = box_inertia_tensor(mass, dimensions);

        for i in 0..3 {
            for j in 0..3 {
                assert!(approx_eq(
                    calculated_tensor.col(j)[i],
                    expected_tensor.col(j)[i],
                    1e-6
                ));
            }
        }
    }

    #[test]
    fn test_rigid_body_energy_calculation() {
        let mass = 5.0;
        let linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        let angular_velocity = Vec3::new(0.0, 1.0, 0.0);
        let inertia_tensor = Mat3::from_diagonal(Vec3::new(2.0, 4.0, 6.0));

        let expected_linear_ke = 0.5 * mass * linear_velocity.length_squared();
        let expected_angular_ke = 0.5 * angular_velocity.dot(inertia_tensor * angular_velocity);
        let expected_total_ke = expected_linear_ke + expected_angular_ke;

        let calculated_ke =
            calculate_kinetic_energy(mass, linear_velocity, angular_velocity, inertia_tensor);

        assert!(approx_eq(calculated_ke, expected_total_ke, 1e-6));
    }

    #[test]
    fn test_rigid_body_creation() {
        let position = Vec3::new(1.0, 2.0, 3.0);
        let rotation = Quat::IDENTITY;
        let mass = 5.0;
        let inertia = 2.0;
        let shape = crate::core::shapes::Shape::Sphere { radius: 1.0 };
        let material = crate::core::materials::Material::default();
        
        let body = RigidBody::new(position, rotation, mass, inertia, shape, material);
        
        assert_eq!(body.position, position);
        assert_eq!(body.mass, mass);
        assert!(approx_eq(body.inverse_mass, 1.0 / mass, 1e-6));
        assert!(!body.is_static());
    }

    #[test]
    fn test_static_body() {
        let mut body = RigidBody::default();
        body.set_mass(0.0);
        
        assert!(body.is_static());
        assert_eq!(body.inverse_mass, 0.0);
    }

    #[test]
    fn test_impulse_application() {
        let mut body = RigidBody::default();
        body.set_mass(2.0);
        body.set_inertia(1.0);
        
        let impulse = Vec3::new(10.0, 0.0, 0.0);
        let point = Vec3::new(0.0, 1.0, 0.0);
        
        body.apply_impulse(impulse, point);
        
        // Linear velocity should be impulse / mass
        let expected_velocity = impulse / body.mass;
        assert!(vec3_approx_eq(body.velocity, expected_velocity, 1e-6));
        
        // Angular velocity should be affected by torque
        assert!(body.angular_velocity.length() > 0.0);
    }
}