// Force calculation methods

use glam::Vec3;

/// Calculate gravitational force: F = mg
pub fn gravitational_force(mass: f32, gravity: Vec3) -> Vec3 {
    mass * gravity
}

/// Calculate spring force using Hooke's law: F = -kx
pub fn spring_force(spring_constant: f32, displacement: Vec3) -> Vec3 {
    -spring_constant * displacement
}

/// Calculate damping force: F = -cv
pub fn damping_force(damping_coefficient: f32, velocity: Vec3) -> Vec3 {
    -damping_coefficient * velocity
}

/// Calculate contact force for collision response
pub fn contact_force(penetration_depth: f32, contact_normal: Vec3, contact_stiffness: f32) -> Vec3 {
    contact_stiffness * penetration_depth * contact_normal
}

/// Force accumulator for combining multiple forces
#[derive(Debug, Default, Clone)]
pub struct ForceAccumulator {
    total_force: Vec3,
}

impl ForceAccumulator {
    pub fn new() -> Self {
        Self {
            total_force: Vec3::ZERO,
        }
    }

    pub fn add_gravitational_force(&mut self, mass: f32, gravity: Vec3) {
        self.total_force += gravitational_force(mass, gravity);
    }

    pub fn add_spring_force(&mut self, spring_constant: f32, displacement: Vec3) {
        self.total_force += spring_force(spring_constant, displacement);
    }

    pub fn add_damping_force(&mut self, damping_coefficient: f32, velocity: Vec3) {
        self.total_force += damping_force(damping_coefficient, velocity);
    }

    pub fn add_contact_force(
        &mut self,
        penetration_depth: f32,
        contact_normal: Vec3,
        contact_stiffness: f32,
    ) {
        self.total_force += contact_force(penetration_depth, contact_normal, contact_stiffness);
    }

    pub fn total(&self) -> Vec3 {
        self.total_force
    }

    pub fn reset(&mut self) {
        self.total_force = Vec3::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::vec3_approx_eq;

    #[test]
    fn test_gravitational_force() {
        let mass = 10.0;
        let gravity = Vec3::new(0.0, -9.81, 0.0);

        let expected_force = mass * gravity;
        let calculated_force = gravitational_force(mass, gravity);

        assert!(vec3_approx_eq(calculated_force, expected_force, 1e-6));
    }

    #[test]
    fn test_spring_force() {
        let spring_constant = 100.0;
        let displacement = Vec3::new(0.5, 0.0, 0.0);

        let expected_force = -spring_constant * displacement;
        let calculated_force = spring_force(spring_constant, displacement);

        assert!(vec3_approx_eq(calculated_force, expected_force, 1e-6));
    }

    #[test]
    fn test_damping_force() {
        let damping_coefficient = 2.0;
        let velocity = Vec3::new(5.0, 0.0, 0.0);

        let expected_force = -damping_coefficient * velocity;
        let calculated_force = damping_force(damping_coefficient, velocity);

        assert!(vec3_approx_eq(calculated_force, expected_force, 1e-6));
    }

    #[test]
    fn test_force_accumulation() {
        let mass = 2.0;
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let spring_displacement = Vec3::new(0.1, 0.0, 0.0);
        let spring_constant = 50.0;
        let velocity = Vec3::new(1.0, 0.0, 0.0);
        let damping = 1.5;

        let gravitational_force = mass * gravity;
        let spring_force = -spring_constant * spring_displacement;
        let damping_force = -damping * velocity;
        let expected_total = gravitational_force + spring_force + damping_force;

        let mut force_accumulator = ForceAccumulator::new();
        force_accumulator.add_gravitational_force(mass, gravity);
        force_accumulator.add_spring_force(spring_constant, spring_displacement);
        force_accumulator.add_damping_force(damping, velocity);
        let total_force = force_accumulator.total();

        assert!(vec3_approx_eq(total_force, expected_total, 1e-6));
    }

    #[test]
    fn test_contact_force_normal() {
        let penetration_depth = 0.01;
        let contact_normal = Vec3::new(0.0, 1.0, 0.0);
        let contact_stiffness = 1000.0;

        let expected_force = contact_stiffness * penetration_depth * contact_normal;
        let calculated_force = contact_force(penetration_depth, contact_normal, contact_stiffness);

        assert!(vec3_approx_eq(calculated_force, expected_force, 1e-6));
    }
}
