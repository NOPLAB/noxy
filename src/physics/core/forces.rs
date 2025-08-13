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

/// Different types of forces that can be applied to rigid bodies
#[derive(Debug, Clone)]
pub enum ForceType {
    /// Constant force (e.g., applied force)
    Constant { force: Vec3 },
    /// Gravitational acceleration
    Gravity { acceleration: Vec3 },
    /// Spring force (toward a target position)
    Spring { 
        target: Vec3, 
        spring_constant: f32, 
        damping: f32,
    },
    /// Damping force (proportional to velocity)
    Damping { coefficient: f32 },
}

/// Force accumulator for combining multiple forces
#[derive(Debug, Default, Clone)]
pub struct ForceAccumulator {
    total_force: Vec3,
    forces: Vec<ForceType>,
}

impl ForceAccumulator {
    pub fn new() -> Self {
        Self {
            total_force: Vec3::ZERO,
            forces: Vec::new(),
        }
    }

    /// Add a force to the accumulator
    pub fn add_force(&mut self, force_type: ForceType) {
        self.forces.push(force_type);
    }

    /// Add a constant force vector
    pub fn add_force_vector(&mut self, force: Vec3) {
        self.total_force += force;
    }

    /// Calculate total force based on current state
    pub fn calculate_total_force(&mut self, position: Vec3, velocity: Vec3, mass: f32) -> Vec3 {
        let mut total = self.total_force;
        
        for force in &self.forces {
            match force {
                ForceType::Constant { force } => {
                    total += *force;
                },
                ForceType::Gravity { acceleration } => {
                    total += mass * acceleration;
                },
                ForceType::Spring { target, spring_constant, damping } => {
                    let displacement = position - *target;
                    let spring_force = -spring_constant * displacement;
                    let damping_force = -damping * velocity;
                    total += spring_force + damping_force;
                },
                ForceType::Damping { coefficient } => {
                    total += -coefficient * velocity;
                },
            }
        }
        
        total
    }

    /// Get current accumulated force
    pub fn get_total_force(&self) -> Vec3 {
        self.total_force
    }

    /// Clear all accumulated forces
    pub fn clear(&mut self) {
        self.total_force = Vec3::ZERO;
        self.forces.clear();
    }

    /// Check if any forces are accumulated
    pub fn has_forces(&self) -> bool {
        !self.forces.is_empty() || self.total_force != Vec3::ZERO
    }

    
    /// Reset all accumulated forces (alias for clear for compatibility)
    pub fn reset(&mut self) {
        self.clear();
    }
    
    /// Get total accumulated force (alias for get_total_force for compatibility)
    pub fn total(&self) -> Vec3 {
        self.total_force
    }
    
    /// Add gravitational force based on mass and gravity acceleration
    pub fn add_gravitational_force(&mut self, _mass: f32, gravity: Vec3) {
        self.add_force(ForceType::Gravity { 
            acceleration: gravity 
        });
    }
    
    /// Add spring force with given parameters
    pub fn add_spring_force(&mut self, spring_constant: f32, displacement: Vec3) {
        // For simple spring force, we treat displacement as relative to origin
        self.add_force_vector(-spring_constant * displacement);
    }
    
    /// Add damping force based on velocity
    pub fn add_damping_force(&mut self, damping_coefficient: f32, velocity: Vec3) {
        self.add_force_vector(-damping_coefficient * velocity);
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
        let position = Vec3::new(0.1, 0.0, 0.0); // Add position for spring force calculation

        let gravitational_force = mass * gravity;
        let spring_force = -spring_constant * spring_displacement;
        let damping_force = -damping * velocity;
        let expected_total = gravitational_force + spring_force + damping_force;

        let mut force_accumulator = ForceAccumulator::new();
        force_accumulator.add_gravitational_force(mass, gravity);
        force_accumulator.add_spring_force(spring_constant, spring_displacement);
        force_accumulator.add_damping_force(damping, velocity);
        
        // Use calculate_total_force instead of total() to properly handle ForceType forces
        let total_force = force_accumulator.calculate_total_force(position, velocity, mass);

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
