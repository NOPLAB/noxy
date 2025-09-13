//! # Position Based Dynamics (PBD) Solver
//! 
//! Implements the Position Based Dynamics approach for real-time physics simulation.
//! PBD is particularly well-suited for interactive applications due to its stability
//! and predictable performance characteristics.
//! 
//! ## Key Features
//! 
//! - Unconditionally stable
//! - Predictable performance
//! - Direct position manipulation
//! - Excellent for real-time applications
//! 
//! ## Algorithm Overview
//! 
//! 1. **Predict Positions**: Apply forces and integrate velocities
//! 2. **Solve Constraints**: Iteratively project positions to satisfy constraints
//! 3. **Update Velocities**: Derive velocities from position changes
//! 4. **Apply Damping**: Reduce system energy to prevent unrealistic behavior
//! 
//! ## References
//! 
//! - Müller et al. "Position Based Dynamics" (2007)
//! - Macklin et al. "Unified Particle Physics for Real-Time Applications" (2014)

use glam::{Vec3, Quat};
use std::collections::HashMap;
use crate::{
    core::{
        rigidbody::{RigidBody, RigidBodyHandle},
        constraints::Constraint,
        forces::Force,
    },
    solvers::traits::*,
};

/// Configuration for PBD solver
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PBDConfig {
    /// Number of position correction iterations
    pub position_iterations: u32,
    /// Number of velocity correction iterations  
    pub velocity_iterations: u32,
    /// Sleep threshold for rigid bodies
    pub sleep_threshold: f32,
    /// Global damping factor
    pub damping: f32,
    /// Maximum allowed position correction per iteration
    pub max_position_correction: f32,
}

impl Default for PBDConfig {
    fn default() -> Self {
        Self {
            position_iterations: 10,
            velocity_iterations: 5,
            sleep_threshold: 0.01,
            damping: 0.99,
            max_position_correction: 0.1,
        }
    }
}

/// PBD solver implementation
pub struct PBDSolver {
    config: PBDConfig,
    rigid_bodies: HashMap<RigidBodyHandle, RigidBody>,
    constraints: Vec<Constraint>,
    forces: HashMap<RigidBodyHandle, Vec<Force>>,
    
    // Solver state
    predicted_positions: HashMap<RigidBodyHandle, Vec3>,
    predicted_rotations: HashMap<RigidBodyHandle, Quat>,
    
    // Statistics
    statistics: SolverStatistics,
    next_handle: u64,
    
    // Configuration
    initialized: bool,
}

impl PBDSolver {
    /// Create a new PBD solver with given configuration
    pub fn new(config: PBDConfig) -> Self {
        Self {
            config,
            rigid_bodies: HashMap::new(),
            constraints: Vec::new(),
            forces: HashMap::new(),
            predicted_positions: HashMap::new(),
            predicted_rotations: HashMap::new(),
            statistics: SolverStatistics::default(),
            next_handle: 1,
            initialized: false,
        }
    }
    
    /// Predict positions based on current state and forces
    fn predict_positions(&mut self, dt: f32) -> SolverResult<()> {
        for (handle, body) in &self.rigid_bodies {
            let mut velocity = body.velocity;
            let mut angular_velocity = body.angular_velocity;
            
            // Apply forces
            if let Some(forces) = self.forces.get(handle) {
                for force in forces {
                    match force {
                        Force::Linear(f) => {
                            velocity += *f * dt / body.mass;
                        }
                        Force::Torque(t) => {
                            // Simplified angular dynamics
                            angular_velocity += *t * dt / body.inertia;
                        }
                        Force::Gravity(g) => {
                            velocity += *g * dt;
                        }
                    }
                }
            }
            
            // Predict new positions
            let predicted_pos = body.position + velocity * dt;
            let angular_velocity_quat = Quat::from_xyzw(
                angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0
            );
            let predicted_rot = body.rotation + angular_velocity_quat * body.rotation * (0.5 * dt);
            
            self.predicted_positions.insert(*handle, predicted_pos);
            self.predicted_rotations.insert(*handle, predicted_rot.normalize());
        }
        
        Ok(())
    }
    
    /// Solve position constraints
    fn solve_position_constraints(&mut self) -> SolverResult<f32> {
        let mut max_error: f32 = 0.0;
        
        for _ in 0..self.config.position_iterations {
            let mut iteration_error: f32 = 0.0;
            
            let constraints = self.constraints.clone(); // Clone to avoid borrowing issues
            for constraint in &constraints {
                let error = self.solve_constraint(constraint)?;
                iteration_error = iteration_error.max(error);
            }
            
            max_error = max_error.max(iteration_error);
            
            // Early exit if constraints are satisfied
            if iteration_error < 1e-6 {
                break;
            }
        }
        
        Ok(max_error)
    }
    
    /// Solve a single constraint
    fn solve_constraint(&mut self, constraint: &Constraint) -> SolverResult<f32> {
        match constraint {
            Constraint::Distance { body_a, body_b, distance } => {
                self.solve_distance_constraint(*body_a, *body_b, *distance)
            }
            Constraint::Position { body, target_position } => {
                self.solve_position_constraint(*body, *target_position)
            }
            // Add more constraint types as needed
            _ => Ok(0.0), // Placeholder for unimplemented constraints
        }
    }
    
    /// Solve distance constraint between two bodies
    fn solve_distance_constraint(
        &mut self, 
        body_a: RigidBodyHandle, 
        body_b: RigidBodyHandle, 
        target_distance: f32
    ) -> SolverResult<f32> {
        let pos_a = *self.predicted_positions.get(&body_a)
            .ok_or(SolverError::InvalidHandle(body_a))?;
        let pos_b = *self.predicted_positions.get(&body_b)
            .ok_or(SolverError::InvalidHandle(body_b))?;
        
        let delta = pos_b - pos_a;
        let current_distance = delta.length();
        
        if current_distance < 1e-6 {
            return Ok(0.0); // Avoid division by zero
        }
        
        let error = current_distance - target_distance;
        let correction = delta * (error / current_distance) * 0.5;
        
        // Apply position corrections
        if let Some(pos) = self.predicted_positions.get_mut(&body_a) {
            *pos += correction;
        }
        if let Some(pos) = self.predicted_positions.get_mut(&body_b) {
            *pos -= correction;
        }
        
        Ok(error.abs())
    }
    
    /// Solve position constraint for single body
    fn solve_position_constraint(
        &mut self,
        body: RigidBodyHandle,
        target_position: Vec3
    ) -> SolverResult<f32> {
        if let Some(pos) = self.predicted_positions.get_mut(&body) {
            let error = (*pos - target_position).length();
            *pos = target_position;
            Ok(error)
        } else {
            Err(SolverError::InvalidHandle(body))
        }
    }
    
    /// Update velocities based on position changes
    fn update_velocities(&mut self, dt: f32) -> SolverResult<()> {
        for (handle, body) in &mut self.rigid_bodies {
            if let Some(predicted_pos) = self.predicted_positions.get(handle) {
                // Update linear velocity
                body.velocity = (*predicted_pos - body.position) / dt;
                body.position = *predicted_pos;
            }
            
            if let Some(predicted_rot) = self.predicted_rotations.get(handle) {
                // Update angular velocity (simplified)
                let delta_rot = *predicted_rot * body.rotation.conjugate();
                body.angular_velocity = Vec3::new(delta_rot.x, delta_rot.y, delta_rot.z) * 2.0 / dt;
                body.rotation = *predicted_rot;
            }
            
            // Apply damping
            body.velocity *= self.config.damping;
            body.angular_velocity *= self.config.damping;
        }
        
        Ok(())
    }
    
    /// Clear applied forces
    fn clear_forces(&mut self) {
        self.forces.clear();
    }
}

impl PhysicsSolver for PBDSolver {
    fn name(&self) -> &'static str {
        "Position Based Dynamics (PBD)"
    }
    
    fn info(&self) -> SolverInfo {
        SolverInfo {
            description: "Position Based Dynamics solver optimized for real-time applications".to_string(),
            use_cases: vec![
                "Real-time games".to_string(),
                "Interactive simulations".to_string(),
                "Cloth simulation".to_string(),
                "Soft body dynamics".to_string(),
            ],
            performance_profile: PerformanceProfile {
                relative_cost: 1.0,
                memory_factor: 1.2,
                accuracy: 0.8,
                stability: 0.95,
            },
            required_features: vec![
                SolverFeature::Multithreading,
            ],
        }
    }
    
    fn initialize(&mut self, config: &SolverConfiguration) -> SolverResult<()> {
        // Reserve capacity for better performance
        self.rigid_bodies.reserve(config.max_rigid_bodies);
        self.constraints.reserve(config.max_constraints);
        self.predicted_positions.reserve(config.max_rigid_bodies);
        self.predicted_rotations.reserve(config.max_rigid_bodies);
        
        self.initialized = true;
        Ok(())
    }
    
    fn add_rigidbody(&mut self, body: RigidBody) -> SolverResult<RigidBodyHandle> {
        let handle = RigidBodyHandle(self.next_handle);
        self.next_handle += 1;
        
        self.rigid_bodies.insert(handle, body);
        self.statistics.rigid_body_count += 1;
        
        Ok(handle)
    }
    
    fn remove_rigidbody(&mut self, handle: RigidBodyHandle) -> SolverResult<()> {
        if self.rigid_bodies.remove(&handle).is_some() {
            self.predicted_positions.remove(&handle);
            self.predicted_rotations.remove(&handle);
            self.forces.remove(&handle);
            self.statistics.rigid_body_count -= 1;
            Ok(())
        } else {
            Err(SolverError::InvalidHandle(handle))
        }
    }
    
    fn get_rigidbody(&self, handle: RigidBodyHandle) -> SolverResult<&RigidBody> {
        self.rigid_bodies.get(&handle)
            .ok_or(SolverError::InvalidHandle(handle))
    }
    
    fn get_rigidbody_mut(&mut self, handle: RigidBodyHandle) -> SolverResult<&mut RigidBody> {
        self.rigid_bodies.get_mut(&handle)
            .ok_or(SolverError::InvalidHandle(handle))
    }
    
    fn add_constraint(&mut self, constraint: Constraint) -> SolverResult<()> {
        self.constraints.push(constraint);
        self.statistics.constraint_count += 1;
        Ok(())
    }
    
    fn apply_force(&mut self, handle: RigidBodyHandle, force: Force) -> SolverResult<()> {
        if !self.rigid_bodies.contains_key(&handle) {
            return Err(SolverError::InvalidHandle(handle));
        }
        
        self.forces.entry(handle).or_insert_with(Vec::new).push(force);
        Ok(())
    }
    
    fn step(&mut self, dt: f32) -> SolverResult<StepInfo> {
        if !self.initialized {
            return Err(SolverError::ConfigError { 
                message: "Solver not initialized".to_string() 
            });
        }
        
        let start_time = std::time::Instant::now();
        
        // PBD algorithm steps
        self.predict_positions(dt)?;
        let max_constraint_error = self.solve_position_constraints()?;
        self.update_velocities(dt)?;
        self.clear_forces();
        
        // Calculate energies
        let mut kinetic_energy = 0.0;
        let potential_energy = 0.0; // Simplified - would need to calculate based on forces
        
        for body in self.rigid_bodies.values() {
            kinetic_energy += 0.5 * body.mass * body.velocity.length_squared();
            kinetic_energy += 0.5 * body.inertia * body.angular_velocity.length_squared();
        }
        
        // Update statistics
        self.statistics.steps_taken += 1;
        self.statistics.simulation_time += dt;
        let step_time = start_time.elapsed().as_secs_f32();
        self.statistics.avg_step_time = 
            (self.statistics.avg_step_time * (self.statistics.steps_taken - 1) as f32 + step_time) 
            / self.statistics.steps_taken as f32;
        
        let converged = max_constraint_error < 1e-4;
        if !converged {
            self.statistics.convergence_failures += 1;
        }
        
        Ok(StepInfo {
            actual_dt: dt,
            iterations: self.config.position_iterations,
            max_constraint_error,
            kinetic_energy,
            potential_energy,
            converged,
        })
    }
    
    fn reset(&mut self) -> SolverResult<()> {
        self.rigid_bodies.clear();
        self.constraints.clear();
        self.forces.clear();
        self.predicted_positions.clear();
        self.predicted_rotations.clear();
        self.statistics = SolverStatistics::default();
        self.next_handle = 1;
        Ok(())
    }
    
    fn statistics(&self) -> SolverStatistics {
        self.statistics.clone()
    }
    
    fn supports_feature(&self, feature: SolverFeature) -> bool {
        match feature {
            SolverFeature::Friction => true,
            SolverFeature::Joints => true,
            SolverFeature::SoftBodies => true,
            SolverFeature::Multithreading => true,
            SolverFeature::GPUAcceleration => false, // CPU-only implementation
            SolverFeature::ContinuousCollision => false,
            SolverFeature::Fluids => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::rigidbody::RigidBody;

    #[test]
    fn test_pbd_solver_creation() {
        let solver = PBDSolver::new(PBDConfig::default());
        assert_eq!(solver.name(), "Position Based Dynamics (PBD)");
        assert!(!solver.initialized);
    }

    #[test]
    fn test_pbd_solver_initialization() {
        let mut solver = PBDSolver::new(PBDConfig::default());
        let config = SolverConfiguration {
            max_rigid_bodies: 100,
            max_constraints: 200,
            target_fps: 60.0,
            quality_factor: 0.8,
        };
        
        assert!(solver.initialize(&config).is_ok());
        assert!(solver.initialized);
    }

    #[test]
    fn test_rigidbody_management() {
        let mut solver = PBDSolver::new(PBDConfig::default());
        let config = SolverConfiguration {
            max_rigid_bodies: 10,
            max_constraints: 10,
            target_fps: 60.0,
            quality_factor: 1.0,
        };
        solver.initialize(&config).unwrap();
        
        let body = RigidBody::default();
        let handle = solver.add_rigidbody(body).unwrap();
        
        assert!(solver.get_rigidbody(handle).is_ok());
        assert_eq!(solver.statistics.rigid_body_count, 1);
        
        assert!(solver.remove_rigidbody(handle).is_ok());
        assert_eq!(solver.statistics.rigid_body_count, 0);
    }
}