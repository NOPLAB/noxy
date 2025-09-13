//! # Extended Position Based Dynamics (XPBD) Solver
//! 
//! XPBD improves upon PBD by incorporating material stiffness and damping
//! more accurately, making it suitable for a wider range of applications.

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

/// Configuration for XPBD solver
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct XPBDConfig {
    /// Number of position correction iterations
    pub position_iterations: u32,
    /// Number of velocity correction iterations  
    pub velocity_iterations: u32,
    /// Global compliance (inverse stiffness)
    pub compliance: f32,
    /// Damping coefficient
    pub damping: f32,
    /// Sleep threshold for rigid bodies
    pub sleep_threshold: f32,
}

impl Default for XPBDConfig {
    fn default() -> Self {
        Self {
            position_iterations: 10,
            velocity_iterations: 5,
            compliance: 1e-6, // Very stiff by default
            damping: 0.1,
            sleep_threshold: 0.01,
        }
    }
}

/// XPBD solver implementation
pub struct XPBDSolver {
    config: XPBDConfig,
    rigid_bodies: HashMap<RigidBodyHandle, RigidBody>,
    constraints: Vec<Constraint>,
    forces: HashMap<RigidBodyHandle, Vec<Force>>,
    
    // XPBD-specific state
    predicted_positions: HashMap<RigidBodyHandle, Vec3>,
    predicted_rotations: HashMap<RigidBodyHandle, Quat>,
    lagrange_multipliers: HashMap<usize, f32>, // Constraint index -> multiplier
    
    // Statistics
    statistics: SolverStatistics,
    next_handle: u64,
    initialized: bool,
}

impl XPBDSolver {
    pub fn new(config: XPBDConfig) -> Self {
        Self {
            config,
            rigid_bodies: HashMap::new(),
            constraints: Vec::new(),
            forces: HashMap::new(),
            predicted_positions: HashMap::new(),
            predicted_rotations: HashMap::new(),
            lagrange_multipliers: HashMap::new(),
            statistics: SolverStatistics::default(),
            next_handle: 1,
            initialized: false,
        }
    }
    
    fn predict_positions(&mut self, dt: f32) -> SolverResult<()> {
        for (handle, body) in &self.rigid_bodies {
            let mut velocity = body.velocity;
            let mut angular_velocity = body.angular_velocity;
            
            // Apply external forces
            if let Some(forces) = self.forces.get(handle) {
                for force in forces {
                    match force {
                        Force::Linear(f) => {
                            velocity += *f * dt / body.mass;
                        }
                        Force::Torque(t) => {
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
    
    fn solve_constraints(&mut self, dt: f32) -> SolverResult<f32> {
        let mut max_error: f32 = 0.0;
        
        for _ in 0..self.config.position_iterations {
            let mut iteration_error: f32 = 0.0;
            
            let constraints = self.constraints.clone(); // Clone to avoid borrowing issues
            for (constraint_idx, constraint) in constraints.iter().enumerate() {
                let error = self.solve_xpbd_constraint(constraint, constraint_idx, dt)?;
                iteration_error = iteration_error.max(error);
            }
            
            max_error = max_error.max(iteration_error);
            
            if iteration_error < 1e-6 {
                break;
            }
        }
        
        Ok(max_error)
    }
    
    fn solve_xpbd_constraint(
        &mut self,
        constraint: &Constraint,
        constraint_idx: usize,
        dt: f32,
    ) -> SolverResult<f32> {
        match constraint {
            Constraint::Distance { body_a, body_b, distance } => {
                self.solve_xpbd_distance_constraint(*body_a, *body_b, *distance, constraint_idx, dt)
            }
            Constraint::Position { body, target_position } => {
                self.solve_xpbd_position_constraint(*body, *target_position, constraint_idx, dt)
            }
            _ => Ok(0.0),
        }
    }
    
    fn solve_xpbd_distance_constraint(
        &mut self,
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        target_distance: f32,
        constraint_idx: usize,
        dt: f32,
    ) -> SolverResult<f32> {
        let pos_a = *self.predicted_positions.get(&body_a)
            .ok_or(SolverError::InvalidHandle(body_a))?;
        let pos_b = *self.predicted_positions.get(&body_b)
            .ok_or(SolverError::InvalidHandle(body_b))?;
        
        let body_a_ref = self.rigid_bodies.get(&body_a)
            .ok_or(SolverError::InvalidHandle(body_a))?;
        let body_b_ref = self.rigid_bodies.get(&body_b)
            .ok_or(SolverError::InvalidHandle(body_b))?;
        
        let delta = pos_b - pos_a;
        let current_distance = delta.length();
        
        if current_distance < 1e-6 {
            return Ok(0.0);
        }
        
        let constraint_value = current_distance - target_distance;
        let gradient_a = -delta / current_distance;
        let gradient_b = delta / current_distance;
        
        // XPBD constraint solving with compliance
        let w_a = body_a_ref.inverse_mass;
        let w_b = body_b_ref.inverse_mass;
        let w_sum = w_a + w_b;
        
        if w_sum < 1e-6 {
            return Ok(constraint_value.abs());
        }
        
        let alpha = self.config.compliance / (dt * dt);
        let gamma = self.config.damping * alpha * dt;
        
        let prev_lambda = *self.lagrange_multipliers.get(&constraint_idx).unwrap_or(&0.0);
        let delta_lambda = -(constraint_value + alpha * prev_lambda + gamma * 0.0) / (w_sum + alpha + gamma);
        
        self.lagrange_multipliers.insert(constraint_idx, prev_lambda + delta_lambda);
        
        // Apply position corrections
        let correction_a = gradient_a * delta_lambda * w_a;
        let correction_b = gradient_b * delta_lambda * w_b;
        
        if let Some(pos) = self.predicted_positions.get_mut(&body_a) {
            *pos += correction_a;
        }
        if let Some(pos) = self.predicted_positions.get_mut(&body_b) {
            *pos += correction_b;
        }
        
        Ok(constraint_value.abs())
    }
    
    fn solve_xpbd_position_constraint(
        &mut self,
        body: RigidBodyHandle,
        target_position: Vec3,
        _constraint_idx: usize,
        _dt: f32,
    ) -> SolverResult<f32> {
        if let Some(pos) = self.predicted_positions.get_mut(&body) {
            let error = (*pos - target_position).length();
            *pos = target_position;
            Ok(error)
        } else {
            Err(SolverError::InvalidHandle(body))
        }
    }
    
    fn update_velocities(&mut self, dt: f32) -> SolverResult<()> {
        for (handle, body) in &mut self.rigid_bodies {
            if let Some(predicted_pos) = self.predicted_positions.get(handle) {
                body.velocity = (*predicted_pos - body.position) / dt;
                body.position = *predicted_pos;
            }
            
            if let Some(predicted_rot) = self.predicted_rotations.get(handle) {
                let delta_rot = *predicted_rot * body.rotation.conjugate();
                body.angular_velocity = Vec3::new(delta_rot.x, delta_rot.y, delta_rot.z) * 2.0 / dt;
                body.rotation = *predicted_rot;
            }
            
            // Apply damping
            body.velocity *= (1.0 - self.config.damping);
            body.angular_velocity *= (1.0 - self.config.damping);
        }
        
        Ok(())
    }
}

impl PhysicsSolver for XPBDSolver {
    fn name(&self) -> &'static str {
        "Extended Position Based Dynamics (XPBD)"
    }
    
    fn info(&self) -> SolverInfo {
        SolverInfo {
            description: "Extended PBD with material stiffness control".to_string(),
            use_cases: vec![
                "High-quality real-time simulation".to_string(),
                "Material-aware physics".to_string(),
                "Advanced constraint systems".to_string(),
            ],
            performance_profile: PerformanceProfile {
                relative_cost: 1.3,
                memory_factor: 1.4,
                accuracy: 0.9,
                stability: 0.95,
            },
            required_features: vec![
                SolverFeature::Multithreading,
            ],
        }
    }
    
    fn initialize(&mut self, config: &SolverConfiguration) -> SolverResult<()> {
        self.rigid_bodies.reserve(config.max_rigid_bodies);
        self.constraints.reserve(config.max_constraints);
        self.predicted_positions.reserve(config.max_rigid_bodies);
        self.predicted_rotations.reserve(config.max_rigid_bodies);
        self.lagrange_multipliers.reserve(config.max_constraints);
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
        
        // XPBD algorithm
        self.predict_positions(dt)?;
        let max_constraint_error = self.solve_constraints(dt)?;
        self.update_velocities(dt)?;
        self.forces.clear();
        
        // Calculate energies
        let mut kinetic_energy = 0.0;
        for body in self.rigid_bodies.values() {
            kinetic_energy += body.kinetic_energy();
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
            potential_energy: 0.0,
            converged,
        })
    }
    
    fn reset(&mut self) -> SolverResult<()> {
        self.rigid_bodies.clear();
        self.constraints.clear();
        self.forces.clear();
        self.predicted_positions.clear();
        self.predicted_rotations.clear();
        self.lagrange_multipliers.clear();
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
            SolverFeature::GPUAcceleration => false,
            SolverFeature::ContinuousCollision => false,
            SolverFeature::Fluids => false,
        }
    }
}