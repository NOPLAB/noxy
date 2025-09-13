//! # Main Simulation API
//! 
//! High-level API for physics simulation combining solvers and backends.

use crate::{
    core::{
        rigidbody::{RigidBody, RigidBodyHandle},
        shapes::Shape,
        materials::Material,
        forces::Force,
        constraints::Constraint,
    },
    solvers::{SolverType, SolverConfig, PhysicsSolver, create_solver, StepInfo},
    backends::{BackendType, BackendConfig},
    utils::math::Transform,
};
use glam::{Vec3, Quat};
use anyhow::Result;

/// Main simulation configuration
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimulationConfig {
    /// Solver configuration
    pub solver: SolverConfig,
    /// Backend configuration
    pub backend: BackendConfig,
    /// Maximum number of rigid bodies
    pub max_rigid_bodies: usize,
    /// Maximum number of constraints
    pub max_constraints: usize,
    /// Target frame rate
    pub target_fps: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            solver: SolverConfig::default(),
            backend: BackendConfig::default(),
            max_rigid_bodies: 1000,
            max_constraints: 2000,
            target_fps: 60.0,
        }
    }
}

/// Builder for creating simulations
pub struct SimulationBuilder {
    config: SimulationConfig,
}

impl Default for SimulationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl SimulationBuilder {
    /// Create a new simulation builder
    pub fn new() -> Self {
        Self {
            config: SimulationConfig::default(),
        }
    }
    
    /// Set the solver type
    pub fn solver(mut self, solver_type: SolverType) -> Self {
        self.config.solver.solver_type = solver_type;
        self
    }
    
    /// Set the backend type
    pub fn backend(mut self, backend_type: BackendType) -> Self {
        self.config.backend.backend_type = backend_type;
        self
    }
    
    /// Set maximum number of rigid bodies
    pub fn max_rigid_bodies(mut self, max: usize) -> Self {
        self.config.max_rigid_bodies = max;
        self
    }
    
    /// Set maximum number of constraints
    pub fn max_constraints(mut self, max: usize) -> Self {
        self.config.max_constraints = max;
        self
    }
    
    /// Set target frame rate
    pub fn target_fps(mut self, fps: f32) -> Self {
        self.config.target_fps = fps;
        self.config.solver.time_step = 1.0 / fps;
        self
    }
    
    /// Build the simulation
    pub fn build(self) -> Result<Simulation> {
        Simulation::new(self.config)
    }
}

/// Main physics simulation
pub struct Simulation {
    config: SimulationConfig,
    solver: Box<dyn PhysicsSolver>,
    current_time: f32,
    step_count: u64,
}

impl Simulation {
    /// Create a new simulation with the given configuration
    pub fn new(config: SimulationConfig) -> Result<Self> {
        let solver = create_solver(&config.solver)?;
        
        let mut sim = Self {
            config,
            solver,
            current_time: 0.0,
            step_count: 0,
        };
        
        // Initialize solver
        let solver_config = crate::solvers::traits::SolverConfiguration {
            max_rigid_bodies: sim.config.max_rigid_bodies,
            max_constraints: sim.config.max_constraints,
            target_fps: sim.config.target_fps,
            quality_factor: 1.0,
        };
        
        sim.solver.initialize(&solver_config)?;
        
        Ok(sim)
    }
    
    /// Add a rigid body to the simulation
    pub fn add_rigidbody(
        &mut self,
        shape: Shape,
        transform: Transform,
        material: Material,
    ) -> Result<RigidBodyHandle> {
        let mass = material.density * shape.volume();
        let inertia = shape.moment_of_inertia(mass);
        
        let body = RigidBody::new(
            transform.translation,
            transform.rotation,
            mass,
            inertia,
            shape,
            material,
        );
        
        Ok(self.solver.add_rigidbody(body)?)
    }
    
    /// Add a rigid body with custom mass
    pub fn add_rigidbody_with_mass(
        &mut self,
        shape: Shape,
        transform: Transform,
        mass: f32,
        material: Material,
    ) -> Result<RigidBodyHandle> {
        let inertia = shape.moment_of_inertia(mass);
        
        let body = RigidBody::new(
            transform.translation,
            transform.rotation,
            mass,
            inertia,
            shape,
            material,
        );
        
        Ok(self.solver.add_rigidbody(body)?)
    }
    
    /// Remove a rigid body from the simulation
    pub fn remove_rigidbody(&mut self, handle: RigidBodyHandle) -> Result<()> {
        Ok(self.solver.remove_rigidbody(handle)?)
    }
    
    /// Get a reference to a rigid body
    pub fn get_rigidbody(&self, handle: RigidBodyHandle) -> Result<&RigidBody> {
        Ok(self.solver.get_rigidbody(handle)?)
    }
    
    /// Get a mutable reference to a rigid body
    pub fn get_rigidbody_mut(&mut self, handle: RigidBodyHandle) -> Result<&mut RigidBody> {
        Ok(self.solver.get_rigidbody_mut(handle)?)
    }
    
    /// Get the position of a rigid body
    pub fn get_position(&self, handle: RigidBodyHandle) -> Result<Vec3> {
        Ok(self.solver.get_rigidbody(handle)?.position)
    }
    
    /// Get the rotation of a rigid body
    pub fn get_rotation(&self, handle: RigidBodyHandle) -> Result<Quat> {
        Ok(self.solver.get_rigidbody(handle)?.rotation)
    }
    
    /// Set the position of a rigid body
    pub fn set_position(&mut self, handle: RigidBodyHandle, position: Vec3) -> Result<()> {
        self.solver.get_rigidbody_mut(handle)?.position = position;
        Ok(())
    }
    
    /// Set the rotation of a rigid body
    pub fn set_rotation(&mut self, handle: RigidBodyHandle, rotation: Quat) -> Result<()> {
        self.solver.get_rigidbody_mut(handle)?.rotation = rotation;
        Ok(())
    }
    
    /// Add a constraint to the simulation
    pub fn add_constraint(&mut self, constraint: Constraint) -> Result<()> {
        Ok(self.solver.add_constraint(constraint)?)
    }
    
    /// Apply a force to a rigid body
    pub fn apply_force(&mut self, handle: RigidBodyHandle, force: Force) -> Result<()> {
        Ok(self.solver.apply_force(handle, force)?)
    }
    
    /// Apply gravity to all bodies
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<()> {
        // This is a simplified implementation
        // In a real implementation, gravity would be stored and applied automatically
        let stats = self.solver.statistics();
        for i in 0..stats.rigid_body_count {
            // This is a placeholder - we'd need to iterate over actual handles
            let handle = RigidBodyHandle(i as u64 + 1);
            if let Ok(_) = self.solver.get_rigidbody(handle) {
                self.solver.apply_force(handle, Force::Gravity(gravity))?;
            }
        }
        Ok(())
    }
    
    /// Step the simulation forward by one time step
    pub fn step(&mut self, dt: f32) -> Result<StepInfo> {
        let step_info = self.solver.step(dt)?;
        self.current_time += step_info.actual_dt;
        self.step_count += 1;
        Ok(step_info)
    }
    
    /// Step the simulation forward using the configured time step
    pub fn step_default(&mut self) -> Result<StepInfo> {
        self.step(self.config.solver.time_step)
    }
    
    /// Reset the simulation to initial state
    pub fn reset(&mut self) -> Result<()> {
        self.solver.reset()?;
        self.current_time = 0.0;
        self.step_count = 0;
        Ok(())
    }
    
    /// Get current simulation time
    pub fn current_time(&self) -> f32 {
        self.current_time
    }
    
    /// Get number of steps taken
    pub fn step_count(&self) -> u64 {
        self.step_count
    }
    
    /// Get solver statistics
    pub fn statistics(&self) -> crate::solvers::traits::SolverStatistics {
        self.solver.statistics()
    }
    
    /// Get solver information
    pub fn solver_info(&self) -> crate::solvers::traits::SolverInfo {
        self.solver.info()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_builder() {
        let sim = SimulationBuilder::new()
            .solver(SolverType::PBD)
            .backend(BackendType::CPU)
            .max_rigid_bodies(100)
            .target_fps(60.0)
            .build();
        
        assert!(sim.is_ok());
    }

    #[test]
    fn test_simulation_basic_usage() {
        let mut sim = SimulationBuilder::new()
            .solver(SolverType::PBD)
            .build()
            .unwrap();
        
        // Add a sphere
        let handle = sim.add_rigidbody(
            Shape::Sphere { radius: 1.0 },
            Transform::from_translation(Vec3::new(0.0, 5.0, 0.0)),
            Material::default(),
        ).unwrap();
        
        // Check initial position
        let initial_pos = sim.get_position(handle).unwrap();
        assert_eq!(initial_pos, Vec3::new(0.0, 5.0, 0.0));
        
        // Step simulation
        let step_info = sim.step(0.016).unwrap();
        assert!(step_info.actual_dt > 0.0);
        
        // Check that time has advanced
        assert!(sim.current_time() > 0.0);
        assert_eq!(sim.step_count(), 1);
    }

    #[test]
    fn test_force_application() {
        let mut sim = SimulationBuilder::new()
            .solver(SolverType::PBD)
            .build()
            .unwrap();
        
        let handle = sim.add_rigidbody(
            Shape::Sphere { radius: 1.0 },
            Transform::default(),
            Material::default(),
        ).unwrap();
        
        // Apply upward force
        sim.apply_force(handle, Force::Linear(Vec3::new(0.0, 100.0, 0.0))).unwrap();
        
        // Step simulation
        sim.step(0.016).unwrap();
        
        // Body should have moved upward
        let pos = sim.get_position(handle).unwrap();
        assert!(pos.y > 0.0);
    }
}