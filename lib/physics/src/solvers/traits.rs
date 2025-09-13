//! # Physics Solver Traits
//! 
//! Unified interface for all physics solvers, enabling runtime switching
//! between different simulation methods while maintaining performance.

use glam::{Vec3, Quat};
use crate::core::{
    rigidbody::{RigidBody, RigidBodyHandle},
    constraints::Constraint,
    forces::Force,
};

/// Result type for solver operations
pub type SolverResult<T> = Result<T, SolverError>;

/// Errors that can occur during physics solving
#[derive(Debug, thiserror::Error)]
pub enum SolverError {
    #[error("Invalid rigid body handle: {0:?}")]
    InvalidHandle(RigidBodyHandle),
    
    #[error("Solver convergence failed after {iterations} iterations")]
    ConvergenceFailure { iterations: u32 },
    
    #[error("Numerical instability detected: {reason}")]
    NumericalInstability { reason: String },
    
    #[error("Backend operation failed: {0}")]
    BackendError(#[from] anyhow::Error),
    
    #[error("Configuration error: {message}")]
    ConfigError { message: String },
}

// SolverError already implements std::error::Error through thiserror
// anyhow automatically implements From for all error types that implement std::error::Error

/// Main trait for physics solvers
/// 
/// This trait defines the interface that all physics solvers must implement.
/// It provides a unified API for different solving approaches while allowing
/// solver-specific optimizations.
pub trait PhysicsSolver: Send + Sync {
    /// Returns the name of this solver
    fn name(&self) -> &'static str;
    
    /// Returns solver-specific information
    fn info(&self) -> SolverInfo;
    
    /// Initialize the solver with given configuration
    fn initialize(&mut self, config: &SolverConfiguration) -> SolverResult<()>;
    
    /// Add a rigid body to the simulation
    fn add_rigidbody(&mut self, body: RigidBody) -> SolverResult<RigidBodyHandle>;
    
    /// Remove a rigid body from the simulation
    fn remove_rigidbody(&mut self, handle: RigidBodyHandle) -> SolverResult<()>;
    
    /// Get a reference to a rigid body
    fn get_rigidbody(&self, handle: RigidBodyHandle) -> SolverResult<&RigidBody>;
    
    /// Get a mutable reference to a rigid body
    fn get_rigidbody_mut(&mut self, handle: RigidBodyHandle) -> SolverResult<&mut RigidBody>;
    
    /// Add a constraint to the simulation
    fn add_constraint(&mut self, constraint: Constraint) -> SolverResult<()>;
    
    /// Apply a force to a rigid body
    fn apply_force(&mut self, handle: RigidBodyHandle, force: Force) -> SolverResult<()>;
    
    /// Step the simulation forward by one time step
    fn step(&mut self, dt: f32) -> SolverResult<StepInfo>;
    
    /// Reset the simulation to initial state
    fn reset(&mut self) -> SolverResult<()>;
    
    /// Get current simulation statistics
    fn statistics(&self) -> SolverStatistics;
    
    /// Check if solver supports specific features
    fn supports_feature(&self, feature: SolverFeature) -> bool;
}

/// Information about a solver implementation
#[derive(Debug, Clone)]
pub struct SolverInfo {
    /// Human-readable description
    pub description: String,
    /// Recommended use cases
    pub use_cases: Vec<String>,
    /// Performance characteristics
    pub performance_profile: PerformanceProfile,
    /// Required features
    pub required_features: Vec<SolverFeature>,
}

/// Performance characteristics of a solver
#[derive(Debug, Clone)]
pub struct PerformanceProfile {
    /// Relative computational cost (1.0 = baseline)
    pub relative_cost: f32,
    /// Memory usage factor (1.0 = baseline)
    pub memory_factor: f32,
    /// Numerical accuracy (0.0-1.0)
    pub accuracy: f32,
    /// Stability under large time steps (0.0-1.0)
    pub stability: f32,
}

/// Configuration for solver initialization
#[derive(Debug, Clone)]
pub struct SolverConfiguration {
    /// Maximum number of rigid bodies
    pub max_rigid_bodies: usize,
    /// Maximum number of constraints
    pub max_constraints: usize,
    /// Target frame rate
    pub target_fps: f32,
    /// Quality vs performance trade-off (0.0-1.0)
    pub quality_factor: f32,
}

/// Information returned after each simulation step
#[derive(Debug, Clone)]
pub struct StepInfo {
    /// Actual time step used
    pub actual_dt: f32,
    /// Number of solver iterations performed
    pub iterations: u32,
    /// Maximum constraint error
    pub max_constraint_error: f32,
    /// Total kinetic energy
    pub kinetic_energy: f32,
    /// Total potential energy
    pub potential_energy: f32,
    /// Whether convergence was achieved
    pub converged: bool,
}

/// Statistics about solver performance
#[derive(Debug, Clone, Default)]
pub struct SolverStatistics {
    /// Number of simulation steps taken
    pub steps_taken: u64,
    /// Average time per step (seconds)
    pub avg_step_time: f32,
    /// Number of rigid bodies currently simulated
    pub rigid_body_count: usize,
    /// Number of active constraints
    pub constraint_count: usize,
    /// Total simulation time
    pub simulation_time: f32,
    /// Number of convergence failures
    pub convergence_failures: u32,
}

/// Features that solvers may or may not support
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SolverFeature {
    /// Continuous collision detection
    ContinuousCollision,
    /// Friction simulation
    Friction,
    /// Joint constraints
    Joints,
    /// Soft body simulation
    SoftBodies,
    /// Fluid simulation
    Fluids,
    /// Multithreading support
    Multithreading,
    /// GPU acceleration
    GPUAcceleration,
}

/// Trait for solvers that can be run on different backends
pub trait BackendAware {
    /// Set the backend for this solver
    fn set_backend(&mut self, backend: Box<dyn crate::backends::PhysicsBackend>) -> SolverResult<()>;
    
    /// Get information about the current backend
    fn backend_info(&self) -> Option<crate::backends::BackendInfo>;
}

/// Trait for solvers that support serialization
pub trait SerializableSolver {
    /// Serialize current state to bytes
    fn serialize_state(&self) -> SolverResult<Vec<u8>>;
    
    /// Restore state from bytes
    fn deserialize_state(&mut self, data: &[u8]) -> SolverResult<()>;
}

/// Trait for solvers that can provide detailed diagnostics
pub trait DiagnosticSolver {
    /// Get detailed information about current simulation state
    fn diagnostic_info(&self) -> DiagnosticInfo;
    
    /// Validate current state for consistency
    fn validate_state(&self) -> SolverResult<Vec<String>>;
}

/// Detailed diagnostic information
#[derive(Debug, Clone)]
pub struct DiagnosticInfo {
    /// Total system energy
    pub total_energy: f32,
    /// Center of mass
    pub center_of_mass: Vec3,
    /// Angular momentum
    pub angular_momentum: Vec3,
    /// Linear momentum  
    pub linear_momentum: Vec3,
    /// Constraint violations
    pub constraint_violations: Vec<ConstraintViolation>,
}

/// Information about constraint violations
#[derive(Debug, Clone)]
pub struct ConstraintViolation {
    /// Type of constraint violated
    pub constraint_type: String,
    /// Magnitude of violation
    pub violation: f32,
    /// Bodies involved
    pub involved_bodies: Vec<RigidBodyHandle>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solver_info_creation() {
        let info = SolverInfo {
            description: "Test solver".to_string(),
            use_cases: vec!["Testing".to_string()],
            performance_profile: PerformanceProfile {
                relative_cost: 1.0,
                memory_factor: 1.0,
                accuracy: 0.9,
                stability: 0.8,
            },
            required_features: vec![SolverFeature::Friction],
        };
        
        assert_eq!(info.description, "Test solver");
        assert_eq!(info.required_features.len(), 1);
    }

    #[test]
    fn test_step_info() {
        let step_info = StepInfo {
            actual_dt: 0.016,
            iterations: 10,
            max_constraint_error: 0.001,
            kinetic_energy: 100.0,
            potential_energy: 50.0,
            converged: true,
        };
        
        assert!(step_info.converged);
        assert!(step_info.kinetic_energy > 0.0);
    }
}