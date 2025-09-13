//! # Physics Solver Abstraction
//! 
//! This module provides a unified interface for different physics solvers,
//! allowing runtime selection between various simulation methods.
//! 
//! ## Supported Solvers
//! 
//! - **Implicit Integration**: Traditional Verlet, Runge-Kutta methods
//! - **Position Based Dynamics (PBD)**: Real-time oriented constraint solving
//! - **Extended Position Based Dynamics (XPBD)**: Improved PBD with stiffness control
//! - **Finite Element Method (FEM)**: High-accuracy deformation analysis (future)
//! 
//! ## Architecture
//! 
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Solver Trait                             │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
//! │  │   step()    │  │ add_body()  │  │constraints()│  ...   │
//! │  └─────────────┘  └─────────────┘  └─────────────┘        │
//! └─────────────────────┬───────────────────────────────────────┘
//!                       │
//!          ┌────────────┼────────────┬────────────┐
//!          │            │            │            │
//!    ┌──────▼─┐   ┌──────▼─┐   ┌──────▼─┐   ┌──────▼─┐
//!    │ Verlet │   │   PBD  │   │  XPBD  │   │  FEM   │
//!    │Solver  │   │ Solver │   │ Solver │   │ Solver │
//!    └────────┘   └────────┘   └────────┘   └────────┘
//! ```

pub mod traits;
pub mod implicit;
pub mod pbd;
pub mod xpbd;

#[cfg(feature = "future-fem")]
pub mod fem;

pub use traits::*;
pub use implicit::*;
pub use pbd::*;
pub use xpbd::*;

/// Available solver types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SolverType {
    /// Traditional implicit integration (Verlet, RK4)
    Implicit,
    /// Position Based Dynamics - real-time oriented
    PBD,
    /// Extended Position Based Dynamics - improved PBD
    XPBD,
    #[cfg(feature = "future-fem")]
    /// Finite Element Method - high accuracy
    FEM,
}

impl Default for SolverType {
    fn default() -> Self {
        SolverType::PBD // PBD is good balance of performance and stability
    }
}

/// Configuration for physics solvers
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SolverConfig {
    /// Which solver to use
    pub solver_type: SolverType,
    /// Number of constraint solver iterations
    pub constraint_iterations: u32,
    /// Time step for simulation
    pub time_step: f32,
    /// Global damping factor
    pub damping: f32,
    /// Solver-specific parameters
    pub solver_params: SolverParams,
}

/// Solver-specific configuration parameters
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SolverParams {
    Implicit(ImplicitConfig),
    PBD(PBDConfig),
    XPBD(XPBDConfig),
    #[cfg(feature = "future-fem")]
    FEM(FEMConfig),
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            solver_type: SolverType::default(),
            constraint_iterations: 10,
            time_step: 1.0 / 60.0, // 60 FPS
            damping: 0.99,
            solver_params: SolverParams::PBD(PBDConfig::default()),
        }
    }
}

/// Factory function to create solvers
pub fn create_solver(config: &SolverConfig) -> Result<Box<dyn PhysicsSolver>, anyhow::Error> {
    match config.solver_type {
        SolverType::Implicit => Ok(Box::new(ImplicitSolver::new(
            match &config.solver_params {
                SolverParams::Implicit(cfg) => cfg.clone(),
                _ => ImplicitConfig::default(),
            }
        ))),
        SolverType::PBD => Ok(Box::new(PBDSolver::new(
            match &config.solver_params {
                SolverParams::PBD(cfg) => cfg.clone(),
                _ => PBDConfig::default(),
            }
        ))),
        SolverType::XPBD => Ok(Box::new(XPBDSolver::new(
            match &config.solver_params {
                SolverParams::XPBD(cfg) => cfg.clone(),
                _ => XPBDConfig::default(),
            }
        ))),
        #[cfg(feature = "future-fem")]
        SolverType::FEM => Ok(Box::new(FEMSolver::new(
            match &config.solver_params {
                SolverParams::FEM(cfg) => cfg.clone(),
                _ => FEMConfig::default(),
            }
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solver_creation() {
        let config = SolverConfig::default();
        let solver = create_solver(&config).unwrap();
        assert!(!solver.name().is_empty());
    }

    #[test]
    fn test_solver_types() {
        for solver_type in [SolverType::Implicit, SolverType::PBD, SolverType::XPBD] {
            let mut config = SolverConfig::default();
            config.solver_type = solver_type;
            let solver = create_solver(&config);
            assert!(solver.is_ok(), "Failed to create solver: {:?}", solver_type);
        }
    }
}