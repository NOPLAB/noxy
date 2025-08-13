// Main physics simulation API
// Provides a high-level interface that abstracts backend details

use crate::physics::backends::{
    factory::BackendManager,
    traits::{BackendError, BackendSelection},
};
use glam::Vec3;
use std::collections::HashMap;

/// High-level physics simulation interface
pub struct PhysicsSimulation {
    backend_manager: BackendManager,
    rigidbody_map: HashMap<RigidBodyId, usize>, // Maps public IDs to backend indices
    next_id: RigidBodyId,
}

/// Public identifier for rigid bodies
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RigidBodyId(u64);

impl RigidBodyId {
    /// Get the internal ID value
    pub fn id(&self) -> u64 {
        self.0
    }
}

/// Configuration for physics simulation
#[derive(Debug, Clone)]
pub struct SimulationConfig {
    pub backend_selection: BackendSelection,
    pub max_rigidbodies: usize,
    pub gravity: Vec3,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            backend_selection: BackendSelection::Auto,
            max_rigidbodies: 1000,
            gravity: Vec3::new(0.0, -9.81, 0.0),
        }
    }
}

/// Rigid body properties for creation
#[derive(Debug, Clone)]
pub struct RigidBodyProperties {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f32,
}

impl Default for RigidBodyProperties {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            mass: 1.0,
        }
    }
}

/// Simulation errors
#[derive(Debug, thiserror::Error)]
pub enum SimulationError {
    #[error("Backend error: {0}")]
    Backend(#[from] BackendError),

    #[error("Rigid body not found: {0:?}")]
    RigidBodyNotFound(RigidBodyId),

    #[error("Simulation not initialized")]
    NotInitialized,

    #[error("Invalid parameters: {0}")]
    InvalidParameters(String),
}

impl PhysicsSimulation {
    /// Create a new physics simulation with default configuration
    pub fn new() -> Result<Self, SimulationError> {
        Self::with_config(SimulationConfig::default())
    }

    /// Create a new physics simulation with custom configuration
    pub fn with_config(config: SimulationConfig) -> Result<Self, SimulationError> {
        let mut backend_manager = BackendManager::new(config.backend_selection)?;
        backend_manager.initialize(config.max_rigidbodies)?;
        backend_manager.set_gravity(config.gravity);

        Ok(Self {
            backend_manager,
            rigidbody_map: HashMap::new(),
            next_id: RigidBodyId(0),
        })
    }

    /// Add a rigid body to the simulation
    pub fn add_rigidbody(
        &mut self,
        properties: RigidBodyProperties,
    ) -> Result<RigidBodyId, SimulationError> {
        // Validate properties
        if properties.mass <= 0.0 {
            return Err(SimulationError::InvalidParameters(
                "Mass must be positive".to_string(),
            ));
        }

        if !properties.position.is_finite() || !properties.velocity.is_finite() {
            return Err(SimulationError::InvalidParameters(
                "Position and velocity must be finite".to_string(),
            ));
        }

        // Add to backend
        let backend_index = self.backend_manager.add_rigidbody(
            properties.position,
            properties.velocity,
            properties.mass,
        )?;

        // Create public ID and map it
        let id = self.next_id;
        self.next_id.0 += 1;
        self.rigidbody_map.insert(id, backend_index);

        Ok(id)
    }

    /// Step the simulation forward by one time step
    pub fn step(&mut self, dt: f32) -> Result<(), SimulationError> {
        if dt <= 0.0 || !dt.is_finite() {
            return Err(SimulationError::InvalidParameters(
                "Time step must be positive and finite".to_string(),
            ));
        }

        self.backend_manager.step(dt)?;
        Ok(())
    }

    /// Get the position of a rigid body
    pub fn get_rigidbody_position(&self, id: RigidBodyId) -> Result<Vec3, SimulationError> {
        let backend_index = self
            .rigidbody_map
            .get(&id)
            .ok_or(SimulationError::RigidBodyNotFound(id))?;

        self.backend_manager
            .get_position(*backend_index)
            .ok_or(SimulationError::RigidBodyNotFound(id))
    }

    /// Get the velocity of a rigid body
    pub fn get_rigidbody_velocity(&self, id: RigidBodyId) -> Result<Vec3, SimulationError> {
        let backend_index = self
            .rigidbody_map
            .get(&id)
            .ok_or(SimulationError::RigidBodyNotFound(id))?;

        self.backend_manager
            .get_velocity(*backend_index)
            .ok_or(SimulationError::RigidBodyNotFound(id))
    }

    /// Set gravity for the simulation
    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.backend_manager.set_gravity(gravity);
    }

    /// Get simulation statistics
    pub fn statistics(&self) -> SimulationStatistics {
        SimulationStatistics {
            backend_name: self.backend_manager.backend_name().to_string(),
            rigidbody_count: self.rigidbody_map.len(),
            used_fallback: self.backend_manager.used_fallback(),
        }
    }

    /// Get the number of rigid bodies in the simulation
    pub fn rigidbody_count(&self) -> usize {
        self.rigidbody_map.len()
    }

    /// Remove a rigid body from the simulation
    pub fn remove_rigidbody(&mut self, id: RigidBodyId) -> Result<(), SimulationError> {
        if !self.rigidbody_map.contains_key(&id) {
            return Err(SimulationError::RigidBodyNotFound(id));
        }

        // Remove from map (simplified implementation)
        // Note: This doesn't actually remove from the backend, which would require
        // additional backend methods for proper implementation
        self.rigidbody_map.remove(&id);
        Ok(())
    }

    /// Get all rigid body IDs
    pub fn rigidbody_ids(&self) -> Vec<RigidBodyId> {
        self.rigidbody_map.keys().copied().collect()
    }
}

/// Statistics about the simulation
#[derive(Debug, Clone)]
pub struct SimulationStatistics {
    pub backend_name: String,
    pub rigidbody_count: usize,
    pub used_fallback: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_creation() {
        let simulation = PhysicsSimulation::new().unwrap();
        let stats = simulation.statistics();
        assert_eq!(stats.backend_name, "CPU");
        assert_eq!(stats.rigidbody_count, 0);
        assert!(!stats.used_fallback);
    }

    #[test]
    fn test_simulation_with_config() {
        let config = SimulationConfig {
            backend_selection: BackendSelection::Cpu,
            max_rigidbodies: 500,
            gravity: Vec3::new(0.0, -10.0, 0.0),
        };

        let simulation = PhysicsSimulation::with_config(config).unwrap();
        assert_eq!(simulation.statistics().backend_name, "CPU");
    }

    #[test]
    fn test_simulation_step() {
        let mut simulation = PhysicsSimulation::new().unwrap();

        // Valid time step
        assert!(simulation.step(0.01).is_ok());

        // Invalid time steps
        assert!(simulation.step(0.0).is_err());
        assert!(simulation.step(-0.01).is_err());
        assert!(simulation.step(f32::NAN).is_err());
    }

    #[test]
    fn test_rigidbody_properties_validation() {
        let mut simulation = PhysicsSimulation::new().unwrap();

        // Valid properties should succeed
        let valid_props = RigidBodyProperties {
            position: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(0.0, 1.0, 0.0),
            mass: 1.5,
        };

        let result = simulation.add_rigidbody(valid_props);
        assert!(result.is_ok());

        // TODO: Add validation for invalid mass and position when implemented
        // For now, test passes basic functionality
    }

    #[test]
    fn test_simulation_statistics() {
        let simulation = PhysicsSimulation::new().unwrap();
        let stats = simulation.statistics();

        assert!(!stats.backend_name.is_empty());
        assert_eq!(stats.rigidbody_count, 0);
    }
}
