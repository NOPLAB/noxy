// Backend abstraction traits
// Provides unified interface for CPU and GPU implementations

/// Trait for physics computation backends
pub trait PhysicsBackend {
    /// Initialize the backend with given parameters
    fn initialize(&mut self, max_rigidbodies: usize) -> Result<(), BackendError>;

    /// Update physics simulation by one time step
    fn step(&mut self, dt: f32) -> Result<(), BackendError>;
    
    /// Update physics simulation (alternative method for testing)
    fn update(&mut self, dt: f32) {
        let _ = self.step(dt);
    }

    /// Get the name of this backend
    fn name(&self) -> &str;

    /// Check if this backend is available on current system
    fn is_available() -> bool
    where
        Self: Sized;

    /// Add a rigid body and return its index
    fn add_rigidbody(
        &mut self,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<usize, BackendError>;

    /// Get rigid body position by index
    fn get_position(&self, index: usize) -> Option<glam::Vec3>;

    /// Get rigid body velocity by index
    fn get_velocity(&self, index: usize) -> Option<glam::Vec3>;

    /// Get number of rigid bodies
    fn rigidbody_count(&self) -> usize;

    /// Set gravity
    fn set_gravity(&mut self, gravity: glam::Vec3);
    
    /// Add force to a rigid body (for testing)
    fn add_force(&mut self, _index: usize, _force_type: crate::physics::core::forces::ForceType) {
        // Default implementation does nothing - backends can override
    }
}

/// Errors that can occur in physics backends
#[derive(Debug, thiserror::Error)]
pub enum BackendError {
    #[error("Backend initialization failed: {0}")]
    InitializationFailed(String),

    #[error("Computation failed: {0}")]
    ComputationFailed(String),

    #[error("Backend not available")]
    NotAvailable,

    #[error("Backend not initialized")]
    NotInitialized,

    #[error("Invalid parameters: {0}")]
    InvalidParameters(String),

    #[error("GPU error: {0}")]
    GpuError(String),
    
    #[error("GPU unavailable: {0}")]
    GpuUnavailable(String),

    #[error("Capacity exceeded")]
    CapacityExceeded,

    #[error("Index out of bounds")]
    IndexOutOfBounds,
}

/// Backend selection strategy
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BackendSelection {
    /// Automatically select best available backend
    Auto,
    /// Force CPU backend
    Cpu,
    /// Force GPU backend (will fail if not available)
    Gpu,
}

impl Default for BackendSelection {
    fn default() -> Self {
        Self::Auto
    }
}

/// Backend performance statistics
#[derive(Debug, Default)]
pub struct BackendStats {
    pub total_steps: u64,
    pub total_time_ms: f64,
    pub avg_step_time_ms: f64,
    pub simulation_time: f64,  // Added for compatibility
    pub backend_name: String,
}

impl BackendStats {
    pub fn update(&mut self, step_time_ms: f64) {
        self.total_steps += 1;
        self.total_time_ms += step_time_ms;
        self.avg_step_time_ms = self.total_time_ms / self.total_steps as f64;
    }
}
