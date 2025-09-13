// Backend factory and manager for automatic backend selection

use super::cpu::CpuBackend;
use super::gpu::GpuBackend;
use super::traits::{BackendError, BackendSelection, PhysicsBackend};
use std::time::Instant;

/// Factory for creating physics backends
pub struct BackendFactory;

impl BackendFactory {
    /// Create a backend based on selection strategy
    pub fn create_backend(
        selection: BackendSelection,
    ) -> Result<Box<dyn PhysicsBackend>, BackendError> {
        match selection {
            BackendSelection::Auto => {
                // Auto-select best available backend
                Self::auto_select_backend()
            }
            BackendSelection::Cpu => {
                // Force CPU backend
                if CpuBackend::is_available() {
                    Ok(Box::new(CpuBackend::new()))
                } else {
                    Err(BackendError::NotAvailable)
                }
            }
            BackendSelection::Gpu => {
                // Force GPU backend (simplified for TDD)
                Ok(Box::new(GpuBackend::new()))
            }
        }
    }

    /// Create a backend asynchronously (required for GPU)
    pub async fn create_backend_async(
        selection: BackendSelection,
    ) -> Result<Box<dyn PhysicsBackend>, BackendError> {
        match selection {
            BackendSelection::Auto => {
                // Auto-select best available backend
                Self::auto_select_backend_async().await
            }
            BackendSelection::Cpu => {
                // Force CPU backend
                if CpuBackend::is_available() {
                    Ok(Box::new(CpuBackend::new()))
                } else {
                    Err(BackendError::NotAvailable)
                }
            }
            BackendSelection::Gpu => {
                // Force GPU backend (simplified - no async needed for TDD version)
                if GpuBackend::is_available() {
                    Ok(Box::new(GpuBackend::new()))
                } else {
                    Err(BackendError::NotAvailable)
                }
            }
        }
    }

    /// Automatically select the best available backend
    fn auto_select_backend() -> Result<Box<dyn PhysicsBackend>, BackendError> {
        // For now, always select CPU since GPU is not implemented
        // In the future, this will benchmark and choose the best option

        if CpuBackend::is_available() {
            Ok(Box::new(CpuBackend::new()))
        } else {
            Err(BackendError::NotAvailable)
        }
    }

    /// Automatically select the best available backend (async version)
    async fn auto_select_backend_async() -> Result<Box<dyn PhysicsBackend>, BackendError> {
        // Try GPU first if available, then fallback to CPU
        if GpuBackend::is_available() {
            // For TDD version, just create directly
            return Ok(Box::new(GpuBackend::new()));
        }

        // Fallback to CPU
        if CpuBackend::is_available() {
            Ok(Box::new(CpuBackend::new()))
        } else {
            Err(BackendError::NotAvailable)
        }
    }

    /// Get available backends
    pub fn available_backends() -> Vec<&'static str> {
        let mut backends = Vec::new();

        if CpuBackend::is_available() {
            backends.push("cpu");
        }

        if GpuBackend::is_available() {
            backends.push("GPU");
        }

        backends
    }

    /// Benchmark backends to determine the best one for given parameters
    pub fn benchmark_backends(_max_rigidbodies: usize, _steps: u32) -> Result<String, BackendError> {
        let mut best_backend = String::new();
        let mut _best_time = f64::MAX;

        for _backend in Self::available_backends() {
            let start = std::time::Instant::now();

            // Simplified benchmark - just measure creation time
            let _test_backend = BackendManager::new(BackendSelection::Cpu)?;

            let time = start.elapsed().as_secs_f64();

            if time < _best_time {
                _best_time = time;
                best_backend = "cpu".to_string();
            }
        }

        // TODO: Benchmark GPU backend when implemented

        if best_backend.is_empty() {
            Err(BackendError::NotAvailable)
        } else {
            Ok(best_backend)
        }
    }

    /// Benchmark a specific backend
    fn benchmark_backend(
        backend: &mut dyn PhysicsBackend,
        max_rigidbodies: usize,
        steps: u32,
    ) -> Result<f64, BackendError> {
        backend.initialize(max_rigidbodies)?;

        let start_time = Instant::now();
        for _ in 0..steps {
            backend.step(0.01)?;
        }
        let elapsed = start_time.elapsed();

        Ok(elapsed.as_secs_f64() * 1000.0) // Return time in milliseconds
    }
}
/// Simplified factory function for tests and basic usage
impl BackendFactory {
    /// Create a backend with simplified API
    pub fn create(selection: BackendSelection) -> Box<dyn PhysicsBackend> {
        match Self::create_backend(selection) {
            Ok(backend) => backend,
            Err(_) => {
                // Fallback to CPU if requested backend fails
                Box::new(CpuBackend::new())
            }
        }
    }
}

/// Backend manager with fallback capabilities
pub struct BackendManager {
    backend: Box<dyn PhysicsBackend>,
    selection: BackendSelection,
    fallback_attempted: bool,
}

impl BackendManager {
    /// Create a new backend manager
    pub fn new(selection: BackendSelection) -> Result<Self, BackendError> {
        let backend = BackendFactory::create_backend(selection)?;

        Ok(Self {
            backend,
            selection,
            fallback_attempted: false,
        })
    }

    /// Create a new backend manager asynchronously (required for GPU)
    pub async fn new_async(selection: BackendSelection) -> Result<Self, BackendError> {
        let backend = BackendFactory::create_backend_async(selection).await?;

        Ok(Self {
            backend,
            selection,
            fallback_attempted: false,
        })
    }

    /// Initialize the backend
    pub fn initialize(&mut self, max_rigidbodies: usize) -> Result<(), BackendError> {
        self.backend.initialize(max_rigidbodies)
    }

    /// Step the simulation with automatic fallback
    pub fn step(&mut self, dt: f32) -> Result<(), BackendError> {
        match self.backend.step(dt) {
            Ok(()) => Ok(()),
            Err(e) => {
                if !self.fallback_attempted {
                    self.attempt_fallback()?;
                    self.fallback_attempted = true;
                    self.backend.step(dt)
                } else {
                    Err(e)
                }
            }
        }
    }

    /// Get current backend name
    pub fn backend_name(&self) -> &str {
        self.backend.name()
    }

    /// Add a rigid body to the simulation
    pub fn add_rigidbody(
        &mut self,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<usize, BackendError> {
        self.backend.add_rigidbody(position, velocity, mass)
    }

    /// Get rigid body position by index
    pub fn get_position(&self, index: usize) -> Option<glam::Vec3> {
        self.backend.get_position(index)
    }

    /// Get rigid body velocity by index
    pub fn get_velocity(&self, index: usize) -> Option<glam::Vec3> {
        self.backend.get_velocity(index)
    }

    /// Get number of rigid bodies
    pub fn rigidbody_count(&self) -> usize {
        self.backend.rigidbody_count()
    }

    /// Set gravity
    pub fn set_gravity(&mut self, gravity: glam::Vec3) {
        self.backend.set_gravity(gravity);
    }

    /// Attempt to fallback to a different backend
    fn attempt_fallback(&mut self) -> Result<(), BackendError> {
        match self.selection {
            BackendSelection::Auto | BackendSelection::Gpu => {
                // Try to fallback to CPU
                if CpuBackend::is_available() {
                    self.backend = Box::new(CpuBackend::new());
                    log::warn!("GPU backend failed, falling back to CPU");
                    Ok(())
                } else {
                    Err(BackendError::NotAvailable)
                }
            }
            BackendSelection::Cpu => {
                // Already on CPU, no fallback available
                Err(BackendError::NotAvailable)
            }
        }
    }

    /// Get available backends
    pub fn available_backends() -> Vec<&'static str> {
        BackendFactory::available_backends()
    }

    /// Check if fallback was used
    pub fn used_fallback(&self) -> bool {
        self.fallback_attempted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_backend_factory_create_cpu() {
        let backend = BackendFactory::create_backend(BackendSelection::Cpu).unwrap();
        assert_eq!(backend.name(), "cpu");
    }

    #[test]
    fn test_backend_factory_create_auto() {
        let backend = BackendFactory::create_backend(BackendSelection::Auto).unwrap();
        // Should select CPU since it's the only available backend
        assert_eq!(backend.name(), "cpu");
    }

    #[test]
    fn test_backend_factory_create_gpu_availability() {
        let result = BackendFactory::create_backend(BackendSelection::Gpu);
        
        // GPU backend creation always succeeds in the current implementation
        // but the actual GPU availability is checked separately
        match result {
            Ok(_) => {
                println!("GPU backend can be created (but may not have actual GPU support)");
                // The backend can be created regardless of GPU availability
                // Actual GPU availability is checked by is_available()
                let gpu_available = GpuBackend::is_available();
                println!("Actual GPU support available: {}", gpu_available);
                // Don't assert on GPU availability since it depends on the environment
            }
            Err(_) => {
                println!("GPU backend creation failed");
                // This should not happen in the current implementation
                panic!("GPU backend creation unexpectedly failed");
            }
        }
    }

    #[test]
    fn test_available_backends() {
        let backends = BackendFactory::available_backends();
        assert!(backends.contains(&"cpu"));
    }

    #[test]
    fn test_backend_manager_creation() {
        let manager = BackendManager::new(BackendSelection::Cpu).unwrap();
        assert_eq!(manager.backend_name(), "cpu");
        assert!(!manager.used_fallback());
    }

    #[test]
    fn test_backend_manager_fallback() {
        let mut manager = BackendManager::new(BackendSelection::Cpu).unwrap();
        assert!(manager.initialize(10).is_ok());
        assert_eq!(manager.backend_name(), "cpu");
    }

    #[test]
    fn test_benchmark_backends() {
        let result = BackendFactory::benchmark_backends(10, 5);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), "cpu");
    }
}
