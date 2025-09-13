//! GPU physics backend implementation using WGPU compute shaders

mod buffers;
mod compute;
mod pipeline;

use super::traits::{BackendError, PhysicsBackend};


/// GPU-accelerated physics backend with CPU fallback
pub struct GpuBackend {
    // GPU resources
    device: Option<wgpu::Device>,
    queue: Option<wgpu::Queue>,
    buffer_manager: Option<buffers::BufferManager>,
    compute_manager: Option<compute::ComputeManager>,
    
    // Basic GPU detection
    gpu_available: bool,
    gpu_validated: bool,
    
    // CPU-side data storage (GPU data mirrored here)
    positions: Vec<glam::Vec3>,
    velocities: Vec<glam::Vec3>,
    masses: Vec<f32>,
    
    // Simulation parameters
    gravity: glam::Vec3,
    rigidbody_count: usize,
    max_rigidbodies: usize,
    
    // State tracking
    initialized: bool,
    simulation_time: f32,
}

impl GpuBackend {
    /// Create a new GPU backend with default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if GPU compute is available on the system
    fn check_gpu_availability() -> bool {
        // Simplified GPU availability check
        pollster::block_on(async {
            let instance = wgpu::Instance::default();
            match instance.request_adapter(&wgpu::RequestAdapterOptions::default()).await {
                Ok(_adapter) => true,  // GPU adapter available
                Err(_) => false,       // No GPU adapter
            }
        })
    }

    /// Validate GPU compute capabilities
    async fn validate_gpu_compute(&mut self) -> bool {
        let instance = wgpu::Instance::default();
        
        match instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }).await {
            Ok(adapter) => {
                // Try to create a device for validation
                match adapter.request_device(
                    &wgpu::DeviceDescriptor {
                        required_features: wgpu::Features::empty(),
                        required_limits: wgpu::Limits::default(),
                        label: Some("GPU Validation"),
                        memory_hints: wgpu::MemoryHints::Performance,
                        trace: wgpu::Trace::Off,
                    },
                ).await {
                    Ok(_) => {
                        tracing::info!("GPU compute validation successful");
                        true
                    }
                    Err(e) => {
                        tracing::warn!("GPU device creation failed: {}", e);
                        false
                    }
                }
            }
            Err(e) => {
                tracing::warn!("No suitable GPU adapter found: {}", e);
                false
            }
        }
    }

    /// Initialize GPU resources (device, queue, buffers)
    async fn initialize_gpu_resources(&mut self) -> bool {
        let instance = wgpu::Instance::default();
        
        match instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }).await {
            Ok(adapter) => {
                match adapter.request_device(
                    &wgpu::DeviceDescriptor {
                        required_features: wgpu::Features::empty(),
                        required_limits: wgpu::Limits::default(),
                        label: Some("Physics GPU Device"),
                        memory_hints: wgpu::MemoryHints::Performance,
                        trace: wgpu::Trace::Off,
                    },
                ).await {
                    Ok((device, queue)) => {
                        self.device = Some(device);
                        self.queue = Some(queue);
                        
                        // Initialize buffer and compute managers
                        // self.buffer_manager = Some(buffers::BufferManager::new(&self.device.as_ref().unwrap()));
                        // self.compute_manager = Some(compute::ComputeManager::new(&self.device.as_ref().unwrap()));
                        
                        tracing::info!("GPU resources initialized successfully");
                        true
                    }
                    Err(e) => {
                        tracing::warn!("Failed to create GPU device: {}", e);
                        false
                    }
                }
            }
            Err(e) => {
                tracing::warn!("No suitable GPU adapter found: {}", e);
                false
            }
        }
    }

    /// Execute physics step on CPU (fallback implementation)
    fn execute_cpu_step(&mut self, dt: f32) -> Result<(), BackendError> {
        for i in 0..self.rigidbody_count {
            // Apply gravity acceleration
            let acceleration = self.gravity;
            
            // Verlet integration: v = v0 + a*dt, p = p0 + v*dt
            self.velocities[i] += acceleration * dt;
            self.positions[i] += self.velocities[i] * dt;
        }
        
        self.simulation_time += dt;
        Ok(())
    }

    /// Execute physics step on GPU (placeholder for future implementation)
    #[allow(unused_variables)]
    async fn execute_gpu_step(&mut self, dt: f32) -> Result<(), BackendError> {
        // Placeholder for actual GPU compute implementation
        // For Phase 1, we'll fall back to CPU
        tracing::debug!("GPU step requested, falling back to CPU for Phase 1");
        self.execute_cpu_step(dt)
    }

    /// Get current simulation statistics
    pub fn stats(&self) -> GpuBackendStats {
        GpuBackendStats {
            gpu_available: self.gpu_available,
            gpu_validated: self.gpu_validated,
            rigidbody_count: self.rigidbody_count,
            simulation_time: self.simulation_time,
            backend_mode: if self.gpu_available && self.gpu_validated {
                "GPU-ready (CPU fallback)".to_string()
            } else {
                "CPU-only".to_string()
            }
        }
    }
}

/// Statistics for GPU backend
#[derive(Debug, Clone)]
pub struct GpuBackendStats {
    pub gpu_available: bool,
    pub gpu_validated: bool,
    pub rigidbody_count: usize,
    pub simulation_time: f32,
    pub backend_mode: String,
}

impl Default for GpuBackend {
    fn default() -> Self {
        Self {
            device: None,
            queue: None,
            buffer_manager: None,
            compute_manager: None,
            gpu_available: false,
            gpu_validated: false,
            positions: Vec::new(),
            velocities: Vec::new(),
            masses: Vec::new(),
            gravity: glam::Vec3::new(0.0, -9.81, 0.0),
            rigidbody_count: 0,
            max_rigidbodies: 1000,
            initialized: false,
            simulation_time: 0.0,
        }
    }
}

impl PhysicsBackend for GpuBackend {
    fn initialize(&mut self, max_rigidbodies: usize) -> Result<(), BackendError> {
        if self.initialized {
            return Ok(());
        }

        self.max_rigidbodies = max_rigidbodies;
        self.rigidbody_count = 0;
        self.simulation_time = 0.0;
        
        // Reserve space for data
        self.positions.reserve(max_rigidbodies);
        self.velocities.reserve(max_rigidbodies);
        self.masses.reserve(max_rigidbodies);
        
        // Check GPU availability
        self.gpu_available = Self::check_gpu_availability();
        
        if self.gpu_available {
            // Initialize GPU resources
            self.gpu_validated = pollster::block_on(async {
                self.initialize_gpu_resources().await
            });
            
            if self.gpu_validated {
                tracing::info!("GPU backend initialized with GPU compute support");
            } else {
                tracing::info!("GPU backend initialized with CPU fallback");
            }
        } else {
            tracing::info!("No GPU available, using CPU-only mode");
            self.gpu_validated = false;
        }
        
        self.initialized = true;
        Ok(())
    }

    fn step(&mut self, dt: f32) -> Result<(), BackendError> {
        if !self.initialized {
            return Err(BackendError::NotInitialized);
        }

        if self.rigidbody_count == 0 {
            return Ok(());
        }

        // For Phase 1, use CPU computation with GPU readiness
        // GPU execution will be implemented in Phase 2
        if self.gpu_available && self.gpu_validated {
            // GPU is available and validated, but for Phase 1 we still use CPU
            match pollster::block_on(async {
                self.execute_gpu_step(dt).await
            }) {
                Ok(()) => Ok(()),
                Err(_gpu_err) => {
                    // If GPU fails, fallback to CPU
                    tracing::debug!("GPU step failed, using CPU fallback");
                    self.execute_cpu_step(dt)
                }
            }
        } else {
            // No GPU available or not validated, use CPU
            self.execute_cpu_step(dt)
        }
    }

    fn name(&self) -> &str {
        if self.gpu_available && self.gpu_validated {
            "gpu-ready"
        } else if self.gpu_available {
            "gpu-unvalidated"
        } else {
            "gpu-unavailable"
        }
    }

    fn is_available() -> bool {
        Self::check_gpu_availability()
    }

    fn add_rigidbody(
        &mut self,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<usize, BackendError> {
        if !self.initialized {
            return Err(BackendError::NotInitialized);
        }

        if self.rigidbody_count >= self.max_rigidbodies {
            return Err(BackendError::CapacityExceeded);
        }

        let index = self.rigidbody_count;
        
        // Store data
        self.positions.push(position);
        self.velocities.push(velocity);
        self.masses.push(mass);
        
        self.rigidbody_count += 1;
        Ok(index)
    }

    fn get_position(&self, index: usize) -> Option<glam::Vec3> {
        self.positions.get(index).copied()
    }

    fn get_velocity(&self, index: usize) -> Option<glam::Vec3> {
        self.velocities.get(index).copied()
    }

    fn rigidbody_count(&self) -> usize {
        self.rigidbody_count
    }

    fn set_gravity(&mut self, gravity: glam::Vec3) {
        self.gravity = gravity;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_gpu_backend_creation() {
        let backend = GpuBackend::new();
        assert_eq!(backend.name(), "gpu");
        assert!(!backend.gpu_available); // Should be false in TDD version
    }
    
    #[test]
    fn test_gpu_backend_initialization() {
        let mut backend = GpuBackend::new();
        // Should fail since GPU is not available in TDD version
        let result = backend.initialize(100);
        assert!(result.is_err());
        
        if let Err(BackendError::GpuUnavailable(_)) = result {
            // Expected error
        } else {
            panic!("Expected GpuUnavailable error");
        }
    }
}