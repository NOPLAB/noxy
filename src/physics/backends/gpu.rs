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
                        let mut buffer_manager = buffers::BufferManager::new();
                        let device_ref = self.device.as_ref().unwrap();
                        
                        // Allocate buffers for maximum capacity
                        if let Err(e) = buffer_manager.allocate_rigidbody_buffers(device_ref, self.max_rigidbodies) {
                            tracing::warn!("Failed to allocate GPU buffers: {:?}", e);
                            return false;
                        }
                        
                        self.buffer_manager = Some(buffer_manager);
                        self.compute_manager = Some(compute::ComputeManager::new());
                        
                        tracing::info!("GPU resources initialized successfully with {} max rigidbodies", self.max_rigidbodies);
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

    /// Execute physics step on GPU using compute shaders
    async fn execute_gpu_step(&mut self, dt: f32) -> Result<(), BackendError> {
        // Ensure we have valid GPU resources
        let device = self.device.as_ref().ok_or(BackendError::NotInitialized)?;
        let queue = self.queue.as_ref().ok_or(BackendError::NotInitialized)?;
        let buffer_manager = self.buffer_manager.as_mut().ok_or(BackendError::NotInitialized)?;
        let compute_manager = self.compute_manager.as_mut().ok_or(BackendError::NotInitialized)?;

        if self.rigidbody_count == 0 {
            return Ok(());
        }

        // Update GPU buffers with current CPU data
        buffer_manager.update_positions(device, queue, &self.positions)?;
        buffer_manager.update_velocities(device, queue, &self.velocities)?;
        buffer_manager.update_masses(device, queue, &self.masses)?;

        // Create compute pipeline if not exists
        let compute_pipeline = compute_manager.create_physics_pipeline(device)?;

        // Execute GPU compute step
        compute_manager.execute_physics_step(
            device,
            queue,
            &compute_pipeline,
            buffer_manager,
            self.rigidbody_count,
            dt,
            self.gravity,
        )?;

        // Read back results from GPU to CPU
        let new_positions = buffer_manager.read_positions(device, queue, self.rigidbody_count).await?;
        let new_velocities = buffer_manager.read_velocities(device, queue, self.rigidbody_count).await?;

        // Update CPU-side data
        self.positions = new_positions;
        self.velocities = new_velocities;
        
        self.simulation_time += dt;
        
        tracing::debug!("GPU physics step completed: {} bodies, dt={:.6}s", self.rigidbody_count, dt);
        Ok(())
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
        // In headless environments without GPU, should return "gpu-unavailable"
        assert_eq!(backend.name(), "gpu-unavailable");
        assert!(!backend.gpu_available); // Should be false in headless/test environment
    }
    
    #[test]
    fn test_gpu_backend_initialization() {
        let mut backend = GpuBackend::new();
        // In headless environments, initialization should succeed but GPU won't be available
        let result = backend.initialize(100);
        
        // Check if GPU is not available in test environment
        if !backend.gpu_available {
            // This is expected in headless/CI environments
            assert!(result.is_ok(), "Initialization should succeed even without GPU");
            assert_eq!(backend.name(), "gpu-unavailable");
        } else {
            // If GPU is available, initialization might still fail due to validation
            // This branch handles actual GPU environments
            match result {
                Ok(_) => {
                    assert!(backend.gpu_available);
                }
                Err(BackendError::GpuUnavailable(_)) => {
                    // This is also acceptable
                    assert!(!backend.gpu_available);
                }
                Err(e) => panic!("Unexpected error: {:?}", e),
            }
        }
    }
}