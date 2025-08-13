//! GPU physics backend implementation using WGPU compute shaders

mod buffers;
mod compute;
mod pipeline;

use super::traits::{BackendError, PhysicsBackend};


/// GPU-accelerated physics backend
pub struct GpuBackend {
    device: wgpu::Device,
    queue: wgpu::Queue,
    compute_pipeline: Option<wgpu::ComputePipeline>,
    
    // Buffers for rigid body data
    position_buffer: Option<wgpu::Buffer>,
    velocity_buffer: Option<wgpu::Buffer>,
    mass_buffer: Option<wgpu::Buffer>,
    force_buffer: Option<wgpu::Buffer>,
    
    // Simulation parameters
    gravity: glam::Vec3,
    rigidbody_count: usize,
    max_rigidbodies: usize,
    
    // GPU memory management
    buffer_manager: buffers::BufferManager,
    compute_manager: compute::ComputeManager,
}

impl GpuBackend {
    /// Create new GPU backend instance
    pub async fn new() -> Result<Self, BackendError> {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..Default::default()
        });

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: None,
            })
            .await
            .map_err(|e| BackendError::InitializationFailed(format!("Failed to find adapter: {}", e)))?;

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("Physics GPU Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
                ..Default::default()
            })
            .await
            .map_err(|e| BackendError::InitializationFailed(e.to_string()))?;

        Ok(Self {
            device,
            queue,
            compute_pipeline: None,
            position_buffer: None,
            velocity_buffer: None,
            mass_buffer: None,
            force_buffer: None,
            gravity: glam::Vec3::new(0.0, -9.81, 0.0),
            rigidbody_count: 0,
            max_rigidbodies: 0,
            buffer_manager: buffers::BufferManager::new(),
            compute_manager: compute::ComputeManager::new(),
        })
    }
    
    /// Initialize compute shaders and buffers
    fn setup_compute_pipeline(&mut self) -> Result<(), BackendError> {
        let compute_pipeline = self.compute_manager.create_physics_pipeline(&self.device)?;
        self.compute_pipeline = Some(compute_pipeline);
        Ok(())
    }
    
    /// Allocate GPU buffers for rigid bodies
    fn allocate_buffers(&mut self) -> Result<(), BackendError> {
        self.buffer_manager.allocate_rigidbody_buffers(
            &self.device,
            self.max_rigidbodies,
        )?;
        
        // Store buffer references for easier access
        self.position_buffer = Some(self.buffer_manager.position_buffer().clone());
        self.velocity_buffer = Some(self.buffer_manager.velocity_buffer().clone());
        self.mass_buffer = Some(self.buffer_manager.mass_buffer().clone());
        self.force_buffer = Some(self.buffer_manager.force_buffer().clone());
        
        Ok(())
    }
}

impl Default for GpuBackend {
    fn default() -> Self {
        // Note: This will not work because new() is async
        // Users should use GpuBackend::new().await
        panic!("Use GpuBackend::new().await instead of default()")
    }
}

impl PhysicsBackend for GpuBackend {
    fn initialize(&mut self, max_rigidbodies: usize) -> Result<(), BackendError> {
        self.max_rigidbodies = max_rigidbodies;
        
        // Setup compute pipeline
        self.setup_compute_pipeline()?;
        
        // Allocate GPU buffers
        self.allocate_buffers()?;
        
        Ok(())
    }

    fn step(&mut self, dt: f32) -> Result<(), BackendError> {
        if self.rigidbody_count == 0 {
            return Ok(()); // Nothing to simulate
        }

        let compute_pipeline = self.compute_pipeline
            .as_ref()
            .ok_or_else(|| BackendError::NotInitialized)?;

        // Execute physics computation on GPU
        self.compute_manager.execute_physics_step(
            &self.device,
            &self.queue,
            compute_pipeline,
            &self.buffer_manager,
            self.rigidbody_count,
            dt,
            self.gravity,
        )?;

        Ok(())
    }

    fn name(&self) -> &str {
        "GPU (WGPU Compute)"
    }

    fn is_available() -> bool {
        // Check if GPU with compute capabilities is available
        // For now, assume true - actual check would be async
        true
    }

    fn add_rigidbody(
        &mut self,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<usize, BackendError> {
        if self.rigidbody_count >= self.max_rigidbodies {
            return Err(BackendError::CapacityExceeded);
        }

        let index = self.rigidbody_count;
        
        // Update GPU buffers with new rigid body data
        self.buffer_manager.update_rigidbody(
            &self.queue,
            index,
            position,
            velocity,
            mass,
        )?;
        
        self.rigidbody_count += 1;
        Ok(index)
    }

    fn get_position(&self, index: usize) -> Option<glam::Vec3> {
        if index >= self.rigidbody_count {
            return None;
        }
        
        // Read back position from GPU buffer
        self.buffer_manager.read_position(&self.device, &self.queue, index)
            .ok()
    }

    fn get_velocity(&self, index: usize) -> Option<glam::Vec3> {
        if index >= self.rigidbody_count {
            return None;
        }
        
        // Read back velocity from GPU buffer  
        self.buffer_manager.read_velocity(&self.device, &self.queue, index)
            .ok()
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
    
    #[tokio::test]
    async fn test_gpu_backend_creation() {
        let result = GpuBackend::new().await;
        // This test may fail on systems without GPU support
        match result {
            Ok(_) => println!("GPU backend created successfully"),
            Err(e) => println!("GPU backend creation failed: {}", e),
        }
    }
    
    #[tokio::test]
    async fn test_gpu_backend_initialization() {
        if let Ok(mut backend) = GpuBackend::new().await {
            let result = backend.initialize(100);
            assert!(result.is_ok());
            assert_eq!(backend.rigidbody_count(), 0);
            assert_eq!(backend.name(), "GPU (WGPU Compute)");
        }
    }
}