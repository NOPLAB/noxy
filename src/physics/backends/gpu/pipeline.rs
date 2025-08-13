//! GPU compute pipeline configuration and management

use super::super::traits::BackendError;

/// Configuration for GPU compute pipeline
pub struct PipelineConfig {
    pub workgroup_size: u32,
    pub max_rigidbodies: u32,
    pub integration_method: IntegrationMethod,
}

/// Available integration methods for GPU computation
#[derive(Clone, Copy, Debug)]
pub enum IntegrationMethod {
    /// Explicit Euler integration (simple, fast)
    Euler,
    /// Velocity Verlet integration (better energy conservation)
    Verlet,
    /// Runge-Kutta 4th order (high accuracy)
    RungeKutta4,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            workgroup_size: 64,
            max_rigidbodies: 10000,
            integration_method: IntegrationMethod::Verlet,
        }
    }
}

/// GPU pipeline manager for different physics algorithms
pub struct PipelineManager {
    force_computation_pipeline: Option<wgpu::ComputePipeline>,
    integration_pipeline: Option<wgpu::ComputePipeline>,
    collision_pipeline: Option<wgpu::ComputePipeline>,
    config: PipelineConfig,
}

impl PipelineManager {
    pub fn new(config: PipelineConfig) -> Self {
        Self {
            force_computation_pipeline: None,
            integration_pipeline: None,
            collision_pipeline: None,
            config,
        }
    }
    
    /// Initialize all compute pipelines
    pub fn initialize_pipelines(&mut self, device: &wgpu::Device) -> Result<(), BackendError> {
        // Create bind group layout for physics buffers
        let bind_group_layout = self.create_physics_bind_group_layout(device);
        
        // Create pipeline layout
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Physics Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });
        
        // Force computation pipeline
        self.force_computation_pipeline = Some(self.create_force_pipeline(device, &pipeline_layout)?);
        
        // Integration pipeline
        self.integration_pipeline = Some(self.create_integration_pipeline(device, &pipeline_layout)?);
        
        // Collision detection pipeline
        self.collision_pipeline = Some(self.create_collision_pipeline(device, &pipeline_layout)?);
        
        Ok(())
    }
    
    fn create_physics_bind_group_layout(&self, device: &wgpu::Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Physics Bind Group Layout"),
            entries: &[
                // Position buffer (read/write)
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Velocity buffer (read/write)
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Force buffer (read/write)
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Mass buffer (read-only)
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Uniform buffer for simulation parameters
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        })
    }
    
    fn create_force_pipeline(
        &self,
        device: &wgpu::Device,
        layout: &wgpu::PipelineLayout,
    ) -> Result<wgpu::ComputePipeline, BackendError> {
        let shader_source = include_str!("../../../../assets/shaders/force_computation.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Force Computation Shader"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });
        
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Force Computation Pipeline"),
            layout: Some(layout),
            module: &shader,
            entry_point: Some("compute_forces"),
            compilation_options: Default::default(),
            cache: None,
        });
        
        Ok(pipeline)
    }
    
    fn create_integration_pipeline(
        &self,
        device: &wgpu::Device,
        layout: &wgpu::PipelineLayout,
    ) -> Result<wgpu::ComputePipeline, BackendError> {
        let (shader_source, entry_point) = match self.config.integration_method {
            IntegrationMethod::Euler => {
                (include_str!("../../../../assets/shaders/euler_integration.wgsl"), "euler_integrate")
            }
            IntegrationMethod::Verlet => {
                (include_str!("../../../../assets/shaders/verlet_integration.wgsl"), "verlet_integrate")
            }
            IntegrationMethod::RungeKutta4 => {
                (include_str!("../../../../assets/shaders/rk4_integration.wgsl"), "rk4_integrate")
            }
        };
        
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Integration Shader"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });
        
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Integration Pipeline"),
            layout: Some(layout),
            module: &shader,
            entry_point: Some(entry_point),
            compilation_options: Default::default(),
            cache: None,
        });
        
        Ok(pipeline)
    }
    
    fn create_collision_pipeline(
        &self,
        device: &wgpu::Device,
        layout: &wgpu::PipelineLayout,
    ) -> Result<wgpu::ComputePipeline, BackendError> {
        let shader_source = include_str!("../../../../assets/shaders/collision_detection.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Collision Detection Shader"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });
        
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Collision Detection Pipeline"),
            layout: Some(layout),
            module: &shader,
            entry_point: Some("detect_collisions"),
            compilation_options: Default::default(),
            cache: None,
        });
        
        Ok(pipeline)
    }
    
    /// Get the force computation pipeline
    pub fn force_pipeline(&self) -> Result<&wgpu::ComputePipeline, BackendError> {
        self.force_computation_pipeline.as_ref().ok_or(BackendError::NotInitialized)
    }
    
    /// Get the integration pipeline
    pub fn integration_pipeline(&self) -> Result<&wgpu::ComputePipeline, BackendError> {
        self.integration_pipeline.as_ref().ok_or(BackendError::NotInitialized)
    }
    
    /// Get the collision detection pipeline
    pub fn collision_pipeline(&self) -> Result<&wgpu::ComputePipeline, BackendError> {
        self.collision_pipeline.as_ref().ok_or(BackendError::NotInitialized)
    }
    
    /// Get workgroup size for dispatch calculations
    pub fn workgroup_size(&self) -> u32 {
        self.config.workgroup_size
    }
}