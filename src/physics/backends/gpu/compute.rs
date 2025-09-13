//! Compute shader execution manager for GPU physics
#![allow(dead_code)]

use super::buffers::BufferManager;
use super::super::traits::BackendError;
use wgpu::util::DeviceExt;

/// Manages compute shader execution for physics simulation
pub struct ComputeManager {
    bind_group_layout: Option<wgpu::BindGroupLayout>,
    bind_group: Option<wgpu::BindGroup>,
}

impl ComputeManager {
    pub fn new() -> Self {
        Self {
            bind_group_layout: None,
            bind_group: None,
        }
    }
    
    /// Create the physics compute pipeline
    pub fn create_physics_pipeline(&mut self, device: &wgpu::Device) -> Result<wgpu::ComputePipeline, BackendError> {
        // Create bind group layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Physics Compute Bind Group Layout"),
            entries: &[
                // Position buffer
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
                // Velocity buffer
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
                // Mass buffer
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Force buffer  
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
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
                        min_binding_size: Some(std::num::NonZeroU64::new(std::mem::size_of::<PhysicsUniforms>() as u64).unwrap()),
                    },
                    count: None,
                },
            ],
        });
        
        self.bind_group_layout = Some(bind_group_layout);
        
        // Load compute shader
        let shader_source = include_str!("../../../../assets/shaders/compute_physics.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Physics Compute Shader"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });
        
        // Create compute pipeline layout
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Physics Compute Pipeline Layout"),
            bind_group_layouts: &[self.bind_group_layout.as_ref().unwrap()],
            push_constant_ranges: &[],
        });
        
        // Create compute pipeline
        let compute_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Physics Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: Some("physics_step"),
            compilation_options: Default::default(),
            cache: None,
        });
        
        Ok(compute_pipeline)
    }
    
    /// Create bind group with buffer bindings
    pub fn create_bind_group(
        &mut self,
        device: &wgpu::Device,
        buffer_manager: &BufferManager,
        uniform_buffer: &wgpu::Buffer,
    ) -> Result<(), BackendError> {
        let bind_group_layout = self.bind_group_layout.as_ref().ok_or(BackendError::NotInitialized)?;
        
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Physics Compute Bind Group"),
            layout: bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer_manager.position_buffer().as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: buffer_manager.velocity_buffer().as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: buffer_manager.mass_buffer().as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: buffer_manager.force_buffer().as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: uniform_buffer.as_entire_binding(),
                },
            ],
        });
        
        self.bind_group = Some(bind_group);
        Ok(())
    }
    
    /// Execute a single physics time step on GPU
    pub fn execute_physics_step(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        compute_pipeline: &wgpu::ComputePipeline,
        buffer_manager: &BufferManager,
        rigidbody_count: usize,
        dt: f32,
        gravity: glam::Vec3,
    ) -> Result<(), BackendError> {
        // Create uniform buffer with simulation parameters
        let uniform_data = PhysicsUniforms {
            dt,
            gravity: [gravity.x, gravity.y, gravity.z, 0.0], // vec4 in WGSL
            rigidbody_count: rigidbody_count as u32,
            _padding0: 0,
            _padding1: 0,
            _padding2: 0,
            _final_padding: [0; 3], // Extra padding to reach 48 bytes
        };
        
        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Physics Uniforms"),
            contents: bytemuck::cast_slice(&[uniform_data]),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        
        // Create bind group if not exists
        if self.bind_group.is_none() {
            self.create_bind_group(device, buffer_manager, &uniform_buffer)?;
        }
        
        let bind_group = self.bind_group.as_ref().ok_or(BackendError::NotInitialized)?;
        
        // Create command encoder
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Physics Compute Encoder"),
        });
        
        // Dispatch compute shader
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Physics Compute Pass"),
                timestamp_writes: None,
            });
            
            compute_pass.set_pipeline(compute_pipeline);
            compute_pass.set_bind_group(0, bind_group, &[]);
            
            // Dispatch with one thread per rigid body
            let workgroup_size = 64; // Typical workgroup size
            let num_workgroups = (rigidbody_count + workgroup_size - 1) / workgroup_size;
            compute_pass.dispatch_workgroups(num_workgroups as u32, 1, 1);
        }
        
        // Submit commands
        queue.submit(Some(encoder.finish()));
        
        Ok(())
    }
}

/// Uniform data structure for compute shader
/// Must exactly match WGSL PhysicsUniforms struct layout
#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct PhysicsUniforms {
    dt: f32,                // 4 bytes
    gravity: [f32; 4],      // 16 bytes (vec4 in WGSL) 
    rigidbody_count: u32,   // 4 bytes
    _padding0: u32,         // 4 bytes
    _padding1: u32,         // 4 bytes
    _padding2: u32,         // 4 bytes
    // Add extra padding to reach 48 bytes as expected by WGSL
    _final_padding: [u32; 3], // 12 bytes  
    // Total: 48 bytes
}