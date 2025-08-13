//! GPU buffer management for rigid body physics data


use super::super::traits::BackendError;

/// Manages GPU buffers for physics simulation
pub struct BufferManager {
    position_buffer: Option<wgpu::Buffer>,
    velocity_buffer: Option<wgpu::Buffer>,
    mass_buffer: Option<wgpu::Buffer>,
    force_buffer: Option<wgpu::Buffer>,
    
    // Read-back buffers for CPU access
    position_read_buffer: Option<wgpu::Buffer>,
    velocity_read_buffer: Option<wgpu::Buffer>,
    
    capacity: usize,
}

impl BufferManager {
    pub fn new() -> Self {
        Self {
            position_buffer: None,
            velocity_buffer: None,
            mass_buffer: None,
            force_buffer: None,
            position_read_buffer: None,
            velocity_read_buffer: None,
            capacity: 0,
        }
    }
    
    /// Allocate all necessary buffers for rigid body simulation
    pub fn allocate_rigidbody_buffers(
        &mut self,
        device: &wgpu::Device,
        capacity: usize,
    ) -> Result<(), BackendError> {
        self.capacity = capacity;
        
        let buffer_size = (capacity * 3 * std::mem::size_of::<f32>()) as u64; // 3 floats per Vec3
        let mass_buffer_size = (capacity * std::mem::size_of::<f32>()) as u64; // 1 float per mass
        
        // Position buffer (Vec3 array)
        self.position_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Position Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        // Velocity buffer (Vec3 array)
        self.velocity_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Velocity Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        // Mass buffer (f32 array)
        self.mass_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Mass Buffer"),
            size: mass_buffer_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        // Force buffer (Vec3 array) - used for accumulating forces
        self.force_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Force Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        // Read-back buffers for CPU access
        self.position_read_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Position Read Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        self.velocity_read_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Velocity Read Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        
        Ok(())
    }
    
    /// Update a single rigid body's data
    pub fn update_rigidbody(
        &self,
        queue: &wgpu::Queue,
        index: usize,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<(), BackendError> {
        if index >= self.capacity {
            return Err(BackendError::IndexOutOfBounds);
        }
        
        let position_offset = (index * 3 * std::mem::size_of::<f32>()) as u64;
        let velocity_offset = (index * 3 * std::mem::size_of::<f32>()) as u64;
        let mass_offset = (index * std::mem::size_of::<f32>()) as u64;
        
        // Write position data
        if let Some(buffer) = &self.position_buffer {
            queue.write_buffer(buffer, position_offset, bytemuck::cast_slice(&[position.x, position.y, position.z]));
        }
        
        // Write velocity data
        if let Some(buffer) = &self.velocity_buffer {
            queue.write_buffer(buffer, velocity_offset, bytemuck::cast_slice(&[velocity.x, velocity.y, velocity.z]));
        }
        
        // Write mass data
        if let Some(buffer) = &self.mass_buffer {
            queue.write_buffer(buffer, mass_offset, bytemuck::cast_slice(&[mass]));
        }
        
        Ok(())
    }
    
    /// Read position from GPU (blocking operation)
    pub fn read_position(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        index: usize,
    ) -> Result<glam::Vec3, BackendError> {
        if index >= self.capacity {
            return Err(BackendError::IndexOutOfBounds);
        }
        
        let position_buffer = self.position_buffer.as_ref().ok_or(BackendError::NotInitialized)?;
        let read_buffer = self.position_read_buffer.as_ref().ok_or(BackendError::NotInitialized)?;
        
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Read Position Encoder"),
        });
        
        // Copy from storage buffer to read buffer
        let offset = (index * 3 * std::mem::size_of::<f32>()) as u64;
        let size = (3 * std::mem::size_of::<f32>()) as u64;
        
        encoder.copy_buffer_to_buffer(position_buffer, offset, read_buffer, 0, size);
        queue.submit(Some(encoder.finish()));
        
        // Map and read the buffer
        let buffer_slice = read_buffer.slice(0..size);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            tx.send(result).unwrap();
        });
        
        device.poll(wgpu::MaintainBase::Wait);
        rx.recv().unwrap().map_err(|e| BackendError::GpuError(e.to_string()))?;
        
        let data = buffer_slice.get_mapped_range();
        let floats: &[f32] = bytemuck::cast_slice(&data);
        let position = glam::Vec3::new(floats[0], floats[1], floats[2]);
        
        drop(data);
        read_buffer.unmap();
        
        Ok(position)
    }
    
    /// Read velocity from GPU (blocking operation)
    pub fn read_velocity(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        index: usize,
    ) -> Result<glam::Vec3, BackendError> {
        if index >= self.capacity {
            return Err(BackendError::IndexOutOfBounds);
        }
        
        let velocity_buffer = self.velocity_buffer.as_ref().ok_or(BackendError::NotInitialized)?;
        let read_buffer = self.velocity_read_buffer.as_ref().ok_or(BackendError::NotInitialized)?;
        
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Read Velocity Encoder"),
        });
        
        // Copy from storage buffer to read buffer
        let offset = (index * 3 * std::mem::size_of::<f32>()) as u64;
        let size = (3 * std::mem::size_of::<f32>()) as u64;
        
        encoder.copy_buffer_to_buffer(velocity_buffer, offset, read_buffer, 0, size);
        queue.submit(Some(encoder.finish()));
        
        // Map and read the buffer
        let buffer_slice = read_buffer.slice(0..size);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            tx.send(result).unwrap();
        });
        
        device.poll(wgpu::MaintainBase::Wait);
        rx.recv().unwrap().map_err(|e| BackendError::GpuError(e.to_string()))?;
        
        let data = buffer_slice.get_mapped_range();
        let floats: &[f32] = bytemuck::cast_slice(&data);
        let velocity = glam::Vec3::new(floats[0], floats[1], floats[2]);
        
        drop(data);
        read_buffer.unmap();
        
        Ok(velocity)
    }
    
    // Getter methods for buffer access
    pub fn position_buffer(&self) -> &wgpu::Buffer {
        self.position_buffer.as_ref().unwrap()
    }
    
    pub fn velocity_buffer(&self) -> &wgpu::Buffer {
        self.velocity_buffer.as_ref().unwrap()
    }
    
    pub fn mass_buffer(&self) -> &wgpu::Buffer {
        self.mass_buffer.as_ref().unwrap()
    }
    
    pub fn force_buffer(&self) -> &wgpu::Buffer {
        self.force_buffer.as_ref().unwrap()
    }
}