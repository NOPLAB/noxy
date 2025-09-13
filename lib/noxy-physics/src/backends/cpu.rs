//! # CPU Physics Backend
//! 
//! CPU-based physics computation using Rayon for parallel processing.

use super::traits::*;
use crate::core::rigidbody::{RigidBody, RigidBodyHandle};
use std::collections::HashMap;
use rayon::prelude::*;
use anyhow::Result;

/// Configuration for CPU backend
#[derive(Debug, Clone)]
pub struct CPUBackendConfig {
    /// Number of threads to use (None for automatic)
    pub thread_count: Option<usize>,
    /// Minimum elements per thread for parallel processing
    pub min_elements_per_thread: usize,
    /// Enable parallel force application
    pub parallel_forces: bool,
    /// Enable parallel integration
    pub parallel_integration: bool,
}

impl Default for CPUBackendConfig {
    fn default() -> Self {
        Self {
            thread_count: None,
            min_elements_per_thread: 64,
            parallel_forces: true,
            parallel_integration: true,
        }
    }
}

/// CPU-based physics backend
pub struct CPUBackend {
    config: CPUBackendConfig,
    rigid_bodies: HashMap<RigidBodyHandle, RigidBody>,
    next_handle: u64,
    initialized: bool,
}

impl Default for CPUBackend {
    fn default() -> Self {
        Self::new(CPUBackendConfig::default())
    }
}

impl CPUBackend {
    /// Create a new CPU backend with configuration
    pub fn new(config: CPUBackendConfig) -> Self {
        Self {
            config,
            rigid_bodies: HashMap::new(),
            next_handle: 1,
            initialized: false,
        }
    }
    
    /// Configure the thread pool if needed
    fn configure_threads(&self) -> Result<()> {
        if let Some(thread_count) = self.config.thread_count {
            rayon::ThreadPoolBuilder::new()
                .num_threads(thread_count)
                .build_global()?;
        }
        Ok(())
    }
    
    /// Apply forces in parallel if beneficial
    fn apply_forces_parallel<F>(&self, bodies: &mut [RigidBody], force_fn: F)
    where
        F: Fn(&mut RigidBody) + Send + Sync,
    {
        if self.config.parallel_forces && bodies.len() >= self.config.min_elements_per_thread {
            bodies.par_iter_mut().for_each(force_fn);
        } else {
            bodies.iter_mut().for_each(force_fn);
        }
    }
    
    /// Integrate motion in parallel if beneficial
    fn integrate_parallel<F>(&self, bodies: &mut [RigidBody], integration_fn: F)
    where
        F: Fn(&mut RigidBody) + Send + Sync,
    {
        if self.config.parallel_integration && bodies.len() >= self.config.min_elements_per_thread {
            bodies.par_iter_mut().for_each(integration_fn);
        } else {
            bodies.iter_mut().for_each(integration_fn);
        }
    }
}

impl PhysicsBackend for CPUBackend {
    fn info(&self) -> BackendInfo {
        BackendInfo {
            name: "CPU Backend".to_string(),
            backend_type: super::BackendType::CPU,
            max_bodies: 10000, // Reasonable limit for CPU
            features: BackendFeatures {
                parallel: true,
                gpu_acceleration: false,
                double_precision: true,
            },
        }
    }
    
    fn initialize(&mut self) -> Result<()> {
        self.configure_threads()?;
        self.initialized = true;
        log::info!("CPU physics backend initialized with {} threads", 
                  rayon::current_num_threads());
        Ok(())
    }
    
    fn add_rigidbody(&mut self, body: RigidBody) -> Result<RigidBodyHandle> {
        let handle = RigidBodyHandle(self.next_handle);
        self.next_handle += 1;
        self.rigid_bodies.insert(handle, body);
        Ok(handle)
    }
    
    fn remove_rigidbody(&mut self, handle: RigidBodyHandle) -> Result<()> {
        self.rigid_bodies.remove(&handle)
            .ok_or_else(|| anyhow::anyhow!("Invalid rigid body handle: {:?}", handle))?;
        Ok(())
    }
    
    fn update_rigidbody(&mut self, handle: RigidBodyHandle, body: &RigidBody) -> Result<()> {
        if let Some(existing_body) = self.rigid_bodies.get_mut(&handle) {
            *existing_body = body.clone();
            Ok(())
        } else {
            Err(anyhow::anyhow!("Invalid rigid body handle: {:?}", handle))
        }
    }
    
    fn get_rigidbody(&self, handle: RigidBodyHandle) -> Result<RigidBody> {
        self.rigid_bodies.get(&handle)
            .cloned()
            .ok_or_else(|| anyhow::anyhow!("Invalid rigid body handle: {:?}", handle))
    }
    
    fn step(&mut self, dt: f32) -> Result<()> {
        if !self.initialized {
            return Err(anyhow::anyhow!("Backend not initialized"));
        }
        
        // Collect bodies into a vector for parallel processing
        let mut bodies: Vec<_> = self.rigid_bodies.values().cloned().collect();
        
        // Apply gravity (simplified force application)
        self.apply_forces_parallel(&mut bodies, |body| {
            if !body.is_static() {
                // Apply gravity
                body.velocity.y -= 9.81 * dt;
            }
        });
        
        // Integrate motion
        self.integrate_parallel(&mut bodies, |body| {
            if !body.is_static() {
                // Simple Euler integration
                body.position += body.velocity * dt;
                
                // Angular integration (simplified)
                let angular_velocity_quat = glam::Quat::from_xyzw(
                    body.angular_velocity.x * 0.5,
                    body.angular_velocity.y * 0.5,
                    body.angular_velocity.z * 0.5,
                    0.0,
                );
                body.rotation = (body.rotation + angular_velocity_quat * body.rotation * dt).normalize();
                
                // Apply damping
                body.velocity *= 0.99;
                body.angular_velocity *= 0.99;
            }
        });
        
        // Update the hash map with processed bodies
        let handles: Vec<_> = self.rigid_bodies.keys().cloned().collect();
        for (i, handle) in handles.iter().enumerate() {
            if i < bodies.len() {
                self.rigid_bodies.insert(*handle, bodies[i].clone());
            }
        }
        
        Ok(())
    }
    
    fn synchronize(&mut self) -> Result<()> {
        // CPU backend doesn't need synchronization
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{shapes::Shape, materials::Material};
    use glam::{Vec3, Quat};

    #[test]
    fn test_cpu_backend_creation() {
        let backend = CPUBackend::default();
        assert!(!backend.initialized);
    }

    #[test]
    fn test_cpu_backend_initialization() {
        let mut backend = CPUBackend::default();
        assert!(backend.initialize().is_ok());
        assert!(backend.initialized);
    }

    #[test]
    fn test_rigidbody_management() {
        let mut backend = CPUBackend::default();
        backend.initialize().unwrap();
        
        let body = RigidBody::new(
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
            1.0,
            Shape::Sphere { radius: 1.0 },
            Material::default(),
        );
        
        let handle = backend.add_rigidbody(body.clone()).unwrap();
        let retrieved_body = backend.get_rigidbody(handle).unwrap();
        
        assert_eq!(retrieved_body.mass, body.mass);
        
        assert!(backend.remove_rigidbody(handle).is_ok());
        assert!(backend.get_rigidbody(handle).is_err());
    }

    #[test]
    fn test_physics_step() {
        let mut backend = CPUBackend::default();
        backend.initialize().unwrap();
        
        let mut body = RigidBody::default();
        body.position = Vec3::new(0.0, 10.0, 0.0);
        body.velocity = Vec3::ZERO;
        
        let handle = backend.add_rigidbody(body).unwrap();
        
        // Step physics (should apply gravity)
        backend.step(0.016).unwrap(); // 60 FPS
        
        let updated_body = backend.get_rigidbody(handle).unwrap();
        
        // Body should have fallen and gained downward velocity
        assert!(updated_body.position.y < 10.0);
        assert!(updated_body.velocity.y < 0.0);
    }
}