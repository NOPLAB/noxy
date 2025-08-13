// CPU backend implementation using Rayon for parallel processing

use super::traits::{BackendError, BackendStats, PhysicsBackend};
use crate::physics::core::forces::ForceAccumulator;
use crate::physics::core::integration::verlet_step;
use glam::Vec3;
use std::time::Instant;

/// CPU-based physics backend
pub struct CpuBackend {
    rigidbodies: Vec<RigidBody>,
    gravity: Vec3,
    stats: BackendStats,
    max_rigidbodies: usize,
}

/// Simple rigid body representation for CPU backend
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f32,
    pub force_accumulator: ForceAccumulator,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            mass: 1.0,
            force_accumulator: ForceAccumulator::new(),
        }
    }
}

impl CpuBackend {
    pub fn new() -> Self {
        Self {
            rigidbodies: Vec::new(),
            gravity: Vec3::new(0.0, -9.81, 0.0),
            stats: BackendStats {
                backend_name: "CPU".to_string(),
                ..Default::default()
            },
            max_rigidbodies: 0,
        }
    }

    pub fn add_rigidbody(&mut self, rigidbody: RigidBody) -> usize {
        self.rigidbodies.push(rigidbody);
        self.rigidbodies.len() - 1
    }

    pub fn get_rigidbody(&self, index: usize) -> Option<&RigidBody> {
        self.rigidbodies.get(index)
    }

    pub fn get_rigidbody_mut(&mut self, index: usize) -> Option<&mut RigidBody> {
        self.rigidbodies.get_mut(index)
    }

    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.gravity = gravity;
    }

    pub fn stats(&self) -> &BackendStats {
        &self.stats
    }

    /// Apply forces to all rigid bodies
    fn apply_forces(&mut self) {
        for rigidbody in &mut self.rigidbodies {
            rigidbody.force_accumulator.reset();
            rigidbody
                .force_accumulator
                .add_gravitational_force(rigidbody.mass, self.gravity);
        }
    }

    /// Integrate motion for all rigid bodies
    fn integrate_motion(&mut self, dt: f32) {
        // Use Rayon for parallel processing
        use rayon::prelude::*;

        self.rigidbodies.par_iter_mut().for_each(|rigidbody| {
            let acceleration = rigidbody.force_accumulator.total() / rigidbody.mass;
            let (new_position, new_velocity) =
                verlet_step(rigidbody.position, rigidbody.velocity, acceleration, dt);
            rigidbody.position = new_position;
            rigidbody.velocity = new_velocity;
        });
    }
}

impl PhysicsBackend for CpuBackend {
    fn initialize(&mut self, max_rigidbodies: usize) -> Result<(), BackendError> {
        self.max_rigidbodies = max_rigidbodies;
        self.rigidbodies.reserve(max_rigidbodies);
        Ok(())
    }

    fn step(&mut self, dt: f32) -> Result<(), BackendError> {
        let start_time = Instant::now();

        // Apply forces
        self.apply_forces();

        // Integrate motion
        self.integrate_motion(dt);

        let step_time_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        self.stats.update(step_time_ms);

        Ok(())
    }

    fn name(&self) -> &str {
        "CPU"
    }

    fn is_available() -> bool {
        true // CPU backend is always available
    }

    fn add_rigidbody(
        &mut self,
        position: glam::Vec3,
        velocity: glam::Vec3,
        mass: f32,
    ) -> Result<usize, BackendError> {
        if mass <= 0.0 {
            return Err(BackendError::InvalidParameters(
                "Mass must be positive".to_string(),
            ));
        }

        if !position.is_finite() || !velocity.is_finite() {
            return Err(BackendError::InvalidParameters(
                "Position and velocity must be finite".to_string(),
            ));
        }

        if self.rigidbodies.len() >= self.max_rigidbodies {
            return Err(BackendError::InvalidParameters(
                "Maximum number of rigid bodies reached".to_string(),
            ));
        }

        let rigidbody = RigidBody {
            position,
            velocity,
            mass,
            force_accumulator: ForceAccumulator::new(),
        };

        self.rigidbodies.push(rigidbody);
        Ok(self.rigidbodies.len() - 1)
    }

    fn get_position(&self, index: usize) -> Option<glam::Vec3> {
        self.rigidbodies.get(index).map(|rb| rb.position)
    }

    fn get_velocity(&self, index: usize) -> Option<glam::Vec3> {
        self.rigidbodies.get(index).map(|rb| rb.velocity)
    }

    fn rigidbody_count(&self) -> usize {
        self.rigidbodies.len()
    }

    fn set_gravity(&mut self, gravity: glam::Vec3) {
        self.gravity = gravity;
    }
}

impl Default for CpuBackend {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpu_backend_initialization() {
        let mut backend = CpuBackend::new();
        assert!(backend.initialize(100).is_ok());
        assert_eq!(backend.name(), "CPU");
        assert!(CpuBackend::is_available());
    }

    #[test]
    fn test_rigidbody_creation() {
        let mut backend = CpuBackend::new();
        let rigidbody = RigidBody {
            position: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(0.0, 0.0, 0.0),
            mass: 5.0,
            force_accumulator: ForceAccumulator::new(),
        };

        let index = backend.add_rigidbody(rigidbody);
        assert_eq!(index, 0);

        let retrieved = backend.get_rigidbody(index).unwrap();
        assert_eq!(retrieved.position, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(retrieved.mass, 5.0);
    }

    #[test]
    fn test_gravity_simulation() {
        let mut backend = CpuBackend::new();
        backend.initialize(1).unwrap();

        let rigidbody = RigidBody {
            position: Vec3::new(0.0, 10.0, 0.0),
            velocity: Vec3::ZERO,
            mass: 1.0,
            force_accumulator: ForceAccumulator::new(),
        };

        let index = backend.add_rigidbody(rigidbody);
        let dt = 0.1;

        // Simulate one step
        backend.step(dt).unwrap();

        let rigidbody = backend.get_rigidbody(index).unwrap();

        // Object should have fallen due to gravity
        assert!(rigidbody.position.y < 10.0);
        assert!(rigidbody.velocity.y < 0.0);
    }

    #[test]
    fn test_parallel_processing_large_scale() {
        let mut backend = CpuBackend::new();
        backend.initialize(1000).unwrap();

        // Add many rigid bodies for parallel processing test
        for i in 0..1000 {
            let rigidbody = RigidBody {
                position: Vec3::new(i as f32 * 0.1, 10.0, 0.0),
                velocity: Vec3::ZERO,
                mass: 1.0,
                force_accumulator: ForceAccumulator::new(),
            };
            backend.add_rigidbody(rigidbody);
        }

        let dt = 0.01;
        let start_time = std::time::Instant::now();

        // Run multiple steps to test parallel performance
        for _ in 0..10 {
            backend.step(dt).unwrap();
        }

        let elapsed = start_time.elapsed();
        println!("Parallel processing of 1000 bodies took: {:?}", elapsed);

        // Verify all bodies have moved due to gravity
        for i in 0..1000 {
            let rigidbody = backend.get_rigidbody(i).unwrap();
            assert!(rigidbody.position.y < 10.0, "Body {} did not fall", i);
            assert!(
                rigidbody.velocity.y < 0.0,
                "Body {} has no downward velocity",
                i
            );
        }

        // Check performance stats
        let stats = backend.stats();
        assert_eq!(stats.backend_name, "CPU");
        assert!(stats.total_steps > 0);
    }

    #[test]
    fn test_energy_conservation_in_parallel() {
        let mut backend = CpuBackend::new();
        backend.initialize(100).unwrap();
        backend.set_gravity(Vec3::ZERO); // No gravity for energy conservation test

        let mut total_initial_energy = 0.0;

        // Add bodies with different initial velocities
        for i in 0..100 {
            let rigidbody = RigidBody {
                position: Vec3::new(i as f32, 0.0, 0.0),
                velocity: Vec3::new(0.0, (i % 10) as f32, 0.0),
                mass: 1.0 + i as f32 * 0.1,
                force_accumulator: ForceAccumulator::new(),
            };

            // Calculate initial kinetic energy
            total_initial_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
            backend.add_rigidbody(rigidbody);
        }

        let dt = 0.01;

        // Run simulation for several steps
        for _ in 0..50 {
            backend.step(dt).unwrap();
        }

        // Calculate final total energy
        let mut total_final_energy = 0.0;
        for i in 0..100 {
            let rigidbody = backend.get_rigidbody(i).unwrap();
            total_final_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
        }

        // Energy should be conserved (within numerical tolerance)
        let energy_diff = (total_final_energy - total_initial_energy).abs();
        let tolerance = total_initial_energy * 0.01; // 1% tolerance
        assert!(
            energy_diff < tolerance,
            "Energy not conserved: initial={}, final={}, diff={}",
            total_initial_energy,
            total_final_energy,
            energy_diff
        );
    }

    #[test]
    fn test_cpu_backend_performance_scaling() {
        use std::time::Instant;

        let sizes = vec![10, 100, 500];
        let mut performance_results = Vec::new();

        for size in sizes {
            let mut backend = CpuBackend::new();
            backend.initialize(size).unwrap();

            // Add rigid bodies
            for i in 0..size {
                let rigidbody = RigidBody {
                    position: Vec3::new(i as f32 * 0.1, 10.0, 0.0),
                    velocity: Vec3::ZERO,
                    mass: 1.0,
                    force_accumulator: ForceAccumulator::new(),
                };
                backend.add_rigidbody(rigidbody);
            }

            let dt = 0.01;
            let start_time = Instant::now();

            // Run 10 simulation steps
            for _ in 0..10 {
                backend.step(dt).unwrap();
            }

            let elapsed = start_time.elapsed();
            let time_per_body = elapsed.as_nanos() as f64 / (size as f64 * 10.0);
            performance_results.push((size, time_per_body));

            println!(
                "Size: {}, Time per body per step: {:.2} ns",
                size, time_per_body
            );
        }

        // Check that performance scales reasonably (not exponentially bad)
        assert!(performance_results.len() == 3);

        // For parallel processing, time per body should not increase drastically
        let small_perf = performance_results[0].1;
        let large_perf = performance_results[2].1;
        let scaling_factor = large_perf / small_perf;

        // Parallel processing should keep scaling reasonable (< 5x for 50x more bodies)
        assert!(
            scaling_factor < 5.0,
            "Poor scaling: {:.2}x slower per body with 50x more bodies",
            scaling_factor
        );
    }
}
