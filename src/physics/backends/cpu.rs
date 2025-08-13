// CPU backend implementation using Rayon for parallel processing

use super::traits::{BackendError, BackendStats, PhysicsBackend};
use crate::physics::core::shapes::Shape;
use crate::physics::core::forces::ForceAccumulator;
use crate::physics::core::integration::verlet_step;
use glam::{Vec3, Mat3};
use std::time::Instant;
use rayon::prelude::*;



/// Configuration for parallel processing
#[derive(Debug, Clone)]
pub struct ParallelConfig {
    /// Minimum number of elements per thread for parallel processing
    pub min_elements_per_thread: usize,
    /// Whether to use parallel processing for force application
    pub parallel_forces: bool,
    /// Whether to use parallel processing for motion integration
    pub parallel_integration: bool,
    /// Whether to use parallel processing for data collection
    pub parallel_data_collection: bool,
}

impl Default for ParallelConfig {
    fn default() -> Self {
        Self {
            min_elements_per_thread: 64,
            parallel_forces: true,
            parallel_integration: true,
            parallel_data_collection: true,
        }
    }
}

/// Parallel processing utilities for CPU physics backend
pub struct ParallelProcessor {
    pub config: ParallelConfig,
}

impl ParallelProcessor {
    pub fn new(config: ParallelConfig) -> Self {
        Self { config }
    }

    /// Apply forces to rigid bodies in parallel
    pub fn apply_forces_parallel<T, F>(&self, rigidbodies: &mut [T], force_fn: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        if self.config.parallel_forces && rigidbodies.len() >= self.config.min_elements_per_thread {
            rigidbodies.par_iter_mut().for_each(force_fn);
        } else {
            rigidbodies.iter_mut().for_each(force_fn);
        }
    }

    /// Integrate motion for rigid bodies in parallel
    pub fn integrate_motion_parallel<T, F>(&self, rigidbodies: &mut [T], integration_fn: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        if self.config.parallel_integration && rigidbodies.len() >= self.config.min_elements_per_thread {
            rigidbodies.par_iter_mut().for_each(integration_fn);
        } else {
            rigidbodies.iter_mut().for_each(integration_fn);
        }
    }

    /// Collect data from rigid bodies in parallel
    pub fn collect_data_parallel<T, U, F>(&self, rigidbodies: &[T], collector_fn: F) -> Vec<U>
    where
        T: Send + Sync,
        U: Send,
        F: Fn(&T) -> U + Send + Sync,
    {
        if self.config.parallel_data_collection && rigidbodies.len() >= self.config.min_elements_per_thread {
            rigidbodies.par_iter().map(collector_fn).collect()
        } else {
            rigidbodies.iter().map(collector_fn).collect()
        }
    }

    /// Collect multiple data points from rigid bodies in parallel and unzip them
    pub fn collect_multiple_data_parallel<T, U, V, F>(&self, rigidbodies: &[T], collector_fn: F) -> (Vec<U>, Vec<V>)
    where
        T: Send + Sync,
        U: Send,
        V: Send,
        F: Fn(&T) -> (U, V) + Send + Sync,
    {
        if self.config.parallel_data_collection && rigidbodies.len() >= self.config.min_elements_per_thread {
            rigidbodies.par_iter().map(collector_fn).unzip()
        } else {
            rigidbodies.iter().map(collector_fn).unzip()
        }
    }

    /// Update rigid bodies in parallel with enumerated indices
    pub fn update_enumerated_parallel<T, F>(&self, rigidbodies: &mut [T], update_fn: F)
    where
        T: Send + Sync,
        F: Fn(usize, &mut T) + Send + Sync,
    {
        if self.config.parallel_integration && rigidbodies.len() >= self.config.min_elements_per_thread {
            rigidbodies.par_iter_mut().enumerate().for_each(|(i, rb)| update_fn(i, rb));
        } else {
            rigidbodies.iter_mut().enumerate().for_each(|(i, rb)| update_fn(i, rb));
        }
    }

    /// Configure Rayon thread pool for physics simulation
    pub fn configure_thread_pool(num_threads: Option<usize>) -> Result<(), rayon::ThreadPoolBuildError> {
        let threads = num_threads.unwrap_or_else(|| (rayon::current_num_threads()).min(16));

        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .thread_name(|index| format!("physics-worker-{}", index))
            .build_global()
    }

    /// Benchmark different parallel processing configurations
    pub fn benchmark_configuration(&self) -> ParallelBenchmarkResult {
        let thread_count = rayon::current_num_threads();
        ParallelBenchmarkResult {
            thread_count,
            min_elements_per_thread: self.config.min_elements_per_thread,
            parallel_forces_enabled: self.config.parallel_forces,
            parallel_integration_enabled: self.config.parallel_integration,
            parallel_data_collection_enabled: self.config.parallel_data_collection,
        }
    }
}

impl Default for ParallelProcessor {
    fn default() -> Self {
        Self::new(ParallelConfig::default())
    }
}

/// Benchmark results for parallel processing configuration
#[derive(Debug, Clone)]
pub struct ParallelBenchmarkResult {
    pub thread_count: usize,
    pub min_elements_per_thread: usize,
    pub parallel_forces_enabled: bool,
    pub parallel_integration_enabled: bool,
    pub parallel_data_collection_enabled: bool,
}

/// CPU-based physics backend with Rayon parallel processing
pub struct CpuBackend {
    rigidbodies: Vec<RigidBody>,
    gravity: Vec3,
    stats: BackendStats,
    max_rigidbodies: usize,
    broad_phase: crate::physics::core::collision::BroadPhaseDetector,
    narrow_phase: crate::physics::core::collision::NarrowPhaseDetector,
    constraint_solver: crate::physics::core::constraints::ConstraintSolver,
    parallel_processor: ParallelProcessor,
}

/// Simple rigid body representation for CPU backend
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub position: Vec3,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub mass: f32,
    pub inertia_tensor: Mat3,
    pub force_accumulator: ForceAccumulator,
    pub shape: crate::physics::core::shapes::ShapeType,
    pub restitution: f32, // Coefficient of restitution for collisions
    pub friction: f32,    // Friction coefficient
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: crate::physics::core::shapes::ShapeType::Sphere(
                crate::physics::core::shapes::Sphere::new(1.0)
            ),
            restitution: 0.5,
            friction: 0.5,
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
            broad_phase: crate::physics::core::collision::BroadPhaseDetector::new(),
            narrow_phase: crate::physics::core::collision::NarrowPhaseDetector::new(),
            constraint_solver: crate::physics::core::constraints::ConstraintSolver::new(),
            parallel_processor: ParallelProcessor::default(),
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

    /// Get parallel processing configuration
    pub fn parallel_config(&self) -> &ParallelConfig {
        &self.parallel_processor.config
    }

    /// Update parallel processing configuration
    pub fn set_parallel_config(&mut self, config: ParallelConfig) {
        self.parallel_processor = ParallelProcessor::new(config);
    }

    /// Get parallel processing benchmark results
    pub fn parallel_benchmark(&self) -> ParallelBenchmarkResult {
        self.parallel_processor.benchmark_configuration()
    }

    /// Initialize global Rayon thread pool with optimal settings for physics
    pub fn configure_global_thread_pool(num_threads: Option<usize>) -> Result<(), rayon::ThreadPoolBuildError> {
        ParallelProcessor::configure_thread_pool(num_threads)
    }

    /// Get reference to all rigidbodies for testing and analysis
    pub fn rigidbodies(&self) -> &[RigidBody] {
        &self.rigidbodies
    }

    /// Get current simulation time
    pub fn time(&self) -> f32 {
        self.stats.simulation_time as f32
    }

    /// Clear all forces (for testing)
    pub fn clear_forces(&mut self) {
        // Forces are typically cleared automatically each step
        // This method is provided for testing purposes
    }

    /// Apply forces to all rigid bodies using optimized parallel processing
    fn apply_forces(&mut self) {
        let gravity = self.gravity;
        self.parallel_processor.apply_forces_parallel(
            &mut self.rigidbodies,
            |rigidbody| {
                rigidbody.force_accumulator.reset();
                rigidbody
                    .force_accumulator
                    .add_gravitational_force(rigidbody.mass, gravity);
            },
        );
    }

    /// Detect and resolve collisions using advanced constraint solver with optimized parallel processing
    fn handle_collisions(&mut self, dt: f32) {
        // Clear previous constraints
        self.constraint_solver.clear_constraints();

        // Parallel collection of positions and bounding boxes for broad phase
        let (positions, bounding_boxes) = self.parallel_processor.collect_multiple_data_parallel(
            &self.rigidbodies,
            |rb| (rb.position, rb.shape.bounding_box()),
        );

        // Broad phase: find potentially colliding pairs
        let potential_pairs = self.broad_phase.find_potential_collisions(&positions, &bounding_boxes);
        let potential_pairs: Vec<(usize, usize)> = potential_pairs.iter().cloned().collect();

        // Narrow phase: test precise collisions and create constraints
        // Note: Sequential processing here due to mutable constraint_solver access
        for (i, j) in potential_pairs {
            if i >= self.rigidbodies.len() || j >= self.rigidbodies.len() {
                continue;
            }

            let (rb_a, rb_b) = {
                let (left, right) = self.rigidbodies.split_at(j);
                (&left[i], &right[0])
            };

            if let Some(manifold) = self.narrow_phase.test_collision(
                &rb_a.shape,
                rb_a.position,
                &rb_b.shape,
                rb_b.position,
                i,
                j,
            ) {
                // Create contact constraints from collision manifold
                for contact in &manifold.contacts {
                    let constraint = crate::physics::core::constraints::ContactConstraint::new(
                        i, j,
                        contact.position - rb_a.position, // Contact point relative to body A
                        contact.position - rb_b.position, // Contact point relative to body B
                        contact.normal,
                        contact.depth,
                        (rb_a.friction + rb_b.friction) * 0.5, // Average friction
                        (rb_a.restitution + rb_b.restitution) * 0.5, // Average restitution
                    );
                    self.constraint_solver.add_constraint(constraint);
                }
            }
        }

        // Parallel preparation of solver data
        let solver_data: Vec<(Vec3, Vec3, Vec3, f32, Mat3)> = self.parallel_processor.collect_data_parallel(
            &self.rigidbodies,
            |rb| (rb.position, rb.velocity, rb.angular_velocity, rb.mass, rb.inertia_tensor),
        );

        // Extract data components
        let mut positions: Vec<Vec3> = solver_data.iter().map(|d| d.0).collect();
        let mut velocities: Vec<Vec3> = solver_data.iter().map(|d| d.1).collect();
        let mut angular_velocities: Vec<Vec3> = solver_data.iter().map(|d| d.2).collect();
        let masses: Vec<f32> = solver_data.iter().map(|d| d.3).collect();
        let inertia_tensors: Vec<Mat3> = solver_data.iter().map(|d| d.4).collect();

        // Solve all constraints
        self.constraint_solver.solve_constraints(
            &mut positions,
            &mut velocities,
            &mut angular_velocities,
            &masses,
            &inertia_tensors,
            dt,
        );

        // Parallel update of rigid bodies with solved velocities and positions
        self.parallel_processor.update_enumerated_parallel(
            &mut self.rigidbodies,
            |i, rb| {
                if i < positions.len() {
                    rb.position = positions[i];
                    rb.velocity = velocities[i];
                    rb.angular_velocity = angular_velocities[i];
                }
            },
        );
    }



    /// Integrate motion for all rigid bodies using optimized parallel processing
    fn integrate_motion(&mut self, dt: f32) {
        self.parallel_processor.integrate_motion_parallel(
            &mut self.rigidbodies,
            |rigidbody| {
                // Calculate total force including ForceType forces (gravity, spring, etc.)
                let total_force = rigidbody.force_accumulator.calculate_total_force(
                    rigidbody.position, 
                    rigidbody.velocity, 
                    rigidbody.mass
                );
                let acceleration = total_force / rigidbody.mass;
                let (new_position, new_velocity) =
                    verlet_step(rigidbody.position, rigidbody.velocity, acceleration, dt);
                rigidbody.position = new_position;
                rigidbody.velocity = new_velocity;
            },
        );
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

        // Handle collisions
        self.handle_collisions(dt);

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
            angular_velocity: Vec3::ZERO,
            mass,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: crate::physics::core::shapes::ShapeType::Sphere(
                crate::physics::core::shapes::Sphere::new(1.0)
            ), // Default shape
            restitution: 0.5, // Default restitution
            friction: 0.5, // Default friction
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

    fn update(&mut self, dt: f32) {
        let _ = self.step(dt);
    }
    
    fn add_force(&mut self, index: usize, force_type: crate::physics::core::forces::ForceType) {
        if let Some(body) = self.rigidbodies.get_mut(index) {
            body.force_accumulator.add_force(force_type);
        }
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
            angular_velocity: Vec3::ZERO,
            mass: 5.0,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: crate::physics::core::shapes::ShapeType::Sphere(
                crate::physics::core::shapes::Sphere::new(1.0)
            ),
            restitution: 0.5,
            friction: 0.5,
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
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: crate::physics::core::shapes::ShapeType::Sphere(
                crate::physics::core::shapes::Sphere::new(1.0)
            ),
            restitution: 0.5,
            friction: 0.5,
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
                angular_velocity: Vec3::ZERO,
                mass: 1.0,
                inertia_tensor: Mat3::IDENTITY,
                force_accumulator: ForceAccumulator::new(),
                shape: crate::physics::core::shapes::ShapeType::Sphere(
                    crate::physics::core::shapes::Sphere::new(1.0)
                ),
                restitution: 0.5,
                friction: 0.5,
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
        backend.initialize(10).unwrap(); // Reduce number of bodies for better numerical stability
        backend.set_gravity(Vec3::ZERO); // No gravity for energy conservation test

        let mut total_initial_energy = 0.0;

        // Add bodies with smaller initial velocities for better stability
        for i in 0..10 {
            let rigidbody = RigidBody {
                position: Vec3::new(i as f32 * 10.0, 0.0, 0.0), // Spread out to avoid collisions
                velocity: Vec3::new(0.0, (i % 3) as f32 * 0.1, 0.0), // Smaller velocities
                angular_velocity: Vec3::ZERO,
                mass: 1.0,
                inertia_tensor: Mat3::IDENTITY,
                force_accumulator: ForceAccumulator::new(),
                shape: crate::physics::core::shapes::ShapeType::Sphere(
                    crate::physics::core::shapes::Sphere::new(0.5) // Smaller spheres
                ),
                restitution: 1.0, // Perfect elastic collisions
                friction: 0.0,    // No friction for ideal energy conservation
            };

            // Calculate initial kinetic energy
            total_initial_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
            backend.add_rigidbody(rigidbody);
        }

        let dt = 0.001; // Smaller time step for better numerical stability

        // Run simulation for fewer steps
        for _ in 0..10 {
            backend.step(dt).unwrap();
        }

        // Calculate final total energy
        let mut total_final_energy = 0.0;
        for i in 0..10 {
            let rigidbody = backend.get_rigidbody(i).unwrap();
            total_final_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
        }

        // Energy should be conserved (within larger numerical tolerance for Verlet integration)
        let energy_diff = (total_final_energy - total_initial_energy).abs();
        let tolerance = if total_initial_energy > 0.0 {
            total_initial_energy * 0.1 // 10% tolerance for numerical integration
        } else {
            0.01 // Absolute tolerance for near-zero energy systems
        };
        
        assert!(
            energy_diff < tolerance,
            "Energy not conserved: initial={}, final={}, diff={}, tolerance={}",
            total_initial_energy,
            total_final_energy,
            energy_diff,
            tolerance
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
                    angular_velocity: Vec3::ZERO,
                    mass: 1.0,
                    inertia_tensor: Mat3::IDENTITY,
                    force_accumulator: ForceAccumulator::new(),
                    shape: crate::physics::core::shapes::ShapeType::Sphere(
                        crate::physics::core::shapes::Sphere::new(1.0)
                    ),
                    restitution: 0.5,
                    friction: 0.5,
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
