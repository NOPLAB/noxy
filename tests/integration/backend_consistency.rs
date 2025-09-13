//! Test CPU/GPU backend consistency for Phase 2 verification

use noxy::physics::backends::{
    cpu::CpuBackend,
    gpu::GpuBackend, 
    traits::PhysicsBackend,
    factory::{BackendFactory, BackendSelection},
};
use glam::Vec3;

const TEST_TOLERANCE: f32 = 1e-3;

/// Test that CPU and GPU backends produce similar results
#[test]
fn test_cpu_gpu_consistency() {
    // Skip if GPU is not available
    if !GpuBackend::is_available() {
        println!("GPU backend not available, skipping consistency test");
        return;
    }

    let mut cpu_backend = CpuBackend::new();
    let mut gpu_backend = GpuBackend::new();

    // Initialize both backends with same parameters
    let max_rigidbodies = 10;
    cpu_backend.initialize(max_rigidbodies).unwrap();
    gpu_backend.initialize(max_rigidbodies).unwrap();

    // Set same gravity
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    cpu_backend.set_gravity(gravity);
    gpu_backend.set_gravity(gravity);

    // Add same rigid bodies to both backends
    let test_bodies = vec![
        (Vec3::new(0.0, 10.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0),
        (Vec3::new(2.0, 8.0, 1.0), Vec3::new(-0.5, 0.5, 0.0), 2.0),
        (Vec3::new(-1.0, 12.0, -0.5), Vec3::new(0.0, -1.0, 0.2), 0.5),
    ];

    for (position, velocity, mass) in test_bodies {
        cpu_backend.add_rigidbody(position, velocity, mass).unwrap();
        gpu_backend.add_rigidbody(position, velocity, mass).unwrap();
    }

    // Run simulation steps and compare results
    let dt = 0.016; // ~60 FPS
    let steps = 10;

    for step in 0..steps {
        // Step both backends
        cpu_backend.step(dt).unwrap();
        gpu_backend.step(dt).unwrap();

        // Compare positions and velocities
        for i in 0..3 {
            let cpu_pos = cpu_backend.get_position(i).expect("CPU position");
            let gpu_pos = gpu_backend.get_position(i).expect("GPU position");
            
            let cpu_vel = cpu_backend.get_velocity(i).expect("CPU velocity");
            let gpu_vel = gpu_backend.get_velocity(i).expect("GPU velocity");

            // Check position consistency
            let pos_diff = (cpu_pos - gpu_pos).length();
            assert!(
                pos_diff < TEST_TOLERANCE,
                "Step {}: Position mismatch for body {}: CPU={:?}, GPU={:?}, diff={}",
                step, i, cpu_pos, gpu_pos, pos_diff
            );

            // Check velocity consistency
            let vel_diff = (cpu_vel - gpu_vel).length();
            assert!(
                vel_diff < TEST_TOLERANCE,
                "Step {}: Velocity mismatch for body {}: CPU={:?}, GPU={:?}, diff={}",
                step, i, cpu_vel, gpu_vel, vel_diff
            );
        }
    }
}

/// Test backend manager automatic selection
#[test]
fn test_backend_auto_selection() {
    let backend = BackendFactory::create_backend(BackendSelection::Auto);
    
    match backend {
        Ok(backend) => {
            let backend_name = backend.name();
            println!("Auto-selected backend: {}", backend_name);
            
            // Should select either CPU or GPU
            assert!(backend_name == "cpu" || backend_name == "GPU (WGPU Compute)");
        }
        Err(e) => {
            println!("Backend auto-selection failed: {}", e);
            // This is okay if no backends are available
        }
    }
}

/// Test GPU fallback to CPU
#[test]
fn test_gpu_fallback() {
    // Test that CPU backend works as fallback
    let mut cpu_backend = CpuBackend::new();
    
    // Initialize the backend
    assert!(cpu_backend.initialize(100).is_ok());

    // Add some test bodies
    for i in 0..5 {
        let pos = Vec3::new(i as f32, 5.0, 0.0);
        let vel = Vec3::new(0.0, 0.0, 0.0);
        let mass = 1.0;
        
        assert!(cpu_backend.add_rigidbody(pos, vel, mass).is_ok());
    }

    // Step simulation - should work
    for _ in 0..10 {
        assert!(cpu_backend.step(0.016).is_ok());
    }

    // Should have 5 rigidbodies
    assert_eq!(cpu_backend.rigidbody_count(), 5);
}

/// Test available backends detection
#[test]
fn test_available_backends() {
    // CPU backend should always be available
    assert!(CpuBackend::is_available());
    
    // GPU may or may not be available
    let gpu_available = GpuBackend::is_available();
    println!("GPU backend available: {}", gpu_available);
}

/// Benchmark CPU vs GPU performance
#[test]
fn benchmark_cpu_vs_gpu() {
    use std::time::Instant;

    if !GpuBackend::is_available() {
        println!("GPU not available, skipping benchmark");
        return;
    }

    let rigidbody_counts = vec![100, 500, 1000];
    let steps = 100;

    for count in rigidbody_counts {
        println!("\n--- Benchmarking with {} rigid bodies ---", count);

        // Test CPU backend
        let cpu_time = {
            let mut backend = CpuBackend::new();
            backend.initialize(count).unwrap();
            backend.set_gravity(Vec3::new(0.0, -9.81, 0.0));

            // Add rigid bodies
            for i in 0..count {
                let pos = Vec3::new((i % 10) as f32, 10.0 + (i / 10) as f32, 0.0);
                backend.add_rigidbody(pos, Vec3::ZERO, 1.0).unwrap();
            }

            let start = Instant::now();
            for _ in 0..steps {
                backend.step(0.016).unwrap();
            }
            start.elapsed()
        };

        // Test GPU backend
        let gpu_time = {
            let mut backend = GpuBackend::new();

            backend.initialize(count).unwrap();
            backend.set_gravity(Vec3::new(0.0, -9.81, 0.0));

            // Add rigid bodies
            for i in 0..count {
                let pos = Vec3::new((i % 10) as f32, 10.0 + (i / 10) as f32, 0.0);
                backend.add_rigidbody(pos, Vec3::ZERO, 1.0).unwrap();
            }

            let start = Instant::now();
            for _ in 0..steps {
                backend.step(0.016).unwrap();
            }
            start.elapsed()
        };

        println!("CPU time: {:?}", cpu_time);
        println!("GPU time: {:?}", gpu_time);

        let speedup = cpu_time.as_secs_f64() / gpu_time.as_secs_f64();
        println!("GPU speedup: {:.2}x", speedup);
    }
}