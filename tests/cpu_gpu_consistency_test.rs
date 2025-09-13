//! CPU/GPU consistency tests for physics simulation

use noxy::physics::backends::{

    factory::BackendFactory,
    traits::BackendSelection
};

#[test]
fn test_cpu_gpu_basic_simulation_consistency() {
    // Create both CPU and GPU backends
    let mut cpu_backend = BackendFactory::create(BackendSelection::Cpu);
    let mut gpu_backend = BackendFactory::create(BackendSelection::Gpu);
    
    // Initialize both backends
    assert!(cpu_backend.initialize(10).is_ok());
    
    // GPU might not be available, so handle gracefully
    let gpu_available = gpu_backend.initialize(10).is_ok();
    
    if !gpu_available {
        println!("GPU not available - skipping consistency test");
        return;
    }
    
    // Add same rigid body to both backends
    let position = glam::Vec3::new(0.0, 10.0, 0.0);
    let velocity = glam::Vec3::new(1.0, 0.0, 0.0);
    let mass = 2.0;
    
    let cpu_body_id = cpu_backend.add_rigidbody(position, velocity, mass);
    let gpu_body_id = gpu_backend.add_rigidbody(position, velocity, mass);
    
    assert!(cpu_body_id.is_ok());
    assert!(gpu_body_id.is_ok());
    
    // Set same gravity
    let gravity = glam::Vec3::new(0.0, -9.81, 0.0);
    cpu_backend.set_gravity(gravity);
    gpu_backend.set_gravity(gravity);
    
    // Run simulation for several steps
    let dt = 0.016; // 60 FPS
    let steps = 10;
    
    for _ in 0..steps {
        assert!(cpu_backend.step(dt).is_ok());
        assert!(gpu_backend.step(dt).is_ok());
    }
    
    // Compare final positions and velocities
    let cpu_final_pos = cpu_backend.get_position(0).unwrap();
    let gpu_final_pos = gpu_backend.get_position(0).unwrap();
    
    let cpu_final_vel = cpu_backend.get_velocity(0).unwrap();
    let gpu_final_vel = gpu_backend.get_velocity(0).unwrap();
    
    // Allow for small numerical differences
    let position_tolerance = 0.01;
    let velocity_tolerance = 0.01;
    
    let pos_diff = (cpu_final_pos - gpu_final_pos).length();
    let vel_diff = (cpu_final_vel - gpu_final_vel).length();
    
    assert!(pos_diff < position_tolerance, 
        "Position difference too large: CPU={:?}, GPU={:?}, diff={}", 
        cpu_final_pos, gpu_final_pos, pos_diff);
    
    assert!(vel_diff < velocity_tolerance,
        "Velocity difference too large: CPU={:?}, GPU={:?}, diff={}", 
        cpu_final_vel, gpu_final_vel, vel_diff);
    
    println!("Consistency test passed:");
    println!("  Final positions - CPU: {:?}, GPU: {:?}", cpu_final_pos, gpu_final_pos);
    println!("  Final velocities - CPU: {:?}, GPU: {:?}", cpu_final_vel, gpu_final_vel);
}

#[test]
fn test_cpu_gpu_multiple_bodies_consistency() {
    let mut cpu_backend = BackendFactory::create(BackendSelection::Cpu);
    let mut gpu_backend = BackendFactory::create(BackendSelection::Gpu);
    
    assert!(cpu_backend.initialize(20).is_ok());
    
    let gpu_available = gpu_backend.initialize(20).is_ok();
    if !gpu_available {
        println!("GPU not available - skipping multi-body consistency test");
        return;
    }
    
    // Add multiple rigid bodies with different parameters
    let test_bodies = vec![
        (glam::Vec3::new(0.0, 5.0, 0.0), glam::Vec3::new(1.0, 0.0, 0.0), 1.0),
        (glam::Vec3::new(2.0, 8.0, 1.0), glam::Vec3::new(-0.5, 1.0, 0.0), 1.5),
        (glam::Vec3::new(-1.0, 3.0, -2.0), glam::Vec3::new(0.0, 2.0, 1.0), 0.8),
        (glam::Vec3::new(3.0, 10.0, 2.0), glam::Vec3::new(0.5, -1.0, -0.5), 2.2),
    ];
    
    for (pos, vel, mass) in &test_bodies {
        assert!(cpu_backend.add_rigidbody(*pos, *vel, *mass).is_ok());
        assert!(gpu_backend.add_rigidbody(*pos, *vel, *mass).is_ok());
    }
    
    // Set gravity
    let gravity = glam::Vec3::new(0.0, -9.81, 0.0);
    cpu_backend.set_gravity(gravity);
    gpu_backend.set_gravity(gravity);
    
    // Run simulation
    let dt = 0.016;
    for _ in 0..20 {
        assert!(cpu_backend.step(dt).is_ok());
        assert!(gpu_backend.step(dt).is_ok());
    }
    
    // Check consistency for all bodies
    for i in 0..test_bodies.len() {
        let cpu_pos = cpu_backend.get_position(i).unwrap();
        let gpu_pos = gpu_backend.get_position(i).unwrap();
        
        let cpu_vel = cpu_backend.get_velocity(i).unwrap();
        let gpu_vel = gpu_backend.get_velocity(i).unwrap();
        
        let pos_diff = (cpu_pos - gpu_pos).length();
        let vel_diff = (cpu_vel - gpu_vel).length();
        
        assert!(pos_diff < 0.02, 
            "Body {} position inconsistent: CPU={:?}, GPU={:?}", 
            i, cpu_pos, gpu_pos);
        
        assert!(vel_diff < 0.02,
            "Body {} velocity inconsistent: CPU={:?}, GPU={:?}", 
            i, cpu_vel, gpu_vel);
    }
    
    println!("Multi-body consistency test passed for {} bodies", test_bodies.len());
}

#[test]
fn test_cpu_gpu_physics_accuracy() {
    let mut cpu_backend = BackendFactory::create(BackendSelection::Cpu);
    let mut gpu_backend = BackendFactory::create(BackendSelection::Gpu);
    
    assert!(cpu_backend.initialize(5).is_ok());
    
    let gpu_available = gpu_backend.initialize(5).is_ok();
    if !gpu_available {
        println!("GPU not available - skipping physics accuracy test");
        return;
    }
    
    // Test free fall physics accuracy
    let initial_height = 10.0;
    let position = glam::Vec3::new(0.0, initial_height, 0.0);
    let velocity = glam::Vec3::ZERO;
    let mass = 1.0;
    
    assert!(cpu_backend.add_rigidbody(position, velocity, mass).is_ok());
    assert!(gpu_backend.add_rigidbody(position, velocity, mass).is_ok());
    
    let gravity = glam::Vec3::new(0.0, -9.81, 0.0);
    cpu_backend.set_gravity(gravity);
    gpu_backend.set_gravity(gravity);
    
    // Simulate for 1 second
    let dt = 0.01; // Smaller time step for accuracy
    let total_time = 1.0;
    let steps = (total_time / dt) as u32;
    
    for _ in 0..steps {
        assert!(cpu_backend.step(dt).is_ok());
        assert!(gpu_backend.step(dt).is_ok());
    }
    
    // Analytical solution for free fall
    let expected_position = initial_height + 0.5 * gravity.y * total_time * total_time;
    let expected_velocity = gravity.y * total_time;
    
    let cpu_pos = cpu_backend.get_position(0).unwrap();
    let gpu_pos = gpu_backend.get_position(0).unwrap();
    let cpu_vel = cpu_backend.get_velocity(0).unwrap();
    let gpu_vel = gpu_backend.get_velocity(0).unwrap();
    
    // Check accuracy against analytical solution
    let cpu_pos_error = (cpu_pos.y - expected_position).abs();
    let gpu_pos_error = (gpu_pos.y - expected_position).abs();
    let cpu_vel_error = (cpu_vel.y - expected_velocity).abs();
    let gpu_vel_error = (gpu_vel.y - expected_velocity).abs();
    
    assert!(cpu_pos_error < 0.1, "CPU position error too large: {}", cpu_pos_error);
    assert!(gpu_pos_error < 0.1, "GPU position error too large: {}", gpu_pos_error);
    assert!(cpu_vel_error < 0.1, "CPU velocity error too large: {}", cpu_vel_error);
    assert!(gpu_vel_error < 0.1, "GPU velocity error too large: {}", gpu_vel_error);
    
    // Check CPU/GPU consistency
    let pos_consistency = (cpu_pos - gpu_pos).length();
    let vel_consistency = (cpu_vel - gpu_vel).length();
    
    assert!(pos_consistency < 0.01, "CPU/GPU position inconsistency: {}", pos_consistency);
    assert!(vel_consistency < 0.01, "CPU/GPU velocity inconsistency: {}", vel_consistency);
    
    println!("Physics accuracy test passed:");
    println!("  Expected: pos={:.3}, vel={:.3}", expected_position, expected_velocity);
    println!("  CPU: pos={:.3}, vel={:.3}", cpu_pos.y, cpu_vel.y);
    println!("  GPU: pos={:.3}, vel={:.3}", gpu_pos.y, gpu_vel.y);
}