//! GPU backend integration tests

use noxy_physics::backends::factory::BackendFactory;
use noxy_physics::backends::traits::BackendSelection;

#[test]
fn test_gpu_physics_pipeline_integration() {
    let backend = BackendFactory::create_backend(BackendSelection::Gpu);
    if backend.is_err() {
        println!("GPU backend not available, skipping test");
        return;
    }
    let mut backend = backend.unwrap();
    
    if backend.initialize(100).is_ok() {
        // Create a small physics scene
        let mut body_ids = Vec::new();
        
        // Add rigid bodies in a grid
        for x in 0..5 {
            for z in 0..5 {
                let body_id = backend.add_rigidbody(
                    glam::Vec3::new(x as f32 * 2.0, 10.0, z as f32 * 2.0),
                    glam::Vec3::new(0.0, 0.0, 0.0),
                    1.0
                );
                assert!(body_id.is_ok());
                body_ids.push(body_id.unwrap());
            }
        }
        
        assert_eq!(backend.rigidbody_count(), 25);
        
        // Set gravity and run simulation
        backend.set_gravity(glam::Vec3::new(0.0, -9.81, 0.0));
        
        // Record initial state
        let initial_positions: Vec<_> = body_ids.iter()
            .map(|&id| backend.get_position(id).unwrap())
            .collect();
        
        // Run simulation for 1 second (60 steps at 16ms each)
        for _ in 0..60 {
            assert!(backend.step(0.016).is_ok());
        }
        
        // Verify that all bodies have fallen due to gravity
        for (i, &body_id) in body_ids.iter().enumerate() {
            let final_pos = backend.get_position(body_id).unwrap();
            let initial_pos = initial_positions[i];
            
            // Bodies should have fallen (Y position should be lower)
            assert!(final_pos.y < initial_pos.y, 
                "Body {} did not fall: initial={}, final={}", 
                body_id, initial_pos.y, final_pos.y);
            
            // X and Z positions should remain roughly the same
            assert!((final_pos.x - initial_pos.x).abs() < 0.1);
            assert!((final_pos.z - initial_pos.z).abs() < 0.1);
        }
        
        println!("GPU physics pipeline integration test passed");
    } else {
        println!("Skipping GPU integration test - GPU not available");
    }
}

#[test]
fn test_gpu_performance_scaling() {
    let mut backend = BackendFactory::create(BackendSelection::Gpu);
    
    if backend.initialize(1000).is_ok() {
        use std::time::Instant;
        
        // Test with different numbers of rigid bodies
        let test_counts = vec![10, 50, 100, 500];
        
        for count in test_counts {
            // Clear previous bodies (in real implementation)
            let mut backend = BackendFactory::create(BackendSelection::Gpu);
            assert!(backend.initialize(count).is_ok());
            
            // Add rigid bodies
            for i in 0..count {
                let body_id = backend.add_rigidbody(
                    glam::Vec3::new(
                        (i % 10) as f32,
                        (i / 10) as f32,
                        0.0
                    ),
                    glam::Vec3::new(0.0, 0.0, 0.0),
                    1.0
                );
                assert!(body_id.is_ok());
            }
            
            backend.set_gravity(glam::Vec3::new(0.0, -9.81, 0.0));
            
            // Measure time for 10 simulation steps
            let start = Instant::now();
            for _ in 0..10 {
                assert!(backend.step(0.016).is_ok());
            }
            let duration = start.elapsed();
            
            println!("GPU performance with {} bodies: {:?}", count, duration);
            
            // Basic sanity check - should complete within reasonable time
            assert!(duration.as_millis() < 1000, 
                "GPU simulation too slow: {} bodies took {:?}", count, duration);
        }
    } else {
        println!("Skipping GPU performance test - GPU not available");
    }
}

#[test]
fn test_gpu_memory_limits() {
    let mut backend = BackendFactory::create(BackendSelection::Gpu);
    
    if backend.initialize(10000).is_ok() {
        // Test adding many rigid bodies to check memory management
        let mut successful_bodies = 0;
        
        for i in 0..1000 {
            let result = backend.add_rigidbody(
                glam::Vec3::new(i as f32 % 100.0, 0.0, 0.0),
                glam::Vec3::new(0.0, 0.0, 0.0),
                1.0
            );
            
            if result.is_ok() {
                successful_bodies += 1;
            } else {
                // Memory limit reached, which is acceptable
                break;
            }
        }
        
        assert!(successful_bodies >= 100, 
            "GPU should handle at least 100 rigid bodies, got {}", successful_bodies);
        
        println!("GPU memory test: successfully created {} bodies", successful_bodies);
        
        // Test that simulation still works with many bodies
        backend.set_gravity(glam::Vec3::new(0.0, -9.81, 0.0));
        assert!(backend.step(0.016).is_ok());
        
    } else {
        println!("Skipping GPU memory test - GPU not available");
    }
}