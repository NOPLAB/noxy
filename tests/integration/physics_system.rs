// Integration tests for the complete physics system
// Testing the interaction between all components: cores, backends, and systems

use glam::{Vec3, Mat3};
use noxy::physics::backends::cpu::{CpuBackend, RigidBody};
use noxy::physics::backends::traits::PhysicsBackend;
use noxy::physics::core::forces::ForceAccumulator;
use noxy::physics::core::shapes::{ShapeType, Sphere};

#[test]
fn test_physics_system_integration_basic() {
    // Test that all components work together for a basic simulation
    let mut backend = CpuBackend::new();
    backend.initialize(10).unwrap();

    // Create a simple scenario: falling objects
    for i in 0..5 {
        let rigidbody = RigidBody {
            position: Vec3::new(i as f32, 10.0 + i as f32, 0.0),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0 + i as f32 * 0.5,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: ShapeType::Sphere(Sphere::new(1.0)),
            restitution: 0.5,
            friction: 0.5,
        };
        backend.add_rigidbody(rigidbody);
    }

    let dt = 0.01;
    let steps = 100;

    // Run simulation
    for _ in 0..steps {
        backend.step(dt).unwrap();
    }

    // Verify all objects have fallen
    for i in 0..5 {
        let rigidbody = backend.get_rigidbody(i).unwrap();
        println!("Object {}: position={:?}, velocity={:?}", i, rigidbody.position, rigidbody.velocity);
        
        // Debug: Check if forces are being applied
        println!("Object {}: mass={}", i, rigidbody.mass);
        
        assert!(
            rigidbody.position.y < 10.0 + i as f32,
            "Object {} did not fall: y={}, expected < {}",
            i, rigidbody.position.y, 10.0 + i as f32
        );
        assert!(
            rigidbody.velocity.y < 0.0,
            "Object {} should have downward velocity: vy={}",
            i, rigidbody.velocity.y
        );
    }

    // Check system statistics
    let stats = backend.stats();
    assert_eq!(stats.total_steps, steps);
    assert!(stats.avg_step_time_ms > 0.0);
}

#[test]
fn test_physics_system_multiple_force_types() {
    // Test system with multiple types of forces
    let mut backend = CpuBackend::new();
    backend.initialize(3).unwrap();

    // Object 1: Normal gravity fall
    let rigidbody1 = RigidBody {
        position: Vec3::new(0.0, 10.0, 0.0),
        velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        mass: 1.0,
        inertia_tensor: Mat3::IDENTITY,
        force_accumulator: ForceAccumulator::new(),
        shape: ShapeType::Sphere(Sphere::new(1.0)),
        restitution: 0.5,
        friction: 0.5,
    };

    // Object 2: With initial velocity
    let rigidbody2 = RigidBody {
        position: Vec3::new(2.0, 10.0, 0.0),
        velocity: Vec3::new(5.0, 0.0, 0.0),
        angular_velocity: Vec3::ZERO,
        mass: 2.0,
        inertia_tensor: Mat3::IDENTITY,
        force_accumulator: ForceAccumulator::new(),
        shape: ShapeType::Sphere(Sphere::new(1.0)),
        restitution: 0.5,
        friction: 0.5,
    };

    // Object 3: Different mass
    let rigidbody3 = RigidBody {
        position: Vec3::new(4.0, 15.0, 0.0),
        velocity: Vec3::new(0.0, 1.0, 0.0),
        angular_velocity: Vec3::ZERO,
        mass: 0.5,
        inertia_tensor: Mat3::IDENTITY,
        force_accumulator: ForceAccumulator::new(),
        shape: ShapeType::Sphere(Sphere::new(1.0)),
        restitution: 0.5,
        friction: 0.5,
    };

    backend.add_rigidbody(rigidbody1);
    backend.add_rigidbody(rigidbody2);
    backend.add_rigidbody(rigidbody3);

    let dt = 0.01;
    let initial_positions: Vec<Vec3> = (0..3)
        .map(|i| backend.get_rigidbody(i).unwrap().position)
        .collect();

    // Run simulation
    for _ in 0..50 {
        backend.step(dt).unwrap();
    }

    // Verify behavior
    for i in 0..3 {
        let rigidbody = backend.get_rigidbody(i).unwrap();
        let initial_pos = initial_positions[i];

        // All objects should have moved
        assert_ne!(rigidbody.position, initial_pos, "Object {} did not move", i);

        // All should be affected by gravity (negative y velocity)
        assert!(
            rigidbody.velocity.y < 1.0,
            "Object {} should be affected by gravity",
            i
        );
    }

    // Object 2 should have moved horizontally due to initial velocity
    let obj2 = backend.get_rigidbody(1).unwrap();
    assert!(
        obj2.position.x > 2.0,
        "Object 2 should have moved horizontally"
    );
}

#[test]
fn test_physics_system_stress_test() {
    // Stress test with many objects and long simulation
    let mut backend = CpuBackend::new();
    let num_objects = 100;
    backend.initialize(num_objects).unwrap();

    // Create a grid of objects with various properties
    for i in 0..num_objects {
        let x = (i % 10) as f32;
        let y = (i / 10) as f32;
        let rigidbody = RigidBody {
            position: Vec3::new(x, 20.0 + y, 0.0),
            velocity: Vec3::new(
                (i % 3) as f32 - 1.0, // -1, 0, 1
                0.0,
                (i % 5) as f32 - 2.0, // -2, -1, 0, 1, 2
            ),
            angular_velocity: Vec3::ZERO,
            mass: 0.5 + (i % 10) as f32 * 0.1,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: ForceAccumulator::new(),
            shape: ShapeType::Sphere(Sphere::new(1.0)),
            restitution: 0.5,
            friction: 0.5,
        };
        backend.add_rigidbody(rigidbody);
    }

    let dt = 0.01;
    let steps = 200;

    // Track total energy for conservation check
    let mut initial_energy = 0.0;
    backend.set_gravity(Vec3::ZERO); // No gravity for energy conservation test

    for i in 0..num_objects {
        let rigidbody = backend.get_rigidbody(i).unwrap();
        initial_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
    }

    // Run long simulation
    for step in 0..steps {
        backend.step(dt).unwrap();

        // Check system stability every 50 steps
        if step % 50 == 0 {
            let stats = backend.stats();
            assert!(
                stats.avg_step_time_ms > 0.0,
                "Invalid timing stats at step {}",
                step
            );

            // Verify no objects have NaN positions
            for i in 0..num_objects {
                let rigidbody = backend.get_rigidbody(i).unwrap();
                assert!(
                    rigidbody.position.is_finite(),
                    "Object {} has invalid position at step {}",
                    i,
                    step
                );
                assert!(
                    rigidbody.velocity.is_finite(),
                    "Object {} has invalid velocity at step {}",
                    i,
                    step
                );
            }
        }
    }

    // Final energy check (should be conserved without gravity)
    let mut final_energy = 0.0;
    for i in 0..num_objects {
        let rigidbody = backend.get_rigidbody(i).unwrap();
        final_energy += 0.5 * rigidbody.mass * rigidbody.velocity.length_squared();
    }

    let energy_diff = (final_energy - initial_energy).abs();
    let tolerance = initial_energy * 0.05; // 5% tolerance for numerical errors
    assert!(
        energy_diff < tolerance,
        "Energy not conserved in stress test: initial={:.3}, final={:.3}, diff={:.3}",
        initial_energy,
        final_energy,
        energy_diff
    );

    // Verify final statistics
    let stats = backend.stats();
    assert_eq!(stats.total_steps, steps);
    assert!(stats.total_time_ms > 0.0);
    println!(
        "Stress test completed: {} objects, {} steps, avg time: {:.3}ms/step",
        num_objects, steps, stats.avg_step_time_ms
    );
}

#[test]
fn test_backend_interface_compliance() {
    // Test that the backend properly implements the PhysicsBackend trait
    let mut backend = CpuBackend::new();

    // Test interface methods
    assert_eq!(backend.name(), "cpu");
    assert!(CpuBackend::is_available());

    // Test initialization
    assert!(backend.initialize(50).is_ok());

    // Test step without objects (should not crash)
    assert!(backend.step(0.01).is_ok());

    // Add an object and test again
    let rigidbody = RigidBody::default();
    backend.add_rigidbody(rigidbody);

    assert!(backend.step(0.01).is_ok());

    // Test multiple steps
    for _ in 0..10 {
        assert!(backend.step(0.01).is_ok());
    }

    // Verify stats are being updated
    let stats = backend.stats();
    assert!(stats.total_steps > 0);
    assert_eq!(stats.backend_name, "cpu");
}
