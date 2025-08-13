// Tests for the main physics API that users will interact with

use glam::Vec3;
use noxy::physics::backends::traits::BackendSelection;
use noxy::physics::simulation::{PhysicsSimulation, RigidBodyProperties, SimulationConfig};

#[test]
fn test_main_physics_api_basic() {
    // Test basic API creation and usage
    let mut simulation = PhysicsSimulation::new().unwrap();

    // Test basic operations
    assert!(simulation.step(0.01).is_ok());
    assert_eq!(simulation.rigidbody_count(), 0);

    let stats = simulation.statistics();
    assert_eq!(stats.backend_name, "CPU");
    assert_eq!(stats.rigidbody_count, 0);
    assert!(!stats.used_fallback);
}

#[test]
fn test_physics_api_with_config() {
    let config = SimulationConfig {
        backend_selection: BackendSelection::Cpu,
        max_rigidbodies: 100,
        gravity: Vec3::new(0.0, -9.81, 0.0),
    };

    let simulation = PhysicsSimulation::with_config(config).unwrap();
    assert_eq!(simulation.statistics().backend_name, "CPU");
}

#[test]
fn test_physics_simulation_lifecycle() {
    // Test the full lifecycle: create, add objects, simulate, query results
    let mut simulation = PhysicsSimulation::new().unwrap();

    // Test rigidbody properties validation
    let invalid_mass = RigidBodyProperties {
        mass: -1.0,
        ..Default::default()
    };
    assert!(simulation.add_rigidbody(invalid_mass).is_err());

    // Test simulation stepping
    assert!(simulation.step(0.01).is_ok());
    assert!(simulation.step(-0.01).is_err()); // Invalid time step

    // Check initial state
    assert_eq!(simulation.rigidbody_count(), 0);
}

#[test]
fn test_physics_simulation_error_handling() {
    // Test proper error handling in the main API
    let mut simulation = PhysicsSimulation::new().unwrap();

    // Test invalid time steps
    assert!(simulation.step(0.0).is_err());
    assert!(simulation.step(f32::NAN).is_err());
    assert!(simulation.step(f32::INFINITY).is_err());

    // Test invalid rigid body properties
    let invalid_position = RigidBodyProperties {
        position: Vec3::new(f32::NAN, 0.0, 0.0),
        ..Default::default()
    };
    assert!(simulation.add_rigidbody(invalid_position).is_err());

    let invalid_velocity = RigidBodyProperties {
        velocity: Vec3::new(0.0, f32::INFINITY, 0.0),
        ..Default::default()
    };
    assert!(simulation.add_rigidbody(invalid_velocity).is_err());
}

#[test]
fn test_physics_simulation_performance() {
    // Test that the main API provides good performance with backend abstraction
    let mut simulation = PhysicsSimulation::new().unwrap();

    let start_time = std::time::Instant::now();

    // Run multiple simulation steps
    for _ in 0..100 {
        simulation.step(0.01).unwrap();
    }

    let elapsed = start_time.elapsed();
    println!("100 simulation steps took: {:?}", elapsed);

    // Should complete in reasonable time (< 1 second for 100 empty steps)
    assert!(elapsed.as_secs() < 1);

    let stats = simulation.statistics();
    assert_eq!(stats.backend_name, "CPU");
}

#[test]
fn test_physics_simulation_backend_switching() {
    // Test that the main API can switch backends transparently

    // Test auto selection (should pick CPU)
    let auto_sim = PhysicsSimulation::new().unwrap();
    assert_eq!(auto_sim.statistics().backend_name, "CPU");

    // Test explicit CPU selection
    let cpu_config = SimulationConfig {
        backend_selection: BackendSelection::Cpu,
        ..Default::default()
    };
    let cpu_sim = PhysicsSimulation::with_config(cpu_config).unwrap();
    assert_eq!(cpu_sim.statistics().backend_name, "CPU");

    // Test GPU selection (should fail since not implemented)
    let gpu_config = SimulationConfig {
        backend_selection: BackendSelection::Gpu,
        ..Default::default()
    };
    assert!(PhysicsSimulation::with_config(gpu_config).is_err());
}
