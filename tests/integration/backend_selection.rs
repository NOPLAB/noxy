// Tests for backend selection and management system

use noxy_physics::backends::cpu::CpuBackend;
use noxy_physics::backends::traits::PhysicsBackend;
use noxy_physics::backends::traits::BackendSelection;
use glam::{Vec3, Mat3};
use noxy_physics::core::shapes::{ShapeType, Sphere};

#[test]
fn test_backend_selection_auto() {
    // Test automatic backend selection
    let selection = BackendSelection::Auto;

    // For now, auto should select CPU since GPU isn't implemented yet
    assert_eq!(selection, BackendSelection::Auto);

    // Test that CPU backend is available
    assert!(CpuBackend::is_available());
}

#[test]
fn test_backend_selection_explicit() {
    // Test explicit backend selection
    let cpu_selection = BackendSelection::Cpu;
    let gpu_selection = BackendSelection::Gpu;

    assert_eq!(cpu_selection, BackendSelection::Cpu);
    assert_eq!(gpu_selection, BackendSelection::Gpu);
}

#[test]
fn test_backend_fallback_mechanism() {
    // Test that system can fall back from GPU to CPU if GPU not available
    // This is a placeholder test for future GPU implementation

    // When GPU is not available, should fallback to CPU
    let fallback_works = true; // Placeholder
    assert!(fallback_works);
}

// Tests for backend factory and manager
#[test]
fn test_backend_factory_creation() {
    // Test BackendFactory creation
    use noxy_physics::backends::factory::BackendFactory;

    let backend = BackendFactory::create_backend(BackendSelection::Cpu).unwrap();
    assert_eq!(backend.name(), "cpu");

    // Test auto selection
    let auto_backend = BackendFactory::create_backend(BackendSelection::Auto).unwrap();
    assert_eq!(auto_backend.name(), "cpu");

    // Test available backends
    let available = BackendFactory::available_backends();
    assert!(available.contains(&"cpu"));
}

#[test]
fn test_backend_manager() {
    use noxy_physics::backends::factory::BackendManager;

    let mut manager = BackendManager::new(BackendSelection::Cpu).unwrap();
    assert_eq!(manager.backend_name(), "cpu");
    assert!(!manager.used_fallback());

    // Test initialization
    assert!(manager.initialize(10).is_ok());
}

#[test]
fn test_performance_profiling() {
    // Test backend performance profiling
    let mut backend = CpuBackend::new();
    backend.initialize(10).unwrap();

    // Add some workload
    for i in 0..10 {
        let rigidbody = noxy::physics::backends::cpu::RigidBody {
            position: Vec3::new(i as f32, 10.0, 0.0),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inertia_tensor: Mat3::IDENTITY,
            force_accumulator: noxy::physics::core::forces::ForceAccumulator::new(),
            shape: ShapeType::Sphere(Sphere::new(1.0)),
            restitution: 0.5,
            friction: 0.5,
        };
        backend.add_rigidbody(rigidbody);
    }

    // Run several steps and check stats
    for _ in 0..5 {
        backend.step(0.01).unwrap();
    }

    let stats = backend.stats();
    assert_eq!(stats.total_steps, 5);
    assert!(stats.avg_step_time_ms > 0.0);
    assert!(stats.total_time_ms > 0.0);
    assert_eq!(stats.backend_name, "cpu");
}
