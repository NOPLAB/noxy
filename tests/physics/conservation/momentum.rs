use glam::Vec3;
use noxy::physics::backends::cpu::CpuBackend;
use noxy::physics::backends::traits::PhysicsBackend;
use noxy::physics::backends::cpu::RigidBody;
use noxy::physics::core::shapes::{ShapeType, Sphere};
use noxy::physics::core::forces::ForceAccumulator;

/// Test linear momentum conservation in collision
#[test]
fn test_linear_momentum_conservation_collision() {
    let mut backend = CpuBackend::new();
    
    // Create two rigid bodies approaching each other
    let mass1 = 1.0;
    let mass2 = 2.0;
    let initial_velocity1 = Vec3::new(5.0, 0.0, 0.0);
    let initial_velocity2 = Vec3::new(-2.0, 0.0, 0.0);
    
    let body1 = RigidBody {
        position: Vec3::new(-3.0, 0.0, 0.0),
        velocity: initial_velocity1,
        angular_velocity: Vec3::ZERO,
        mass: mass1,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.8, // Partially elastic collision
        friction: 0.0,
    };
    
    let body2 = RigidBody {
        position: Vec3::new(3.0, 0.0, 0.0),
        velocity: initial_velocity2,
        angular_velocity: Vec3::ZERO,
        mass: mass2,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.8,
        friction: 0.0,
    };
    
    backend.add_rigidbody(body1);
    backend.add_rigidbody(body2);
    
    // CRITICAL: Set gravity to zero for momentum conservation test
    backend.set_gravity(Vec3::ZERO);
    
    // Calculate initial total momentum
    let initial_momentum = calculate_total_linear_momentum(&backend);
    
    // Simulate until collision and afterwards
    let dt = 0.001; // Small timestep for accuracy
    let steps = 2000; // Enough time for collision to occur and settle
    
    for _ in 0..steps {
        backend.update(dt);
    }
    
    // Calculate final total momentum
    let final_momentum = calculate_total_linear_momentum(&backend);
    
    // Momentum should be conserved (within numerical tolerance)
    let momentum_tolerance = 0.02; // 2% tolerance
    let momentum_error = (final_momentum - initial_momentum).length() / initial_momentum.length();
    
    assert!(
        momentum_error < momentum_tolerance,
        "Linear momentum conservation violated: initial={:?}, final={:?}, error={:.6}%",
        initial_momentum,
        final_momentum,
        momentum_error * 100.0
    );
}

/// Test angular momentum conservation in collision with rotation
#[test]
fn test_angular_momentum_conservation_collision() {
    let mut backend = CpuBackend::new();
    
    // Create two rigid bodies, one spinning
    let mass1 = 1.0;
    let mass2 = 1.0;
    let inertia1 = 0.4 * mass1; // Sphere inertia
    let inertia2 = 0.4 * mass2;
    
    let body1 = RigidBody {
        position: Vec3::new(-2.0, 0.0, 0.0),
        velocity: Vec3::new(3.0, 0.0, 0.0),
        angular_velocity: Vec3::new(0.0, 0.0, 10.0), // Spinning around Z-axis
        mass: mass1,
        inertia_tensor: glam::Mat3::from_diagonal(Vec3::new(inertia1, inertia1, inertia1)),
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.9,
        friction: 0.1, // Some friction for angular momentum transfer
    };
    
    let body2 = RigidBody {
        position: Vec3::new(2.0, 0.0, 0.0),
        velocity: Vec3::new(-3.0, 0.0, 0.0),
        angular_velocity: Vec3::ZERO,
        mass: mass2,
        inertia_tensor: glam::Mat3::from_diagonal(Vec3::new(inertia2, inertia2, inertia2)),
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.9,
        friction: 0.1,
    };
    
    backend.add_rigidbody(body1);
    backend.add_rigidbody(body2);
    
    // Calculate initial total angular momentum around center of mass
    let initial_angular_momentum = calculate_total_angular_momentum(&backend);
    
    // Simulate collision
    let dt = 0.001;
    let steps = 1500;
    
    for _ in 0..steps {
        backend.update(dt);
    }
    
    // Calculate final total angular momentum
    let final_angular_momentum = calculate_total_angular_momentum(&backend);
    
    // Angular momentum should be conserved
    let momentum_tolerance = 0.05; // 5% tolerance (angular momentum is more sensitive)
    let momentum_error = (final_angular_momentum - initial_angular_momentum).length() 
        / (initial_angular_momentum.length() + 1e-6); // Avoid division by zero
    
    assert!(
        momentum_error < momentum_tolerance,
        "Angular momentum conservation violated: initial={:?}, final={:?}, error={:.6}%",
        initial_angular_momentum,
        final_angular_momentum,
        momentum_error * 100.0
    );
}

/// Test momentum conservation with multiple objects
#[test]
fn test_momentum_conservation_multi_body_system() {
    let mut backend = CpuBackend::new();
    
    // Create multiple rigid bodies with random initial conditions
    let bodies = vec![
        RigidBody {
            position: Vec3::new(0.0, 0.0, 0.0),
            velocity: Vec3::new(2.0, 1.0, -0.5),
            angular_velocity: Vec3::new(1.0, 0.0, 2.0),
            mass: 1.0,
            inertia_tensor: glam::Mat3::from_diagonal(Vec3::splat(0.4)),
            shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.7,
        friction: 0.2,
        },
        RigidBody {
            position: Vec3::new(3.0, 1.0, 0.0),
            velocity: Vec3::new(-1.5, -0.5, 1.0),
            angular_velocity: Vec3::new(0.0, -1.5, 0.5),
            mass: 1.5,
            inertia_tensor: glam::Mat3::from_diagonal(Vec3::splat(0.6)),
            shape: ShapeType::Sphere(Sphere { radius: 0.6 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.7,
        friction: 0.2,
        },
        RigidBody {
            position: Vec3::new(-2.0, -1.0, 2.0),
            velocity: Vec3::new(0.5, 2.0, -1.5),
            angular_velocity: Vec3::new(-2.0, 1.0, 0.0),
            mass: 0.8,
            inertia_tensor: glam::Mat3::from_diagonal(Vec3::splat(0.32)),
            shape: ShapeType::Sphere(Sphere { radius: 0.4 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.7,
        friction: 0.2,
        },
    ];
    
    for body in bodies {
        backend.add_rigidbody(body);
    }
    
    // CRITICAL: Set gravity to zero for multi-body momentum conservation test
    backend.set_gravity(Vec3::ZERO);
    
    // Calculate initial total momentum
    let initial_linear_momentum = calculate_total_linear_momentum(&backend);
    let initial_angular_momentum = calculate_total_angular_momentum(&backend);
    
    // Simulate complex multi-body interactions
    let dt = 0.002;
    let steps = 2500; // 5 seconds of simulation
    
    for _ in 0..steps {
        backend.update(dt);
    }
    
    // Calculate final total momentum
    let final_linear_momentum = calculate_total_linear_momentum(&backend);
    let final_angular_momentum = calculate_total_angular_momentum(&backend);
    
    // Both linear and angular momentum should be conserved
    let linear_tolerance = 0.03;
    let angular_tolerance = 0.06;
    
    let linear_error = (final_linear_momentum - initial_linear_momentum).length() 
        / (initial_linear_momentum.length() + 1e-6);
    let angular_error = (final_angular_momentum - initial_angular_momentum).length() 
        / (initial_angular_momentum.length() + 1e-6);
    
    assert!(
        linear_error < linear_tolerance,
        "Linear momentum conservation in multi-body system violated: error={:.6}%",
        linear_error * 100.0
    );
    
    assert!(
        angular_error < angular_tolerance,
        "Angular momentum conservation in multi-body system violated: error={:.6}%",
        angular_error * 100.0
    );
}

// Helper functions for momentum calculations

fn calculate_total_linear_momentum(backend: &CpuBackend) -> Vec3 {
    backend.rigidbodies().iter()
        .map(|body| body.mass * body.velocity)
        .fold(Vec3::ZERO, |acc, momentum| acc + momentum)
}

fn calculate_total_angular_momentum(backend: &CpuBackend) -> Vec3 {
    // Calculate total angular momentum about the center of mass of the system
    let center_of_mass = calculate_center_of_mass(backend);
    
    backend.rigidbodies().iter()
        .map(|body| {
            // Angular momentum = L_orbital + L_spin
            // L_orbital = r × (m * v)
            // L_spin = I * ω
            let r = body.position - center_of_mass;
            let orbital_momentum = r.cross(body.mass * body.velocity);
            let spin_momentum = body.inertia_tensor * body.angular_velocity;
            orbital_momentum + spin_momentum
        })
        .fold(Vec3::ZERO, |acc, momentum| acc + momentum)
}

fn calculate_center_of_mass(backend: &CpuBackend) -> Vec3 {
    let total_mass: f32 = backend.rigidbodies().iter().map(|body| body.mass).sum();
    let weighted_position: Vec3 = backend.rigidbodies().iter()
        .map(|body| body.mass * body.position)
        .fold(Vec3::ZERO, |acc, pos| acc + pos);
    
    if total_mass > 0.0 {
        weighted_position / total_mass
    } else {
        Vec3::ZERO
    }
}