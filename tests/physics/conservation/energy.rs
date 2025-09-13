use glam::Vec3;
use noxy::physics::backends::cpu::CpuBackend;
use noxy::physics::backends::traits::PhysicsBackend;
use noxy::physics::backends::cpu::RigidBody;
use noxy::physics::core::shapes::{ShapeType, Sphere};
use noxy::physics::core::forces::{ForceType, ForceAccumulator};

/// Test energy conservation in isolated system
#[test]
fn test_energy_conservation_isolated_system() {
    let mut backend = CpuBackend::new();
    
    // Create two rigid bodies with initial velocities (no gravity)
    let mass1 = 1.0;
    let mass2 = 2.0;
    let initial_velocity1 = Vec3::new(2.0, 0.0, 0.0);
    let initial_velocity2 = Vec3::new(-1.0, 0.0, 0.0);
    
    let body1 = RigidBody {
        position: Vec3::new(-5.0, 0.0, 0.0),
        velocity: initial_velocity1,
        angular_velocity: Vec3::ZERO,
        mass: mass1,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 1.0, // Perfect elastic collision
        friction: 0.0,   // No friction
    };
    
    let body2 = RigidBody {
        position: Vec3::new(5.0, 0.0, 0.0),
        velocity: initial_velocity2,
        angular_velocity: Vec3::ZERO,
        mass: mass2,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 1.0,
        friction: 0.0,
    };
    
    backend.add_rigidbody(body1);
    backend.add_rigidbody(body2);
    
    // CRITICAL: Set gravity to zero for isolated system test
    backend.set_gravity(Vec3::ZERO);
    
    // Calculate initial total energy
    let initial_kinetic_energy = calculate_total_kinetic_energy(&backend);
    let initial_potential_energy = 0.0; // No gravity, no potential energy
    let initial_total_energy = initial_kinetic_energy + initial_potential_energy;
    
    // Simulate for several steps
    let dt = 0.016; // 60fps
    let steps = 300; // About 5 seconds
    
    println!("Initial energy: {:.6}", initial_total_energy);
    println!("Initial kinetic: {:.6}, potential: {:.6}", initial_kinetic_energy, initial_potential_energy);
    
    for i in 0..steps {
        backend.update(dt);
        
        if i % 60 == 0 {
            let current_ke = calculate_total_kinetic_energy(&backend);
            let current_pe = 0.0;
            let current_total = current_ke + current_pe;
            println!("Step {}: KE={:.6}, PE={:.6}, Total={:.6}", i, current_ke, current_pe, current_total);
        }
    }
    
    // Calculate final total energy
    let final_kinetic_energy = calculate_total_kinetic_energy(&backend);
    let final_potential_energy = 0.0;
    let final_total_energy = final_kinetic_energy + final_potential_energy;
    
    // Energy should be conserved (within numerical tolerance)
    let energy_tolerance = 0.01; // 1% tolerance
    let energy_error = (final_total_energy - initial_total_energy).abs() / initial_total_energy;
    
    assert!(
        energy_error < energy_tolerance,
        "Energy conservation violated: initial={:.6}, final={:.6}, error={:.6}%",
        initial_total_energy,
        final_total_energy,
        energy_error * 100.0
    );
}

/// Test energy conservation with gravity
#[test]
fn test_energy_conservation_with_gravity() {
    let mut backend = CpuBackend::new();
    
    // Single rigid body falling under gravity
    let mass = 1.0;
    let initial_height = 10.0;
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    
    let body = RigidBody {
        position: Vec3::new(0.0, initial_height, 0.0),
        velocity: Vec3::ZERO, // Start at rest
        angular_velocity: Vec3::ZERO,
        mass,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.5 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 1.0,
        friction: 0.0,
    };
    
    backend.add_rigidbody(body);
    backend.add_force(0, ForceType::Gravity { acceleration: gravity });
    
    // Calculate initial total energy
    let initial_kinetic_energy = calculate_total_kinetic_energy(&backend);
    let initial_potential_energy = calculate_gravitational_potential_energy(&backend, gravity);
    let initial_total_energy = initial_kinetic_energy + initial_potential_energy;
    
    // Simulate falling motion
    let dt = 0.001; // Small timestep for accuracy
    let steps = 1000; // 1 second of fall
    
    for _ in 0..steps {
        backend.update(dt);
    }
    
    // Calculate final total energy
    let final_kinetic_energy = calculate_total_kinetic_energy(&backend);
    let final_potential_energy = calculate_gravitational_potential_energy(&backend, gravity);
    let final_total_energy = final_kinetic_energy + final_potential_energy;
    
    // Energy should be conserved
    let energy_tolerance = 0.02; // 2% tolerance for numerical integration
    let energy_error = (final_total_energy - initial_total_energy).abs() / initial_total_energy;
    
    assert!(
        energy_error < energy_tolerance,
        "Energy conservation with gravity violated: initial={:.6}, final={:.6}, error={:.6}%",
        initial_total_energy,
        final_total_energy,
        energy_error * 100.0
    );
}

/// Test rotational energy conservation
#[test]
fn test_rotational_energy_conservation() {
    let mut backend = CpuBackend::new();
    
    // Create a spinning rigid body (no forces, isolated system)
    let mass = 1.0;
    let initial_angular_velocity = Vec3::new(0.0, 5.0, 0.0); // Spinning around Y-axis
    let inertia = 0.4 * mass; // Sphere inertia around axis
    
    let body = RigidBody {
        position: Vec3::ZERO,
        velocity: Vec3::ZERO,
        angular_velocity: initial_angular_velocity,
        mass,
        inertia_tensor: glam::Mat3::from_diagonal(Vec3::new(inertia, inertia, inertia)),
        shape: ShapeType::Sphere(Sphere { radius: 1.0 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 1.0,
        friction: 0.0,
    };
    
    backend.add_rigidbody(body);
    
    // Calculate initial rotational energy
    let initial_rotational_energy = calculate_total_rotational_energy(&backend);
    
    // Simulate for several steps
    let dt = 0.016;
    let steps = 100;
    
    for _ in 0..steps {
        backend.update(dt);
    }
    
    // Calculate final rotational energy
    let final_rotational_energy = calculate_total_rotational_energy(&backend);
    
    // Rotational energy should be conserved
    let energy_tolerance = 0.01;
    let energy_error = (final_rotational_energy - initial_rotational_energy).abs() / initial_rotational_energy;
    
    assert!(
        energy_error < energy_tolerance,
        "Rotational energy conservation violated: initial={:.6}, final={:.6}, error={:.6}%",
        initial_rotational_energy,
        final_rotational_energy,
        energy_error * 100.0
    );
}

// Helper functions for energy calculations

fn calculate_total_kinetic_energy(backend: &CpuBackend) -> f32 {
    backend.rigidbodies().iter()
        .map(|body| 0.5 * body.mass * body.velocity.length_squared())
        .sum()
}

fn calculate_total_rotational_energy(backend: &CpuBackend) -> f32 {
    backend.rigidbodies().iter()
        .map(|body| {
            let angular_momentum = body.inertia_tensor * body.angular_velocity;
            0.5 * body.angular_velocity.dot(angular_momentum)
        })
        .sum()
}

fn calculate_gravitational_potential_energy(backend: &CpuBackend, gravity: Vec3) -> f32 {
    backend.rigidbodies().iter()
        .map(|body| {
            // Gravitational potential energy = m * g * h
            // Using negative gravity since we typically use negative Y for down
            -body.mass * gravity.y * body.position.y
        })
        .sum()
}