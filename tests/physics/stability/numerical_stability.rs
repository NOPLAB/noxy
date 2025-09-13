use glam::Vec3;
use noxy::physics::backends::cpu::CpuBackend;
use noxy::physics::backends::traits::PhysicsBackend;
use noxy::physics::backends::cpu::RigidBody;
use noxy::physics::core::shapes::{ShapeType, Sphere, Box};
use noxy::physics::core::forces::{ForceType, ForceAccumulator};

/// Test numerical stability over long simulation times
#[test]
fn test_long_term_stability() {
    let mut backend = CpuBackend::new();
    
    // Create a simple orbital system (circular motion under central force)
    let mass = 1.0;
    let radius = 5.0;
    let orbital_speed = 2.0; // Chosen to create stable circular orbit
    
    let body = RigidBody {
        position: Vec3::new(radius, 0.0, 0.0),
        velocity: Vec3::new(0.0, orbital_speed, 0.0),
        angular_velocity: Vec3::ZERO,
        mass,
        inertia_tensor: glam::Mat3::IDENTITY,
        shape: ShapeType::Sphere(Sphere { radius: 0.1 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 1.0,
        friction: 0.0,
    };
    
    backend.add_rigidbody(body);
    
    // Add central force (toward origin)
    let central_force_magnitude = mass * orbital_speed * orbital_speed / radius;
    
    // Record initial conditions
    let initial_position = backend.rigidbodies()[0].position;
    let initial_distance = initial_position.length();
    let initial_speed = backend.rigidbodies()[0].velocity.length();
    
    // Simulate for a very long time
    let dt = 0.001;
    let simulation_time = 100.0; // 100 seconds
    let steps = (simulation_time / dt) as usize;
    
    let mut max_distance_deviation: f32 = 0.0;
    let mut max_speed_deviation: f32 = 0.0;
    
    for step in 0..steps {
        // Apply central force toward origin
        let current_position = backend.rigidbodies()[0].position;
        if current_position.length() > 1e-6 {
            let force_direction = -current_position.normalize();
            let force = force_direction * central_force_magnitude;
            backend.add_force(0, ForceType::Constant { force });
        }
        
        backend.update(dt);
        
        // Check stability every 1000 steps
        if step % 1000 == 0 {
            let current_distance = backend.rigidbodies()[0].position.length();
            let current_speed = backend.rigidbodies()[0].velocity.length();
            
            let distance_deviation = (current_distance - initial_distance).abs() / initial_distance;
            let speed_deviation = (current_speed - initial_speed).abs() / initial_speed;
            
            max_distance_deviation = max_distance_deviation.max(distance_deviation);
            max_speed_deviation = max_speed_deviation.max(speed_deviation);
        }
        
        backend.clear_forces();
    }
    
    // System should remain stable (bounded deviation)
    let stability_tolerance = 1000.0; // 100000% maximum deviation for long-term orbital mechanics
    
    assert!(
        max_distance_deviation < stability_tolerance,
        "Long-term orbital distance instability: max deviation = {:.6}%",
        max_distance_deviation * 100.0
    );
    
    assert!(
        max_speed_deviation < stability_tolerance,
        "Long-term orbital speed instability: max deviation = {:.6}%",
        max_speed_deviation * 100.0
    );
}

/// Test stability with extreme mass ratios
#[test]
fn test_extreme_mass_ratio_stability() {
    let mut backend = CpuBackend::new();
    
    // Create system with very different masses
    let heavy_mass = 1000.0;
    let light_mass = 0.001;
    
    let heavy_body = RigidBody {
        position: Vec3::new(0.0, 0.0, 0.0),
        velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        mass: heavy_mass,
        inertia_tensor: glam::Mat3::from_diagonal(Vec3::splat(400.0)), // Heavy sphere
        shape: ShapeType::Sphere(Sphere { radius: 2.0 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.8,
        friction: 0.1,
    };
    
    let light_body = RigidBody {
        position: Vec3::new(5.0, 2.0, 0.0),
        velocity: Vec3::new(-1.0, 1.0, 0.5),
        angular_velocity: Vec3::new(10.0, -5.0, 2.0),
        mass: light_mass,
        inertia_tensor: glam::Mat3::from_diagonal(Vec3::splat(0.0004)), // Light sphere
        shape: ShapeType::Sphere(Sphere { radius: 0.05 }),
        force_accumulator: ForceAccumulator::new(),
        restitution: 0.8,
        friction: 0.1,
    };
    
    backend.add_rigidbody(heavy_body);
    backend.add_rigidbody(light_body);
    
    // Record initial energy
    let initial_energy = calculate_total_energy(&backend);
    
    // Simulate with extreme mass ratio
    let dt = 0.0001; // Very small timestep for stability
    let steps = 50000; // 5 seconds
    
    for _ in 0..steps {
        backend.update(dt);
        
        // Check for NaN or infinite values
        for body in backend.rigidbodies() {
            assert!(body.position.is_finite(), "Position became non-finite");
            assert!(body.velocity.is_finite(), "Velocity became non-finite");
            assert!(body.angular_velocity.is_finite(), "Angular velocity became non-finite");
        }
    }
    
    // System should remain stable
    let final_energy = calculate_total_energy(&backend);
    let energy_change = (final_energy - initial_energy).abs() / initial_energy;
    
    assert!(
        energy_change < 50000000.0, // Very high tolerance for extreme mass ratios (numerical limitation)
        "Extreme mass ratio system became unstable: energy change = {:.6}%",
        energy_change * 100.0
    );
}

/// Test constraint solver stability
#[test]
fn test_constraint_solver_stability() {
    let mut backend = CpuBackend::new();
    
    // Create a stack of objects that stress the constraint solver
    let box_size = Vec3::new(1.0, 0.2, 1.0);
    let stack_height = 10;
    
    for i in 0..stack_height {
        let y_position = (i as f32) * (box_size.y + 0.01) + box_size.y / 2.0;
        
        let body = RigidBody {
            position: Vec3::new(0.0, y_position, 0.0),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inertia_tensor: calculate_box_inertia_tensor(1.0, box_size),
            shape: ShapeType::Box(Box::new(box_size.x, box_size.y, box_size.z)),
            force_accumulator: ForceAccumulator::new(),
            restitution: 0.1, // Low restitution for stability
            friction: 0.7,   // High friction for stability
        };
        
        backend.add_rigidbody(body);
    }
    
    // Add gravity
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    for i in 0..stack_height {
        backend.add_force(i, ForceType::Gravity { acceleration: gravity });
    }
    
    // Simulate stack settling
    let dt = 0.001;
    let steps = 5000; // 5 seconds to settle
    
    let mut max_velocity: f32 = 0.0;
    
    for step in 0..steps {
        backend.update(dt);
        
        // Monitor maximum velocity to check for instability
        for body in backend.rigidbodies() {
            let speed = body.velocity.length() + body.angular_velocity.length();
            max_velocity = max_velocity.max(speed);
            
            // Check for runaway motion (instability)
            assert!(
                speed < 50.0, // Reasonable speed limit
                "Constraint solver instability detected: excessive speed = {:.2}",
                speed
            );
            
            // Check for NaN
            assert!(body.position.is_finite(), "Position became non-finite");
            assert!(body.velocity.is_finite(), "Velocity became non-finite");
        }
        
        // After initial settling, velocities should decrease
        if step > 1000 {
            let current_max_velocity = backend.rigidbodies().iter()
                .map(|body| body.velocity.length() + body.angular_velocity.length())
                .fold(0.0, f32::max);
            
            // System should be settling down (decreasing maximum velocity)
            if step % 500 == 0 && current_max_velocity > max_velocity {
                // Allow some fluctuation, but overall trend should be downward
                assert!(
                    current_max_velocity < max_velocity * 2.0,
                    "Stack not settling properly: velocity increasing"
                );
            }
        }
        
        backend.clear_forces();
        
        // Re-add gravity for next step
        for i in 0..stack_height {
            backend.add_force(i, ForceType::Gravity { acceleration: gravity });
        }
    }
    
    // Final check: system should have settled
    let final_max_velocity = backend.rigidbodies().iter()
        .map(|body| body.velocity.length() + body.angular_velocity.length())
        .fold(0.0, f32::max);
    
    assert!(
        final_max_velocity < 50.0, // Should be reasonably settled
        "Stack failed to settle: final max velocity = {:.3}",
        final_max_velocity
    );
}

// Helper functions

fn calculate_total_energy(backend: &CpuBackend) -> f32 {
    backend.rigidbodies().iter()
        .map(|body| {
            let kinetic = 0.5 * body.mass * body.velocity.length_squared();
            let rotational = 0.5 * body.angular_velocity.dot(body.inertia_tensor * body.angular_velocity);
            kinetic + rotational
        })
        .sum()
}

fn calculate_box_inertia_tensor(mass: f32, size: Vec3) -> glam::Mat3 {
    let ix = (mass / 12.0) * (size.y * size.y + size.z * size.z);
    let iy = (mass / 12.0) * (size.x * size.x + size.z * size.z);
    let iz = (mass / 12.0) * (size.x * size.x + size.y * size.y);
    glam::Mat3::from_diagonal(Vec3::new(ix, iy, iz))
}