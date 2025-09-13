// Numerical integration methods for physics simulation

use glam::Vec3;

/// Verlet integration step
/// Returns (new_position, new_velocity)
pub fn verlet_step(position: Vec3, velocity: Vec3, acceleration: Vec3, dt: f32) -> (Vec3, Vec3) {
    // Standard Verlet integration
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
    // v(t+dt) = v(t) + a(t)*dt

    let new_position = position + velocity * dt + 0.5 * acceleration * dt * dt;
    let new_velocity = velocity + acceleration * dt;

    (new_position, new_velocity)
}

/// Verlet integration step with stability improvements for long simulations
/// Returns (new_position, new_velocity)
pub fn stable_verlet_step(position: Vec3, velocity: Vec3, acceleration: Vec3, dt: f32) -> (Vec3, Vec3) {
    // Velocity Verlet integration with optional damping for stability
    
    // Apply very minimal damping only for high velocities to prevent runaway
    let damping_threshold = 50.0; // m/s
    let velocity_magnitude = velocity.length();
    
    let damped_velocity = if velocity_magnitude > damping_threshold {
        let damping_factor = 0.999; // Minimal damping
        velocity * damping_factor
    } else {
        velocity
    };

    let new_position = position + damped_velocity * dt + 0.5 * acceleration * dt * dt;
    let new_velocity = damped_velocity + acceleration * dt;

    // Clamp velocity only at extreme values to prevent numerical overflow
    let max_velocity = 1000.0; // m/s
    let clamped_velocity = if new_velocity.length() > max_velocity {
        new_velocity.normalize() * max_velocity
    } else {
        new_velocity
    };

    (new_position, clamped_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::vec3_approx_eq;

    #[test]
    fn test_verlet_integration_constant_velocity() {
        let initial_position = Vec3::new(0.0, 0.0, 0.0);
        let initial_velocity = Vec3::new(1.0, 0.0, 0.0);
        let acceleration = Vec3::ZERO;
        let dt = 0.1;

        let expected_position = initial_position + initial_velocity * dt;

        let (new_position, _) = verlet_step(initial_position, initial_velocity, acceleration, dt);

        assert!(vec3_approx_eq(new_position, expected_position, 1e-6));
    }

    #[test]
    fn test_verlet_integration_constant_acceleration() {
        let initial_position = Vec3::new(0.0, 10.0, 0.0);
        let initial_velocity = Vec3::ZERO;
        let acceleration = Vec3::new(0.0, -9.81, 0.0);
        let dt = 0.1;

        let expected_position =
            initial_position + initial_velocity * dt + 0.5 * acceleration * dt * dt;

        let (new_position, _) = verlet_step(initial_position, initial_velocity, acceleration, dt);

        assert!(vec3_approx_eq(new_position, expected_position, 1e-6));
    }

    #[test]
    fn test_energy_conservation_free_fall() {
        let initial_position = Vec3::new(0.0, 10.0, 0.0);
        let initial_velocity = Vec3::ZERO;
        let acceleration = Vec3::new(0.0, -9.81, 0.0);
        let mass = 1.0;
        let dt = 0.01;
        let steps = 100;

        let initial_potential_energy = mass * 9.81 * initial_position.y;
        let initial_kinetic_energy = 0.5 * mass * initial_velocity.length_squared();
        let initial_total_energy = initial_potential_energy + initial_kinetic_energy;

        let mut position = initial_position;
        let mut velocity = initial_velocity;

        for _ in 0..steps {
            (position, velocity) = verlet_step(position, velocity, acceleration, dt);
        }

        let final_potential_energy = mass * 9.81 * position.y;
        let final_kinetic_energy = 0.5 * mass * velocity.length_squared();
        let final_total_energy = final_potential_energy + final_kinetic_energy;

        let energy_error = (final_total_energy - initial_total_energy).abs();
        assert!(
            energy_error < 0.01,
            "Energy not conserved: error = {}",
            energy_error
        );
    }

    #[test]
    fn test_numerical_stability_long_simulation() {
        let initial_position = Vec3::new(0.0, 0.0, 0.0);
        let initial_velocity = Vec3::new(1.0, 1.0, 0.0);
        let acceleration = Vec3::ZERO;
        let dt = 0.001;
        let steps = 10000;

        let mut position = initial_position;
        let mut velocity = initial_velocity;

        for _ in 0..steps {
            (position, velocity) = verlet_step(position, velocity, acceleration, dt);
        }

        let expected_position = initial_position + initial_velocity * (dt * steps as f32);
        let position_error = (position - expected_position).length();

        assert!(
            position_error < 0.1,
            "Numerical instability detected: error = {}",
            position_error
        );
    }
}
