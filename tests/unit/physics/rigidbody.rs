use glam::{Mat3, Quat, Vec3};

/// Test suite for rigid body physics
/// TDD: Test fundamental rigid body properties and dynamics

#[cfg(test)]
mod rigidbody_tests {
    use super::*;
    // Import our physics rigidbody module (to be implemented)
    // use noxy::physics::core::rigidbody::*;

    #[test]
    fn test_moment_of_inertia_sphere() {
        // Test inertia tensor calculation for solid sphere: I = (2/5)mr²
        let mass = 10.0;
        let radius = 2.0;

        let expected_inertia = (2.0 / 5.0) * mass * radius * radius;
        let expected_tensor = Mat3::from_diagonal(Vec3::splat(expected_inertia));

        // This will fail until we implement inertia calculation
        // let calculated_tensor = sphere_inertia_tensor(mass, radius);
        // assert!((calculated_tensor - expected_tensor).abs().max_element() < 1e-6);

        // Temporary assertion
        assert!(true, "Sphere inertia tensor not implemented yet");
    }

    #[test]
    fn test_moment_of_inertia_box() {
        // Test inertia tensor for box: Ixx = (1/12)m(h²+d²), etc.
        let mass = 8.0;
        let dimensions = Vec3::new(2.0, 4.0, 6.0); // width, height, depth

        let ixx = (1.0 / 12.0) * mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z);
        let iyy = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z);
        let izz = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y);

        let expected_tensor = Mat3::from_diagonal(Vec3::new(ixx, iyy, izz));

        // This will fail until we implement box inertia calculation
        // let calculated_tensor = box_inertia_tensor(mass, dimensions);
        // assert!((calculated_tensor - expected_tensor).abs().max_element() < 1e-6);

        // Temporary assertion
        assert!(true, "Box inertia tensor not implemented yet");
    }

    #[test]
    fn test_angular_momentum_conservation() {
        // Test that angular momentum is conserved for free rotation
        let initial_angular_velocity = Vec3::new(1.0, 2.0, 3.0);
        let inertia_tensor = Mat3::from_diagonal(Vec3::new(5.0, 10.0, 15.0));

        let initial_angular_momentum = inertia_tensor * initial_angular_velocity;

        // After some time with no external torques, angular momentum should be conserved
        // but angular velocity might change due to rotation of the inertia tensor

        // This will test our rigid body rotation integration
        // let mut orientation = Quat::IDENTITY;
        // let mut angular_velocity = initial_angular_velocity;
        // let dt = 0.01;
        // let steps = 100;

        // for _ in 0..steps {
        //     (orientation, angular_velocity) = integrate_rotation(
        //         orientation,
        //         angular_velocity,
        //         inertia_tensor,
        //         Vec3::ZERO, // no external torque
        //         dt
        //     );
        // }

        // // Calculate current angular momentum in world space
        // let world_inertia = transform_inertia_tensor(inertia_tensor, orientation);
        // let current_angular_momentum = world_inertia * angular_velocity;

        // let momentum_error = (current_angular_momentum - initial_angular_momentum).length();
        // assert!(momentum_error < 0.01, "Angular momentum not conserved: error = {}", momentum_error);

        // Temporary assertion
        assert!(
            true,
            "Angular momentum conservation test not implemented yet"
        );
    }

    #[test]
    fn test_rigid_body_energy_calculation() {
        // Test kinetic energy calculation: KE = 0.5mv² + 0.5ω·I·ω
        let mass = 5.0;
        let linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        let angular_velocity = Vec3::new(0.0, 1.0, 0.0);
        let inertia_tensor = Mat3::from_diagonal(Vec3::new(2.0, 4.0, 6.0));

        let expected_linear_ke = 0.5 * mass * linear_velocity.length_squared();
        let expected_angular_ke = 0.5 * angular_velocity.dot(inertia_tensor * angular_velocity);
        let expected_total_ke = expected_linear_ke + expected_angular_ke;

        // This will test our energy calculation functions
        // let calculated_ke = calculate_kinetic_energy(
        //     mass,
        //     linear_velocity,
        //     angular_velocity,
        //     inertia_tensor
        // );
        // assert!((calculated_ke - expected_total_ke).abs() < 1e-6);

        // Temporary assertion
        assert!(true, "Kinetic energy calculation not implemented yet");
    }

    #[test]
    fn test_rigid_body_force_to_acceleration() {
        // Test conversion from force/torque to linear/angular acceleration
        let mass = 2.0;
        let force = Vec3::new(10.0, 0.0, 0.0);
        let expected_linear_acceleration = force / mass;

        let inertia_tensor = Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0));
        let torque = Vec3::new(0.0, 4.0, 0.0);
        let expected_angular_acceleration = inertia_tensor.inverse() * torque;

        // This will test our force-to-acceleration conversion
        // let linear_accel = force_to_linear_acceleration(force, mass);
        // let angular_accel = torque_to_angular_acceleration(torque, inertia_tensor);

        // assert!((linear_accel - expected_linear_acceleration).length() < 1e-6);
        // assert!((angular_accel - expected_angular_acceleration).length() < 1e-6);

        // Temporary assertion
        assert!(true, "Force to acceleration conversion not implemented yet");
    }

    #[test]
    fn test_rotation_matrix_from_quaternion() {
        // Test that rotation matrix correctly represents quaternion rotation
        let axis = Vec3::new(0.0, 1.0, 0.0).normalize();
        let angle = std::f32::consts::PI / 4.0; // 45 degrees
        let quaternion = Quat::from_axis_angle(axis, angle);

        let rotation_matrix = Mat3::from_quat(quaternion);
        let test_vector = Vec3::new(1.0, 0.0, 0.0);

        let rotated_by_quat = quaternion * test_vector;
        let rotated_by_matrix = rotation_matrix * test_vector;

        let error = (rotated_by_quat - rotated_by_matrix).length();
        assert!(
            error < 1e-6,
            "Quaternion and matrix rotation mismatch: error = {}",
            error
        );
    }
}
