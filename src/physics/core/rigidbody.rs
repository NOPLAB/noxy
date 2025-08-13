// Rigid body physics implementation

use glam::{Mat3, Quat, Vec3};

/// Calculate inertia tensor for a solid sphere
/// Formula: I = (2/5) * m * r²
pub fn sphere_inertia_tensor(mass: f32, radius: f32) -> Mat3 {
    let inertia = (2.0 / 5.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::splat(inertia))
}

/// Calculate inertia tensor for a box
/// Formula: Ixx = (1/12)*m*(h²+d²), Iyy = (1/12)*m*(w²+d²), Izz = (1/12)*m*(w²+h²)
pub fn box_inertia_tensor(mass: f32, dimensions: Vec3) -> Mat3 {
    let w = dimensions.x;
    let h = dimensions.y;
    let d = dimensions.z;

    let ixx = (1.0 / 12.0) * mass * (h * h + d * d);
    let iyy = (1.0 / 12.0) * mass * (w * w + d * d);
    let izz = (1.0 / 12.0) * mass * (w * w + h * h);

    Mat3::from_diagonal(Vec3::new(ixx, iyy, izz))
}

/// Calculate kinetic energy: KE = 0.5*m*v² + 0.5*ω·I·ω
pub fn calculate_kinetic_energy(
    mass: f32,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    inertia_tensor: Mat3,
) -> f32 {
    let linear_ke = 0.5 * mass * linear_velocity.length_squared();
    let angular_ke = 0.5 * angular_velocity.dot(inertia_tensor * angular_velocity);
    linear_ke + angular_ke
}

/// Convert force to linear acceleration: a = F/m
pub fn force_to_linear_acceleration(force: Vec3, mass: f32) -> Vec3 {
    force / mass
}

/// Convert torque to angular acceleration: α = I⁻¹ * τ
pub fn torque_to_angular_acceleration(torque: Vec3, inertia_tensor: Mat3) -> Vec3 {
    inertia_tensor.inverse() * torque
}

/// Transform inertia tensor from body space to world space
/// I_world = R * I_body * R^T
pub fn transform_inertia_tensor(inertia_tensor: Mat3, orientation: Quat) -> Mat3 {
    let rotation_matrix = Mat3::from_quat(orientation);
    rotation_matrix * inertia_tensor * rotation_matrix.transpose()
}

/// Integration step for rigid body rotation
/// Returns (new_orientation, new_angular_velocity)
pub fn integrate_rotation(
    orientation: Quat,
    angular_velocity: Vec3,
    inertia_tensor: Mat3,
    torque: Vec3,
    dt: f32,
) -> (Quat, Vec3) {
    // Calculate angular acceleration
    let angular_acceleration = torque_to_angular_acceleration(torque, inertia_tensor);

    // Update angular velocity
    let new_angular_velocity = angular_velocity + angular_acceleration * dt;

    // Update orientation using quaternion integration
    let angular_velocity_quat = Quat::from_xyzw(
        angular_velocity.x * 0.5,
        angular_velocity.y * 0.5,
        angular_velocity.z * 0.5,
        0.0,
    );

    let new_orientation = (orientation + angular_velocity_quat * orientation * dt).normalize();

    (new_orientation, new_angular_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::{approx_eq, vec3_approx_eq};

    #[test]
    fn test_moment_of_inertia_sphere() {
        let mass = 10.0;
        let radius = 2.0;

        let expected_inertia = (2.0 / 5.0) * mass * radius * radius;
        let expected_tensor = Mat3::from_diagonal(Vec3::splat(expected_inertia));
        let calculated_tensor = sphere_inertia_tensor(mass, radius);

        for i in 0..3 {
            for j in 0..3 {
                assert!(approx_eq(
                    calculated_tensor.col(j)[i],
                    expected_tensor.col(j)[i],
                    1e-6
                ));
            }
        }
    }

    #[test]
    fn test_moment_of_inertia_box() {
        let mass = 8.0;
        let dimensions = Vec3::new(2.0, 4.0, 6.0);

        let ixx = (1.0 / 12.0) * mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z);
        let iyy = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z);
        let izz = (1.0 / 12.0) * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y);

        let expected_tensor = Mat3::from_diagonal(Vec3::new(ixx, iyy, izz));
        let calculated_tensor = box_inertia_tensor(mass, dimensions);

        for i in 0..3 {
            for j in 0..3 {
                assert!(approx_eq(
                    calculated_tensor.col(j)[i],
                    expected_tensor.col(j)[i],
                    1e-6
                ));
            }
        }
    }

    #[test]
    fn test_rigid_body_energy_calculation() {
        let mass = 5.0;
        let linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        let angular_velocity = Vec3::new(0.0, 1.0, 0.0);
        let inertia_tensor = Mat3::from_diagonal(Vec3::new(2.0, 4.0, 6.0));

        let expected_linear_ke = 0.5 * mass * linear_velocity.length_squared();
        let expected_angular_ke = 0.5 * angular_velocity.dot(inertia_tensor * angular_velocity);
        let expected_total_ke = expected_linear_ke + expected_angular_ke;

        let calculated_ke =
            calculate_kinetic_energy(mass, linear_velocity, angular_velocity, inertia_tensor);

        assert!(approx_eq(calculated_ke, expected_total_ke, 1e-6));
    }

    #[test]
    fn test_rigid_body_force_to_acceleration() {
        let mass = 2.0;
        let force = Vec3::new(10.0, 0.0, 0.0);
        let expected_linear_acceleration = force / mass;

        let inertia_tensor = Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0));
        let torque = Vec3::new(0.0, 4.0, 0.0);
        let expected_angular_acceleration = inertia_tensor.inverse() * torque;

        let linear_accel = force_to_linear_acceleration(force, mass);
        let angular_accel = torque_to_angular_acceleration(torque, inertia_tensor);

        assert!(vec3_approx_eq(
            linear_accel,
            expected_linear_acceleration,
            1e-6
        ));
        assert!(vec3_approx_eq(
            angular_accel,
            expected_angular_acceleration,
            1e-6
        ));
    }

    #[test]
    fn test_rotation_matrix_from_quaternion() {
        let axis = Vec3::new(0.0, 1.0, 0.0).normalize();
        let angle = std::f32::consts::PI / 4.0;
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
