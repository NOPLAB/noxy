/// Unit tests for physics shapes
use glam::Vec3;
use noxy::physics::core::shapes::{Box as BoxShape, Cylinder, Shape, Sphere};

#[cfg(test)]
mod shape_tests {
    use super::*;

    #[test]
    fn test_box_creation() {
        let box_shape = BoxShape::new(2.0, 3.0, 4.0);

        assert_eq!(box_shape.width(), 2.0);
        assert_eq!(box_shape.height(), 3.0);
        assert_eq!(box_shape.depth(), 4.0);
    }

    #[test]
    fn test_box_volume() {
        let box_shape = BoxShape::new(2.0, 3.0, 4.0);
        let expected_volume = 2.0 * 3.0 * 4.0;

        assert!((box_shape.volume() - expected_volume).abs() < f32::EPSILON);
    }

    #[test]
    fn test_box_surface_area() {
        let box_shape = BoxShape::new(2.0, 3.0, 4.0);
        let expected_surface_area = 2.0 * (2.0 * 3.0 + 3.0 * 4.0 + 4.0 * 2.0);

        assert!((box_shape.surface_area() - expected_surface_area).abs() < f32::EPSILON);
    }

    #[test]
    fn test_sphere_creation() {
        let sphere_shape = Sphere::new(5.0);

        assert_eq!(sphere_shape.radius(), 5.0);
    }

    #[test]
    fn test_sphere_volume() {
        let sphere_shape = Sphere::new(3.0);
        let expected_volume = (4.0 / 3.0) * std::f32::consts::PI * 3.0_f32.powi(3);

        assert!((sphere_shape.volume() - expected_volume).abs() < 0.001);
    }

    #[test]
    fn test_sphere_surface_area() {
        let sphere_shape = Sphere::new(3.0);
        let expected_surface_area = 4.0 * std::f32::consts::PI * 3.0_f32.powi(2);

        assert!((sphere_shape.surface_area() - expected_surface_area).abs() < 0.001);
    }

    #[test]
    fn test_cylinder_creation() {
        let cylinder_shape = Cylinder::new(2.0, 5.0);

        assert_eq!(cylinder_shape.radius(), 2.0);
        assert_eq!(cylinder_shape.height(), 5.0);
    }

    #[test]
    fn test_cylinder_volume() {
        let cylinder_shape = Cylinder::new(2.0, 5.0);
        let expected_volume = std::f32::consts::PI * 2.0_f32.powi(2) * 5.0;

        assert!((cylinder_shape.volume() - expected_volume).abs() < 0.001);
    }

    #[test]
    fn test_shape_trait_polymorphism() {
        let shapes: Vec<Box<dyn Shape>> = vec![
            Box::new(BoxShape::new(1.0, 2.0, 3.0)),
            Box::new(Sphere::new(2.0)),
            Box::new(Cylinder::new(1.5, 3.0)),
        ];

        for shape in shapes {
            // All shapes should have positive volume
            assert!(shape.volume() > 0.0);
            assert!(shape.surface_area() > 0.0);
        }
    }

    #[test]
    fn test_box_contains_point() {
        let box_shape = BoxShape::new(4.0, 6.0, 8.0);

        // Point inside the box (centered at origin)
        assert!(box_shape.contains_point(Vec3::new(1.0, 2.0, 3.0)));

        // Point outside the box
        assert!(!box_shape.contains_point(Vec3::new(3.0, 4.0, 5.0)));

        // Point on the boundary (should be considered inside)
        assert!(box_shape.contains_point(Vec3::new(2.0, 3.0, 4.0)));
    }

    #[test]
    fn test_sphere_contains_point() {
        let sphere_shape = Sphere::new(5.0);

        // Point inside the sphere (centered at origin)
        assert!(sphere_shape.contains_point(Vec3::new(3.0, 0.0, 0.0)));

        // Point outside the sphere
        assert!(!sphere_shape.contains_point(Vec3::new(6.0, 0.0, 0.0)));

        // Point on the boundary
        assert!(sphere_shape.contains_point(Vec3::new(5.0, 0.0, 0.0)));
    }

    #[test]
    fn test_bounding_box() {
        let box_shape = BoxShape::new(4.0, 6.0, 8.0);
        let sphere_shape = Sphere::new(3.0);

        let box_bb = box_shape.bounding_box();
        assert_eq!(box_bb.min, Vec3::new(-2.0, -3.0, -4.0));
        assert_eq!(box_bb.max, Vec3::new(2.0, 3.0, 4.0));

        let sphere_bb = sphere_shape.bounding_box();
        assert_eq!(sphere_bb.min, Vec3::new(-3.0, -3.0, -3.0));
        assert_eq!(sphere_bb.max, Vec3::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn test_moment_of_inertia() {
        let box_shape = BoxShape::new(2.0, 4.0, 6.0);
        let sphere_shape = Sphere::new(3.0);
        let mass = 10.0;

        let box_inertia = box_shape.moment_of_inertia(mass);
        let sphere_inertia = sphere_shape.moment_of_inertia(mass);

        // Check that inertia tensors are positive definite
        assert!(box_inertia.x_axis.x > 0.0);
        assert!(box_inertia.y_axis.y > 0.0);
        assert!(box_inertia.z_axis.z > 0.0);

        assert!(sphere_inertia.x_axis.x > 0.0);
        assert!(sphere_inertia.y_axis.y > 0.0);
        assert!(sphere_inertia.z_axis.z > 0.0);

        // For a sphere, all diagonal elements should be equal
        assert!((sphere_inertia.x_axis.x - sphere_inertia.y_axis.y).abs() < f32::EPSILON);
        assert!((sphere_inertia.y_axis.y - sphere_inertia.z_axis.z).abs() < f32::EPSILON);
    }
}
