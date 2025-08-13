use bytemuck::{Pod, Zeroable};
/// Shape definitions for physics simulation
use glam::{Mat3, Vec3};

/// Axis-aligned bounding box
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoundingBox {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Check if this bounding box intersects with another
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.max.x > other.min.x && self.min.x < other.max.x &&
        self.max.y > other.min.y && self.min.y < other.max.y &&
        self.max.z > other.min.z && self.min.z < other.max.z
    }

    /// Calculate the volume of this bounding box
    pub fn volume(&self) -> f32 {
        let size = self.max - self.min;
        size.x * size.y * size.z
    }

    /// Get the center point of this bounding box
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }
}

/// Common shape trait for all geometric shapes
pub trait Shape: Send + Sync {
    /// Calculate the volume of the shape
    fn volume(&self) -> f32;

    /// Calculate the surface area of the shape
    fn surface_area(&self) -> f32;

    /// Check if a point is inside the shape (assuming shape is centered at origin)
    fn contains_point(&self, point: Vec3) -> bool;

    /// Get the axis-aligned bounding box for the shape
    fn bounding_box(&self) -> BoundingBox;

    /// Calculate the moment of inertia tensor for the given mass
    fn moment_of_inertia(&self, mass: f32) -> Mat3;
}

/// Box shape (rectangular cuboid)
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Box {
    width: f32,  // x-dimension
    height: f32, // y-dimension
    depth: f32,  // z-dimension
}

unsafe impl Pod for Box {}
unsafe impl Zeroable for Box {}

impl Box {
    /// Create a new box with given dimensions
    pub fn new(width: f32, height: f32, depth: f32) -> Self {
        Self {
            width,
            height,
            depth,
        }
    }

    /// Get the width (x-dimension)
    pub fn width(&self) -> f32 {
        self.width
    }

    /// Get the height (y-dimension)
    pub fn height(&self) -> f32 {
        self.height
    }

    /// Get the depth (z-dimension)
    pub fn depth(&self) -> f32 {
        self.depth
    }
}

impl Shape for Box {
    fn volume(&self) -> f32 {
        self.width * self.height * self.depth
    }

    fn surface_area(&self) -> f32 {
        2.0 * (self.width * self.height + self.height * self.depth + self.depth * self.width)
    }

    fn contains_point(&self, point: Vec3) -> bool {
        let half_width = self.width / 2.0;
        let half_height = self.height / 2.0;
        let half_depth = self.depth / 2.0;

        point.x.abs() <= half_width && point.y.abs() <= half_height && point.z.abs() <= half_depth
    }

    fn bounding_box(&self) -> BoundingBox {
        let half_width = self.width / 2.0;
        let half_height = self.height / 2.0;
        let half_depth = self.depth / 2.0;

        BoundingBox::new(
            Vec3::new(-half_width, -half_height, -half_depth),
            Vec3::new(half_width, half_height, half_depth),
        )
    }

    fn moment_of_inertia(&self, mass: f32) -> Mat3 {
        // Moment of inertia for a box around its center
        let ixx = mass * (self.height * self.height + self.depth * self.depth) / 12.0;
        let iyy = mass * (self.width * self.width + self.depth * self.depth) / 12.0;
        let izz = mass * (self.width * self.width + self.height * self.height) / 12.0;

        Mat3::from_diagonal(Vec3::new(ixx, iyy, izz))
    }
}

/// Sphere shape
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Sphere {
    pub radius: f32,  // Made public for testing
}

unsafe impl Pod for Sphere {}
unsafe impl Zeroable for Sphere {}

impl Sphere {
    /// Create a new sphere with given radius
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }

    /// Get the radius
    pub fn radius(&self) -> f32 {
        self.radius
    }
}

impl Shape for Sphere {
    fn volume(&self) -> f32 {
        (4.0 / 3.0) * std::f32::consts::PI * self.radius.powi(3)
    }

    fn surface_area(&self) -> f32 {
        4.0 * std::f32::consts::PI * self.radius.powi(2)
    }

    fn contains_point(&self, point: Vec3) -> bool {
        point.length() <= self.radius
    }

    fn bounding_box(&self) -> BoundingBox {
        let extent = Vec3::splat(self.radius);
        BoundingBox::new(-extent, extent)
    }

    fn moment_of_inertia(&self, mass: f32) -> Mat3 {
        // Moment of inertia for a solid sphere
        let inertia = (2.0 / 5.0) * mass * self.radius * self.radius;
        Mat3::from_diagonal(Vec3::splat(inertia))
    }
}

/// Cylinder shape
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Cylinder {
    radius: f32,
    height: f32,
}

unsafe impl Pod for Cylinder {}
unsafe impl Zeroable for Cylinder {}

impl Cylinder {
    /// Create a new cylinder with given radius and height
    pub fn new(radius: f32, height: f32) -> Self {
        Self { radius, height }
    }

    /// Get the radius
    pub fn radius(&self) -> f32 {
        self.radius
    }

    /// Get the height
    pub fn height(&self) -> f32 {
        self.height
    }
}

impl Shape for Cylinder {
    fn volume(&self) -> f32 {
        std::f32::consts::PI * self.radius.powi(2) * self.height
    }

    fn surface_area(&self) -> f32 {
        // 2 * circular ends + side surface
        2.0 * std::f32::consts::PI * self.radius.powi(2)
            + 2.0 * std::f32::consts::PI * self.radius * self.height
    }

    fn contains_point(&self, point: Vec3) -> bool {
        let radial_distance_squared = point.x * point.x + point.z * point.z;
        let half_height = self.height / 2.0;

        radial_distance_squared <= self.radius * self.radius && point.y.abs() <= half_height
    }

    fn bounding_box(&self) -> BoundingBox {
        let half_height = self.height / 2.0;
        BoundingBox::new(
            Vec3::new(-self.radius, -half_height, -self.radius),
            Vec3::new(self.radius, half_height, self.radius),
        )
    }

    fn moment_of_inertia(&self, mass: f32) -> Mat3 {
        // Moment of inertia for a cylinder around its center axis (y-axis)
        let ixx_izz = mass * (3.0 * self.radius * self.radius + self.height * self.height) / 12.0;
        let iyy = mass * self.radius * self.radius / 2.0;

        Mat3::from_diagonal(Vec3::new(ixx_izz, iyy, ixx_izz))
    }
}

/// Shape variants enum for runtime polymorphism
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShapeType {
    Box(Box),
    Sphere(Sphere),
    Cylinder(Cylinder),
}

impl Shape for ShapeType {
    fn volume(&self) -> f32 {
        match self {
            ShapeType::Box(shape) => shape.volume(),
            ShapeType::Sphere(shape) => shape.volume(),
            ShapeType::Cylinder(shape) => shape.volume(),
        }
    }

    fn surface_area(&self) -> f32 {
        match self {
            ShapeType::Box(shape) => shape.surface_area(),
            ShapeType::Sphere(shape) => shape.surface_area(),
            ShapeType::Cylinder(shape) => shape.surface_area(),
        }
    }

    fn contains_point(&self, point: Vec3) -> bool {
        match self {
            ShapeType::Box(shape) => shape.contains_point(point),
            ShapeType::Sphere(shape) => shape.contains_point(point),
            ShapeType::Cylinder(shape) => shape.contains_point(point),
        }
    }

    fn bounding_box(&self) -> BoundingBox {
        match self {
            ShapeType::Box(shape) => shape.bounding_box(),
            ShapeType::Sphere(shape) => shape.bounding_box(),
            ShapeType::Cylinder(shape) => shape.bounding_box(),
        }
    }

    fn moment_of_inertia(&self, mass: f32) -> Mat3 {
        match self {
            ShapeType::Box(shape) => shape.moment_of_inertia(mass),
            ShapeType::Sphere(shape) => shape.moment_of_inertia(mass),
            ShapeType::Cylinder(shape) => shape.moment_of_inertia(mass),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_box_basic_properties() {
        let box_shape = Box::new(2.0, 3.0, 4.0);
        assert_eq!(box_shape.width(), 2.0);
        assert_eq!(box_shape.height(), 3.0);
        assert_eq!(box_shape.depth(), 4.0);
        assert_eq!(box_shape.volume(), 24.0);
    }

    #[test]
    fn test_sphere_basic_properties() {
        let sphere = Sphere::new(1.0);
        assert_eq!(sphere.radius(), 1.0);
        assert!((sphere.volume() - (4.0 / 3.0) * std::f32::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_cylinder_basic_properties() {
        let cylinder = Cylinder::new(1.0, 2.0);
        assert_eq!(cylinder.radius(), 1.0);
        assert_eq!(cylinder.height(), 2.0);
        assert!((cylinder.volume() - 2.0 * std::f32::consts::PI).abs() < 0.001);
    }
}
