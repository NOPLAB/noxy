use glam::Vec3;
use crate::physics::core::shapes::{BoundingBox, Box as BoxShape, Sphere};

/// Contact point information for collision response
#[derive(Debug, Clone)]
pub struct ContactPoint {
    /// Contact point position in world coordinates
    pub position: Vec3,
    /// Contact normal (from body A to body B)
    pub normal: Vec3,
    /// Penetration depth (positive means overlap)
    pub depth: f32,
    /// Indices of the two colliding bodies
    pub body_a: usize,
    pub body_b: usize,
}

/// Collision detection result
#[derive(Debug)]
pub struct CollisionManifold {
    pub contacts: Vec<ContactPoint>,
}

impl CollisionManifold {
    pub fn new() -> Self {
        Self {
            contacts: Vec::new(),
        }
    }

    pub fn add_contact(&mut self, contact: ContactPoint) {
        self.contacts.push(contact);
    }

    pub fn is_colliding(&self) -> bool {
        !self.contacts.is_empty()
    }
}

/// Broad phase collision detection using axis-aligned bounding boxes
pub struct BroadPhaseDetector {
    potential_pairs: Vec<(usize, usize)>,
}

impl BroadPhaseDetector {
    pub fn new() -> Self {
        Self {
            potential_pairs: Vec::new(),
        }
    }

    /// Find potentially colliding pairs using AABB intersection
    pub fn find_potential_collisions(
        &mut self,
        positions: &[Vec3],
        bounding_boxes: &[BoundingBox],
    ) -> &[(usize, usize)] {
        self.potential_pairs.clear();
        
        // Simple O(n²) broad phase - can be optimized with spatial partitioning later
        for i in 0..positions.len() {
            for j in (i + 1)..positions.len() {
                let bbox_a = BoundingBox {
                    min: positions[i] + bounding_boxes[i].min,
                    max: positions[i] + bounding_boxes[i].max,
                };
                let bbox_b = BoundingBox {
                    min: positions[j] + bounding_boxes[j].min,
                    max: positions[j] + bounding_boxes[j].max,
                };

                if bbox_a.intersects(&bbox_b) {
                    self.potential_pairs.push((i, j));
                }
            }
        }

        &self.potential_pairs
    }
}

/// Narrow phase collision detection for precise collision testing
pub struct NarrowPhaseDetector;

impl NarrowPhaseDetector {
    pub fn new() -> Self {
        Self
    }

    /// Test collision between two shapes and return contact manifold  
    pub fn test_collision(
        &self,
        shape_a: &crate::physics::core::shapes::ShapeType,
        pos_a: Vec3,
        shape_b: &crate::physics::core::shapes::ShapeType,
        pos_b: Vec3,
        body_a: usize,
        body_b: usize,
    ) -> Option<CollisionManifold> {
        use crate::physics::core::shapes::ShapeType;
        
        match (shape_a, shape_b) {
            (ShapeType::Sphere(sphere_a), ShapeType::Sphere(sphere_b)) => {
                self.sphere_sphere_collision(sphere_a, pos_a, sphere_b, pos_b, body_a, body_b)
            }
            (ShapeType::Box(box_a), ShapeType::Box(box_b)) => {
                self.box_box_collision(box_a, pos_a, box_b, pos_b, body_a, body_b)
            }
            (ShapeType::Sphere(sphere), ShapeType::Box(box_shape)) => {
                self.sphere_box_collision(sphere, pos_a, box_shape, pos_b, body_a, body_b)
            }
            (ShapeType::Box(box_shape), ShapeType::Sphere(sphere)) => {
                self.sphere_box_collision(sphere, pos_b, box_shape, pos_a, body_b, body_a)
                    .map(|mut manifold| {
                        // Flip contact normals since we swapped the bodies
                        for contact in &mut manifold.contacts {
                            contact.normal = -contact.normal;
                            let temp = contact.body_a;
                            contact.body_a = contact.body_b;
                            contact.body_b = temp;
                        }
                        manifold
                    })
            }
            _ => None, // Unsupported shape combinations
        }
    }

    /// Sphere-sphere collision detection
    fn sphere_sphere_collision(
        &self,
        sphere_a: &Sphere,
        pos_a: Vec3,
        sphere_b: &Sphere,
        pos_b: Vec3,
        body_a: usize,
        body_b: usize,
    ) -> Option<CollisionManifold> {
        let distance_vec = pos_b - pos_a;
        let distance = distance_vec.length();
        let sum_radii = sphere_a.radius() + sphere_b.radius();

        if distance < sum_radii {
            let mut manifold = CollisionManifold::new();
            
            let normal = if distance > f32::EPSILON {
                distance_vec / distance
            } else {
                Vec3::Y // Default separation direction
            };

            let contact_point = pos_a + normal * sphere_a.radius();
            let penetration = sum_radii - distance;

            manifold.add_contact(ContactPoint {
                position: contact_point,
                normal,
                depth: penetration,
                body_a,
                body_b,
            });

            Some(manifold)
        } else {
            None
        }
    }

    /// Box-box collision detection (simplified SAT - Separating Axes Theorem)
    fn box_box_collision(
        &self,
        _box_a: &BoxShape,
        _pos_a: Vec3,
        _box_b: &BoxShape,
        _pos_b: Vec3,
        _body_a: usize,
        _body_b: usize,
    ) -> Option<CollisionManifold> {
        // TODO: Implement proper SAT algorithm for oriented boxes
        // For now, return None to avoid incorrect collision detection
        None
    }

    /// Sphere-box collision detection
    fn sphere_box_collision(
        &self,
        _sphere: &Sphere,
        _sphere_pos: Vec3,
        _box_shape: &BoxShape,
        _box_pos: Vec3,
        _sphere_body: usize,
        _box_body: usize,
    ) -> Option<CollisionManifold> {
        // TODO: Implement sphere-box collision detection
        // This requires finding the closest point on the box to the sphere center
        None
    }


}

/// Spatial partitioning for optimized broad phase collision detection
pub struct SpatialGrid {
    cell_size: f32,
    grid: std::collections::HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SpatialGrid {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            grid: std::collections::HashMap::new(),
        }
    }

    /// Insert a body into the spatial grid
    pub fn insert(&mut self, body_id: usize, bbox: &BoundingBox) {
        let min_cell = self.world_to_cell(bbox.min);
        let max_cell = self.world_to_cell(bbox.max);

        for x in min_cell.0..=max_cell.0 {
            for y in min_cell.1..=max_cell.1 {
                for z in min_cell.2..=max_cell.2 {
                    self.grid
                        .entry((x, y, z))
                        .or_insert_with(Vec::new)
                        .push(body_id);
                }
            }
        }
    }

    /// Clear the spatial grid
    pub fn clear(&mut self) {
        self.grid.clear();
    }

    /// Find potential collision pairs using spatial partitioning
    pub fn find_potential_pairs(&self) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        let mut seen_pairs = std::collections::HashSet::new();

        for cell_bodies in self.grid.values() {
            for i in 0..cell_bodies.len() {
                for j in (i + 1)..cell_bodies.len() {
                    let pair = if cell_bodies[i] < cell_bodies[j] {
                        (cell_bodies[i], cell_bodies[j])
                    } else {
                        (cell_bodies[j], cell_bodies[i])
                    };

                    if seen_pairs.insert(pair) {
                        pairs.push(pair);
                    }
                }
            }
        }

        pairs
    }

    fn world_to_cell(&self, world_pos: Vec3) -> (i32, i32, i32) {
        (
            (world_pos.x / self.cell_size).floor() as i32,
            (world_pos.y / self.cell_size).floor() as i32,
            (world_pos.z / self.cell_size).floor() as i32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_sphere_collision() {
        let detector = NarrowPhaseDetector::new();
        let sphere_a = Sphere { radius: 1.0 };
        let sphere_b = Sphere { radius: 1.0 };

        let pos_a = Vec3::new(0.0, 0.0, 0.0);
        let pos_b = Vec3::new(1.5, 0.0, 0.0); // Overlapping

        let result = detector.sphere_sphere_collision(
            &sphere_a, pos_a, &sphere_b, pos_b, 0, 1
        );

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contacts.len(), 1);
        assert!(manifold.contacts[0].depth > 0.0);
    }

    #[test]
    fn test_spatial_grid() {
        let mut grid = SpatialGrid::new(10.0);
        
        let bbox_a = BoundingBox {
            min: Vec3::new(-1.0, -1.0, -1.0),
            max: Vec3::new(1.0, 1.0, 1.0),
        };
        
        grid.insert(0, &bbox_a);
        grid.insert(1, &bbox_a); // Same cell

        let pairs = grid.find_potential_pairs();
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (0, 1));
    }
}