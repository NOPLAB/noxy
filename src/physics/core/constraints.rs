use glam::{Vec3, Mat3};

/// Contact constraint for collision response
#[derive(Debug, Clone)]
pub struct ContactConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub contact_point_a: Vec3, // Contact point relative to body A
    pub contact_point_b: Vec3, // Contact point relative to body B
    pub contact_normal: Vec3,
    pub penetration: f32,
    pub friction_coefficient: f32,
    pub restitution: f32,
    
    // Cached constraint values for stability
    pub normal_mass: f32,
    pub tangent_mass: [f32; 2],
    pub bias: f32,
    
    // Accumulated impulse for warm starting
    pub normal_impulse: f32,
    pub tangent_impulse: [f32; 2],
}

impl ContactConstraint {
    pub fn new(
        body_a: usize,
        body_b: usize,
        contact_point_a: Vec3,
        contact_point_b: Vec3,
        contact_normal: Vec3,
        penetration: f32,
        friction_coefficient: f32,
        restitution: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            contact_point_a,
            contact_point_b,
            contact_normal,
            penetration,
            friction_coefficient,
            restitution,
            normal_mass: 0.0,
            tangent_mass: [0.0, 0.0],
            bias: 0.0,
            normal_impulse: 0.0,
            tangent_impulse: [0.0, 0.0],
        }
    }

    /// Pre-compute constraint values for efficiency
    pub fn prepare(
        &mut self,
        mass_a: f32,
        mass_b: f32,
        inertia_a: Mat3,
        inertia_b: Mat3,
        dt: f32,
    ) {
        let inv_mass_a = if mass_a > 0.0 { 1.0 / mass_a } else { 0.0 };
        let inv_mass_b = if mass_b > 0.0 { 1.0 / mass_b } else { 0.0 };

        let inv_inertia_a = if mass_a > 0.0 { inertia_a.inverse() } else { Mat3::ZERO };
        let inv_inertia_b = if mass_b > 0.0 { inertia_b.inverse() } else { Mat3::ZERO };

        // Calculate effective mass for normal direction
        let ra_cross_n = self.contact_point_a.cross(self.contact_normal);
        let rb_cross_n = self.contact_point_b.cross(self.contact_normal);

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        let inv_inertia_contribution = 
            (inv_inertia_a * ra_cross_n).dot(ra_cross_n) +
            (inv_inertia_b * rb_cross_n).dot(rb_cross_n);

        self.normal_mass = 1.0 / (inv_mass_sum + inv_inertia_contribution);

        // Calculate tangent directions for friction
        let tangent1 = if self.contact_normal.x.abs() > 0.1 {
            Vec3::new(-self.contact_normal.y, self.contact_normal.x, 0.0).normalize()
        } else {
            Vec3::new(0.0, -self.contact_normal.z, self.contact_normal.y).normalize()
        };
        let tangent2 = self.contact_normal.cross(tangent1);

        // Calculate effective masses for tangent directions
        for (i, tangent) in [tangent1, tangent2].iter().enumerate() {
            let ra_cross_t = self.contact_point_a.cross(*tangent);
            let rb_cross_t = self.contact_point_b.cross(*tangent);

            let tangent_inv_inertia = 
                (inv_inertia_a * ra_cross_t).dot(ra_cross_t) +
                (inv_inertia_b * rb_cross_t).dot(rb_cross_t);

            self.tangent_mass[i] = 1.0 / (inv_mass_sum + tangent_inv_inertia);
        }

        // Calculate bias for positional correction (Baumgarte stabilization)
        let bias_factor = 0.2; // Baumgarte factor
        let slop = 0.01; // Penetration slop
        self.bias = (bias_factor / dt) * (self.penetration - slop).max(0.0);
    }

    /// Apply constraint impulse to resolve penetration and calculate relative velocity
    pub fn solve_velocity_constraint(
        &mut self,
        velocity_a: &mut Vec3,
        angular_velocity_a: &mut Vec3,
        velocity_b: &mut Vec3,
        angular_velocity_b: &mut Vec3,
        mass_a: f32,
        mass_b: f32,
        inertia_a: Mat3,
        inertia_b: Mat3,
    ) {
        let inv_mass_a = if mass_a > 0.0 { 1.0 / mass_a } else { 0.0 };
        let inv_mass_b = if mass_b > 0.0 { 1.0 / mass_b } else { 0.0 };
        let inv_inertia_a = if mass_a > 0.0 { inertia_a.inverse() } else { Mat3::ZERO };
        let inv_inertia_b = if mass_b > 0.0 { inertia_b.inverse() } else { Mat3::ZERO };

        // Calculate relative velocity at contact point
        let va = *velocity_a + angular_velocity_a.cross(self.contact_point_a);
        let vb = *velocity_b + angular_velocity_b.cross(self.contact_point_b);
        let relative_velocity = vb - va;

        // Solve normal constraint
        let jvn = relative_velocity.dot(self.contact_normal);
        let lambda = self.normal_mass * (-jvn + self.bias);

        // Clamp accumulated impulse (non-penetration constraint)
        let old_impulse = self.normal_impulse;
        self.normal_impulse = (self.normal_impulse + lambda).max(0.0);
        let delta_lambda = self.normal_impulse - old_impulse;

        // Apply normal impulse
        let impulse = delta_lambda * self.contact_normal;
        *velocity_a -= inv_mass_a * impulse;
        *angular_velocity_a -= inv_inertia_a * self.contact_point_a.cross(impulse);
        *velocity_b += inv_mass_b * impulse;
        *angular_velocity_b += inv_inertia_b * self.contact_point_b.cross(impulse);

        // Solve friction constraints
        if self.friction_coefficient > 0.0 {
            self.solve_friction_constraint(
                velocity_a,
                angular_velocity_a,
                velocity_b,
                angular_velocity_b,
                inv_mass_a,
                inv_mass_b,
                inv_inertia_a,
                inv_inertia_b,
            );
        }
    }

    fn solve_friction_constraint(
        &mut self,
        velocity_a: &mut Vec3,
        angular_velocity_a: &mut Vec3,
        velocity_b: &mut Vec3,
        angular_velocity_b: &mut Vec3,
        inv_mass_a: f32,
        inv_mass_b: f32,
        inv_inertia_a: Mat3,
        inv_inertia_b: Mat3,
    ) {
        // Calculate tangent directions
        let tangent1 = if self.contact_normal.x.abs() > 0.1 {
            Vec3::new(-self.contact_normal.y, self.contact_normal.x, 0.0).normalize()
        } else {
            Vec3::new(0.0, -self.contact_normal.z, self.contact_normal.y).normalize()
        };
        let tangent2 = self.contact_normal.cross(tangent1);
        let tangents = [tangent1, tangent2];

        // Maximum friction impulse (Coulomb friction)
        let max_friction = self.friction_coefficient * self.normal_impulse;

        for (i, &tangent) in tangents.iter().enumerate() {
            // Calculate relative velocity at contact point
            let va = *velocity_a + angular_velocity_a.cross(self.contact_point_a);
            let vb = *velocity_b + angular_velocity_b.cross(self.contact_point_b);
            let relative_velocity = vb - va;

            let jvt = relative_velocity.dot(tangent);
            let lambda = self.tangent_mass[i] * (-jvt);

            // Clamp friction impulse
            let old_impulse = self.tangent_impulse[i];
            self.tangent_impulse[i] = (old_impulse + lambda).clamp(-max_friction, max_friction);
            let delta_lambda = self.tangent_impulse[i] - old_impulse;

            // Apply friction impulse
            let impulse = delta_lambda * tangent;
            *velocity_a -= inv_mass_a * impulse;
            *angular_velocity_a -= inv_inertia_a * self.contact_point_a.cross(impulse);
            *velocity_b += inv_mass_b * impulse;
            *angular_velocity_b += inv_inertia_b * self.contact_point_b.cross(impulse);
        }
    }
}

/// Sequential Impulse solver for contact constraints
pub struct ConstraintSolver {
    constraints: Vec<ContactConstraint>,
    velocity_iterations: u32,
    position_iterations: u32,
}

impl ConstraintSolver {
    pub fn new() -> Self {
        Self {
            constraints: Vec::new(),
            velocity_iterations: 10,  // Number of velocity solver iterations
            position_iterations: 3,  // Number of position solver iterations
        }
    }

    pub fn clear_constraints(&mut self) {
        self.constraints.clear();
    }

    pub fn add_constraint(&mut self, constraint: ContactConstraint) {
        self.constraints.push(constraint);
    }

    /// Solve all constraints using Sequential Impulse method with safe borrowing
    pub fn solve_constraints(
        &mut self,
        positions: &mut [Vec3],
        velocities: &mut [Vec3],
        angular_velocities: &mut [Vec3],
        masses: &[f32],
        inertia_tensors: &[Mat3],
        dt: f32,
    ) {
        if self.constraints.is_empty() {
            return;
        }

        // Prepare all constraints
        for constraint in &mut self.constraints {
            if constraint.body_a < masses.len() && constraint.body_b < masses.len() {
                constraint.prepare(
                    masses[constraint.body_a],
                    masses[constraint.body_b],
                    inertia_tensors[constraint.body_a],
                    inertia_tensors[constraint.body_b],
                    dt,
                );
            }
        }

        // Solve velocity constraints using safe indexing
        for _ in 0..self.velocity_iterations {
            for i in 0..self.constraints.len() {
                let body_a = self.constraints[i].body_a;
                let body_b = self.constraints[i].body_b;
                
                if body_a < velocities.len() && body_b < velocities.len() && body_a != body_b {
                    // Extract constraint data to avoid borrowing issues
                    let mut constraint_copy = self.constraints[i].clone();
                    
                    // Safe indexing using split_at_mut to avoid borrowing conflicts
                    if body_a < body_b {
                        let (left_vel, right_vel) = velocities.split_at_mut(body_b);
                        let (left_ang, right_ang) = angular_velocities.split_at_mut(body_b);
                        
                        constraint_copy.solve_velocity_constraint(
                            &mut left_vel[body_a],
                            &mut left_ang[body_a],
                            &mut right_vel[0],
                            &mut right_ang[0],
                            masses[body_a],
                            masses[body_b],
                            inertia_tensors[body_a],
                            inertia_tensors[body_b],
                        );
                    } else {
                        let (left_vel, right_vel) = velocities.split_at_mut(body_a);
                        let (left_ang, right_ang) = angular_velocities.split_at_mut(body_a);
                        
                        constraint_copy.solve_velocity_constraint(
                            &mut right_vel[0],
                            &mut right_ang[0],
                            &mut left_vel[body_b],
                            &mut left_ang[body_b],
                            masses[body_a],
                            masses[body_b],
                            inertia_tensors[body_a],
                            inertia_tensors[body_b],
                        );
                    }
                    
                    // Copy back the constraint state (accumulated impulses)
                    self.constraints[i].normal_impulse = constraint_copy.normal_impulse;
                    self.constraints[i].tangent_impulse = constraint_copy.tangent_impulse;
                }
            }
        }

        // Solve position constraints with safe borrowing
        for _ in 0..self.position_iterations {
            for i in 0..self.constraints.len() {
                let constraint = &self.constraints[i];
                let body_a = constraint.body_a;
                let body_b = constraint.body_b;
                
                if body_a < positions.len() && body_b < positions.len() && body_a != body_b {
                    // Use safe indexing to avoid borrowing conflicts
                    if body_a < body_b {
                        let (left, right) = positions.split_at_mut(body_b);
                        Self::solve_position_constraint_static(
                            constraint,
                            &mut left[body_a],
                            &mut right[0],
                            masses[body_a],
                            masses[body_b],
                        );
                    } else {
                        let (left, right) = positions.split_at_mut(body_a);
                        Self::solve_position_constraint_static(
                            constraint,
                            &mut right[0],
                            &mut left[body_b],
                            masses[body_a],
                            masses[body_b],
                        );
                    }
                }
            }
        }
    }

    fn solve_position_constraint_static(
        constraint: &ContactConstraint,
        position_a: &mut Vec3,
        position_b: &mut Vec3,
        mass_a: f32,
        mass_b: f32,
    ) {
        // Simple position correction to prevent sinking
        if constraint.penetration > 0.01 {
            let inv_mass_a = if mass_a > 0.0 { 1.0 / mass_a } else { 0.0 };
            let inv_mass_b = if mass_b > 0.0 { 1.0 / mass_b } else { 0.0 };

            let correction_factor = 0.2; // How much to correct per iteration
            let total_inv_mass = inv_mass_a + inv_mass_b;

            if total_inv_mass > 0.0 {
                let correction = correction_factor * constraint.penetration * constraint.contact_normal / total_inv_mass;
                *position_a -= inv_mass_a * correction;
                *position_b += inv_mass_b * correction;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_contact_constraint_creation() {
        let constraint = ContactConstraint::new(
            0, 1,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::Y,
            0.1,
            0.5,
            0.8,
        );

        assert_eq!(constraint.body_a, 0);
        assert_eq!(constraint.body_b, 1);
        assert_eq!(constraint.penetration, 0.1);
        assert_eq!(constraint.friction_coefficient, 0.5);
        assert_eq!(constraint.restitution, 0.8);
    }

    #[test]
    fn test_constraint_solver() {
        let mut solver = ConstraintSolver::new();
        assert_eq!(solver.constraints.len(), 0);

        let constraint = ContactConstraint::new(
            0, 1,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::Y,
            0.1,
            0.5,
            0.8,
        );

        solver.add_constraint(constraint);
        assert_eq!(solver.constraints.len(), 1);

        solver.clear_constraints();
        assert_eq!(solver.constraints.len(), 0);
    }
}