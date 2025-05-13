use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A ball-socket (spherical) constraint that allows rotation in all directions
#[derive(Debug)]
pub struct BallSocketConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The anchor point on the first body (in local space)
    anchor_a: Vector3,
    
    /// The anchor point on the second body (in local space)
    anchor_b: Vector3,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The accumulated impulses (x, y, z)
    impulses: [f32; 3],
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// The constraint damping factor
    damping: f32,
    
    /// The softness of the constraint (0 = hard, > 0 = soft)
    softness: f32,
}

impl BallSocketConstraint {
    /// Creates a new ball-socket constraint
    pub fn new(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            bodies: [body_a, body_b],
            impulses: [0.0; 3],
            enabled: true,
            damping: 0.0,
            softness: 0.0,
        }
    }
    
    /// Creates a new soft ball-socket constraint with damping
    pub fn new_soft(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
        softness: f32,
        damping: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            bodies: [body_a, body_b],
            impulses: [0.0; 3],
            enabled: true,
            damping: damping.max(0.0),
            softness: softness.max(0.0),
        }
    }
    
    /// Returns whether the constraint is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the constraint is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Returns the damping factor
    pub fn get_damping(&self) -> f32 {
        self.damping
    }
    
    /// Sets the damping factor
    pub fn set_damping(&mut self, damping: f32) {
        self.damping = damping.max(0.0);
    }
    
    /// Returns the softness
    pub fn get_softness(&self) -> f32 {
        self.softness
    }
    
    /// Sets the softness
    pub fn set_softness(&mut self, softness: f32) {
        self.softness = softness.max(0.0);
    }
}

impl Constraint for BallSocketConstraint {
    fn constraint_type(&self) -> &'static str {
        "BallSocket"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, _bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulses
        self.impulses = [0.0; 3];
    }
    
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Store handles for later use
        let body_a_handle = self.body_a;
        let body_b_handle = self.body_b;

        // Get all data we need using immutable borrows first
        let (
            anchor_a_world,
            anchor_b_world,
            pos_a,
            pos_b,
            vel_a,
            vel_b,
            omega_a,
            omega_b,
            inv_mass_a,
            inv_mass_b,
            inv_inertia_a,
            inv_inertia_b
        );

        {
            // Get body A with immutable borrow
            let body_a = match bodies.get_body(body_a_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get body B with immutable borrow
            let body_b = match bodies.get_body(body_b_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get world position of anchor points
            anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);

            // Get the position of the bodies
            pos_a = body_a.get_position();
            pos_b = body_b.get_position();

            // Get the velocities
            vel_a = body_a.get_linear_velocity();
            vel_b = body_b.get_linear_velocity();
            omega_a = body_a.get_angular_velocity();
            omega_b = body_b.get_angular_velocity();

            // Get inverse mass and inertia
            inv_mass_a = body_a.get_inverse_mass();
            inv_mass_b = body_b.get_inverse_mass();
            inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            inv_inertia_b = body_b.get_inverse_inertia_tensor_world();
        }

        // Calculate the relative anchor positions
        let ra = anchor_a_world - pos_a;
        let rb = anchor_b_world - pos_b;

        // Calculate the velocity of the anchor points
        let vel_a_point = vel_a + omega_a.cross(&ra);
        let vel_b_point = vel_b + omega_b.cross(&rb);

        // Relative velocity at the constraint point
        let rel_vel = vel_b_point - vel_a_point;

        // If using soft constraint, apply spring force - special handling with separate borrows
        if self.softness > 0.0 {
            // Calculate distance error
            let distance = anchor_b_world - anchor_a_world;

            // Calculate spring force
            let spring_force = distance * self.softness;

            // Apply damping
            let damping_force = if self.damping > 0.0 {
                rel_vel * self.damping
            } else {
                Vector3::zero()
            };

            // Total force
            let total_force = (spring_force - damping_force) * dt;

            // Apply forces to body A
            if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                body_a.apply_force_at_point(total_force, anchor_a_world);
            }

            // Apply forces to body B
            if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                body_b.apply_force_at_point(-total_force, anchor_b_world);
            }

            return;
        }

        // Calculate the effective mass matrix
        let mut k = Matrix3x3::zero();

        // Contribution from inverse mass
        k.m[0][0] += inv_mass_a + inv_mass_b;
        k.m[1][1] += inv_mass_a + inv_mass_b;
        k.m[2][2] += inv_mass_a + inv_mass_b;

        // Contribution from inverse inertia for body A
        let ra_skew = SkewSymmetricMatrix::new(ra);
        let inertia_contribution_a = ra_skew.multiply_with_transpose(
            &ra_skew.multiply_matrix(&inv_inertia_a)
        );

        k.m[0][0] += inertia_contribution_a.m[0][0];
        k.m[0][1] += inertia_contribution_a.m[0][1];
        k.m[0][2] += inertia_contribution_a.m[0][2];
        k.m[1][0] += inertia_contribution_a.m[1][0];
        k.m[1][1] += inertia_contribution_a.m[1][1];
        k.m[1][2] += inertia_contribution_a.m[1][2];
        k.m[2][0] += inertia_contribution_a.m[2][0];
        k.m[2][1] += inertia_contribution_a.m[2][1];
        k.m[2][2] += inertia_contribution_a.m[2][2];

        // Contribution from inverse inertia for body B
        let rb_skew = SkewSymmetricMatrix::new(rb);
        let inertia_contribution_b = rb_skew.multiply_with_transpose(
            &rb_skew.multiply_matrix(&inv_inertia_b)
        );

        k.m[0][0] += inertia_contribution_b.m[0][0];
        k.m[0][1] += inertia_contribution_b.m[0][1];
        k.m[0][2] += inertia_contribution_b.m[0][2];
        k.m[1][0] += inertia_contribution_b.m[1][0];
        k.m[1][1] += inertia_contribution_b.m[1][1];
        k.m[1][2] += inertia_contribution_b.m[1][2];
        k.m[2][0] += inertia_contribution_b.m[2][0];
        k.m[2][1] += inertia_contribution_b.m[2][1];
        k.m[2][2] += inertia_contribution_b.m[2][2];

        // Invert the effective mass matrix
        let k_inv = k.inverse();

        // Calculate the constraint impulse
        let bias = Vector3::zero(); // We'll handle position correction in solve_position
        let lambda = k_inv.multiply_vector(-(rel_vel + bias));

        // Accumulate impulses
        let old_impulses = Vector3::new(self.impulses[0], self.impulses[1], self.impulses[2]);
        let new_impulses = old_impulses + lambda;

        self.impulses[0] = new_impulses.x;
        self.impulses[1] = new_impulses.y;
        self.impulses[2] = new_impulses.z;

        // Calculate new velocities
        let new_vel_a = vel_a - lambda * inv_mass_a;
        let new_omega_a = omega_a - inv_inertia_a.multiply_vector(ra.cross(&lambda));
        let new_vel_b = vel_b + lambda * inv_mass_b;
        let new_omega_b = omega_b + inv_inertia_b.multiply_vector(rb.cross(&lambda));

        // Apply new velocities to body A with a separate mutable borrow
        if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
            body_a.set_linear_velocity(new_vel_a);
            body_a.set_angular_velocity(new_omega_a);
        }

        // Apply new velocities to body B with a separate mutable borrow
        if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
            body_b.set_linear_velocity(new_vel_b);
            body_b.set_angular_velocity(new_omega_b);
        }
    }
    
    fn solve_position(&mut self, _dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled || self.softness > 0.0 {
            return; // Skip position solving for soft constraints
        }

        // Get body handles
        let body_a_handle = self.body_a;
        let body_b_handle = self.body_b;

        // Get properties from bodies with immutable borrows first
        let (
            anchor_a_world,
            anchor_b_world,
            inv_mass_a,
            inv_mass_b,
            transform_a,
            transform_b
        );

        {
            let body_a = match bodies.get_body(body_a_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(body_b_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get world position of anchor points
            anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);

            // Get inverse masses
            inv_mass_a = body_a.get_inverse_mass();
            inv_mass_b = body_b.get_inverse_mass();

            // Get transforms for later modification
            transform_a = body_a.get_transform();
            transform_b = body_b.get_transform();
        }

        // Calculate position error
        let error = anchor_b_world - anchor_a_world;

        // Calculate correction factor
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop

        let correction = (error.length() - slop).max(0.0) * beta;

        if correction < crate::math::EPSILON {
            return;
        }

        let correction_dir = error.normalize();

        // Calculate position correction impulse
        let mass_sum = inv_mass_a + inv_mass_b;
        if mass_sum <= crate::math::EPSILON {
            return;
        }

        let pos_impulse = correction_dir * correction / mass_sum;
        
        // Create new transforms with position corrections
        let mut new_transform_a = transform_a;
        let mut new_transform_b = transform_b;

        new_transform_a.position -= pos_impulse * inv_mass_a;
        new_transform_b.position += pos_impulse * inv_mass_b;

        // Now apply the transforms one at a time
        if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
            body_a.set_transform(new_transform_a);
        }

        if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
            body_b.set_transform(new_transform_b);
        }
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_constraint(&self) -> Box<dyn Constraint> {
        Box::new(Self {
            body_a: self.body_a,
            body_b: self.body_b,
            anchor_a: self.anchor_a,
            anchor_b: self.anchor_b,
            bodies: self.bodies,
            impulses: [0.0; 3], // Reset impulses on clone
            enabled: self.enabled,
            damping: self.damping,
            softness: self.softness,
        })
    }
}

/// A 3x3 matrix for constraint solving
#[derive(Debug, Clone, Copy)]
struct Matrix3x3 {
    m: [[f32; 3]; 3],
}

impl Matrix3x3 {
    /// Creates a new zero matrix
    fn zero() -> Self {
        Self {
            m: [[0.0; 3]; 3],
        }
    }
    
    /// Multiplies the matrix by a vector
    fn multiply_vector(&self, v: Vector3) -> Vector3 {
        Vector3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }
    
    /// Computes the inverse of the matrix
    fn inverse(&self) -> Self {
        // Calculate determinant
        let det = 
            self.m[0][0] * (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) -
            self.m[0][1] * (self.m[1][0] * self.m[2][2] - self.m[1][2] * self.m[2][0]) +
            self.m[0][2] * (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]);
        
        if det.abs() < crate::math::EPSILON {
            return Self::zero();
        }
        
        let inv_det = 1.0 / det;
        
        // Calculate adjugate matrix and multiply by 1/det
        Self {
            m: [
                [
                    (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) * inv_det,
                    (self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2]) * inv_det,
                    (self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1]) * inv_det,
                ],
                [
                    (self.m[1][2] * self.m[2][0] - self.m[1][0] * self.m[2][2]) * inv_det,
                    (self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0]) * inv_det,
                    (self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2]) * inv_det,
                ],
                [
                    (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]) * inv_det,
                    (self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1]) * inv_det,
                    (self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]) * inv_det,
                ],
            ],
        }
    }
}

/// A skew-symmetric matrix for cross product operations
#[derive(Debug, Clone, Copy)]
struct SkewSymmetricMatrix {
    v: Vector3,
}

impl SkewSymmetricMatrix {
    /// Creates a new skew-symmetric matrix from a vector
    fn new(v: Vector3) -> Self {
        Self { v }
    }
    
    /// Multiplies the skew-symmetric matrix by another matrix
    fn multiply_matrix(&self, matrix: &crate::math::Matrix3) -> Matrix3x3 {
        let mut result = Matrix3x3::zero();
        
        // [  0, -z,  y]   [a, b, c]
        // [  z,  0, -x] * [d, e, f]
        // [ -y,  x,  0]   [g, h, i]
        
        let x = self.v.x;
        let y = self.v.y;
        let z = self.v.z;
        
        // First row
        result.m[0][0] = -z * matrix.data[1][0] + y * matrix.data[2][0];
        result.m[0][1] = -z * matrix.data[1][1] + y * matrix.data[2][1];
        result.m[0][2] = -z * matrix.data[1][2] + y * matrix.data[2][2];
        
        // Second row
        result.m[1][0] = z * matrix.data[0][0] - x * matrix.data[2][0];
        result.m[1][1] = z * matrix.data[0][1] - x * matrix.data[2][1];
        result.m[1][2] = z * matrix.data[0][2] - x * matrix.data[2][2];
        
        // Third row
        result.m[2][0] = -y * matrix.data[0][0] + x * matrix.data[1][0];
        result.m[2][1] = -y * matrix.data[0][1] + x * matrix.data[1][1];
        result.m[2][2] = -y * matrix.data[0][2] + x * matrix.data[1][2];
        
        result
    }
    
    /// Multiplies the skew-symmetric matrix by another skew-symmetric matrix
    fn multiply_with_transpose(&self, other: &Matrix3x3) -> Matrix3x3 {
        let mut result = Matrix3x3::zero();
        
        // [  0, -z,  y]^T   [a, b, c]
        // [  z,  0, -x]   * [d, e, f]
        // [ -y,  x,  0]     [g, h, i]
        
        let x = self.v.x;
        let y = self.v.y;
        let z = self.v.z;
        
        // First row
        result.m[0][0] = -z * other.m[1][0] - y * other.m[2][0];
        result.m[0][1] = -z * other.m[1][1] - y * other.m[2][1];
        result.m[0][2] = -z * other.m[1][2] - y * other.m[2][2];
        
        // Second row
        result.m[1][0] = z * other.m[0][0] - x * other.m[2][0];
        result.m[1][1] = z * other.m[0][1] - x * other.m[2][1];
        result.m[1][2] = z * other.m[0][2] - x * other.m[2][2];
        
        // Third row
        result.m[2][0] = y * other.m[0][0] + x * other.m[1][0];
        result.m[2][1] = y * other.m[0][1] + x * other.m[1][1];
        result.m[2][2] = y * other.m[0][2] + x * other.m[1][2];
        
        result
    }
}