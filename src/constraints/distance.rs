use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A distance constraint maintains a fixed distance between two points on different bodies
#[derive(Debug)]
pub struct DistanceConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The anchor point on the first body (in local space)
    anchor_a: Vector3,
    
    /// The anchor point on the second body (in local space)
    anchor_b: Vector3,
    
    /// The desired distance between the anchor points
    distance: f32,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The impulse accumulated during the current step
    impulse: f32,
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// The minimum distance allowed (if negative, ignored)
    min_distance: f32,
    
    /// The maximum distance allowed (if negative, ignored)
    max_distance: f32,
    
    /// The spring stiffness (0 for rigid constraint)
    stiffness: f32,
    
    /// The damping factor
    damping: f32,
}

impl DistanceConstraint {
    /// Creates a new distance constraint
    pub fn new(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
        distance: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            distance: distance.max(0.0),
            bodies: [body_a, body_b],
            impulse: 0.0,
            enabled: true,
            min_distance: -1.0,
            max_distance: -1.0,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
    
    /// Creates a spring constraint (soft distance constraint)
    pub fn new_spring(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
        distance: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            distance: distance.max(0.0),
            bodies: [body_a, body_b],
            impulse: 0.0,
            enabled: true,
            min_distance: -1.0,
            max_distance: -1.0,
            stiffness: stiffness.max(0.0),
            damping: damping.max(0.0),
        }
    }
    
    /// Creates a rope constraint (maximum distance constraint)
    pub fn new_rope(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
        max_distance: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            distance: 0.0, // Not used for ropes
            bodies: [body_a, body_b],
            impulse: 0.0,
            enabled: true,
            min_distance: -1.0,
            max_distance: max_distance.max(0.0),
            stiffness: 0.0,
            damping: 0.0,
        }
    }
    
    /// Returns the desired distance
    pub fn get_distance(&self) -> f32 {
        self.distance
    }
    
    /// Sets the desired distance
    pub fn set_distance(&mut self, distance: f32) {
        self.distance = distance.max(0.0);
    }
    
    /// Returns the minimum distance (if enabled)
    pub fn get_min_distance(&self) -> Option<f32> {
        if self.min_distance >= 0.0 {
            Some(self.min_distance)
        } else {
            None
        }
    }
    
    /// Sets the minimum distance
    pub fn set_min_distance(&mut self, min_distance: Option<f32>) {
        self.min_distance = min_distance.unwrap_or(-1.0);
    }
    
    /// Returns the maximum distance (if enabled)
    pub fn get_max_distance(&self) -> Option<f32> {
        if self.max_distance >= 0.0 {
            Some(self.max_distance)
        } else {
            None
        }
    }
    
    /// Sets the maximum distance
    pub fn set_max_distance(&mut self, max_distance: Option<f32>) {
        self.max_distance = max_distance.unwrap_or(-1.0);
    }
    
    /// Returns the spring stiffness
    pub fn get_stiffness(&self) -> f32 {
        self.stiffness
    }
    
    /// Sets the spring stiffness
    pub fn set_stiffness(&mut self, stiffness: f32) {
        self.stiffness = stiffness.max(0.0);
    }
    
    /// Returns the damping factor
    pub fn get_damping(&self) -> f32 {
        self.damping
    }
    
    /// Sets the damping factor
    pub fn set_damping(&mut self, damping: f32) {
        self.damping = damping.max(0.0);
    }
    
    /// Returns whether the constraint is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the constraint is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
}

impl Constraint for DistanceConstraint {
    fn constraint_type(&self) -> &'static str {
        "Distance"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulse
        self.impulse = 0.0;
    }
    
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Get all the data we need with immutable borrows first
        let (anchor_a_world, anchor_b_world, ra, rb, va, vb, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get world position of anchor points
            let anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            let anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);

            // Get relative positions
            let ra = anchor_a_world - body_a.get_position();
            let rb = anchor_b_world - body_b.get_position();

            // Get velocities
            let va = body_a.get_linear_velocity() + body_a.get_angular_velocity().cross(&ra);
            let vb = body_b.get_linear_velocity() + body_b.get_angular_velocity().cross(&rb);

            // Get inverse masses and inertias
            let inv_mass_a = body_a.get_inverse_mass();
            let inv_mass_b = body_b.get_inverse_mass();

            let inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            let inv_inertia_b = body_b.get_inverse_inertia_tensor_world();

            // Return the tuple of collected data
            (
                anchor_a_world,
                anchor_b_world,
                ra,
                rb,
                va,
                vb,
                inv_mass_a,
                inv_mass_b,
                inv_inertia_a,
                inv_inertia_b
            )
        };

        // Vector from A to B
        let ab = anchor_b_world - anchor_a_world;
        let ab_length = ab.length();

        // Early out if bodies are at the same position
        if ab_length < crate::math::EPSILON {
            return;
        }

        // Normalized direction from A to B
        let n = ab / ab_length;

        // Relative velocity
        let rel_vel = vb - va;

        // Relative velocity along the constraint axis
        let rel_vel_along_n = rel_vel.dot(&n);

        // If using a spring, apply spring force
        if self.stiffness > 0.0 {
            // Calculate spring force
            let force = (ab_length - self.distance) * self.stiffness;

            // Apply damping
            let damping_force = rel_vel_along_n * self.damping;

            // Total force
            let total_force = (force - damping_force) * dt;

            // Apply forces to bodies - using separate mutable borrows
            if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
                body_a.apply_force_at_point(n * total_force, anchor_a_world);
            }

            if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
                body_b.apply_force_at_point(-n * total_force, anchor_b_world);
            }

            return;
        }

        // For a rigid constraint, we need to check if the distance constraint is violated
        let mut constraint_violated = false;

        // Check if we have a distance constraint
        if self.distance > 0.0 {
            constraint_violated = !crate::math::approx_eq(ab_length, self.distance);
        }

        // Check if we have a rope constraint
        if self.max_distance >= 0.0 && ab_length > self.max_distance {
            constraint_violated = true;
        }

        // Check if we have a rod constraint
        if self.min_distance >= 0.0 && ab_length < self.min_distance {
            constraint_violated = true;
        }

        if !constraint_violated {
            return;
        }

        // Compute effective mass
        let ra_cross_n = ra.cross(&n);
        let rb_cross_n = rb.cross(&n);

        let mass_term = inv_mass_a + inv_mass_b;
        let inertia_term_a = ra_cross_n.dot(&inv_inertia_a.multiply_vector(ra_cross_n));
        let inertia_term_b = rb_cross_n.dot(&inv_inertia_b.multiply_vector(rb_cross_n));

        let k = mass_term + inertia_term_a + inertia_term_b;

        if k <= crate::math::EPSILON {
            return;
        }

        let eff_mass = 1.0 / k;

        // Calculate error
        let bias = 0.0; // We'll handle position correction in solve_position

        // Calculate lambda
        let lambda = -(rel_vel_along_n + bias) * eff_mass;

        // Accumulate impulse
        let old_impulse = self.impulse;
        self.impulse += lambda;

        // Clamp impulse
        // For a distance constraint, we need to limit the impulse differently
        // based on whether the constraint is violated in the min or max direction
        let min_impulse: f32;
        let max_impulse: f32;

        if self.min_distance >= 0.0 && ab_length < self.min_distance {
            // Need to push bodies apart
            min_impulse = 0.0;
            max_impulse = f32::MAX;
        } else if self.max_distance >= 0.0 && ab_length > self.max_distance {
            // Need to pull bodies together
            min_impulse = -f32::MAX;
            max_impulse = 0.0;
        } else if ab_length < self.distance {
            // Need to push bodies apart
            min_impulse = 0.0;
            max_impulse = f32::MAX;
        } else {
            // Need to pull bodies together
            min_impulse = -f32::MAX;
            max_impulse = 0.0;
        }

        self.impulse = self.impulse.clamp(min_impulse, max_impulse);

        // Calculate the impulse to apply
        let impulse = (self.impulse - old_impulse) * n;

        // Calculate the new velocities
        let new_vel_a_linear = va - impulse * inv_mass_a;
        let new_vel_a_angular = va.cross(&ra) - inv_inertia_a.multiply_vector(ra.cross(&impulse));

        let new_vel_b_linear = vb + impulse * inv_mass_b;
        let new_vel_b_angular = vb.cross(&rb) + inv_inertia_b.multiply_vector(rb.cross(&impulse));

        // Update velocities with separate mutable borrows
        if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
            body_a.set_linear_velocity(new_vel_a_linear);
            body_a.set_angular_velocity(new_vel_a_angular);
        }

        if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
            body_b.set_linear_velocity(new_vel_b_linear);
            body_b.set_angular_velocity(new_vel_b_angular);
        }
    }
    
    fn solve_position(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Skip position solving for springs
        if self.stiffness > 0.0 {
            return;
        }

        // Get all the data we need with immutable borrows first
        let (anchor_a_world, anchor_b_world, transform_a, transform_b, inv_mass_a, inv_mass_b) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get transforms
            let transform_a = body_a.get_transform();
            let transform_b = body_b.get_transform();

            // Get world position of anchor points
            let anchor_a_world = transform_a.transform_point(self.anchor_a);
            let anchor_b_world = transform_b.transform_point(self.anchor_b);

            // Get inverse masses
            let inv_mass_a = body_a.get_inverse_mass();
            let inv_mass_b = body_b.get_inverse_mass();

            (
                anchor_a_world,
                anchor_b_world,
                transform_a,
                transform_b,
                inv_mass_a,
                inv_mass_b
            )
        };

        // Vector from A to B
        let ab = anchor_b_world - anchor_a_world;
        let ab_length = ab.length();

        // Early out if bodies are at the same position
        if ab_length < crate::math::EPSILON {
            return;
        }

        // Normalized direction from A to B
        let n = ab / ab_length;

        // Calculate error
        let mut error = 0.0;

        // Calculate error based on constraint type
        if self.distance > 0.0 {
            error = ab_length - self.distance;
        }

        // Check rope constraint
        if self.max_distance >= 0.0 && ab_length > self.max_distance {
            error = ab_length - self.max_distance;
        }

        // Check rod constraint
        if self.min_distance >= 0.0 && ab_length < self.min_distance {
            error = ab_length - self.min_distance;
        }

        // Early out if there's no error
        if (error as f32).abs() < crate::math::EPSILON {
            return;
        }

        // Calculate correction factor
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop

        let correction = ((error as f32).abs() - slop).max(0.0) * beta * (error as f32).signum();

        // Compute effective mass
        let mass_sum = inv_mass_a + inv_mass_b;
        if mass_sum <= crate::math::EPSILON {
            return;
        }

        // Calculate position correction
        let pos_impulse = correction / mass_sum;
        let impulse = n * pos_impulse;

        // Calculate new transforms
        let mut new_transform_a = transform_a;
        let mut new_transform_b = transform_b;

        new_transform_a.position -= impulse * inv_mass_a;
        new_transform_b.position += impulse * inv_mass_b;

        // Apply position correction with separate mutable borrows
        if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
            body_a.set_transform(new_transform_a);
        }

        if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
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
            distance: self.distance,
            bodies: self.bodies,
            impulse: 0.0, // Reset impulse on clone
            enabled: self.enabled,
            min_distance: self.min_distance,
            max_distance: self.max_distance,
            stiffness: self.stiffness,
            damping: self.damping,
        })
    }
}