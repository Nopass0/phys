use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::{Vector3, Matrix3, Quaternion, Rotation};
use std::any::Any;

/// A fixed constraint that rigidly connects two bodies, preventing all relative movement
#[derive(Debug)]
pub struct FixedConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The anchor point on the first body (in local space)
    local_anchor_a: Vector3,
    
    /// The anchor point on the second body (in local space)
    local_anchor_b: Vector3,
    
    /// The relative rotation from body A to body B (in local space)
    local_rotation: Quaternion,
    
    /// The linear impulses accumulated during the current step (x, y, z)
    linear_impulses: [f32; 3],
    
    /// The angular impulses accumulated during the current step (x, y, z)
    angular_impulses: [f32; 3],
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// The constraint damping factor
    damping: f32,
    
    /// The constraint breaking threshold (force required to break the constraint)
    breaking_threshold: f32,
}

impl FixedConstraint {
    /// Creates a new fixed constraint that locks bodies in their current relative positions
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> Self {
        Self {
            body_a,
            body_b,
            bodies: [body_a, body_b],
            // Default anchors to center of mass (will be set in initialize)
            local_anchor_a: Vector3::zero(),
            local_anchor_b: Vector3::zero(),
            // Default rotation to identity (will be set in initialize)
            local_rotation: Quaternion::identity(),
            linear_impulses: [0.0; 3],
            angular_impulses: [0.0; 3],
            enabled: true,
            damping: 0.3,
            breaking_threshold: f32::MAX, // No breaking by default
        }
    }
    
    /// Initializes the constraint with the current relative transform between the bodies
    pub fn initialize(&mut self, bodies: &BodyStorage<RigidBody>) {
        let body_a = match bodies.get_body(self.body_a) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        let body_b = match bodies.get_body(self.body_b) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        // Get world transforms
        let transform_a = body_a.get_transform();
        let transform_b = body_b.get_transform();
        
        // Store the local anchor points (center of mass of each body)
        self.local_anchor_a = Vector3::zero();
        self.local_anchor_b = Vector3::zero();
        
        // Store the relative rotation in local space
        let rot_a_inv = transform_a.rotation.conjugate();
        self.local_rotation = rot_a_inv * transform_b.rotation;
    }
    
    /// Creates a new fixed constraint with specified anchor points
    pub fn new_with_anchors(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            bodies: [body_a, body_b],
            local_anchor_a: anchor_a,
            local_anchor_b: anchor_b,
            local_rotation: Quaternion::identity(), // Will be set in initialize
            linear_impulses: [0.0; 3],
            angular_impulses: [0.0; 3],
            enabled: true,
            damping: 0.3,
            breaking_threshold: f32::MAX, // No breaking by default
        }
    }
    
    /// Sets the relative transform between the bodies explicitly
    pub fn set_relative_transform(&mut self, anchor_a: Vector3, anchor_b: Vector3, relative_rotation: Quaternion) {
        self.local_anchor_a = anchor_a;
        self.local_anchor_b = anchor_b;
        self.local_rotation = relative_rotation;
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
        self.damping = damping.max(0.0).min(1.0);
    }
    
    /// Returns the breaking threshold
    pub fn get_breaking_threshold(&self) -> f32 {
        self.breaking_threshold
    }
    
    /// Sets the breaking threshold (force required to break the constraint)
    pub fn set_breaking_threshold(&mut self, threshold: f32) {
        self.breaking_threshold = threshold.max(0.0);
    }
    
    /// Checks if the constraint force exceeds the breaking threshold
    fn is_broken(&self) -> bool {
        if self.breaking_threshold >= f32::MAX {
            return false;
        }
        
        // Calculate total impulse magnitude
        let linear_impulse = Vector3::new(
            self.linear_impulses[0],
            self.linear_impulses[1],
            self.linear_impulses[2]
        ).length();
        
        let angular_impulse = Vector3::new(
            self.angular_impulses[0],
            self.angular_impulses[1], 
            self.angular_impulses[2]
        ).length();
        
        // Check if either exceeds threshold
        linear_impulse > self.breaking_threshold || angular_impulse > self.breaking_threshold
    }
}

impl Constraint for FixedConstraint {
    fn constraint_type(&self) -> &'static str {
        "Fixed"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, _bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulses
        self.linear_impulses = [0.0; 3];
        self.angular_impulses = [0.0; 3];
    }
    
    fn solve_velocity(&mut self, _dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }
        
        // Get the bodies (immutable access first)
        let (body_a_data, body_b_data) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };
            
            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };
            
            // Get all necessary data from both bodies
            let transform_a = body_a.get_transform();
            let transform_b = body_b.get_transform();
            
            // Get anchor points in world space
            let anchor_a_world = transform_a.transform_point(self.local_anchor_a);
            let anchor_b_world = transform_b.transform_point(self.local_anchor_b);
            
            // Calculate relative positions
            let ra = anchor_a_world - body_a.get_position();
            let rb = anchor_b_world - body_b.get_position();
            
            // Get velocities
            let vel_a = body_a.get_linear_velocity();
            let vel_b = body_b.get_linear_velocity();
            let omega_a = body_a.get_angular_velocity();
            let omega_b = body_b.get_angular_velocity();
            
            // Get inverse mass and inertia
            let inv_mass_a = body_a.get_inverse_mass();
            let inv_mass_b = body_b.get_inverse_mass();
            
            let inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            let inv_inertia_b = body_b.get_inverse_inertia_tensor_world();
            
            // Calculate expected rotation for body B
            let expected_rot_b = transform_a.rotation * self.local_rotation;
            
            // Calculate the angular error
            let rot_error = expected_rot_b.conjugate() * transform_b.rotation;
            let (axis_error, angle_error) = rot_error.get_axis_angle();
            
            // Get the axis of correction in world space
            let axis_world = transform_a.rotation.rotate_vector(axis_error);
            
            // Collect all data needed for velocity solving
            (
                (
                    vel_a, 
                    omega_a, 
                    ra, 
                    inv_mass_a, 
                    inv_inertia_a,
                    transform_a,
                ),
                (
                    vel_b, 
                    omega_b, 
                    rb, 
                    inv_mass_b, 
                    inv_inertia_b,
                    transform_b,
                    axis_world,
                    angle_error,
                )
            )
        };
        
        // Unpack body data
        let (vel_a, omega_a, ra, inv_mass_a, inv_inertia_a, _transform_a) = body_a_data;
        let (vel_b, omega_b, rb, inv_mass_b, inv_inertia_b, _transform_b, axis_world, angle_error) = body_b_data;
        
        // Calculate point velocities
        let vel_a_point = vel_a + omega_a.cross(&ra);
        let vel_b_point = vel_b + omega_b.cross(&rb);
        
        // Relative linear velocity
        let rel_linear_vel = vel_b_point - vel_a_point;
        
        // Store all changes to be applied after computation
        let mut new_vel_a = vel_a;
        let mut new_omega_a = omega_a;
        let mut new_vel_b = vel_b;
        let mut new_omega_b = omega_b;
        
        // Solve linear constraints (position match)
        for i in 0..3 {
            // Create basis vector for this constraint
            let mut basis = Vector3::zero();
            if i == 0 { basis.x = 1.0; }
            else if i == 1 { basis.y = 1.0; }
            else { basis.z = 1.0; }
            
            // Calculate jacobian for this constraint
            let j_lin_a = -basis;
            let j_ang_a = -ra.cross(&basis);
            let j_lin_b = basis;
            let j_ang_b = rb.cross(&basis);
            
            // Calculate relative velocity along constraint
            let rel_vel = rel_linear_vel.dot(&basis);
            
            // Apply damping
            let rel_vel_with_damping = rel_vel * (1.0 - self.damping);
            
            // Calculate effective mass
            let k = inv_mass_a * j_lin_a.dot(&j_lin_a) +
                    inv_mass_b * j_lin_b.dot(&j_lin_b) +
                    j_ang_a.dot(&inv_inertia_a.multiply_vector(j_ang_a)) +
                    j_ang_b.dot(&inv_inertia_b.multiply_vector(j_ang_b));
                    
            if k <= crate::math::EPSILON {
                continue;
            }
            
            let eff_mass = 1.0 / k;
            
            // Calculate lambda (impulse magnitude)
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(rel_vel_with_damping + bias) * eff_mass;
            
            // Accumulate impulses
            self.linear_impulses[i] += lambda;
            
            // Apply impulses
            let p = basis * lambda;
            
            // Update velocities (stored for later application)
            new_vel_a = new_vel_a - p * inv_mass_a;
            new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(j_ang_a * lambda);
            
            new_vel_b = new_vel_b + p * inv_mass_b;
            new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(j_ang_b * lambda);
        }
        
        // Skip angular constraints if there's no angular error
        if angle_error >= crate::math::EPSILON {
            // Solve angular constraints (orientation match)
            for i in 0..3 {
                // Create basis vector for this constraint
                let mut basis = Vector3::zero();
                if i == 0 { basis.x = 1.0; }
                else if i == 1 { basis.y = 1.0; }
                else { basis.z = 1.0; }
                
                // Project axis error onto this basis
                let axis_error_comp = axis_world.dot(&basis);
                if axis_error_comp.abs() < crate::math::EPSILON {
                    continue;
                }
                
                // Calculate jacobian for this constraint
                let j_ang_a = -basis;
                let j_ang_b = basis;
                
                // Calculate relative angular velocity
                let rel_ang_vel = (omega_b - omega_a).dot(&basis);
                
                // Apply damping
                let rel_ang_vel_with_damping = rel_ang_vel * (1.0 - self.damping);
                
                // Calculate effective mass
                let k = j_ang_a.dot(&inv_inertia_a.multiply_vector(j_ang_a)) +
                        j_ang_b.dot(&inv_inertia_b.multiply_vector(j_ang_b));
                        
                if k <= crate::math::EPSILON {
                    continue;
                }
                
                let eff_mass = 1.0 / k;
                
                // Calculate lambda (impulse magnitude)
                let bias = 0.0; // We'll handle position correction in solve_position
                let lambda = -(rel_ang_vel_with_damping + bias) * eff_mass;
                
                // Accumulate impulses
                self.angular_impulses[i] += lambda;
                
                // Update velocities (stored for later application)
                new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(j_ang_a * lambda);
                new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(j_ang_b * lambda);
            }
        }
        
        // Now apply all velocity changes
        {
            // Apply to body A
            if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
                body_a.set_linear_velocity(new_vel_a);
                body_a.set_angular_velocity(new_omega_a);
            }
            
            // Apply to body B
            if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
                body_b.set_linear_velocity(new_vel_b);
                body_b.set_angular_velocity(new_omega_b);
            }
        }
        
        // Check if the constraint should break
        if self.is_broken() {
            self.enabled = false;
        }
    }
    
    fn solve_position(&mut self, _dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }
        
        // Get the bodies data (immutable access first)
        let (transform_a, inv_mass_a, inv_inertia_a, transform_b, inv_mass_b, inv_inertia_b) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };
            
            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };
            
            // Get transforms and inverse masses/inertias
            (
                body_a.get_transform(),
                body_a.get_inverse_mass(),
                body_a.get_inverse_inertia_tensor_world(),
                body_b.get_transform(),
                body_b.get_inverse_mass(),
                body_b.get_inverse_inertia_tensor_world()
            )
        };
        
        // Get anchor points in world space
        let anchor_a_world = transform_a.transform_point(self.local_anchor_a);
        let anchor_b_world = transform_b.transform_point(self.local_anchor_b);
        
        // Calculate position error
        let position_error = anchor_b_world - anchor_a_world;
        
        // Calculate correction parameters
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop
        
        // Calculate position correction
        let mut new_transform_a = transform_a;
        let mut new_transform_b = transform_b;
        let mut position_correction_applied = false;
        
        // Apply position correction
        let correction_magnitude = (position_error.length() - slop).max(0.0) * beta;
        
        if correction_magnitude > crate::math::EPSILON {
            let correction_dir = position_error.normalize();
            let position_impulse = correction_dir * correction_magnitude;
            
            let mass_sum = inv_mass_a + inv_mass_b;
            if mass_sum > crate::math::EPSILON {
                new_transform_a.position -= position_impulse * inv_mass_a / mass_sum;
                new_transform_b.position += position_impulse * inv_mass_b / mass_sum;
                position_correction_applied = true;
            }
        }
        
        // Calculate target rotation for body B
        let expected_rot_b = transform_a.rotation * self.local_rotation;
        
        // Calculate the angular error
        let rot_error = expected_rot_b.conjugate() * transform_b.rotation;
        let (axis, angle) = rot_error.get_axis_angle();
        let mut rotation_correction_applied = false;

        if angle > crate::math::EPSILON {
            // Apply orientation correction
            let correction_angle = angle * beta;

            // Convert to world space
            let axis_world = transform_b.rotation.rotate_vector(axis);

            // Calculate angular impulse distribution based on inertia
            let inertia_a = inv_inertia_a.multiply_vector(axis_world);
            let inertia_b = inv_inertia_b.multiply_vector(axis_world);
            let inertia_sum = inertia_a.length_squared() + inertia_b.length_squared();

            if inertia_sum > crate::math::EPSILON {
                let ratio_a = inertia_a.length_squared() / inertia_sum;
                let ratio_b = inertia_b.length_squared() / inertia_sum;

                // Create quaternions for the rotations
                let rot_a = Quaternion::from_axis_angle(axis_world, -correction_angle * ratio_a);
                let rot_b = Quaternion::from_axis_angle(axis_world, correction_angle * ratio_b);
                
                new_transform_a.rotation = rot_a * new_transform_a.rotation;
                new_transform_b.rotation = rot_b * new_transform_b.rotation;
                rotation_correction_applied = true;
            }
        }
        
        // Apply all transform changes
        if position_correction_applied || rotation_correction_applied {
            // Apply to body A
            if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
                body_a.set_transform(new_transform_a);
            }
            
            // Apply to body B
            if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
                body_b.set_transform(new_transform_b);
            }
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
            bodies: self.bodies,
            local_anchor_a: self.local_anchor_a,
            local_anchor_b: self.local_anchor_b,
            local_rotation: self.local_rotation,
            linear_impulses: [0.0; 3], // Reset impulses on clone
            angular_impulses: [0.0; 3], // Reset impulses on clone
            enabled: self.enabled,
            damping: self.damping,
            breaking_threshold: self.breaking_threshold,
        })
    }
}