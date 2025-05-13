use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::{Vector3, Matrix3, Quaternion};
use std::any::Any;

/// A cone-twist constraint that allows rotation within a cone and around a twist axis
#[derive(Debug)]
pub struct ConeTwistConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The anchor point on the first body (in local space)
    anchor_a: Vector3,
    
    /// The anchor point on the second body (in local space)
    anchor_b: Vector3,
    
    /// The twist axis on the first body (in local space)
    axis_a: Vector3,
    
    /// The twist axis on the second body (in local space)
    axis_b: Vector3,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The accumulated impulses (3 for linear, 3 for angular)
    impulses: [f32; 6],
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// The swing span 1 (rotation limit around the first axis perpendicular to the twist axis)
    swing_span1: f32,
    
    /// The swing span 2 (rotation limit around the second axis perpendicular to the twist axis)
    swing_span2: f32,
    
    /// The twist span (rotation limit around the twist axis)
    twist_span: f32,
    
    /// The softness of the constraint (0 = hard, > 0 = soft)
    softness: f32,
    
    /// The bias factor for position correction
    bias_factor: f32,
    
    /// The relaxation factor for the constraint
    relaxation_factor: f32,
    
    /// The initial relative rotation between the bodies (used as reference)
    initial_orientation: Quaternion,
}

impl ConeTwistConstraint {
    /// Creates a new cone-twist constraint
    pub fn new(
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vector3,
        anchor_b: Vector3,
        axis_a: Vector3,
        axis_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            axis_a: axis_a.normalize(),
            axis_b: axis_b.normalize(),
            bodies: [body_a, body_b],
            impulses: [0.0; 6],
            enabled: true,
            swing_span1: std::f32::consts::FRAC_PI_4, // Default 45 degrees
            swing_span2: std::f32::consts::FRAC_PI_4, // Default 45 degrees
            twist_span: std::f32::consts::FRAC_PI_2,  // Default 90 degrees
            softness: 0.0,
            bias_factor: 0.3,
            relaxation_factor: 1.0,
            initial_orientation: Quaternion::identity(),
        }
    }
    
    /// Initializes the constraint based on the current orientation of the bodies
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
        
        // Calculate initial relative orientation
        let rot_a_inv = transform_a.rotation.conjugate();
        self.initial_orientation = rot_a_inv * transform_b.rotation;
    }
    
    /// Sets the rotation limits for the constraint
    pub fn set_limits(
        &mut self,
        swing_span1: f32,
        swing_span2: f32,
        twist_span: f32
    ) {
        self.swing_span1 = swing_span1.max(0.0);
        self.swing_span2 = swing_span2.max(0.0);
        self.twist_span = twist_span.max(0.0);
    }
    
    /// Sets the softness parameters for the constraint
    pub fn set_softness(
        &mut self,
        softness: f32,
        bias_factor: f32,
        relaxation_factor: f32
    ) {
        self.softness = softness.max(0.0);
        self.bias_factor = bias_factor.max(0.0);
        self.relaxation_factor = relaxation_factor.max(0.0);
    }
    
    /// Returns whether the constraint is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the constraint is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Gets the swing span 1 (rotation limit around the first axis perpendicular to the twist axis)
    pub fn get_swing_span1(&self) -> f32 {
        self.swing_span1
    }
    
    /// Gets the swing span 2 (rotation limit around the second axis perpendicular to the twist axis)
    pub fn get_swing_span2(&self) -> f32 {
        self.swing_span2
    }
    
    /// Gets the twist span (rotation limit around the twist axis)
    pub fn get_twist_span(&self) -> f32 {
        self.twist_span
    }
    
    /// Decomposes the relative rotation into swing and twist components
    fn get_swing_twist(&self, body_a: &RigidBody, body_b: &RigidBody) -> (Quaternion, Quaternion) {
        // Get world transforms
        let transform_a = body_a.get_transform();
        let transform_b = body_b.get_transform();
        
        // Calculate current relative rotation
        let rot_a_inv = transform_a.rotation.conjugate();
        let rel_rot = rot_a_inv * transform_b.rotation;
        
        // Normalize to account for numerical errors
        let rel_rot = rel_rot.normalize();
        
        // Get the twist axis in local space of body A
        let twist_axis = self.axis_a;
        
        // Project the quaternion onto the twist axis
        let proj = rel_rot.dot(&Quaternion::from_axis_angle(twist_axis, 0.0));
        let twist_angle = 2.0 * proj.acos();
        let twist_sign = if twist_axis.dot(&Vector3::new(rel_rot.x, rel_rot.y, rel_rot.z)) >= 0.0 { 1.0 } else { -1.0 };
        
        // Create the twist quaternion
        let twist = Quaternion::from_axis_angle(twist_axis, twist_angle * twist_sign);
        
        // Calculate the swing as the remaining rotation (rel_rot = swing * twist)
        let swing = rel_rot * twist.conjugate();
        
        (swing, twist)
    }
}

impl Constraint for ConeTwistConstraint {
    fn constraint_type(&self) -> &'static str {
        "ConeTwist"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, _bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulses
        self.impulses = [0.0; 6];
    }
    
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // We need to handle one body at a time to avoid borrow checker issues
        let body_a_handle = self.body_a;
        let body_b_handle = self.body_b;

        // Get properties from both bodies first
        let (
            pos_a, mut vel_a, mut omega_a, inv_mass_a, anchor_a_world, ra,
            pos_b, mut vel_b, mut omega_b, inv_mass_b, anchor_b_world, rb,
            twist_axis_world, swing, twist, inv_inertia_a, inv_inertia_b
        );

        {
            // First get body A
            let body_a = match bodies.get_body(body_a_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Then get body B
            let body_b = match bodies.get_body(body_b_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Calculate properties for body A
            pos_a = body_a.get_position();
            vel_a = body_a.get_linear_velocity();
            omega_a = body_a.get_angular_velocity();
            inv_mass_a = body_a.get_inverse_mass();
            inv_inertia_a = body_a.get_inverse_inertia_tensor_world();

            // Calculate anchor point in world space for body A
            anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            ra = anchor_a_world - pos_a;

            // Get twist axis in world space
            twist_axis_world = body_a.get_transform().transform_direction(self.axis_a);

            // Calculate properties for body B
            pos_b = body_b.get_position();
            vel_b = body_b.get_linear_velocity();
            omega_b = body_b.get_angular_velocity();
            inv_mass_b = body_b.get_inverse_mass();
            inv_inertia_b = body_b.get_inverse_inertia_tensor_world();

            // Calculate anchor point in world space for body B
            anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);
            rb = anchor_b_world - pos_b;

            // Calculate swing and twist components
            let swing_twist = self.get_swing_twist(body_a, body_b);
            swing = swing_twist.0;
            twist = swing_twist.1;
        }

        // Calculate velocity of the anchor points
        let vel_a_point = vel_a + omega_a.cross(&ra);
        let vel_b_point = vel_b + omega_b.cross(&rb);

        // Relative velocity at the constraint point
        let rel_vel = vel_b_point - vel_a_point;
        
        // Solve linear constraints (ball joint position match)
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
            
            // Calculate the constraint violation
            let rel_vel_along_constraint = rel_vel.dot(&basis);
            
            // Calculate effective mass
            let k = inv_mass_a * j_lin_a.dot(&j_lin_a) +
                    inv_mass_b * j_lin_b.dot(&j_lin_b) +
                    j_ang_a.dot(&inv_inertia_a.multiply_vector(j_ang_a)) +
                    j_ang_b.dot(&inv_inertia_b.multiply_vector(j_ang_b));
                    
            if k <= crate::math::EPSILON {
                continue;
            }
            
            let eff_mass = 1.0 / k;
            
            // Calculate lambda
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(rel_vel_along_constraint + bias) * eff_mass;
            
            // Accumulate impulse
            let _old_impulse = self.impulses[i];
            self.impulses[i] += lambda;
            
            // Calculate impulse to apply
            let p = basis * lambda;
            
            // Update velocities (we'll apply them at the end)
            vel_a = vel_a - p * inv_mass_a;
            omega_a = omega_a - inv_inertia_a.multiply_vector(j_ang_a * lambda);

            vel_b = vel_b + p * inv_mass_b;
            omega_b = omega_b + inv_inertia_b.multiply_vector(j_ang_b * lambda);
        }

        // We already have swing and twist components
        
        // Get the swing axis
        let swing_axis = if swing.w.abs() < 0.999 {
            let axis = Vector3::new(swing.x, swing.y, swing.z).normalize();
            let angle = 2.0 * swing.w.acos();
            axis * angle
        } else {
            Vector3::zero()
        };
        
        // Project swing axis onto plane perpendicular to twist axis
        let swing_axis_proj = swing_axis - twist_axis_world * swing_axis.dot(&twist_axis_world);
        
        // Calculate twist angle
        let twist_angle = if twist.w.abs() < 0.999 {
            2.0 * twist.w.acos() * twist.x.signum()
        } else {
            0.0
        };
        
        // Check if limits are exceeded
        let swing_angle = swing_axis_proj.length();
        let mut swing_limit_exceeded = false;
        let mut twist_limit_exceeded = false;
        
        // Calculate angular corrections if limits exceeded
        if swing_angle > self.swing_span1 || swing_angle > self.swing_span2 {
            swing_limit_exceeded = true;
        }
        
        if twist_angle.abs() > self.twist_span {
            twist_limit_exceeded = true;
        }
        
        // Apply swing limit constraint
        if swing_limit_exceeded && swing_angle > crate::math::EPSILON {
            // Normalize the swing axis
            let swing_normal = swing_axis_proj.normalize();
            
            // Calculate the limit violation
            let max_swing = self.swing_span1.min(self.swing_span2);
            let swing_error = swing_angle - max_swing;
            
            // Create jacobian for this constraint
            let j_ang_a = -swing_normal;
            let j_ang_b = swing_normal;
            
            // Calculate relative angular velocity along the swing axis
            let rel_ang_vel = (omega_b - omega_a).dot(&swing_normal);

            // Calculate effective mass
            let k = j_ang_a.dot(&inv_inertia_a.multiply_vector(j_ang_a)) +
                    j_ang_b.dot(&inv_inertia_b.multiply_vector(j_ang_b));

            if k > crate::math::EPSILON {
                let eff_mass = 1.0 / k;

                // Calculate lambda with position correction bias
                let bias = self.bias_factor * swing_error / dt;
                let softness = self.softness / dt;

                let lambda = -(rel_ang_vel + bias) * eff_mass / (1.0 + softness);

                // Update angular velocities
                omega_a = omega_a - inv_inertia_a.multiply_vector(j_ang_a * lambda);
                omega_b = omega_b + inv_inertia_b.multiply_vector(j_ang_b * lambda);
            }
        }
        
        // Apply twist limit constraint
        if twist_limit_exceeded {
            // Calculate the constraint axis (twist axis)
            let j_ang_a = -twist_axis_world;
            let j_ang_b = twist_axis_world;
            
            // Calculate the limit violation
            let twist_error = if twist_angle > 0.0 {
                twist_angle - self.twist_span
            } else {
                twist_angle + self.twist_span
            };
            
            // Calculate relative angular velocity along the twist axis
            let rel_ang_vel = (omega_b - omega_a).dot(&twist_axis_world);
            
            // Calculate effective mass
            let k = j_ang_a.dot(&inv_inertia_a.multiply_vector(j_ang_a)) +
                    j_ang_b.dot(&inv_inertia_b.multiply_vector(j_ang_b));
                    
            if k > crate::math::EPSILON {
                let eff_mass = 1.0 / k;
                
                // Calculate lambda with position correction bias
                let bias = self.bias_factor * twist_error / dt;
                let softness = self.softness / dt;
                
                let lambda = -(rel_ang_vel + bias) * eff_mass / (1.0 + softness);
                
                // Update angular velocities
                omega_a = omega_a - inv_inertia_a.multiply_vector(j_ang_a * lambda);
                omega_b = omega_b + inv_inertia_b.multiply_vector(j_ang_b * lambda);
            }
        }

        // Now apply the calculated velocities to the bodies
        if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
            body_a.set_linear_velocity(vel_a);
            body_a.set_angular_velocity(omega_a);
        }

        if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
            body_b.set_linear_velocity(vel_b);
            body_b.set_angular_velocity(omega_b);
        }
    }
    
    fn solve_position(&mut self, _dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Get body handles
        let body_a_handle = self.body_a;
        let body_b_handle = self.body_b;

        // Get properties we need from both bodies
        let (anchor_a_world, anchor_b_world, inv_mass_a, inv_mass_b, position_error, twist_axis_world, swing, twist);

        {
            // Get bodies as immutable references first to calculate positions and errors
            let body_a = match bodies.get_body(body_a_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(body_b_handle) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get anchor points in world space
            anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);

            // Calculate position error
            position_error = anchor_b_world - anchor_a_world;

            // Get inverse masses
            inv_mass_a = body_a.get_inverse_mass();
            inv_mass_b = body_b.get_inverse_mass();

            // Get the twist axis in world space
            twist_axis_world = body_a.get_transform().transform_direction(self.axis_a);

            // Calculate swing and twist components
            let swing_twist = self.get_swing_twist(body_a, body_b);
            swing = swing_twist.0;
            twist = swing_twist.1;
        }

        // Calculate correction factor
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop
        
        let correction_magnitude = (position_error.length() - slop).max(0.0) * beta;
        
        if correction_magnitude > crate::math::EPSILON {
            let correction_dir = position_error.normalize();

            // Apply position correction for the point constraint
            let mass_sum = inv_mass_a + inv_mass_b;
            if mass_sum > crate::math::EPSILON {
                let position_impulse = correction_dir * correction_magnitude / mass_sum;

                // Calculate new transforms
                let mut new_transform_a = {
                    let body_a = match bodies.get_body(body_a_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_a.get_transform();
                    transform.position -= position_impulse * inv_mass_a;
                    transform
                };

                let mut new_transform_b = {
                    let body_b = match bodies.get_body(body_b_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_b.get_transform();
                    transform.position += position_impulse * inv_mass_b;
                    transform
                };

                // Apply the transforms with separate mutable borrows
                if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                    body_a.set_transform(new_transform_a);
                }

                if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                    body_b.set_transform(new_transform_b);
                }
            }
        }

        // We already have swing and twist from before
        
        // Get the swing axis
        let swing_axis = if swing.w.abs() < 0.999 {
            let axis = Vector3::new(swing.x, swing.y, swing.z).normalize();
            let angle = 2.0 * swing.w.acos();
            axis * angle
        } else {
            Vector3::zero()
        };
        
        // Project swing axis onto plane perpendicular to twist axis
        let swing_axis_proj = swing_axis - twist_axis_world * swing_axis.dot(&twist_axis_world);
        
        // Calculate twist angle
        let twist_angle = if twist.w.abs() < 0.999 {
            2.0 * twist.w.acos() * twist.x.signum()
        } else {
            0.0
        };
        
        // Check if limits are exceeded
        let swing_angle = swing_axis_proj.length();
        let mut swing_correction = Vector3::zero();
        let mut twist_correction = 0.0;
        
        if swing_angle > self.swing_span1 || swing_angle > self.swing_span2 {
            // Calculate swing correction
            if swing_angle > crate::math::EPSILON {
                let max_swing = self.swing_span1.min(self.swing_span2);
                let correction_ratio = max_swing / swing_angle;
                swing_correction = swing_axis_proj * (1.0 - correction_ratio) * beta;
            }
        }
        
        if twist_angle.abs() > self.twist_span {
            // Calculate twist correction
            if twist_angle > 0.0 {
                twist_correction = (twist_angle - self.twist_span) * beta;
            } else {
                twist_correction = (twist_angle + self.twist_span) * beta;
            }
        }
        
        // Apply swing correction
        if swing_correction.length_squared() > crate::math::EPSILON {
            // Get body inertia tensors
            let (inv_inertia_a, inv_inertia_b);
            {
                let body_a = match bodies.get_body(body_a_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                let body_b = match bodies.get_body(body_b_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
                inv_inertia_b = body_b.get_inverse_inertia_tensor_world();
            }

            // Calculate how to distribute the correction
            let inertia_a = inv_inertia_a.multiply_vector(swing_correction);
            let inertia_b = inv_inertia_b.multiply_vector(swing_correction);
            let inertia_sum = inertia_a.length_squared() + inertia_b.length_squared();

            if inertia_sum > crate::math::EPSILON {
                let ratio_a = inertia_a.length_squared() / inertia_sum;
                let ratio_b = inertia_b.length_squared() / inertia_sum;

                // Create rotations from the swing correction
                let angle_a = (swing_correction * ratio_a).length();
                let angle_b = (swing_correction * ratio_b).length();

                // Calculate transformations
                let mut apply_to_a = false;
                let mut apply_to_b = false;
                let mut new_transform_a = {
                    let body_a = match bodies.get_body(body_a_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    body_a.get_transform()
                };
                let mut new_transform_b = {
                    let body_b = match bodies.get_body(body_b_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    body_b.get_transform()
                };

                // Calculate rotations for body A
                if angle_a > crate::math::EPSILON {
                    let axis_a = (swing_correction * ratio_a).normalize();
                    let rot_a = Quaternion::from_axis_angle(axis_a, -angle_a);
                    new_transform_a.rotation = rot_a * new_transform_a.rotation;
                    apply_to_a = true;
                }

                // Calculate rotations for body B
                if angle_b > crate::math::EPSILON {
                    let axis_b = (swing_correction * ratio_b).normalize();
                    let rot_b = Quaternion::from_axis_angle(axis_b, angle_b);
                    new_transform_b.rotation = rot_b * new_transform_b.rotation;
                    apply_to_b = true;
                }

                // Apply transformations with separate mutable borrows
                if apply_to_a {
                    if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                        body_a.set_transform(new_transform_a);
                    }
                }
                if apply_to_b {
                    if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                        body_b.set_transform(new_transform_b);
                    }
                }
            }
        }
        
        // Apply twist correction
        if twist_correction.abs() > crate::math::EPSILON {
            // Get body inertia tensors
            let (inv_inertia_a, inv_inertia_b);
            {
                let body_a = match bodies.get_body(body_a_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                let body_b = match bodies.get_body(body_b_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
                inv_inertia_b = body_b.get_inverse_inertia_tensor_world();
            }

            // Calculate how to distribute the correction
            let inertia_a = inv_inertia_a.multiply_vector(twist_axis_world);
            let inertia_b = inv_inertia_b.multiply_vector(twist_axis_world);
            let inertia_sum = inertia_a.length_squared() + inertia_b.length_squared();

            if inertia_sum > crate::math::EPSILON {
                let ratio_a = inertia_a.length_squared() / inertia_sum;
                let ratio_b = inertia_b.length_squared() / inertia_sum;

                let angle_a = twist_correction.abs() * ratio_a;
                let angle_b = twist_correction.abs() * ratio_b;

                // Calculate transformations
                let mut apply_to_a = false;
                let mut apply_to_b = false;
                let mut new_transform_a = {
                    let body_a = match bodies.get_body(body_a_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    body_a.get_transform()
                };
                let mut new_transform_b = {
                    let body_b = match bodies.get_body(body_b_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    body_b.get_transform()
                };

                // Calculate rotations for body A
                if angle_a > crate::math::EPSILON {
                    let rot_a = Quaternion::from_axis_angle(twist_axis_world, -twist_correction.signum() * angle_a);
                    new_transform_a.rotation = rot_a * new_transform_a.rotation;
                    apply_to_a = true;
                }

                // Calculate rotations for body B
                if angle_b > crate::math::EPSILON {
                    let rot_b = Quaternion::from_axis_angle(twist_axis_world, twist_correction.signum() * angle_b);
                    new_transform_b.rotation = rot_b * new_transform_b.rotation;
                    apply_to_b = true;
                }

                // Apply transformations with separate mutable borrows
                if apply_to_a {
                    if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                        body_a.set_transform(new_transform_a);
                    }
                }
                if apply_to_b {
                    if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                        body_b.set_transform(new_transform_b);
                    }
                }
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
            anchor_a: self.anchor_a,
            anchor_b: self.anchor_b,
            axis_a: self.axis_a,
            axis_b: self.axis_b,
            bodies: self.bodies,
            impulses: [0.0; 6], // Reset impulses on clone
            enabled: self.enabled,
            swing_span1: self.swing_span1,
            swing_span2: self.swing_span2,
            twist_span: self.twist_span,
            softness: self.softness,
            bias_factor: self.bias_factor,
            relaxation_factor: self.relaxation_factor,
            initial_orientation: self.initial_orientation,
        })
    }
}