use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::{Vector3, Quaternion};
use std::any::Any;

/// A slider (prismatic) constraint that allows translation along a single axis
#[derive(Debug)]
pub struct SliderConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The anchor point on the first body (in local space)
    anchor_a: Vector3,
    
    /// The anchor point on the second body (in local space)
    anchor_b: Vector3,
    
    /// The axis of translation on the first body (in local space)
    axis_a: Vector3,
    
    /// The axis of translation on the second body (in local space)
    axis_b: Vector3,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The impulses accumulated during the current step
    /// [0-2]: Angular constraints, [3-4]: Linear constraints perpendicular to axis, [5]: Motor
    impulses: [f32; 6],
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// Whether to use limits for the translation
    use_limits: bool,
    
    /// The lower limit of the translation
    lower_limit: f32,
    
    /// The upper limit of the translation
    upper_limit: f32,
    
    /// The current offset (calculated during prepare)
    current_offset: f32,
    
    /// The motor impulse accumulated during the current step
    motor_impulse: f32,
    
    /// Whether the motor is enabled
    motor_enabled: bool,
    
    /// The target velocity of the motor (in units per second)
    motor_target_velocity: f32,
    
    /// The maximum force that the motor can apply
    motor_max_force: f32,
}

impl SliderConstraint {
    /// Creates a new slider constraint
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
            use_limits: false,
            lower_limit: 0.0,
            upper_limit: 0.0,
            current_offset: 0.0,
            motor_impulse: 0.0,
            motor_enabled: false,
            motor_target_velocity: 0.0,
            motor_max_force: 0.0,
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
    
    /// Sets the limits for the translation
    pub fn set_limits(&mut self, lower_limit: f32, upper_limit: f32) {
        self.lower_limit = lower_limit;
        self.upper_limit = upper_limit;
        self.use_limits = true;
    }
    
    /// Disables the limits for the translation
    pub fn disable_limits(&mut self) {
        self.use_limits = false;
    }
    
    /// Returns whether the limits are enabled
    pub fn are_limits_enabled(&self) -> bool {
        self.use_limits
    }
    
    /// Returns the lower limit of the translation
    pub fn get_lower_limit(&self) -> f32 {
        self.lower_limit
    }
    
    /// Returns the upper limit of the translation
    pub fn get_upper_limit(&self) -> f32 {
        self.upper_limit
    }
    
    /// Enables the motor
    pub fn enable_motor(&mut self, target_velocity: f32, max_force: f32) {
        self.motor_enabled = true;
        self.motor_target_velocity = target_velocity;
        self.motor_max_force = max_force.max(0.0);
    }
    
    /// Disables the motor
    pub fn disable_motor(&mut self) {
        self.motor_enabled = false;
    }
    
    /// Returns whether the motor is enabled
    pub fn is_motor_enabled(&self) -> bool {
        self.motor_enabled
    }
    
    /// Returns the target velocity of the motor
    pub fn get_motor_target_velocity(&self) -> f32 {
        self.motor_target_velocity
    }
    
    /// Returns the maximum force of the motor
    pub fn get_motor_max_force(&self) -> f32 {
        self.motor_max_force
    }
    
    /// Returns the current offset
    pub fn get_offset(&self) -> f32 {
        self.current_offset
    }
    
    /// Calculates the current offset between the bodies
    fn calculate_offset(&self, body_a: &RigidBody, body_b: &RigidBody) -> f32 {
        // Transform anchor points to world space
        let anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
        let anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);
        
        // Get the axis in world space
        let axis_a_world = body_a.get_transform().transform_direction(self.axis_a);
        
        // Vector from anchor_a to anchor_b
        let r = anchor_b_world - anchor_a_world;
        
        // Project onto the axis
        r.dot(&axis_a_world)
    }
}

impl Constraint for SliderConstraint {
    fn constraint_type(&self) -> &'static str {
        "Slider"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulses
        self.impulses = [0.0; 6];
        self.motor_impulse = 0.0;
        
        // Calculate current offset
        let body_a = match bodies.get_body(self.body_a) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        let body_b = match bodies.get_body(self.body_b) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        self.current_offset = self.calculate_offset(body_a, body_b);
    }
    
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // We need to handle one body at a time to avoid borrow checker issues
        let body_a_handle = self.body_a;
        let body_b_handle = self.body_b;

        // Get all the data we need first using immutable borrows
        let pos_a;
        let mut vel_a;
        let mut omega_a;
        let inv_mass_a;
        let anchor_a_world;
        let ra;
        let pos_b;
        let mut vel_b;
        let mut omega_b;
        let inv_mass_b;
        let anchor_b_world;
        let rb;
        let axis_a_world;

        // Pre-calculate all jacobian-related data
        let j_lin_a_arr = [Vector3::ZERO; 3];
        let j_lin_b_arr = [Vector3::ZERO; 3];
        let j_ang_a_arr = [Vector3::ZERO; 3];
        let j_ang_b_arr = [Vector3::ZERO; 3];

        // Calculate and store all data we need before any mutable operations
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

            // Calculate anchor point in world space for body A
            anchor_a_world = body_a.get_transform().transform_point(self.anchor_a);
            ra = anchor_a_world - pos_a;

            // Get axis in world space
            axis_a_world = body_a.get_transform().transform_direction(self.axis_a);

            // Calculate properties for body B
            pos_b = body_b.get_position();
            vel_b = body_b.get_linear_velocity();
            omega_b = body_b.get_angular_velocity();
            inv_mass_b = body_b.get_inverse_mass();

            // Calculate anchor point in world space for body B
            anchor_b_world = body_b.get_transform().transform_point(self.anchor_b);
            rb = anchor_b_world - pos_b;
        }
        
        // Calculate basis vectors for the constraint
        let n = axis_a_world; // Primary axis (allows motion)
        let t1 = if n.cross(&Vector3::new(1.0, 0.0, 0.0)).length_squared() > 0.1 {
            n.cross(&Vector3::new(1.0, 0.0, 0.0)).normalize()
        } else {
            n.cross(&Vector3::new(0.0, 1.0, 0.0)).normalize()
        };
        let t2 = n.cross(&t1);

        // Calculate the velocity of the anchor points
        let vel_a_point = vel_a + omega_a.cross(&ra);
        let vel_b_point = vel_b + omega_b.cross(&rb);

        // Relative velocity at the constraint point
        let rel_vel = vel_b_point - vel_a_point;

        // Pre-calculate all the inertia-related calculations
        let inv_inertia_a;
        let inv_inertia_b;
        let eff_mass_array = [0.0; 3]; // For angular constraints
        let mut eff_mass_lin_array = [0.0; 2]; // For linear constraints (t1, t2)
        let mut motor_eff_mass = 0.0;
        let mut limit_eff_mass = 0.0;

        {
            // Get bodies again to calculate inertia tensors
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

            // Pre-calculate effective masses for t1 and t2 directions
            let j_lin_a_t1 = -t1;
            let j_ang_a_t1 = -ra.cross(&t1);
            let j_lin_b_t1 = t1;
            let j_ang_b_t1 = rb.cross(&t1);

            let k_t1 = inv_mass_a * j_lin_a_t1.dot(&j_lin_a_t1) +
                     inv_mass_b * j_lin_b_t1.dot(&j_lin_b_t1) +
                     j_ang_a_t1.dot(&inv_inertia_a.multiply_vector(j_ang_a_t1)) +
                     j_ang_b_t1.dot(&inv_inertia_b.multiply_vector(j_ang_b_t1));

            if k_t1 > crate::math::EPSILON {
                eff_mass_lin_array[0] = 1.0 / k_t1;
            }

            let j_lin_a_t2 = -t2;
            let j_ang_a_t2 = -ra.cross(&t2);
            let j_lin_b_t2 = t2;
            let j_ang_b_t2 = rb.cross(&t2);

            let k_t2 = inv_mass_a * j_lin_a_t2.dot(&j_lin_a_t2) +
                     inv_mass_b * j_lin_b_t2.dot(&j_lin_b_t2) +
                     j_ang_a_t2.dot(&inv_inertia_a.multiply_vector(j_ang_a_t2)) +
                     j_ang_b_t2.dot(&inv_inertia_b.multiply_vector(j_ang_b_t2));

            if k_t2 > crate::math::EPSILON {
                eff_mass_lin_array[1] = 1.0 / k_t2;
            }

            // Pre-calculate for motor constraint
            let j_lin_a_motor = -axis_a_world;
            let j_ang_a_motor = -ra.cross(&axis_a_world);
            let j_lin_b_motor = axis_a_world;
            let j_ang_b_motor = rb.cross(&axis_a_world);

            let k_motor = inv_mass_a * j_lin_a_motor.dot(&j_lin_a_motor) +
                        inv_mass_b * j_lin_b_motor.dot(&j_lin_b_motor) +
                        j_ang_a_motor.dot(&inv_inertia_a.multiply_vector(j_ang_a_motor)) +
                        j_ang_b_motor.dot(&inv_inertia_b.multiply_vector(j_ang_b_motor));

            if k_motor > crate::math::EPSILON {
                motor_eff_mass = 1.0 / k_motor;
            } else {
                motor_eff_mass = 0.0;
            }

            // Pre-calculate for limits constraint
            let limit_axis = axis_a_world;
            let j_lin_a_limit = -limit_axis;
            let j_ang_a_limit = -ra.cross(&limit_axis);
            let j_lin_b_limit = limit_axis;
            let j_ang_b_limit = rb.cross(&limit_axis);

            let k_limit = inv_mass_a * j_lin_a_limit.dot(&j_lin_a_limit) +
                        inv_mass_b * j_lin_b_limit.dot(&j_lin_b_limit) +
                        j_ang_a_limit.dot(&inv_inertia_a.multiply_vector(j_ang_a_limit)) +
                        j_ang_b_limit.dot(&inv_inertia_b.multiply_vector(j_ang_b_limit));

            if k_limit > crate::math::EPSILON {
                limit_eff_mass = 1.0 / k_limit;
            } else {
                limit_eff_mass = 0.0;
            }
        }

        // Solve linear constraint along t1 and t2 (perpendicular to axis)
        for i in 0..2 {
            let tangent = if i == 0 { t1 } else { t2 };

            // Calculate the constraint violation
            let rel_vel_along_constraint = rel_vel.dot(&tangent);

            // Create jacobian for this constraint
            let j_lin_a = -tangent;
            let j_ang_a = -ra.cross(&tangent);
            let j_lin_b = tangent;
            let j_ang_b = rb.cross(&tangent);

            // Get pre-calculated effective mass
            let eff_mass = if i == 0 { eff_mass_lin_array[0] } else { eff_mass_lin_array[1] };

            if eff_mass <= crate::math::EPSILON {
                continue;
            }

            // Calculate lambda
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(rel_vel_along_constraint + bias) * eff_mass;

            // Accumulate impulse
            self.impulses[i + 3] += lambda;

            // Calculate impulse to apply
            let p = tangent * lambda;

            // Calculate angular impulses directly
            let ang_impulse_a = j_ang_a * lambda;
            let ang_impulse_b = j_ang_b * lambda;

            // Update velocities (we'll apply at the end)
            vel_a = vel_a - p * inv_mass_a;
            omega_a = omega_a - ang_impulse_a;

            vel_b = vel_b + p * inv_mass_b;
            omega_b = omega_b + ang_impulse_b;
        }

        // Solve angular constraints - need to align axes except along sliding axis
        // Calculate perpendicular directions
        let perp1 = if axis_a_world.cross(&axis_a_world).length_squared() > crate::math::EPSILON {
            axis_a_world.cross(&axis_a_world).normalize()
        } else {
            t1
        };

        let perp2 = axis_a_world.cross(&perp1).normalize();
        let perp3 = axis_a_world.cross(&perp1).normalize();

        // Solve angular constraint to keep axes aligned
        for i in 0..3 {
            let perp = if i == 0 { perp1 } else if i == 1 { perp2 } else { perp3 };

            // Calculate the constraint violation
            let angular_error = axis_a_world.cross(&axis_a_world).dot(&perp);

            // Create jacobian for this constraint
            let j_ang_a = -perp;
            let j_ang_b = perp;

            // Use pre-calculated effective mass
            let eff_mass = eff_mass_array[i];

            if eff_mass <= crate::math::EPSILON {
                continue;
            }

            // Calculate lambda
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(angular_error + bias) * eff_mass;

            // Accumulate impulse
            self.impulses[i] += lambda;

            // Calculate angular impulses directly
            let ang_impulse_a = j_ang_a * lambda;
            let ang_impulse_b = j_ang_b * lambda;

            // Update velocities (we'll apply at the end)
            omega_a = omega_a - ang_impulse_a;
            omega_b = omega_b + ang_impulse_b;
        }

        // Solve motor constraint
        if self.motor_enabled && self.motor_max_force > 0.0 {
            let rel_lin_vel = rel_vel.dot(&axis_a_world);
            let motor_vel_error = self.motor_target_velocity - rel_lin_vel;

            // Use pre-calculated effective mass
            if motor_eff_mass <= crate::math::EPSILON {
                return;
            }

            // Calculate lambda
            let lambda = motor_vel_error * motor_eff_mass;

            // Accumulate impulse
            let old_impulse = self.motor_impulse;
            self.motor_impulse += lambda;

            // Clamp impulse
            let max_impulse = self.motor_max_force * dt;
            self.motor_impulse = self.motor_impulse.clamp(-max_impulse, max_impulse);

            // Calculate impulse to apply
            let motor_lambda = self.motor_impulse - old_impulse;
            let p = axis_a_world * motor_lambda;

            // Calculate angular impulses directly
            let j_ang_a = -ra.cross(&axis_a_world);
            let j_ang_b = rb.cross(&axis_a_world);
            let ang_impulse_a = j_ang_a * motor_lambda;
            let ang_impulse_b = j_ang_b * motor_lambda;

            // Update velocities (we'll apply at the end)
            vel_a = vel_a - p * inv_mass_a;
            omega_a = omega_a - ang_impulse_a;

            vel_b = vel_b + p * inv_mass_b;
            omega_b = omega_b + ang_impulse_b;
        }

        // Solve limits
        if self.use_limits {
            // Check if the offset is out of bounds
            let offset_violation = if self.current_offset < self.lower_limit {
                self.lower_limit - self.current_offset
            } else if self.current_offset > self.upper_limit {
                self.upper_limit - self.current_offset
            } else {
                0.0
            };

            if offset_violation != 0.0 {
                // Calculate the constraint axis
                let limit_axis = if offset_violation < 0.0 {
                    -axis_a_world // Need to decrease the offset
                } else {
                    axis_a_world // Need to increase the offset
                };

                // Use pre-calculated effective mass
                if limit_eff_mass <= crate::math::EPSILON {
                    return;
                }

                // Calculate limits impulse
                let rel_lin_vel = rel_vel.dot(&limit_axis);
                let bias = 0.2 * offset_violation / dt; // Position correction

                let lambda = -(rel_lin_vel + bias) * limit_eff_mass;

                // Calculate angular impulses directly
                let j_ang_a = -ra.cross(&limit_axis);
                let j_ang_b = rb.cross(&limit_axis);
                let ang_impulse_a = j_ang_a * lambda;
                let ang_impulse_b = j_ang_b * lambda;

                // Update velocities (we'll apply at the end)
                vel_a = vel_a - limit_axis * lambda * inv_mass_a;
                omega_a = omega_a - ang_impulse_a;

                vel_b = vel_b + limit_axis * lambda * inv_mass_b;
                omega_b = omega_b + ang_impulse_b;
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

        // Get all data we need using immutable borrows first
        let anchor_a_world;
        let anchor_b_world;
        let axis_a_world;
        let inv_mass_a;
        let inv_mass_b;
        let pos_a;
        let pos_b;
        let ra;
        let rb;

        // Pre-calculate all vectors used by the constraint
        let j_ang_a_t1;
        let j_ang_b_t1;
        let j_ang_a_t2;
        let j_ang_b_t2;
        let axis_a_t1;
        let axis_b_t1;
        let axis_a_t2;
        let axis_b_t2;
        let t1;
        let t2;

        // First, get all the data we need using immutable borrows
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

            // Get world axes
            axis_a_world = body_a.get_transform().transform_direction(self.axis_a);

            // Get body positions for calculating ra and rb
            pos_a = body_a.get_position();
            pos_b = body_b.get_position();

            // Get inverse masses
            inv_mass_a = body_a.get_inverse_mass();
            inv_mass_b = body_b.get_inverse_mass();

            // Calculate relative anchor positions
            ra = anchor_a_world - pos_a;
            rb = anchor_b_world - pos_b;

            // Get inverse inertia tensors and pre-calculate all required vectors
            let inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            let inv_inertia_b = body_b.get_inverse_inertia_tensor_world();

            // Calculate perpendicular directions
            let n = axis_a_world;
            t1 = if n.cross(&Vector3::new(1.0, 0.0, 0.0)).length_squared() > 0.1 {
                n.cross(&Vector3::new(1.0, 0.0, 0.0)).normalize()
            } else {
                n.cross(&Vector3::new(0.0, 1.0, 0.0)).normalize()
            };
            t2 = n.cross(&t1);

            // Pre-calculate jacobians and axes for t1 direction
            j_ang_a_t1 = -ra.cross(&t1);
            j_ang_b_t1 = rb.cross(&t1);
            axis_a_t1 = inv_inertia_a.multiply_vector(j_ang_a_t1).normalize();
            axis_b_t1 = inv_inertia_b.multiply_vector(j_ang_b_t1).normalize();

            // Pre-calculate jacobians and axes for t2 direction
            j_ang_a_t2 = -ra.cross(&t2);
            j_ang_b_t2 = rb.cross(&t2);
            axis_a_t2 = inv_inertia_a.multiply_vector(j_ang_a_t2).normalize();
            axis_b_t2 = inv_inertia_b.multiply_vector(j_ang_b_t2).normalize();
        }
        
        // Calculate position error perpendicular to the axis
        let error_vec = anchor_b_world - anchor_a_world;
        let error_t1 = error_vec.dot(&t1);
        let error_t2 = error_vec.dot(&t2);

        // Calculate correction factor
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop

        // Correct position for t1 direction
        if error_t1.abs() > slop {
            let correction = (error_t1.abs() - slop) * beta * error_t1.signum();
            let correction_dir = t1 * correction;

            // Calculate effective mass for this constraint
            let j_lin_a = -t1;
            let j_lin_b = t1;

            // We already calculated j_ang_a_t1 and j_ang_b_t1 earlier
            // Instead of recalculating, we use our pre-calculated values

            // Calculate k manually using pre-calculated values
            let k = inv_mass_a * j_lin_a.dot(&j_lin_a) +
                    inv_mass_b * j_lin_b.dot(&j_lin_b);

            // Effective mass calculation
            if k > crate::math::EPSILON {
                let pos_impulse = correction_dir / k;

                // Use pre-calculated axis and calculate angles
                let angle_a = j_ang_a_t1.length() * correction;
                let angle_b = j_ang_b_t1.length() * correction;
                let rot_a = Quaternion::from_axis_angle(axis_a_t1, -angle_a);
                let rot_b = Quaternion::from_axis_angle(axis_b_t1, angle_b);

                // Calculate new transforms
                let mut new_transform_a = {
                    let body_a = match bodies.get_body(body_a_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_a.get_transform();
                    transform.position -= pos_impulse * inv_mass_a;

                    if angle_a > crate::math::EPSILON {
                        transform.rotation = rot_a * transform.rotation;
                    }

                    transform
                };

                let mut new_transform_b = {
                    let body_b = match bodies.get_body(body_b_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_b.get_transform();
                    transform.position += pos_impulse * inv_mass_b;

                    if angle_b > crate::math::EPSILON {
                        transform.rotation = rot_b * transform.rotation;
                    }

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

        // Correct position for t2 direction (similar to t1)
        if error_t2.abs() > slop {
            let correction = (error_t2.abs() - slop) * beta * error_t2.signum();
            let correction_dir = t2 * correction;

            // Calculate effective mass for this constraint
            let j_lin_a = -t2;
            let j_lin_b = t2;

            // We already calculated j_ang_a_t2 and j_ang_b_t2 earlier
            // Instead of recalculating, we use our pre-calculated values

            // Calculate k manually using pre-calculated values
            let k = inv_mass_a * j_lin_a.dot(&j_lin_a) +
                    inv_mass_b * j_lin_b.dot(&j_lin_b);

            // Effective mass calculation
            if k > crate::math::EPSILON {
                let pos_impulse = correction_dir / k;

                // Use pre-calculated axis and calculate angles
                let angle_a = j_ang_a_t2.length() * correction;
                let angle_b = j_ang_b_t2.length() * correction;
                let rot_a = Quaternion::from_axis_angle(axis_a_t2, -angle_a);
                let rot_b = Quaternion::from_axis_angle(axis_b_t2, angle_b);

                // Calculate new transforms
                let mut new_transform_a = {
                    let body_a = match bodies.get_body(body_a_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_a.get_transform();
                    transform.position -= pos_impulse * inv_mass_a;

                    if angle_a > crate::math::EPSILON {
                        transform.rotation = rot_a * transform.rotation;
                    }

                    transform
                };

                let mut new_transform_b = {
                    let body_b = match bodies.get_body(body_b_handle) {
                        Ok(body) => body,
                        Err(_) => return,
                    };
                    let mut transform = body_b.get_transform();
                    transform.position += pos_impulse * inv_mass_b;

                    if angle_b > crate::math::EPSILON {
                        transform.rotation = rot_b * transform.rotation;
                    }

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

        // Apply limits if enabled
        if self.use_limits {
            // Get body references to calculate current offset
            let current_offset;
            {
                let body_a = match bodies.get_body(body_a_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                let body_b = match bodies.get_body(body_b_handle) {
                    Ok(body) => body,
                    Err(_) => return,
                };

                current_offset = self.calculate_offset(body_a, body_b);
            }

            // Check if the offset is out of bounds
            if current_offset < self.lower_limit {
                // Apply correction to bring offset to lower limit
                let offset_correction = self.lower_limit - current_offset;

                // Create correction vector along axis
                let correction_dir = axis_a_world * offset_correction;

                // Calculate how to distribute the correction based on mass
                let mass_sum = inv_mass_a + inv_mass_b;
                if mass_sum > crate::math::EPSILON {
                    let ratio_a = inv_mass_a / mass_sum;
                    let ratio_b = inv_mass_b / mass_sum;

                    // Calculate new transforms
                    let mut new_transform_a = {
                        let body_a = match bodies.get_body(body_a_handle) {
                            Ok(body) => body,
                            Err(_) => return,
                        };
                        let mut transform = body_a.get_transform();
                        transform.position -= correction_dir * ratio_a;
                        transform
                    };

                    let mut new_transform_b = {
                        let body_b = match bodies.get_body(body_b_handle) {
                            Ok(body) => body,
                            Err(_) => return,
                        };
                        let mut transform = body_b.get_transform();
                        transform.position += correction_dir * ratio_b;
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
            } else if current_offset > self.upper_limit {
                // Apply correction to bring offset to upper limit
                let offset_correction = self.upper_limit - current_offset;

                // Create correction vector along axis
                let correction_dir = axis_a_world * offset_correction;

                // Calculate how to distribute the correction based on mass
                let mass_sum = inv_mass_a + inv_mass_b;
                if mass_sum > crate::math::EPSILON {
                    let ratio_a = inv_mass_a / mass_sum;
                    let ratio_b = inv_mass_b / mass_sum;

                    // Calculate new transforms
                    let mut new_transform_a = {
                        let body_a = match bodies.get_body(body_a_handle) {
                            Ok(body) => body,
                            Err(_) => return,
                        };
                        let mut transform = body_a.get_transform();
                        transform.position -= correction_dir * ratio_a;
                        transform
                    };

                    let mut new_transform_b = {
                        let body_b = match bodies.get_body(body_b_handle) {
                            Ok(body) => body,
                            Err(_) => return,
                        };
                        let mut transform = body_b.get_transform();
                        transform.position += correction_dir * ratio_b;
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
            use_limits: self.use_limits,
            lower_limit: self.lower_limit,
            upper_limit: self.upper_limit,
            current_offset: 0.0,
            motor_impulse: 0.0,
            motor_enabled: self.motor_enabled,
            motor_target_velocity: self.motor_target_velocity,
            motor_max_force: self.motor_max_force,
        })
    }
}