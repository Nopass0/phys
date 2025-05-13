use crate::constraints::Constraint;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::{Vector3, Matrix3};
use std::any::Any;

/// A hinge (revolute) constraint that allows rotation around a single axis
#[derive(Debug)]
pub struct HingeConstraint {
    /// The first body in the constraint
    body_a: BodyHandle,
    
    /// The second body in the constraint
    body_b: BodyHandle,
    
    /// The pivot point on the first body (in local space)
    pivot_a: Vector3,
    
    /// The pivot point on the second body (in local space)
    pivot_b: Vector3,
    
    /// The axis of rotation on the first body (in local space)
    axis_a: Vector3,
    
    /// The axis of rotation on the second body (in local space)
    axis_b: Vector3,
    
    /// The bodies involved in the constraint (cached for quick lookup)
    bodies: [BodyHandle; 2],
    
    /// The impulses accumulated during the current step
    impulses: [f32; 5], // 3 for linear constraints, 2 for angular constraints
    
    /// Whether the constraint is enabled
    enabled: bool,
    
    /// Whether to use limits for the rotation
    use_limits: bool,
    
    /// The lower limit of the angle (in radians)
    lower_limit: f32,
    
    /// The upper limit of the angle (in radians)
    upper_limit: f32,
    
    /// The current angle (calculated during prepare)
    current_angle: f32,
    
    /// The motor impulse accumulated during the current step
    motor_impulse: f32,
    
    /// Whether the motor is enabled
    motor_enabled: bool,
    
    /// The target velocity of the motor (in radians per second)
    motor_target_velocity: f32,
    
    /// The maximum torque that the motor can apply
    motor_max_torque: f32,
}

impl HingeConstraint {
    /// Creates a new hinge constraint
    pub fn new(
        body_a: BodyHandle,
        body_b: BodyHandle,
        pivot_a: Vector3,
        pivot_b: Vector3,
        axis_a: Vector3,
        axis_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a: axis_a.normalize(),
            axis_b: axis_b.normalize(),
            bodies: [body_a, body_b],
            impulses: [0.0; 5],
            enabled: true,
            use_limits: false,
            lower_limit: 0.0,
            upper_limit: 0.0,
            current_angle: 0.0,
            motor_impulse: 0.0,
            motor_enabled: false,
            motor_target_velocity: 0.0,
            motor_max_torque: 0.0,
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
    
    /// Sets the limits for the rotation
    pub fn set_limits(&mut self, lower_limit: f32, upper_limit: f32) {
        self.lower_limit = lower_limit;
        self.upper_limit = upper_limit;
        self.use_limits = true;
    }
    
    /// Disables the limits for the rotation
    pub fn disable_limits(&mut self) {
        self.use_limits = false;
    }
    
    /// Returns whether the limits are enabled
    pub fn are_limits_enabled(&self) -> bool {
        self.use_limits
    }
    
    /// Returns the lower limit of the angle
    pub fn get_lower_limit(&self) -> f32 {
        self.lower_limit
    }
    
    /// Returns the upper limit of the angle
    pub fn get_upper_limit(&self) -> f32 {
        self.upper_limit
    }
    
    /// Enables the motor
    pub fn enable_motor(&mut self, target_velocity: f32, max_torque: f32) {
        self.motor_enabled = true;
        self.motor_target_velocity = target_velocity;
        self.motor_max_torque = max_torque.max(0.0);
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
    
    /// Returns the maximum torque of the motor
    pub fn get_motor_max_torque(&self) -> f32 {
        self.motor_max_torque
    }
    
    /// Returns the current angle
    pub fn get_angle(&self) -> f32 {
        self.current_angle
    }
    
    /// Calculates the current angle between the bodies
    fn calculate_angle(&self, body_a: &RigidBody, body_b: &RigidBody) -> f32 {
        // Get the world axes
        let axis_a_world = body_a.get_transform().transform_direction(self.axis_a);
        let axis_b_world = body_b.get_transform().transform_direction(self.axis_b);
        
        // Calculate angle between axes
        let cos_angle = axis_a_world.dot(&axis_b_world).clamp(-1.0, 1.0);
        let angle = cos_angle.acos();
        
        // Determine sign
        let cross = axis_a_world.cross(&axis_b_world);
        let dot = cross.dot(&axis_a_world);
        let sign = if dot < 0.0 { -1.0 } else { 1.0 };
        
        angle * sign
    }
}

impl Constraint for HingeConstraint {
    fn constraint_type(&self) -> &'static str {
        "Hinge"
    }
    
    fn get_bodies(&self) -> &[BodyHandle] {
        &self.bodies
    }
    
    fn prepare(&mut self, bodies: &BodyStorage<RigidBody>) {
        // Reset accumulated impulses
        self.impulses = [0.0; 5];
        self.motor_impulse = 0.0;
        
        // Calculate current angle
        let body_a = match bodies.get_body(self.body_a) {
            Ok(body) => body,
            Err(_) => return,
        };

        let body_b = match bodies.get_body(self.body_b) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        self.current_angle = self.calculate_angle(body_a, body_b);
    }
    
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Get all the data we need with immutable borrows first
        let (
            pivot_a_world, pivot_b_world, axis_a_world, axis_b_world,
            pos_a, pos_b, vel_a, vel_b, omega_a, omega_b,
            inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b
        ) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };

            // Get world position of anchor points
            let pivot_a_world = body_a.get_transform().transform_point(self.pivot_a);
            let pivot_b_world = body_b.get_transform().transform_point(self.pivot_b);

            // Get world axes
            let axis_a_world = body_a.get_transform().transform_direction(self.axis_a);
            let axis_b_world = body_b.get_transform().transform_direction(self.axis_b);

            // Get the position and orientation of the bodies
            let pos_a = body_a.get_position();
            let pos_b = body_b.get_position();

            // Get the velocities
            let vel_a = body_a.get_linear_velocity();
            let vel_b = body_b.get_linear_velocity();
            let omega_a = body_a.get_angular_velocity();
            let omega_b = body_b.get_angular_velocity();

            // Calculate inverse mass matrices
            let inv_mass_a = body_a.get_inverse_mass();
            let inv_mass_b = body_b.get_inverse_mass();

            let inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            let inv_inertia_b = body_b.get_inverse_inertia_tensor_world();

            (
                pivot_a_world, pivot_b_world, axis_a_world, axis_b_world,
                pos_a, pos_b, vel_a, vel_b, omega_a, omega_b,
                inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b
            )
        };

        // Calculate the relative anchor positions
        let ra = pivot_a_world - pos_a;
        let rb = pivot_b_world - pos_b;

        // Calculate the velocity of the anchor points
        let vel_a_point = vel_a + omega_a.cross(&ra);
        let vel_b_point = vel_b + omega_b.cross(&rb);

        // Relative velocity at the constraint point
        let rel_vel = vel_b_point - vel_a_point;

        // Calculate basis vectors for the constraint
        let n1 = axis_a_world;
        let n2 = if n1.cross(&Vector3::new(1.0, 0.0, 0.0)).length_squared() > 0.1 {
            n1.cross(&Vector3::new(1.0, 0.0, 0.0)).normalize()
        } else {
            n1.cross(&Vector3::new(0.0, 1.0, 0.0)).normalize()
        };
        let n3 = n1.cross(&n2);

        // Create the jacobian for the linear constraints
        // The jacobian maps velocities to constraint violations
        let j_linear = [
            (-n2, -ra.cross(&n2), n2, rb.cross(&n2)),  // Linear constraint along n2
            (-n3, -ra.cross(&n3), n3, rb.cross(&n3)),  // Linear constraint along n3
            (-n1, -ra.cross(&n1), n1, rb.cross(&n1)),  // Linear constraint along n1 (ensure colinearity)
        ];

        // Create the jacobian for the angular constraints
        let perp_a = if n1.cross(&axis_b_world).length_squared() > 0.1 {
            n1.cross(&axis_b_world).normalize()
        } else {
            n2
        };

        let perp_b = n1.cross(&perp_a);

        let j_angular = [
            (Vector3::zero(), -perp_a, Vector3::zero(), perp_a),  // Angular constraint to align axes
            (Vector3::zero(), -perp_b, Vector3::zero(), perp_b),  // Angular constraint to align axes
        ];

        // Store all velocity changes to apply at the end
        let mut new_vel_a = vel_a;
        let mut new_vel_b = vel_b;
        let mut new_omega_a = omega_a;
        let mut new_omega_b = omega_b;

        // Solve linear constraints
        for i in 0..3 {
            let (lin_a, ang_a, lin_b, ang_b) = j_linear[i];

            // Calculate the constraint violation
            let rel_vel_along_constraint = rel_vel.dot(&lin_a);

            // Calculate effective mass
            let k = inv_mass_a * lin_a.dot(&lin_a) +
                    inv_mass_b * lin_b.dot(&lin_b) +
                    ang_a.dot(&inv_inertia_a.multiply_vector(ang_a)) +
                    ang_b.dot(&inv_inertia_b.multiply_vector(ang_b));

            if k <= crate::math::EPSILON {
                continue;
            }

            let eff_mass = 1.0 / k;

            // Calculate lambda
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(rel_vel_along_constraint + bias) * eff_mass;

            // Accumulate impulse
            let old_impulse = self.impulses[i];
            self.impulses[i] += lambda;

            // Calculate impulse to apply
            let p = lin_a * lambda;

            // Update velocities (will be applied at the end)
            new_vel_a = new_vel_a - p * inv_mass_a;
            new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(ang_a * lambda);

            new_vel_b = new_vel_b + p * inv_mass_b;
            new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(ang_b * lambda);
        }

        // Solve angular constraints
        for i in 0..2 {
            let (lin_a, ang_a, lin_b, ang_b) = j_angular[i];

            // Calculate the constraint violation
            let rel_ang_vel = new_omega_b - new_omega_a;
            let rel_vel_along_constraint = rel_ang_vel.dot(&ang_b);

            // Calculate effective mass
            let k = ang_a.dot(&inv_inertia_a.multiply_vector(ang_a)) +
                    ang_b.dot(&inv_inertia_b.multiply_vector(ang_b));

            if k <= crate::math::EPSILON {
                continue;
            }

            let eff_mass = 1.0 / k;

            // Calculate lambda
            let bias = 0.0; // We'll handle position correction in solve_position
            let lambda = -(rel_vel_along_constraint + bias) * eff_mass;

            // Accumulate impulse
            let old_impulse = self.impulses[i + 3];
            self.impulses[i + 3] += lambda;

            // Update velocities (will be applied at the end)
            new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(ang_a * lambda);
            new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(ang_b * lambda);
        }

        // Solve motor constraint
        if self.motor_enabled && self.motor_max_torque > 0.0 {
            let rel_ang_vel = (new_omega_b - new_omega_a).dot(&axis_a_world);
            let motor_vel_error = self.motor_target_velocity - rel_ang_vel;

            // Calculate effective mass
            let k = axis_a_world.dot(&inv_inertia_a.multiply_vector(axis_a_world)) +
                    axis_a_world.dot(&inv_inertia_b.multiply_vector(axis_a_world));

            if k <= crate::math::EPSILON {
                return;
            }

            let eff_mass = 1.0 / k;

            // Calculate lambda
            let lambda = motor_vel_error * eff_mass;

            // Accumulate impulse
            let old_impulse = self.motor_impulse;
            self.motor_impulse += lambda;

            // Clamp impulse
            let max_impulse = self.motor_max_torque * dt;
            self.motor_impulse = self.motor_impulse.clamp(-max_impulse, max_impulse);

            // Calculate impulse to apply
            let motor_lambda = self.motor_impulse - old_impulse;

            // Update velocities (will be applied at the end)
            new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(axis_a_world * motor_lambda);
            new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(axis_a_world * motor_lambda);
        }

        // Solve limits
        if self.use_limits {
            // Check if the angle is out of bounds
            let angle_violation = if self.current_angle < self.lower_limit {
                self.lower_limit - self.current_angle
            } else if self.current_angle > self.upper_limit {
                self.upper_limit - self.current_angle
            } else {
                0.0
            };

            if angle_violation != 0.0 {
                // Calculate the constraint axis
                let limit_axis = if angle_violation < 0.0 {
                    -axis_a_world // Need to decrease the angle
                } else {
                    axis_a_world // Need to increase the angle
                };

                // Calculate effective mass
                let k = limit_axis.dot(&inv_inertia_a.multiply_vector(limit_axis)) +
                        limit_axis.dot(&inv_inertia_b.multiply_vector(limit_axis));

                if k <= crate::math::EPSILON {
                    return;
                }

                let eff_mass = 1.0 / k;

                // Calculate limits impulse
                let rel_ang_vel = (new_omega_b - new_omega_a).dot(&limit_axis);
                let bias = 0.2 * angle_violation / dt; // Position correction

                let lambda = -(rel_ang_vel + bias) * eff_mass;

                // Update velocities (will be applied at the end)
                new_omega_a = new_omega_a - inv_inertia_a.multiply_vector(limit_axis * lambda);
                new_omega_b = new_omega_b + inv_inertia_b.multiply_vector(limit_axis * lambda);
            }
        }

        // Apply all velocity changes with separate mutable borrows
        if let Ok(body_a) = bodies.get_body_mut(self.body_a) {
            body_a.set_linear_velocity(new_vel_a);
            body_a.set_angular_velocity(new_omega_a);
        }

        if let Ok(body_b) = bodies.get_body_mut(self.body_b) {
            body_b.set_linear_velocity(new_vel_b);
            body_b.set_angular_velocity(new_omega_b);
        }
    }
    
    fn solve_position(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>) {
        if !self.enabled {
            return;
        }

        // Get all the data we need with immutable borrows first
        let (pivot_a_world, pivot_b_world, axis_a_world, axis_b_world, transform_a, transform_b, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b) = {
            let body_a = match bodies.get_body(self.body_a) {
                Ok(body) => body,
                Err(_) => return,
            };

            let body_b = match bodies.get_body(self.body_b) {
                Ok(body) => body,
                Err(_) => return,
            };

            let transform_a = body_a.get_transform();
            let transform_b = body_b.get_transform();

            // Get world position of anchor points
            let pivot_a_world = transform_a.transform_point(self.pivot_a);
            let pivot_b_world = transform_b.transform_point(self.pivot_b);

            // Get world axes
            let axis_a_world = transform_a.transform_direction(self.axis_a);
            let axis_b_world = transform_b.transform_direction(self.axis_b);

            // Get inverse masses and inertias
            let inv_mass_a = body_a.get_inverse_mass();
            let inv_mass_b = body_b.get_inverse_mass();

            let inv_inertia_a = body_a.get_inverse_inertia_tensor_world();
            let inv_inertia_b = body_b.get_inverse_inertia_tensor_world();

            (
                pivot_a_world, pivot_b_world, axis_a_world, axis_b_world,
                transform_a, transform_b, inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b
            )
        };

        // Calculate positional error
        let position_error = pivot_b_world - pivot_a_world;

        // Calculate angular error
        let axis_error = axis_a_world.cross(&axis_b_world);

        // Calculate correction factor
        let beta = 0.2; // Position correction factor
        let slop = 0.01; // Penetration slop

        // Initialize new transforms
        let mut new_transform_a = transform_a;
        let mut new_transform_b = transform_b;

        // Apply position correction for the point constraint
        let mass_sum = inv_mass_a + inv_mass_b;
        if mass_sum > crate::math::EPSILON {
            let correction = (position_error.length() - slop).max(0.0) * beta;
            let position_impulse = position_error.normalize() * correction / mass_sum;

            new_transform_a.position -= position_impulse * inv_mass_a;
            new_transform_b.position += position_impulse * inv_mass_b;
        }

        // Apply correction for the axis alignment
        let angular_error = axis_error.length();
        if angular_error > slop {
            let correction = (angular_error - slop) * beta;
            let axis = axis_error.normalize();

            let k_angular = axis.dot(&inv_inertia_a.multiply_vector(axis)) +
                           axis.dot(&inv_inertia_b.multiply_vector(axis));

            if k_angular > crate::math::EPSILON {
                let angular_impulse = axis * correction / k_angular;

                // Apply angular impulse
                let delta_rot_a = inv_inertia_a.multiply_vector(angular_impulse);
                let delta_rot_b = inv_inertia_b.multiply_vector(angular_impulse);

                // Convert to quaternion changes (simplified)
                let angle_a = delta_rot_a.length();
                let angle_b = delta_rot_b.length();

                if angle_a > crate::math::EPSILON {
                    let axis_a = delta_rot_a / angle_a;
                    let rot_a = crate::math::Quaternion::from_axis_angle(axis_a, -angle_a);
                    new_transform_a.rotation = rot_a * new_transform_a.rotation;
                }

                if angle_b > crate::math::EPSILON {
                    let axis_b = delta_rot_b / angle_b;
                    let rot_b = crate::math::Quaternion::from_axis_angle(axis_b, angle_b);
                    new_transform_b.rotation = rot_b * new_transform_b.rotation;
                }
            }
        }

        // Calculate the current angle directly using the new transforms
        let axis_a_world_updated = new_transform_a.transform_direction(self.axis_a);
        let axis_b_world_updated = new_transform_b.transform_direction(self.axis_b);

        // Calculate angle between axes (same logic as in calculate_angle)
        let cos_angle = axis_a_world_updated.dot(&axis_b_world_updated).clamp(-1.0, 1.0);
        let angle = cos_angle.acos();

        // Determine sign
        let cross = axis_a_world_updated.cross(&axis_b_world_updated);
        let dot = cross.dot(&axis_a_world_updated);
        let sign = if dot < 0.0 { -1.0 } else { 1.0 };

        let current_angle = angle * sign;

        // Apply limits
        if self.use_limits {
            // Check if the angle is out of bounds
            if current_angle < self.lower_limit {
                // Apply rotation to bring angle to lower limit
                let angle_correction = self.lower_limit - current_angle;
                let rot_axis = axis_a_world_updated;

                // Apply rotation to both bodies
                let rot_a = crate::math::Quaternion::from_axis_angle(rot_axis, angle_correction * 0.5);
                let rot_b = crate::math::Quaternion::from_axis_angle(rot_axis, -angle_correction * 0.5);

                new_transform_a.rotation = rot_a * new_transform_a.rotation;
                new_transform_b.rotation = rot_b * new_transform_b.rotation;
            } else if current_angle > self.upper_limit {
                // Apply rotation to bring angle to upper limit
                let angle_correction = self.upper_limit - current_angle;
                let rot_axis = axis_a_world_updated;

                // Apply rotation to both bodies
                let rot_a = crate::math::Quaternion::from_axis_angle(rot_axis, angle_correction * 0.5);
                let rot_b = crate::math::Quaternion::from_axis_angle(rot_axis, -angle_correction * 0.5);

                new_transform_a.rotation = rot_a * new_transform_a.rotation;
                new_transform_b.rotation = rot_b * new_transform_b.rotation;
            }
        }

        // Apply all transform changes with separate mutable borrows
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
            pivot_a: self.pivot_a,
            pivot_b: self.pivot_b,
            axis_a: self.axis_a,
            axis_b: self.axis_b,
            bodies: self.bodies,
            impulses: [0.0; 5], // Reset impulses on clone
            enabled: self.enabled,
            use_limits: self.use_limits,
            lower_limit: self.lower_limit,
            upper_limit: self.upper_limit,
            current_angle: 0.0,
            motor_impulse: 0.0,
            motor_enabled: self.motor_enabled,
            motor_target_velocity: self.motor_target_velocity,
            motor_max_torque: self.motor_max_torque,
        })
    }
}