use crate::bodies::{body_flags::BodyFlags, Material, RigidBodyType, ForceType};
use crate::core::BodyHandle;
use crate::math::{Vector3, Transform, Matrix3, Quaternion};
use crate::shapes::Shape;

use std::sync::Arc;

/// Type alias for a handle to a rigid body
pub type RigidBodyHandle = BodyHandle;

/// A rigid body for physics simulation
pub struct RigidBody {
    /// The body's transform in world space
    transform: Transform,
    
    /// The body's linear velocity
    linear_velocity: Vector3,
    
    /// The body's angular velocity
    angular_velocity: Vector3,
    
    /// The body's material properties
    material: Material,
    
    /// The body's collision shape
    shape: Option<Arc<dyn Shape>>,
    
    /// The body's type (dynamic, kinematic, or static)
    body_type: RigidBodyType,
    
    /// The body's mass
    mass: f32,
    
    /// Inverse of the body's mass (for efficiency)
    inv_mass: f32,
    
    /// The body's inertia tensor in local space
    inertia_tensor: Matrix3,
    
    /// Inverse of the body's inertia tensor in local space
    inv_inertia_tensor: Matrix3,
    
    /// Inverse of the body's inertia tensor in world space
    inv_inertia_tensor_world: Matrix3,
    
    /// The body's linear damping
    linear_damping: f32,
    
    /// The body's angular damping
    angular_damping: f32,
    
    /// The body's flags
    flags: BodyFlags,
    
    /// How long the body has been "inactive" (for sleeping)
    sleeping_time: f32,
    
    /// Forces to be applied in the next integration step
    forces: Vec<ForceType>,
}

impl RigidBody {
    /// Creates a new rigid body with the given shape and transform
    pub fn new(shape: Arc<dyn Shape>, transform: Transform, body_type: RigidBodyType) -> Self {
        let mut body = Self {
            transform,
            linear_velocity: Vector3::zero(),
            angular_velocity: Vector3::zero(),
            material: Material::default(),
            shape: Some(shape),
            body_type,
            mass: 1.0,
            inv_mass: 1.0,
            inertia_tensor: Matrix3::identity(),
            inv_inertia_tensor: Matrix3::identity(),
            inv_inertia_tensor_world: Matrix3::identity(),
            linear_damping: 0.0,
            angular_damping: 0.0,
            flags: BodyFlags::CAN_SLEEP | BodyFlags::AFFECTED_BY_GRAVITY | BodyFlags::GENERATE_COLLISION_EVENTS,
            sleeping_time: 0.0,
            forces: Vec::new(),
        };
        
        // Set mass and inertia based on the shape and material
        body.update_mass_properties();
        
        body
    }
    
    /// Creates a new dynamic rigid body with the given shape and position
    pub fn new_dynamic(shape: Arc<dyn Shape>, position: Vector3) -> Self {
        Self::new(
            shape,
            Transform::from_position(position),
            RigidBodyType::Dynamic,
        )
    }
    
    /// Creates a new kinematic rigid body with the given shape and position
    pub fn new_kinematic(shape: Arc<dyn Shape>, position: Vector3) -> Self {
        let mut body = Self::new(
            shape,
            Transform::from_position(position),
            RigidBodyType::Kinematic,
        );
        
        body.flags.insert(BodyFlags::KINEMATIC);
        
        body
    }
    
    /// Creates a new static rigid body with the given shape and position
    pub fn new_static(shape: Arc<dyn Shape>, position: Vector3) -> Self {
        Self::new(
            shape,
            Transform::from_position(position),
            RigidBodyType::Static,
        )
    }
    
    /// Creates a trigger volume (non-solid collider) with the given shape and position
    pub fn new_trigger(shape: Arc<dyn Shape>, position: Vector3) -> Self {
        let mut body = Self::new(
            shape,
            Transform::from_position(position),
            RigidBodyType::Kinematic,
        );
        
        body.flags.insert(BodyFlags::TRIGGER);
        
        body
    }
    
    /// Returns the body's transform
    pub fn get_transform(&self) -> Transform {
        self.transform
    }
    
    /// Sets the body's transform
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.update_inertia_tensor_world();
    }
    
    /// Returns the body's position
    pub fn get_position(&self) -> Vector3 {
        self.transform.position
    }
    
    /// Sets the body's position
    pub fn set_position(&mut self, position: Vector3) {
        self.transform.position = position;
    }
    
    /// Returns the body's rotation as a quaternion
    pub fn get_rotation(&self) -> Quaternion {
        self.transform.rotation
    }
    
    /// Sets the body's rotation as a quaternion
    pub fn set_rotation(&mut self, rotation: Quaternion) {
        self.transform.rotation = rotation;
        self.update_inertia_tensor_world();
    }
    
    /// Returns the body's linear velocity
    pub fn get_linear_velocity(&self) -> Vector3 {
        self.linear_velocity
    }
    
    /// Sets the body's linear velocity
    pub fn set_linear_velocity(&mut self, velocity: Vector3) {
        self.linear_velocity = velocity;
        self.wake_up();
    }
    
    /// Returns the body's angular velocity
    pub fn get_angular_velocity(&self) -> Vector3 {
        self.angular_velocity
    }
    
    /// Sets the body's angular velocity
    pub fn set_angular_velocity(&mut self, velocity: Vector3) {
        self.angular_velocity = velocity;
        self.wake_up();
    }
    
    /// Returns the body's material
    pub fn get_material(&self) -> &Material {
        &self.material
    }
    
    /// Returns a mutable reference to the body's material
    pub fn get_material_mut(&mut self) -> &mut Material {
        &mut self.material
    }
    
    /// Sets the body's material
    pub fn set_material(&mut self, material: Material) {
        self.material = material;
        self.update_mass_properties();
    }
    
    /// Returns a reference to the body's shape
    pub fn get_shape(&self) -> Option<&Arc<dyn Shape>> {
        self.shape.as_ref()
    }
    
    /// Sets the body's shape
    pub fn set_shape(&mut self, shape: Arc<dyn Shape>) {
        self.shape = Some(shape);
        self.update_mass_properties();
    }
    
    /// Returns the body's mass
    pub fn get_mass(&self) -> f32 {
        self.mass
    }
    
    /// Sets the body's mass (and inverse mass)
    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        
        if self.body_type == RigidBodyType::Dynamic && mass > 0.0 {
            self.inv_mass = 1.0 / mass;
        } else {
            self.inv_mass = 0.0;
        }
    }
    
    /// Returns the body's inverse mass
    pub fn get_inverse_mass(&self) -> f32 {
        self.inv_mass
    }
    
    /// Returns the body's inertia tensor in local space
    pub fn get_inertia_tensor(&self) -> &Matrix3 {
        &self.inertia_tensor
    }
    
    /// Sets the body's inertia tensor in local space
    pub fn set_inertia_tensor(&mut self, tensor: Matrix3) {
        self.inertia_tensor = tensor;
        
        // Update inverse tensors
        if self.body_type == RigidBodyType::Dynamic {
            if let Some(inv) = tensor.inverse() {
                self.inv_inertia_tensor = inv;
                self.update_inertia_tensor_world();
            }
        } else {
            self.inv_inertia_tensor = Matrix3::zero();
            self.inv_inertia_tensor_world = Matrix3::zero();
        }
    }
    
    /// Returns the body's inverse inertia tensor in world space
    pub fn get_inverse_inertia_tensor_world(&self) -> &Matrix3 {
        &self.inv_inertia_tensor_world
    }
    
    /// Returns the body type
    pub fn get_body_type(&self) -> RigidBodyType {
        self.body_type
    }
    
    /// Sets the body type and updates mass properties accordingly
    pub fn set_body_type(&mut self, body_type: RigidBodyType) {
        self.body_type = body_type;
        self.update_mass_properties();
        
        // Reset velocities and forces for non-dynamic bodies
        if body_type != RigidBodyType::Dynamic {
            self.linear_velocity = Vector3::zero();
            self.angular_velocity = Vector3::zero();
            self.forces.clear();
        }
    }
    
    /// Sets the body's linear damping
    pub fn set_linear_damping(&mut self, damping: f32) {
        self.linear_damping = damping.max(0.0);
    }
    
    /// Returns the body's linear damping
    pub fn get_linear_damping(&self) -> f32 {
        self.linear_damping
    }
    
    /// Sets the body's angular damping
    pub fn set_angular_damping(&mut self, damping: f32) {
        self.angular_damping = damping.max(0.0);
    }
    
    /// Returns the body's angular damping
    pub fn get_angular_damping(&self) -> f32 {
        self.angular_damping
    }
    
    /// Returns whether the body is sleeping
    pub fn is_sleeping(&self) -> bool {
        self.flags.contains(BodyFlags::SLEEPING)
    }
    
    /// Puts the body to sleep
    pub fn put_to_sleep(&mut self) {
        if self.body_type == RigidBodyType::Dynamic && !self.is_sleeping() {
            self.flags.insert(BodyFlags::SLEEPING);
            self.linear_velocity = Vector3::zero();
            self.angular_velocity = Vector3::zero();
            self.forces.clear();
        }
    }
    
    /// Wakes up the body
    pub fn wake_up(&mut self) {
        if self.is_sleeping() {
            self.flags.remove(BodyFlags::SLEEPING);
            self.sleeping_time = 0.0;
        }
    }
    
    /// Returns whether the body can sleep
    pub fn can_sleep(&self) -> bool {
        self.flags.contains(BodyFlags::CAN_SLEEP)
    }
    
    /// Sets whether the body can sleep
    pub fn set_can_sleep(&mut self, can_sleep: bool) {
        if can_sleep {
            self.flags.insert(BodyFlags::CAN_SLEEP);
        } else {
            self.flags.remove(BodyFlags::CAN_SLEEP);
            self.wake_up();
        }
    }
    
    /// Returns whether the body is affected by gravity
    pub fn is_affected_by_gravity(&self) -> bool {
        self.flags.contains(BodyFlags::AFFECTED_BY_GRAVITY)
    }
    
    /// Sets whether the body is affected by gravity
    pub fn set_affected_by_gravity(&mut self, affected: bool) {
        if affected {
            self.flags.insert(BodyFlags::AFFECTED_BY_GRAVITY);
        } else {
            self.flags.remove(BodyFlags::AFFECTED_BY_GRAVITY);
        }
    }
    
    /// Returns whether the body has CCD enabled
    pub fn is_ccd_enabled(&self) -> bool {
        self.flags.contains(BodyFlags::CCD_ENABLED)
    }
    
    /// Sets whether the body has CCD enabled
    pub fn set_ccd_enabled(&mut self, enabled: bool) {
        if enabled {
            self.flags.insert(BodyFlags::CCD_ENABLED);
        } else {
            self.flags.remove(BodyFlags::CCD_ENABLED);
        }
    }
    
    /// Returns whether the body is a trigger
    pub fn is_trigger(&self) -> bool {
        self.flags.contains(BodyFlags::TRIGGER)
    }
    
    /// Sets whether the body is a trigger
    pub fn set_trigger(&mut self, is_trigger: bool) {
        if is_trigger {
            self.flags.insert(BodyFlags::TRIGGER);
        } else {
            self.flags.remove(BodyFlags::TRIGGER);
        }
    }
    
    /// Returns whether the body generates collision events
    pub fn generates_collision_events(&self) -> bool {
        self.flags.contains(BodyFlags::GENERATE_COLLISION_EVENTS)
    }
    
    /// Sets whether the body generates collision events
    pub fn set_generates_collision_events(&mut self, generates: bool) {
        if generates {
            self.flags.insert(BodyFlags::GENERATE_COLLISION_EVENTS);
        } else {
            self.flags.remove(BodyFlags::GENERATE_COLLISION_EVENTS);
        }
    }
    
    /// Returns the time the body has been inactive
    pub fn get_sleeping_time(&self) -> f32 {
        self.sleeping_time
    }
    
    /// Sets the time the body has been inactive
    pub fn set_sleeping_time(&mut self, time: f32) {
        self.sleeping_time = time;
    }
    
    /// Updates the body's mass properties based on its shape and material
    pub fn update_mass_properties(&mut self) {
        // Reset to default values
        self.mass = 1.0;
        self.inv_mass = 1.0;
        self.inertia_tensor = Matrix3::identity();
        self.inv_inertia_tensor = Matrix3::identity();
        
        // Update based on shape and material
        if let Some(shape) = &self.shape {
            if self.body_type == RigidBodyType::Dynamic {
                // Compute mass from shape volume and material density
                let volume = shape.get_volume();
                self.mass = volume * self.material.density;
                
                if self.mass > 0.0 {
                    self.inv_mass = 1.0 / self.mass;
                } else {
                    self.inv_mass = 0.0;
                }
                
                // Compute inertia tensor
                self.inertia_tensor = shape.get_inertia_tensor(self.mass);
                
                // Compute inverse inertia tensor
                if let Some(inv) = self.inertia_tensor.inverse() {
                    self.inv_inertia_tensor = inv;
                } else {
                    self.inv_inertia_tensor = Matrix3::zero();
                }
                
                // Update world space inertia tensor
                self.update_inertia_tensor_world();
            } else {
                // Non-dynamic bodies have infinite mass and zero inverse mass/inertia
                self.mass = 0.0;
                self.inv_mass = 0.0;
                self.inertia_tensor = Matrix3::zero();
                self.inv_inertia_tensor = Matrix3::zero();
                self.inv_inertia_tensor_world = Matrix3::zero();
            }
        }
    }
    
    /// Updates the inverse inertia tensor in world space
    fn update_inertia_tensor_world(&mut self) {
        if self.body_type != RigidBodyType::Dynamic {
            self.inv_inertia_tensor_world = Matrix3::zero();
            return;
        }
        
        // Convert the local inverse inertia tensor to world space
        let rotation_matrix = self.transform.rotation.to_rotation_matrix();
        let rotation_transpose = rotation_matrix.transpose();
        
        // Compute R * inv_I * R^T
        let temp = rotation_matrix.multiply_matrix(&self.inv_inertia_tensor);
        self.inv_inertia_tensor_world = temp.multiply_matrix(&rotation_transpose);
    }
    
    /// Applies a force to the body
    pub fn apply_force(&mut self, force: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::Force(force));
        }
    }
    
    /// Applies a force at a specific point
    pub fn apply_force_at_point(&mut self, force: Vector3, point: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::ForceAtPoint { force, point });
        }
    }
    
    /// Applies a torque to the body
    pub fn apply_torque(&mut self, torque: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::Torque(torque));
        }
    }
    
    /// Applies an impulse to the body
    pub fn apply_impulse(&mut self, impulse: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::Impulse(impulse));
        }
    }
    
    /// Applies an impulse at a specific point
    pub fn apply_impulse_at_point(&mut self, impulse: Vector3, point: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::ImpulseAtPoint { impulse, point });
        }
    }
    
    /// Applies an angular impulse to the body
    pub fn apply_angular_impulse(&mut self, impulse: Vector3) {
        if self.body_type == RigidBodyType::Dynamic {
            // Wake up the body if it's sleeping
            if self.is_sleeping() {
                self.wake_up();
            }
            self.forces.push(ForceType::AngularImpulse(impulse));
        }
    }
    
    /// Applies damping to the body's velocities
    pub fn apply_damping(&mut self, linear_damping: f32, angular_damping: f32) {
        if self.body_type != RigidBodyType::Dynamic || self.is_sleeping() {
            return;
        }
        
        // Apply linear damping
        let linear_damping_factor = 1.0 - (self.linear_damping + linear_damping).clamp(0.0, 1.0);
        self.linear_velocity *= linear_damping_factor;
        
        // Apply angular damping
        let angular_damping_factor = 1.0 - (self.angular_damping + angular_damping).clamp(0.0, 1.0);
        self.angular_velocity *= angular_damping_factor;
    }
    
    /// Integrates forces to update velocities
    pub fn integrate_forces(&mut self, dt: f32) {
        if self.body_type != RigidBodyType::Dynamic || self.is_sleeping() {
            return;
        }
        
        // Apply accumulated forces and impulses
        for force in &self.forces {
            match force {
                ForceType::Force(force) => {
                    // F = ma, a = F/m
                    let acceleration = *force * self.inv_mass;
                    self.linear_velocity += acceleration * dt;
                }
                ForceType::ForceAtPoint { force, point } => {
                    // Linear component: F = ma, a = F/m
                    let acceleration = *force * self.inv_mass;
                    self.linear_velocity += acceleration * dt;
                    
                    // Torque component: τ = r × F
                    let r = *point - self.transform.position;
                    let torque = r.cross(force);
                    
                    // Angular acceleration: α = I^-1 * τ
                    let angular_acceleration = self.inv_inertia_tensor_world.multiply_vector(torque);
                    self.angular_velocity += angular_acceleration * dt;
                }
                ForceType::Torque(torque) => {
                    // Angular acceleration: α = I^-1 * τ
                    let angular_acceleration = self.inv_inertia_tensor_world.multiply_vector(*torque);
                    self.angular_velocity += angular_acceleration * dt;
                }
                ForceType::Impulse(impulse) => {
                    // Δv = impulse / m
                    self.linear_velocity += *impulse * self.inv_mass;
                }
                ForceType::ImpulseAtPoint { impulse, point } => {
                    // Linear component: Δv = impulse / m
                    self.linear_velocity += *impulse * self.inv_mass;
                    
                    // Angular component: r × impulse
                    let r = *point - self.transform.position;
                    let angular_impulse = r.cross(impulse);
                    
                    // Δω = I^-1 * angular_impulse
                    let delta_omega = self.inv_inertia_tensor_world.multiply_vector(angular_impulse);
                    self.angular_velocity += delta_omega;
                }
                ForceType::AngularImpulse(impulse) => {
                    // Δω = I^-1 * impulse
                    let delta_omega = self.inv_inertia_tensor_world.multiply_vector(*impulse);
                    self.angular_velocity += delta_omega;
                }
            }
        }
        
        // Clear forces now that they've been applied
        self.forces.clear();
    }
    
    /// Integrates velocities to update position
    pub fn integrate_velocity(&mut self, dt: f32) {
        if self.body_type != RigidBodyType::Dynamic || self.is_sleeping() {
            return;
        }
        
        // Update position
        self.transform.position += self.linear_velocity * dt;
        
        // Update rotation
        if !self.angular_velocity.is_zero() {
            let angle = self.angular_velocity.length() * dt;
            let axis = self.angular_velocity.normalize();
            
            let rotation = Quaternion::from_axis_angle(axis, angle);
            self.transform.rotation = rotation * self.transform.rotation;
            self.transform.rotation = self.transform.rotation.normalize();
            
            // Update world space inertia tensor
            self.update_inertia_tensor_world();
        }
    }
}