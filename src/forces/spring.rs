use crate::forces::ForceGenerator;
use crate::core::{BodyHandle, BodyStorage, Storage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A force generator that simulates a spring between two bodies or a body and a fixed point
#[derive(Debug, Clone)]
pub struct SpringForce {
    /// The first body connected by the spring (must be set)
    body_a: BodyHandle,
    
    /// The second body connected by the spring (optional, if None then the spring connects to a fixed world point)
    body_b: Option<BodyHandle>,
    
    /// The attachment point on the first body (in local space)
    local_connection_a: Vector3,
    
    /// The attachment point on the second body (in local space) or fixed world point if body_b is None
    connection_b: Vector3,
    
    /// Whether connection_b is in local or world space
    connection_b_is_local: bool,
    
    /// The spring rest length
    rest_length: f32,
    
    /// The spring stiffness coefficient (higher values mean stiffer springs)
    stiffness: f32,
    
    /// The spring damping coefficient (higher values mean more damping)
    damping: f32,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// The bodies affected by this spring (cached for quick lookup)
    affected_bodies: Vec<BodyHandle>,
}

impl SpringForce {
    /// Creates a new spring between two bodies
    pub fn new_between_bodies(
        body_a: BodyHandle,
        body_b: BodyHandle,
        local_connection_a: Vector3,
        local_connection_b: Vector3,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        let mut affected_bodies = Vec::new();
        affected_bodies.push(body_a);
        affected_bodies.push(body_b);
        
        Self {
            body_a,
            body_b: Some(body_b),
            local_connection_a,
            connection_b: local_connection_b,
            connection_b_is_local: true,
            rest_length: rest_length.max(0.0),
            stiffness: stiffness.max(0.0),
            damping: damping.max(0.0),
            enabled: true,
            affected_bodies,
        }
    }
    
    /// Creates a new spring between a body and a fixed world point
    pub fn new_to_world_point(
        body: BodyHandle,
        local_connection: Vector3,
        world_point: Vector3,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        let mut affected_bodies = Vec::new();
        affected_bodies.push(body);
        
        Self {
            body_a: body,
            body_b: None,
            local_connection_a: local_connection,
            connection_b: world_point,
            connection_b_is_local: false,
            rest_length: rest_length.max(0.0),
            stiffness: stiffness.max(0.0),
            damping: damping.max(0.0),
            enabled: true,
            affected_bodies,
        }
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Gets the spring rest length
    pub fn get_rest_length(&self) -> f32 {
        self.rest_length
    }
    
    /// Sets the spring rest length
    pub fn set_rest_length(&mut self, rest_length: f32) {
        self.rest_length = rest_length.max(0.0);
    }
    
    /// Gets the spring stiffness coefficient
    pub fn get_stiffness(&self) -> f32 {
        self.stiffness
    }
    
    /// Sets the spring stiffness coefficient
    pub fn set_stiffness(&mut self, stiffness: f32) {
        self.stiffness = stiffness.max(0.0);
    }
    
    /// Gets the spring damping coefficient
    pub fn get_damping(&self) -> f32 {
        self.damping
    }
    
    /// Sets the spring damping coefficient
    pub fn set_damping(&mut self, damping: f32) {
        self.damping = damping.max(0.0);
    }
    
    /// Gets the local connection point on body A
    pub fn get_local_connection_a(&self) -> Vector3 {
        self.local_connection_a
    }
    
    /// Sets the local connection point on body A
    pub fn set_local_connection_a(&mut self, local_connection: Vector3) {
        self.local_connection_a = local_connection;
    }
    
    /// Gets the connection point B (interpretation depends on whether it's a local point on body B or a world point)
    pub fn get_connection_b(&self) -> Vector3 {
        self.connection_b
    }
    
    /// Sets the connection point B as a local point on body B
    pub fn set_local_connection_b(&mut self, local_connection: Vector3) {
        self.connection_b = local_connection;
        self.connection_b_is_local = true;
    }
    
    /// Sets the connection point B as a world point
    pub fn set_world_connection_b(&mut self, world_point: Vector3) {
        self.connection_b = world_point;
        self.connection_b_is_local = false;
    }
}

impl ForceGenerator for SpringForce {
    fn generator_type(&self) -> &'static str {
        "Spring"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        // Get first body - use immutable borrow first
        let body_a = match bodies.get(self.body_a) {
            Some(body) => body,
            None => return,
        };
        
        // Get connection point A in world space
        let connection_a_world = body_a.get_transform().transform_point(self.local_connection_a);
        
        // Get connection point B in world space
        let connection_b_world = match self.body_b {
            Some(body_b_handle) => {
                // Spring connected to another body
                let body_b = match bodies.get(body_b_handle) {
                    Some(body) => body,
                    None => return,
                };
                
                if self.connection_b_is_local {
                    body_b.get_transform().transform_point(self.connection_b)
                } else {
                    self.connection_b
                }
            },
            None => {
                // Spring connected to a fixed world point
                self.connection_b
            }
        };
        
        // Calculate velocities at connection points
        let vel_a = body_a.get_linear_velocity() + 
                   body_a.get_angular_velocity().cross(&(connection_a_world - body_a.get_position()));
        
        let vel_b = match self.body_b {
            Some(body_b_handle) => {
                let body_b = match bodies.get(body_b_handle) {
                    Some(body) => body,
                    None => return,
                };
                body_b.get_linear_velocity() + 
                body_b.get_angular_velocity().cross(&(connection_b_world - body_b.get_position()))
            },
            None => Vector3::zero(), // Fixed point has no velocity
        };
        
        // Calculate vector from A to B
        let spring_vector = connection_b_world - connection_a_world;
        let current_length = spring_vector.length();
        
        if current_length < crate::math::EPSILON {
            return; // Points are practically coincident, no force to apply
        }
        
        // Calculate normalized direction
        let spring_direction = spring_vector / current_length;
        
        // Calculate spring force using Hooke's Law: F = -k * (x - x0)
        // where:
        // - k is the spring stiffness
        // - (x - x0) is the displacement from rest length
        let spring_stretch = current_length - self.rest_length;
        let spring_force_magnitude = self.stiffness * spring_stretch;
        
        // Calculate relative velocity along the spring direction
        let rel_vel = vel_b - vel_a;
        let rel_vel_along_spring = rel_vel.dot(&spring_direction);
        
        // Calculate damping force: F_d = -c * v
        // where:
        // - c is the damping coefficient
        // - v is the relative velocity along the spring direction
        let damping_force_magnitude = self.damping * rel_vel_along_spring;
        
        // Total force
        let total_force_magnitude = spring_force_magnitude + damping_force_magnitude;
        let force = spring_direction * total_force_magnitude;
        
        // Apply forces at connection points - now use mutable borrows one at a time
        if let Ok(body_a_mut) = bodies.get_body_mut(self.body_a) {
            body_a_mut.apply_force_at_point(force, connection_a_world);
        }
        
        // Apply opposite force to body B if it exists
        if let Some(body_b_handle) = self.body_b {
            if let Ok(body_b_mut) = bodies.get_body_mut(body_b_handle) {
                body_b_mut.apply_force_at_point(-force, connection_b_world);
            }
        }
    }
    
    fn get_affected_bodies(&self) -> &[BodyHandle] {
        &self.affected_bodies
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_generator(&self) -> Box<dyn ForceGenerator> {
        Box::new(self.clone())
    }
}

/// A force generator that simulates a bungee cord (only applies forces when stretched beyond rest length)
#[derive(Debug, Clone)]
pub struct BungeeForce {
    /// The internal spring used for the bungee implementation
    spring: SpringForce,
}

impl BungeeForce {
    /// Creates a new bungee between two bodies
    pub fn new_between_bodies(
        body_a: BodyHandle,
        body_b: BodyHandle,
        local_connection_a: Vector3,
        local_connection_b: Vector3,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        Self {
            spring: SpringForce::new_between_bodies(
                body_a,
                body_b,
                local_connection_a,
                local_connection_b,
                rest_length,
                stiffness,
                damping,
            )
        }
    }
    
    /// Creates a new bungee between a body and a fixed world point
    pub fn new_to_world_point(
        body: BodyHandle,
        local_connection: Vector3,
        world_point: Vector3,
        rest_length: f32,
        stiffness: f32,
        damping: f32,
    ) -> Self {
        Self {
            spring: SpringForce::new_to_world_point(
                body,
                local_connection,
                world_point,
                rest_length,
                stiffness,
                damping,
            )
        }
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.spring.is_enabled()
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.spring.set_enabled(enabled);
    }
    
    /// Gets the bungee rest length
    pub fn get_rest_length(&self) -> f32 {
        self.spring.get_rest_length()
    }
    
    /// Sets the bungee rest length
    pub fn set_rest_length(&mut self, rest_length: f32) {
        self.spring.set_rest_length(rest_length);
    }
    
    /// Gets the bungee stiffness coefficient
    pub fn get_stiffness(&self) -> f32 {
        self.spring.get_stiffness()
    }
    
    /// Sets the bungee stiffness coefficient
    pub fn set_stiffness(&mut self, stiffness: f32) {
        self.spring.set_stiffness(stiffness);
    }
    
    /// Gets the bungee damping coefficient
    pub fn get_damping(&self) -> f32 {
        self.spring.get_damping()
    }
    
    /// Sets the bungee damping coefficient
    pub fn set_damping(&mut self, damping: f32) {
        self.spring.set_damping(damping);
    }
}

impl ForceGenerator for BungeeForce {
    fn generator_type(&self) -> &'static str {
        "Bungee"
    }
    
    fn update(&mut self, dt: f32) {
        self.spring.update(dt);
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.spring.is_enabled() {
            return;
        }
        
        // Get first body - use immutable borrow first
        let body_a = match bodies.get(self.spring.body_a) {
            Some(body) => body,
            None => return,
        };
        
        // Get connection point A in world space
        let connection_a_world = body_a.get_transform().transform_point(self.spring.local_connection_a);
        
        // Get connection point B in world space
        let connection_b_world = match self.spring.body_b {
            Some(body_b_handle) => {
                // Bungee connected to another body
                let body_b = match bodies.get(body_b_handle) {
                    Some(body) => body,
                    None => return,
                };
                
                if self.spring.connection_b_is_local {
                    body_b.get_transform().transform_point(self.spring.connection_b)
                } else {
                    self.spring.connection_b
                }
            },
            None => {
                // Bungee connected to a fixed world point
                self.spring.connection_b
            }
        };
        
        // Calculate velocities at connection points
        let vel_a = body_a.get_linear_velocity() + 
                   body_a.get_angular_velocity().cross(&(connection_a_world - body_a.get_position()));
        
        let vel_b = match self.spring.body_b {
            Some(body_b_handle) => {
                let body_b = match bodies.get(body_b_handle) {
                    Some(body) => body,
                    None => return,
                };
                body_b.get_linear_velocity() + 
                body_b.get_angular_velocity().cross(&(connection_b_world - body_b.get_position()))
            },
            None => Vector3::zero(), // Fixed point has no velocity
        };
        
        // Calculate vector from A to B
        let bungee_vector = connection_b_world - connection_a_world;
        let current_length = bungee_vector.length();
        
        if current_length < crate::math::EPSILON {
            return; // Points are practically coincident, no force to apply
        }
        
        // Only apply forces if the bungee is stretched beyond rest length
        if current_length <= self.spring.rest_length {
            return;
        }
        
        // Calculate normalized direction
        let bungee_direction = bungee_vector / current_length;
        
        // Calculate bungee force using Hooke's Law for the part beyond rest length
        let bungee_stretch = current_length - self.spring.rest_length;
        let bungee_force_magnitude = self.spring.stiffness * bungee_stretch;
        
        // Calculate relative velocity along the bungee direction
        let rel_vel = vel_b - vel_a;
        let rel_vel_along_bungee = rel_vel.dot(&bungee_direction);
        
        // Only apply damping when the bungee is extending
        let damping_force_magnitude = if rel_vel_along_bungee > 0.0 {
            0.0
        } else {
            self.spring.damping * rel_vel_along_bungee
        };
        
        // Total force
        let total_force_magnitude = bungee_force_magnitude + damping_force_magnitude;
        let force = bungee_direction * total_force_magnitude;
        
        // Apply forces at connection points - now use mutable borrows one at a time
        if let Ok(body_a_mut) = bodies.get_body_mut(self.spring.body_a) {
            body_a_mut.apply_force_at_point(force, connection_a_world);
        }
        
        // Apply opposite force to body B if it exists
        if let Some(body_b_handle) = self.spring.body_b {
            if let Ok(body_b_mut) = bodies.get_body_mut(body_b_handle) {
                body_b_mut.apply_force_at_point(-force, connection_b_world);
            }
        }
    }
    
    fn get_affected_bodies(&self) -> &[BodyHandle] {
        self.spring.get_affected_bodies()
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_generator(&self) -> Box<dyn ForceGenerator> {
        Box::new(self.clone())
    }
}