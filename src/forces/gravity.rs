use crate::forces::ForceGenerator;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A force generator that applies gravity to bodies
#[derive(Debug, Clone)]
pub struct GravityForce {
    /// The gravity acceleration vector
    gravity: Vector3,
    
    /// The bodies affected by this gravity
    affected_bodies: Vec<BodyHandle>,
    
    /// Whether the force generator is enabled
    enabled: bool,
}

impl GravityForce {
    /// Creates a new gravity force generator with the given gravity vector
    pub fn new(gravity: Vector3) -> Self {
        Self {
            gravity,
            affected_bodies: Vec::new(),
            enabled: true,
        }
    }
    
    /// Creates a new gravity force generator with Earth-like gravity (-9.81 in y direction)
    pub fn new_earth_gravity() -> Self {
        Self::new(Vector3::new(0.0, -9.81, 0.0))
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Gets the current gravity acceleration
    pub fn get_gravity(&self) -> Vector3 {
        self.gravity
    }
    
    /// Sets the gravity acceleration vector
    pub fn set_gravity(&mut self, gravity: Vector3) {
        self.gravity = gravity;
    }
    
    /// Adds a body to be affected by this gravity
    pub fn add_body(&mut self, body: BodyHandle) {
        if !self.affected_bodies.contains(&body) {
            self.affected_bodies.push(body);
        }
    }
    
    /// Removes a body from being affected by this gravity
    pub fn remove_body(&mut self, body: BodyHandle) {
        self.affected_bodies.retain(|&b| b != body);
    }
    
    /// Clears all bodies affected by this gravity
    pub fn clear_bodies(&mut self) {
        self.affected_bodies.clear();
    }
}

impl ForceGenerator for GravityForce {
    fn generator_type(&self) -> &'static str {
        "Gravity"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update for constant gravity
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        for &body_handle in &self.affected_bodies {
            let body = match bodies.get_body_mut(body_handle) {
                Ok(body) => body,
                Err(_) => continue,
            };
            
            // Skip if the body is not affected by gravity
            if !body.is_affected_by_gravity() {
                continue;
            }
            
            // F = m * g
            let mass = body.get_mass();
            let force = self.gravity * mass;
            
            // Apply the force at the center of mass
            body.apply_force(force);
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

/// A force generator that applies gravity from a point (like a planet)
#[derive(Debug, Clone)]
pub struct PointGravityForce {
    /// The position of the gravity source
    position: Vector3,
    
    /// The gravitational constant strength
    strength: f32,
    
    /// The bodies affected by this gravity
    affected_bodies: Vec<BodyHandle>,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// Minimum distance to avoid numerical instability
    min_distance: f32,
}

impl PointGravityForce {
    /// Creates a new point gravity force generator
    pub fn new(position: Vector3, strength: f32) -> Self {
        Self {
            position,
            strength: strength.max(0.0),
            affected_bodies: Vec::new(),
            enabled: true,
            min_distance: 0.1, // Minimum distance to avoid division by zero
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
    
    /// Gets the current position of the gravity source
    pub fn get_position(&self) -> Vector3 {
        self.position
    }
    
    /// Sets the position of the gravity source
    pub fn set_position(&mut self, position: Vector3) {
        self.position = position;
    }
    
    /// Gets the current strength
    pub fn get_strength(&self) -> f32 {
        self.strength
    }
    
    /// Sets the strength of the gravity
    pub fn set_strength(&mut self, strength: f32) {
        self.strength = strength.max(0.0);
    }
    
    /// Sets the minimum distance for the gravity calculation
    pub fn set_min_distance(&mut self, min_distance: f32) {
        self.min_distance = min_distance.max(0.001);
    }
    
    /// Gets the minimum distance
    pub fn get_min_distance(&self) -> f32 {
        self.min_distance
    }
    
    /// Adds a body to be affected by this gravity
    pub fn add_body(&mut self, body: BodyHandle) {
        if !self.affected_bodies.contains(&body) {
            self.affected_bodies.push(body);
        }
    }
    
    /// Removes a body from being affected by this gravity
    pub fn remove_body(&mut self, body: BodyHandle) {
        self.affected_bodies.retain(|&b| b != body);
    }
    
    /// Clears all bodies affected by this gravity
    pub fn clear_bodies(&mut self) {
        self.affected_bodies.clear();
    }
}

impl ForceGenerator for PointGravityForce {
    fn generator_type(&self) -> &'static str {
        "PointGravity"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        for &body_handle in &self.affected_bodies {
            let body = match bodies.get_body_mut(body_handle) {
                Ok(body) => body,
                Err(_) => continue,
            };
            
            // Skip if the body is not affected by gravity
            if !body.is_affected_by_gravity() {
                continue;
            }
            
            // Calculate vector from body to gravity source
            let body_pos = body.get_position();
            let direction = self.position - body_pos;
            
            // Calculate distance squared
            let distance_squared = direction.length_squared();
            if distance_squared < self.min_distance * self.min_distance {
                continue; // Too close, skip to avoid instability
            }
            
            // Calculate normalized direction
            let distance = distance_squared.sqrt();
            let direction_normalized = direction / distance;
            
            // F = G * m1 * m2 / r^2
            let mass = body.get_mass();
            let force_magnitude = self.strength * mass / distance_squared;
            let force = direction_normalized * force_magnitude;
            
            // Apply the force at the center of mass
            body.apply_force(force);
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