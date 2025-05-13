use crate::forces::ForceGenerator;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::{Vector3, Aabb};
use std::any::Any;

/// Defines the type of force field
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ForceFieldType {
    /// Uniform field that applies the same force to all affected bodies
    Uniform,
    
    /// Radial field that applies force away from or toward a center point
    Radial,
    
    /// Vortex field that applies force to create a swirling motion
    Vortex,
    
    /// Custom field that uses a function to calculate the force
    Custom,
}

/// A force generator that simulates a force field affecting multiple bodies
#[derive(Debug, Clone)]
pub struct ForceField {
    /// The type of force field
    field_type: ForceFieldType,
    
    /// The center of the force field (for radial and vortex fields)
    center: Vector3,
    
    /// The direction of the force field (for uniform fields)
    direction: Vector3,
    
    /// The strength of the force field
    strength: f32,
    
    /// The radius of influence for the force field
    radius: f32,
    
    /// The falloff exponent (1 = linear, 2 = quadratic, etc.)
    falloff_exponent: f32,
    
    /// The bodies affected by this force field
    affected_bodies: Vec<BodyHandle>,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// Optional bounding box to limit the effect of the force field
    bounds: Option<Aabb>,
}

impl ForceField {
    /// Creates a new uniform force field
    pub fn new_uniform(direction: Vector3, strength: f32) -> Self {
        Self {
            field_type: ForceFieldType::Uniform,
            center: Vector3::zero(),
            direction: direction.normalize(),
            strength: strength,
            radius: f32::MAX,
            falloff_exponent: 0.0, // No falloff for uniform field
            affected_bodies: Vec::new(),
            enabled: true,
            bounds: None,
        }
    }
    
    /// Creates a new radial force field (positive strength = away from center, negative = toward center)
    pub fn new_radial(center: Vector3, strength: f32, radius: f32, falloff_exponent: f32) -> Self {
        Self {
            field_type: ForceFieldType::Radial,
            center,
            direction: Vector3::zero(), // Not used for radial fields
            strength,
            radius: radius.max(0.0),
            falloff_exponent: falloff_exponent.max(0.0),
            affected_bodies: Vec::new(),
            enabled: true,
            bounds: None,
        }
    }
    
    /// Creates a new vortex force field
    pub fn new_vortex(center: Vector3, strength: f32, radius: f32, falloff_exponent: f32) -> Self {
        Self {
            field_type: ForceFieldType::Vortex,
            center,
            direction: Vector3::new(0.0, 1.0, 0.0), // Default axis is y-up
            strength,
            radius: radius.max(0.0),
            falloff_exponent: falloff_exponent.max(0.0),
            affected_bodies: Vec::new(),
            enabled: true,
            bounds: None,
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
    
    /// Gets the type of force field
    pub fn get_field_type(&self) -> ForceFieldType {
        self.field_type
    }
    
    /// Sets the type of force field
    pub fn set_field_type(&mut self, field_type: ForceFieldType) {
        self.field_type = field_type;
    }
    
    /// Gets the center of the force field
    pub fn get_center(&self) -> Vector3 {
        self.center
    }
    
    /// Sets the center of the force field
    pub fn set_center(&mut self, center: Vector3) {
        self.center = center;
    }
    
    /// Gets the direction of the force field
    pub fn get_direction(&self) -> Vector3 {
        self.direction
    }
    
    /// Sets the direction of the force field (for uniform fields or axis for vortex fields)
    pub fn set_direction(&mut self, direction: Vector3) {
        self.direction = direction.normalize();
    }
    
    /// Gets the strength of the force field
    pub fn get_strength(&self) -> f32 {
        self.strength
    }
    
    /// Sets the strength of the force field
    pub fn set_strength(&mut self, strength: f32) {
        self.strength = strength;
    }
    
    /// Gets the radius of influence for the force field
    pub fn get_radius(&self) -> f32 {
        self.radius
    }
    
    /// Sets the radius of influence for the force field
    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius.max(0.0);
    }
    
    /// Gets the falloff exponent for the force field
    pub fn get_falloff_exponent(&self) -> f32 {
        self.falloff_exponent
    }
    
    /// Sets the falloff exponent for the force field
    pub fn set_falloff_exponent(&mut self, exponent: f32) {
        self.falloff_exponent = exponent.max(0.0);
    }
    
    /// Sets the bounding box to limit the effect of the force field
    pub fn set_bounds(&mut self, min: Vector3, max: Vector3) {
        self.bounds = Some(Aabb { min, max });
    }
    
    /// Clears the bounding box, making the force field unbounded
    pub fn clear_bounds(&mut self) {
        self.bounds = None;
    }
    
    /// Adds a body to be affected by this force field
    pub fn add_body(&mut self, body: BodyHandle) {
        if !self.affected_bodies.contains(&body) {
            self.affected_bodies.push(body);
        }
    }
    
    /// Removes a body from being affected by this force field
    pub fn remove_body(&mut self, body: BodyHandle) {
        self.affected_bodies.retain(|&b| b != body);
    }
    
    /// Clears all bodies affected by this force field
    pub fn clear_bodies(&mut self) {
        self.affected_bodies.clear();
    }
    
    /// Calculates the force to apply to a body at the given position
    fn calculate_force(&self, position: Vector3) -> Vector3 {
        // Check if the position is within bounds, if bounds are set
        if let Some(bounds) = &self.bounds {
            if position.x < bounds.min.x || position.x > bounds.max.x ||
               position.y < bounds.min.y || position.y > bounds.max.y ||
               position.z < bounds.min.z || position.z > bounds.max.z {
                return Vector3::zero();
            }
        }
        
        match self.field_type {
            ForceFieldType::Uniform => {
                // Uniform field applies the same force everywhere
                self.direction * self.strength
            },
            
            ForceFieldType::Radial => {
                // Calculate the vector from the center to the position
                let offset = position - self.center;
                let distance = offset.length();
                
                // If distance is too small or beyond radius, no force
                if distance < crate::math::EPSILON || distance > self.radius {
                    return Vector3::zero();
                }
                
                // Calculate direction (away from center for positive strength)
                let direction = offset / distance;
                
                // Calculate force magnitude with falloff
                let force_magnitude = if self.falloff_exponent <= crate::math::EPSILON {
                    self.strength // No falloff
                } else {
                    // Apply falloff: strength * (1 - (distance/radius)^exponent)
                    let normalized_distance = distance / self.radius;
                    let falloff = 1.0 - normalized_distance.powf(self.falloff_exponent);
                    self.strength * falloff.max(0.0)
                };
                
                direction * force_magnitude
            },
            
            ForceFieldType::Vortex => {
                // Calculate the vector from the center to the position
                let offset = position - self.center;
                let distance = offset.length();
                
                // If distance is too small or beyond radius, no force
                if distance < crate::math::EPSILON || distance > self.radius {
                    return Vector3::zero();
                }
                
                // Calculate the vortex force direction (perpendicular to both the axis and the offset)
                let vortex_direction = self.direction.cross(&offset).normalize();
                
                // Calculate force magnitude with falloff
                let force_magnitude = if self.falloff_exponent <= crate::math::EPSILON {
                    self.strength // No falloff
                } else {
                    // Apply falloff: strength * (1 - (distance/radius)^exponent)
                    let normalized_distance = distance / self.radius;
                    let falloff = 1.0 - normalized_distance.powf(self.falloff_exponent);
                    self.strength * falloff.max(0.0)
                };
                
                vortex_direction * force_magnitude
            },
            
            ForceFieldType::Custom => {
                // Custom field should be implemented by subclassing
                Vector3::zero()
            }
        }
    }
}

impl ForceGenerator for ForceField {
    fn generator_type(&self) -> &'static str {
        "ForceField"
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
            
            // Get the position of the body
            let position = body.get_position();
            
            // Calculate the force to apply
            let force = self.calculate_force(position);
            
            if force.length_squared() > crate::math::EPSILON {
                // Apply the force at the center of mass
                body.apply_force(force);
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

// Manual implementation of Debug for CustomForceField since we can't derive it
impl std::fmt::Debug for CustomForceField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CustomForceField")
            .field("base", &self.base)
            .field("force_fn", &"<function>")
            .finish()
    }
}

/// A custom force field that allows defining a force calculation function
pub struct CustomForceField {
    /// The base force field
    base: ForceField,

    /// The custom force calculation function
    /// Takes a position and returns a force vector
    #[allow(clippy::type_complexity)]
    force_fn: Box<dyn Fn(Vector3) -> Vector3 + Send + Sync>,
}

impl CustomForceField {
    /// Creates a new custom force field with the given force calculation function
    pub fn new(
        center: Vector3, 
        radius: f32,
        force_fn: impl Fn(Vector3) -> Vector3 + Send + Sync + 'static
    ) -> Self {
        Self {
            base: ForceField {
                field_type: ForceFieldType::Custom,
                center,
                direction: Vector3::zero(),
                strength: 1.0, // Not used directly for custom fields
                radius: radius.max(0.0),
                falloff_exponent: 0.0, // Handled by the force function
                affected_bodies: Vec::new(),
                enabled: true,
                bounds: None,
            },
            force_fn: Box::new(force_fn),
        }
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.base.is_enabled()
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.base.set_enabled(enabled);
    }
    
    /// Gets the center of the force field
    pub fn get_center(&self) -> Vector3 {
        self.base.get_center()
    }
    
    /// Sets the center of the force field
    pub fn set_center(&mut self, center: Vector3) {
        self.base.set_center(center);
    }
    
    /// Gets the radius of influence for the force field
    pub fn get_radius(&self) -> f32 {
        self.base.get_radius()
    }
    
    /// Sets the radius of influence for the force field
    pub fn set_radius(&mut self, radius: f32) {
        self.base.set_radius(radius);
    }
    
    /// Sets the bounding box to limit the effect of the force field
    pub fn set_bounds(&mut self, min: Vector3, max: Vector3) {
        self.base.set_bounds(min, max);
    }
    
    /// Clears the bounding box, making the force field unbounded
    pub fn clear_bounds(&mut self) {
        self.base.clear_bounds();
    }
    
    /// Adds a body to be affected by this force field
    pub fn add_body(&mut self, body: BodyHandle) {
        self.base.add_body(body);
    }
    
    /// Removes a body from being affected by this force field
    pub fn remove_body(&mut self, body: BodyHandle) {
        self.base.remove_body(body);
    }
    
    /// Clears all bodies affected by this force field
    pub fn clear_bodies(&mut self) {
        self.base.clear_bodies();
    }
    
    /// Sets a new force calculation function
    pub fn set_force_function(&mut self, force_fn: impl Fn(Vector3) -> Vector3 + Send + Sync + 'static) {
        self.force_fn = Box::new(force_fn);
    }
}

impl ForceGenerator for CustomForceField {
    fn generator_type(&self) -> &'static str {
        "CustomForceField"
    }
    
    fn update(&mut self, dt: f32) {
        self.base.update(dt);
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.base.is_enabled() {
            return;
        }
        
        for &body_handle in self.base.get_affected_bodies() {
            let body = match bodies.get_body_mut(body_handle) {
                Ok(body) => body,
                Err(_) => continue,
            };
            
            // Get the position of the body
            let position = body.get_position();
            
            // Check if the position is within bounds and radius
            let distance_to_center = (position - self.base.center).length();
            if distance_to_center > self.base.radius {
                continue;
            }
            
            if let Some(bounds) = &self.base.bounds {
                if position.x < bounds.min.x || position.x > bounds.max.x ||
                position.y < bounds.min.y || position.y > bounds.max.y ||
                position.z < bounds.min.z || position.z > bounds.max.z {
                    continue;
                }
            }
            
            // Calculate the force using the custom function
            let force = (self.force_fn)(position);
            
            if force.length_squared() > crate::math::EPSILON {
                // Apply the force at the center of mass
                body.apply_force(force);
            }
        }
    }
    
    fn get_affected_bodies(&self) -> &[BodyHandle] {
        self.base.get_affected_bodies()
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_generator(&self) -> Box<dyn ForceGenerator> {
        // Cloning is not directly supported for custom force fields due to the closure
        // This is a limitation - in a real implementation, you might want to handle this differently
        Box::new(ForceField {
            field_type: ForceFieldType::Custom,
            center: self.base.center,
            direction: self.base.direction,
            strength: self.base.strength,
            radius: self.base.radius,
            falloff_exponent: self.base.falloff_exponent,
            affected_bodies: self.base.affected_bodies.clone(),
            enabled: self.base.enabled,
            bounds: self.base.bounds.clone(),
        })
    }
}