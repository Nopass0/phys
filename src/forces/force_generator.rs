use crate::core::BodyHandle;
use crate::bodies::RigidBody;
use crate::core::BodyStorage;
use std::any::Any;

/// Base trait for force generators that can apply forces to bodies
pub trait ForceGenerator: Send + Sync + std::fmt::Debug + 'static {
    /// Returns the type name of the force generator
    fn generator_type(&self) -> &'static str;
    
    /// Updates the force generator's internal state if needed
    fn update(&mut self, dt: f32);
    
    /// Applies forces to the bodies in the current state
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, dt: f32);
    
    /// Returns the bodies affected by this force generator
    fn get_affected_bodies(&self) -> &[BodyHandle];
    
    /// Returns true if the force generator affects the given body
    fn affects_body(&self, body: BodyHandle) -> bool {
        self.get_affected_bodies().contains(&body)
    }
    
    /// Returns a dynamic reference to any for downcasting
    fn as_any(&self) -> &dyn Any;
    
    /// Returns a dynamic mutable reference to any for downcasting
    fn as_any_mut(&mut self) -> &mut dyn Any;
    
    /// Clone the force generator to create a new box
    fn clone_generator(&self) -> Box<dyn ForceGenerator>;
}

/// A simple force generator that applies a constant force to a body
#[derive(Debug, Clone)]
pub struct ConstantForceGenerator {
    /// The body to apply the force to
    body: BodyHandle,
    
    /// The force to apply (in world space)
    force: crate::math::Vector3,
    
    /// The point to apply the force at (in local space, if None applies at center of mass)
    local_point: Option<crate::math::Vector3>,
    
    /// Whether the force is enabled
    enabled: bool,
    
    /// The bodies affected by this force generator (cached for quick lookup)
    affected_bodies: Vec<BodyHandle>,
}

impl ConstantForceGenerator {
    /// Creates a new constant force generator
    pub fn new(body: BodyHandle, force: crate::math::Vector3) -> Self {
        Self {
            body,
            force,
            local_point: None,
            enabled: true,
            affected_bodies: vec![body],
        }
    }
    
    /// Creates a new constant force generator that applies the force at a specific point
    pub fn new_at_point(
        body: BodyHandle,
        force: crate::math::Vector3,
        local_point: crate::math::Vector3
    ) -> Self {
        Self {
            body,
            force,
            local_point: Some(local_point),
            enabled: true,
            affected_bodies: vec![body],
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
    
    /// Gets the current force
    pub fn get_force(&self) -> crate::math::Vector3 {
        self.force
    }
    
    /// Sets the force to apply
    pub fn set_force(&mut self, force: crate::math::Vector3) {
        self.force = force;
    }
    
    /// Gets the current local point
    pub fn get_local_point(&self) -> Option<crate::math::Vector3> {
        self.local_point
    }
    
    /// Sets the local point to apply the force at
    pub fn set_local_point(&mut self, local_point: Option<crate::math::Vector3>) {
        self.local_point = local_point;
    }
}

impl ForceGenerator for ConstantForceGenerator {
    fn generator_type(&self) -> &'static str {
        "ConstantForce"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update for constant force
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        let body = match bodies.get_body_mut(self.body) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        match self.local_point {
            Some(local_point) => {
                // Apply force at the specified point in local space
                let world_point = body.get_transform().transform_point(local_point);
                body.apply_force_at_point(self.force, world_point);
            }
            None => {
                // Apply force at center of mass
                body.apply_force(self.force);
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

/// A force generator registry that manages multiple force generators
#[derive(Debug, Default)]
pub struct ForceRegistry {
    /// The list of force generators
    generators: Vec<Box<dyn ForceGenerator>>,
}

impl ForceRegistry {
    /// Creates a new empty force registry
    pub fn new() -> Self {
        Self {
            generators: Vec::new(),
        }
    }
    
    /// Adds a force generator to the registry
    pub fn add_generator(&mut self, generator: Box<dyn ForceGenerator>) {
        self.generators.push(generator);
    }
    
    /// Removes a force generator from the registry
    pub fn remove_generator(&mut self, index: usize) -> Option<Box<dyn ForceGenerator>> {
        if index < self.generators.len() {
            Some(self.generators.remove(index))
        } else {
            None
        }
    }
    
    /// Returns the number of generators in the registry
    pub fn len(&self) -> usize {
        self.generators.len()
    }
    
    /// Returns whether the registry is empty
    pub fn is_empty(&self) -> bool {
        self.generators.is_empty()
    }
    
    /// Returns a generator by index
    pub fn get(&self, index: usize) -> Option<&dyn ForceGenerator> {
        self.generators.get(index).map(|g| g.as_ref())
    }
    
    /// Returns a mutable generator by index
    pub fn get_mut(&mut self, index: usize) -> Option<&mut dyn ForceGenerator> {
        self.generators.get_mut(index).map(|g| g.as_mut())
    }
    
    /// Updates all force generators
    pub fn update(&mut self, dt: f32) {
        for generator in &mut self.generators {
            generator.update(dt);
        }
    }
    
    /// Applies all force generators to the bodies
    pub fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, dt: f32) {
        for generator in &self.generators {
            generator.apply_forces(bodies, dt);
        }
    }
    
    /// Clears all force generators
    pub fn clear(&mut self) {
        self.generators.clear();
    }
    
    /// Removes all force generators affecting a specific body
    pub fn remove_generators_for_body(&mut self, body: BodyHandle) {
        self.generators.retain(|generator| !generator.affects_body(body));
    }
}