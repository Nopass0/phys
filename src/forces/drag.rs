use crate::forces::ForceGenerator;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A force generator that simulates drag (air resistance)
#[derive(Debug, Clone)]
pub struct DragForce {
    /// Linear drag coefficient (k1 * v)
    linear_coefficient: f32,
    
    /// Quadratic drag coefficient (k2 * v^2)
    quadratic_coefficient: f32,
    
    /// The bodies affected by this drag
    affected_bodies: Vec<BodyHandle>,
    
    /// Whether the force generator is enabled
    enabled: bool,
}

impl DragForce {
    /// Creates a new drag force generator with the given coefficients
    pub fn new(linear_coefficient: f32, quadratic_coefficient: f32) -> Self {
        Self {
            linear_coefficient: linear_coefficient.max(0.0),
            quadratic_coefficient: quadratic_coefficient.max(0.0),
            affected_bodies: Vec::new(),
            enabled: true,
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
    
    /// Gets the linear drag coefficient
    pub fn get_linear_coefficient(&self) -> f32 {
        self.linear_coefficient
    }
    
    /// Sets the linear drag coefficient
    pub fn set_linear_coefficient(&mut self, coefficient: f32) {
        self.linear_coefficient = coefficient.max(0.0);
    }
    
    /// Gets the quadratic drag coefficient
    pub fn get_quadratic_coefficient(&self) -> f32 {
        self.quadratic_coefficient
    }
    
    /// Sets the quadratic drag coefficient
    pub fn set_quadratic_coefficient(&mut self, coefficient: f32) {
        self.quadratic_coefficient = coefficient.max(0.0);
    }
    
    /// Adds a body to be affected by this drag
    pub fn add_body(&mut self, body: BodyHandle) {
        if !self.affected_bodies.contains(&body) {
            self.affected_bodies.push(body);
        }
    }
    
    /// Removes a body from being affected by this drag
    pub fn remove_body(&mut self, body: BodyHandle) {
        self.affected_bodies.retain(|&b| b != body);
    }
    
    /// Clears all bodies affected by this drag
    pub fn clear_bodies(&mut self) {
        self.affected_bodies.clear();
    }
}

impl ForceGenerator for DragForce {
    fn generator_type(&self) -> &'static str {
        "Drag"
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
            
            // Get linear velocity
            let velocity = body.get_linear_velocity();
            let velocity_magnitude = velocity.length();
            
            if velocity_magnitude < crate::math::EPSILON {
                continue; // No velocity, no drag
            }
            
            // Calculate drag coefficient based on velocity
            // F_drag = -(k1 * v + k2 * v^2) * v_hat
            // where v_hat is the normalized velocity vector
            let drag_coefficient = self.linear_coefficient * velocity_magnitude +
                                  self.quadratic_coefficient * velocity_magnitude * velocity_magnitude;
            
            // Calculate drag force
            let drag_force = velocity.normalize() * -drag_coefficient;
            
            // Apply the force at the center of mass
            body.apply_force(drag_force);
            
            // Apply angular drag to slow down rotation
            let angular_velocity = body.get_angular_velocity();
            let angular_velocity_magnitude = angular_velocity.length();
            
            if angular_velocity_magnitude > crate::math::EPSILON {
                // Apply angular drag scaled by linear coefficients (simplified approximation)
                let angular_drag_coefficient = self.linear_coefficient * angular_velocity_magnitude +
                                             self.quadratic_coefficient * angular_velocity_magnitude * angular_velocity_magnitude;
                
                let angular_drag_torque = angular_velocity.normalize() * -angular_drag_coefficient;
                
                body.apply_torque(angular_drag_torque);
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

/// A force generator that applies aerodynamic forces based on the orientation of a surface
#[derive(Debug, Clone)]
pub struct AerodynamicForce {
    /// The position of the aerodynamic surface in local space
    local_position: Vector3,
    
    /// The normal of the aerodynamic surface in local space
    local_normal: Vector3,
    
    /// The area of the aerodynamic surface
    area: f32,
    
    /// The body the aerodynamic surface is attached to
    body: BodyHandle,
    
    /// The fluid density
    fluid_density: f32,
    
    /// The lift coefficient
    lift_coefficient: f32,
    
    /// The drag coefficient
    drag_coefficient: f32,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// The bodies affected by this force generator (cached for quick lookup)
    affected_bodies: Vec<BodyHandle>,
}

impl AerodynamicForce {
    /// Creates a new aerodynamic force generator
    pub fn new(
        body: BodyHandle,
        local_position: Vector3,
        local_normal: Vector3,
        area: f32,
        fluid_density: f32,
        lift_coefficient: f32,
        drag_coefficient: f32,
    ) -> Self {
        Self {
            local_position,
            local_normal: local_normal.normalize(),
            area: area.max(0.0),
            body,
            fluid_density: fluid_density.max(0.0),
            lift_coefficient: lift_coefficient,
            drag_coefficient: drag_coefficient.max(0.0),
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
    
    /// Gets the local position of the aerodynamic surface
    pub fn get_local_position(&self) -> Vector3 {
        self.local_position
    }
    
    /// Sets the local position of the aerodynamic surface
    pub fn set_local_position(&mut self, position: Vector3) {
        self.local_position = position;
    }
    
    /// Gets the local normal of the aerodynamic surface
    pub fn get_local_normal(&self) -> Vector3 {
        self.local_normal
    }
    
    /// Sets the local normal of the aerodynamic surface
    pub fn set_local_normal(&mut self, normal: Vector3) {
        self.local_normal = normal.normalize();
    }
    
    /// Gets the area of the aerodynamic surface
    pub fn get_area(&self) -> f32 {
        self.area
    }
    
    /// Sets the area of the aerodynamic surface
    pub fn set_area(&mut self, area: f32) {
        self.area = area.max(0.0);
    }
    
    /// Gets the fluid density
    pub fn get_fluid_density(&self) -> f32 {
        self.fluid_density
    }
    
    /// Sets the fluid density
    pub fn set_fluid_density(&mut self, density: f32) {
        self.fluid_density = density.max(0.0);
    }
    
    /// Gets the lift coefficient
    pub fn get_lift_coefficient(&self) -> f32 {
        self.lift_coefficient
    }
    
    /// Sets the lift coefficient
    pub fn set_lift_coefficient(&mut self, coefficient: f32) {
        self.lift_coefficient = coefficient;
    }
    
    /// Gets the drag coefficient
    pub fn get_drag_coefficient(&self) -> f32 {
        self.drag_coefficient
    }
    
    /// Sets the drag coefficient
    pub fn set_drag_coefficient(&mut self, coefficient: f32) {
        self.drag_coefficient = coefficient.max(0.0);
    }
}

impl ForceGenerator for AerodynamicForce {
    fn generator_type(&self) -> &'static str {
        "Aerodynamic"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        let body = match bodies.get_body_mut(self.body) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        // Get the world position of the aerodynamic surface
        let world_position = body.get_transform().transform_point(self.local_position);
        
        // Get the world normal of the aerodynamic surface
        let world_normal = body.get_transform().transform_direction(self.local_normal).normalize();
        
        // Calculate the velocity at the aerodynamic surface
        let body_velocity = body.get_linear_velocity();
        let angular_velocity = body.get_angular_velocity();
        let offset = world_position - body.get_position();
        let point_velocity = body_velocity + angular_velocity.cross(&offset);
        
        let velocity_magnitude = point_velocity.length();
        
        if velocity_magnitude < crate::math::EPSILON {
            return; // No velocity, no aerodynamic forces
        }
        
        // Calculate the normalized velocity
        let velocity_direction = point_velocity / velocity_magnitude;
        
        // Calculate the angle of attack (dot product with normal gives cosine of angle)
        let angle_cosine = world_normal.dot(&-velocity_direction);
        
        // Ensure the angle is in the valid range [-1, 1]
        let angle_cosine_clamped = angle_cosine.clamp(-1.0, 1.0);
        
        // Calculate the force components
        // Dynamic pressure: 0.5 * rho * v^2
        let dynamic_pressure = 0.5 * self.fluid_density * velocity_magnitude * velocity_magnitude;
        
        // Drag force (proportional to sin^2(angle))
        let drag_magnitude = self.drag_coefficient * dynamic_pressure * self.area *
                            (1.0 - angle_cosine_clamped * angle_cosine_clamped);
        
        // Lift force (proportional to sin(angle) * cos(angle))
        let lift_magnitude = self.lift_coefficient * dynamic_pressure * self.area *
                           angle_cosine_clamped * ((1.0 - angle_cosine_clamped * angle_cosine_clamped) as f32).sqrt();
        
        // Calculate the lift direction (perpendicular to velocity and normal)
        let lift_direction = velocity_direction.cross(&world_normal).cross(&velocity_direction).normalize();
        
        // Apply the forces
        let drag_force = velocity_direction * -drag_magnitude;
        let lift_force = lift_direction * lift_magnitude;
        
        let total_force = drag_force + lift_force;
        
        // Apply the total force at the aerodynamic surface position
        body.apply_force_at_point(total_force, world_position);
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