use crate::bodies::RigidBody;
use crate::core::{BodyHandle, BodyStorage, Storage};
use crate::math::{Vector3, Transform};
use crate::shapes::Shape;

/// Result of a CCD sweep test
pub struct SweepResult {
    /// Time of impact (0.0 to 1.0)
    pub hit_time: f32,
    
    /// Hit point in world space
    pub hit_point: Vector3,
    
    /// Surface normal at hit point
    pub hit_normal: Vector3,
    
    /// Whether a hit occurred
    pub hit: bool,
}

impl Default for SweepResult {
    fn default() -> Self {
        Self {
            hit_time: 1.0,
            hit_point: Vector3::zero(),
            hit_normal: Vector3::zero(),
            hit: false,
        }
    }
}

/// Continuous Collision Detection system
pub struct ContinuousCollisionDetection {
    /// Maximum number of iterations for ray-casting
    max_iterations: usize,
    
    /// Tolerance for numerics
    tolerance: f32,
}

impl ContinuousCollisionDetection {
    /// Creates a new CCD system
    pub fn new(max_iterations: usize, tolerance: f32) -> Self {
        Self {
            max_iterations,
            tolerance,
        }
    }
    
    /// Performs CCD between a moving body and all other bodies
    pub fn detect(&self, 
                 body_handle: BodyHandle, 
                 bodies: &BodyStorage<RigidBody>, 
                 dt: f32) -> Option<(BodyHandle, SweepResult)> {
        // Get the moving body
        let body = match bodies.get(body_handle) {
            Some(b) => b,
            None => return None,
        };
        
        // Skip if the body is static or doesn't have CCD enabled
        if !body.is_ccd_enabled() || body.is_sleeping() {
            return None;
        }
        
        // Get the body's current transform and velocity
        let current_transform = body.get_transform();
        let linear_velocity = body.get_linear_velocity();
        let angular_velocity = body.get_angular_velocity();
        
        // Skip if velocity is too low
        if linear_velocity.length_squared() < 0.01 && angular_velocity.length_squared() < 0.01 {
            return None;
        }
        
        // Calculate next transform using explicit integration
        let mut next_transform = current_transform.clone();
        next_transform.position += linear_velocity * dt;
        
        // Approximate rotation change using small angle approximation for simplicity
        // For more accuracy, should use proper quaternion integration
        if angular_velocity.length_squared() > 0.0001 {
            let angle = angular_velocity.length() * dt;
            let axis = angular_velocity.normalize();
            let rotation = crate::math::Quaternion::from_axis_angle(axis, angle);
            next_transform.rotation = rotation * current_transform.rotation;
        }
        
        // Get the body's shape
        let shape = match body.get_shape() {
            Some(s) => s,
            None => return None,
        };
        
        // Test against all other bodies
        let mut earliest_hit: Option<(BodyHandle, SweepResult)> = None;
        
        for (other_handle, other_body) in bodies.iter() {
            // Skip self, static bodies, and bodies without shapes
            if other_handle == body_handle || other_body.is_sleeping() {
                continue;
            }
            
            if let Some(other_shape) = other_body.get_shape() {
                // Perform the sweep test
                let result = self.sweep_test(
                    shape.as_ref(),
                    other_shape.as_ref(),
                    &current_transform,
                    &next_transform,
                    &other_body.get_transform(),
                    dt,
                );
                
                if result.hit {
                    // Update earliest hit if this is earlier than current earliest
                    match earliest_hit {
                        Some((_, ref current_earliest)) if result.hit_time < current_earliest.hit_time => {
                            earliest_hit = Some((other_handle, result));
                        }
                        None => {
                            earliest_hit = Some((other_handle, result));
                        }
                        _ => {}
                    }
                }
            }
        }
        
        earliest_hit
    }
    
    /// Performs a sweep test between two shapes
    pub fn sweep_test(&self,
                     shape_a: &dyn Shape,
                     shape_b: &dyn Shape,
                     transform_a_start: &Transform,
                     transform_a_end: &Transform,
                     transform_b: &Transform,
                     dt: f32) -> SweepResult {
        // Implementation of conservative advancement algorithm
        // We use ray casting from the center of shape A toward its velocity direction
        // and iteratively refine the time of impact
        
        let mut t = 0.0;
        let mut current_transform = transform_a_start.clone();
        let direction = (transform_a_end.position - transform_a_start.position) / dt;
        
        let mut result = SweepResult::default();
        
        // Start conservative advancement loop
        for _ in 0..self.max_iterations {
            // Get closest points between the shapes at current transform
            let support_a = current_transform.position;
            let support_b = transform_b.position;
            
            // Get distance between shapes (simplified - should use actual closest points)
            let delta = support_b - support_a;
            let distance = delta.length();
            
            // If shapes are already penetrating, we're done
            if distance < self.tolerance {
                result.hit = true;
                result.hit_time = t / dt;
                result.hit_point = current_transform.position;
                result.hit_normal = delta.normalize();
                return result;
            }
            
            // Get relative velocity at closest points
            let relative_velocity = direction;
            let velocity_dot_delta = relative_velocity.dot(&delta.normalize());
            
            // If shapes are moving away from each other, no hit will occur
            if velocity_dot_delta <= 0.0 {
                return result;
            }
            
            // Compute time of impact
            let toi = distance / velocity_dot_delta;
            
            // Advance time
            t += toi;
            
            // If we've gone past the end time, no collision this frame
            if t > dt {
                return result;
            }
            
            // Update current transform
            current_transform.position = transform_a_start.position + direction * t;
            
            // Interpolate rotation (simplified - should use proper quaternion interpolation)
            current_transform.rotation = transform_a_start.rotation.slerp(&transform_a_end.rotation, t / dt);
        }
        
        // If we've reached the maximum iterations, we consider this a hit at the current time
        if t < dt {
            result.hit = true;
            result.hit_time = t / dt;
            result.hit_point = current_transform.position;
            
            // Calculate normal (simplified)
            let delta = transform_b.position - current_transform.position;
            result.hit_normal = delta.normalize();
        }
        
        result
    }
    
    /// Resolves collisions detected by CCD
    pub fn resolve_collisions(&self, 
                              body_handle: BodyHandle, 
                              hit: &SweepResult, 
                              bodies: &mut BodyStorage<RigidBody>) {
        // Get the body
        let body = match bodies.get_body_mut(body_handle) {
            Ok(b) => b,
            Err(_) => return,
        };
        
        // Calculate new position
        let current_position = body.get_position();
        let velocity = body.get_linear_velocity();
        
        // Move the body to just before the collision
        let safe_time = hit.hit_time * 0.9; // Back up a bit for safety
        let new_position = current_position + velocity * safe_time;
        
        // Update position
        body.set_position(new_position);
        
        // Apply collision response
        let normal = hit.hit_normal;
        let restitution = body.get_material().restitution;
        
        // Reflect velocity along the normal with restitution
        let dot = velocity.dot(&normal);
        if dot < 0.0 {
            let reflect = velocity - normal * (1.0 + restitution) * dot;
            body.set_linear_velocity(reflect);
        }
    }
}

/// Extension to the physics world for CCD
pub trait CCDPhysicsWorld {
    /// Perform CCD for all bodies
    fn perform_ccd(&mut self, dt: f32);
}

// Implementation of CCD for the physics world
impl CCDPhysicsWorld for crate::core::PhysicsWorld {
    fn perform_ccd(&mut self, dt: f32) {
        // Only perform CCD if enabled in config
        if !self.get_config().use_ccd {
            return;
        }

        // Create the CCD system
        let ccd = ContinuousCollisionDetection::new(10, 0.01);
        
        // Get all bodies that need CCD
        let mut bodies_to_check = Vec::new();
        let body_count = self.body_count();
        
        // First pass - gather bodies that need CCD
        for i in 0..body_count {
            let handle = BodyHandle(i as u32);
            
            // Skip if body doesn't exist
            let body = match self.get_body(handle) {
                Ok(b) => b,
                Err(_) => continue,
            };
            
            // Skip static, sleeping or triggers
            if body.is_sleeping() || body.get_body_type() == crate::bodies::RigidBodyType::Static {
                continue;
            }
            
            // Skip if not CCD enabled or velocity is too low
            if !body.is_ccd_enabled() {
                continue;
            }
            
            let speed = body.get_linear_velocity().length();
            if speed < 1.0 {  // Only check fast bodies
                continue;
            }
            
            bodies_to_check.push(handle);
        }
        
        // Second pass - identify potential collisions
        // Using public API methods to avoid accessing private fields
        let mut collision_events = Vec::new();
        
        for &body_handle in &bodies_to_check {
            // Find the earliest collision with any other body
            let mut earliest_hit: Option<(BodyHandle, SweepResult)> = None;
            
            // We can only get the current body safely once
            let current_body = match self.get_body(body_handle) {
                Ok(body) => body,
                Err(_) => continue,
            };
            
            // Create a velocity-adjusted transform for where body will be at end of step
            let current_transform = current_body.get_transform();
            let vel = current_body.get_linear_velocity();
            let mut target_transform = current_transform.clone();
            target_transform.position += vel * dt;
            
            // Get the body's shape
            let shape = match current_body.get_shape() {
                Some(s) => s,
                None => continue,
            };
            
            // Check against all other bodies
            for i in 0..body_count {
                let other_handle = BodyHandle(i as u32);
                
                // Skip self
                if other_handle == body_handle {
                    continue;
                }
                
                // Get other body
                let other_body = match self.get_body(other_handle) {
                    Ok(body) => body, 
                    Err(_) => continue,
                };
                
                // Skip if sleeping, static, or no shape
                if other_body.is_sleeping() || other_body.is_trigger() {
                    continue;
                }
                
                let other_shape = match other_body.get_shape() {
                    Some(s) => s,
                    None => continue,
                };
                
                // Perform sweep test using our CCD system
                let result = ccd.sweep_test(
                    shape.as_ref(),
                    other_shape.as_ref(),
                    &current_transform,
                    &target_transform,
                    &other_body.get_transform(),
                    dt
                );
                
                // If we hit something, check if it's earlier than our current earliest
                if result.hit {
                    match earliest_hit {
                        Some((_, ref current_earliest)) if result.hit_time < current_earliest.hit_time => {
                            earliest_hit = Some((other_handle, result));
                        }
                        None => {
                            earliest_hit = Some((other_handle, result));
                        }
                        _ => {}
                    }
                }
            }
            
            // If we found a collision, add it to our list
            if let Some((other_handle, hit)) = earliest_hit {
                collision_events.push((body_handle, other_handle, hit));
            }
        }
        
        // Third pass - resolve collisions using public API
        for (body_handle, other_handle, hit) in collision_events {
            // Resolve the collision using velocity reflection
            if let Ok(mut body) = self.get_body_mut(body_handle) {
                // Calculate new position just before collision
                let current_pos = body.get_position();
                let velocity = body.get_linear_velocity();
                
                // Move to just before collision
                let safe_time = hit.hit_time * 0.9; // Back up slightly for safety
                let new_position = current_pos + velocity * dt * safe_time;
                body.set_position(new_position);
                
                // Reflect velocity with restitution
                let normal = hit.hit_normal;
                let restitution = body.get_material().restitution;
                
                let dot = velocity.dot(&normal);
                if dot < 0.0 {
                    let reflect = velocity - normal * (1.0 + restitution) * dot;
                    body.set_linear_velocity(reflect);
                }
                
                // Create collision event
                let should_generate_event = body.generates_collision_events();
                
                // Need to get other body separately due to borrow rules
                let other_generates_events = self.get_body(other_handle)
                    .map(|body| body.generates_collision_events())
                    .unwrap_or(false);
                
                if should_generate_event || other_generates_events {
                    let event = crate::core::events::CollisionEvent {
                        body_a: body_handle,
                        body_b: other_handle,
                        event_type: crate::core::events::CollisionEventType::Begin,
                        contacts: Vec::new(),  // Skip contacts for CCD
                        normal_impulse: None,
                        tangent_impulse: None,
                    };
                    
                    // Use the events queue from the physics world
                    self.get_events_mut().add_collision_event(event);
                }
            }
        }
    }
}