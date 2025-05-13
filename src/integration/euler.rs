use crate::bodies::{RigidBody, RigidBodyType};
use crate::integration::Integrator;
use crate::math::{Vector3, Quaternion};

/// Simple forward Euler integrator
pub struct EulerIntegrator;

impl EulerIntegrator {
    /// Creates a new Euler integrator
    pub fn new() -> Self {
        Self
    }
}

impl Integrator for EulerIntegrator {
    fn integrate(&mut self, body: &mut RigidBody, dt: f32) {
        // Skip integration for non-dynamic bodies or sleeping bodies
        if body.get_body_type() != RigidBodyType::Dynamic || body.is_sleeping() {
            return;
        }
        
        // Cache properties
        let pos = body.get_position();
        let rot = body.get_rotation();
        let vel = body.get_linear_velocity();
        let ang_vel = body.get_angular_velocity();
        
        // Update position
        let new_pos = pos + vel * dt;
        
        // Update rotation
        let mut new_rot = rot;
        if !ang_vel.is_zero() {
            let angle = ang_vel.length() * dt;
            let axis = ang_vel.normalize();
            let rot_delta = Quaternion::from_axis_angle(axis, angle);
            new_rot = rot_delta * rot;
            new_rot = new_rot.normalize();
        }
        
        // Update transform
        let mut transform = body.get_transform();
        transform.position = new_pos;
        transform.rotation = new_rot;
        body.set_transform(transform);
    }
    
    fn name(&self) -> &str {
        "Euler"
    }
}