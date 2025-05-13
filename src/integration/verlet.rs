use crate::bodies::{RigidBody, RigidBodyType};
use crate::integration::Integrator;
use crate::math::{Vector3, Quaternion};

/// Velocity Verlet integrator for more stable physics simulations
pub struct VerletIntegrator {
    /// Previous positions of bodies
    previous_positions: std::collections::HashMap<u32, Vector3>,
    
    /// Previous velocities for computing acceleration
    previous_velocities: std::collections::HashMap<u32, Vector3>,
}

impl VerletIntegrator {
    /// Creates a new Velocity Verlet integrator
    pub fn new() -> Self {
        Self {
            previous_positions: std::collections::HashMap::new(),
            previous_velocities: std::collections::HashMap::new(),
        }
    }
    
    /// Gets the body key for the hashmap
    fn get_body_key(&self, body: &RigidBody) -> u32 {
        // In a real implementation, we would use a proper handle
        // Here we'll just use a hash of the position
        let pos = body.get_position();
        (pos.x.to_bits() ^ pos.y.to_bits() ^ pos.z.to_bits())
    }
}

impl Integrator for VerletIntegrator {
    fn integrate(&mut self, body: &mut RigidBody, dt: f32) {
        // Skip integration for non-dynamic bodies or sleeping bodies
        if body.get_body_type() != RigidBodyType::Dynamic || body.is_sleeping() {
            return;
        }

        // Get the body key
        let key = self.get_body_key(body);

        // Cache current state
        let pos = body.get_position();
        let rot = body.get_rotation();
        let vel = body.get_linear_velocity();
        let ang_vel = body.get_angular_velocity();

        // Get previous position
        let prev_pos = self.previous_positions.get(&key).copied().unwrap_or(pos);

        // Get previous velocity for acceleration calculation
        let prev_vel = self.previous_velocities.get(&key).copied().unwrap_or(vel);

        // Calculate acceleration
        let accel = (vel - prev_vel) / dt;

        // Update position using Verlet integration
        let new_pos = pos * 2.0 - prev_pos + accel * dt * dt;

        // Update rotation (we'll use simple angular velocity integration for rotation)
        let mut new_rot = rot;
        if !ang_vel.is_zero() {
            let angle = ang_vel.length() * dt;
            let axis = ang_vel.normalize();
            let rot_delta = Quaternion::from_axis_angle(axis, angle);
            new_rot = rot_delta * rot;
            new_rot = new_rot.normalize();
        }

        // Calculate new velocity
        let new_vel = (new_pos - pos) / dt;

        // Update transform
        let mut transform = body.get_transform();
        transform.position = new_pos;
        transform.rotation = new_rot;
        body.set_transform(transform);

        // Update velocity
        body.set_linear_velocity(new_vel);

        // Store current position and velocity for next frame
        self.previous_positions.insert(key, pos);
        self.previous_velocities.insert(key, vel);
    }
    
    fn name(&self) -> &str {
        "Verlet"
    }
}