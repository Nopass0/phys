use crate::bodies::RigidBody;

/// Trait for numerical integration algorithms
pub trait Integrator: Send + Sync {
    /// Integrates a rigid body over a time step
    fn integrate(&mut self, body: &mut RigidBody, dt: f32);

    /// Returns the name of the integrator
    fn name(&self) -> &str;
}