use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use std::any::Any;

/// Base trait for physics constraints
pub trait Constraint: Send + Sync + 'static {
    /// Returns the type name of the constraint
    fn constraint_type(&self) -> &'static str;
    
    /// Returns the bodies involved in the constraint
    fn get_bodies(&self) -> &[BodyHandle];
    
    /// Checks if the constraint involves a specific body
    fn involves_body(&self, body: BodyHandle) -> bool {
        self.get_bodies().contains(&body)
    }
    
    /// Prepares the constraint for solving
    fn prepare(&mut self, bodies: &BodyStorage<RigidBody>);
    
    /// Solves the velocity constraint
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>);
    
    /// Solves the position constraint
    fn solve_position(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>);
    
    /// Returns a dynamic reference to any for downcasting
    fn as_any(&self) -> &dyn Any;
    
    /// Returns a dynamic mutable reference to any for downcasting
    fn as_any_mut(&mut self) -> &mut dyn Any;
    
    /// Clone the constraint to create a new box
    fn clone_constraint(&self) -> Box<dyn Constraint>;
}