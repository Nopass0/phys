pub mod world;
pub mod config;
pub mod storage;
pub mod events;
mod island;
mod scheduler;
pub mod detect_collisions;
pub mod body_tracker;

pub use self::world::PhysicsWorld;
pub use self::config::SimulationConfig;
pub use self::storage::{BodyStorage, ConstraintStorage, Storage};
pub use self::events::{EventQueue, CollisionEvent, BodyEvent};
pub use self::island::Island;
pub use self::scheduler::SimulationScheduler;
pub use self::body_tracker::{BodyTracker, BodyEventTrigger, BodyEventData, EventCondition};

use crate::math::Vector3;

/// A unique identifier for a body in the physics world
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BodyHandle(pub(crate) u32);

/// A unique identifier for a constraint in the physics world
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConstraintHandle(pub(crate) u32);

/// A contact point between two bodies
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// The position of the contact point in world space
    pub position: Vector3,
    
    /// The normal of the contact surface
    pub normal: Vector3,
    
    /// The penetration depth of the contact
    pub penetration: f32,
}

/// The gravity to apply to the physics world
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GravityType {
    /// No gravity
    None,
    
    /// Constant gravity in a direction (typically downward along y-axis)
    Constant(Vector3),
    
    /// Point gravity source (e.g., planet/star)
    Point {
        /// The position of the gravity source
        position: Vector3,
        
        /// The strength of the gravity
        strength: f32,
    },
}

impl Default for GravityType {
    fn default() -> Self {
        // Default gravity is -9.81 in y direction
        Self::Constant(Vector3::new(0.0, -9.81, 0.0))
    }
}