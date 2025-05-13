mod rigid_body;
mod material;
mod body_type;

pub use self::rigid_body::{RigidBody, RigidBodyHandle};
pub use self::material::Material;
pub use self::body_type::RigidBodyType;

use crate::math::Vector3;

/// Types of forces that can be applied to a body
#[derive(Debug, Clone, Copy)]
pub enum ForceType {
    /// Force applied at the center of mass
    Force(Vector3),
    
    /// Force applied at a specific point (can cause torque)
    ForceAtPoint {
        /// The force to apply
        force: Vector3,
        
        /// The point to apply the force at, in world space
        point: Vector3,
    },
    
    /// Torque that causes angular acceleration
    Torque(Vector3),
    
    /// Impulse applied at the center of mass (instantaneous change in velocity)
    Impulse(Vector3),
    
    /// Impulse applied at a specific point (can cause angular velocity change)
    ImpulseAtPoint {
        /// The impulse to apply
        impulse: Vector3,
        
        /// The point to apply the impulse at, in world space
        point: Vector3,
    },
    
    /// Angular impulse (instantaneous change in angular velocity)
    AngularImpulse(Vector3),
}

/// Flags for controlling body behavior
pub mod body_flags {
    use bitflags::bitflags;
    
    bitflags! {
        /// Flags for controlling the behavior of rigid bodies
        #[derive(Default)]
        pub struct BodyFlags: u32 {
            /// Body can go to sleep when inactive
            const CAN_SLEEP = 0x01;
            
            /// Body is currently sleeping
            const SLEEPING = 0x02;
            
            /// Body has infinite mass and doesn't respond to forces
            const KINEMATIC = 0x04;
            
            /// Body is affected by gravity
            const AFFECTED_BY_GRAVITY = 0x08;
            
            /// Body is enabled for continuous collision detection
            const CCD_ENABLED = 0x10;
            
            /// Body generates collision events
            const GENERATE_COLLISION_EVENTS = 0x20;
            
            /// Body is a trigger volume (doesn't respond to collisions)
            const TRIGGER = 0x40;
        }
    }
}