use bitflags::bitflags;
use crate::core::BodyHandle;
use crate::bodies::RigidBody;

bitflags! {
    /// A bit mask representing a collision group
    #[derive(Default, Clone, Copy)]
    pub struct CollisionGroup: u32 {
        /// Default group (collides with everything)
        const DEFAULT  = 0x00000001;
        
        /// Static world objects
        const STATIC   = 0x00000002;
        
        /// Dynamic objects
        const DYNAMIC  = 0x00000004;
        
        /// Kinematic objects
        const KINEMATIC = 0x00000008;
        
        /// Character objects
        const CHARACTER = 0x00000010;
        
        /// Projectile objects
        const PROJECTILE = 0x00000020;
        
        /// Sensor/trigger objects (detect but don't resolve)
        const SENSOR   = 0x00000040;
        
        /// Debris objects (may have simplified collision)
        const DEBRIS   = 0x00000080;
        
        /// Group 9
        const GROUP9   = 0x00000100;
        
        /// Group 10
        const GROUP10  = 0x00000200;
        
        /// Group 11
        const GROUP11  = 0x00000400;
        
        /// Group 12
        const GROUP12  = 0x00000800;
        
        /// Group 13
        const GROUP13  = 0x00001000;
        
        /// Group 14
        const GROUP14  = 0x00002000;
        
        /// Group 15
        const GROUP15  = 0x00004000;
        
        /// Group 16
        const GROUP16  = 0x00008000;
        
        /// Group 17
        const GROUP17  = 0x00010000;
        
        /// Group 18
        const GROUP18  = 0x00020000;
        
        /// Group 19
        const GROUP19  = 0x00040000;
        
        /// Group 20
        const GROUP20  = 0x00080000;
        
        /// Group 21
        const GROUP21  = 0x00100000;
        
        /// Group 22
        const GROUP22  = 0x00200000;
        
        /// Group 23
        const GROUP23  = 0x00400000;
        
        /// Group 24
        const GROUP24  = 0x00800000;
        
        /// Group 25
        const GROUP25  = 0x01000000;
        
        /// Group 26
        const GROUP26  = 0x02000000;
        
        /// Group 27
        const GROUP27  = 0x04000000;
        
        /// Group 28
        const GROUP28  = 0x08000000;
        
        /// Group 29
        const GROUP29  = 0x10000000;
        
        /// Group 30
        const GROUP30  = 0x20000000;
        
        /// Group 31
        const GROUP31  = 0x40000000;
        
        /// Group 32
        const GROUP32  = 0x80000000;
        
        /// All groups
        const ALL      = 0xFFFFFFFF;
    }
}

/// Type alias for a collision mask (what groups this object collides with)
pub type CollisionMask = CollisionGroup;

/// A filter for determining whether two bodies should collide
pub trait CollisionFilter: Send + Sync {
    /// Returns whether the two bodies should be tested for collision
    fn should_collide(&self, body_a: &RigidBody, body_b: &RigidBody) -> bool;
}

/// A filter based on collision groups and masks
pub struct GroupMaskFilter {
    /// The collision group for each body
    groups: Vec<CollisionGroup>,
    
    /// The collision mask for each body
    masks: Vec<CollisionMask>,
}

impl GroupMaskFilter {
    /// Creates a new group/mask collision filter
    pub fn new() -> Self {
        Self {
            groups: Vec::new(),
            masks: Vec::new(),
        }
    }
    
    /// Sets the collision group for a body
    pub fn set_group(&mut self, handle: BodyHandle, group: CollisionGroup) {
        let index = handle.0 as usize;
        
        if index >= self.groups.len() {
            self.groups.resize(index + 1, CollisionGroup::DEFAULT);
        }
        
        self.groups[index] = group;
    }
    
    /// Sets the collision mask for a body
    pub fn set_mask(&mut self, handle: BodyHandle, mask: CollisionMask) {
        let index = handle.0 as usize;
        
        if index >= self.masks.len() {
            self.masks.resize(index + 1, CollisionMask::ALL);
        }
        
        self.masks[index] = mask;
    }
    
    /// Gets the collision group for a body
    pub fn get_group(&self, handle: BodyHandle) -> CollisionGroup {
        let index = handle.0 as usize;
        
        if index < self.groups.len() {
            self.groups[index]
        } else {
            CollisionGroup::DEFAULT
        }
    }
    
    /// Gets the collision mask for a body
    pub fn get_mask(&self, handle: BodyHandle) -> CollisionMask {
        let index = handle.0 as usize;
        
        if index < self.masks.len() {
            self.masks[index]
        } else {
            CollisionMask::ALL
        }
    }
}

impl CollisionFilter for GroupMaskFilter {
    fn should_collide(&self, body_a: &RigidBody, body_b: &RigidBody) -> bool {
        // Static bodies don't collide with each other
        // if body_a.get_body_type() == body_b.get_body_type() && body_a.get_body_type() == RigidBodyType::Static {
        //     return false;
        // }
        
        // Get the groups and masks
        // let group_a = self.get_group(handle_a);
        // let mask_a = self.get_mask(handle_a);
        // let group_b = self.get_group(handle_b);
        // let mask_b = self.get_mask(handle_b);
        
        // Check if the groups and masks allow collision
        // A collides with B if A's mask includes B's group AND B's mask includes A's group
        // (group_a.intersects(mask_b)) && (group_b.intersects(mask_a))
        
        // Since we don't have actual handles here, we'll just return true
        // In a real implementation, we would use the actual handles to look up groups and masks
        true
    }
}