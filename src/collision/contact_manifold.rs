use crate::math::Vector3;
use crate::core::{BodyHandle, ContactPoint};
use crate::collision::collision_pair::CollisionPair;

/// Maximum number of contact points to store in a manifold
pub const MAX_CONTACT_POINTS: usize = 4;

/// A collection of contact points between two colliding bodies
#[derive(Debug, Clone)]
pub struct ContactManifold {
    /// The collision pair this manifold belongs to
    pub pair: CollisionPair,

    /// The contact points
    pub contacts: Vec<ContactPoint>,

    /// The normal of the contact
    pub normal: Vector3,

    /// The fraction of the frame's timestep at which the collision occurred (0-1)
    pub time_of_impact: f32,

    /// The restitution coefficient for this collision
    pub restitution: f32,

    /// The friction coefficient for this collision
    pub friction: f32,
}

impl ContactManifold {
    /// Creates a new contact manifold for the given collision pair
    pub fn new(pair: CollisionPair) -> Self {
        Self {
            pair,
            contacts: Vec::with_capacity(MAX_CONTACT_POINTS),
            normal: Vector3::zero(),
            time_of_impact: 0.0,
            restitution: 0.0,
            friction: 0.0,
        }
    }
    
    /// Adds a contact point to the manifold
    pub fn add_contact(&mut self, contact: ContactPoint) {
        // If we already have the maximum number of contacts, remove the one with the least penetration
        if self.contacts.len() >= MAX_CONTACT_POINTS {
            // Find the contact with the least penetration
            let mut min_idx = 0;
            let mut min_penetration = self.contacts[0].penetration;
            
            for (i, c) in self.contacts.iter().enumerate().skip(1) {
                if c.penetration < min_penetration {
                    min_idx = i;
                    min_penetration = c.penetration;
                }
            }
            
            // Replace it with the new contact if the new one has more penetration
            if contact.penetration > min_penetration {
                self.contacts[min_idx] = contact;
            }
        } else {
            // Otherwise, just add the contact
            self.contacts.push(contact);
        }
    }
    
    /// Clears all contacts from the manifold
    pub fn clear(&mut self) {
        self.contacts.clear();
        self.normal = Vector3::zero();
        self.time_of_impact = 0.0;
    }
    
    /// Returns whether the manifold is empty
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }
    
    /// Updates the contact manifold for the next frame
    /// This maintains existing contacts and updates their positions based on the new transforms
    pub fn update(&mut self, body_a_moved: bool, body_b_moved: bool) {
        // If neither body moved, there's nothing to update
        if !body_a_moved && !body_b_moved {
            return;
        }
        
        // In a real implementation, we would update the positions of the contact points
        // based on the new transforms of the bodies. This would involve some complex
        // logic to ensure that the contacts are still valid and to adjust their positions.
        
        // For now, we'll just clear the manifold if either body moved significantly
        // In a more sophisticated implementation, we would track and update the contacts
        self.clear();
    }
    
    /// Sets the material properties for this collision
    pub fn set_material_properties(&mut self, restitution: f32, friction: f32) {
        self.restitution = restitution;
        self.friction = friction;
    }
}