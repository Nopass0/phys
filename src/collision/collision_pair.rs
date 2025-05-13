use crate::core::BodyHandle;
use crate::collision::contact_manifold::ContactManifold;

/// A pair of bodies that could potentially collide
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CollisionPair {
    /// The first body in the collision pair
    pub body_a: BodyHandle,

    /// The second body in the collision pair
    pub body_b: BodyHandle,
}

impl CollisionPair {
    /// Creates a new collision pair
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> Self {
        // Always sort the handles to ensure consistent ordering
        if body_a.0 <= body_b.0 {
            Self { body_a, body_b }
        } else {
            Self { body_a: body_b, body_b: body_a }
        }
    }

    /// Checks if this collision pair contains the specified body
    pub fn contains(&self, body: BodyHandle) -> bool {
        self.body_a == body || self.body_b == body
    }
    
    /// Returns the other body in the pair
    pub fn other(&self, body: BodyHandle) -> Option<BodyHandle> {
        if self.body_a == body {
            Some(self.body_b)
        } else if self.body_b == body {
            Some(self.body_a)
        } else {
            None
        }
    }
}

/// The state of a collision between two bodies
#[derive(Debug, Clone)]
pub struct CollisionState {
    /// The collision pair
    pub pair: CollisionPair,

    /// Whether the bodies were colliding in the previous frame
    pub was_colliding: bool,

    /// Whether the bodies are colliding in the current frame
    pub is_colliding: bool,

    /// The contact manifold for the collision
    pub manifold: ContactManifold,
}

impl CollisionState {
    /// Creates a new collision state
    pub fn new(pair: CollisionPair) -> Self {
        Self {
            pair,
            was_colliding: false,
            is_colliding: false,
            manifold: ContactManifold::new(pair),
        }
    }
    
    /// Updates the collision state for the next frame
    pub fn update(&mut self) {
        self.was_colliding = self.is_colliding;
        
        // Clear the manifold if the bodies are no longer colliding
        if !self.is_colliding {
            self.manifold.clear();
        }
    }
    
    /// Returns whether this is a new collision
    pub fn is_new_collision(&self) -> bool {
        self.is_colliding && !self.was_colliding
    }
    
    /// Returns whether this is a persistent collision
    pub fn is_persistent_collision(&self) -> bool {
        self.is_colliding && self.was_colliding
    }
    
    /// Returns whether this is a collision that has ended
    pub fn is_collision_end(&self) -> bool {
        !self.is_colliding && self.was_colliding
    }
}