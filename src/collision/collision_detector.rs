use crate::core::{BodyHandle, BodyStorage, Storage};
use crate::bodies::RigidBody;
use crate::collision::{
    collision_pair::CollisionPair,
    collision_pair::CollisionState,
    collision_filter::CollisionFilter,
    broad_phase::BroadPhase,
    narrow_phase::NarrowPhase,
    contact_generator::ContactGenerator,
    contact_manifold::ContactManifold,
};

use std::collections::HashMap;

/// A system for detecting collisions between bodies
pub struct CollisionDetector {
    /// The broad-phase algorithm to use
    broad_phase: Box<dyn BroadPhase>,
    
    /// The narrow-phase algorithm to use
    narrow_phase: Box<dyn NarrowPhase>,
    
    /// The contact generator to use
    contact_generator: Box<dyn ContactGenerator>,
    
    /// The collision filter to use
    filter: Box<dyn CollisionFilter>,
    
    /// The current collision states
    collision_states: HashMap<CollisionPair, CollisionState>,
    
    /// The current contact manifolds
    contact_manifolds: Vec<ContactManifold>,
}

impl CollisionDetector {
    /// Creates a new collision detector
    pub fn new(
        broad_phase: Box<dyn BroadPhase>,
        narrow_phase: Box<dyn NarrowPhase>,
        contact_generator: Box<dyn ContactGenerator>,
        filter: Box<dyn CollisionFilter>,
    ) -> Self {
        Self {
            broad_phase,
            narrow_phase,
            contact_generator,
            filter,
            collision_states: HashMap::new(),
            contact_manifolds: Vec::new(),
        }
    }
    
    /// Updates the collision detector with the current body states
    pub fn update(&mut self, bodies: &BodyStorage<RigidBody>) {
        // Clear current manifolds
        self.contact_manifolds.clear();

        // Update collision states for the next frame
        for state in self.collision_states.values_mut() {
            state.update();
        }

        // Collect body information
        let body_info: Vec<(BodyHandle, &RigidBody)> = bodies
            .iter()
            .map(|(handle, body)| (handle, body))
            .collect();

        // Update broad-phase
        self.broad_phase.update(&body_info);

        // Get potential collision pairs
        let potential_pairs = self.broad_phase.get_collision_pairs();

        // Check for collisions using narrow-phase
        let mut new_manifolds = self.narrow_phase.detect_collisions(
            &potential_pairs,
            &body_info,
            self.filter.as_ref(),
        );

        // Update collision states
        for manifold in &new_manifolds {
            let pair = manifold.pair;

            // Get or create the collision state
            let state = self.collision_states
                .entry(pair)
                .or_insert_with(|| CollisionState::new(pair));

            // Set the collision state
            state.is_colliding = !manifold.is_empty();

            // Update the manifold
            if state.is_colliding {
                state.manifold = manifold.clone();
            }
        }

        // Generate contacts
        self.contact_generator.generate_contacts(
            &mut new_manifolds,
            &body_info,
        );

        // Store the contact manifolds
        self.contact_manifolds = new_manifolds;
    }
    
    /// Returns the current collision states
    pub fn get_collision_states(&self) -> &HashMap<CollisionPair, CollisionState> {
        &self.collision_states
    }
    
    /// Returns the current contact manifolds
    pub fn get_contact_manifolds(&self) -> &[ContactManifold] {
        &self.contact_manifolds
    }
    
    /// Returns whether two bodies are colliding
    pub fn are_colliding(&self, body_a: BodyHandle, body_b: BodyHandle) -> bool {
        let pair = CollisionPair::new(body_a, body_b);
        
        if let Some(state) = self.collision_states.get(&pair) {
            state.is_colliding
        } else {
            false
        }
    }
    
    /// Returns whether a collision between two bodies has just started
    pub fn is_collision_start(&self, body_a: BodyHandle, body_b: BodyHandle) -> bool {
        let pair = CollisionPair::new(body_a, body_b);
        
        if let Some(state) = self.collision_states.get(&pair) {
            state.is_new_collision()
        } else {
            false
        }
    }
    
    /// Returns whether a collision between two bodies has just ended
    pub fn is_collision_end(&self, body_a: BodyHandle, body_b: BodyHandle) -> bool {
        let pair = CollisionPair::new(body_a, body_b);
        
        if let Some(state) = self.collision_states.get(&pair) {
            state.is_collision_end()
        } else {
            false
        }
    }
    
    /// Returns the contact manifold between two bodies, if any
    pub fn get_contact_manifold(&self, body_a: BodyHandle, body_b: BodyHandle) -> Option<&ContactManifold> {
        let pair = CollisionPair::new(body_a, body_b);
        
        self.contact_manifolds.iter().find(|m| m.pair == pair)
    }
}