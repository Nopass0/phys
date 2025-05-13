use crate::core::{BodyHandle, ContactPoint};
use std::collections::VecDeque;

/// Types of collision events
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEventType {
    /// Bodies have just started colliding
    Begin,

    /// Bodies are still colliding (persisting contact)
    Persist,

    /// Bodies have just stopped colliding
    End,

    /// Bodies experienced a collision impulse (used for audio/visual effects)
    Impulse,

    /// Continuous collision detection event (high-speed collision)
    CCD,
}

/// A collision event between two bodies
#[derive(Debug, Clone)]
pub struct CollisionEvent {
    /// The type of collision event
    pub event_type: CollisionEventType,
    
    /// The first body in the collision
    pub body_a: BodyHandle,
    
    /// The second body in the collision
    pub body_b: BodyHandle,
    
    /// The contact points of the collision (may be empty for End events)
    pub contacts: Vec<ContactPoint>,
    
    /// The normal impulse magnitude of the collision (for Impulse events)
    pub normal_impulse: Option<f32>,
    
    /// The tangent impulse magnitude of the collision (for Impulse events)
    pub tangent_impulse: Option<f32>,
}

/// Types of body events
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyEventType {
    /// A body has been added to the world
    Added,
    
    /// A body has been removed from the world
    Removed,
    
    /// A body has gone to sleep
    Sleep,
    
    /// A body has been awakened
    Awake,
    
    /// A body's transform has been changed directly
    TransformChanged,
}

/// An event related to a single body
#[derive(Debug, Clone)]
pub struct BodyEvent {
    /// The type of body event
    pub event_type: BodyEventType,
    
    /// The body that the event refers to
    pub body: BodyHandle,
}

/// A queue of physics events
#[derive(Debug, Default)]
pub struct EventQueue {
    /// Collision events
    collision_events: VecDeque<CollisionEvent>,
    
    /// Body events
    body_events: VecDeque<BodyEvent>,
}

impl EventQueue {
    /// Creates a new empty event queue
    pub fn new() -> Self {
        Self {
            collision_events: VecDeque::new(),
            body_events: VecDeque::new(),
        }
    }
    
    /// Adds a collision event to the queue
    pub fn add_collision_event(&mut self, event: CollisionEvent) {
        self.collision_events.push_back(event);
    }
    
    /// Adds a body event to the queue
    pub fn add_body_event(&mut self, event: BodyEvent) {
        self.body_events.push_back(event);
    }
    
    /// Gets the next collision event from the queue
    pub fn next_collision_event(&mut self) -> Option<CollisionEvent> {
        self.collision_events.pop_front()
    }
    
    /// Gets the next body event from the queue
    pub fn next_body_event(&mut self) -> Option<BodyEvent> {
        self.body_events.pop_front()
    }
    
    /// Returns whether there are any collision events in the queue
    pub fn has_collision_events(&self) -> bool {
        !self.collision_events.is_empty()
    }
    
    /// Returns whether there are any body events in the queue
    pub fn has_body_events(&self) -> bool {
        !self.body_events.is_empty()
    }
    
    /// Returns whether the queue is empty
    pub fn is_empty(&self) -> bool {
        self.collision_events.is_empty() && self.body_events.is_empty()
    }
    
    /// Clears all events from the queue
    pub fn clear(&mut self) {
        self.collision_events.clear();
        self.body_events.clear();
    }
    
    /// Gets all collision events of a specific type
    pub fn get_collision_events_of_type(&self, event_type: CollisionEventType) -> Vec<&CollisionEvent> {
        self.collision_events
            .iter()
            .filter(|e| e.event_type == event_type)
            .collect()
    }
    
    /// Gets all body events of a specific type
    pub fn get_body_events_of_type(&self, event_type: BodyEventType) -> Vec<&BodyEvent> {
        self.body_events
            .iter()
            .filter(|e| e.event_type == event_type)
            .collect()
    }
    
    /// Gets all collision events involving a specific body
    pub fn get_collision_events_for_body(&self, body: BodyHandle) -> Vec<&CollisionEvent> {
        self.collision_events
            .iter()
            .filter(|e| e.body_a == body || e.body_b == body)
            .collect()
    }
    
    /// Gets all body events for a specific body
    pub fn get_body_events_for_body(&self, body: BodyHandle) -> Vec<&BodyEvent> {
        self.body_events
            .iter()
            .filter(|e| e.body == body)
            .collect()
    }
}