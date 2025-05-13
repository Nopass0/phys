use crate::core::BodyHandle;
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::collections::HashMap;

/// Type of event that can be triggered based on body conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyEventTrigger {
    /// Triggered when a body's velocity exceeds a threshold
    VelocityExceeds,
    /// Triggered when a body's position changes beyond a threshold
    PositionChanges,
    /// Triggered when a body's position enters a specific region
    EntersRegion,
    /// Triggered when a body's position exits a specific region
    ExitsRegion,
    /// Triggered when a body rotates beyond a threshold
    RotationExceeds,
    /// Custom event triggered by user criteria
    Custom,
}

/// Data associated with a triggered event
#[derive(Debug, Clone)]
pub struct BodyEventData {
    /// The body handle that triggered the event
    pub body: BodyHandle,
    /// The type of event that was triggered
    pub trigger_type: BodyEventTrigger,
    /// Current position when the event was triggered
    pub position: Vector3,
    /// Current velocity when the event was triggered
    pub velocity: Vector3,
    /// Additional data associated with the event
    pub data: Option<String>,
}

/// Callback function type for body events
pub type BodyEventCallback = Box<dyn FnMut(&RigidBody, &BodyEventData) + Send + Sync>;

/// A condition for triggering an event
pub struct EventCondition {
    /// Type of trigger for this condition
    pub trigger_type: BodyEventTrigger,
    /// Threshold value for the condition
    pub threshold: f32,
    /// Optional region bounds for region-based conditions
    pub region_min: Option<Vector3>,
    /// Optional region bounds for region-based conditions
    pub region_max: Option<Vector3>,
    /// Custom condition function
    custom_condition: Option<Box<dyn Fn(&RigidBody) -> bool + Send + Sync>>,
    /// User data associated with this condition
    pub user_data: Option<String>,
    /// Callback function to execute when the condition is met
    pub callback: BodyEventCallback,
    /// Previous state for state-based conditions (e.g., entering/exiting regions)
    pub previous_state: bool,
}

impl EventCondition {
    /// Creates a new velocity threshold condition
    pub fn new_velocity_threshold(
        threshold: f32,
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self {
        Self {
            trigger_type: BodyEventTrigger::VelocityExceeds,
            threshold,
            region_min: None,
            region_max: None,
            custom_condition: None,
            user_data,
            callback,
            previous_state: false,
        }
    }

    /// Creates a new position change threshold condition
    pub fn new_position_change(
        threshold: f32, 
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self {
        Self {
            trigger_type: BodyEventTrigger::PositionChanges,
            threshold,
            region_min: None,
            region_max: None,
            custom_condition: None,
            user_data,
            callback,
            previous_state: false,
        }
    }

    /// Creates a region entry condition
    pub fn new_region_entry(
        min: Vector3,
        max: Vector3,
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self {
        Self {
            trigger_type: BodyEventTrigger::EntersRegion,
            threshold: 0.0,
            region_min: Some(min),
            region_max: Some(max),
            custom_condition: None,
            user_data,
            callback,
            previous_state: false,
        }
    }

    /// Creates a region exit condition
    pub fn new_region_exit(
        min: Vector3,
        max: Vector3,
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self {
        Self {
            trigger_type: BodyEventTrigger::ExitsRegion,
            threshold: 0.0,
            region_min: Some(min),
            region_max: Some(max),
            custom_condition: None,
            user_data,
            callback,
            previous_state: true, // Assume initially inside
        }
    }

    /// Creates a rotation threshold condition
    pub fn new_rotation_threshold(
        threshold: f32,
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self {
        Self {
            trigger_type: BodyEventTrigger::RotationExceeds,
            threshold,
            region_min: None,
            region_max: None,
            custom_condition: None,
            user_data,
            callback,
            previous_state: false,
        }
    }

    /// Creates a custom condition
    pub fn new_custom<F>(
        condition: F,
        user_data: Option<String>,
        callback: BodyEventCallback,
    ) -> Self
    where
        F: Fn(&RigidBody) -> bool + Send + Sync + 'static,
    {
        Self {
            trigger_type: BodyEventTrigger::Custom,
            threshold: 0.0,
            region_min: None,
            region_max: None,
            custom_condition: Some(Box::new(condition)),
            user_data,
            callback,
            previous_state: false,
        }
    }

    /// Checks if the condition is met for the given body
    pub fn is_condition_met(&self, body: &RigidBody, last_position: Option<&Vector3>) -> bool {
        match self.trigger_type {
            BodyEventTrigger::VelocityExceeds => {
                let velocity = body.get_linear_velocity();
                velocity.length() > self.threshold
            }
            BodyEventTrigger::PositionChanges => {
                if let Some(last_pos) = last_position {
                    let current_pos = body.get_position();
                    let distance = (current_pos - *last_pos).length();
                    distance > self.threshold
                } else {
                    false
                }
            }
            BodyEventTrigger::EntersRegion => {
                if let (Some(min), Some(max)) = (self.region_min, self.region_max) {
                    let pos = body.get_position();
                    let in_region = pos.x >= min.x && pos.x <= max.x &&
                                    pos.y >= min.y && pos.y <= max.y &&
                                    pos.z >= min.z && pos.z <= max.z;
                    in_region && !self.previous_state
                } else {
                    false
                }
            }
            BodyEventTrigger::ExitsRegion => {
                if let (Some(min), Some(max)) = (self.region_min, self.region_max) {
                    let pos = body.get_position();
                    let in_region = pos.x >= min.x && pos.x <= max.x &&
                                    pos.y >= min.y && pos.y <= max.y &&
                                    pos.z >= min.z && pos.z <= max.z;
                    !in_region && self.previous_state
                } else {
                    false
                }
            }
            BodyEventTrigger::RotationExceeds => {
                let angular_velocity = body.get_angular_velocity();
                angular_velocity.length() > self.threshold
            }
            BodyEventTrigger::Custom => {
                if let Some(ref condition) = self.custom_condition {
                    condition(body)
                } else {
                    false
                }
            }
        }
    }

    /// Updates the state of the condition
    pub fn update_state(&mut self, new_state: bool) {
        self.previous_state = new_state;
    }
}

/// Tracks rigid bodies and triggers events based on their state
pub struct BodyTracker {
    /// Maps body handles to their tracking data
    pub tracked_bodies: HashMap<BodyHandle, Vec<EventCondition>>,
    /// Maps body handles to their last known position
    pub last_positions: HashMap<BodyHandle, Vector3>,
}

impl BodyTracker {
    /// Creates a new body tracker
    pub fn new() -> Self {
        Self {
            tracked_bodies: HashMap::new(),
            last_positions: HashMap::new(),
        }
    }

    /// Adds a tracking condition for a body
    pub fn add_tracking<F>(
        &mut self,
        body_handle: BodyHandle,
        trigger_type: BodyEventTrigger,
        threshold: f32,
        callback: F,
        user_data: Option<String>,
    ) where
        F: FnMut(&RigidBody, &BodyEventData) + Send + Sync + 'static,
    {
        let condition = match trigger_type {
            BodyEventTrigger::VelocityExceeds => {
                EventCondition::new_velocity_threshold(threshold, user_data, Box::new(callback))
            }
            BodyEventTrigger::PositionChanges => {
                EventCondition::new_position_change(threshold, user_data, Box::new(callback))
            }
            BodyEventTrigger::RotationExceeds => {
                EventCondition::new_rotation_threshold(threshold, user_data, Box::new(callback))
            }
            _ => return, // Other types need different constructors
        };

        self.tracked_bodies
            .entry(body_handle)
            .or_insert_with(Vec::new)
            .push(condition);
    }

    /// Adds a region entry tracking condition
    pub fn add_region_entry_tracking<F>(
        &mut self,
        body_handle: BodyHandle,
        min: Vector3,
        max: Vector3,
        callback: F,
        user_data: Option<String>,
    ) where
        F: FnMut(&RigidBody, &BodyEventData) + Send + Sync + 'static,
    {
        let condition = EventCondition::new_region_entry(
            min,
            max,
            user_data,
            Box::new(callback),
        );

        self.tracked_bodies
            .entry(body_handle)
            .or_insert_with(Vec::new)
            .push(condition);
    }

    /// Adds a region exit tracking condition
    pub fn add_region_exit_tracking<F>(
        &mut self,
        body_handle: BodyHandle,
        min: Vector3,
        max: Vector3,
        callback: F,
        user_data: Option<String>,
    ) where
        F: FnMut(&RigidBody, &BodyEventData) + Send + Sync + 'static,
    {
        let condition = EventCondition::new_region_exit(
            min,
            max,
            user_data,
            Box::new(callback),
        );

        self.tracked_bodies
            .entry(body_handle)
            .or_insert_with(Vec::new)
            .push(condition);
    }

    /// Adds a custom tracking condition
    pub fn add_custom_tracking<F, C>(
        &mut self,
        body_handle: BodyHandle,
        condition: C,
        callback: F,
        user_data: Option<String>,
    ) where
        F: FnMut(&RigidBody, &BodyEventData) + Send + Sync + 'static,
        C: Fn(&RigidBody) -> bool + Send + Sync + 'static,
    {
        let event_condition = EventCondition::new_custom(
            condition,
            user_data,
            Box::new(callback),
        );

        self.tracked_bodies
            .entry(body_handle)
            .or_insert_with(Vec::new)
            .push(event_condition);
    }

    /// Removes all tracking conditions for a body
    pub fn remove_tracking(&mut self, body_handle: BodyHandle) {
        self.tracked_bodies.remove(&body_handle);
        self.last_positions.remove(&body_handle);
    }

    /// Updates tracking for all bodies
    pub fn update(&mut self, body_storage: &mut crate::core::BodyStorage<RigidBody>) {
        // First update last positions
        for (handle, _) in self.tracked_bodies.iter() {
            if let Ok(body) = body_storage.get_body(*handle) {
                let position = body.get_position();
                self.last_positions.insert(*handle, position);
            }
        }

        // Collect bodies that need tracking updates
        let handles: Vec<_> = self.tracked_bodies.keys().cloned().collect();

        // Check conditions and trigger callbacks
        for handle in handles {
            if let Ok(body) = body_storage.get_body(handle) {
                let last_position = self.last_positions.get(&handle);
                let position = body.get_position();
                let velocity = body.get_linear_velocity();

                // Get mutable reference to conditions
                if let Some(conditions) = self.tracked_bodies.get_mut(&handle) {
                    for condition in conditions.iter_mut() {
                        let current_state = match condition.trigger_type {
                            BodyEventTrigger::EntersRegion | BodyEventTrigger::ExitsRegion => {
                                if let (Some(min), Some(max)) = (condition.region_min, condition.region_max) {
                                    position.x >= min.x && position.x <= max.x &&
                                    position.y >= min.y && position.y <= max.y &&
                                    position.z >= min.z && position.z <= max.z
                                } else {
                                    false
                                }
                            },
                            _ => false,
                        };

                        let is_met = condition.is_condition_met(body, last_position);
                        if is_met {
                            let event_data = BodyEventData {
                                body: handle,
                                trigger_type: condition.trigger_type,
                                position,
                                velocity,
                                data: condition.user_data.clone(),
                            };

                            // Call the callback
                            (condition.callback)(body, &event_data);
                        }

                        // Update state for state-based conditions
                        if condition.trigger_type == BodyEventTrigger::EntersRegion || 
                           condition.trigger_type == BodyEventTrigger::ExitsRegion {
                            condition.update_state(current_state);
                        }
                    }
                }
            }
        }
    }
}