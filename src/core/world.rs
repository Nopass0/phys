use crate::core::{
    BodyHandle, ConstraintHandle, SimulationConfig, GravityType,
    EventQueue, BodyEvent, CollisionEvent, Island, BodyStorage,
    ConstraintStorage, ContactPoint, SimulationScheduler,
};
use crate::core::events::{CollisionEventType, BodyEventType};
use crate::core::island::IslandBuilder;
use crate::core::scheduler::SequentialScheduler;
use crate::core::storage::Storage;
use crate::bodies::{RigidBody, RigidBodyType};
use crate::constraints::Constraint;
use crate::math::{Vector3, Transform};
use crate::Result;

#[cfg(feature = "parallel")]
use crate::core::scheduler::parallel::ParallelScheduler;

/// The main physics world class that manages all physics objects and simulation
pub struct PhysicsWorld {
    /// All rigid bodies in the world
    bodies: BodyStorage<RigidBody>,
    
    /// All constraints in the world
    constraints: ConstraintStorage<Box<dyn Constraint>>,
    
    /// Configuration for the simulation
    config: SimulationConfig,
    
    /// Queue of physics events
    events: EventQueue,
    
    /// The total elapsed simulation time
    time: f32,
    
    /// The scheduler for parallel processing
    #[allow(dead_code)]
    scheduler: Box<dyn SimulationScheduler>,
}

impl PhysicsWorld {
    /// Creates a new physics world with default settings
    pub fn new() -> Self {
        Self::with_config(SimulationConfig::default())
    }
    
    /// Creates a new physics world with the given configuration
    pub fn with_config(config: SimulationConfig) -> Self {
        #[cfg(feature = "parallel")]
        let scheduler: Box<dyn SimulationScheduler> = Box::new(ParallelScheduler::new(
            num_cpus::get().max(1),
        ));
        
        #[cfg(not(feature = "parallel"))]
        let scheduler: Box<dyn SimulationScheduler> = Box::new(SequentialScheduler);
        
        Self {
            bodies: BodyStorage::new(),
            constraints: ConstraintStorage::new(),
            config,
            events: EventQueue::new(),
            time: 0.0,
            scheduler,
        }
    }
    
    /// Returns the current simulation time
    pub fn get_time(&self) -> f32 {
        self.time
    }
    
    /// Sets the gravity for the simulation
    pub fn set_gravity(&mut self, gravity: GravityType) {
        self.config.gravity = gravity;
    }
    
    /// Gets the current gravity
    pub fn get_gravity(&self) -> GravityType {
        self.config.gravity
    }
    
    /// Returns a reference to the simulation configuration
    pub fn get_config(&self) -> &SimulationConfig {
        &self.config
    }
    
    /// Returns a mutable reference to the simulation configuration
    pub fn get_config_mut(&mut self) -> &mut SimulationConfig {
        &mut self.config
    }
    
    /// Adds a rigid body to the world and returns its handle
    pub fn add_body(&mut self, body: RigidBody) -> BodyHandle {
        let handle = self.bodies.add(body);
        
        // Add event
        self.events.add_body_event(BodyEvent {
            event_type: BodyEventType::Added,
            body: handle,
        });
        
        handle
    }
    
    /// Removes a rigid body from the world
    pub fn remove_body(&mut self, handle: BodyHandle) -> Result<RigidBody> {
        // Remove any constraints involving this body
        let constraint_handles: Vec<ConstraintHandle> = self.constraints
            .iter()
            .filter_map(|(c_handle, constraint)| {
                if constraint.involves_body(handle) {
                    Some(c_handle)
                } else {
                    None
                }
            })
            .collect();
        
        for c_handle in constraint_handles {
            let _ = self.remove_constraint(c_handle);
        }
        
        // Add event before removal
        self.events.add_body_event(BodyEvent {
            event_type: BodyEventType::Removed,
            body: handle,
        });
        
        // Remove the body
        let body = self.bodies.get_body_mut(handle)?;
        Ok(self.bodies.remove(handle).unwrap())
    }
    
    /// Gets a reference to a rigid body by its handle
    pub fn get_body(&self, handle: BodyHandle) -> Result<&RigidBody> {
        self.bodies.get_body(handle)
    }
    
    /// Gets a mutable reference to a rigid body by its handle
    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Result<&mut RigidBody> {
        self.bodies.get_body_mut(handle)
    }
    
    /// Gets the transform of a rigid body
    pub fn get_transform(&self, handle: BodyHandle) -> Result<Transform> {
        let body = self.bodies.get_body(handle)?;
        Ok(body.get_transform())
    }
    
    /// Sets the transform of a rigid body
    pub fn set_transform(&mut self, handle: BodyHandle, transform: Transform) -> Result<()> {
        let body = self.bodies.get_body_mut(handle)?;
        body.set_transform(transform);
        
        // Wake the body up
        if body.is_sleeping() {
            body.wake_up();
            
            self.events.add_body_event(BodyEvent {
                event_type: BodyEventType::Awake,
                body: handle,
            });
        }
        
        // Add event
        self.events.add_body_event(BodyEvent {
            event_type: BodyEventType::TransformChanged,
            body: handle,
        });
        
        Ok(())
    }
    
    /// Adds a constraint to the world and returns its handle
    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) -> ConstraintHandle {
        // Wake up bodies involved in the constraint
        for &body_handle in constraint.get_bodies() {
            if let Ok(body) = self.bodies.get_body_mut(body_handle) {
                if body.is_sleeping() {
                    body.wake_up();
                    
                    self.events.add_body_event(BodyEvent {
                        event_type: BodyEventType::Awake,
                        body: body_handle,
                    });
                }
            }
        }
        
        self.constraints.add(constraint)
    }
    
    /// Removes a constraint from the world
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> Result<Box<dyn Constraint>> {
        let constraint = self.constraints.get_constraint(handle)?;
        
        // Wake up bodies involved in the constraint
        for &body_handle in constraint.get_bodies() {
            if let Ok(body) = self.bodies.get_body_mut(body_handle) {
                if body.is_sleeping() {
                    body.wake_up();
                    
                    self.events.add_body_event(BodyEvent {
                        event_type: BodyEventType::Awake,
                        body: body_handle,
                    });
                }
            }
        }
        
        Ok(self.constraints.remove(handle).unwrap())
    }
    
    /// Gets a reference to a constraint by its handle
    pub fn get_constraint(&self, handle: ConstraintHandle) -> Result<&Box<dyn Constraint>> {
        self.constraints.get_constraint(handle)
    }
    
    /// Gets a mutable reference to a constraint by its handle
    pub fn get_constraint_mut(&mut self, handle: ConstraintHandle) -> Result<&mut Box<dyn Constraint>> {
        self.constraints.get_constraint_mut(handle)
    }
    
    /// Runs the physics simulation for the given time step
    pub fn step(&mut self, time_step: f32) {
        // Clear events from previous step
        self.events.clear();
        
        // Apply sub-stepping if needed
        let fixed_step = self.config.time_step;
        let max_steps = self.config.max_substeps;
        
        let mut remaining_time = time_step;
        let mut steps = 0;
        
        while remaining_time > 0.0 && steps < max_steps {
            let dt = fixed_step.min(remaining_time);
            
            self.step_simulation(dt);
            
            remaining_time -= dt;
            steps += 1;
        }
        
        // Update total time
        self.time += time_step;
    }
    
    /// Performs a single step of the physics simulation
    fn step_simulation(&mut self, dt: f32) {
        // Update all rigid bodies
        for (handle, body) in self.bodies.iter_mut() {
            if body.get_body_type() == RigidBodyType::Static || body.is_sleeping() {
                continue;
            }

            // Apply gravity
            match self.config.gravity {
                GravityType::None => {},
                GravityType::Constant(gravity) => {
                    body.apply_force(gravity * body.get_mass());
                },
                GravityType::Point { position, strength } => {
                    let to_body = position - body.get_position();
                    let distance_sq = to_body.length_squared();
                    if distance_sq > 0.0 {
                        let distance = distance_sq.sqrt();
                        let direction = to_body / distance;
                        let force = direction * (strength * body.get_mass() / distance_sq);
                        body.apply_force(force);
                    }
                },
            }

            // Apply damping
            body.apply_damping(self.config.linear_damping, self.config.angular_damping);

            // Integrate forces to update velocities
            body.integrate_forces(dt);
        }

        // Build islands for constraint solving
        let islands = self.build_islands();

        // Solve velocity constraints
        for _ in 0..self.config.velocity_iterations {
            for constraint in self.constraints.iter_mut() {
                let (_, constraint) = constraint;
                constraint.solve_velocity(dt, &mut self.bodies);
            }
        }

        // Perform continuous collision detection before position integration
        // This prevents fast-moving objects from tunneling through thin objects
        if self.config.use_ccd {
            use crate::collision::continuous::CCDPhysicsWorld;
            self.perform_ccd(dt);
        }

        // Integrate velocities to update positions
        for (handle, body) in self.bodies.iter_mut() {
            if body.get_body_type() == RigidBodyType::Static || body.is_sleeping() {
                continue;
            }

            body.integrate_velocity(dt);
        }

        // Detect collisions using our optimized implementation
        self.detect_collisions();

        // Solve position constraints
        for _ in 0..self.config.position_iterations {
            for constraint in self.constraints.iter_mut() {
                let (_, constraint) = constraint;
                constraint.solve_position(dt, &mut self.bodies);
            }
        }
        
        // Handle sleeping
        if self.config.allow_sleeping {
            for island in islands {
                let mut can_sleep = true;
                let mut min_sleeping_time = f32::MAX;
                
                for &body_handle in &island.bodies {
                    if let Ok(body) = self.bodies.get_body(body_handle) {
                        if body.get_body_type() == RigidBodyType::Dynamic {
                            let linear_vel_sq = body.get_linear_velocity().length_squared();
                            let angular_vel_sq = body.get_angular_velocity().length_squared();
                            
                            if linear_vel_sq > self.config.linear_sleep_threshold.powi(2) ||
                               angular_vel_sq > self.config.angular_sleep_threshold.powi(2) {
                                can_sleep = false;
                                min_sleeping_time = 0.0;
                                break;
                            }
                            
                            min_sleeping_time = min_sleeping_time.min(body.get_sleeping_time());
                        }
                    }
                }
                
                // Update sleeping time
                if can_sleep {
                    min_sleeping_time += dt;
                    
                    // Put bodies to sleep if they've been inactive long enough
                    if min_sleeping_time >= self.config.sleep_time_threshold {
                        for &body_handle in &island.bodies {
                            if let Ok(body) = self.bodies.get_body_mut(body_handle) {
                                if !body.is_sleeping() && body.get_body_type() == RigidBodyType::Dynamic {
                                    body.put_to_sleep();
                                    
                                    self.events.add_body_event(BodyEvent {
                                        event_type: BodyEventType::Sleep,
                                        body: body_handle,
                                    });
                                }
                            }
                        }
                    } else {
                        // Update sleeping time
                        for &body_handle in &island.bodies {
                            if let Ok(body) = self.bodies.get_body_mut(body_handle) {
                                if body.get_body_type() == RigidBodyType::Dynamic {
                                    body.set_sleeping_time(min_sleeping_time);
                                }
                            }
                        }
                    }
                } else {
                    // Reset sleeping time
                    for &body_handle in &island.bodies {
                        if let Ok(body) = self.bodies.get_body_mut(body_handle) {
                            if body.get_body_type() == RigidBodyType::Dynamic {
                                body.set_sleeping_time(0.0);
                            }
                        }
                    }
                }
            }
        }
    }
    
    /// Builds islands of connected bodies and constraints
    fn build_islands(&self) -> Vec<Island> {
        let mut builder = IslandBuilder::new();
        
        // Add constraints to islands
        for (c_handle, constraint) in self.constraints.iter() {
            let bodies = constraint.get_bodies();
            if bodies.len() == 2 {
                builder.add_connection(bodies[0], bodies[1], c_handle);
            }
        }
        
        // Add single dynamic bodies that aren't in any constraint
        for (b_handle, body) in self.bodies.iter() {
            if body.get_body_type() == RigidBodyType::Dynamic {
                builder.add_single_body(b_handle);
            }
        }
        
        builder.build()
    }
    
    /// Detects collisions between bodies
    
    /// Detects collisions between bodies
    fn detect_collisions(&mut self) {
        // Use helper function from detect_collisions module that's compatible with existing code
        crate::core::detect_collisions::detect_collisions(&mut self.bodies, &mut self.events);
    }
    /// Returns a reference to the event queue
    pub fn get_events(&self) -> &EventQueue {
        &self.events
    }
    
    /// Returns a mutable reference to the event queue
    pub fn get_events_mut(&mut self) -> &mut EventQueue {
        &mut self.events
    }
    
    /// Clears the world of all bodies and constraints
    pub fn clear(&mut self) {
        self.bodies.clear();
        self.constraints.clear();
        self.events.clear();
        self.time = 0.0;
    }
    
    /// Returns the number of bodies in the world
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    
    /// Returns the number of constraints in the world
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }
    
    /// Casts a ray through the world and returns the first hit
    pub fn raycast(&self, _ray: crate::math::Ray, _max_distance: f32) -> Option<RaycastHit> {
        // This is a placeholder for the actual raycast implementation
        // In a real implementation, this would use spatial partitioning and
        // detailed collision detection algorithms
        None
    }
    
    /// Casts a ray through the world and returns all hits
    pub fn raycast_all(&self, _ray: crate::math::Ray, _max_distance: f32) -> Vec<RaycastHit> {
        // This is a placeholder for the actual raycast implementation
        Vec::new()
    }
}

/// The result of a raycast
#[derive(Debug, Clone)]
pub struct RaycastHit {
    /// The handle of the body that was hit
    pub body: BodyHandle,
    
    /// The position of the hit in world space
    pub position: Vector3,
    
    /// The normal of the surface at the hit point
    pub normal: Vector3,
    
    /// The distance from the ray origin to the hit point
    pub distance: f32,
}