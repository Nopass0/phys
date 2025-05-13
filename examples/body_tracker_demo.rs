use phys_engine::{
    core::{GravityType, BodyTracker, BodyEventTrigger, BodyEventData},
    math::Vector3,
    shapes::{Sphere, box_shape::BoxShape},
    Material, PhysicsWorld, RigidBody, BodyHandle
};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::collections::VecDeque;

// Maximum number of events to remember for display
const MAX_EVENT_HISTORY: usize = 10;

// Store events for display
struct EventHistory {
    events: VecDeque<(String, Instant)>,
}

impl EventHistory {
    fn new() -> Self {
        Self {
            events: VecDeque::with_capacity(MAX_EVENT_HISTORY),
        }
    }

    fn add_event(&mut self, message: String) {
        if self.events.len() >= MAX_EVENT_HISTORY {
            self.events.pop_front();
        }
        self.events.push_back((message, Instant::now()));
    }
}

// Console demo of the RigidBody tracker system
fn main() {
    // Create physics world
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));

    // Configure physics
    {
        let mut config = physics_world.get_config_mut();
        config.position_iterations = 10;
        config.velocity_iterations = 8;
        config.collision_margin = 0.01;
        config.use_ccd = true;
        config.time_step = 1.0 / 60.0;
    }

    // Create floor
    let floor_shape = Arc::new(BoxShape::new_with_dimensions(80.0, 1.0, 10.0));
    let floor_material = Material::new(1000.0, 0.8, 0.5);
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(40.0, 0.0, 0.0));
    floor.set_material(floor_material);
    physics_world.add_body(floor);

    // Create walls
    let left_wall_shape = Arc::new(BoxShape::new_with_dimensions(1.0, 30.0, 10.0));
    let mut left_wall = RigidBody::new_static(left_wall_shape, Vector3::new(0.0, 15.0, 0.0));
    left_wall.set_material(floor_material.clone());
    physics_world.add_body(left_wall);

    let right_wall_shape = Arc::new(BoxShape::new_with_dimensions(1.0, 30.0, 10.0));
    let mut right_wall = RigidBody::new_static(right_wall_shape, Vector3::new(80.0, 15.0, 0.0));
    right_wall.set_material(floor_material.clone());
    physics_world.add_body(right_wall);

    // Create a region box for entry/exit events
    let region_min = Vector3::new(30.0, 0.0, -5.0);
    let region_max = Vector3::new(50.0, 20.0, 5.0);

    // Create event history to store recent events
    let event_history = Arc::new(Mutex::new(EventHistory::new()));

    // Create balls with different properties for tracking
    let num_balls = 5;
    let mut ball_handles = Vec::new();

    for i in 0..num_balls {
        let x = 10.0 + (i as f32 * 15.0);
        let y = 15.0 + (i as f32 * 2.0);

        // Create physics ball
        let radius = 1.0;
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, Vector3::new(x, y, 0.0));

        // Set material properties - different restitution for each ball
        let ball_material = Material::new(1.0, 0.5, 0.5 + (i as f32 * 0.1));
        ball.set_material(ball_material);

        // Add initial velocities
        let vx = (i as f32 * 0.5) - 1.0;
        ball.set_linear_velocity(Vector3::new(vx, 0.0, 0.0));

        let handle = physics_world.add_body(ball);
        ball_handles.push(handle);
    }

    // Create and set up the body tracker
    let mut body_tracker = BodyTracker::new();

    // Set up tracking for different events on different balls
    for (i, &handle) in ball_handles.iter().enumerate() {
        let event_history_clone = event_history.clone();
        
        // 1. Track velocity threshold for ball 0
        if i == 0 {
            body_tracker.add_tracking(
                handle,
                BodyEventTrigger::VelocityExceeds,
                5.0, // Velocity threshold
                move |body, event_data| {
                    let msg = format!(
                        "Ball 1 velocity exceeds 5.0 m/s - current: {:.2} m/s",
                        event_data.velocity.length()
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("High velocity warning".to_string()),
            );
        }
        
        // 2. Track position changes for ball 1
        if i == 1 {
            let event_history_clone = event_history.clone();
            body_tracker.add_tracking(
                handle,
                BodyEventTrigger::PositionChanges,
                3.0, // Position change threshold
                move |body, event_data| {
                    let msg = format!(
                        "Ball 2 moved more than 3.0 units - pos: ({:.1}, {:.1}, {:.1})",
                        event_data.position.x, event_data.position.y, event_data.position.z
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("Large position change".to_string()),
            );
        }
        
        // 3. Track region entry for ball 2
        if i == 2 {
            let event_history_clone = event_history.clone();
            body_tracker.add_region_entry_tracking(
                handle,
                region_min,
                region_max,
                move |body, event_data| {
                    let msg = format!(
                        "Ball 3 entered the center region at pos: ({:.1}, {:.1}, {:.1})",
                        event_data.position.x, event_data.position.y, event_data.position.z
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("Region entry event".to_string()),
            );
        }
        
        // 4. Track region exit for ball 2
        if i == 2 {
            let event_history_clone = event_history.clone();
            body_tracker.add_region_exit_tracking(
                handle,
                region_min,
                region_max,
                move |body, event_data| {
                    let msg = format!(
                        "Ball 3 exited the center region at pos: ({:.1}, {:.1}, {:.1})",
                        event_data.position.x, event_data.position.y, event_data.position.z
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("Region exit event".to_string()),
            );
        }
        
        // 5. Track rotation for ball 3
        if i == 3 {
            let event_history_clone = event_history.clone();
            body_tracker.add_tracking(
                handle,
                BodyEventTrigger::RotationExceeds,
                1.5, // Angular velocity threshold
                move |body, event_data| {
                    let msg = format!(
                        "Ball 4 rotation exceeds 1.5 rad/s - current: {:.2} rad/s",
                        body.get_angular_velocity().length()
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("High rotation warning".to_string()),
            );
        }
        
        // 6. Custom tracking for ball 4 - track when it's near the ground
        if i == 4 {
            let event_history_clone = event_history.clone();
            body_tracker.add_custom_tracking(
                handle,
                |body| {
                    let pos = body.get_position();
                    // Trigger when the ball is close to the ground and moving downward
                    pos.y < 3.0 && body.get_linear_velocity().y < 0.0
                },
                move |body, event_data| {
                    let msg = format!(
                        "Ball 5 is near the ground at height {:.2} and falling",
                        event_data.position.y
                    );
                    event_history_clone.lock().unwrap().add_event(msg);
                },
                Some("Ground proximity alert".to_string()),
            );
        }
    }

    // Simulation loop
    let total_frames = 1000;
    let frame_time = 1.0 / 30.0; // 30 FPS
    let grid_width = 80;
    let grid_height = 24;

    for frame in 0..total_frames {
        // Record start time for frame rate control
        let frame_start = Instant::now();
        
        // Step physics
        physics_world.step(frame_time);
        
        // Custom BodyTracker update since we don't have direct access to BodyStorage
        // First update last positions (same as in BodyTracker)
        for &handle in &ball_handles {
            if let Ok(body) = physics_world.get_body(handle) {
                let position = body.get_position();
                body_tracker.last_positions.insert(handle, position);
            }
        }
        
        // Then check conditions and trigger callbacks
        for &handle in &ball_handles {
            if let Ok(body) = physics_world.get_body(handle) {
                let position = body.get_position();
                let velocity = body.get_linear_velocity();
                let last_position = body_tracker.last_positions.get(&handle);
                
                // If this body has tracking conditions
                if let Some(conditions) = body_tracker.tracked_bodies.get_mut(&handle) {
                    for condition in conditions.iter_mut() {
                        // Check if the body is in the region (for region-based events)
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
                        
                        // Check if condition is met
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
        
        // Clear screen
        print!("\x1B[2J\x1B[1;1H");
        
        // Create grid for visualization
        let mut grid = vec![vec![' '; grid_width]; grid_height];
        
        // Draw floor
        for x in 0..grid_width {
            grid[grid_height - 1][x] = '=';
        }
        
        // Draw walls
        for y in 0..grid_height {
            grid[y][0] = '|';
            grid[y][grid_width - 1] = '|';
        }
        
        // Draw region box (dotted outline)
        let region_left = region_min.x as usize;
        let region_right = region_max.x as usize;
        let region_top = grid_height - region_max.y as usize - 1;
        let region_bottom = grid_height - region_min.y as usize - 1;
        
        for x in region_left..=region_right {
            if x < grid_width {
                if region_top < grid_height {
                    grid[region_top][x] = '-';
                }
                if region_bottom < grid_height {
                    grid[region_bottom][x] = '-';
                }
            }
        }
        
        for y in region_top..=region_bottom {
            if y < grid_height {
                if region_left < grid_width {
                    grid[y][region_left] = ':';
                }
                if region_right < grid_width {
                    grid[y][region_right] = ':';
                }
            }
        }
        
        // Update ball positions and draw
        for (i, handle) in ball_handles.iter().enumerate() {
            if let Ok(ball) = physics_world.get_body(*handle) {
                let pos = ball.get_position();
                
                // Convert physics coordinates to grid coordinates
                let grid_x = pos.x.round() as usize;
                let grid_y = grid_height - pos.y.round() as usize - 1;
                
                // Draw ball if in bounds
                if grid_x < grid_width && grid_y < grid_height {
                    // Use a different character for each ball
                    let ball_char = match i {
                        0 => '1',
                        1 => '2',
                        2 => '3',
                        3 => '4',
                        4 => '5',
                        _ => 'o',
                    };
                    
                    grid[grid_y][grid_x] = ball_char;
                }
            }
        }
        
        // Draw grid
        for row in &grid {
            let line: String = row.iter().collect();
            println!("{}", line);
        }
        
        // Draw event log
        println!("\nEvent Log:");
        println!("----------");
        
        let events = event_history.lock().unwrap();
        if events.events.is_empty() {
            println!("No events yet");
        } else {
            for (idx, (msg, time)) in events.events.iter().enumerate() {
                let age = Instant::now().duration_since(*time).as_millis();
                println!("{}: {} ({} ms ago)", idx + 1, msg, age);
            }
        }
        
        // Display legend
        println!("\nLegend:");
        println!("1 - Ball with velocity tracking");
        println!("2 - Ball with position change tracking");
        println!("3 - Ball with region entry/exit tracking");
        println!("4 - Ball with rotation tracking");
        println!("5 - Ball with custom ground proximity tracking");
        println!("-- - Center region boundaries");
        
        // Display simulation time and frame
        println!("\nSimulation time: {:.2}s (Frame {}/{})", 
                frame as f32 * frame_time, frame + 1, total_frames);
        
        // Wait for next frame (simple frame rate control)
        let elapsed = frame_start.elapsed();
        if elapsed < Duration::from_secs_f32(frame_time) {
            sleep(Duration::from_secs_f32(frame_time) - elapsed);
        }
    }
}