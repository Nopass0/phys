use phys_engine::{
    PhysicsWorld, RigidBody, Material,
    shapes::{Sphere, box_shape::BoxShape},
    math::{Vector3, Quaternion},
    core::{GravityType, BodyHandle},
    constraints::BallSocketConstraint,
};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;
use rand::Rng;

// Characters representing 3D depth, from closest to farthest
const DEPTH_CHARS: [char; 8] = ['@', '#', 'O', 'o', '0', '*', '.', ' '];

// Main simulation
fn main() {
    // Create physics world
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));
    
    // Configure physics
    let config = physics_world.get_config_mut();
    config.position_iterations = 12;
    config.velocity_iterations = 10;
    config.collision_margin = 0.01;
    config.use_ccd = true;
    config.time_step = 1.0/60.0;
    
    // Create enclosure (walls, floor, ceiling)
    create_enclosure(&mut physics_world);
    
    // Create pivoting platforms
    let platform_handles = create_pivoting_platforms(&mut physics_world);
    
    // Create bouncing balls with random positions and impulses
    let ball_handles = create_bouncing_balls(&mut physics_world, 10);
    
    // Simulation loop
    let total_frames = 1200; // 40 seconds at 30 FPS
    let frame_time = 1.0 / 30.0; // 30 FPS

    let start_time = std::time::Instant::now();

    for frame in 0..total_frames {
        // Step physics
        physics_world.step(frame_time);
        
        // Clear screen
        print!("\x1B[2J\x1B[1;1H");
        
        // Create grid
        let width = 80;
        let height = 30;
        let mut grid = vec![vec![' '; width]; height];
        
        // Draw enclosure (walls, floor, ceiling)
        draw_enclosure(&mut grid, width, height);
        
        // Draw pivoting platforms
        draw_platforms(&mut grid, width, height, &platform_handles, &physics_world);
        
        // Draw balls
        draw_balls(&mut grid, width, height, &ball_handles, &physics_world);
        
        // Display grid
        for row in &grid {
            let line: String = row.iter().collect();
            println!("{}", line);
        }
        
        // FPS counter and frame info
        println!("Frame: {} / {}", frame + 1, total_frames);
        println!("Press Ctrl+C to exit");
        
        // Wait for next frame
        sleep(Duration::from_millis((frame_time * 1000.0) as u64));
    }
}

// Create walls, floor and ceiling to contain the simulation
fn create_enclosure(physics_world: &mut PhysicsWorld) {
    // Floor
    let floor_shape = Arc::new(BoxShape::new_with_dimensions(80.0, 2.0, 80.0));
    let floor_material = Material::new(1000.0, 0.8, 0.3); // Medium bounce
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(40.0, -1.0, 40.0));
    floor.set_material(floor_material);
    physics_world.add_body(floor);
    
    // Ceiling
    let ceiling_shape = Arc::new(BoxShape::new_with_dimensions(80.0, 2.0, 80.0));
    let ceiling_material = Material::new(1000.0, 0.8, 0.3);
    let mut ceiling = RigidBody::new_static(ceiling_shape, Vector3::new(40.0, 30.0, 40.0));
    ceiling.set_material(ceiling_material);
    physics_world.add_body(ceiling);
    
    // Left wall
    let left_wall_shape = Arc::new(BoxShape::new_with_dimensions(2.0, 30.0, 80.0));
    let left_wall_material = Material::new(1000.0, 0.8, 0.3);
    let mut left_wall = RigidBody::new_static(left_wall_shape, Vector3::new(-1.0, 15.0, 40.0));
    left_wall.set_material(left_wall_material);
    physics_world.add_body(left_wall);
    
    // Right wall
    let right_wall_shape = Arc::new(BoxShape::new_with_dimensions(2.0, 30.0, 80.0));
    let right_wall_material = Material::new(1000.0, 0.8, 0.3);
    let mut right_wall = RigidBody::new_static(right_wall_shape, Vector3::new(81.0, 15.0, 40.0));
    right_wall.set_material(right_wall_material);
    physics_world.add_body(right_wall);
    
    // Back wall
    let back_wall_shape = Arc::new(BoxShape::new_with_dimensions(80.0, 30.0, 2.0));
    let back_wall_material = Material::new(1000.0, 0.8, 0.3);
    let mut back_wall = RigidBody::new_static(back_wall_shape, Vector3::new(40.0, 15.0, -1.0));
    back_wall.set_material(back_wall_material);
    physics_world.add_body(back_wall);
    
    // Front wall (invisible to see the simulation but still has physics)
    let front_wall_shape = Arc::new(BoxShape::new_with_dimensions(80.0, 30.0, 2.0));
    let front_wall_material = Material::new(1000.0, 0.8, 0.3);
    let mut front_wall = RigidBody::new_static(front_wall_shape, Vector3::new(40.0, 15.0, 81.0));
    front_wall.set_material(front_wall_material);
    physics_world.add_body(front_wall);
}

// Create platforms that pivot in the middle with ball-socket joints
fn create_pivoting_platforms(physics_world: &mut PhysicsWorld) -> Vec<BodyHandle> {
    let mut platform_handles = Vec::new();
    
    // Define platform positions in the center of the scene
    let platform_positions = [
        (40.0, 10.0, 40.0),  // x, y, z (center)
        (40.0, 15.0, 40.0),  // x, y, z (center)
        (40.0, 20.0, 40.0),  // x, y, z (center)
    ];

    // Platform properties
    let platform_width = 30.0;
    let platform_height = 0.5;
    let platform_depth = 8.0;
    
    for (i, &(x, y, z)) in platform_positions.iter().enumerate() {
        // Create the platform body
        let platform_shape = Arc::new(BoxShape::new_with_dimensions(
            platform_width, platform_height, platform_depth
        ));
        
        // Material properties - make platforms very lightweight so they pivot easily under weight
        let platform_material = Material::new(
            5.0,  // Very light - will pivot easily under weight
            0.7,  // Higher friction to slow rotation
            0.1   // Low bounce to prevent excessive bouncing
        );
        
        // Create the platform as a dynamic body
        let mut platform = RigidBody::new_dynamic(
            platform_shape, 
            Vector3::new(x, y, z)
        );
        platform.set_material(platform_material);
        
        // Start with perfectly balanced horizontal platforms
        // The rotation will happen naturally due to the weight of the balls
        platform.set_rotation(Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.0));
        
        let platform_handle = physics_world.add_body(platform);
        platform_handles.push(platform_handle);
        
        // Create an anchor point (small static box) in the middle of the platform
        let anchor_shape = Arc::new(BoxShape::new_with_dimensions(1.0, 1.0, 1.0));
        let anchor_material = Material::new(1000.0, 0.8, 0.1);
        let mut anchor = RigidBody::new_static(anchor_shape, Vector3::new(x, y, z));
        anchor.set_material(anchor_material);
        let anchor_handle = physics_world.add_body(anchor);
        
        // Create a ball-socket joint at the center of the platform
        let constraint = Box::new(BallSocketConstraint::new(
            platform_handle,
            anchor_handle,
            Vector3::new(0.0, 0.0, 0.0), // Local anchor point on platform (center)
            Vector3::new(0.0, 0.0, 0.0)  // Local anchor point on static anchor (center)
        ));
        
        // Add the constraint to the physics world
        physics_world.add_constraint(constraint);
    }
    
    platform_handles
}

// Create random bouncing balls
fn create_bouncing_balls(physics_world: &mut PhysicsWorld, num_balls: usize) -> Vec<BodyHandle> {
    let mut ball_handles = Vec::new();
    let mut rng = rand::thread_rng();
    
    for _ in 0..num_balls {
        // Random position within the enclosure
        let x = rng.gen_range(5.0..75.0);
        let y = rng.gen_range(5.0..25.0);
        let z = rng.gen_range(5.0..75.0);
        
        // Random radius between 0.8 and 1.5
        let radius = rng.gen_range(0.8..1.5);
        
        // Create the ball with higher density to make the platforms tilt
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, Vector3::new(x, y, z));

        // Set very heavy material to affect platforms
        let ball_material = Material::new(
            20.0, // Very high density - will cause platforms to tilt
            0.3,  // Low friction
            0.8   // Very bouncy
        );
        ball.set_material(ball_material);
        
        // Add random initial velocities
        let vx = rng.gen_range(-5.0..5.0);
        let vy = rng.gen_range(-2.0..2.0);
        let vz = rng.gen_range(-5.0..5.0);
        ball.set_linear_velocity(Vector3::new(vx, vy, vz));
        
        // Add the ball to the physics world
        let handle = physics_world.add_body(ball);
        ball_handles.push(handle);
    }
    
    ball_handles
}

// Draw the enclosure
fn draw_enclosure(grid: &mut Vec<Vec<char>>, width: usize, height: usize) {
    // Draw floor
    for x in 0..width {
        grid[height - 1][x] = '=';
    }
    
    // Draw ceiling
    for x in 0..width {
        grid[0][x] = '=';
    }
    
    // Draw left wall
    for y in 0..height {
        grid[y][0] = '|';
    }
    
    // Draw right wall
    for y in 0..height {
        grid[y][width - 1] = '|';
    }
}

// Draw the pivoting platforms
fn draw_platforms(
    grid: &mut Vec<Vec<char>>, 
    width: usize, 
    height: usize, 
    platform_handles: &[BodyHandle], 
    physics_world: &PhysicsWorld
) {
    for &handle in platform_handles {
        if let Ok(platform) = physics_world.get_body(handle) {
            let pos = platform.get_position();
            let rot = platform.get_rotation();
            
            // Get the platform dimensions (assuming it's a box)
            let shape = platform.get_shape().unwrap();
            let box_shape = shape.as_any().downcast_ref::<BoxShape>().unwrap();
            let half_extents = box_shape.get_half_extents();
            
            // Calculate the platform endpoints with rotation
            let platform_width = half_extents.x * 2.0;
            
            // Calculate rotation angle from quaternion for Z axis (simplified)
            let angle = 2.0 * (rot.z.atan2(rot.w));
            
            // Draw a line representing the platform
            for i in -(platform_width as i32 / 2)..(platform_width as i32 / 2) {
                // Calculate point with rotation
                let dx = i as f32;
                let dy = dx * angle.sin();

                let x = (pos.x + dx) as usize;
                let y = (height as f32 - (pos.y + dy)) as usize;
                let z_depth = pos.z as usize;

                // Only draw if in bounds
                if x < width && y < height {
                    // Draw character based on platform tilt angle
                    let angle_magnitude = angle.abs();

                    // Choose character based on tilt angle (more visually distinct characters for more tilt)
                    let platform_char = if i == 0 {
                        '+' // Center pivot point
                    } else if angle_magnitude > 0.5 {
                        '≈' // Very tilted
                    } else if angle_magnitude > 0.2 {
                        '~' // Moderately tilted
                    } else {
                        '-' // Barely tilted or flat
                    };

                    grid[y][x] = platform_char;
                }
            }
        }
    }
}

// Draw the bouncing balls
fn draw_balls(
    grid: &mut Vec<Vec<char>>, 
    width: usize, 
    height: usize, 
    ball_handles: &[BodyHandle], 
    physics_world: &PhysicsWorld
) {
    for &handle in ball_handles {
        if let Ok(ball) = physics_world.get_body(handle) {
            let pos = ball.get_position();
            let vel = ball.get_linear_velocity();
            
            // Convert physics coordinates to grid coordinates
            let grid_x = pos.x.round() as usize;
            let grid_y = (height as f32 - pos.y).round() as usize;
            
            // Only draw if in bounds
            if grid_x < width && grid_y < height {
                // Get the shape for 3D visualization
                let shape = ball.get_shape().unwrap();
                
                // Choose character based on z position (depth) and velocity
                let z_depth = pos.z as usize % DEPTH_CHARS.len();
                let vel_magnitude = vel.length();
                
                // Combine depth and velocity for visual effect
                let char_to_draw = if vel_magnitude > 10.0 {
                    '@' // Very fast, use most visible character
                } else {
                    // Base character on Z-depth
                    DEPTH_CHARS[z_depth]
                };
                
                grid[grid_y][grid_x] = char_to_draw;
            }
        }
    }
}