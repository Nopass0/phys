use phys_engine::{
    core::GravityType, math::Vector3, shapes::Sphere, Material, PhysicsWorld, RigidBody,
};
use std::sync::Arc;
use std::thread::sleep;
use std::time::{Duration, Instant};

// Simple ASCII visualization of bouncing balls
fn main() {
    // Create physics world
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));

    // Configure physics
    let mut config = physics_world.get_config_mut();
    config.position_iterations = 10;
    config.velocity_iterations = 8;
    config.collision_margin = 0.01;
    config.use_ccd = true;
    config.time_step = 1.0 / 60.0;

    // Create floor
    let floor_shape =
        Arc::new(phys_engine::shapes::box_shape::BoxShape::new_with_dimensions(80.0, 1.0, 10.0));
    let floor_material = Material::new(1000.0, 0.8, 0.5);
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(40.0, 0.0, 0.0));
    floor.set_material(floor_material);
    physics_world.add_body(floor);

    // Create balls
    let num_balls = 5;
    let mut ball_handles = Vec::new();

    for i in 0..num_balls {
        let x = 10.0 + (i as f32 * 12.0);
        let y = 20.0 - (i as f32 * 2.0);

        // Create physics ball
        let radius = 1.0;
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, Vector3::new(x, y, 0.0));

        // Set material properties - make it very bouncy
        let ball_material = Material::new(1.0, 0.5, 0.9);
        ball.set_material(ball_material);

        // Add initial velocities
        let vx = (i as f32 * 0.5) - 1.0;
        ball.set_linear_velocity(Vector3::new(vx, 0.0, 0.0));

        let handle = physics_world.add_body(ball);
        ball_handles.push(handle);
    }

    // Simulation loop
    let total_frames = 800; // 10 seconds at 30 FPS
    let frame_time = 1.0 / 30.0; // 30 FPS

    for frame in 0..total_frames {
        // Step physics
        physics_world.step(frame_time);

        // Clear screen
        print!("\x1B[2J\x1B[1;1H");

        // Create grid
        let width = 80;
        let height = 24;
        let mut grid = vec![vec![' '; width]; height];

        // Draw floor
        for x in 0..width {
            grid[height - 1][x] = '=';
        }

        // Update ball positions and draw
        for (i, handle) in ball_handles.iter().enumerate() {
            if let Ok(ball) = physics_world.get_body(*handle) {
                let pos = ball.get_position();
                let vel = ball.get_linear_velocity();

                // Convert physics coordinates to grid coordinates
                let grid_x = pos.x.round() as usize;
                let grid_y = height - pos.y.round() as usize - 1;

                // Draw ball if in bounds
                if grid_x < width && grid_y < height {
                    // Choose character based on velocity for visual effect
                    let char_to_draw = if vel.y.abs() > 5.0 {
                        '@' // Fast vertical
                    } else if vel.y.abs() > 2.0 {
                        'O' // Medium vertical
                    } else {
                        'o' // Slow
                    };

                    grid[grid_y][grid_x] = char_to_draw;
                }

                // Print ball info
                println!(
                    "Ball {}: pos=({:.1},{:.1}), vel=({:.1},{:.1})",
                    i + 1,
                    pos.x,
                    pos.y,
                    vel.x,
                    vel.y
                );
            }
        }

        // Draw grid
        for row in &grid {
            let line: String = row.iter().collect();
            println!("{}", line);
        }

        // Wait for next frame
        sleep(Duration::from_millis((frame_time * 1000.0) as u64));
    }
}
