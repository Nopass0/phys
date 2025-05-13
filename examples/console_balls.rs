use phys_engine::{
    PhysicsWorld, RigidBody, Material,
    shapes::Sphere,
    math::Vector3,
    core::GravityType,
};
use std::sync::Arc;
use std::io::{stdout, Write};
use std::time::{Duration, Instant};
use std::thread::sleep;
use crossterm::{
    ExecutableCommand, QueueableCommand,
    terminal::{Clear, ClearType, size},
    cursor::{Hide, Show, MoveTo},
    style::{Color, Print, SetForegroundColor, ResetColor},
};

const FRAME_DURATION: Duration = Duration::from_millis(33); // ~30 FPS
const SIMULATION_DURATION: f32 = 15.0; // seconds
const GROUND_HEIGHT: usize = 20;
const NUM_BALLS: usize = 5;
const BALL_CHARS: [char; 5] = ['●', '○', '◆', '◇', '■'];
const BALL_COLORS: [Color; 5] = [
    Color::Red, 
    Color::Green, 
    Color::Blue, 
    Color::Yellow, 
    Color::Magenta
];

struct Ball {
    position: Vector3,
    velocity: Vector3,
    character: char,
    color: Color,
    handle: phys_engine::core::BodyHandle,
}

fn main() -> Result<(), std::io::Error> {
    // Initialize terminal
    let mut stdout = stdout();
    stdout.execute(Hide)?;
    
    // Get terminal size
    let (width, height) = size()?;
    
    // Create physics world with reasonable settings
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));
    
    // Configure physics for stability
    let mut config = physics_world.get_config_mut();
    config.position_iterations = 10;
    config.velocity_iterations = 8;
    config.collision_margin = 0.01;
    config.use_ccd = true;
    config.time_step = 1.0/60.0;
    
    // Create ground
    let ground_shape = Arc::new(phys_engine::shapes::box_shape::BoxShape::new_with_dimensions(
        width as f32, 2.0, 10.0,
    ));
    let ground_material = Material::new(1000.0, 0.8, 0.5);
    let mut ground = RigidBody::new_static(ground_shape, Vector3::new(width as f32 / 2.0, 0.0, 0.0));
    ground.set_material(ground_material);
    physics_world.add_body(ground);
    
    // Create balls
    let mut balls = Vec::new();
    
    for i in 0..NUM_BALLS {
        let x = (i as f32 * (width as f32 / (NUM_BALLS as f32 + 1.0))) + (width as f32 / (NUM_BALLS as f32 + 1.0));
        let y = height as f32 - 5.0 - (i as f32 * 3.0);
        let z = 0.0;
        
        let position = Vector3::new(x, y, z);
        
        // Create physics ball
        let radius = 1.0;
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, position);
        
        // Set material properties
        let ball_material = Material::new(
            1.0, // Normal density 
            0.5, // Medium friction
            0.7, // High bounciness
        );
        
        ball.set_material(ball_material);
        let handle = physics_world.add_body(ball);
        
        // Add to visual balls list
        balls.push(Ball {
            position,
            velocity: Vector3::new(0.0, 0.0, 0.0),
            character: BALL_CHARS[i % BALL_CHARS.len()],
            color: BALL_COLORS[i % BALL_COLORS.len()],
            handle,
        });
    }
    
    // Clear screen
    stdout.execute(Clear(ClearType::All))?;
    
    // Draw ground
    draw_ground(&mut stdout, height as u16 - GROUND_HEIGHT as u16, width as u16)?;
    
    // Main simulation loop
    let start_time = Instant::now();
    let mut frame_time = Instant::now();
    let mut last_update_time = Instant::now();
    
    while start_time.elapsed().as_secs_f32() < SIMULATION_DURATION {
        // Calculate delta time
        let dt = last_update_time.elapsed().as_secs_f32();
        last_update_time = Instant::now();
        
        // Step physics
        physics_world.step(dt);
        
        // Update ball positions from physics
        for ball in &mut balls {
            if let Ok(phys_ball) = physics_world.get_body(ball.handle) {
                ball.position = phys_ball.get_position();
                ball.velocity = phys_ball.get_linear_velocity();
            }
        }
        
        // Draw balls
        for ball in &balls {
            // Clear previous position
            // (In a real application, we would have a more sophisticated way to redraw only changed areas)
            
            // Calculate screen position - y is inverted in terminal
            let screen_x = ball.position.x.round() as u16;
            // Invert y-axis and offset by height (ground at bottom)
            let screen_y = (height as f32 - ball.position.y).round() as u16;
            
            // Only draw if within screen bounds
            if screen_x < width as u16 && screen_y < height as u16 {
                // Draw ball with color
                stdout.queue(MoveTo(screen_x, screen_y))?
                      .queue(SetForegroundColor(ball.color))?
                      .queue(Print(ball.character))?
                      .queue(ResetColor)?;
            }
        }
        
        // Flush output
        stdout.flush()?;
        
        // Timing control
        let elapsed = frame_time.elapsed();
        if elapsed < FRAME_DURATION {
            sleep(FRAME_DURATION - elapsed);
        }
        frame_time = Instant::now();
    }
    
    // Clean up
    stdout.execute(Show)?;
    stdout.execute(MoveTo(0, height as u16 - 1))?;
    
    Ok(())
}

fn draw_ground(stdout: &mut std::io::Stdout, y_pos: u16, width: u16) -> Result<(), std::io::Error> {
    stdout.queue(MoveTo(0, y_pos))?
          .queue(SetForegroundColor(Color::White))?;
    
    for x in 0..width {
        stdout.queue(MoveTo(x, y_pos))?
              .queue(Print("▬"))?;
    }
    
    stdout.queue(ResetColor)?;
    stdout.flush()?;
    
    Ok(())
}