use phys_engine::{
    PhysicsWorld, RigidBody, RigidBodyType, Material,
    shapes::Sphere,
    math::{Vector3, Transform},
};
use std::sync::Arc;
use std::thread::sleep;
use std::time::{Duration, Instant};

fn main() {
    // Create a physics world
    let mut world = PhysicsWorld::new();
    
    // Set gravity
    world.set_gravity(phys_engine::core::GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));
    
    // Create a floor
    let floor_material = Material::new(1000.0, 0.3, 0.5);
    let floor_shape = Arc::new(Sphere::new(20.0));
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(0.0, -19.5, 0.0));
    floor.set_material(floor_material);
    world.add_body(floor);
    
    // Create 10 bouncing balls
    let ball_handles = create_bouncing_balls(&mut world, 10);
    
    // Animation loop
    let time_step = 1.0 / 60.0;
    let mut last_time = Instant::now();
    
    loop {
        // Calculate delta time
        let now = Instant::now();
        let dt = now.duration_since(last_time).as_secs_f32();
        last_time = now;
        
        // Step the simulation
        world.step(time_step);
        
        // Print ball positions
        print!("\x1B[2J\x1B[1;1H"); // Clear terminal
        println!("Bouncing Balls Simulation");
        println!("-------------------------");
        
        for (i, handle) in ball_handles.iter().enumerate() {
            let body = world.get_body(*handle).unwrap();
            let pos = body.get_position();
            let vel = body.get_linear_velocity();
            
            println!("Ball {}: Pos=({:.2}, {:.2}, {:.2}), Vel=({:.2}, {:.2}, {:.2})",
                i + 1, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        }
        
        // Sleep to limit the frame rate
        sleep(Duration::from_millis((time_step * 1000.0) as u64));
        
        // Break after 10 seconds
        if world.get_time() > 10.0 {
            break;
        }
    }
}

fn create_bouncing_balls(world: &mut PhysicsWorld, count: usize) -> Vec<phys_engine::core::BodyHandle> {
    let mut handles = Vec::new();
    
    // Create different ball materials
    let materials = [
        Material::rubber(),  // Rubber (high restitution)
        Material::metal(),   // Metal (medium restitution)
        Material::wood(),    // Wood (low restitution)
    ];
    
    for i in 0..count {
        // Choose a random position
        let x = (i as f32 * 2.0) - (count as f32);
        let y = 5.0 + (i as f32 * 0.5);
        let z = 0.0;
        
        let position = Vector3::new(x, y, z);
        
        // Choose a material
        let material_index = i % materials.len();
        let material = materials[material_index];
        
        // Create the ball
        let radius = 0.5 + (i % 3) as f32 * 0.2;
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, position);
        
        // Set the material
        ball.set_material(material);
        
        // Give it an initial velocity
        let vx = (i as f32 * 0.5) - (count as f32 * 0.25);
        let vy = 0.0;
        let vz = 0.0;
        ball.set_linear_velocity(Vector3::new(vx, vy, vz));
        
        // Add to the world
        let handle = world.add_body(ball);
        handles.push(handle);
    }
    
    handles
}