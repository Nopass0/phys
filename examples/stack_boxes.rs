use phys_engine::{
    PhysicsWorld, RigidBody, RigidBodyType, Material,
    shapes::box_shape::BoxShape,
    math::{Vector3, Transform, Quaternion},
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
    let floor_material = Material::concrete();
    let floor_shape = Arc::new(BoxShape::new_with_dimensions(20.0, 1.0, 20.0));
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(0.0, -0.5, 0.0));
    floor.set_material(floor_material);
    world.add_body(floor);
    
    // Create a stack of boxes
    create_box_stack(&mut world, 10);
    
    // Animation loop
    let time_step = 1.0 / 60.0;
    let mut last_time = Instant::now();
    let start_time = Instant::now();
    
    while start_time.elapsed().as_secs_f32() < 30.0 {
        // Calculate delta time
        let now = Instant::now();
        let dt = now.duration_since(last_time).as_secs_f32();
        last_time = now;
        
        // Step the simulation
        world.step(time_step);
        
        // Print simulation status
        print!("\x1B[2J\x1B[1;1H"); // Clear terminal
        println!("Box Stack Simulation");
        println!("-------------------");
        println!("Time: {:.2}s", world.get_time());
        println!("Bodies: {}", world.body_count());
        println!("FPS: {:.2}", 1.0 / dt.max(0.001));
        
        // Print some collision events
        let events = world.get_events();
        if events.has_collision_events() {
            println!("\nActive Collisions");
        }
        
        // Sleep to limit the frame rate
        sleep(Duration::from_millis((time_step * 1000.0) as u64));
    }
}

fn create_box_stack(world: &mut PhysicsWorld, height: usize) {
    // Box size
    let box_size = Vector3::new(1.0, 1.0, 1.0);
    let half_box_size = box_size * 0.5;
    
    // Create different box materials
    let materials = [
        Material::wood(),    // Wood
        Material::metal(),   // Metal
    ];
    
    // Create a vertical stack of boxes
    for i in 0..height {
        // Position is increasing in height
        let y = 0.5 + i as f32 * box_size.y;
        let position = Vector3::new(0.0, y, 0.0);
        
        // Choose a material
        let material_index = i % materials.len();
        let material = materials[material_index];
        
        // Create the box
        let box_shape = Arc::new(BoxShape::new(half_box_size));
        let mut rigid_box = RigidBody::new_dynamic(box_shape, position);
        
        // Set the material
        rigid_box.set_material(material);
        
        // Add slight rotation to make it more interesting
        let angle = (i as f32 * 0.1).sin() * 0.05;
        let rotation = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), angle);
        let mut transform = rigid_box.get_transform();
        transform.rotation = rotation;
        rigid_box.set_transform(transform);
        
        // Add to the world
        world.add_body(rigid_box);
    }
    
    // Create a projectile to knock the stack over
    let projectile_shape = Arc::new(BoxShape::new(Vector3::new(0.5, 0.5, 0.5)));
    let mut projectile = RigidBody::new_dynamic(
        projectile_shape,
        Vector3::new(-10.0, 2.0, 0.0),
    );
    
    // Give it a high velocity toward the stack
    projectile.set_linear_velocity(Vector3::new(15.0, 0.0, 0.0));
    
    // Make it heavy
    let mut metal = Material::metal();
    metal.density = 10000.0;
    projectile.set_material(metal);
    
    // Add to the world
    world.add_body(projectile);
}