# PhysEngine Usage Guide

This guide provides comprehensive examples and best practices for using the PhysEngine physics library in your game or simulation projects.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Creating a Physics World](#creating-a-physics-world)
3. [Creating Bodies](#creating-bodies)
4. [Shapes](#shapes)
5. [Materials](#materials)
6. [Simulation](#simulation)
7. [Collision Detection](#collision-detection)
8. [Forces and Impulses](#forces-and-impulses)
9. [Constraints](#constraints)
10. [Performance Optimization](#performance-optimization)
11. [Debugging](#debugging)

## Getting Started

Add PhysEngine to your `Cargo.toml`:

```toml
[dependencies]
phys-engine = "0.1.0"
```

Basic example:

```rust
use phys_engine::{PhysicsWorld, RigidBody, shapes::Box};
use std::sync::Arc;

fn main() {
    // Create a physics world
    let mut world = PhysicsWorld::new();
    
    // Create a ground box
    let ground_shape = Arc::new(Box::new_with_dimensions(100.0, 1.0, 100.0));
    let ground = RigidBody::new_static(ground_shape, [0.0, -0.5, 0.0].into());
    world.add_body(ground);
    
    // Create a dynamic box
    let box_shape = Arc::new(Box::new_with_dimensions(1.0, 1.0, 1.0));
    let dynamic_box = RigidBody::new_dynamic(box_shape, [0.0, 5.0, 0.0].into());
    let box_handle = world.add_body(dynamic_box);
    
    // Run the simulation
    let time_step = 1.0 / 60.0;
    for i in 0..300 {
        world.step(time_step);
        
        // Get the box position
        let position = world.get_body(box_handle).unwrap().get_position();
        println!("Box position at step {}: {:?}", i, position);
    }
}
```

## Creating a Physics World

The `PhysicsWorld` is the main container for the simulation:

```rust
// Create a world with default settings
let mut world = PhysicsWorld::new();

// Create a world with custom configuration
let mut config = SimulationConfig::default();
config.gravity = GravityType::Constant([0.0, -5.0, 0.0].into());
config.time_step = 1.0 / 120.0; // Higher precision simulation
config.position_iterations = 20; // More constraint solver iterations
config.velocity_iterations = 10;

let mut world = PhysicsWorld::with_config(config);
```

### Setting Gravity

```rust
// Standard Earth gravity
world.set_gravity(GravityType::Constant([0.0, -9.81, 0.0].into()));

// No gravity
world.set_gravity(GravityType::None);

// Point gravity (e.g., planet)
world.set_gravity(GravityType::Point {
    position: [0.0, 0.0, 0.0].into(),
    strength: 100.0,
});
```

## Creating Bodies

PhysEngine supports three types of rigid bodies:

1. **Dynamic bodies** - Fully simulated, affected by forces and collisions
2. **Kinematic bodies** - Moved manually, affect dynamic bodies
3. **Static bodies** - Never move, not affected by forces or collisions

```rust
// Creating a dynamic body
let body = RigidBody::new_dynamic(
    shape,                    // Shape (Arc<dyn Shape>)
    [0.0, 1.0, 0.0].into(),   // Position
);

// Creating a kinematic body
let body = RigidBody::new_kinematic(
    shape,
    [0.0, 1.0, 0.0].into(),
);

// Creating a static body
let body = RigidBody::new_static(
    shape,
    [0.0, 0.0, 0.0].into(),
);

// Creating a trigger (non-solid collider)
let body = RigidBody::new_trigger(
    shape,
    [0.0, 0.0, 0.0].into(),
);
```

### Body Properties

```rust
// Getting and setting the position
let position = body.get_position();
body.set_position([10.0, 0.0, 0.0].into());

// Getting and setting the rotation
let rotation = body.get_rotation();
body.set_rotation(Quaternion::from_axis_angle([0.0, 1.0, 0.0].into(), 0.5));

// Getting and setting the transform
let transform = body.get_transform();
let new_transform = Transform::new(
    [1.0, 2.0, 3.0].into(),
    Quaternion::from_axis_angle([0.0, 1.0, 0.0].into(), 0.5),
    [1.0, 1.0, 1.0].into(),
);
body.set_transform(new_transform);

// Getting and setting velocities
let vel = body.get_linear_velocity();
body.set_linear_velocity([1.0, 0.0, 0.0].into());

let ang_vel = body.get_angular_velocity();
body.set_angular_velocity([0.0, 1.0, 0.0].into());

// Controlling sleeping
let is_sleeping = body.is_sleeping();
body.put_to_sleep();
body.wake_up();
body.set_can_sleep(false); // Disable sleeping
```

## Shapes

PhysEngine provides various collision shapes:

```rust
// Sphere shape
let sphere = Arc::new(Sphere::new(1.0)); // Radius

// Box shape
let box_shape = Arc::new(Box::new([1.0, 1.0, 1.0].into())); // Half-extents
// or
let box_shape = Arc::new(Box::new_with_dimensions(2.0, 2.0, 2.0)); // Full dimensions

// Plane shape
let plane = Arc::new(Plane::new([0.0, 1.0, 0.0].into(), 0.0)); // Normal and distance

// Capsule shape
let capsule = Arc::new(Capsule::new(0.5, 2.0)); // Radius and height

// Cylinder shape
let cylinder = Arc::new(Cylinder::new(0.5, 2.0)); // Radius and height

// Compound shape (multiple shapes combined)
let mut compound = Compound::new();
compound.add_shape(
    sphere.clone(), 
    Transform::from_position([0.0, 1.0, 0.0].into())
);
compound.add_shape(
    box_shape.clone(),
    Transform::from_position([0.0, -1.0, 0.0].into())
);
let compound_shape = Arc::new(compound);
```

## Materials

Materials define how surfaces interact:

```rust
// Create a custom material
let material = Material::new(
    1000.0,  // Density (kg/m^3)
    0.5,     // Friction (0-1)
    0.3,     // Restitution/bounciness (0-1)
);

// Set rolling friction
let mut material = Material::default();
material.rolling_friction = 0.1;

// Pre-defined materials
let rubber = Material::rubber();
let metal = Material::metal();
let wood = Material::wood();
let ice = Material::ice();
let concrete = Material::concrete();

// Assign material to a body
body.set_material(material);
```

## Simulation

Running the physics simulation:

```rust
// Simple fixed timestep
let time_step = 1.0 / 60.0;
world.step(time_step);

// Variable timestep with sub-stepping
let mut accumulator = 0.0;
let fixed_time_step = 1.0 / 60.0;
let max_time_step = 1.0 / 30.0;

// Game loop
loop {
    let delta_time = get_frame_time(); // From your game engine
    
    // Cap the timestep to avoid tunneling
    let frame_time = delta_time.min(max_time_step);
    accumulator += frame_time;
    
    // Run simulation with fixed steps
    while accumulator >= fixed_time_step {
        world.step(fixed_time_step);
        accumulator -= fixed_time_step;
    }
    
    // Render at interpolated state
    let alpha = accumulator / fixed_time_step;
    render_scene(alpha);
}
```

## Collision Detection

### Collision Events

```rust
// Get collision events after a step
let events = world.get_events();

// Check for specific collision events
while let Some(event) = events.next_collision_event() {
    match event.event_type {
        CollisionEventType::Begin => {
            println!("Collision started between bodies {:?} and {:?}", 
                     event.body_a, event.body_b);
        },
        CollisionEventType::End => {
            println!("Collision ended between bodies {:?} and {:?}", 
                     event.body_a, event.body_b);
        },
        _ => {}
    }
}

// Check collisions directly
if world.are_colliding(body1_handle, body2_handle) {
    println!("Bodies are colliding");
}
```

### Raycasting

```rust
// Create a ray
let ray = Ray::new(
    [0.0, 10.0, 0.0].into(),   // Origin
    [0.0, -1.0, 0.0].into(),   // Direction
);

// Cast the ray
let max_distance = 100.0;
if let Some(hit) = world.raycast(ray, max_distance) {
    println!("Hit body {:?} at position {:?}", hit.body, hit.position);
    println!("Hit normal: {:?}, distance: {}", hit.normal, hit.distance);
}

// Get all hits along a ray
let hits = world.raycast_all(ray, max_distance);
for hit in hits {
    println!("Hit body {:?} at distance {}", hit.body, hit.distance);
}
```

## Forces and Impulses

```rust
// Apply a force at the center of mass
body.apply_force([0.0, 100.0, 0.0].into());

// Apply a force at a specific point
body.apply_force_at_point(
    [10.0, 0.0, 0.0].into(),     // Force
    [0.0, 1.0, 0.0].into(),      // Point
);

// Apply a torque
body.apply_torque([0.0, 10.0, 0.0].into());

// Apply an impulse (instantaneous)
body.apply_impulse([0.0, 5.0, 0.0].into());

// Apply an angular impulse
body.apply_angular_impulse([0.0, 1.0, 0.0].into());
```

## Constraints

Constraints restrict movement between bodies:

```rust
// Create a distance constraint
let constraint = DistanceConstraint::new(
    body_a_handle,
    body_b_handle,
    [0.0, 0.0, 0.0].into(),   // Anchor point on body A (local space)
    [0.0, 0.0, 0.0].into(),   // Anchor point on body B (local space)
    5.0,                       // Distance
);
world.add_constraint(Box::new(constraint));

// Create a hinge constraint
let hinge = HingeConstraint::new(
    body_a_handle,
    body_b_handle,
    [0.0, 0.0, 0.0].into(),   // Pivot in body A's local space
    [0.0, 0.0, 0.0].into(),   // Pivot in body B's local space
    [0.0, 1.0, 0.0].into(),   // Axis in body A's local space
    [0.0, 1.0, 0.0].into(),   // Axis in body B's local space
);
let hinge_handle = world.add_constraint(Box::new(hinge));

// Removing a constraint
world.remove_constraint(hinge_handle);
```

## Performance Optimization

### Body Sleeping

```rust
// Configure body sleeping
let mut config = world.get_config_mut();
config.allow_sleeping = true;
config.linear_sleep_threshold = 0.01;
config.angular_sleep_threshold = 0.01;
config.sleep_time_threshold = 0.5;

// Disable sleeping for certain bodies
let body = world.get_body_mut(handle).unwrap();
body.set_can_sleep(false);

// Check if a body is sleeping
if body.is_sleeping() {
    // Skip processing...
}
```

### Broadphase Collision Detection

```rust
// Use spatial hashing for better performance with many bodies
let cell_size = 2.0;  // Choose based on average body size
let broad_phase = SpatialHashingBroadPhase::new(cell_size);

// Use the Collision Detector with your preferred algorithms
let narrow_phase = GjkNarrowPhase::new();
let contact_generator = GjkContactGenerator::new(4); // Max 4 contacts per pair
let filter = GroupMaskFilter::new();

let detector = CollisionDetector::new(
    Box::new(broad_phase),
    Box::new(narrow_phase),
    Box::new(contact_generator),
    Box::new(filter),
);
```

## Debugging

```rust
// Enable debug rendering by checking body properties
for (handle, body) in bodies.iter() {
    let pos = body.get_position();
    let rot = body.get_rotation();
    
    // Get the shape for rendering
    if let Some(shape) = body.get_shape() {
        if let Some(sphere) = shape.as_any().downcast_ref::<Sphere>() {
            render_sphere(pos, rot, sphere.get_radius());
        } else if let Some(box_shape) = shape.as_any().downcast_ref::<Box>() {
            render_box(pos, rot, box_shape.get_half_extents());
        }
        // etc.
    }
    
    // Visualize velocities
    let vel = body.get_linear_velocity();
    render_arrow(pos, pos + vel, Color::RED);
    
    let ang_vel = body.get_angular_velocity();
    render_arrow(pos, pos + ang_vel, Color::BLUE);
}

// Visualize contacts
for manifold in world.get_contact_manifolds() {
    for contact in &manifold.contacts {
        render_point(contact.position, Color::GREEN);
        render_arrow(contact.position, contact.position + contact.normal, Color::YELLOW);
    }
}
```

## Further Resources

- [API Documentation](./API.md)
- [Physics Concepts](./CONCEPTS.md)
- [Examples](../examples/)
- [Performance Guide](./PERFORMANCE.md)