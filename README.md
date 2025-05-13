# PhysEngine

PhysEngine is a high-performance physics engine library written in Rust, designed specifically for game development. It provides a modular and extendable architecture that can be easily integrated into game engines.

## Features

- Rigid body dynamics simulation
- Multiple collision shapes (spheres, boxes, capsules, etc.)
- Efficient collision detection algorithms
- Constraint-based physics
- Continuous collision detection
- Stable numerical integration methods
- Collision filtering and layering
- Easy-to-use API
- Cross-platform support

## Getting Started

Add this to your `Cargo.toml`:

```toml
[dependencies]
phys-engine = "0.1.0"
```

## Example Usage

```rust
use phys_engine::{PhysicsWorld, RigidBody, shapes::Box};

fn main() {
    // Create a new physics world
    let mut world = PhysicsWorld::new();

    // Configure gravity
    world.set_gravity([0.0, -9.81, 0.0]);

    // Create a floor
    let floor = RigidBody::new_static(
        Box::new([10.0, 1.0, 10.0]),
        [0.0, -1.0, 0.0],
    );
    world.add_body(floor);

    // Create a falling box
    let box_body = RigidBody::new_dynamic(
        Box::new([1.0, 1.0, 1.0]),
        [0.0, 5.0, 0.0],
    );
    let box_handle = world.add_body(box_body);

    // Simulate 60 steps (1 second at 60 FPS)
    let time_step = 1.0 / 60.0;
    for _ in 0..60 {
        world.step(time_step);
        
        // Get position of the box
        let transform = world.get_transform(box_handle);
        println!("Box position: {:?}", transform.position);
    }
}
```

## Architecture

The engine uses an Entity-Component-System (ECS) inspired architecture for flexibility:

- **Math**: Vector and matrix operations, quaternions
- **Core**: World management, simulation loop, time handling
- **Bodies**: Rigid body dynamics, material properties
- **Shapes**: Collision shapes and volume calculations
- **Collision**: Detection and resolution of collisions
- **Constraints**: Joint constraints, contact constraints
- **Forces**: Gravity, springs, force fields
- **Integration**: Numerical integration methods

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.