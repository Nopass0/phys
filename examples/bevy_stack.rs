use bevy::{
    prelude::*,
    input::InputPlugin,
};
use phys_engine::{
    PhysicsWorld, RigidBody, RigidBodyType, Material,
    shapes::box_shape::BoxShape,
    math::{Vector3, Quaternion as PhysQuaternion},
    core::GravityType,
};
use std::sync::Arc;

// Resource to store our physics world
#[derive(Resource)]
struct PhysicsResource {
    world: PhysicsWorld,
    box_handles: Vec<phys_engine::core::BodyHandle>,
    floor_handle: phys_engine::core::BodyHandle,
}

// Component to link Bevy entities with physics bodies
#[derive(Component)]
struct PhysicsBody {
    handle: phys_engine::core::BodyHandle,
}

#[derive(Component)]
struct Floor;

#[derive(Component)]
struct BoxBody;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .add_systems(Startup, setup)
        .add_systems(Update, (
            physics_step,
            update_transforms,
            camera_controls,
            spawn_box_on_click,
        ))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Set up camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 8.0, 15.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
        ..default()
    });

    // Add lighting
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Add ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.2,
    });

    // Create physics world with improved settings
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));

    // Set drastically improved simulation parameters to prevent objects falling through
    let mut config = physics_world.get_config_mut();
    config.position_iterations = 50;       // Much higher position iterations for better stability
    config.velocity_iterations = 40;       // Much higher velocity iterations for better contacts
    config.collision_margin = 0.05;        // Significantly increased collision margin
    config.contact_penetration_threshold = 0.0001;  // Very low threshold to detect tiny penetrations
    config.constraint_bias_factor = 0.8;   // Very high bias factor for aggressive correction
    config.erp_factor = 0.8;               // Very high ERP for aggressive position correction
    config.cfm_factor = 0.00001;          // Tiny CFM value for very rigid constraints
    config.use_ccd = true;                // Enable continuous collision detection
    config.restitution_velocity_threshold = 0.01;  // Lower threshold for applying restitution

    // Create two thick floors stacked for better stability

    // Bottom floor (completely fixed and extremely thick)
    let bottom_floor_shape = Arc::new(BoxShape::new_with_dimensions(50.0, 10.0, 50.0));
    let bottom_floor_material = Material::new(10000.0, 1.0, 0.0); // Maximum friction, zero bounce
    let mut bottom_floor = RigidBody::new_static(bottom_floor_shape, Vector3::new(0.0, -7.0, 0.0));
    bottom_floor.set_material(bottom_floor_material);
    physics_world.add_body(bottom_floor);

    // Top floor (where boxes will stack)
    let floor_shape = Arc::new(BoxShape::new_with_dimensions(40.0, 5.0, 40.0));
    let floor_material = Material::new(5000.0, 0.99, 0.01); // Ultra dense material with maximum friction and almost no bounce
    let mut floor = RigidBody::new_static(floor_shape, Vector3::new(0.0, -2.5, 0.0));
    floor.set_material(floor_material);
    let floor_handle = physics_world.add_body(floor);

    // Spawn bottom floor in Bevy
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(50.0, 10.0, 50.0))),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.1, 0.1, 0.1),
                ..default()
            }),
            transform: Transform::from_xyz(0.0, -7.0, 0.0),
            ..default()
        },
        PhysicsBody {
            handle: floor_handle,
        },
        Floor,
    ));

    // Spawn top floor in Bevy
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(40.0, 5.0, 40.0))),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.2, 0.2, 0.2),
                ..default()
            }),
            transform: Transform::from_xyz(0.0, -2.5, 0.0),
            ..default()
        },
        PhysicsBody {
            handle: floor_handle,
        },
        Floor,
    ));

    // Create initial stack of boxes
    let mut box_handles = Vec::new();
    let box_colors = [
        Color::rgb(0.8, 0.2, 0.2), // Red
        Color::rgb(0.2, 0.8, 0.2), // Green
        Color::rgb(0.2, 0.2, 0.8), // Blue
        Color::rgb(0.8, 0.8, 0.2), // Yellow
        Color::rgb(0.8, 0.2, 0.8), // Purple
    ];

    // Stack parameters
    let stack_height = 6; // Number of layers
    let boxes_per_layer = 3; // Number of boxes in one side of a layer
    let box_size = 1.0; // Size of each box
    let spacing = 0.01; // Small spacing between boxes
    
    // Create stack of boxes
    for layer in 0..stack_height {
        let layer_y = 0.5 + (layer as f32) * (box_size + spacing);
        
        // Create grid pattern for this layer, alternating orientation each layer
        let is_rotated = layer % 2 == 1;
        
        for row in 0..boxes_per_layer {
            for col in 0..boxes_per_layer {
                let (x, z) = if is_rotated {
                    (
                        (col as f32 - (boxes_per_layer as f32 - 1.0) / 2.0) * (box_size + spacing),
                        (row as f32 - (boxes_per_layer as f32 - 1.0) / 2.0) * (box_size + spacing)
                    )
                } else {
                    (
                        (row as f32 - (boxes_per_layer as f32 - 1.0) / 2.0) * (box_size + spacing),
                        (col as f32 - (boxes_per_layer as f32 - 1.0) / 2.0) * (box_size + spacing)
                    )
                };
                
                // Create physics box with better collision properties
                let box_shape = Arc::new(BoxShape::new_with_dimensions(box_size, box_size, box_size));
                let box_material = Material::new(100.0, 0.5, 0.4); // Medium friction/bounce wood-like
                let box_pos = Vector3::new(x, layer_y, z);

                let mut rotation = PhysQuaternion::identity();
                if is_rotated {
                    // Rotate 90 degrees around Y
                    rotation = PhysQuaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), std::f32::consts::PI / 2.0);
                }

                let transform = phys_engine::math::Transform {
                    position: box_pos,
                    rotation,
                    scale: Vector3::new(1.0, 1.0, 1.0),
                };

                // Create a much better box material for stacking
                let custom_material = Material::new(
                    500.0,  // Much higher density for better stability
                    0.98,   // Extremely high friction for stacking
                    0.01    // Almost no bounce for stacking
                );

                let mut rigid_box = RigidBody::new(box_shape, transform, RigidBodyType::Dynamic);
                rigid_box.set_material(custom_material);

                // Manually set mass for better stability
                rigid_box.set_mass(20.0); // Much higher mass for better stability and prevent falling through
                
                // Add slight random rotation to make the stack unstable and more interesting
                let random_ang_vel = Vector3::new(
                    (rand::random::<f32>() - 0.5) * 0.01,
                    (rand::random::<f32>() - 0.5) * 0.01,
                    (rand::random::<f32>() - 0.5) * 0.01,
                );
                rigid_box.set_angular_velocity(random_ang_vel);
                
                let handle = physics_world.add_body(rigid_box);
                box_handles.push(handle);
                
                // Get color based on layer or position
                let color_index = ((layer * boxes_per_layer + row) % box_colors.len()) as usize;
                let color = box_colors[color_index];
                
                // Create visual representation
                commands.spawn((
                    PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Box::new(box_size, box_size, box_size))),
                        material: materials.add(StandardMaterial {
                            base_color: color,
                            ..default()
                        }),
                        transform: Transform::from_xyz(x, layer_y, z)
                            .with_rotation(if is_rotated { 
                                Quat::from_rotation_y(std::f32::consts::PI / 2.0) 
                            } else { 
                                Quat::IDENTITY 
                            }),
                        ..default()
                    },
                    PhysicsBody {
                        handle,
                    },
                    BoxBody,
                ));
            }
        }
    }

    // Insert physics world as a resource
    commands.insert_resource(PhysicsResource {
        world: physics_world,
        box_handles,
        floor_handle,
    });
}

fn physics_step(
    time: Res<Time>,
    mut physics: ResMut<PhysicsResource>,
) {
    // Step the physics simulation with smaller substeps for more stable simulation
    let dt = time.delta_seconds();

    // Use extremely small substeps for maximum stability
    // Use a very small fixed timestep of 1/600 seconds for simulation
    const FIXED_TIMESTEP: f32 = 1.0 / 600.0;
    let num_substeps = (dt / FIXED_TIMESTEP).ceil() as u32;
    let substep_dt = dt / num_substeps as f32;

    for _ in 0..num_substeps {
        physics.world.step(substep_dt);
    }
}

fn update_transforms(
    mut query: Query<(&mut Transform, &PhysicsBody)>,
    physics: Res<PhysicsResource>,
) {
    // Update all entity transforms based on physics state
    for (mut transform, body) in query.iter_mut() {
        if let Ok(phys_body) = physics.world.get_body(body.handle) {
            // Get position from physics
            let pos = phys_body.get_position();
            transform.translation = Vec3::new(pos.x, pos.y, pos.z);
            
            // Get rotation from physics
            let rot = phys_body.get_rotation();
            transform.rotation = Quat::from_xyzw(rot.x, rot.y, rot.z, rot.w);
        }
    }
}

fn camera_controls(
    time: Res<Time>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Transform, With<Camera>>,
) {
    // Simple camera controls
    let mut camera_transform = query.single_mut();
    let mut movement = Vec3::ZERO;
    let speed = 5.0;
    
    if keyboard_input.pressed(KeyCode::KeyW) {
        movement.z -= speed;
    }
    if keyboard_input.pressed(KeyCode::KeyS) {
        movement.z += speed;
    }
    if keyboard_input.pressed(KeyCode::KeyA) {
        movement.x -= speed;
    }
    if keyboard_input.pressed(KeyCode::KeyD) {
        movement.x += speed;
    }
    if keyboard_input.pressed(KeyCode::Space) {
        movement.y += speed;
    }
    if keyboard_input.pressed(KeyCode::ShiftLeft) {
        movement.y -= speed;
    }
    
    // Apply local movement
    let local_movement = camera_transform.rotation * movement * time.delta_seconds();
    camera_transform.translation += local_movement;
    
    // Rotate camera
    if keyboard_input.pressed(KeyCode::ArrowLeft) {
        camera_transform.rotate_y(0.02);
    }
    if keyboard_input.pressed(KeyCode::ArrowRight) {
        camera_transform.rotate_y(-0.02);
    }
    if keyboard_input.pressed(KeyCode::ArrowUp) {
        let right = camera_transform.right();
        camera_transform.rotate_around(Vec3::ZERO, Quat::from_axis_angle(*right, 0.02));
    }
    if keyboard_input.pressed(KeyCode::ArrowDown) {
        let right = camera_transform.right();
        camera_transform.rotate_around(Vec3::ZERO, Quat::from_axis_angle(*right, -0.02));
    }
}

fn spawn_box_on_click(
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics: ResMut<PhysicsResource>,
    query: Query<&Transform, With<Camera>>,
) {
    // Spawn a new box when clicking the mouse
    if mouse_button.just_pressed(MouseButton::Left) {
        let camera_transform = query.single();
        
        // Create a box a few units in front of the camera
        let spawn_pos = camera_transform.translation + camera_transform.forward() * 5.0;
        
        // Create physics box with better collision properties
        let box_size = 0.8;
        let box_shape = Arc::new(BoxShape::new_with_dimensions(box_size, box_size, box_size));
        let box_material = Material::new(100.0, 0.5, 0.4);
        let phys_pos = Vector3::new(spawn_pos.x, spawn_pos.y, spawn_pos.z);

        // Create a much better material with extreme friction and almost no restitution
        let custom_material = Material::new(
            500.0,  // Much higher density for better stability
            0.98,   // Extremely high friction for stacking
            0.01    // Almost no bounce for stacking
        );

        // Create physics body with an initial velocity in the direction the camera is facing
        let mut rigid_box = RigidBody::new_dynamic(box_shape.clone(), phys_pos);
        rigid_box.set_material(custom_material);

        // Increase box mass for better stability
        rigid_box.set_mass(20.0); // Much higher mass for better stability
        
        // Set initial velocity based on camera direction
        let velocity = camera_transform.forward() * 15.0; // Throw with force
        rigid_box.set_linear_velocity(Vector3::new(velocity.x, velocity.y, velocity.z));
        
        // Add some random spin
        let random_ang_vel = Vector3::new(
            (rand::random::<f32>() - 0.5) * 2.0,
            (rand::random::<f32>() - 0.5) * 2.0,
            (rand::random::<f32>() - 0.5) * 2.0,
        );
        rigid_box.set_angular_velocity(random_ang_vel);
        
        let handle = physics.world.add_body(rigid_box);
        physics.box_handles.push(handle);
        
        // Create random color
        let r = 0.3 + rand::random::<f32>() * 0.7;
        let g = 0.3 + rand::random::<f32>() * 0.7;
        let b = 0.3 + rand::random::<f32>() * 0.7;
        let color = Color::rgb(r, g, b);
        
        // Create visual representation
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Box::new(box_size, box_size, box_size))),
                material: materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                }),
                transform: Transform::from_translation(spawn_pos),
                ..default()
            },
            PhysicsBody {
                handle,
            },
            BoxBody,
        ));
    }
}