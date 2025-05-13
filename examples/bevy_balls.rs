use bevy::{
    prelude::*,
    input::InputPlugin,
};
use phys_engine::{
    PhysicsWorld, RigidBody, Material,
    shapes::{Sphere, box_shape::BoxShape},
    math::{Vector3},
    core::GravityType,
};
use std::sync::Arc;

// Resource to store our physics world
#[derive(Resource)]
struct PhysicsResource {
    world: PhysicsWorld,
    body_handles: Vec<phys_engine::core::BodyHandle>,
    platform_handle: phys_engine::core::BodyHandle,
}

// Component to link Bevy entities with physics bodies
#[derive(Component)]
struct PhysicsBody {
    handle: phys_engine::core::BodyHandle,
}

#[derive(Component)]
struct Platform;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::rgb(0.2, 0.2, 0.2)))
        .add_systems(Startup, setup)
        .add_systems(Update, (
            physics_step,
            update_transforms,
            camera_controls,
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
        transform: Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
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

    // Create physics world with improved settings
    let mut physics_world = PhysicsWorld::new();
    physics_world.set_gravity(GravityType::Constant(Vector3::new(0.0, -9.81, 0.0)));

    // Set reasonable simulation parameters that balance realism and stability
    let mut config = physics_world.get_config_mut();
    config.position_iterations = 10;      // Standard position iterations
    config.velocity_iterations = 8;       // Standard velocity iterations
    config.collision_margin = 0.01;       // Small collision margin for accurate collisions
    config.contact_penetration_threshold = 0.001;  // Standard penetration threshold
    config.constraint_bias_factor = 0.2;  // Standard bias factor
    config.erp_factor = 0.2;             // Standard ERP value
    config.cfm_factor = 0.00001;         // Standard CFM value
    config.use_ccd = true;               // Enable continuous collision detection
    config.restitution_velocity_threshold = 0.05;  // Standard threshold
    config.max_substeps = 4;             // Standard substeps
    config.time_step = 1.0/60.0;         // Standard timestep

    // Create two thick platforms stacked for better stability

    // Bottom platform (completely fixed and extremely thick)
    let bottom_platform_shape = Arc::new(BoxShape::new_with_dimensions(40.0, 10.0, 40.0));
    let bottom_platform_material = Material::new(10000.0, 1.0, 0.0); // Maximum friction, zero bounce
    let mut bottom_platform = RigidBody::new_static(bottom_platform_shape, Vector3::new(0.0, -7.0, 0.0));
    bottom_platform.set_material(bottom_platform_material);
    physics_world.add_body(bottom_platform);

    // Top platform (where balls will land)
    let platform_shape = Arc::new(BoxShape::new_with_dimensions(30.0, 5.0, 30.0));
    let platform_material = Material::new(5000.0, 0.95, 0.01); // Ultra dense material with high friction and almost no bounce
    let mut platform = RigidBody::new_static(platform_shape, Vector3::new(0.0, -2.5, 0.0));
    platform.set_material(platform_material);
    let platform_handle = physics_world.add_body(platform);

    // Spawn bottom platform in Bevy
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(40.0, 10.0, 40.0))),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.2, 0.3, 0.2),
                ..default()
            }),
            transform: Transform::from_xyz(0.0, -7.0, 0.0),
            ..default()
        },
        PhysicsBody {
            handle: platform_handle,
        },
        Platform,
    ));

    // Spawn top platform in Bevy
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(30.0, 5.0, 30.0))),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.3, 0.5, 0.3),
                ..default()
            }),
            transform: Transform::from_xyz(0.0, -2.5, 0.0),
            ..default()
        },
        PhysicsBody {
            handle: platform_handle,
        },
        Platform,
    ));

    // Create balls with different materials
    let mut body_handles = Vec::new();

    // Ball materials
    let materials_list = [
        (Material::rubber(), Color::rgb(0.8, 0.2, 0.2)), // Red rubber
        (Material::metal(), Color::rgb(0.2, 0.8, 0.2)),  // Green metal
        (Material::wood(), Color::rgb(0.2, 0.2, 0.8)),   // Blue wood
    ];

    // Spawn 15 balls to demonstrate proper physics behavior
    for i in 0..15 {
        // Place balls in a 3D grid formation with some randomness
        let row = i / 5;
        let col = i % 5;
        let x = (col as f32 * 2.0) - 4.0 + (rand::random::<f32>() * 0.5 - 0.25);
        let y = 5.0 + (row as f32 * 2.0) + (rand::random::<f32>() * 0.5);
        let z = (rand::random::<f32>() * 2.0) - 1.0;

        let position = Vector3::new(x, y, z);
        
        // Choose a material
        let material_index = i % materials_list.len();
        let phys_material = materials_list[material_index].0;
        let color = materials_list[material_index].1;
        
        // Create the ball with reasonable radius that won't cause performance issues
        let radius = 1.0; // Smaller radius for better performance and still stable collision detection
        let ball_shape = Arc::new(Sphere::new(radius));
        let mut ball = RigidBody::new_dynamic(ball_shape, position);

        // Use a physically realistic material
        let custom_material = Material::new(
            phys_material.density, // Normal density for realistic behavior
            0.8, // Moderate friction
            0.4  // Moderate restitution for some bounce
        );

        // Enable CCD (Continuous Collision Detection) for this ball
        ball.set_ccd_enabled(true);
        ball.set_material(custom_material);
        
        // Add reasonable initial velocities for interesting physics interactions
        let vx = (rand::random::<f32>() * 2.0) - 1.0; // Moderate random horizontal velocity
        let vy = 0.0; // No initial vertical velocity
        let vz = (rand::random::<f32>() * 2.0) - 1.0; // Moderate random horizontal velocity
        ball.set_linear_velocity(Vector3::new(vx, vy, vz));

        // Add some initial rotation for natural movement
        let rot_x = (rand::random::<f32>() * 0.2) - 0.1;
        let rot_y = (rand::random::<f32>() * 0.2) - 0.1;
        let rot_z = (rand::random::<f32>() * 0.2) - 0.1;
        ball.set_angular_velocity(Vector3::new(rot_x, rot_y, rot_z));
        
        // Add to the world
        let handle = physics_world.add_body(ball);
        body_handles.push(handle);
        
        // Spawn ball in Bevy
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Mesh::from(shape::UVSphere {
                    radius,
                    ..default()
                })),
                material: materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                }),
                transform: Transform::from_xyz(position.x, position.y, position.z),
                ..default()
            },
            PhysicsBody {
                handle,
            },
        ));
    }

    // Insert physics world as a resource
    commands.insert_resource(PhysicsResource {
        world: physics_world,
        body_handles,
        platform_handle,
    });
}

fn physics_step(
    time: Res<Time>,
    mut physics: ResMut<PhysicsResource>,
) {
    // Step the physics simulation with smaller substeps for more stable simulation
    let dt = time.delta_seconds();

    // Use a reasonable number of substeps that balances stability and performance
    // A fixed timestep of 1/200 seconds provides good stability without excessive computation
    const FIXED_TIMESTEP: f32 = 1.0 / 200.0;
    let num_substeps = (dt / FIXED_TIMESTEP).ceil() as u32;
    let substep_dt = dt / num_substeps as f32;

    // Use standard physics timestep for realistic behavior
    physics.world.step(dt);
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