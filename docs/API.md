# PhysEngine API Reference

This document provides a comprehensive API reference for the PhysEngine physics library.

## Core Components

### PhysicsWorld

The main simulation world that manages all physics objects.

```rust
// Create a new physics world with default settings
let world = PhysicsWorld::new();

// Create a world with custom configuration
let world = PhysicsWorld::with_config(config);

// Add a rigid body to the world
let handle = world.add_body(body);

// Get a reference to a body
let body = world.get_body(handle).unwrap();

// Get a mutable reference to a body
let body = world.get_body_mut(handle).unwrap();

// Remove a body from the world
let body = world.remove_body(handle).unwrap();

// Add a constraint
let constraint_handle = world.add_constraint(constraint);

// Remove a constraint
let constraint = world.remove_constraint(constraint_handle).unwrap();

// Step the simulation
world.step(time_step);

// Get collision events
let events = world.get_events();

// Clear the world
world.clear();

// Get the number of bodies
let count = world.body_count();

// Set gravity
world.set_gravity(gravity);

// Get the current simulation time
let time = world.get_time();

// Cast a ray through the world
let hit = world.raycast(ray, max_distance);
```

### SimulationConfig

Configuration parameters for the physics simulation.

```rust
let mut config = SimulationConfig::default();

// Time stepping
config.time_step = 1.0 / 60.0;
config.max_substeps = 10;

// Gravity
config.gravity = GravityType::Constant(Vector3::new(0.0, -9.81, 0.0));

// Constraint solving iterations
config.position_iterations = 10;
config.velocity_iterations = 8;

// Collision detection
config.use_ccd = true;
config.collision_margin = 0.01;

// Sleeping
config.allow_sleeping = true;
config.linear_sleep_threshold = 0.01;
config.angular_sleep_threshold = 0.01;
config.sleep_time_threshold = 0.5;

// Damping
config.linear_damping = 0.01;
config.angular_damping = 0.05;

// Constraint solving parameters
config.constraint_bias_factor = 0.2;
config.cfm_factor = 0.0;
config.erp_factor = 0.2;
config.contact_penetration_threshold = 0.005;
config.restitution_velocity_threshold = 0.5;
```

## Rigid Bodies

### RigidBody

Represents a physical object in the simulation.

```rust
// Creation
let body = RigidBody::new(shape, transform, body_type);
let body = RigidBody::new_dynamic(shape, position);
let body = RigidBody::new_kinematic(shape, position);
let body = RigidBody::new_static(shape, position);
let body = RigidBody::new_trigger(shape, position);

// Getters and setters
let transform = body.get_transform();
body.set_transform(transform);

let position = body.get_position();
body.set_position(position);

let rotation = body.get_rotation();
body.set_rotation(rotation);

let velocity = body.get_linear_velocity();
body.set_linear_velocity(velocity);

let angular_velocity = body.get_angular_velocity();
body.set_angular_velocity(angular_velocity);

let material = body.get_material();
body.set_material(material);

let shape = body.get_shape();
body.set_shape(shape);

let mass = body.get_mass();
body.set_mass(mass);

let body_type = body.get_body_type();
body.set_body_type(body_type);

// Sleeping
let sleeping = body.is_sleeping();
body.put_to_sleep();
body.wake_up();
body.set_can_sleep(can_sleep);

// Forces and impulses
body.apply_force(force);
body.apply_force_at_point(force, point);
body.apply_torque(torque);
body.apply_impulse(impulse);
body.apply_impulse_at_point(impulse, point);
body.apply_angular_impulse(impulse);

// Integration
body.integrate_forces(dt);
body.integrate_velocity(dt);
```

### RigidBodyType

The type of rigid body, determining how it behaves in the simulation.

```rust
// Dynamic bodies are fully simulated
RigidBodyType::Dynamic

// Kinematic bodies are moved programmatically but affect dynamic bodies
RigidBodyType::Kinematic

// Static bodies don't move and aren't affected by forces
RigidBodyType::Static
```

### Material

Material properties for physics objects.

```rust
// Create a new material
let material = Material::new(density, friction, restitution);
let material = Material::new_with_rolling(density, friction, restitution, rolling_friction);

// Predefined materials
let rubber = Material::rubber();
let metal = Material::metal();
let wood = Material::wood();
let ice = Material::ice();
let concrete = Material::concrete();

// Properties
let density = material.density;
let friction = material.friction;
let restitution = material.restitution;
let rolling_friction = material.rolling_friction;
```

## Shapes

### Shape Trait

Base trait for all collision shapes.

```rust
trait Shape: Send + Sync + 'static {
    fn shape_type(&self) -> &'static str;
    fn get_volume(&self) -> f32;
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3;
    fn get_local_bounds(&self) -> Aabb;
    fn get_world_bounds(&self, transform: &Transform) -> Aabb;
    fn get_support_point(&self, direction: Vector3) -> Vector3;
    fn get_world_support_point(&self, direction: Vector3, transform: &Transform) -> Vector3;
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32>;
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn clone_shape(&self) -> Box<dyn Shape>;
}
```

### Available Shapes

```rust
// Sphere
let sphere = Sphere::new(radius);
sphere.get_radius();
sphere.set_radius(radius);

// Box
let box_shape = Box::new(half_extents);
let box_shape = Box::new_with_dimensions(width, height, depth);
box_shape.get_half_extents();
box_shape.set_half_extents(half_extents);
box_shape.get_dimensions();
box_shape.set_dimensions(dimensions);

// Plane
let plane = Plane::new(normal, distance);
let plane = Plane::from_point_normal(point, normal);
plane.get_normal();
plane.get_distance();
plane.closest_point_to(point);
plane.signed_distance_to(point);

// Capsule
let capsule = Capsule::new(radius, height);
capsule.get_radius();
capsule.get_height();

// Cylinder
let cylinder = Cylinder::new(radius, height);
cylinder.get_radius();
cylinder.get_height();

// Compound
let mut compound = Compound::new();
compound.add_shape(shape, transform);
compound.get_shapes();
```

## Math Components

### Vector3

A 3D vector representation for physics calculations.

```rust
// Creation
let v = Vector3::new(x, y, z);
let v = Vector3::zero();
let v = Vector3::one();
let v = Vector3::unit_x();
let v = Vector3::unit_y();
let v = Vector3::unit_z();

// Operations
let dot = v1.dot(&v2);
let cross = v1.cross(&v2);
let length = v.length();
let length_sq = v.length_squared();
let normalized = v.normalize();
v.normalize_mut();
let is_zero = v.is_zero();
let distance = v1.distance(&v2);
let lerp_result = v1.lerp(&v2, t);
let projection = v1.project(&v2);
let rejection = v1.reject(&v2);
let angle = v1.angle_between(&v2);
```

### Quaternion

Quaternion for representing rotations in 3D space.

```rust
// Creation
let q = Quaternion::new(w, x, y, z);
let q = Quaternion::identity();
let q = Quaternion::from_axis_angle(axis, angle);
let q = Quaternion::from_euler(x, y, z);
let q = Quaternion::from_rotation_matrix(matrix);

// Operations
let rotated = q.rotate_vector(vector);
let conjugate = q.conjugate();
let length = q.length();
let normalized = q.normalize();
q.normalize_mut();
let inverse = q.inverse();
let dot = q1.dot(&q2);
let interpolated = q1.slerp(&q2, t);
let matrix = q.to_rotation_matrix();
```

### Transform

Represents a transformation in 3D space (position, rotation, and scale).

```rust
// Creation
let transform = Transform::new(position, rotation, scale);
let transform = Transform::identity();
let transform = Transform::from_position(position);
let transform = Transform::from_position_rotation(position, rotation);
let transform = Transform::from_matrix(matrix);

// Operations
let matrix = transform.to_matrix();
let transformed = transform.transform_point(point);
let transformed_dir = transform.transform_direction(direction);
let transformed_normal = transform.transform_normal(normal);
let inverse = transform.inverse();
let combined = transform1.combine(&transform2);
let interpolated = transform1.interpolate(&transform2, t);
```

### AABB (Axis-Aligned Bounding Box)

```rust
// Creation
let aabb = Aabb::new(min, max);
let aabb = Aabb::from_center_half_extents(center, half_extents);
let aabb = Aabb::from_points(&points).unwrap();

// Properties
let center = aabb.center();
let extents = aabb.extents();
let half_extents = aabb.half_extents();
let volume = aabb.volume();
let surface_area = aabb.surface_area();

// Operations
let contains = aabb.contains_point(point);
let contains_aabb = aabb.contains_aabb(&other);
let intersects = aabb.intersects(&other);
let intersection = aabb.intersection(&other);
let union = aabb.union(&other);
aabb.expand_to_include_point(point);
aabb.expand_to_include_aabb(&other);
let expanded = aabb.expand(margin);
let closest = aabb.closest_point(point);
let distance = aabb.distance_to_point(point);
let hit = aabb.intersects_ray(&ray, t_min, t_max);
```

### Ray

```rust
// Creation
let ray = Ray::new(origin, direction);
let ray = Ray::new_normalized(origin, direction);

// Operations
let point = ray.point_at(t);
let normalized = ray.normalized_direction();
let transformed = ray.transform(&matrix);
let closest = ray.closest_point(point);
let distance = ray.distance_to_point(point);
```

## Collision Detection

### CollisionDetector

```rust
// Creation
let detector = CollisionDetector::new(
    broad_phase,
    narrow_phase,
    contact_generator,
    filter,
);

// Operations
detector.update(bodies);
let states = detector.get_collision_states();
let manifolds = detector.get_contact_manifolds();
let is_colliding = detector.are_colliding(body_a, body_b);
let is_start = detector.is_collision_start(body_a, body_b);
let is_end = detector.is_collision_end(body_a, body_b);
let manifold = detector.get_contact_manifold(body_a, body_b);
```

### ContactManifold

```rust
// Properties
let pair = manifold.pair;
let contacts = &manifold.contacts;
let normal = manifold.normal;
let restitution = manifold.restitution;
let friction = manifold.friction;

// Operations
manifold.add_contact(contact);
manifold.clear();
let is_empty = manifold.is_empty();
manifold.update(body_a_moved, body_b_moved);
manifold.set_material_properties(restitution, friction);
```

### ContactPoint

```rust
// Properties
let position = contact.position;
let normal = contact.normal;
let penetration = contact.penetration;
```

## Constraints

### Constraint Trait

```rust
trait Constraint: Send + Sync + 'static {
    fn constraint_type(&self) -> &'static str;
    fn get_bodies(&self) -> &[BodyHandle];
    fn involves_body(&self, body: BodyHandle) -> bool;
    fn prepare(&mut self, bodies: &BodyStorage<RigidBody>);
    fn solve_velocity(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>);
    fn solve_position(&mut self, dt: f32, bodies: &mut BodyStorage<RigidBody>);
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn clone_constraint(&self) -> Box<dyn Constraint>;
}
```

### Available Constraints

```rust
// Distance constraint
let constraint = DistanceConstraint::new(
    body_a, body_b, 
    anchor_a, anchor_b, 
    distance
);

// Hinge constraint
let constraint = HingeConstraint::new(
    body_a, body_b,
    pivot_a, pivot_b,
    axis_a, axis_b
);

// Ball socket constraint
let constraint = BallSocketConstraint::new(
    body_a, body_b,
    anchor_a, anchor_b
);

// Slider constraint
let constraint = SliderConstraint::new(
    body_a, body_b,
    anchor_a, anchor_b,
    axis_a, axis_b
);

// Fixed constraint
let constraint = FixedConstraint::new(
    body_a, body_b
);

// Cone twist constraint
let constraint = ConeTwistConstraint::new(
    body_a, body_b,
    frame_a, frame_b
);
```

## Integration

### Integrator Trait

```rust
trait Integrator: Send + Sync {
    fn integrate(&self, body: &mut RigidBody, dt: f32);
    fn name(&self) -> &str;
}
```

### Available Integrators

```rust
// Forward Euler
let integrator = EulerIntegrator::new();

// Symplectic Euler (semi-implicit)
let integrator = SymplecticEulerIntegrator::new();

// Velocity Verlet
let integrator = VerletIntegrator::new();

// Runge-Kutta 4th order
let integrator = RungeKuttaIntegrator::new();
```

## Events

### EventQueue

```rust
// Check for events
let has_collision_events = events.has_collision_events();
let has_body_events = events.has_body_events();
let is_empty = events.is_empty();

// Get events
let next_collision = events.next_collision_event();
let next_body = events.next_body_event();

// Filter events
let begin_events = events.get_collision_events_of_type(CollisionEventType::Begin);
let sleep_events = events.get_body_events_of_type(BodyEventType::Sleep);
let body_collisions = events.get_collision_events_for_body(body);
let body_events = events.get_body_events_for_body(body);

// Clear events
events.clear();
```

### CollisionEvent

```rust
// Properties
let event_type = event.event_type; // Begin, Persist, End, Impulse
let body_a = event.body_a;
let body_b = event.body_b;
let contacts = &event.contacts;
let normal_impulse = event.normal_impulse;
let tangent_impulse = event.tangent_impulse;
```

### BodyEvent

```rust
// Properties
let event_type = event.event_type; // Added, Removed, Sleep, Awake, TransformChanged
let body = event.body;
```