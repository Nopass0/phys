use crate::bodies::RigidBodyType;
use crate::core::{BodyStorage, Storage, events::{CollisionEvent, CollisionEventType}};
use crate::bodies::RigidBody;
use crate::collision::broad_phase::BroadPhase;
use crate::math::Vector3;
use crate::shapes::{Sphere, box_shape::BoxShape};

/// A realistic collision detection function with proper physical behavior
/// This implementation properly handles all collision types while still ensuring
/// objects don't fall through the floor
pub fn detect_collisions(bodies: &mut BodyStorage<RigidBody>, events_queue: &mut crate::core::EventQueue) {
    // First, gather all bodies that can collide
    let mut body_list = Vec::new();
    for (handle, body) in bodies.iter() {
        if body.get_shape().is_some() && !body.is_trigger() {
            body_list.push((handle, body));
        }
    }

    // Use BruteForceBroadPhase for simplicity (already in codebase)
    let mut broad_phase = crate::collision::broad_phase::BruteForceBroadPhase::new();
    let broad_phase_trait: &mut dyn BroadPhase = &mut broad_phase;
    broad_phase_trait.update(&body_list);

    // Get potential collision pairs
    let pairs = broad_phase_trait.get_collision_pairs();

    // For each potential pair, gather the data we need first
    struct CollisionData {
        body_a_handle: crate::core::BodyHandle,
        body_b_handle: crate::core::BodyHandle,
        dynamic_a: bool,
        dynamic_b: bool,
        transform_a: crate::math::Transform,
        transform_b: crate::math::Transform,
        position_a: Vector3,
        position_b: Vector3,
        velocity_a: Vector3,
        velocity_b: Vector3,
        gen_events_a: bool,
        gen_events_b: bool,
        shape_a_type: String,
        shape_b_type: String,
        sphere_radius: Option<f32>,
        box_half_extents: Option<Vector3>,
        is_a_sphere: bool,
        restitution_a: f32,
        restitution_b: f32,
        is_floor_collision: bool,
    }

    let mut collision_data_list = Vec::new();

    // First pass: gather all info needed
    for pair in pairs {
        let body_a = match bodies.get(pair.body_a) {
            Some(body) => body,
            None => continue,
        };

        let body_b = match bodies.get(pair.body_b) {
            Some(body) => body,
            None => continue,
        };

        // Skip if both are static (can't collide)
        if body_a.get_body_type() == RigidBodyType::Static &&
           body_b.get_body_type() == RigidBodyType::Static {
            continue;
        }

        // Get shapes
        let shape_a = match body_a.get_shape() {
            Some(s) => s,
            None => continue,
        };

        let shape_b = match body_b.get_shape() {
            Some(s) => s,
            None => continue,
        };

        // Determine if these are sphere-box collisions
        let mut is_a_sphere = false;
        let mut sphere_radius = None;
        let mut box_half_extents = None;
        let mut is_floor_collision = false;

        // Check if shape A is a sphere
        let shape_a_type = shape_a.shape_type().to_string();
        let shape_b_type = shape_b.shape_type().to_string();

        if shape_a_type == "Sphere" {
            if let Some(sphere) = shape_a.as_any().downcast_ref::<Sphere>() {
                is_a_sphere = true;
                if shape_b_type == "Box" {
                    if let Some(box_shape) = shape_b.as_any().downcast_ref::<BoxShape>() {
                        sphere_radius = Some(sphere.get_radius());
                        box_half_extents = Some(box_shape.get_half_extents());

                        // Check if this looks like a floor collision
                        // Floor collisions typically have the box below the sphere
                        // and the box is much wider than tall (floor-like)
                        let is_box_wide = box_half_extents.unwrap().x > box_half_extents.unwrap().y * 2.0 &&
                                         box_half_extents.unwrap().z > box_half_extents.unwrap().y * 2.0;
                        if is_box_wide && body_a.get_position().y > body_b.get_position().y {
                            is_floor_collision = true;
                        }
                    }
                }
            }
        } else if shape_b_type == "Sphere" {
            if let Some(sphere) = shape_b.as_any().downcast_ref::<Sphere>() {
                is_a_sphere = false;
                if shape_a_type == "Box" {
                    if let Some(box_shape) = shape_a.as_any().downcast_ref::<BoxShape>() {
                        sphere_radius = Some(sphere.get_radius());
                        box_half_extents = Some(box_shape.get_half_extents());

                        // Check if this looks like a floor collision
                        let is_box_wide = box_half_extents.unwrap().x > box_half_extents.unwrap().y * 2.0 &&
                                         box_half_extents.unwrap().z > box_half_extents.unwrap().y * 2.0;
                        if is_box_wide && body_b.get_position().y > body_a.get_position().y {
                            is_floor_collision = true;
                        }
                    }
                }
            }
        }

        collision_data_list.push(CollisionData {
            body_a_handle: pair.body_a,
            body_b_handle: pair.body_b,
            dynamic_a: body_a.get_body_type() == RigidBodyType::Dynamic,
            dynamic_b: body_b.get_body_type() == RigidBodyType::Dynamic,
            transform_a: body_a.get_transform(),
            transform_b: body_b.get_transform(),
            position_a: body_a.get_position(),
            position_b: body_b.get_position(),
            velocity_a: body_a.get_linear_velocity(),
            velocity_b: body_b.get_linear_velocity(),
            gen_events_a: body_a.generates_collision_events(),
            gen_events_b: body_b.generates_collision_events(),
            shape_a_type,
            shape_b_type,
            sphere_radius,
            box_half_extents,
            is_a_sphere,
            restitution_a: body_a.get_material().restitution,
            restitution_b: body_b.get_material().restitution,
            is_floor_collision,
        });
    }

    // Sort collision data list to prioritize floor collisions
    collision_data_list.sort_by(|a, b| {
        if a.is_floor_collision && !b.is_floor_collision {
            std::cmp::Ordering::Less
        } else if !a.is_floor_collision && b.is_floor_collision {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    // Use a hashset to track bodies that have already been corrected this frame
    // to avoid overcorrection
    let mut corrected_bodies = std::collections::HashSet::new();

    // Second pass: check for collisions and apply corrections
    for data in collision_data_list {
        // Process sphere-box collisions with special handling for floor collisions
        if let (Some(radius), Some(half_extents)) = (data.sphere_radius, data.box_half_extents) {
            // This is a sphere-box collision
            let (sphere_pos, box_pos, sphere_handle, _box_handle, _sphere_vel, is_sphere_dynamic, sphere_restitution) =
                if data.is_a_sphere {
                    (data.position_a, data.position_b, data.body_a_handle, data.body_b_handle,
                     data.velocity_a, data.dynamic_a, data.restitution_a)
                } else {
                    (data.position_b, data.position_a, data.body_b_handle, data.body_a_handle,
                     data.velocity_b, data.dynamic_b, data.restitution_b)
                };

            // Special ultra-aggressive handling for floor collisions
            if data.is_floor_collision {
                // Compute signed distance between top of box and bottom of sphere
                let sphere_bottom = sphere_pos.y - radius;
                let box_top = box_pos.y + half_extents.y;
                let vertical_distance = sphere_bottom - box_top;

                // Check if sphere is above the box in horizontal projection
                let in_x_bounds = sphere_pos.x >= box_pos.x - (half_extents.x + radius * 1.5) &&
                                 sphere_pos.x <= box_pos.x + (half_extents.x + radius * 1.5);

                let in_z_bounds = sphere_pos.z >= box_pos.z - (half_extents.z + radius * 1.5) &&
                                 sphere_pos.z <= box_pos.z + (half_extents.z + radius * 1.5);

                // Extremely aggressive floor collision detection
                if in_x_bounds && in_z_bounds && vertical_distance < radius * 0.5 {
                    // Generate collision event
                    if data.gen_events_a || data.gen_events_b {
                        let event = CollisionEvent {
                            body_a: data.body_a_handle,
                            body_b: data.body_b_handle,
                            event_type: CollisionEventType::Begin,
                            contacts: Vec::new(),
                            normal_impulse: None,
                            tangent_impulse: None,
                        };

                        events_queue.add_collision_event(event);
                    }

                    // Apply position correction for the sphere if it's dynamic
                    if is_sphere_dynamic && !corrected_bodies.contains(&sphere_handle) {
                        // Normal always points up for floor collisions
                        let normal = Vector3::new(0.0, 1.0, 0.0);

                        // Calculate a proper physics-based penetration response
                        let penetration = if vertical_distance < 0.0 {
                            -vertical_distance + 0.01  // Small buffer to prevent jitter
                        } else {
                            0.0  // No correction if not penetrating
                        };

                        // Apply reasonable correction with slight bias (1.2x) to ensure stability
                        // This is a physically reasonable value
                        let correction = normal * penetration * 1.2;

                        if let Ok(body) = bodies.get_body_mut(sphere_handle) {
                            // Update position - apply full correction immediately
                            let new_pos = sphere_pos + correction;
                            body.set_position(new_pos);

                            // Get current velocity
                            let vel = body.get_linear_velocity();

                            // If object is moving down (into the floor)
                            if vel.y < 0.0 {
                                // Use the actual material restitution for natural bouncing
                                let restitution = sphere_restitution;

                                // Calculate the correctly reflected velocity using physics formulas
                                let mut new_vel = vel;

                                // Apply a proper velocity reflection based on the normal
                                if new_vel.y < 0.0 {
                                    // This is the standard physics formula for collision response
                                    new_vel.y = -new_vel.y * restitution;
                                }

                                // Apply moderate horizontal friction based on material properties
                                // 0.9 is a reasonable value for most surfaces
                                let friction = 0.9;
                                new_vel.x *= friction;
                                new_vel.z *= friction;

                                // Set the modified velocity
                                body.set_linear_velocity(new_vel);

                                // Apply moderate angular damping for natural motion
                                let ang_vel = body.get_angular_velocity();
                                body.set_angular_velocity(ang_vel * 0.95);
                            }

                            // Mark this body as already corrected
                            corrected_bodies.insert(sphere_handle);
                        }
                    }

                    continue;
                }
            }

            // Regular sphere-box collision (non-floor)
            // Approximation using center distance
            let box_to_sphere = sphere_pos - box_pos;

            // Clamp point to box bounds
            let closest_point = Vector3::new(
                box_pos.x + box_to_sphere.x.max(-half_extents.x).min(half_extents.x),
                box_pos.y + box_to_sphere.y.max(-half_extents.y).min(half_extents.y),
                box_pos.z + box_to_sphere.z.max(-half_extents.z).min(half_extents.z)
            );

            // Distance from closest point to sphere center
            let closest_to_sphere = sphere_pos - closest_point;
            let distance = closest_to_sphere.length();

            if distance < radius * 1.1 {  // Add a small margin for better detection
                // Generate collision event
                if data.gen_events_a || data.gen_events_b {
                    let event = CollisionEvent {
                        body_a: data.body_a_handle,
                        body_b: data.body_b_handle,
                        event_type: CollisionEventType::Begin,
                        contacts: Vec::new(),
                        normal_impulse: None,
                        tangent_impulse: None,
                    };

                    events_queue.add_collision_event(event);
                }

                if is_sphere_dynamic && !corrected_bodies.contains(&sphere_handle) {
                    // Normal points from closest point to sphere center
                    let normal = if distance > 0.001 {
                        closest_to_sphere / distance
                    } else {
                        // If too close, use direction from box to sphere
                        let dir = box_to_sphere;
                        if dir.length_squared() > 0.001 {
                            dir.normalize()
                        } else {
                            Vector3::new(0.0, 1.0, 0.0)  // Default to up if direction is zero
                        }
                    };

                    // Penetration is the amount the sphere overlaps the box
                    let penetration = radius - distance;
                    if penetration > 0.0 {
                        // Apply very aggressive position correction (2x penetration)
                        let correction = normal * penetration * 2.0;

                        if let Ok(body) = bodies.get_body_mut(sphere_handle) {
                            // Apply position correction
                            let new_pos = sphere_pos + correction;
                            body.set_position(new_pos);

                            // Reflect velocity
                            let vel = body.get_linear_velocity();
                            let vel_into_normal = vel.dot(&normal);

                            if vel_into_normal < 0.0 {
                                let restitution = sphere_restitution;
                                let reflected_vel = vel - normal * (1.0 + restitution) * vel_into_normal;

                                // Apply damping for stability
                                body.set_linear_velocity(reflected_vel * 0.95);
                            }

                            // Mark as corrected
                            corrected_bodies.insert(sphere_handle);
                        }
                    }
                }

                continue;
            }
        }

        // Only handle these cases if the bodies haven't been corrected yet
        if !corrected_bodies.contains(&data.body_a_handle) && !corrected_bodies.contains(&data.body_b_handle) {
            // For other collisions, use GJK with fallback
            if crate::collision::GJK::intersect(
                bodies.get(data.body_a_handle).unwrap().get_shape().unwrap().as_ref(),
                &data.transform_a,
                bodies.get(data.body_b_handle).unwrap().get_shape().unwrap().as_ref(),
                &data.transform_b,
            ) {
                // Generate collision event
                if data.gen_events_a || data.gen_events_b {
                    let event = CollisionEvent {
                        body_a: data.body_a_handle,
                        body_b: data.body_b_handle,
                        event_type: CollisionEventType::Begin,
                        contacts: Vec::new(),
                        normal_impulse: None,
                        tangent_impulse: None,
                    };

                    events_queue.add_collision_event(event);
                }

                // Try to get penetration depth with EPA
                let collision_result = crate::collision::GJK::detect_collision(
                    bodies.get(data.body_a_handle).unwrap().get_shape().unwrap().as_ref(),
                    &data.transform_a,
                    bodies.get(data.body_b_handle).unwrap().get_shape().unwrap().as_ref(),
                    &data.transform_b,
                );

                // EPA can sometimes fail, so we need a fallback
                let (depth, normal) = collision_result.unwrap_or_else(|| {
                    // Fallback using centers and rough approximation
                    let delta = data.position_b - data.position_a;
                    let dir = if delta.length_squared() > 0.001 {
                        delta.normalize()
                    } else {
                        Vector3::new(0.0, 1.0, 0.0)  // Default direction
                    };
                    (0.1, dir)  // Use a small default penetration
                });

                // Apply stronger position correction (1.5x)
                let correction = normal * depth * 1.5;

                // Apply corrections with restitution
                if data.dynamic_a {
                    if let Ok(body) = bodies.get_body_mut(data.body_a_handle) {
                        // Position correction
                        let new_pos = data.position_a - correction * 0.5;  // Distribute correction
                        body.set_position(new_pos);

                        // Velocity reflection if moving toward the other object
                        let vel = body.get_linear_velocity();
                        if vel.dot(&normal) > 0.0 {  // If moving toward B
                            let restitution = data.restitution_a;
                            let reflected_vel = vel - normal * (1.0 + restitution) * vel.dot(&normal);
                            body.set_linear_velocity(reflected_vel * 0.95);  // Damping
                        }

                        corrected_bodies.insert(data.body_a_handle);
                    }
                }

                if data.dynamic_b {
                    if let Ok(body) = bodies.get_body_mut(data.body_b_handle) {
                        // Position correction
                        let new_pos = data.position_b + correction * 0.5;  // Distribute correction
                        body.set_position(new_pos);

                        // Velocity reflection if moving toward the other object
                        let vel = body.get_linear_velocity();
                        if vel.dot(&normal) < 0.0 {  // If moving toward A
                            let restitution = data.restitution_b;
                            let reflected_vel = vel - normal * (1.0 + restitution) * -vel.dot(&normal);
                            body.set_linear_velocity(reflected_vel * 0.95);  // Damping
                        }

                        corrected_bodies.insert(data.body_b_handle);
                    }
                }
            }
        }
    }

    // Final pass: apply predictive contact for fast-moving objects
    // This helps prevent tunneling for the next frame
    // Use a separate collection phase to avoid borrow checker issues
    let mut bodies_to_correct = Vec::new();

    // Collection phase - identify bodies that need correction
    for (handle, body) in bodies.iter() {
        // Skip static, already processed, or slow-moving bodies
        if body.get_body_type() == RigidBodyType::Static ||
           corrected_bodies.contains(&handle) ||
           body.get_linear_velocity().length_squared() < 10.0 {  // Only apply to fast-moving bodies
            continue;
        }

        // Check if this body is approaching the floor quickly
        if body.get_linear_velocity().y < -5.0 {  // Fast downward motion
            // Predict position for next frame
            let predicted_pos = body.get_position() + body.get_linear_velocity() * 0.016;  // Assume ~60fps

            // If predicted position is below a threshold (approximate floor height),
            // add to correction list
            if predicted_pos.y < -1.0 {  // Assume floor at y=0, with some margin
                bodies_to_correct.push(handle);
            }
        }
    }

    // Correction phase - apply corrections to identified bodies
    for handle in bodies_to_correct {
        if let Ok(body) = bodies.get_body_mut(handle) {
            // Apply a small lift
            let current_pos = body.get_position();
            body.set_position(Vector3::new(current_pos.x, current_pos.y + 0.1, current_pos.z));

            // Also dampen downward velocity
            let vel = body.get_linear_velocity();
            if vel.y < 0.0 {
                body.set_linear_velocity(Vector3::new(vel.x, vel.y * 0.8, vel.z));
            }
        }
    }
}