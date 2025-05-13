use crate::core::{BodyStorage, Storage};
use crate::bodies::{RigidBody, RigidBodyType};
use crate::collision::contact_manifold::ContactManifold;
use crate::math::Vector3;

/// Trait for contact constraint solvers
pub trait ContactSolver {
    /// Prepares the solver for the given contacts
    fn prepare(&mut self, contacts: &mut [ContactManifold], bodies: &BodyStorage<RigidBody>);
    
    /// Solves velocity constraints
    fn solve_velocity(&mut self, contacts: &mut [ContactManifold], bodies: &mut BodyStorage<RigidBody>, dt: f32);
    
    /// Solves position constraints
    fn solve_position(&mut self, contacts: &mut [ContactManifold], bodies: &mut BodyStorage<RigidBody>, dt: f32);
}

/// Sequential impulse-based contact solver
pub struct SequentialImpulseSolver {
    /// Bias factor for the constraint solver (Baumgarte stabilization)
    bias_factor: f32,
    
    /// Velocity threshold for restitution
    restitution_threshold: f32,
}

impl SequentialImpulseSolver {
    /// Creates a new sequential impulse solver
    pub fn new(bias_factor: f32, restitution_threshold: f32) -> Self {
        Self {
            bias_factor,
            restitution_threshold,
        }
    }
}

impl ContactSolver for SequentialImpulseSolver {
    fn prepare(&mut self, contacts: &mut [ContactManifold], bodies: &BodyStorage<RigidBody>) {
        for manifold in contacts {
            let body_a = match bodies.get(manifold.pair.body_a) {
                Some(body) => body,
                None => continue,
            };
            
            let body_b = match bodies.get(manifold.pair.body_b) {
                Some(body) => body,
                None => continue,
            };
            
            // Calculate the effective restitution and friction
            let restitution = body_a.get_material().restitution.max(body_b.get_material().restitution);
            let friction = body_a.get_material().friction.min(body_b.get_material().friction);
            
            manifold.restitution = restitution;
            manifold.friction = friction;
        }
    }
    
    fn solve_velocity(&mut self, contacts: &mut [ContactManifold], bodies: &mut BodyStorage<RigidBody>, dt: f32) {
        for manifold in contacts {
            // Get both bodies, but don't keep the mutable references across the loop
            let body_a_handle = manifold.pair.body_a;
            let body_b_handle = manifold.pair.body_b;

            // Store all impulses to apply after the calculation
            let mut body_a_impulses = Vec::new();
            let mut body_b_impulses = Vec::new();

            // Gather all the data we need with immutable borrows
            let (
                body_a_type,
                inv_mass_a,
                inv_inertia_a,
                vel_a,
                ang_vel_a,
                pos_a
            );

            match bodies.get(body_a_handle) {
                Some(body) => {
                    body_a_type = body.get_body_type();
                    inv_mass_a = body.get_inverse_mass();
                    inv_inertia_a = body.get_inverse_inertia_tensor_world();
                    vel_a = body.get_linear_velocity();
                    ang_vel_a = body.get_angular_velocity();
                    pos_a = body.get_position();
                },
                None => continue,
            };

            let (
                body_b_type,
                inv_mass_b,
                inv_inertia_b,
                vel_b,
                ang_vel_b,
                pos_b
            );

            match bodies.get(body_b_handle) {
                Some(body) => {
                    body_b_type = body.get_body_type();
                    inv_mass_b = body.get_inverse_mass();
                    inv_inertia_b = body.get_inverse_inertia_tensor_world();
                    vel_b = body.get_linear_velocity();
                    ang_vel_b = body.get_angular_velocity();
                    pos_b = body.get_position();
                },
                None => continue,
            };

            // Skip if both bodies are static
            if body_a_type == RigidBodyType::Static && body_b_type == RigidBodyType::Static {
                continue;
            }

            let normal = manifold.normal;
            let tangent = if normal.cross(&Vector3::new(0.0, 1.0, 0.0)).length_squared() > 0.01 {
                normal.cross(&Vector3::new(0.0, 1.0, 0.0)).normalize()
            } else {
                normal.cross(&Vector3::new(1.0, 0.0, 0.0)).normalize()
            };

            let bitangent = normal.cross(&tangent);

            // Process all contacts and collect impulses to apply
            for contact in &manifold.contacts {
                // Calculate relative position from center of mass to contact point
                let r_a = contact.position - pos_a;
                let r_b = contact.position - pos_b;

                // Calculate relative velocity at the contact point
                let rel_vel = vel_b + ang_vel_b.cross(&r_b) - vel_a - ang_vel_a.cross(&r_a);

                // Calculate normal velocity
                let normal_vel = rel_vel.dot(&normal);

                // Skip separating contacts
                if normal_vel > 0.0 {
                    continue;
                }

                // Calculate the effective mass for the normal constraint
                let r_a_cross_n = r_a.cross(&normal);
                let r_b_cross_n = r_b.cross(&normal);

                let normal_mass = inv_mass_a + inv_mass_b +
                    r_a_cross_n.dot(&inv_inertia_a.multiply_vector(r_a_cross_n)) +
                    r_b_cross_n.dot(&inv_inertia_b.multiply_vector(r_b_cross_n));

                if normal_mass <= 0.0 {
                    continue;
                }

                let normal_mass_inv = 1.0 / normal_mass;

                // Calculate the restitution
                let restitution = if normal_vel.abs() < self.restitution_threshold {
                    0.0
                } else {
                    manifold.restitution
                };

                // Calculate the velocity bias - FIXED to use actual bias calculation
                // The bias factor will help prevent penetration by applying velocity correction
                let bias = -self.bias_factor * contact.penetration / dt;
                // Use higher bias for severe penetrations
                let bias = if contact.penetration > 0.05 { bias * 2.0 } else { bias };

                // Calculate the normal impulse
                let j_n = -(1.0 + restitution) * normal_vel * normal_mass_inv + bias;

                // Calculate and store impulses if needed
                if j_n > 0.0 {
                    let normal_impulse = normal * j_n;

                    // Calculate tangent impulses (friction)
                    let tangent_vel = rel_vel.dot(&tangent);
                    let bitangent_vel = rel_vel.dot(&bitangent);

                    // Calculate the effective mass for the tangent constraint
                    let r_a_cross_t = r_a.cross(&tangent);
                    let r_b_cross_t = r_b.cross(&tangent);

                    let tangent_mass = inv_mass_a + inv_mass_b +
                        r_a_cross_t.dot(&inv_inertia_a.multiply_vector(r_a_cross_t)) +
                        r_b_cross_t.dot(&inv_inertia_b.multiply_vector(r_b_cross_t));

                    // Calculate the effective mass for the bitangent constraint
                    let r_a_cross_bt = r_a.cross(&bitangent);
                    let r_b_cross_bt = r_b.cross(&bitangent);

                    let bitangent_mass = inv_mass_a + inv_mass_b +
                        r_a_cross_bt.dot(&inv_inertia_a.multiply_vector(r_a_cross_bt)) +
                        r_b_cross_bt.dot(&inv_inertia_b.multiply_vector(r_b_cross_bt));

                    // Calculate friction impulses
                    let tangent_mass_inv = if tangent_mass > 0.0 { 1.0 / tangent_mass } else { 0.0 };
                    let bitangent_mass_inv = if bitangent_mass > 0.0 { 1.0 / bitangent_mass } else { 0.0 };

                    let j_t = -tangent_vel * tangent_mass_inv;
                    let j_bt = -bitangent_vel * bitangent_mass_inv;

                    // Calculate the maximum friction impulse
                    let max_friction = manifold.friction * j_n;

                    // Apply the tangent impulse (with friction)
                    let impulse_t = (j_t.abs().min(max_friction) * j_t.signum()) * tangent;
                    let impulse_bt = (j_bt.abs().min(max_friction) * j_bt.signum()) * bitangent;

                    let tangent_impulse = impulse_t + impulse_bt;

                    // Store all the impulses to apply later
                    if body_a_type != RigidBodyType::Static {
                        let impulse_a_linear = -(normal_impulse + tangent_impulse);
                        let impulse_a_angular = -r_a.cross(&(normal_impulse + tangent_impulse));
                        body_a_impulses.push((impulse_a_linear, impulse_a_angular));
                    }

                    if body_b_type != RigidBodyType::Static {
                        let impulse_b_linear = normal_impulse + tangent_impulse;
                        let impulse_b_angular = r_b.cross(&(normal_impulse + tangent_impulse));
                        body_b_impulses.push((impulse_b_linear, impulse_b_angular));
                    }
                }
            }

            // Now apply all impulses for body A with a single mutable borrow
            if !body_a_impulses.is_empty() {
                if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                    for (linear, angular) in body_a_impulses {
                        body_a.apply_impulse(linear);
                        body_a.apply_angular_impulse(angular);
                    }
                }
            }

            // Apply all impulses for body B with a single mutable borrow
            if !body_b_impulses.is_empty() {
                if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                    for (linear, angular) in body_b_impulses {
                        body_b.apply_impulse(linear);
                        body_b.apply_angular_impulse(angular);
                    }
                }
            }
        }
    }
    
    fn solve_position(&mut self, contacts: &mut [ContactManifold], bodies: &mut BodyStorage<RigidBody>, dt: f32) {
        for manifold in contacts {
            // Get body handles
            let body_a_handle = manifold.pair.body_a;
            let body_b_handle = manifold.pair.body_b;

            // Store corrections to apply after calculation
            let mut body_a_transform = None;
            let mut body_b_transform = None;

            // First get immutable references to gather all needed data
            let (
                body_a_type,
                inv_mass_a,
                inv_inertia_a,
                pos_a,
                transform_a
            );

            match bodies.get(body_a_handle) {
                Some(body) => {
                    body_a_type = body.get_body_type();
                    inv_mass_a = body.get_inverse_mass();
                    inv_inertia_a = body.get_inverse_inertia_tensor_world();
                    pos_a = body.get_position();
                    transform_a = body.get_transform();
                },
                None => continue,
            };

            let (
                body_b_type,
                inv_mass_b,
                inv_inertia_b,
                pos_b,
                transform_b
            );

            match bodies.get(body_b_handle) {
                Some(body) => {
                    body_b_type = body.get_body_type();
                    inv_mass_b = body.get_inverse_mass();
                    inv_inertia_b = body.get_inverse_inertia_tensor_world();
                    pos_b = body.get_position();
                    transform_b = body.get_transform();
                },
                None => continue,
            };

            // Skip if both bodies are static
            if body_a_type == RigidBodyType::Static && body_b_type == RigidBodyType::Static {
                continue;
            }

            let normal = manifold.normal;

            // Create new transforms for both bodies
            let mut new_transform_a = transform_a.clone();
            let mut new_transform_b = transform_b.clone();
            let mut transform_modified = false;

            for contact in &manifold.contacts {
                // Calculate relative position from center of mass to contact point
                let r_a = contact.position - pos_a;
                let r_b = contact.position - pos_b;

                // Calculate the effective mass for the normal constraint
                let r_a_cross_n = r_a.cross(&normal);
                let r_b_cross_n = r_b.cross(&normal);

                let normal_mass = inv_mass_a + inv_mass_b +
                    r_a_cross_n.dot(&inv_inertia_a.multiply_vector(r_a_cross_n)) +
                    r_b_cross_n.dot(&inv_inertia_b.multiply_vector(r_b_cross_n));

                if normal_mass <= 0.0 {
                    continue;
                }

                let normal_mass_inv = 1.0 / normal_mass;

                // Calculate the position correction with zero slop for maximum accuracy
                let slop = 0.0; // No slop for most aggressive correction possible
                let penetration = (contact.penetration - slop).max(0.0);
                // Use extremely aggressive position correction with maximum bias factor
                let correction = penetration * normal_mass_inv * (self.bias_factor * 3.0);
                // Quadruple the correction for severe penetrations
                let correction = if contact.penetration > 0.05 { correction * 4.0 } else { correction };

                // Calculate the correction impulse
                let correction_impulse = normal * correction;

                // Apply corrections to the transforms
                if body_a_type != RigidBodyType::Static {
                    let pos_correction = correction_impulse * inv_mass_a;
                    new_transform_a.position -= pos_correction;
                    transform_modified = true;
                    body_a_transform = Some(new_transform_a.clone());
                }

                if body_b_type != RigidBodyType::Static {
                    let pos_correction = correction_impulse * inv_mass_b;
                    new_transform_b.position += pos_correction;
                    transform_modified = true;
                    body_b_transform = Some(new_transform_b.clone());
                }
            }

            // Apply transforms with separate mutable borrows
            if transform_modified {
                if let Some(new_transform) = body_a_transform {
                    if let Ok(body_a) = bodies.get_body_mut(body_a_handle) {
                        body_a.set_transform(new_transform);
                    }
                }

                if let Some(new_transform) = body_b_transform {
                    if let Ok(body_b) = bodies.get_body_mut(body_b_handle) {
                        body_b.set_transform(new_transform);
                    }
                }
            }
        }
    }
}