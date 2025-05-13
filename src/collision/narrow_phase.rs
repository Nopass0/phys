use crate::core::{BodyHandle, ContactPoint};
use crate::bodies::RigidBody;
use crate::collision::{
    collision_pair::CollisionPair,
    collision_filter::CollisionFilter,
    contact_manifold::ContactManifold,
    gjk::GJK
};
use crate::math::Vector3;

/// Trait for narrow-phase collision detection algorithms
pub trait NarrowPhase {
    /// Detects collisions between the given pairs of bodies
    fn detect_collisions(
        &self,
        pairs: &[CollisionPair],
        bodies: &[(BodyHandle, &RigidBody)],
        filter: &dyn CollisionFilter,
    ) -> Vec<ContactManifold>;
}

/// GJK/EPA-based narrow-phase algorithm
pub struct GjkNarrowPhase;

impl GjkNarrowPhase {
    /// Creates a new GJK/EPA-based narrow-phase
    pub fn new() -> Self {
        Self
    }
}

impl NarrowPhase for GjkNarrowPhase {
    fn detect_collisions(
        &self,
        pairs: &[CollisionPair],
        bodies: &[(BodyHandle, &RigidBody)],
        filter: &dyn CollisionFilter,
    ) -> Vec<ContactManifold> {
        let mut manifolds = Vec::new();
        
        // Create a map for easier body lookups
        let body_map: std::collections::HashMap<BodyHandle, &RigidBody> = bodies
            .iter()
            .map(|(handle, body)| (*handle, *body))
            .collect();
        
        // Check each pair for collision
        for pair in pairs {
            let body_a = match body_map.get(&pair.body_a) {
                Some(body) => body,
                None => continue,
            };
            
            let body_b = match body_map.get(&pair.body_b) {
                Some(body) => body,
                None => continue,
            };
            
            // Check if the bodies should collide
            if !filter.should_collide(body_a, body_b) {
                continue;
            }
            
            // Skip if either body doesn't have a shape
            let shape_a = match body_a.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            let shape_b = match body_b.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            // Check for collision using GJK/EPA
            if let Some((penetration, normal)) = GJK::detect_collision(
                shape_a.as_ref(),
                &body_a.get_transform(),
                shape_b.as_ref(),
                &body_b.get_transform(),
            ) {
                // Create a contact manifold
                let mut manifold = ContactManifold::new(*pair);
                
                // Set the normal
                manifold.normal = normal;
                
                // Create a simple contact point at the center of the penetration
                let point_a = body_a.get_position() + normal * (penetration * 0.5);
                
                let contact = ContactPoint {
                    position: point_a,
                    normal,
                    penetration,
                };
                
                manifold.add_contact(contact);
                
                // Set the material properties
                manifold.set_material_properties(
                    body_a.get_material().restitution.max(body_b.get_material().restitution),
                    body_a.get_material().friction.min(body_b.get_material().friction),
                );
                
                // Add the manifold
                manifolds.push(manifold);
            }
        }
        
        manifolds
    }
}

/// Sphere-based narrow-phase algorithm (handles sphere-sphere collisions efficiently)
pub struct SphereNarrowPhase {
    /// Fallback narrow-phase for non-sphere shapes
    fallback: Box<dyn NarrowPhase>,
}

impl SphereNarrowPhase {
    /// Creates a new sphere-based narrow-phase
    pub fn new(fallback: Box<dyn NarrowPhase>) -> Self {
        Self { fallback }
    }
    
    /// Detects collision between two spheres
    fn detect_sphere_collision(
        &self,
        body_a: &RigidBody,
        body_b: &RigidBody,
        shape_a: &crate::shapes::Sphere,
        shape_b: &crate::shapes::Sphere,
    ) -> Option<ContactManifold> {
        let pos_a = body_a.get_position();
        let pos_b = body_b.get_position();
        
        let radius_a = shape_a.get_radius() * body_a.get_transform().scale.x;
        let radius_b = shape_b.get_radius() * body_b.get_transform().scale.x;
        
        let dist_sq = (pos_b - pos_a).length_squared();
        let sum_radius = radius_a + radius_b;
        
        if dist_sq < sum_radius * sum_radius {
            // Calculate penetration depth
            let dist = dist_sq.sqrt();
            let penetration = sum_radius - dist;
            
            // Calculate contact normal
            let normal = if dist > 0.0 {
                (pos_b - pos_a) / dist
            } else {
                // If the centers are at the same position, use a default normal
                Vector3::new(0.0, 1.0, 0.0)
            };
            
            // Create a contact manifold
            let pair = CollisionPair::new(
                crate::core::BodyHandle(0), // These will be filled in by the caller
                crate::core::BodyHandle(0),
            );
            let mut manifold = ContactManifold::new(pair);
            
            // Set the normal
            manifold.normal = normal;
            
            // Create a contact point
            let contact_point = pos_a + normal * radius_a;
            let contact = ContactPoint {
                position: contact_point,
                normal,
                penetration,
            };
            
            manifold.add_contact(contact);
            
            // Set the material properties
            manifold.set_material_properties(
                body_a.get_material().restitution.max(body_b.get_material().restitution),
                body_a.get_material().friction.min(body_b.get_material().friction),
            );
            
            Some(manifold)
        } else {
            None
        }
    }
}

impl NarrowPhase for SphereNarrowPhase {
    fn detect_collisions(
        &self,
        pairs: &[CollisionPair],
        bodies: &[(BodyHandle, &RigidBody)],
        filter: &dyn CollisionFilter,
    ) -> Vec<ContactManifold> {
        let mut manifolds = Vec::new();
        let mut non_sphere_pairs = Vec::new();
        
        // Create a map for easier body lookups
        let body_map: std::collections::HashMap<BodyHandle, &RigidBody> = bodies
            .iter()
            .map(|(handle, body)| (*handle, *body))
            .collect();
        
        // Check each pair for collision
        for pair in pairs {
            let body_a = match body_map.get(&pair.body_a) {
                Some(body) => body,
                None => continue,
            };
            
            let body_b = match body_map.get(&pair.body_b) {
                Some(body) => body,
                None => continue,
            };
            
            // Check if the bodies should collide
            if !filter.should_collide(body_a, body_b) {
                continue;
            }
            
            // Skip if either body doesn't have a shape
            let shape_a = match body_a.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            let shape_b = match body_b.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            // Check if both shapes are spheres
            let sphere_a = shape_a.as_ref().as_any().downcast_ref::<crate::shapes::Sphere>();
            let sphere_b = shape_b.as_ref().as_any().downcast_ref::<crate::shapes::Sphere>();
            
            match (sphere_a, sphere_b) {
                (Some(sphere_a), Some(sphere_b)) => {
                    // Use sphere-sphere collision detection
                    if let Some(mut manifold) = self.detect_sphere_collision(body_a, body_b, sphere_a, sphere_b) {
                        // Fix the collision pair
                        manifold.pair = *pair;
                        manifolds.push(manifold);
                    }
                }
                _ => {
                    // Use the fallback for non-sphere shapes
                    non_sphere_pairs.push(*pair);
                }
            }
        }
        
        // Handle non-sphere pairs with the fallback
        if !non_sphere_pairs.is_empty() {
            let fallback_manifolds = self.fallback.detect_collisions(&non_sphere_pairs, bodies, filter);
            manifolds.extend(fallback_manifolds);
        }
        
        manifolds
    }
}