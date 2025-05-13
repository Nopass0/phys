use crate::core::{BodyHandle, ContactPoint};
use crate::bodies::RigidBody;
use crate::shapes::Shape;
use crate::math::{Vector3, Transform};
use crate::collision::contact_manifold::ContactManifold;

/// Trait for contact generation algorithms
pub trait ContactGenerator: Send + Sync {
    /// Generates contacts for the given manifolds
    fn generate_contacts(
        &self,
        manifolds: &mut [ContactManifold],
        bodies: &[(BodyHandle, &RigidBody)],
    );
}

/// Contact generator that refines GJK/EPA results to find contact points
pub struct GjkContactGenerator {
    /// Maximum number of contacts per manifold
    max_contacts: usize,
}

impl GjkContactGenerator {
    /// Creates a new GJK-based contact generator
    pub fn new(max_contacts: usize) -> Self {
        Self { max_contacts }
    }
    
    /// Generates contact points between a sphere and another shape
    fn generate_sphere_contacts(
        &self,
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
        normal: Vector3,
        penetration: f32,
    ) -> Vec<ContactPoint> {
        let mut contacts = Vec::new();
        
        // Check if shape_a is a sphere
        if let Some(sphere_a) = shape_a.as_any().downcast_ref::<crate::shapes::Sphere>() {
            let center_a = transform_a.position;
            let radius_a = sphere_a.get_radius() * transform_a.scale.x;
            
            // The contact point is along the normal from the sphere center
            let contact_point = center_a - normal * radius_a;
            
            contacts.push(ContactPoint {
                position: contact_point,
                normal,
                penetration,
            });
        } 
        // Check if shape_b is a sphere
        else if let Some(sphere_b) = shape_b.as_any().downcast_ref::<crate::shapes::Sphere>() {
            let center_b = transform_b.position;
            let radius_b = sphere_b.get_radius() * transform_b.scale.x;
            
            // The contact point is along the normal from the sphere center
            let contact_point = center_b + normal * radius_b;
            
            contacts.push(ContactPoint {
                position: contact_point,
                normal,
                penetration,
            });
        }
        
        contacts
    }
    
    /// Generates contact points between a box and another shape
    fn generate_box_contacts(
        &self,
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
        normal: Vector3,
        penetration: f32,
    ) -> Vec<ContactPoint> {
        let mut contacts = Vec::new();
        
        // Check if shape_a is a box
        if let Some(box_a) = shape_a.as_any().downcast_ref::<crate::shapes::BoxShape>() {
            // For simplicity, we'll just create a contact at the center of the face
            let center_a = transform_a.position;
            let half_extents = box_a.get_half_extents();
            
            // Find the face normal that's most aligned with the collision normal
            let abs_normal = Vector3::new(normal.x.abs(), normal.y.abs(), normal.z.abs());
            let face_normal = if abs_normal.x > abs_normal.y && abs_normal.x > abs_normal.z {
                Vector3::new(normal.x.signum(), 0.0, 0.0)
            } else if abs_normal.y > abs_normal.z {
                Vector3::new(0.0, normal.y.signum(), 0.0)
            } else {
                Vector3::new(0.0, 0.0, normal.z.signum())
            };
            
            // World-space face normal
            let face_normal_world = transform_a.transform_direction(face_normal);
            
            // Calculate the face center
            let mut face_center = Vector3::zero();
            if face_normal.x != 0.0 {
                face_center.x = half_extents.x * face_normal.x;
            } else if face_normal.y != 0.0 {
                face_center.y = half_extents.y * face_normal.y;
            } else {
                face_center.z = half_extents.z * face_normal.z;
            }
            
            // Transform to world space
            let face_center_world = transform_a.transform_point(face_center);
            
            // Create a contact point
            contacts.push(ContactPoint {
                position: face_center_world,
                normal,
                penetration,
            });
        }
        // Check if shape_b is a box
        else if let Some(box_b) = shape_b.as_any().downcast_ref::<crate::shapes::BoxShape>() {
            // For simplicity, we'll just create a contact at the center of the face
            let center_b = transform_b.position;
            let half_extents = box_b.get_half_extents();
            
            // Find the face normal that's most aligned with the collision normal
            let abs_normal = Vector3::new(normal.x.abs(), normal.y.abs(), normal.z.abs());
            let face_normal = if abs_normal.x > abs_normal.y && abs_normal.x > abs_normal.z {
                Vector3::new(-normal.x.signum(), 0.0, 0.0)
            } else if abs_normal.y > abs_normal.z {
                Vector3::new(0.0, -normal.y.signum(), 0.0)
            } else {
                Vector3::new(0.0, 0.0, -normal.z.signum())
            };
            
            // World-space face normal
            let face_normal_world = transform_b.transform_direction(face_normal);
            
            // Calculate the face center
            let mut face_center = Vector3::zero();
            if face_normal.x != 0.0 {
                face_center.x = half_extents.x * face_normal.x;
            } else if face_normal.y != 0.0 {
                face_center.y = half_extents.y * face_normal.y;
            } else {
                face_center.z = half_extents.z * face_normal.z;
            }
            
            // Transform to world space
            let face_center_world = transform_b.transform_point(face_center);
            
            // Create a contact point
            contacts.push(ContactPoint {
                position: face_center_world,
                normal,
                penetration,
            });
        }
        
        contacts
    }
}

impl ContactGenerator for GjkContactGenerator {
    fn generate_contacts(
        &self,
        manifolds: &mut [ContactManifold],
        bodies: &[(BodyHandle, &RigidBody)],
    ) {
        // Create a map for easier body lookups
        let body_map: std::collections::HashMap<BodyHandle, &RigidBody> = bodies
            .iter()
            .map(|(handle, body)| (*handle, *body))
            .collect();
        
        for manifold in manifolds {
            // Skip if the manifold already has contacts
            if !manifold.contacts.is_empty() {
                continue;
            }
            
            let body_a = match body_map.get(&manifold.pair.body_a) {
                Some(body) => body,
                None => continue,
            };
            
            let body_b = match body_map.get(&manifold.pair.body_b) {
                Some(body) => body,
                None => continue,
            };
            
            // Skip if either body doesn't have a shape
            let shape_a = match body_a.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            let shape_b = match body_b.get_shape() {
                Some(shape) => shape,
                None => continue,
            };
            
            // Check if we have a normal
            if manifold.normal.is_zero() {
                continue;
            }
            
            // Generate contacts based on the shape types
            let mut contacts = Vec::new();
            
            // Check for sphere contacts
            if shape_a.as_any().is::<crate::shapes::Sphere>() || shape_b.as_any().is::<crate::shapes::Sphere>() {
                contacts = self.generate_sphere_contacts(
                    shape_a.as_ref(),
                    &body_a.get_transform(),
                    shape_b.as_ref(),
                    &body_b.get_transform(),
                    manifold.normal,
                    0.0, // We don't have penetration info here
                );
            }
            // Check for box contacts
            else if shape_a.as_any().is::<crate::shapes::BoxShape>() || shape_b.as_any().is::<crate::shapes::BoxShape>() {
                contacts = self.generate_box_contacts(
                    shape_a.as_ref(),
                    &body_a.get_transform(),
                    shape_b.as_ref(),
                    &body_b.get_transform(),
                    manifold.normal,
                    0.0, // We don't have penetration info here
                );
            }
            
            // If we couldn't generate any contacts, create a basic one
            if contacts.is_empty() {
                // Create a single contact at the midpoint between the bodies
                let pos_a = body_a.get_position();
                let pos_b = body_b.get_position();
                let mid_point = (pos_a + pos_b) * 0.5;
                
                contacts.push(ContactPoint {
                    position: mid_point,
                    normal: manifold.normal,
                    penetration: 0.0, // We don't have penetration info here
                });
            }
            
            // Add the contacts to the manifold (up to the maximum)
            for contact in contacts.into_iter().take(self.max_contacts) {
                manifold.add_contact(contact);
            }
        }
    }
}

/// Contact generator for sphere-sphere collisions
pub struct SphereContactGenerator;

impl SphereContactGenerator {
    /// Creates a new sphere contact generator
    pub fn new() -> Self {
        Self
    }
}

impl ContactGenerator for SphereContactGenerator {
    fn generate_contacts(
        &self,
        manifolds: &mut [ContactManifold],
        bodies: &[(BodyHandle, &RigidBody)],
    ) {
        // Create a map for easier body lookups
        let body_map: std::collections::HashMap<BodyHandle, &RigidBody> = bodies
            .iter()
            .map(|(handle, body)| (*handle, *body))
            .collect();
        
        for manifold in manifolds {
            let body_a = match body_map.get(&manifold.pair.body_a) {
                Some(body) => body,
                None => continue,
            };
            
            let body_b = match body_map.get(&manifold.pair.body_b) {
                Some(body) => body,
                None => continue,
            };
            
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
            
            if let (Some(sphere_a), Some(sphere_b)) = (sphere_a, sphere_b) {
                // Get the positions and radii
                let pos_a = body_a.get_position();
                let pos_b = body_b.get_position();
                
                let radius_a = sphere_a.get_radius() * body_a.get_transform().scale.x;
                let radius_b = sphere_b.get_radius() * body_b.get_transform().scale.x;
                
                // Calculate the distance between the centers
                let delta = pos_b - pos_a;
                let dist_squared = delta.length_squared();
                
                // Skip if not colliding
                let sum_radii = radius_a + radius_b;
                if dist_squared >= sum_radii * sum_radii {
                    continue;
                }
                
                // Calculate the distance and penetration
                let dist = dist_squared.sqrt();
                let penetration = sum_radii - dist;
                
                // Calculate the normal
                let normal = if dist > 0.0 {
                    delta / dist
                } else {
                    // If the centers are at the same position, use a default normal
                    Vector3::new(0.0, 1.0, 0.0)
                };
                
                // Calculate the contact point
                let contact_point = pos_a + normal * radius_a;
                
                // Create a contact point
                let contact = ContactPoint {
                    position: contact_point,
                    normal,
                    penetration,
                };
                
                // Add the contact to the manifold
                manifold.clear();
                manifold.normal = normal;
                manifold.add_contact(contact);
            }
        }
    }
}