use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;

/// A spherical collision shape
#[derive(Debug, Clone)]
pub struct Sphere {
    /// The radius of the sphere
    radius: f32,
}

impl Sphere {
    /// Creates a new sphere with the given radius
    pub fn new(radius: f32) -> Self {
        Self {
            radius: radius.max(0.0),
        }
    }
    
    /// Returns the radius of the sphere
    pub fn get_radius(&self) -> f32 {
        self.radius
    }
    
    /// Sets the radius of the sphere
    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius.max(0.0);
    }
}

impl Shape for Sphere {
    fn shape_type(&self) -> &'static str {
        "Sphere"
    }
    
    fn get_volume(&self) -> f32 {
        // Volume of a sphere: (4/3) * π * r^3
        (4.0 / 3.0) * std::f32::consts::PI * self.radius.powi(3)
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        // Inertia tensor of a sphere:
        // The inertia of a sphere is the same along all axes
        // I = (2/5) * mass * radius^2
        let inertia = (2.0 / 5.0) * mass * self.radius.powi(2);
        
        Matrix3::new([
            [inertia, 0.0, 0.0],
            [0.0, inertia, 0.0],
            [0.0, 0.0, inertia],
        ])
    }
    
    fn get_local_bounds(&self) -> Aabb {
        // Local AABB is a cube with sides of length 2*radius
        let half_size = Vector3::new(self.radius, self.radius, self.radius);
        
        Aabb::new(
            -half_size,
            half_size,
        )
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        // World AABB is the local AABB transformed by position only
        // For a sphere, we don't need to account for rotation
        let half_size = Vector3::new(
            self.radius * transform.scale.x,
            self.radius * transform.scale.y,
            self.radius * transform.scale.z,
        );
        
        Aabb::new(
            transform.position - half_size,
            transform.position + half_size,
        )
    }
    
    fn get_support_point(&self, direction: Vector3) -> Vector3 {
        if direction.is_zero() {
            return Vector3::zero();
        }
        
        // Support point is in the direction of the input vector, at distance radius
        direction.normalize() * self.radius
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Simple sphere-ray intersection
        let sphere_to_ray = local_ray.origin;
        let a = local_ray.direction.length_squared();
        let b = 2.0 * sphere_to_ray.dot(&local_ray.direction);
        let c = sphere_to_ray.length_squared() - self.radius.powi(2);
        
        let discriminant = b * b - 4.0 * a * c;
        
        if discriminant < 0.0 {
            // No intersection
            return None;
        }
        
        let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
        let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
        
        // Check if closest intersection is within max distance
        if t1 > 0.0 && t1 <= max_distance {
            return Some(t1);
        }
        
        if t2 > 0.0 && t2 <= max_distance {
            return Some(t2);
        }
        
        None
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_shape(&self) -> Box<dyn Shape> {
        Box::new(self.clone())
    }
}