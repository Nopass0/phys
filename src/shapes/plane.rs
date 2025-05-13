use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;

/// An infinite plane collision shape defined by a normal and distance from origin
#[derive(Debug, Clone)]
pub struct Plane {
    /// The normal vector of the plane (must be normalized)
    normal: Vector3,
    
    /// The distance from the origin to the plane along the normal
    distance: f32,
}

impl Plane {
    /// Creates a new plane with the given normal and distance
    pub fn new(normal: Vector3, distance: f32) -> Self {
        Self {
            normal: normal.normalize(),
            distance,
        }
    }
    
    /// Creates a new plane from a normal and a point on the plane
    pub fn from_point_normal(point: Vector3, normal: Vector3) -> Self {
        let normal = normal.normalize();
        let distance = normal.dot(&point);
        Self { normal, distance }
    }
    
    /// Returns the normal of the plane
    pub fn get_normal(&self) -> Vector3 {
        self.normal
    }
    
    /// Returns the distance of the plane from the origin
    pub fn get_distance(&self) -> f32 {
        self.distance
    }
    
    /// Sets the normal of the plane (will be normalized)
    pub fn set_normal(&mut self, normal: Vector3) {
        self.normal = normal.normalize();
    }
    
    /// Sets the distance of the plane from the origin
    pub fn set_distance(&mut self, distance: f32) {
        self.distance = distance;
    }
    
    /// Returns the closest point on the plane to the given point
    pub fn closest_point_to(&self, point: Vector3) -> Vector3 {
        // Project the point onto the plane
        let signed_distance = self.normal.dot(&point) - self.distance;
        point - self.normal * signed_distance
    }
    
    /// Returns the signed distance from a point to the plane
    pub fn signed_distance_to(&self, point: Vector3) -> f32 {
        self.normal.dot(&point) - self.distance
    }
}

impl Shape for Plane {
    fn shape_type(&self) -> &'static str {
        "Plane"
    }
    
    fn get_volume(&self) -> f32 {
        // An infinite plane has zero volume
        0.0
    }
    
    fn get_inertia_tensor(&self, _mass: f32) -> Matrix3 {
        // An infinite plane has an infinite inertia tensor
        // For practical purposes, we use a zero tensor
        Matrix3::zero()
    }
    
    fn get_local_bounds(&self) -> Aabb {
        // An infinite plane doesn't have a finite AABB
        // For practical purposes, we create a large but finite AABB
        let large_number = 1000.0;
        
        // Choose the dimension along the normal to be thin
        let bounds_min = Vector3::new(
            if self.normal.x.abs() > 0.9 { -0.01 } else { -large_number },
            if self.normal.y.abs() > 0.9 { -0.01 } else { -large_number },
            if self.normal.z.abs() > 0.9 { -0.01 } else { -large_number },
        );
        
        let bounds_max = Vector3::new(
            if self.normal.x.abs() > 0.9 { 0.01 } else { large_number },
            if self.normal.y.abs() > 0.9 { 0.01 } else { large_number },
            if self.normal.z.abs() > 0.9 { 0.01 } else { large_number },
        );
        
        Aabb::new(bounds_min, bounds_max)
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        // Transform the local bounds
        let local_bounds = self.get_local_bounds();
        
        // For a plane, transform the vertices of the AABB
        let vertices = [
            Vector3::new(local_bounds.min.x, local_bounds.min.y, local_bounds.min.z),
            Vector3::new(local_bounds.max.x, local_bounds.min.y, local_bounds.min.z),
            Vector3::new(local_bounds.min.x, local_bounds.max.y, local_bounds.min.z),
            Vector3::new(local_bounds.max.x, local_bounds.max.y, local_bounds.min.z),
            Vector3::new(local_bounds.min.x, local_bounds.min.y, local_bounds.max.z),
            Vector3::new(local_bounds.max.x, local_bounds.min.y, local_bounds.max.z),
            Vector3::new(local_bounds.min.x, local_bounds.max.y, local_bounds.max.z),
            Vector3::new(local_bounds.max.x, local_bounds.max.y, local_bounds.max.z),
        ];
        
        // Transform vertices to world space
        let world_vertices: Vec<Vector3> = vertices.iter()
            .map(|v| transform.transform_point(*v))
            .collect();
        
        // Find the min and max of the transformed vertices
        Aabb::from_points(&world_vertices).unwrap_or_else(|| {
            // Fallback in case of empty points (should never happen)
            Aabb::from_center_half_extents(transform.position, Vector3::zero())
        })
    }
    
    fn get_support_point(&self, direction: Vector3) -> Vector3 {
        // For an infinite plane, there's no well-defined support point
        // However, for practical purposes, we can return a point on the plane
        // in the direction of the query if the query is away from the plane
        let dot = direction.dot(&self.normal);
        
        if dot > 0.0 {
            // Direction points away from the plane normal, use a large number
            direction.normalize() * 1000.0
        } else {
            // Direction points toward or along the plane
            // Return the plane origin (point closest to world origin)
            self.normal * self.distance
        }
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Ray-plane intersection
        let denominator = local_ray.direction.dot(&self.normal);
        
        // If the ray is parallel to the plane
        if denominator.abs() < crate::math::EPSILON {
            return None;
        }
        
        // Calculate the distance along the ray
        let t = (self.distance - local_ray.origin.dot(&self.normal)) / denominator;
        
        // Check if the intersection is within the ray's range
        if t >= 0.0 && t <= max_distance {
            Some(t)
        } else {
            None
        }
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