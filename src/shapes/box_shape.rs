use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;

/// A box (cuboid) collision shape
#[derive(Debug, Clone)]
pub struct BoxShape {
    /// The half-extents of the box (half-width, half-height, half-depth)
    half_extents: Vector3,
}

impl BoxShape {
    /// Creates a new box with the given half-extents
    pub fn new(half_extents: Vector3) -> Self {
        Self {
            half_extents: Vector3::new(
                half_extents.x.max(0.0),
                half_extents.y.max(0.0),
                half_extents.z.max(0.0),
            ),
        }
    }
    
    /// Creates a new box with the given full dimensions
    pub fn new_with_dimensions(width: f32, height: f32, depth: f32) -> Self {
        Self::new(Vector3::new(
            width.max(0.0) * 0.5,
            height.max(0.0) * 0.5,
            depth.max(0.0) * 0.5,
        ))
    }
    
    /// Returns the half-extents of the box
    pub fn get_half_extents(&self) -> Vector3 {
        self.half_extents
    }
    
    /// Sets the half-extents of the box
    pub fn set_half_extents(&mut self, half_extents: Vector3) {
        self.half_extents = Vector3::new(
            half_extents.x.max(0.0),
            half_extents.y.max(0.0),
            half_extents.z.max(0.0),
        );
    }
    
    /// Returns the full dimensions of the box
    pub fn get_dimensions(&self) -> Vector3 {
        self.half_extents * 2.0
    }
    
    /// Sets the full dimensions of the box
    pub fn set_dimensions(&mut self, dimensions: Vector3) {
        self.half_extents = Vector3::new(
            dimensions.x.max(0.0) * 0.5,
            dimensions.y.max(0.0) * 0.5,
            dimensions.z.max(0.0) * 0.5,
        );
    }
    
    /// Returns the 8 vertices of the box in local space
    pub fn get_vertices(&self) -> [Vector3; 8] {
        let x = self.half_extents.x;
        let y = self.half_extents.y;
        let z = self.half_extents.z;
        
        [
            Vector3::new(-x, -y, -z), // 0: -X, -Y, -Z
            Vector3::new(x, -y, -z),  // 1: +X, -Y, -Z
            Vector3::new(x, y, -z),   // 2: +X, +Y, -Z
            Vector3::new(-x, y, -z),  // 3: -X, +Y, -Z
            Vector3::new(-x, -y, z),  // 4: -X, -Y, +Z
            Vector3::new(x, -y, z),   // 5: +X, -Y, +Z
            Vector3::new(x, y, z),    // 6: +X, +Y, +Z
            Vector3::new(-x, y, z),   // 7: -X, +Y, +Z
        ]
    }
}

impl Shape for BoxShape {
    fn shape_type(&self) -> &'static str {
        "Box"
    }
    
    fn get_volume(&self) -> f32 {
        // Volume of a box: width * height * depth
        self.half_extents.x * 2.0 * self.half_extents.y * 2.0 * self.half_extents.z * 2.0
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        // Inertia tensor of a box:
        // Ixx = (1/12) * m * (y^2 + z^2)
        // Iyy = (1/12) * m * (x^2 + z^2)
        // Izz = (1/12) * m * (x^2 + y^2)
        let x2 = (self.half_extents.x * 2.0).powi(2);
        let y2 = (self.half_extents.y * 2.0).powi(2);
        let z2 = (self.half_extents.z * 2.0).powi(2);
        
        let factor = mass / 12.0;
        
        Matrix3::new([
            [factor * (y2 + z2), 0.0, 0.0],
            [0.0, factor * (x2 + z2), 0.0],
            [0.0, 0.0, factor * (x2 + y2)],
        ])
    }
    
    fn get_local_bounds(&self) -> Aabb {
        // Local AABB is defined by the half-extents
        Aabb::new(
            -self.half_extents,
            self.half_extents,
        )
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        // For a box, we need to transform all 8 vertices and find the min/max
        let vertices = self.get_vertices();
        
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
        if direction.is_zero() {
            return Vector3::zero();
        }
        
        // Support point is the vertex in the direction of the input vector
        Vector3::new(
            if direction.x >= 0.0 { self.half_extents.x } else { -self.half_extents.x },
            if direction.y >= 0.0 { self.half_extents.y } else { -self.half_extents.y },
            if direction.z >= 0.0 { self.half_extents.z } else { -self.half_extents.z },
        )
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Ray-box intersection using slab method
        let mut t_min = -f32::MAX;
        let mut t_max = f32::MAX;
        
        // Check X slabs
        if local_ray.direction.x.abs() < crate::math::EPSILON {
            // Ray is parallel to X slabs, check if origin is between slabs
            if local_ray.origin.x < -self.half_extents.x || local_ray.origin.x > self.half_extents.x {
                return None;
            }
        } else {
            let inv_d = 1.0 / local_ray.direction.x;
            let mut t1 = (-self.half_extents.x - local_ray.origin.x) * inv_d;
            let mut t2 = (self.half_extents.x - local_ray.origin.x) * inv_d;
            
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            
            t_min = t1.max(t_min);
            t_max = t2.min(t_max);
            
            if t_min > t_max {
                return None;
            }
        }
        
        // Check Y slabs
        if local_ray.direction.y.abs() < crate::math::EPSILON {
            // Ray is parallel to Y slabs, check if origin is between slabs
            if local_ray.origin.y < -self.half_extents.y || local_ray.origin.y > self.half_extents.y {
                return None;
            }
        } else {
            let inv_d = 1.0 / local_ray.direction.y;
            let mut t1 = (-self.half_extents.y - local_ray.origin.y) * inv_d;
            let mut t2 = (self.half_extents.y - local_ray.origin.y) * inv_d;
            
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            
            t_min = t1.max(t_min);
            t_max = t2.min(t_max);
            
            if t_min > t_max {
                return None;
            }
        }
        
        // Check Z slabs
        if local_ray.direction.z.abs() < crate::math::EPSILON {
            // Ray is parallel to Z slabs, check if origin is between slabs
            if local_ray.origin.z < -self.half_extents.z || local_ray.origin.z > self.half_extents.z {
                return None;
            }
        } else {
            let inv_d = 1.0 / local_ray.direction.z;
            let mut t1 = (-self.half_extents.z - local_ray.origin.z) * inv_d;
            let mut t2 = (self.half_extents.z - local_ray.origin.z) * inv_d;
            
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            
            t_min = t1.max(t_min);
            t_max = t2.min(t_max);
            
            if t_min > t_max {
                return None;
            }
        }
        
        // Check if the intersection is within the ray's range
        if t_min > max_distance || t_min < 0.0 {
            if t_max > 0.0 && t_max <= max_distance {
                return Some(t_max);
            }
            return None;
        }
        
        Some(t_min)
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_shape(&self) -> std::boxed::Box<dyn Shape> {
        std::boxed::Box::new(self.clone())
    }
}