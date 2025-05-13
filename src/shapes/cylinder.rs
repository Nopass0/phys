use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;
use std::f32::consts::PI;

/// A cylinder collision shape
#[derive(Debug, Clone)]
pub struct Cylinder {
    /// The radius of the cylinder
    radius: f32,
    
    /// The height of the cylinder
    height: f32,
}

impl Cylinder {
    /// Creates a new cylinder with the given radius and height
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius: radius.max(0.0),
            height: height.max(0.0),
        }
    }
    
    /// Returns the radius of the cylinder
    pub fn get_radius(&self) -> f32 {
        self.radius
    }
    
    /// Sets the radius of the cylinder
    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius.max(0.0);
    }
    
    /// Returns the height of the cylinder
    pub fn get_height(&self) -> f32 {
        self.height
    }
    
    /// Sets the height of the cylinder
    pub fn set_height(&mut self, height: f32) {
        self.height = height.max(0.0);
    }
    
    /// Returns the half-height of the cylinder
    pub fn get_half_height(&self) -> f32 {
        self.height * 0.5
    }
}

impl Shape for Cylinder {
    fn shape_type(&self) -> &'static str {
        "Cylinder"
    }
    
    fn get_volume(&self) -> f32 {
        PI * self.radius.powi(2) * self.height
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        let r2 = self.radius.powi(2);
        let h2 = self.height.powi(2);
        
        // Formula for solid cylinder
        let ix = (1.0 / 12.0) * mass * (3.0 * r2 + h2);
        let iz = ix;
        let iy = 0.5 * mass * r2;
        
        Matrix3::new([
            [ix, 0.0, 0.0],
            [0.0, iy, 0.0],
            [0.0, 0.0, iz],
        ])
    }
    
    fn get_local_bounds(&self) -> Aabb {
        let half_height = self.height * 0.5;
        let extent = Vector3::new(
            self.radius,
            half_height,
            self.radius,
        );
        
        Aabb::new(-extent, extent)
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        // Get the eight corners of the local AABB
        let local_bounds = self.get_local_bounds();
        let corners = [
            Vector3::new(local_bounds.min.x, local_bounds.min.y, local_bounds.min.z),
            Vector3::new(local_bounds.max.x, local_bounds.min.y, local_bounds.min.z),
            Vector3::new(local_bounds.min.x, local_bounds.max.y, local_bounds.min.z),
            Vector3::new(local_bounds.max.x, local_bounds.max.y, local_bounds.min.z),
            Vector3::new(local_bounds.min.x, local_bounds.min.y, local_bounds.max.z),
            Vector3::new(local_bounds.max.x, local_bounds.min.y, local_bounds.max.z),
            Vector3::new(local_bounds.min.x, local_bounds.max.y, local_bounds.max.z),
            Vector3::new(local_bounds.max.x, local_bounds.max.y, local_bounds.max.z),
        ];
        
        // Transform each corner to world space
        let mut min = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut max = Vector3::new(f32::MIN, f32::MIN, f32::MIN);
        
        for corner in &corners {
            let world_corner = transform.transform_point(*corner);
            
            min.x = min.x.min(world_corner.x);
            min.y = min.y.min(world_corner.y);
            min.z = min.z.min(world_corner.z);
            
            max.x = max.x.max(world_corner.x);
            max.y = max.y.max(world_corner.y);
            max.z = max.z.max(world_corner.z);
        }
        
        Aabb::new(min, max)
    }
    
    fn get_support_point(&self, direction: Vector3) -> Vector3 {
        if direction.is_zero() {
            return Vector3::zero();
        }
        
        // Normalize the direction
        let dir = direction.normalize();
        
        // The cylinder is aligned with the Y-axis in local space
        let half_height = self.height * 0.5;
        
        // Project the direction onto the Y-axis
        let axis_dir = Vector3::new(0.0, 1.0, 0.0);
        let axis_proj = axis_dir.dot(&dir);
        
        // Set the Y component based on the sign of the projection
        let mut extreme_point = Vector3::zero();
        if axis_proj > 0.0 {
            extreme_point.y = half_height;
        } else if axis_proj < 0.0 {
            extreme_point.y = -half_height;
        }
        
        // Find the extreme point on the circular edge
        let perp_dir = Vector3::new(dir.x, 0.0, dir.z);
        let perp_length = perp_dir.length();
        
        if perp_length > crate::math::EPSILON {
            let normalized_perp = perp_dir / perp_length;
            extreme_point.x = normalized_perp.x * self.radius;
            extreme_point.z = normalized_perp.z * self.radius;
        }
        
        extreme_point
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // The cylinder is aligned with the Y-axis in local space
        let ray_origin = local_ray.origin;
        let ray_dir = local_ray.direction;
        let half_height = self.height * 0.5;
        
        // Check infinite cylinder intersection (XZ plane)
        let ray_origin_xz = Vector3::new(ray_origin.x, 0.0, ray_origin.z);
        let ray_dir_xz = Vector3::new(ray_dir.x, 0.0, ray_dir.z);
        
        let a = ray_dir_xz.length_squared();
        
        // If ray is parallel to Y-axis
        if a < crate::math::EPSILON {
            // Check if ray is inside the cylinder
            if ray_origin_xz.length_squared() <= self.radius.powi(2) {
                // Check intersection with top and bottom caps
                let t_bottom = (-half_height - ray_origin.y) / ray_dir.y;
                let t_top = (half_height - ray_origin.y) / ray_dir.y;
                
                // Get the closest hit in front of the ray
                let mut t = f32::MAX;
                if t_bottom > 0.0 && t_bottom < t && t_bottom <= max_distance {
                    t = t_bottom;
                }
                if t_top > 0.0 && t_top < t && t_top <= max_distance {
                    t = t_top;
                }
                
                if t != f32::MAX {
                    return Some(t);
                }
            }
            
            return None;
        }
        
        // Ray not parallel to Y-axis
        let b = 2.0 * ray_origin_xz.dot(&ray_dir_xz);
        let c = ray_origin_xz.length_squared() - self.radius.powi(2);
        
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None; // No infinite cylinder intersection
        }
        
        // Calculate cylinder intersection points
        let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
        let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
        
        // Check if cylinder intersections are within height bounds
        let y1 = ray_origin.y + ray_dir.y * t1;
        let y2 = ray_origin.y + ray_dir.y * t2;
        
        // Closest valid intersection with cylinder body
        let mut t_cylinder = f32::MAX;
        
        if t1 > 0.0 && y1.abs() <= half_height && t1 <= max_distance {
            t_cylinder = t1;
        }
        
        if t2 > 0.0 && y2.abs() <= half_height && t2 < t_cylinder && t2 <= max_distance {
            t_cylinder = t2;
        }
        
        // Check intersection with top and bottom caps
        let mut t_cap = f32::MAX;
        
        let t_bottom = (-half_height - ray_origin.y) / ray_dir.y;
        if t_bottom > 0.0 && t_bottom <= max_distance {
            let hit_point_xz = Vector3::new(
                ray_origin.x + ray_dir.x * t_bottom,
                0.0,
                ray_origin.z + ray_dir.z * t_bottom,
            );
            
            if hit_point_xz.length_squared() <= self.radius.powi(2) {
                t_cap = t_bottom;
            }
        }
        
        let t_top = (half_height - ray_origin.y) / ray_dir.y;
        if t_top > 0.0 && t_top < t_cap && t_top <= max_distance {
            let hit_point_xz = Vector3::new(
                ray_origin.x + ray_dir.x * t_top,
                0.0,
                ray_origin.z + ray_dir.z * t_top,
            );
            
            if hit_point_xz.length_squared() <= self.radius.powi(2) {
                t_cap = t_top;
            }
        }
        
        // Return the closest hit
        if t_cylinder != f32::MAX || t_cap != f32::MAX {
            return Some(t_cylinder.min(t_cap));
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