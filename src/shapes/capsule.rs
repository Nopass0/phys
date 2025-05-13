use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;
use std::f32::consts::PI;

/// A capsule collision shape (cylinder with hemisphere caps at both ends)
#[derive(Debug, Clone)]
pub struct Capsule {
    /// The radius of the capsule
    radius: f32,
    
    /// The height of the capsule (cylinder part only, excluding the hemisphere caps)
    height: f32,
}

impl Capsule {
    /// Creates a new capsule with the given radius and height
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius: radius.max(0.0),
            height: height.max(0.0),
        }
    }
    
    /// Returns the radius of the capsule
    pub fn get_radius(&self) -> f32 {
        self.radius
    }
    
    /// Sets the radius of the capsule
    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius.max(0.0);
    }
    
    /// Returns the height of the capsule (cylinder part only)
    pub fn get_height(&self) -> f32 {
        self.height
    }
    
    /// Sets the height of the capsule (cylinder part only)
    pub fn set_height(&mut self, height: f32) {
        self.height = height.max(0.0);
    }
    
    /// Returns the total height of the capsule (including hemisphere caps)
    pub fn get_total_height(&self) -> f32 {
        self.height + 2.0 * self.radius
    }
    
    /// Returns the half-height of the capsule's cylinder part
    pub fn get_half_height(&self) -> f32 {
        self.height * 0.5
    }
}

impl Shape for Capsule {
    fn shape_type(&self) -> &'static str {
        "Capsule"
    }
    
    fn get_volume(&self) -> f32 {
        // Volume of cylinder + volume of two hemispheres
        let cylinder_volume = PI * self.radius.powi(2) * self.height;
        let sphere_volume = (4.0 / 3.0) * PI * self.radius.powi(3);
        
        cylinder_volume + sphere_volume
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        // This is an approximation. For more accuracy, the capsule should be
        // treated as a compound shape of a cylinder and two hemispheres.
        let total_height = self.get_total_height();
        let r2 = self.radius.powi(2);
        
        // Approximate using formula for solid cylinder
        let ix = (1.0 / 12.0) * mass * (3.0 * r2 + total_height.powi(2));
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
            half_height + self.radius,
            self.radius,
        );
        
        Aabb::new(-extent, extent)
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        // Start with the local AABB
        let local_bounds = self.get_local_bounds();
        
        // Get the eight corners of the local AABB
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
        
        // Find the extreme point along the axis of the capsule
        let half_height = self.height * 0.5;
        let axis_dir = Vector3::new(0.0, 1.0, 0.0); // Capsule aligned with Y-axis
        
        let axis_proj = axis_dir.dot(&dir);
        let mut extreme_point = Vector3::zero();
        
        if axis_proj > 0.0 {
            extreme_point.y = half_height;
        } else if axis_proj < 0.0 {
            extreme_point.y = -half_height;
        }
        
        // Add the radius along the direction projected onto the plane perpendicular to the axis
        let perp_dir = dir - axis_dir * axis_proj;
        let perp_length = perp_dir.length();
        
        if perp_length > crate::math::EPSILON {
            let normalized_perp = perp_dir / perp_length;
            extreme_point += normalized_perp * self.radius;
        }
        
        extreme_point
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // The capsule is aligned with the Y-axis in local space
        let half_height = self.height * 0.5;
        
        // Check infinite cylinder intersection
        let ray_origin = local_ray.origin;
        let ray_dir = local_ray.direction;
        
        // Remove Y component for cylinder test
        let ray_origin_xz = Vector3::new(ray_origin.x, 0.0, ray_origin.z);
        let ray_dir_xz = Vector3::new(ray_dir.x, 0.0, ray_dir.z);
        
        let a = ray_dir_xz.length_squared();
        if a < crate::math::EPSILON {
            // Ray is parallel to Y-axis, check if it's inside the cylinder
            if ray_origin_xz.length_squared() <= self.radius.powi(2) {
                // Check if it intersects either cap
                let t_bottom = (-half_height - ray_origin.y) / ray_dir.y;
                let t_top = (half_height - ray_origin.y) / ray_dir.y;
                
                let t = if t_bottom > 0.0 && t_bottom < t_top {
                    t_bottom
                } else {
                    t_top
                };
                
                if t > 0.0 && t <= max_distance {
                    return Some(t);
                }
            }
            
            // Check spherical caps
            let bottom_center = Vector3::new(0.0, -half_height, 0.0);
            let top_center = Vector3::new(0.0, half_height, 0.0);
            
            // Check bottom cap
            let oc_bottom = ray_origin - bottom_center;
            let b_bottom = 2.0 * oc_bottom.dot(&ray_dir);
            let c_bottom = oc_bottom.length_squared() - self.radius.powi(2);
            let discriminant_bottom = b_bottom * b_bottom - 4.0 * a * c_bottom;
            
            if discriminant_bottom > 0.0 {
                let t = (-b_bottom - discriminant_bottom.sqrt()) / (2.0 * a);
                if t > 0.0 && t <= max_distance {
                    return Some(t);
                }
            }
            
            // Check top cap
            let oc_top = ray_origin - top_center;
            let b_top = 2.0 * oc_top.dot(&ray_dir);
            let c_top = oc_top.length_squared() - self.radius.powi(2);
            let discriminant_top = b_top * b_top - 4.0 * a * c_top;
            
            if discriminant_top > 0.0 {
                let t = (-b_top - discriminant_top.sqrt()) / (2.0 * a);
                if t > 0.0 && t <= max_distance {
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
            return None; // No cylinder intersection
        }
        
        let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
        let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
        
        // Check if cylinder hit points are within the finite height
        let hit_points = [
            ray_origin + ray_dir * t1,
            ray_origin + ray_dir * t2,
        ];
        
        for (i, hit_point) in hit_points.iter().enumerate() {
            let t = if i == 0 { t1 } else { t2 };
            
            if t <= 0.0 || t > max_distance {
                continue;
            }
            
            if hit_point.y.abs() <= half_height {
                return Some(t);
            }
        }
        
        // Check spherical caps
        let bottom_center = Vector3::new(0.0, -half_height, 0.0);
        let top_center = Vector3::new(0.0, half_height, 0.0);
        
        // Check bottom cap
        let oc_bottom = ray_origin - bottom_center;
        let a_bottom = ray_dir.length_squared();
        let b_bottom = 2.0 * oc_bottom.dot(&ray_dir);
        let c_bottom = oc_bottom.length_squared() - self.radius.powi(2);
        let discriminant_bottom = b_bottom * b_bottom - 4.0 * a_bottom * c_bottom;
        
        if discriminant_bottom > 0.0 {
            let t = (-b_bottom - discriminant_bottom.sqrt()) / (2.0 * a_bottom);
            if t > 0.0 && t <= max_distance {
                let hit_point = ray_origin + ray_dir * t;
                if hit_point.y <= -half_height {
                    return Some(t);
                }
            }
        }
        
        // Check top cap
        let oc_top = ray_origin - top_center;
        let a_top = ray_dir.length_squared();
        let b_top = 2.0 * oc_top.dot(&ray_dir);
        let c_top = oc_top.length_squared() - self.radius.powi(2);
        let discriminant_top = b_top * b_top - 4.0 * a_top * c_top;
        
        if discriminant_top > 0.0 {
            let t = (-b_top - discriminant_top.sqrt()) / (2.0 * a_top);
            if t > 0.0 && t <= max_distance {
                let hit_point = ray_origin + ray_dir * t;
                if hit_point.y >= half_height {
                    return Some(t);
                }
            }
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