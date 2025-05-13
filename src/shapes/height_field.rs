use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;

/// A height field collision shape (terrain)
#[derive(Debug, Clone)]
pub struct HeightField {
    /// The height values of the terrain
    heights: Vec<f32>,
    
    /// The width of the height field (number of columns)
    width: usize,
    
    /// The depth of the height field (number of rows)
    depth: usize,
    
    /// The minimum height value
    min_height: f32,
    
    /// The maximum height value
    max_height: f32,
    
    /// The scale of the height field in each dimension
    scale: Vector3,
}

impl HeightField {
    /// Creates a new height field with the given dimensions and height values
    pub fn new(heights: Vec<f32>, width: usize, depth: usize, scale: Vector3) -> Self {
        // Ensure the height values match the dimensions
        assert!(heights.len() == width * depth, "Height values don't match dimensions");
        
        // Find the min and max height values
        let min_height = heights.iter().fold(f32::MAX, |min, &height| min.min(height));
        let max_height = heights.iter().fold(f32::MIN, |max, &height| max.max(height));
        
        Self {
            heights,
            width,
            depth,
            min_height,
            max_height,
            scale,
        }
    }
    
    /// Returns the height values of the terrain
    pub fn get_heights(&self) -> &[f32] {
        &self.heights
    }
    
    /// Returns the width of the height field
    pub fn get_width(&self) -> usize {
        self.width
    }
    
    /// Returns the depth of the height field
    pub fn get_depth(&self) -> usize {
        self.depth
    }
    
    /// Returns the scale of the height field
    pub fn get_scale(&self) -> Vector3 {
        self.scale
    }
    
    /// Sets the scale of the height field
    pub fn set_scale(&mut self, scale: Vector3) {
        self.scale = scale;
    }
    
    /// Returns the height at the given coordinates
    pub fn get_height(&self, x: usize, z: usize) -> f32 {
        if x < self.width && z < self.depth {
            self.heights[z * self.width + x]
        } else {
            0.0
        }
    }
    
    /// Sets the height at the given coordinates
    pub fn set_height(&mut self, x: usize, z: usize, height: f32) {
        if x < self.width && z < self.depth {
            self.heights[z * self.width + x] = height;
            
            // Update min and max heights
            self.min_height = self.min_height.min(height);
            self.max_height = self.max_height.max(height);
        }
    }
    
    /// Returns the interpolated height at the given coordinates
    pub fn get_height_interpolated(&self, x: f32, z: f32) -> f32 {
        // Convert to grid coordinates
        let gx = x / self.scale.x;
        let gz = z / self.scale.z;
        
        // Get the grid cell
        let gx0 = gx.floor() as usize;
        let gz0 = gz.floor() as usize;
        let gx1 = gx0 + 1;
        let gz1 = gz0 + 1;
        
        // Clamp to grid boundaries
        let gx0 = gx0.min(self.width - 1);
        let gz0 = gz0.min(self.depth - 1);
        let gx1 = gx1.min(self.width - 1);
        let gz1 = gz1.min(self.depth - 1);
        
        // Get the heights at the grid corners
        let h00 = self.get_height(gx0, gz0);
        let h10 = self.get_height(gx1, gz0);
        let h01 = self.get_height(gx0, gz1);
        let h11 = self.get_height(gx1, gz1);
        
        // Calculate the interpolation fractions
        let fx = gx - gx0 as f32;
        let fz = gz - gz0 as f32;
        
        // Bilinear interpolation
        let h0 = h00 * (1.0 - fx) + h10 * fx;
        let h1 = h01 * (1.0 - fx) + h11 * fx;
        
        h0 * (1.0 - fz) + h1 * fz
    }
    
    /// Returns the normal at the given coordinates
    pub fn get_normal(&self, x: usize, z: usize) -> Vector3 {
        // Get the heights of the surrounding points
        let h = self.get_height(x, z);
        
        let h_left = if x > 0 {
            self.get_height(x - 1, z)
        } else {
            h
        };
        
        let h_right = if x < self.width - 1 {
            self.get_height(x + 1, z)
        } else {
            h
        };
        
        let h_up = if z > 0 {
            self.get_height(x, z - 1)
        } else {
            h
        };
        
        let h_down = if z < self.depth - 1 {
            self.get_height(x, z + 1)
        } else {
            h
        };
        
        // Calculate the normal using central differences
        let dx = (h_right - h_left) / (2.0 * self.scale.x);
        let dz = (h_down - h_up) / (2.0 * self.scale.z);
        
        Vector3::new(-dx, 1.0, -dz).normalize()
    }
}

impl Shape for HeightField {
    fn shape_type(&self) -> &'static str {
        "HeightField"
    }
    
    fn get_volume(&self) -> f32 {
        // Volume of a height field is the area times the average height
        let area = (self.width as f32) * self.scale.x * (self.depth as f32) * self.scale.z;
        let avg_height = self.heights.iter().sum::<f32>() / (self.width * self.depth) as f32;
        
        area * avg_height
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        // Approximate as a box with the same dimensions
        let total_width = self.width as f32 * self.scale.x;
        let total_depth = self.depth as f32 * self.scale.z;
        let height = (self.max_height - self.min_height) * self.scale.y;
        
        let xx = (total_depth * total_depth + height * height) / 12.0;
        let yy = (total_width * total_width + total_depth * total_depth) / 12.0;
        let zz = (total_width * total_width + height * height) / 12.0;
        
        Matrix3::new([
            [xx * mass, 0.0, 0.0],
            [0.0, yy * mass, 0.0],
            [0.0, 0.0, zz * mass],
        ])
    }
    
    fn get_local_bounds(&self) -> Aabb {
        let half_width = (self.width as f32 * self.scale.x) * 0.5;
        let half_depth = (self.depth as f32 * self.scale.z) * 0.5;
        
        let min = Vector3::new(
            -half_width,
            self.min_height * self.scale.y,
            -half_depth,
        );
        
        let max = Vector3::new(
            half_width,
            self.max_height * self.scale.y,
            half_depth,
        );
        
        Aabb::new(min, max)
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        let local_bounds = self.get_local_bounds();
        
        // Transform the eight corners of the local AABB
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
        
        // Find the furthest point in the given direction
        let mut furthest_distance = -f32::MAX;
        let mut furthest_point = Vector3::zero();
        
        let half_width = (self.width as f32 * self.scale.x) * 0.5;
        let half_depth = (self.depth as f32 * self.scale.z) * 0.5;
        
        // Check each vertex of the height field
        for z in 0..self.depth {
            for x in 0..self.width {
                let height = self.get_height(x, z);
                
                let point = Vector3::new(
                    (x as f32 / (self.width - 1) as f32) * self.width as f32 * self.scale.x - half_width,
                    height * self.scale.y,
                    (z as f32 / (self.depth - 1) as f32) * self.depth as f32 * self.scale.z - half_depth,
                );
                
                let distance = point.dot(&direction);
                if distance > furthest_distance {
                    furthest_distance = distance;
                    furthest_point = point;
                }
            }
        }
        
        furthest_point
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Check AABB intersection first
        let bounds = self.get_local_bounds();
        if let Some((t_min, t_max)) = bounds.intersects_ray(&local_ray, 0.0, max_distance) {
            // Ray intersects the AABB, now check the height field
            
            // Start at the entry point
            let mut t = t_min;
            let step_size = self.scale.x.min(self.scale.z) * 0.5;
            
            while t <= t_max && t <= max_distance {
                // Get the point along the ray
                let point = local_ray.point_at(t);
                
                // Convert to height field coordinates
                let half_width = (self.width as f32 * self.scale.x) * 0.5;
                let half_depth = (self.depth as f32 * self.scale.z) * 0.5;
                
                let x = (point.x + half_width) / self.scale.x;
                let z = (point.z + half_depth) / self.scale.z;
                
                // Check if point is within height field bounds
                if x >= 0.0 && x < self.width as f32 && z >= 0.0 && z < self.depth as f32 {
                    // Get the interpolated height at this point
                    let height = self.get_height_interpolated(x, z) * self.scale.y;
                    
                    // Check if the ray point is below the height field
                    if point.y <= height {
                        return Some(t);
                    }
                }
                
                // Step forward
                t += step_size;
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