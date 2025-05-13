use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform, Rotation};
use std::any::Any;
use std::sync::Arc;
use std::sync::Mutex;

/// A compound collision shape composed of multiple shapes
#[derive(Debug)]
pub struct Compound {
    /// The shapes that make up the compound shape, with their local transforms
    shapes: Vec<(Arc<dyn Shape>, Transform)>,

    /// The local AABB of the compound shape (cached)
    /// Using Mutex for thread-safe caching
    local_bounds: Mutex<Option<Aabb>>,
}

impl Clone for Compound {
    fn clone(&self) -> Self {
        let bounds = if let Ok(bounds) = self.local_bounds.lock() {
            bounds.clone()
        } else {
            None
        };

        Self {
            shapes: self.shapes.clone(),
            local_bounds: Mutex::new(bounds),
        }
    }
}

impl Compound {
    /// Creates a new empty compound shape
    pub fn new() -> Self {
        Self {
            shapes: Vec::new(),
            local_bounds: Mutex::new(None),
        }
    }
    
    /// Adds a shape to the compound shape
    pub fn add_shape(&mut self, shape: Arc<dyn Shape>, transform: Transform) {
        self.shapes.push((shape, transform));
        // Invalidate cached bounds
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Returns the shapes that make up the compound shape
    pub fn get_shapes(&self) -> &[(Arc<dyn Shape>, Transform)] {
        &self.shapes
    }
    
    /// Clears all shapes from the compound shape
    pub fn clear(&mut self) {
        self.shapes.clear();
        // Invalidate cached bounds
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Removes a shape at the given index
    pub fn remove_shape(&mut self, index: usize) -> Option<(Arc<dyn Shape>, Transform)> {
        if index < self.shapes.len() {
            let shape = self.shapes.remove(index);
            // Invalidate cached bounds
            if let Ok(mut bounds) = self.local_bounds.lock() {
                *bounds = None;
            }
            Some(shape)
        } else {
            None
        }
    }
    
    /// Updates the transform of a shape at the given index
    pub fn update_transform(&mut self, index: usize, transform: Transform) -> bool {
        if index < self.shapes.len() {
            self.shapes[index].1 = transform;
            // Invalidate cached bounds
            if let Ok(mut bounds) = self.local_bounds.lock() {
                *bounds = None;
            }
            true
        } else {
            false
        }
    }
    
    /// Returns the number of shapes in the compound shape
    pub fn shape_count(&self) -> usize {
        self.shapes.len()
    }
    
    /// Recomputes and caches the local AABB of the compound shape
    fn compute_local_bounds(&self) {
        if self.shapes.is_empty() {
            if let Ok(mut bounds) = self.local_bounds.lock() {
                *bounds = Some(Aabb::new(Vector3::zero(), Vector3::zero()));
            }
            return;
        }

        let mut bounds = self.shapes[0].0.get_world_bounds(&self.shapes[0].1);

        for (shape, transform) in self.shapes.iter().skip(1) {
            let shape_bounds = shape.get_world_bounds(transform);
            bounds = bounds.union(&shape_bounds);
        }

        if let Ok(mut local_bounds) = self.local_bounds.lock() {
            *local_bounds = Some(bounds);
        }
    }
}

impl Shape for Compound {
    fn shape_type(&self) -> &'static str {
        "Compound"
    }
    
    fn get_volume(&self) -> f32 {
        // Sum the volumes of all shapes (not accounting for overlapping)
        self.shapes.iter()
            .map(|(shape, _)| shape.get_volume())
            .sum()
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        if self.shapes.is_empty() {
            return Matrix3::zero();
        }
        
        // This is a simplification. A proper implementation would:
        // 1. Compute center of mass for all shapes
        // 2. Apply parallel axis theorem to each shape's inertia tensor
        // 3. Sum these tensors
        
        // For now, we'll use an approximation by averaging the inertia tensors
        let mut tensor = Matrix3::zero();
        let shape_count = self.shapes.len() as f32;
        let shape_mass = mass / shape_count;
        
        for (shape, _) in &self.shapes {
            tensor = Matrix3::new([
                [tensor.data[0][0] + shape.get_inertia_tensor(shape_mass).data[0][0], 0.0, 0.0],
                [0.0, tensor.data[1][1] + shape.get_inertia_tensor(shape_mass).data[1][1], 0.0],
                [0.0, 0.0, tensor.data[2][2] + shape.get_inertia_tensor(shape_mass).data[2][2]],
            ]);
        }
        
        tensor
    }
    
    fn get_local_bounds(&self) -> Aabb {
        // Try to get cached bounds first
        if let Ok(bounds) = self.local_bounds.lock() {
            if let Some(aabb) = bounds.as_ref() {
                return aabb.clone();
            }
        }

        // Compute bounds if not cached
        self.compute_local_bounds();

        // Return the computed bounds
        if let Ok(bounds) = self.local_bounds.lock() {
            bounds.clone().unwrap_or_else(|| Aabb::new(Vector3::zero(), Vector3::zero()))
        } else {
            // Fall back to default in case of lock error
            Aabb::new(Vector3::zero(), Vector3::zero())
        }
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        if self.shapes.is_empty() {
            return Aabb::new(
                transform.position,
                transform.position,
            );
        }
        
        // Transform each shape's bounds to world space
        let mut bounds = None;
        
        for (shape, local_transform) in &self.shapes {
            // Combine the local transform with the world transform
            let combined_transform = transform.combine(local_transform);
            
            // Get the world bounds for this shape
            let shape_bounds = shape.get_world_bounds(&combined_transform);
            
            // Union with the current bounds
            bounds = match bounds {
                None => Some(shape_bounds),
                Some(current_bounds) => Some(current_bounds.union(&shape_bounds)),
            };
        }
        
        bounds.unwrap_or_else(|| Aabb::new(transform.position, transform.position))
    }
    
    fn get_support_point(&self, direction: Vector3) -> Vector3 {
        if self.shapes.is_empty() {
            return Vector3::zero();
        }
        
        // Find the furthest point in the given direction
        let mut furthest_distance = -f32::MAX;
        let mut furthest_point = Vector3::zero();
        
        for (shape, transform) in &self.shapes {
            // Transform the direction to shape's local space
            let local_dir = transform.rotation.conjugate().rotate_vector(direction);
            
            // Get the support point in local space
            let local_support = shape.get_support_point(local_dir);
            
            // Transform to compound shape's local space
            let support = transform.transform_point(local_support);
            
            // Check if this point is further along the direction
            let distance = support.dot(&direction);
            if distance > furthest_distance {
                furthest_distance = distance;
                furthest_point = support;
            }
        }
        
        furthest_point
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Early out if there are no shapes
        if self.shapes.is_empty() {
            return None;
        }
        
        // Check intersection with each shape
        let mut closest_t = f32::MAX;
        
        for (shape, local_transform) in &self.shapes {
            // Combine the local transform with the world transform
            let combined_transform = transform.combine(local_transform);
            
            // Check ray intersection with this shape
            if let Some(t) = shape.intersects_ray(ray, &combined_transform, max_distance) {
                if t < closest_t {
                    closest_t = t;
                }
            }
        }
        
        if closest_t != f32::MAX {
            Some(closest_t)
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