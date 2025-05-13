use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform, Rotation};
use std::any::Any;
use std::fmt::Debug;

/// Base trait for collision shapes
pub trait Shape: Send + Sync + Debug + 'static {
    /// Returns the type name of the shape
    fn shape_type(&self) -> &'static str;
    
    /// Returns the volume of the shape
    fn get_volume(&self) -> f32;
    
    /// Returns the inertia tensor of the shape with the given mass
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3;
    
    /// Returns the axis-aligned bounding box of the shape in local space
    fn get_local_bounds(&self) -> Aabb;
    
    /// Returns the axis-aligned bounding box of the shape in world space
    fn get_world_bounds(&self, transform: &Transform) -> Aabb;
    
    /// Returns the support point of the shape in the given direction (used for GJK algorithm)
    fn get_support_point(&self, direction: Vector3) -> Vector3;
    
    /// Returns the support point of the shape in the given direction in world space
    fn get_world_support_point(&self, direction: Vector3, transform: &Transform) -> Vector3 {
        // Transform the direction to local space
        let local_dir = transform.rotation.conjugate().rotate_vector(direction);
        
        // Get the support point in local space
        let local_support = self.get_support_point(local_dir);
        
        // Transform the support point to world space
        transform.transform_point(local_support)
    }
    
    /// Returns whether a ray intersects with this shape in world space
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32>;
    
    /// Returns a dynamic reference to any for downcasting
    fn as_any(&self) -> &dyn Any;
    
    /// Returns a dynamic mutable reference to any for downcasting
    fn as_any_mut(&mut self) -> &mut dyn Any;
    
    /// Clone the shape to create a new box
    fn clone_shape(&self) -> Box<dyn Shape>;
}