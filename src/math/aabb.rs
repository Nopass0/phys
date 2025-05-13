use crate::math::{Vector3, Ray};

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// Axis-Aligned Bounding Box (AABB) for efficient collision detection
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Aabb {
    /// Minimum corner of the AABB
    pub min: Vector3,
    
    /// Maximum corner of the AABB
    pub max: Vector3,
}

impl Aabb {
    /// Creates a new AABB from minimum and maximum points
    #[inline]
    pub fn new(min: Vector3, max: Vector3) -> Self {
        Self { min, max }
    }

    /// Creates an AABB centered at a position with the given half extents
    #[inline]
    pub fn from_center_half_extents(center: Vector3, half_extents: Vector3) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Creates an AABB from a set of points
    pub fn from_points(points: &[Vector3]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }
        
        let mut min = points[0];
        let mut max = points[0];
        
        for point in points.iter().skip(1) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }
        
        Some(Self { min, max })
    }

    /// Returns the center of the AABB
    #[inline]
    pub fn center(&self) -> Vector3 {
        (self.min + self.max) * 0.5
    }

    /// Returns the extents of the AABB in each dimension
    #[inline]
    pub fn extents(&self) -> Vector3 {
        self.max - self.min
    }

    /// Returns half the extents of the AABB in each dimension
    #[inline]
    pub fn half_extents(&self) -> Vector3 {
        self.extents() * 0.5
    }

    /// Returns the volume of the AABB
    #[inline]
    pub fn volume(&self) -> f32 {
        let extents = self.extents();
        extents.x * extents.y * extents.z
    }

    /// Returns the surface area of the AABB
    #[inline]
    pub fn surface_area(&self) -> f32 {
        let extents = self.extents();
        2.0 * (extents.x * extents.y + extents.x * extents.z + extents.y * extents.z)
    }

    /// Checks if this AABB contains a point
    #[inline]
    pub fn contains_point(&self, point: Vector3) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }

    /// Checks if this AABB fully contains another AABB
    #[inline]
    pub fn contains_aabb(&self, other: &Self) -> bool {
        self.min.x <= other.min.x && self.max.x >= other.max.x &&
        self.min.y <= other.min.y && self.max.y >= other.max.y &&
        self.min.z <= other.min.z && self.max.z >= other.max.z
    }

    /// Checks if this AABB intersects with another AABB
    #[inline]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

    /// Returns the intersection of this AABB with another, if they intersect
    #[inline]
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        if !self.intersects(other) {
            return None;
        }
        
        let min = Vector3::new(
            self.min.x.max(other.min.x),
            self.min.y.max(other.min.y),
            self.min.z.max(other.min.z),
        );
        
        let max = Vector3::new(
            self.max.x.min(other.max.x),
            self.max.y.min(other.max.y),
            self.max.z.min(other.max.z),
        );
        
        Some(Self { min, max })
    }

    /// Returns the union of this AABB with another
    #[inline]
    pub fn union(&self, other: &Self) -> Self {
        let min = Vector3::new(
            self.min.x.min(other.min.x),
            self.min.y.min(other.min.y),
            self.min.z.min(other.min.z),
        );
        
        let max = Vector3::new(
            self.max.x.max(other.max.x),
            self.max.y.max(other.max.y),
            self.max.z.max(other.max.z),
        );
        
        Self { min, max }
    }

    /// Expands this AABB to include a point
    #[inline]
    pub fn expand_to_include_point(&mut self, point: Vector3) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Expands this AABB to include another AABB
    #[inline]
    pub fn expand_to_include_aabb(&mut self, other: &Self) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.min.z = self.min.z.min(other.min.z);
        
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
        self.max.z = self.max.z.max(other.max.z);
    }

    /// Expands this AABB by a margin in all directions
    #[inline]
    pub fn expand(&self, margin: f32) -> Self {
        let margin_vec = Vector3::new(margin, margin, margin);
        Self {
            min: self.min - margin_vec,
            max: self.max + margin_vec,
        }
    }

    /// Returns the closest point on the AABB to a given point
    pub fn closest_point(&self, point: Vector3) -> Vector3 {
        let mut result = point;
        
        // Clamp the point to the AABB
        result.x = result.x.max(self.min.x).min(self.max.x);
        result.y = result.y.max(self.min.y).min(self.max.y);
        result.z = result.z.max(self.min.z).min(self.max.z);
        
        result
    }

    /// Returns the squared distance from a point to the AABB
    #[inline]
    pub fn squared_distance_to_point(&self, point: Vector3) -> f32 {
        let closest = self.closest_point(point);
        (closest - point).length_squared()
    }

    /// Returns the distance from a point to the AABB
    #[inline]
    pub fn distance_to_point(&self, point: Vector3) -> f32 {
        self.squared_distance_to_point(point).sqrt()
    }

    /// Checks if a ray intersects this AABB
    pub fn intersects_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<(f32, f32)> {
        let inv_dir = Vector3::new(
            if ray.direction.x.abs() > crate::math::EPSILON { 1.0 / ray.direction.x } else { 0.0 },
            if ray.direction.y.abs() > crate::math::EPSILON { 1.0 / ray.direction.y } else { 0.0 },
            if ray.direction.z.abs() > crate::math::EPSILON { 1.0 / ray.direction.z } else { 0.0 },
        );
        
        let mut t1 = (self.min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (self.max.x - ray.origin.x) * inv_dir.x;
        
        let mut t_near = t1.min(t2);
        let mut t_far = t1.max(t2);
        
        t1 = (self.min.y - ray.origin.y) * inv_dir.y;
        t2 = (self.max.y - ray.origin.y) * inv_dir.y;
        
        t_near = t_near.max(t1.min(t2));
        t_far = t_far.min(t1.max(t2));
        
        t1 = (self.min.z - ray.origin.z) * inv_dir.z;
        t2 = (self.max.z - ray.origin.z) * inv_dir.z;
        
        t_near = t_near.max(t1.min(t2));
        t_far = t_far.min(t1.max(t2));
        
        if t_near > t_far || t_far < t_min || t_near > t_max {
            None
        } else {
            Some((t_near.max(t_min), t_far.min(t_max)))
        }
    }
}