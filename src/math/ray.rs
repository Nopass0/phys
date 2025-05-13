use crate::math::Vector3;

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// Ray representation for intersection tests
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Ray {
    /// Origin of the ray
    pub origin: Vector3,
    
    /// Direction of the ray (not necessarily normalized)
    pub direction: Vector3,
}

impl Ray {
    /// Creates a new ray with the given origin and direction
    #[inline]
    pub fn new(origin: Vector3, direction: Vector3) -> Self {
        Self { origin, direction }
    }

    /// Creates a new ray with the given origin and direction, ensuring the direction is normalized
    #[inline]
    pub fn new_normalized(origin: Vector3, direction: Vector3) -> Self {
        Self { 
            origin, 
            direction: direction.normalize(),
        }
    }

    /// Returns the point at a given distance along the ray
    #[inline]
    pub fn point_at(&self, t: f32) -> Vector3 {
        self.origin + self.direction * t
    }

    /// Returns the normalized direction of the ray
    #[inline]
    pub fn normalized_direction(&self) -> Vector3 {
        self.direction.normalize()
    }

    /// Transforms the ray by a matrix (assuming the matrix is a transform matrix)
    pub fn transform(&self, matrix: &crate::math::Matrix4) -> Self {
        let new_origin = matrix.multiply_point(self.origin);
        let new_direction = matrix.multiply_direction(self.direction);
        
        Self {
            origin: new_origin,
            direction: new_direction,
        }
    }

    /// Returns the closest point on the ray to a given point
    pub fn closest_point(&self, point: Vector3) -> Vector3 {
        let dir = self.normalized_direction();
        let to_point = point - self.origin;
        let project_length = to_point.dot(&dir);
        
        if project_length < 0.0 {
            // The closest point is the ray origin
            return self.origin;
        }
        
        // The closest point is along the ray
        self.origin + dir * project_length
    }

    /// Returns the squared distance from the ray to a point
    #[inline]
    pub fn squared_distance_to_point(&self, point: Vector3) -> f32 {
        let closest = self.closest_point(point);
        (point - closest).length_squared()
    }

    /// Returns the distance from the ray to a point
    #[inline]
    pub fn distance_to_point(&self, point: Vector3) -> f32 {
        self.squared_distance_to_point(point).sqrt()
    }
}