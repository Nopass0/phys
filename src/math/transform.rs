use crate::math::{Vector3, Matrix4, Quaternion, Matrix3};
use crate::math::rotation::Rotation;

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// Represents a transformation in 3D space (position, rotation, and scale)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Transform {
    /// Position in 3D space
    pub position: Vector3,
    
    /// Rotation as a quaternion
    pub rotation: Quaternion,
    
    /// Scale in each axis
    pub scale: Vector3,
}

impl Transform {
    /// Creates a new transform with the given position, rotation, and scale
    #[inline]
    pub fn new(position: Vector3, rotation: Quaternion, scale: Vector3) -> Self {
        Self {
            position,
            rotation,
            scale,
        }
    }

    /// Creates a new identity transform (no translation, no rotation, unit scale)
    #[inline]
    pub fn identity() -> Self {
        Self {
            position: Vector3::zero(),
            rotation: Quaternion::identity(),
            scale: Vector3::one(),
        }
    }

    /// Creates a new transform from just a position
    #[inline]
    pub fn from_position(position: Vector3) -> Self {
        Self {
            position,
            rotation: Quaternion::identity(),
            scale: Vector3::one(),
        }
    }

    /// Creates a new transform from a position and rotation
    #[inline]
    pub fn from_position_rotation(position: Vector3, rotation: Quaternion) -> Self {
        Self {
            position,
            rotation,
            scale: Vector3::one(),
        }
    }

    /// Creates a new transform from a 4x4 transformation matrix
    pub fn from_matrix(matrix: &Matrix4) -> Self {
        // Extract position
        let position = matrix.get_translation();
        
        // Extract rotation (from the upper-left 3x3 submatrix)
        let rot_mat = matrix.to_matrix3();
        
        // Extract scale
        let scale_x = Vector3::new(rot_mat.data[0][0], rot_mat.data[1][0], rot_mat.data[2][0]).length();
        let scale_y = Vector3::new(rot_mat.data[0][1], rot_mat.data[1][1], rot_mat.data[2][1]).length();
        let scale_z = Vector3::new(rot_mat.data[0][2], rot_mat.data[1][2], rot_mat.data[2][2]).length();
        
        let scale = Vector3::new(scale_x, scale_y, scale_z);
        
        // Remove scale from rotation matrix
        let mut rot_mat_no_scale = rot_mat;
        if scale_x > crate::math::EPSILON {
            rot_mat_no_scale.data[0][0] /= scale_x;
            rot_mat_no_scale.data[1][0] /= scale_x;
            rot_mat_no_scale.data[2][0] /= scale_x;
        }
        if scale_y > crate::math::EPSILON {
            rot_mat_no_scale.data[0][1] /= scale_y;
            rot_mat_no_scale.data[1][1] /= scale_y;
            rot_mat_no_scale.data[2][1] /= scale_y;
        }
        if scale_z > crate::math::EPSILON {
            rot_mat_no_scale.data[0][2] /= scale_z;
            rot_mat_no_scale.data[1][2] /= scale_z;
            rot_mat_no_scale.data[2][2] /= scale_z;
        }
        
        // Convert rotation matrix to quaternion
        let rotation = Quaternion::from_rotation_matrix(&rot_mat_no_scale);
        
        Self {
            position,
            rotation,
            scale,
        }
    }

    /// Converts the transform to a 4x4 transformation matrix
    pub fn to_matrix(&self) -> Matrix4 {
        // Create rotation matrix from quaternion
        let rot_mat = self.rotation.to_rotation_matrix();
        
        // Apply scale to rotation matrix
        let mut scaled_rot_mat = Matrix3::zero();
        for i in 0..3 {
            for j in 0..3 {
                match j {
                    0 => scaled_rot_mat.data[i][j] = rot_mat.data[i][j] * self.scale.x,
                    1 => scaled_rot_mat.data[i][j] = rot_mat.data[i][j] * self.scale.y,
                    2 => scaled_rot_mat.data[i][j] = rot_mat.data[i][j] * self.scale.z,
                    _ => unreachable!(),
                }
            }
        }
        
        // Combine into a 4x4 matrix
        Matrix4::from_rotation_translation(scaled_rot_mat, self.position)
    }

    /// Transforms a point by this transform
    #[inline]
    pub fn transform_point(&self, point: Vector3) -> Vector3 {
        // Scale
        let scaled = Vector3::new(
            point.x * self.scale.x,
            point.y * self.scale.y,
            point.z * self.scale.z,
        );
        
        // Rotate
        let rotated = self.rotation.rotate_vector(scaled);
        
        // Translate
        rotated + self.position
    }

    /// Transforms a direction vector by this transform (ignoring translation)
    #[inline]
    pub fn transform_direction(&self, direction: Vector3) -> Vector3 {
        // Scale
        let scaled = Vector3::new(
            direction.x * self.scale.x,
            direction.y * self.scale.y,
            direction.z * self.scale.z,
        );
        
        // Rotate
        self.rotation.rotate_vector(scaled)
    }

    /// Transforms a vector as a normal (inverse-transpose for scale)
    pub fn transform_normal(&self, normal: Vector3) -> Vector3 {
        // For normal vectors, we need to use the inverse-transpose of the scale
        let inv_scale = Vector3::new(
            if self.scale.x.abs() > crate::math::EPSILON { 1.0 / self.scale.x } else { 0.0 },
            if self.scale.y.abs() > crate::math::EPSILON { 1.0 / self.scale.y } else { 0.0 },
            if self.scale.z.abs() > crate::math::EPSILON { 1.0 / self.scale.z } else { 0.0 },
        );
        
        let scaled = Vector3::new(
            normal.x * inv_scale.x,
            normal.y * inv_scale.y,
            normal.z * inv_scale.z,
        );
        
        // Rotate
        let result = self.rotation.rotate_vector(scaled);
        
        // Normalize the result
        result.normalize()
    }

    /// Inverts this transform
    pub fn inverse(&self) -> Self {
        // Inverse of scale
        let inv_scale = Vector3::new(
            if self.scale.x.abs() > crate::math::EPSILON { 1.0 / self.scale.x } else { 1.0 },
            if self.scale.y.abs() > crate::math::EPSILON { 1.0 / self.scale.y } else { 1.0 },
            if self.scale.z.abs() > crate::math::EPSILON { 1.0 / self.scale.z } else { 1.0 },
        );
        
        // Inverse of rotation
        let inv_rotation = self.rotation.conjugate();
        
        // Inverse of position requires applying inverse rotation and scale
        let inv_position = -(inv_rotation.rotate_vector(Vector3::new(
            self.position.x * inv_scale.x,
            self.position.y * inv_scale.y,
            self.position.z * inv_scale.z,
        )));
        
        Self {
            position: inv_position,
            rotation: inv_rotation,
            scale: inv_scale,
        }
    }

    /// Combines this transform with another, applying this one first
    pub fn combine(&self, other: &Self) -> Self {
        // First scale
        let combined_scale = Vector3::new(
            self.scale.x * other.scale.x,
            self.scale.y * other.scale.y,
            self.scale.z * other.scale.z,
        );
        
        // Then rotate
        let combined_rotation = other.rotation * self.rotation;
        
        // Then translate
        let rotated_scaled_translation = other.rotation.rotate_vector(Vector3::new(
            self.position.x * other.scale.x,
            self.position.y * other.scale.y,
            self.position.z * other.scale.z,
        ));
        
        let combined_position = other.position + rotated_scaled_translation;
        
        Self {
            position: combined_position,
            rotation: combined_rotation,
            scale: combined_scale,
        }
    }

    /// Interpolates between this transform and another
    pub fn interpolate(&self, other: &Self, t: f32) -> Self {
        let position = self.position.lerp(&other.position, t);
        let rotation = self.rotation.slerp(&other.rotation, t);
        let scale = self.scale.lerp(&other.scale, t);
        
        Self {
            position,
            rotation,
            scale,
        }
    }
}