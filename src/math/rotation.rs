use crate::math::{Vector3, Matrix3};
use std::fmt;
use std::ops::{Mul, MulAssign};

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// Quaternion for representing rotations in 3D space
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Quaternion {
    /// Real component
    pub w: f32,
    
    /// First imaginary component
    pub x: f32,
    
    /// Second imaginary component
    pub y: f32,
    
    /// Third imaginary component
    pub z: f32,
}

/// Rotation trait for rotation representations
pub trait Rotation {
    /// Rotate a vector by this rotation
    fn rotate_vector(&self, v: Vector3) -> Vector3;
    
    /// Get the angle in radians of this rotation
    fn angle(&self) -> f32;
    
    /// Get the axis of this rotation
    fn axis(&self) -> Vector3;
}

impl Quaternion {
    /// Creates a new quaternion
    #[inline]
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Returns the axis and angle of this quaternion rotation
    pub fn get_axis_angle(&self) -> (Vector3, f32) {
        // Get the axis
        let axis = self.axis();

        // Get the angle
        let angle = self.angle();

        (axis, angle)
    }

    /// Creates an identity quaternion (no rotation)
    #[inline]
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Creates a quaternion from an axis-angle representation
    pub fn from_axis_angle(axis: Vector3, angle: f32) -> Self {
        let half_angle = angle * 0.5;
        let s = half_angle.sin();
        let c = half_angle.cos();
        
        // Normalize the axis
        let axis = axis.normalize();
        
        Self {
            w: c,
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
    }

    /// Creates a quaternion from Euler angles (in radians)
    pub fn from_euler(x: f32, y: f32, z: f32) -> Self {
        // Compute half angles
        let half_x = x * 0.5;
        let half_y = y * 0.5;
        let half_z = z * 0.5;
        
        // Compute sines and cosines
        let sin_x = half_x.sin();
        let cos_x = half_x.cos();
        let sin_y = half_y.sin();
        let cos_y = half_y.cos();
        let sin_z = half_z.sin();
        let cos_z = half_z.cos();
        
        // Compute quaternion components
        Self {
            w: cos_x * cos_y * cos_z + sin_x * sin_y * sin_z,
            x: sin_x * cos_y * cos_z - cos_x * sin_y * sin_z,
            y: cos_x * sin_y * cos_z + sin_x * cos_y * sin_z,
            z: cos_x * cos_y * sin_z - sin_x * sin_y * cos_z,
        }
    }

    /// Creates a quaternion from a rotation matrix
    pub fn from_rotation_matrix(m: &Matrix3) -> Self {
        let trace = m.data[0][0] + m.data[1][1] + m.data[2][2];
        
        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            Self {
                w: 0.25 / s,
                x: (m.data[2][1] - m.data[1][2]) * s,
                y: (m.data[0][2] - m.data[2][0]) * s,
                z: (m.data[1][0] - m.data[0][1]) * s,
            }
        } else if m.data[0][0] > m.data[1][1] && m.data[0][0] > m.data[2][2] {
            let s = 2.0 * (1.0 + m.data[0][0] - m.data[1][1] - m.data[2][2]).sqrt();
            Self {
                w: (m.data[2][1] - m.data[1][2]) / s,
                x: 0.25 * s,
                y: (m.data[0][1] + m.data[1][0]) / s,
                z: (m.data[0][2] + m.data[2][0]) / s,
            }
        } else if m.data[1][1] > m.data[2][2] {
            let s = 2.0 * (1.0 + m.data[1][1] - m.data[0][0] - m.data[2][2]).sqrt();
            Self {
                w: (m.data[0][2] - m.data[2][0]) / s,
                x: (m.data[0][1] + m.data[1][0]) / s,
                y: 0.25 * s,
                z: (m.data[1][2] + m.data[2][1]) / s,
            }
        } else {
            let s = 2.0 * (1.0 + m.data[2][2] - m.data[0][0] - m.data[1][1]).sqrt();
            Self {
                w: (m.data[1][0] - m.data[0][1]) / s,
                x: (m.data[0][2] + m.data[2][0]) / s,
                y: (m.data[1][2] + m.data[2][1]) / s,
                z: 0.25 * s,
            }
        }
    }

    /// Converts the quaternion to a rotation matrix
    pub fn to_rotation_matrix(&self) -> Matrix3 {
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;
        
        let xx = x * x;
        let xy = x * y;
        let xz = x * z;
        let xw = x * w;
        
        let yy = y * y;
        let yz = y * z;
        let yw = y * w;
        
        let zz = z * z;
        let zw = z * w;
        
        Matrix3 {
            data: [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - zw), 2.0 * (xz + yw)],
                [2.0 * (xy + zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - xw)],
                [2.0 * (xz - yw), 2.0 * (yz + xw), 1.0 - 2.0 * (xx + yy)],
            ],
        }
    }

    /// Returns the conjugate of this quaternion
    #[inline]
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Returns the squared length of this quaternion
    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Returns the length of this quaternion
    #[inline]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Normalizes this quaternion
    #[inline]
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len > crate::math::EPSILON {
            Self {
                w: self.w / len,
                x: self.x / len,
                y: self.y / len,
                z: self.z / len,
            }
        } else {
            Quaternion::identity()
        }
    }

    /// Normalizes this quaternion in-place
    #[inline]
    pub fn normalize_mut(&mut self) {
        let len = self.length();
        if len > crate::math::EPSILON {
            self.w /= len;
            self.x /= len;
            self.y /= len;
            self.z /= len;
        } else {
            *self = Quaternion::identity();
        }
    }

    /// Returns the inverse of this quaternion
    #[inline]
    pub fn inverse(&self) -> Self {
        let len_sq = self.length_squared();
        if len_sq > crate::math::EPSILON {
            let inv_len_sq = 1.0 / len_sq;
            Self {
                w: self.w * inv_len_sq,
                x: -self.x * inv_len_sq,
                y: -self.y * inv_len_sq,
                z: -self.z * inv_len_sq,
            }
        } else {
            Quaternion::identity()
        }
    }

    /// Computes the dot product of two quaternions
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Spherical linear interpolation between two quaternions
    pub fn slerp(&self, other: &Self, t: f32) -> Self {
        let mut cos_half_theta = self.dot(other);
        
        // If the dot product is negative, slerp won't take the shorter path.
        // Fix by reversing one quaternion.
        let mut other_adj = *other;
        if cos_half_theta < 0.0 {
            other_adj.w = -other.w;
            other_adj.x = -other.x;
            other_adj.y = -other.y;
            other_adj.z = -other.z;
            cos_half_theta = -cos_half_theta;
        }
        
        // Handle edge cases
        if cos_half_theta > 0.999 {
            // Quaternions are very close - linear interpolation
            return Self {
                w: self.w + t * (other_adj.w - self.w),
                x: self.x + t * (other_adj.x - self.x),
                y: self.y + t * (other_adj.y - self.y),
                z: self.z + t * (other_adj.z - self.z),
            }.normalize();
        }
        
        // Calculate coefficients
        let half_theta = cos_half_theta.acos();
        let sin_half_theta = (1.0 - cos_half_theta * cos_half_theta).sqrt();
        
        // If sin of half theta is close to zero, use linear interpolation
        if sin_half_theta.abs() < 0.001 {
            return Self {
                w: self.w * 0.5 + other_adj.w * 0.5,
                x: self.x * 0.5 + other_adj.x * 0.5,
                y: self.y * 0.5 + other_adj.y * 0.5,
                z: self.z * 0.5 + other_adj.z * 0.5,
            }.normalize();
        }
        
        // Calculate final values
        let ratio_a = ((1.0 - t) * half_theta).sin() / sin_half_theta;
        let ratio_b = (t * half_theta).sin() / sin_half_theta;
        
        Self {
            w: self.w * ratio_a + other_adj.w * ratio_b,
            x: self.x * ratio_a + other_adj.x * ratio_b,
            y: self.y * ratio_a + other_adj.y * ratio_b,
            z: self.z * ratio_a + other_adj.z * ratio_b,
        }
    }

    /// Convert to nalgebra Quaternion
    #[inline]
    pub fn to_nalgebra(&self) -> nalgebra::Quaternion<f32> {
        nalgebra::Quaternion::new(self.w, self.x, self.y, self.z)
    }

    /// Convert from nalgebra Quaternion
    #[inline]
    pub fn from_nalgebra(q: &nalgebra::Quaternion<f32>) -> Self {
        Self {
            w: q.scalar(),
            x: q.vector()[0],
            y: q.vector()[1],
            z: q.vector()[2],
        }
    }
}

impl Rotation for Quaternion {
    /// Rotates a vector by this quaternion
    fn rotate_vector(&self, v: Vector3) -> Vector3 {
        // q * v * q^-1
        let vec_quat = Quaternion::new(0.0, v.x, v.y, v.z);
        let result = *self * vec_quat * self.conjugate();
        
        Vector3::new(result.x, result.y, result.z)
    }

    /// Returns the angle in radians of this rotation
    fn angle(&self) -> f32 {
        2.0 * self.w.acos()
    }

    /// Returns the normalized axis of this rotation
    fn axis(&self) -> Vector3 {
        // Extract the vector part
        let mut v = Vector3::new(self.x, self.y, self.z);
        
        // Normalize if possible
        let len = v.length();
        if len > crate::math::EPSILON {
            v = v / len;
        }
        
        v
    }
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {}, {}, {})", self.w, self.x, self.y, self.z)
    }
}

// Quaternion multiplication
impl Mul for Quaternion {
    type Output = Self;
    
    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl MulAssign for Quaternion {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}