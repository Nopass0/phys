use nalgebra as na;
use crate::math::{Vector3, Vector4};
use std::fmt;

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// A 3x3 matrix representation for physics calculations
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Matrix3 {
    pub data: [[f32; 3]; 3],
}

/// A 4x4 matrix representation for physics calculations
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Matrix4 {
    pub data: [[f32; 4]; 4],
}

// === Matrix3 Implementation ===

impl Matrix3 {
    /// Creates a new 3x3 matrix from a 2D array
    #[inline]
    pub fn new(data: [[f32; 3]; 3]) -> Self {
        Self { data }
    }

    /// Creates a new 3x3 identity matrix
    #[inline]
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    /// Creates a new 3x3 zero matrix
    #[inline]
    pub fn zero() -> Self {
        Self {
            data: [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
        }
    }

    /// Creates a new 3x3 scaling matrix
    #[inline]
    pub fn from_scale(scale: Vector3) -> Self {
        Self {
            data: [
                [scale.x, 0.0, 0.0],
                [0.0, scale.y, 0.0],
                [0.0, 0.0, scale.z],
            ],
        }
    }

    /// Creates a new 3x3 rotation matrix from Euler angles (XYZ rotation)
    pub fn from_euler_angles(x_angle: f32, y_angle: f32, z_angle: f32) -> Self {
        let cos_x = x_angle.cos();
        let sin_x = x_angle.sin();
        let cos_y = y_angle.cos();
        let sin_y = y_angle.sin();
        let cos_z = z_angle.cos();
        let sin_z = z_angle.sin();

        Self {
            data: [
                [
                    cos_y * cos_z,
                    cos_y * sin_z,
                    -sin_y,
                ],
                [
                    sin_x * sin_y * cos_z - cos_x * sin_z,
                    sin_x * sin_y * sin_z + cos_x * cos_z,
                    sin_x * cos_y,
                ],
                [
                    cos_x * sin_y * cos_z + sin_x * sin_z,
                    cos_x * sin_y * sin_z - sin_x * cos_z,
                    cos_x * cos_y,
                ],
            ],
        }
    }

    /// Returns the determinant of the matrix
    pub fn determinant(&self) -> f32 {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.data;
        
        a * (e * i - f * h) -
        b * (d * i - f * g) +
        c * (d * h - e * g)
    }

    /// Returns the inverse of the matrix, or None if it is not invertible
    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        
        if det.abs() < crate::math::EPSILON {
            return None;
        }
        
        let [[a, b, c], [d, e, f], [g, h, i]] = self.data;
        let inv_det = 1.0 / det;
        
        Some(Self {
            data: [
                [
                    (e * i - f * h) * inv_det,
                    (c * h - b * i) * inv_det,
                    (b * f - c * e) * inv_det,
                ],
                [
                    (f * g - d * i) * inv_det,
                    (a * i - c * g) * inv_det,
                    (c * d - a * f) * inv_det,
                ],
                [
                    (d * h - e * g) * inv_det,
                    (g * b - a * h) * inv_det,
                    (a * e - b * d) * inv_det,
                ],
            ],
        })
    }

    /// Returns the transpose of the matrix
    #[inline]
    pub fn transpose(&self) -> Self {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.data;
        
        Self {
            data: [
                [a, d, g],
                [b, e, h],
                [c, f, i],
            ],
        }
    }

    /// Multiplies the matrix by a vector
    #[inline]
    pub fn multiply_vector(&self, v: Vector3) -> Vector3 {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.data;
        
        Vector3::new(
            a * v.x + b * v.y + c * v.z,
            d * v.x + e * v.y + f * v.z,
            g * v.x + h * v.y + i * v.z,
        )
    }
    
    /// Multiplies the matrix by another matrix
    pub fn multiply_matrix(&self, other: &Self) -> Self {
        let mut result = Self::zero();
        
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += self.data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        
        result
    }

    /// Convert to nalgebra Matrix3
    #[inline]
    pub fn to_nalgebra(&self) -> na::Matrix3<f32> {
        let [[a, b, c], [d, e, f], [g, h, i]] = self.data;
        
        na::Matrix3::new(
            a, b, c,
            d, e, f,
            g, h, i,
        )
    }

    /// Convert from nalgebra Matrix3
    #[inline]
    pub fn from_nalgebra(m: &na::Matrix3<f32>) -> Self {
        Self {
            data: [
                [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
                [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
                [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
            ],
        }
    }
}

impl fmt::Display for Matrix3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "[ {}, {}, {} ]", self.data[0][0], self.data[0][1], self.data[0][2])?;
        writeln!(f, "[ {}, {}, {} ]", self.data[1][0], self.data[1][1], self.data[1][2])?;
        write!(f, "[ {}, {}, {} ]", self.data[2][0], self.data[2][1], self.data[2][2])
    }
}

// === Matrix4 Implementation ===

impl Matrix4 {
    /// Creates a new 4x4 matrix from a 2D array
    #[inline]
    pub fn new(data: [[f32; 4]; 4]) -> Self {
        Self { data }
    }

    /// Creates a new 4x4 identity matrix
    #[inline]
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Creates a new 4x4 zero matrix
    #[inline]
    pub fn zero() -> Self {
        Self {
            data: [
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
            ],
        }
    }

    /// Creates a new 4x4 translation matrix
    #[inline]
    pub fn from_translation(translation: Vector3) -> Self {
        let mut result = Self::identity();
        result.data[0][3] = translation.x;
        result.data[1][3] = translation.y;
        result.data[2][3] = translation.z;
        result
    }

    /// Creates a new 4x4 scaling matrix
    #[inline]
    pub fn from_scale(scale: Vector3) -> Self {
        Self {
            data: [
                [scale.x, 0.0, 0.0, 0.0],
                [0.0, scale.y, 0.0, 0.0],
                [0.0, 0.0, scale.z, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Creates a 4x4 matrix from a 3x3 rotation matrix and a translation
    #[inline]
    pub fn from_rotation_translation(rotation: Matrix3, translation: Vector3) -> Self {
        Self {
            data: [
                [rotation.data[0][0], rotation.data[0][1], rotation.data[0][2], translation.x],
                [rotation.data[1][0], rotation.data[1][1], rotation.data[1][2], translation.y],
                [rotation.data[2][0], rotation.data[2][1], rotation.data[2][2], translation.z],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Returns the determinant of the matrix
    pub fn determinant(&self) -> f32 {
        // For a 4x4 matrix, the determinant calculation is quite lengthy
        // Here we'll use a more concise approach using cofactor expansion
        let m = &self.data;
        
        let s0 = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        let s1 = m[0][0] * m[1][2] - m[0][2] * m[1][0];
        let s2 = m[0][0] * m[1][3] - m[0][3] * m[1][0];
        let s3 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let s4 = m[0][1] * m[1][3] - m[0][3] * m[1][1];
        let s5 = m[0][2] * m[1][3] - m[0][3] * m[1][2];

        let c0 = m[2][0] * m[3][1] - m[2][1] * m[3][0];
        let c1 = m[2][0] * m[3][2] - m[2][2] * m[3][0];
        let c2 = m[2][0] * m[3][3] - m[2][3] * m[3][0];
        let c3 = m[2][1] * m[3][2] - m[2][2] * m[3][1];
        let c4 = m[2][1] * m[3][3] - m[2][3] * m[3][1];
        let c5 = m[2][2] * m[3][3] - m[2][3] * m[3][2];

        s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0
    }

    /// Returns the inverse of the matrix, or None if it is not invertible
    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        
        if det.abs() < crate::math::EPSILON {
            return None;
        }
        
        // For a 4x4 matrix, the inverse calculation is quite lengthy
        // Using the adjugate formula for the inverse
        let m = &self.data;
        let mut adj = [[0.0; 4]; 4];
        
        // Compute cofactors
        adj[0][0] = m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - 
                    m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) + 
                    m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]);
                    
        adj[0][1] = -(m[0][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - 
                     m[0][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) + 
                     m[0][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]));
                     
        adj[0][2] = m[0][1] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) - 
                    m[0][2] * (m[1][1] * m[3][3] - m[1][3] * m[3][1]) + 
                    m[0][3] * (m[1][1] * m[3][2] - m[1][2] * m[3][1]);
                    
        adj[0][3] = -(m[0][1] * (m[1][2] * m[2][3] - m[1][3] * m[2][2]) - 
                     m[0][2] * (m[1][1] * m[2][3] - m[1][3] * m[2][1]) + 
                     m[0][3] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]));
                     
        adj[1][0] = -(m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - 
                     m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + 
                     m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]));
                     
        adj[1][1] = m[0][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) - 
                    m[0][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + 
                    m[0][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]);
                    
        adj[1][2] = -(m[0][0] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) - 
                     m[0][2] * (m[1][0] * m[3][3] - m[1][3] * m[3][0]) + 
                     m[0][3] * (m[1][0] * m[3][2] - m[1][2] * m[3][0]));
                     
        adj[1][3] = m[0][0] * (m[1][2] * m[2][3] - m[1][3] * m[2][2]) - 
                    m[0][2] * (m[1][0] * m[2][3] - m[1][3] * m[2][0]) + 
                    m[0][3] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]);
                    
        adj[2][0] = m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) - 
                    m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + 
                    m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);
                    
        adj[2][1] = -(m[0][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) - 
                     m[0][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) + 
                     m[0][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
                     
        adj[2][2] = m[0][0] * (m[1][1] * m[3][3] - m[1][3] * m[3][1]) - 
                    m[0][1] * (m[1][0] * m[3][3] - m[1][3] * m[3][0]) + 
                    m[0][3] * (m[1][0] * m[3][1] - m[1][1] * m[3][0]);
                    
        adj[2][3] = -(m[0][0] * (m[1][1] * m[2][3] - m[1][3] * m[2][1]) - 
                     m[0][1] * (m[1][0] * m[2][3] - m[1][3] * m[2][0]) + 
                     m[0][3] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));
                     
        adj[3][0] = -(m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) - 
                     m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) + 
                     m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
                     
        adj[3][1] = m[0][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) - 
                    m[0][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) + 
                    m[0][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);
                    
        adj[3][2] = -(m[0][0] * (m[1][1] * m[3][2] - m[1][2] * m[3][1]) - 
                     m[0][1] * (m[1][0] * m[3][2] - m[1][2] * m[3][0]) + 
                     m[0][2] * (m[1][0] * m[3][1] - m[1][1] * m[3][0]));
                     
        adj[3][3] = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - 
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + 
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        
        // Divide by determinant
        let inv_det = 1.0 / det;
        let mut result = Self::zero();
        
        for i in 0..4 {
            for j in 0..4 {
                result.data[i][j] = adj[i][j] * inv_det;
            }
        }
        
        Some(result)
    }

    /// Returns the transpose of the matrix
    #[inline]
    pub fn transpose(&self) -> Self {
        let mut result = Self::zero();
        
        for i in 0..4 {
            for j in 0..4 {
                result.data[i][j] = self.data[j][i];
            }
        }
        
        result
    }

    /// Multiplies the matrix by a vector
    #[inline]
    pub fn multiply_vector(&self, v: Vector4) -> Vector4 {
        let mut result = Vector4::zero();
        
        for i in 0..4 {
            result.x += self.data[0][i] * match i {
                0 => v.x,
                1 => v.y,
                2 => v.z,
                3 => v.w,
                _ => unreachable!(),
            };
            
            result.y += self.data[1][i] * match i {
                0 => v.x,
                1 => v.y,
                2 => v.z,
                3 => v.w,
                _ => unreachable!(),
            };
            
            result.z += self.data[2][i] * match i {
                0 => v.x,
                1 => v.y,
                2 => v.z,
                3 => v.w,
                _ => unreachable!(),
            };
            
            result.w += self.data[3][i] * match i {
                0 => v.x,
                1 => v.y,
                2 => v.z,
                3 => v.w,
                _ => unreachable!(),
            };
        }
        
        result
    }
    
    /// Multiplies the matrix by a 3D vector (as if w=1)
    #[inline]
    pub fn multiply_point(&self, v: Vector3) -> Vector3 {
        let result = self.multiply_vector(Vector4::new(v.x, v.y, v.z, 1.0));
        
        if result.w.abs() > crate::math::EPSILON {
            Vector3::new(result.x / result.w, result.y / result.w, result.z / result.w)
        } else {
            Vector3::new(result.x, result.y, result.z)
        }
    }
    
    /// Multiplies the matrix by a 3D direction vector (as if w=0)
    #[inline]
    pub fn multiply_direction(&self, v: Vector3) -> Vector3 {
        let result = self.multiply_vector(Vector4::new(v.x, v.y, v.z, 0.0));
        Vector3::new(result.x, result.y, result.z)
    }
    
    /// Multiplies the matrix by another matrix
    pub fn multiply_matrix(&self, other: &Self) -> Self {
        let mut result = Self::zero();
        
        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += self.data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        
        result
    }

    /// Extract the upper-left 3x3 matrix (rotation part)
    #[inline]
    pub fn to_matrix3(&self) -> Matrix3 {
        Matrix3 {
            data: [
                [self.data[0][0], self.data[0][1], self.data[0][2]],
                [self.data[1][0], self.data[1][1], self.data[1][2]],
                [self.data[2][0], self.data[2][1], self.data[2][2]],
            ],
        }
    }

    /// Extract the translation part of the matrix
    #[inline]
    pub fn get_translation(&self) -> Vector3 {
        Vector3::new(self.data[0][3], self.data[1][3], self.data[2][3])
    }

    /// Convert to nalgebra Matrix4
    #[inline]
    pub fn to_nalgebra(&self) -> na::Matrix4<f32> {
        let m = &self.data;
        
        na::Matrix4::new(
            m[0][0], m[0][1], m[0][2], m[0][3],
            m[1][0], m[1][1], m[1][2], m[1][3],
            m[2][0], m[2][1], m[2][2], m[2][3],
            m[3][0], m[3][1], m[3][2], m[3][3],
        )
    }

    /// Convert from nalgebra Matrix4
    #[inline]
    pub fn from_nalgebra(m: &na::Matrix4<f32>) -> Self {
        Self {
            data: [
                [m[(0, 0)], m[(0, 1)], m[(0, 2)], m[(0, 3)]],
                [m[(1, 0)], m[(1, 1)], m[(1, 2)], m[(1, 3)]],
                [m[(2, 0)], m[(2, 1)], m[(2, 2)], m[(2, 3)]],
                [m[(3, 0)], m[(3, 1)], m[(3, 2)], m[(3, 3)]],
            ],
        }
    }
}

impl fmt::Display for Matrix4 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "[ {}, {}, {}, {} ]", self.data[0][0], self.data[0][1], self.data[0][2], self.data[0][3])?;
        writeln!(f, "[ {}, {}, {}, {} ]", self.data[1][0], self.data[1][1], self.data[1][2], self.data[1][3])?;
        writeln!(f, "[ {}, {}, {}, {} ]", self.data[2][0], self.data[2][1], self.data[2][2], self.data[2][3])?;
        write!(f, "[ {}, {}, {}, {} ]", self.data[3][0], self.data[3][1], self.data[3][2], self.data[3][3])
    }
}