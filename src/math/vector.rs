use nalgebra as na;
use std::ops::{Add, Sub, Mul, Div, Neg, AddAssign, SubAssign, MulAssign, DivAssign};
use std::fmt;

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// A 2D vector representation for physics calculations
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

/// A 3D vector representation for physics calculations
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// A 4D vector representation for physics calculations
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Vector4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

// === Vector2 Implementation ===

impl Vector2 {
    /// Creates a new 2D vector
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Creates a new 2D vector with all components set to zero
    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Creates a new 2D vector with all components set to one
    #[inline]
    pub fn one() -> Self {
        Self { x: 1.0, y: 1.0 }
    }

    /// Creates a unit vector pointing in the x direction
    #[inline]
    pub fn unit_x() -> Self {
        Self { x: 1.0, y: 0.0 }
    }

    /// Creates a unit vector pointing in the y direction
    #[inline]
    pub fn unit_y() -> Self {
        Self { x: 0.0, y: 1.0 }
    }

    /// Computes the dot product of two vectors
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Computes the cross product magnitude of two 2D vectors
    #[inline]
    pub fn cross(&self, other: &Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Returns the squared length of the vector
    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Returns the length of the vector
    #[inline]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Returns a normalized version of the vector
    #[inline]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        if length > crate::math::EPSILON {
            *self / length
        } else {
            *self
        }
    }

    /// Normalizes the vector in-place
    #[inline]
    pub fn normalize_mut(&mut self) {
        let length = self.length();
        if length > crate::math::EPSILON {
            self.x /= length;
            self.y /= length;
        }
    }

    /// Returns true if the vector is approximately zero
    #[inline]
    pub fn is_zero(&self) -> bool {
        crate::math::approx_zero(self.length_squared())
    }

    /// Convert to nalgebra Vector2
    #[inline]
    pub fn to_nalgebra(&self) -> na::Vector2<f32> {
        na::Vector2::new(self.x, self.y)
    }

    /// Convert from nalgebra Vector2
    #[inline]
    pub fn from_nalgebra(v: &na::Vector2<f32>) -> Self {
        Self::new(v.x, v.y)
    }

    /// Distance between two vectors
    #[inline]
    pub fn distance(&self, other: &Self) -> f32 {
        (*self - *other).length()
    }

    /// Squared distance between two vectors
    #[inline]
    pub fn distance_squared(&self, other: &Self) -> f32 {
        (*self - *other).length_squared()
    }

    /// Linear interpolation between two vectors
    #[inline]
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        *self + (*other - *self) * t
    }

    /// Returns the angle in radians of this vector (in the range [-PI, PI])
    #[inline]
    pub fn angle(&self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Returns a vector that is perpendicular to this vector
    #[inline]
    pub fn perpendicular(&self) -> Self {
        Self::new(-self.y, self.x)
    }
}

impl From<[f32; 2]> for Vector2 {
    #[inline]
    fn from(array: [f32; 2]) -> Self {
        Self::new(array[0], array[1])
    }
}

impl From<Vector2> for [f32; 2] {
    #[inline]
    fn from(vector: Vector2) -> Self {
        [vector.x, vector.y]
    }
}

impl fmt::Display for Vector2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

// Operator implementations for Vector2
impl Add for Vector2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vector2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f32> for Vector2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vector2> for f32 {
    type Output = Vector2;
    #[inline]
    fn mul(self, rhs: Vector2) -> Self::Output {
        Vector2::new(self * rhs.x, self * rhs.y)
    }
}

impl Div<f32> for Vector2 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

impl Neg for Vector2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y)
    }
}

impl AddAssign for Vector2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl SubAssign for Vector2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl MulAssign<f32> for Vector2 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl DivAssign<f32> for Vector2 {
    #[inline]
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

// === Vector3 Implementation ===

impl Vector3 {
    /// A zero vector constant
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    /// Creates a new 3D vector
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Creates a new 3D vector with all components set to zero
    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Creates a new 3D vector with all components set to one
    #[inline]
    pub fn one() -> Self {
        Self { x: 1.0, y: 1.0, z: 1.0 }
    }

    /// Creates a unit vector pointing in the x direction
    #[inline]
    pub fn unit_x() -> Self {
        Self { x: 1.0, y: 0.0, z: 0.0 }
    }

    /// Creates a unit vector pointing in the y direction
    #[inline]
    pub fn unit_y() -> Self {
        Self { x: 0.0, y: 1.0, z: 0.0 }
    }

    /// Creates a unit vector pointing in the z direction
    #[inline]
    pub fn unit_z() -> Self {
        Self { x: 0.0, y: 0.0, z: 1.0 }
    }

    /// Computes the dot product of two vectors
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Computes the cross product of two vectors
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Returns the squared length of the vector
    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Returns the length of the vector
    #[inline]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Returns a normalized version of the vector
    #[inline]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        if length > crate::math::EPSILON {
            *self / length
        } else {
            *self
        }
    }

    /// Normalizes the vector in-place
    #[inline]
    pub fn normalize_mut(&mut self) {
        let length = self.length();
        if length > crate::math::EPSILON {
            self.x /= length;
            self.y /= length;
            self.z /= length;
        }
    }

    /// Returns true if the vector is approximately zero
    #[inline]
    pub fn is_zero(&self) -> bool {
        crate::math::approx_zero(self.length_squared())
    }

    /// Convert to nalgebra Vector3
    #[inline]
    pub fn to_nalgebra(&self) -> na::Vector3<f32> {
        na::Vector3::new(self.x, self.y, self.z)
    }

    /// Convert from nalgebra Vector3
    #[inline]
    pub fn from_nalgebra(v: &na::Vector3<f32>) -> Self {
        Self::new(v.x, v.y, v.z)
    }

    /// Distance between two vectors
    #[inline]
    pub fn distance(&self, other: &Self) -> f32 {
        (*self - *other).length()
    }

    /// Squared distance between two vectors
    #[inline]
    pub fn distance_squared(&self, other: &Self) -> f32 {
        (*self - *other).length_squared()
    }

    /// Linear interpolation between two vectors
    #[inline]
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        *self + (*other - *self) * t
    }

    /// Projects one vector onto another
    #[inline]
    pub fn project(&self, onto: &Self) -> Self {
        let onto_normalized = onto.normalize();
        onto_normalized * self.dot(&onto_normalized)
    }

    /// Rejects one vector from another (opposite of projection)
    #[inline]
    pub fn reject(&self, from: &Self) -> Self {
        *self - self.project(from)
    }

    /// Returns the angle in radians between two vectors
    #[inline]
    pub fn angle_between(&self, other: &Self) -> f32 {
        let dot = self.dot(other);
        let len_sq1 = self.length_squared();
        let len_sq2 = other.length_squared();
        
        if len_sq1 < crate::math::EPSILON || len_sq2 < crate::math::EPSILON {
            0.0
        } else {
            let cosine = dot / (len_sq1.sqrt() * len_sq2.sqrt());
            cosine.clamp(-1.0, 1.0).acos()
        }
    }
}

impl From<[f32; 3]> for Vector3 {
    #[inline]
    fn from(array: [f32; 3]) -> Self {
        Self::new(array[0], array[1], array[2])
    }
}

impl From<Vector3> for [f32; 3] {
    #[inline]
    fn from(vector: Vector3) -> Self {
        [vector.x, vector.y, vector.z]
    }
}

impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

// Operator implementations for Vector3
impl Add for Vector3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f32> for Vector3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vector3> for f32 {
    type Output = Vector3;
    #[inline]
    fn mul(self, rhs: Vector3) -> Self::Output {
        Vector3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl Div<f32> for Vector3 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Neg for Vector3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vector3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl SubAssign for Vector3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl MulAssign<f32> for Vector3 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl DivAssign<f32> for Vector3 {
    #[inline]
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

// === Vector4 Implementation ===

impl Vector4 {
    /// Creates a new 4D vector
    #[inline]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// Creates a new 4D vector with all components set to zero
    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }
    }

    /// Creates a new 4D vector with all components set to one
    #[inline]
    pub fn one() -> Self {
        Self { x: 1.0, y: 1.0, z: 1.0, w: 1.0 }
    }

    /// Computes the dot product of two vectors
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }

    /// Returns the squared length of the vector
    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    /// Returns the length of the vector
    #[inline]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Returns a normalized version of the vector
    #[inline]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        if length > crate::math::EPSILON {
            *self / length
        } else {
            *self
        }
    }

    /// Normalizes the vector in-place
    #[inline]
    pub fn normalize_mut(&mut self) {
        let length = self.length();
        if length > crate::math::EPSILON {
            self.x /= length;
            self.y /= length;
            self.z /= length;
            self.w /= length;
        }
    }

    /// Returns true if the vector is approximately zero
    #[inline]
    pub fn is_zero(&self) -> bool {
        crate::math::approx_zero(self.length_squared())
    }

    /// Convert to nalgebra Vector4
    #[inline]
    pub fn to_nalgebra(&self) -> na::Vector4<f32> {
        na::Vector4::new(self.x, self.y, self.z, self.w)
    }

    /// Convert from nalgebra Vector4
    #[inline]
    pub fn from_nalgebra(v: &na::Vector4<f32>) -> Self {
        Self::new(v.x, v.y, v.z, v.w)
    }

    /// Returns the x, y, z components as a Vector3
    #[inline]
    pub fn xyz(&self) -> Vector3 {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Creates a new Vector4 from a Vector3 and a w component
    #[inline]
    pub fn from_vector3(v: Vector3, w: f32) -> Self {
        Self::new(v.x, v.y, v.z, w)
    }
}

impl From<[f32; 4]> for Vector4 {
    #[inline]
    fn from(array: [f32; 4]) -> Self {
        Self::new(array[0], array[1], array[2], array[3])
    }
}

impl From<Vector4> for [f32; 4] {
    #[inline]
    fn from(vector: Vector4) -> Self {
        [vector.x, vector.y, vector.z, vector.w]
    }
}

impl fmt::Display for Vector4 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {}, {}, {})", self.x, self.y, self.z, self.w)
    }
}

// Operator implementations for Vector4
impl Add for Vector4 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z, self.w + rhs.w)
    }
}

impl Sub for Vector4 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z, self.w - rhs.w)
    }
}

impl Mul<f32> for Vector4 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs, self.w * rhs)
    }
}

impl Mul<Vector4> for f32 {
    type Output = Vector4;
    #[inline]
    fn mul(self, rhs: Vector4) -> Self::Output {
        Vector4::new(self * rhs.x, self * rhs.y, self * rhs.z, self * rhs.w)
    }
}

impl Div<f32> for Vector4 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs, self.w / rhs)
    }
}

impl Neg for Vector4 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z, -self.w)
    }
}

impl AddAssign for Vector4 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self.w += rhs.w;
    }
}

impl SubAssign for Vector4 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
        self.w -= rhs.w;
    }
}

impl MulAssign<f32> for Vector4 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
        self.w *= rhs;
    }
}

impl DivAssign<f32> for Vector4 {
    #[inline]
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
        self.w /= rhs;
    }
}