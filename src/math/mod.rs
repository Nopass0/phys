mod vector;
mod matrix;
mod transform;
mod rotation;
mod aabb;
mod ray;

pub use vector::{Vector2, Vector3, Vector4};
pub use matrix::{Matrix3, Matrix4};
pub use transform::Transform;
pub use rotation::{Quaternion, Rotation};
pub use aabb::Aabb;
pub use ray::Ray;

/// Constant for a very small number, used for comparisons
pub const EPSILON: f32 = 1.0e-6;

/// Returns true if the two floating point values are approximately equal
#[inline]
pub fn approx_eq(a: f32, b: f32) -> bool {
    (a - b).abs() < EPSILON
}

/// Returns true if the value is approximately zero
#[inline]
pub fn approx_zero(a: f32) -> bool {
    a.abs() < EPSILON
}

/// Clamps a value between a minimum and maximum value
#[inline]
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

/// Linearly interpolates between two values
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Converts degrees to radians
#[inline]
pub fn to_radians(degrees: f32) -> f32 {
    degrees * std::f32::consts::PI / 180.0
}

/// Converts radians to degrees
#[inline]
pub fn to_degrees(radians: f32) -> f32 {
    radians * 180.0 / std::f32::consts::PI
}