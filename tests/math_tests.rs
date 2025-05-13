use phys_engine::math::{Vector3, Matrix3, Quaternion, Aabb, Ray, Transform, Rotation};
use std::f32::consts::PI;
use approx::assert_relative_eq;

#[test]
fn test_vector3_operations() {
    let v1 = Vector3::new(1.0, 2.0, 3.0);
    let v2 = Vector3::new(4.0, 5.0, 6.0);
    
    // Addition
    let sum = v1 + v2;
    assert_eq!(sum.x, 5.0);
    assert_eq!(sum.y, 7.0);
    assert_eq!(sum.z, 9.0);
    
    // Subtraction
    let diff = v2 - v1;
    assert_eq!(diff.x, 3.0);
    assert_eq!(diff.y, 3.0);
    assert_eq!(diff.z, 3.0);
    
    // Scalar multiplication
    let scaled = v1 * 2.0;
    assert_eq!(scaled.x, 2.0);
    assert_eq!(scaled.y, 4.0);
    assert_eq!(scaled.z, 6.0);
    
    // Dot product
    let dot = v1.dot(&v2);
    assert_eq!(dot, 1.0 * 4.0 + 2.0 * 5.0 + 3.0 * 6.0);
    
    // Cross product
    let cross = v1.cross(&v2);
    assert_eq!(cross.x, v1.y * v2.z - v1.z * v2.y);
    assert_eq!(cross.y, v1.z * v2.x - v1.x * v2.z);
    assert_eq!(cross.z, v1.x * v2.y - v1.y * v2.x);
    
    // Length
    let length = v1.length();
    assert_relative_eq!(length, (1.0f32.powi(2) + 2.0f32.powi(2) + 3.0f32.powi(2)).sqrt());
    
    // Normalize
    let normalized = v1.normalize();
    assert_relative_eq!(normalized.length(), 1.0);
    assert_relative_eq!(normalized.x, v1.x / length);
    assert_relative_eq!(normalized.y, v1.y / length);
    assert_relative_eq!(normalized.z, v1.z / length);
}

#[test]
fn test_quaternion_operations() {
    // Create a quaternion from axis-angle
    let axis = Vector3::new(0.0, 1.0, 0.0);
    let angle = PI / 2.0; // 90 degrees
    let q = Quaternion::from_axis_angle(axis, angle);
    
    // Normalize
    let q_norm = q.normalize();
    assert_relative_eq!(q_norm.length(), 1.0);
    
    // Rotate a vector
    let v = Vector3::new(1.0, 0.0, 0.0);
    let rotated = q.rotate_vector(v);
    
    // v should be rotated 90 degrees around Y-axis to (0, 0, -1)
    assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.y, 0.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.z, -1.0, epsilon = 1e-5);
    
    // Conjugate
    let q_conj = q.conjugate();
    assert_eq!(q_conj.w, q.w);
    assert_eq!(q_conj.x, -q.x);
    assert_eq!(q_conj.y, -q.y);
    assert_eq!(q_conj.z, -q.z);
    
    // Inverse (same as conjugate for unit quaternions)
    let q_inv = q_norm.inverse();
    assert_relative_eq!(q_inv.w, q_norm.w);
    assert_relative_eq!(q_inv.x, -q_norm.x);
    assert_relative_eq!(q_inv.y, -q_norm.y);
    assert_relative_eq!(q_inv.z, -q_norm.z);
    
    // Multiplication (composition of rotations)
    let q1 = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), PI / 4.0);
    let q2 = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), PI / 4.0);
    let q3 = q2 * q1; // Apply q1 then q2
    
    let v2 = Vector3::new(0.0, 0.0, 1.0);
    let rotated_by_q3 = q3.rotate_vector(v2);
    let rotated_by_q1_then_q2 = q2.rotate_vector(q1.rotate_vector(v2));
    
    assert_relative_eq!(rotated_by_q3.x, rotated_by_q1_then_q2.x, epsilon = 1e-5);
    assert_relative_eq!(rotated_by_q3.y, rotated_by_q1_then_q2.y, epsilon = 1e-5);
    assert_relative_eq!(rotated_by_q3.z, rotated_by_q1_then_q2.z, epsilon = 1e-5);
}

#[test]
fn test_matrix3_operations() {
    // Identity matrix
    let identity = Matrix3::identity();
    
    // Test multiplication by identity
    let m = Matrix3::new([
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0],
    ]);
    
    let result = m.multiply_matrix(&identity);
    assert_eq!(result.data, m.data);
    
    // Test vector multiplication
    let v = Vector3::new(1.0, 2.0, 3.0);
    let mv = m.multiply_vector(v);
    
    assert_eq!(mv.x, 1.0 * v.x + 2.0 * v.y + 3.0 * v.z);
    assert_eq!(mv.y, 4.0 * v.x + 5.0 * v.y + 6.0 * v.z);
    assert_eq!(mv.z, 7.0 * v.x + 8.0 * v.y + 9.0 * v.z);
    
    // Test transpose
    let m_transpose = m.transpose();
    assert_eq!(m_transpose.data[0][0], m.data[0][0]);
    assert_eq!(m_transpose.data[0][1], m.data[1][0]);
    assert_eq!(m_transpose.data[0][2], m.data[2][0]);
    assert_eq!(m_transpose.data[1][0], m.data[0][1]);
    // ... and so on
}

#[test]
fn test_transform_operations() {
    // Create a transform
    let position = Vector3::new(1.0, 2.0, 3.0);
    let rotation = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), PI / 2.0);
    let scale = Vector3::new(2.0, 2.0, 2.0);
    
    let transform = Transform::new(position, rotation, scale);
    
    // Transform a point
    let point = Vector3::new(1.0, 0.0, 0.0);
    let transformed_point = transform.transform_point(point);
    
    // Expected: Scale by 2, rotate 90 degrees around Y, then translate
    // After scaling: (2, 0, 0)
    // After rotation: (0, 0, -2)
    // After translation: (1, 2, 1)
    assert_relative_eq!(transformed_point.x, 1.0, epsilon = 1e-5);
    assert_relative_eq!(transformed_point.y, 2.0, epsilon = 1e-5);
    assert_relative_eq!(transformed_point.z, 1.0, epsilon = 1e-5);
    
    // Test inverse transform
    let inverse = transform.inverse();
    let original = inverse.transform_point(transformed_point);
    
    assert_relative_eq!(original.x, point.x, epsilon = 1e-5);
    assert_relative_eq!(original.y, point.y, epsilon = 1e-5);
    assert_relative_eq!(original.z, point.z, epsilon = 1e-5);
}

#[test]
fn test_aabb_operations() {
    // Create an AABB
    let min = Vector3::new(-1.0, -2.0, -3.0);
    let max = Vector3::new(1.0, 2.0, 3.0);
    let aabb = Aabb::new(min, max);
    
    // Test center
    let center = aabb.center();
    assert_eq!(center.x, 0.0);
    assert_eq!(center.y, 0.0);
    assert_eq!(center.z, 0.0);
    
    // Test extents
    let extents = aabb.extents();
    assert_eq!(extents.x, 2.0);
    assert_eq!(extents.y, 4.0);
    assert_eq!(extents.z, 6.0);
    
    // Test contains point
    assert!(aabb.contains_point(Vector3::zero()));
    assert!(aabb.contains_point(Vector3::new(0.5, 1.0, 1.5)));
    assert!(!aabb.contains_point(Vector3::new(2.0, 0.0, 0.0)));
    
    // Test intersection
    let aabb2 = Aabb::new(Vector3::new(0.5, 0.5, 0.5), Vector3::new(2.0, 3.0, 4.0));
    assert!(aabb.intersects(&aabb2));
    
    let aabb3 = Aabb::new(Vector3::new(2.0, 3.0, 4.0), Vector3::new(3.0, 4.0, 5.0));
    assert!(!aabb.intersects(&aabb3));
    
    // Test union
    let union = aabb.union(&aabb2);
    assert_eq!(union.min.x, -1.0);
    assert_eq!(union.min.y, -2.0);
    assert_eq!(union.min.z, -3.0);
    assert_eq!(union.max.x, 2.0);
    assert_eq!(union.max.y, 3.0);
    assert_eq!(union.max.z, 4.0);
}

#[test]
fn test_ray_operations() {
    // Create a ray
    let origin = Vector3::new(0.0, 0.0, 0.0);
    let direction = Vector3::new(1.0, 0.0, 0.0);
    let ray = Ray::new(origin, direction);
    
    // Test point at parameter
    let point = ray.point_at(2.0);
    assert_eq!(point.x, 2.0);
    assert_eq!(point.y, 0.0);
    assert_eq!(point.z, 0.0);
    
    // Test closest point
    let test_point = Vector3::new(3.0, 2.0, 0.0);
    let closest = ray.closest_point(test_point);
    assert_eq!(closest.x, 3.0);
    assert_eq!(closest.y, 0.0);
    assert_eq!(closest.z, 0.0);
    
    // Test distance to point
    let distance = ray.distance_to_point(test_point);
    assert_eq!(distance, 2.0);
}