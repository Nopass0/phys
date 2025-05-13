use crate::math::{Vector3, Transform};
use crate::shapes::Shape;
use crate::collision::epa::EPA;

/// GJK (Gilbert-Johnson-Keerthi) algorithm for collision detection
pub struct GJK;

/// Maximum number of iterations for the GJK algorithm
const MAX_ITERATIONS: usize = 32;

/// A simplex is a geometric shape: point, line, triangle, or tetrahedron
#[derive(Debug, Clone)]
pub struct Simplex {
    /// The points in the simplex (max 4)
    points: Vec<Vector3>,
}

impl Simplex {
    /// Creates a new empty simplex
    pub fn new() -> Self {
        Self {
            points: Vec::with_capacity(4),
        }
    }
    
    /// Adds a point to the simplex
    pub fn add_point(&mut self, point: Vector3) {
        self.points.push(point);
    }
    
    /// Gets the number of points in the simplex
    pub fn size(&self) -> usize {
        self.points.len()
    }
    
    /// Gets a reference to the points in the simplex
    pub fn get_points(&self) -> &[Vector3] {
        &self.points
    }
    
    /// Gets a mutable reference to the points in the simplex
    pub fn get_points_mut(&mut self) -> &mut Vec<Vector3> {
        &mut self.points
    }
    
    /// Clears the simplex
    pub fn clear(&mut self) {
        self.points.clear();
    }
    
    /// Gets the last point added to the simplex
    pub fn last_point(&self) -> Option<Vector3> {
        self.points.last().copied()
    }
}

impl GJK {
    /// Checks if two shapes are colliding
    pub fn intersect(
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
    ) -> bool {
        let mut simplex = Simplex::new();
        
        // Get initial direction (can be any non-zero vector)
        let mut direction = Vector3::new(1.0, 0.0, 0.0);
        
        // Get first support point
        let support = Self::support(shape_a, transform_a, shape_b, transform_b, direction);
        simplex.add_point(support);
        
        // Negate the direction for the next iteration
        direction = -direction;
        
        // Iterate until we find a separating axis or detect a collision
        for _ in 0..MAX_ITERATIONS {
            // Get the next support point
            let support = Self::support(shape_a, transform_a, shape_b, transform_b, direction);
            
            // If the support point is not past the origin in the specified direction,
            // then the shapes are not colliding
            if support.dot(&direction) < 0.0 {
                return false;
            }
            
            // Add the support point to the simplex
            simplex.add_point(support);
            
            // Check if the simplex contains the origin
            if Self::next_simplex(&mut simplex, &mut direction) {
                return true;
            }
        }
        
        // If we reach the maximum iterations without finding a separating axis,
        // we conservatively assume the shapes are not colliding
        false
    }
    
    /// Returns the closest point to the origin on the given line segment
    fn closest_point_on_line(a: Vector3, b: Vector3) -> Vector3 {
        let ab = b - a;
        let t = -a.dot(&ab) / ab.length_squared();
        
        if t <= 0.0 {
            a
        } else if t >= 1.0 {
            b
        } else {
            a + ab * t
        }
    }
    
    /// Returns the closest point to the origin on the given triangle
    fn closest_point_on_triangle(a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
        // Check if the origin is in the Voronoi region of vertex a
        let ab = b - a;
        let ac = c - a;
        let ao = -a;
        
        let d1 = ab.dot(&ao);
        let d2 = ac.dot(&ao);
        
        if d1 <= 0.0 && d2 <= 0.0 {
            return a;
        }
        
        // Check if the origin is in the Voronoi region of vertex b
        let bo = -b;
        let d3 = ab.dot(&bo);
        let d4 = ac.dot(&bo);
        
        if d3 >= 0.0 && d4 <= d3 {
            return b;
        }
        
        // Check if the origin is in the Voronoi region of edge ab
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return a + ab * v;
        }
        
        // Check if the origin is in the Voronoi region of vertex c
        let co = -c;
        let d5 = ab.dot(&co);
        let d6 = ac.dot(&co);
        
        if d6 >= 0.0 && d5 <= d6 {
            return c;
        }
        
        // Check if the origin is in the Voronoi region of edge ac
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return a + ac * w;
        }
        
        // Check if the origin is in the Voronoi region of edge bc
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w;
        }
        
        // The origin is inside the triangle
        // Project the origin onto the triangle plane
        let n = ab.cross(&ac);
        let t = n.dot(&a) / n.dot(&n);
        n * t
    }
    
    /// Checks if the simplex contains the origin and updates the search direction
    fn next_simplex(simplex: &mut Simplex, direction: &mut Vector3) -> bool {
        match simplex.size() {
            2 => Self::line_case(simplex, direction),
            3 => Self::triangle_case(simplex, direction),
            4 => Self::tetrahedron_case(simplex, direction),
            _ => false,
        }
    }
    
    /// Handles the case where the simplex is a line
    fn line_case(simplex: &mut Simplex, direction: &mut Vector3) -> bool {
        let points = simplex.get_points();
        let a = points[1];
        let b = points[0];
        
        let ab = b - a;
        let ao = -a;
        
        if ab.dot(&ao) > 0.0 {
            // The origin is in the direction of the line
            *direction = ab.cross(&ao).cross(&ab);
        } else {
            // The origin is behind a
            simplex.get_points_mut().remove(0);
            *direction = ao;
        }
        
        false
    }
    
    /// Handles the case where the simplex is a triangle
    fn triangle_case(simplex: &mut Simplex, direction: &mut Vector3) -> bool {
        let points = simplex.get_points();
        let a = points[2];
        let b = points[1];
        let c = points[0];
        
        let ab = b - a;
        let ac = c - a;
        let ao = -a;
        
        let abc = ab.cross(&ac);
        
        if abc.cross(&ac).dot(&ao) > 0.0 {
            if ac.dot(&ao) > 0.0 {
                // Origin is in the Voronoi region of AC
                simplex.get_points_mut().remove(1);
                *direction = ac.cross(&ao).cross(&ac);
            } else if ab.dot(&ao) > 0.0 {
                // Origin is in the Voronoi region of AB
                simplex.get_points_mut().remove(0);
                *direction = ab.cross(&ao).cross(&ab);
            } else {
                // Origin is in the Voronoi region of A
                simplex.get_points_mut().remove(0);
                simplex.get_points_mut().remove(0);
                *direction = ao;
            }
        } else if abc.cross(&ab).dot(&ao) > 0.0 {
            if ab.dot(&ao) > 0.0 {
                // Origin is in the Voronoi region of AB
                simplex.get_points_mut().remove(0);
                *direction = ab.cross(&ao).cross(&ab);
            } else if ac.dot(&ao) > 0.0 {
                // Origin is in the Voronoi region of AC
                simplex.get_points_mut().remove(1);
                *direction = ac.cross(&ao).cross(&ac);
            } else {
                // Origin is in the Voronoi region of A
                simplex.get_points_mut().remove(0);
                simplex.get_points_mut().remove(0);
                *direction = ao;
            }
        } else {
            // Origin is above or below the triangle
            if abc.dot(&ao) > 0.0 {
                // Origin is above the triangle
                *direction = abc;
            } else {
                // Origin is below the triangle
                simplex.get_points_mut().swap(0, 1);
                *direction = -abc;
            }
        }
        
        false
    }
    
    /// Handles the case where the simplex is a tetrahedron
    fn tetrahedron_case(simplex: &mut Simplex, direction: &mut Vector3) -> bool {
        let points = simplex.get_points();
        let a = points[3];
        let b = points[2];
        let c = points[1];
        let d = points[0];
        
        let ab = b - a;
        let ac = c - a;
        let ad = d - a;
        let ao = -a;
        
        let abc = ab.cross(&ac);
        let acd = ac.cross(&ad);
        let adb = ad.cross(&ab);
        
        // Check if the origin is in the positive side of any face
        if abc.dot(&ao) > 0.0 {
            // Origin is outside face ABC
            simplex.get_points_mut().remove(0);
            *direction = abc;
            return false;
        }
        
        if acd.dot(&ao) > 0.0 {
            // Origin is outside face ACD
            simplex.get_points_mut().remove(2);
            *direction = acd;
            return false;
        }
        
        if adb.dot(&ao) > 0.0 {
            // Origin is outside face ADB
            simplex.get_points_mut().remove(1);
            *direction = adb;
            return false;
        }
        
        // Check if the origin is in the positive side of the fourth face
        let bcd = (c - b).cross(&(d - b));
        if bcd.dot(&(b - Vector3::zero())) > 0.0 {
            // Origin is outside face BCD
            simplex.get_points_mut().remove(3);
            *direction = bcd;
            return false;
        }
        
        // If we get here, the origin is inside the tetrahedron
        true
    }
    
    /// Gets the support point in the given direction
    fn support(
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
        direction: Vector3,
    ) -> Vector3 {
        let support_a = shape_a.get_world_support_point(direction, transform_a);
        let support_b = shape_b.get_world_support_point(-direction, transform_b);
        
        support_a - support_b
    }
    
    /// Performs collision detection and returns the penetration depth and normal
    pub fn detect_collision(
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
    ) -> Option<(f32, Vector3)> {
        let mut simplex = Simplex::new();
        
        // Get initial direction (can be any non-zero vector)
        let mut direction = Vector3::new(1.0, 0.0, 0.0);
        
        // Get first support point
        let support = Self::support(shape_a, transform_a, shape_b, transform_b, direction);
        simplex.add_point(support);
        
        // Negate the direction for the next iteration
        direction = -direction;
        
        // Iterate until we find a separating axis or detect a collision
        for _ in 0..MAX_ITERATIONS {
            // Get the next support point
            let support = Self::support(shape_a, transform_a, shape_b, transform_b, direction);
            
            // If the support point is not past the origin in the specified direction,
            // then the shapes are not colliding
            if support.dot(&direction) < 0.0 {
                return None;
            }
            
            // Add the support point to the simplex
            simplex.add_point(support);
            
            // Check if the simplex contains the origin
            if Self::next_simplex(&mut simplex, &mut direction) {
                // If the simplex contains the origin, use EPA to find the penetration depth
                // If EPA fails for any reason, we still want to report a collision

                // Clone the simplex to avoid move issue
                let simplex_clone = simplex.clone();

                // Get a copy of the points for possible fallback
                let points_copy: Vec<Vector3> = simplex.get_points().to_vec();

                let epa_result = EPA::penetration_depth(
                    simplex_clone,
                    shape_a,
                    transform_a,
                    shape_b,
                    transform_b,
                );

                // If EPA fails, use a fallback value
                return epa_result.or_else(|| {
                    // Get the centroid of the simplex as a fallback direction
                    let mut centroid = Vector3::zero();
                    for point in &points_copy {
                        centroid = centroid + *point;
                    }
                    if points_copy.len() > 0 {
                        centroid = centroid / points_copy.len() as f32;
                        let dir = -centroid.normalize();
                        Some((0.01, dir))
                    } else {
                        // Last resort fallback
                        Some((0.01, Vector3::new(0.0, 1.0, 0.0)))
                    }
                });
            }
        }
        
        // If we reach the maximum iterations without finding a separating axis,
        // we conservatively assume the shapes are not colliding
        None
    }
}