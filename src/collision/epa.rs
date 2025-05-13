use crate::math::{Vector3, Transform};
use crate::shapes::Shape;
use crate::collision::gjk::Simplex;

/// Expanding Polytope Algorithm (EPA) for computing penetration depth
pub struct EPA;

/// Maximum number of iterations for the EPA algorithm
const MAX_ITERATIONS: usize = 32;

/// Small value to handle numerical stability issues
const EPSILON: f32 = 1e-6;

/// A face of the polytope
#[derive(Debug, Clone)]
struct Face {
    /// The vertices of the face
    vertices: [usize; 3],
    
    /// The normal of the face
    normal: Vector3,
    
    /// The distance from the origin to the face
    distance: f32,
}

impl EPA {
    /// Compute the penetration depth and normal of an intersection
    pub fn penetration_depth(
        simplex: Simplex,
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
    ) -> Option<(f32, Vector3)> {
        let mut vertices = simplex.get_points().to_vec();
        let mut faces = Vec::new();

        // Build the initial polytope from the simplex (should be a tetrahedron)
        if vertices.len() < 4 {
            // If the simplex is not a tetrahedron, expand it to one
            Self::expand_simplex_to_tetrahedron(
                &mut vertices,
                shape_a,
                transform_a,
                shape_b,
                transform_b,
            );
        }

        // If we still don't have enough vertices, return a default value
        if vertices.len() < 4 {
            // Return a small penetration in the y-direction as a fallback
            return Some((0.01, Vector3::new(0.0, 1.0, 0.0)));
        }

        // Create the initial faces
        Self::create_initial_faces(&vertices, &mut faces);

        // If no faces were created, return a default value
        if faces.is_empty() {
            return Some((0.01, Vector3::new(0.0, 1.0, 0.0)));
        }

        // Expand the polytope until we find the closest face to the origin
        for _ in 0..MAX_ITERATIONS {
            // Find the face closest to the origin
            let (closest_face_idx, closest_face) = match Self::find_closest_face(&faces) {
                Some(result) => result,
                None => {
                    // If there are no faces, return a default penetration
                    return Some((0.01, Vector3::new(0.0, 1.0, 0.0)));
                }
            };

            // Get the normal of the closest face
            let normal = closest_face.normal;
            
            // Get the support point in the direction of the normal
            let support = Self::get_support(
                shape_a,
                transform_a,
                shape_b,
                transform_b,
                normal,
            );
            
            // Calculate the distance from the origin to the support point along the normal
            let support_dist = support.dot(&normal);
            
            // If the support point is not significantly further from the origin than the closest face,
            // we've found the closest face
            if support_dist - closest_face.distance < EPSILON {
                return Some((closest_face.distance, normal));
            }
            
            // Otherwise, expand the polytope with the new support point
            vertices.push(support);
            
            // Remove faces that can "see" the new point
            let mut edges = Vec::new();
            let mut i = 0;
            while i < faces.len() {
                let face = &faces[i];
                let can_see = (support - vertices[face.vertices[0]]).dot(&face.normal) > 0.0;

                if can_see {
                    // Add the edges of the face to the list of edges
                    Self::add_edge(&mut edges, face.vertices[0], face.vertices[1]);
                    Self::add_edge(&mut edges, face.vertices[1], face.vertices[2]);
                    Self::add_edge(&mut edges, face.vertices[2], face.vertices[0]);

                    // Remove the face
                    faces.swap_remove(i);
                } else {
                    i += 1;
                }
            }

            // Add new faces connecting the edges to the new point
            let new_vertex_idx = vertices.len() - 1;
            let edges_copy = edges.clone(); // Create a copy to iterate over
            for (i, j) in edges_copy {
                let normal = Self::compute_normal(&vertices, i, j, new_vertex_idx);
                let distance = normal.dot(&vertices[i]);

                faces.push(Face {
                    vertices: [i, j, new_vertex_idx],
                    normal,
                    distance,
                });
            }

            // If no faces were removed, we've failed to expand the polytope
            if edges.is_empty() {
                break;
            }
        }
        
        // If we reach here, the algorithm has failed to converge
        // We return the best estimate we have
        if let Some((_, closest_face)) = Self::find_closest_face(&faces) {
            Some((closest_face.distance, closest_face.normal))
        } else {
            // Fallback if we have no faces - shouldn't happen but let's be safe
            Some((0.01, Vector3::new(0.0, 1.0, 0.0)))
        }
    }
    
    /// Expands the simplex to a tetrahedron
    fn expand_simplex_to_tetrahedron(
        vertices: &mut Vec<Vector3>,
        shape_a: &dyn Shape,
        transform_a: &Transform,
        shape_b: &dyn Shape,
        transform_b: &Transform,
    ) {
        // Ensure we have at least one point
        if vertices.is_empty() {
            let support = Self::get_support(
                shape_a,
                transform_a,
                shape_b,
                transform_b,
                Vector3::new(1.0, 0.0, 0.0),
            );
            vertices.push(support);
        }
        
        // If we only have one point, find a second point
        if vertices.len() == 1 {
            let support = Self::get_support(
                shape_a,
                transform_a,
                shape_b,
                transform_b,
                Vector3::new(-1.0, 0.0, 0.0),
            );
            
            if (support - vertices[0]).length_squared() < EPSILON {
                // Try a different direction
                let support = Self::get_support(
                    shape_a,
                    transform_a,
                    shape_b,
                    transform_b,
                    Vector3::new(0.0, 1.0, 0.0),
                );
                vertices.push(support);
            } else {
                vertices.push(support);
            }
        }
        
        // If we only have two points, find a third point
        if vertices.len() == 2 {
            let a = vertices[0];
            let b = vertices[1];
            let ab = b - a;
            
            // Choose a direction perpendicular to the line
            let direction = ab.cross(&Vector3::new(1.0, 0.0, 0.0));
            if direction.length_squared() < EPSILON {
                // Try a different direction
                let direction = ab.cross(&Vector3::new(0.0, 1.0, 0.0));
                
                if direction.length_squared() < EPSILON {
                    // Try a different direction
                    let direction = ab.cross(&Vector3::new(0.0, 0.0, 1.0));
                    
                    let support = Self::get_support(
                        shape_a,
                        transform_a,
                        shape_b,
                        transform_b,
                        direction.normalize(),
                    );
                    vertices.push(support);
                } else {
                    let support = Self::get_support(
                        shape_a,
                        transform_a,
                        shape_b,
                        transform_b,
                        direction.normalize(),
                    );
                    vertices.push(support);
                }
            } else {
                let support = Self::get_support(
                    shape_a,
                    transform_a,
                    shape_b,
                    transform_b,
                    direction.normalize(),
                );
                vertices.push(support);
            }
        }
        
        // If we only have three points, find a fourth point
        if vertices.len() == 3 {
            let a = vertices[0];
            let b = vertices[1];
            let c = vertices[2];
            
            // Compute the normal of the triangle
            let normal = (b - a).cross(&(c - a)).normalize();
            
            // Get a support point in the direction of the normal
            let support1 = Self::get_support(
                shape_a,
                transform_a,
                shape_b,
                transform_b,
                normal,
            );
            
            // Get a support point in the opposite direction
            let support2 = Self::get_support(
                shape_a,
                transform_a,
                shape_b,
                transform_b,
                -normal,
            );
            
            // Choose the support point that's furthest from the triangle
            let dist1 = (support1 - a).dot(&normal);
            let dist2 = (support2 - a).dot(&-normal);
            
            if dist1 > dist2 {
                vertices.push(support1);
            } else {
                vertices.push(support2);
            }
        }
    }
    
    /// Creates the initial faces of the polytope
    fn create_initial_faces(vertices: &[Vector3], faces: &mut Vec<Face>) {
        // Clear any existing faces
        faces.clear();

        // The polytope should be a tetrahedron with 4 vertices
        if vertices.len() < 4 {
            return;
        }

        // Detect if the tetrahedron is degenerate - we need to check if vertices are coplanar
        let a = vertices[0];
        let b = vertices[1];
        let c = vertices[2];
        let d = vertices[3];

        // Compute volume of tetrahedron using triple scalar product
        let ab = b - a;
        let ac = c - a;
        let ad = d - a;
        let volume = ab.dot(&ac.cross(&ad)).abs() / 6.0;

        // If the volume is too small, the tetrahedron is nearly degenerate
        // In this case, don't create faces to avoid numerical issues
        if volume < 1e-8 {
            return;
        }

        // Create the faces of the tetrahedron
        Self::add_face(vertices, faces, 0, 1, 2);
        Self::add_face(vertices, faces, 0, 3, 1);
        Self::add_face(vertices, faces, 0, 2, 3);
        Self::add_face(vertices, faces, 1, 3, 2);
    }
    
    /// Adds a face to the polytope
    fn add_face(vertices: &[Vector3], faces: &mut Vec<Face>, a: usize, b: usize, c: usize) {
        // Compute the normal of the face
        let ab = vertices[b] - vertices[a];
        let ac = vertices[c] - vertices[a];
        let cross = ab.cross(&ac);

        // Check if the face is degenerate (triangle has zero area)
        if cross.length_squared() < 1e-10 {
            return;  // Skip degenerate faces
        }

        // Normalize the normal
        let normal = cross.normalize();
        let distance = normal.dot(&vertices[a]);

        // Ensure the normal points outward (away from the origin)
        if distance < 0.0 {
            faces.push(Face {
                vertices: [a, c, b],
                normal: -normal,
                distance: -distance,
            });
        } else {
            faces.push(Face {
                vertices: [a, b, c],
                normal,
                distance,
            });
        }
    }
    
    /// Computes the normal of a face
    fn compute_normal(vertices: &[Vector3], a: usize, b: usize, c: usize) -> Vector3 {
        let ab = vertices[b] - vertices[a];
        let ac = vertices[c] - vertices[a];
        let cross = ab.cross(&ac);

        // Check if the normal is degenerate
        let length_squared = cross.length_squared();
        if length_squared < 1e-10 {
            // Return a non-zero vector as a fallback
            // We could use a more sophisticated approach, but this is simple and effective
            Vector3::new(0.0, 1.0, 0.0)
        } else {
            cross / length_squared.sqrt()
        }
    }
    
    /// Finds the face closest to the origin
    fn find_closest_face<'a>(faces: &'a [Face]) -> Option<(usize, &'a Face)> {
        // Make sure we have at least one face
        if faces.is_empty() {
            return None;
        }

        let mut closest_idx = 0;
        let mut closest_distance = faces[0].distance;

        for (i, face) in faces.iter().enumerate().skip(1) {
            if face.distance < closest_distance {
                closest_idx = i;
                closest_distance = face.distance;
            }
        }

        Some((closest_idx, &faces[closest_idx]))
    }
    
    /// Adds an edge to the edge list, ensuring it's unique
    fn add_edge(edges: &mut Vec<(usize, usize)>, a: usize, b: usize) {
        // Ensure a < b
        let (a, b) = if a < b { (a, b) } else { (b, a) };
        
        // Check if the edge already exists and remove it (boundary edge)
        let idx = edges.iter().position(|&(x, y)| x == a && y == b);
        if let Some(idx) = idx {
            edges.swap_remove(idx);
        } else {
            // Add the edge (if it doesn't exist)
            edges.push((a, b));
        }
    }
    
    /// Gets the support point in the given direction
    fn get_support(
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
}