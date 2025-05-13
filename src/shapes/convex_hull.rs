use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;
use std::sync::Mutex;

/// A convex hull collision shape
#[derive(Debug)]
pub struct ConvexHull {
    /// The vertices of the convex hull
    vertices: Vec<Vector3>,

    /// The local AABB of the convex hull (cached)
    /// Using Mutex for thread-safe caching
    local_bounds: Mutex<Option<Aabb>>,
}

impl Clone for ConvexHull {
    fn clone(&self) -> Self {
        let bounds = if let Ok(bounds) = self.local_bounds.lock() {
            bounds.clone()
        } else {
            None
        };

        Self {
            vertices: self.vertices.clone(),
            local_bounds: Mutex::new(bounds),
        }
    }
}

impl ConvexHull {
    /// Creates a new empty convex hull
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            local_bounds: Mutex::new(None),
        }
    }
    
    /// Creates a convex hull from a set of points
    pub fn from_points(points: &[Vector3]) -> Self {
        let mut hull = Self {
            vertices: Vec::new(),
            local_bounds: Mutex::new(None),
        };

        hull.build_from_points(points);

        hull
    }
    
    /// Returns the vertices of the convex hull
    pub fn get_vertices(&self) -> &[Vector3] {
        &self.vertices
    }
    
    /// Returns the number of vertices in the convex hull
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
    
    /// Adds a vertex to the convex hull (does not maintain convexity)
    pub fn add_vertex(&mut self, vertex: Vector3) {
        self.vertices.push(vertex);
        // Invalidate cached bounds
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Clears the convex hull
    pub fn clear(&mut self) {
        self.vertices.clear();
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Builds the convex hull from a set of points
    fn build_from_points(&mut self, points: &[Vector3]) {
        // This is a very simplified convex hull algorithm for demonstration.
        // In a real implementation, you would use a proper convex hull algorithm
        // like Quickhull or Gift wrapping.
        
        // For now, we'll just copy all points and assume they form a convex hull
        self.vertices = points.to_vec();
        self.compute_local_bounds();
    }
    
    /// Recomputes and caches the local AABB of the convex hull
    fn compute_local_bounds(&self) {
        if self.vertices.is_empty() {
            if let Ok(mut bounds) = self.local_bounds.lock() {
                *bounds = Some(Aabb::new(Vector3::zero(), Vector3::zero()));
            }
            return;
        }

        let mut min = self.vertices[0];
        let mut max = self.vertices[0];

        for vertex in &self.vertices {
            min.x = min.x.min(vertex.x);
            min.y = min.y.min(vertex.y);
            min.z = min.z.min(vertex.z);

            max.x = max.x.max(vertex.x);
            max.y = max.y.max(vertex.y);
            max.z = max.z.max(vertex.z);
        }

        if let Ok(mut local_bounds) = self.local_bounds.lock() {
            *local_bounds = Some(Aabb::new(min, max));
        }
    }
}

impl Shape for ConvexHull {
    fn shape_type(&self) -> &'static str {
        "ConvexHull"
    }
    
    fn get_volume(&self) -> f32 {
        // Computing the volume of a general convex polyhedron requires triangulation
        // For a more accurate result, you would use a method like the divergence theorem
        // or decompose into tetrahedra.
        
        // For now, we'll estimate the volume as the volume of the AABB
        let bounds = self.get_local_bounds();
        let extents = bounds.extents();
        extents.x * extents.y * extents.z
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        if self.vertices.is_empty() {
            return Matrix3::zero();
        }
        
        // Compute inertia tensor by approximating with a box
        let bounds = self.get_local_bounds();
        let extents = bounds.extents();
        
        let xx = (extents.y * extents.y + extents.z * extents.z) / 12.0;
        let yy = (extents.x * extents.x + extents.z * extents.z) / 12.0;
        let zz = (extents.x * extents.x + extents.y * extents.y) / 12.0;
        
        Matrix3::new([
            [xx * mass, 0.0, 0.0],
            [0.0, yy * mass, 0.0],
            [0.0, 0.0, zz * mass],
        ])
    }
    
    fn get_local_bounds(&self) -> Aabb {
        // Try to get cached bounds first
        if let Ok(bounds) = self.local_bounds.lock() {
            if let Some(aabb) = bounds.as_ref() {
                return aabb.clone();
            }
        }

        // Compute bounds if not cached
        self.compute_local_bounds();

        // Return the computed bounds
        if let Ok(bounds) = self.local_bounds.lock() {
            bounds.clone().unwrap_or_else(|| Aabb::new(Vector3::zero(), Vector3::zero()))
        } else {
            // Fall back to default in case of lock error
            Aabb::new(Vector3::zero(), Vector3::zero())
        }
    }
    
    fn get_world_bounds(&self, transform: &Transform) -> Aabb {
        if self.vertices.is_empty() {
            return Aabb::new(
                transform.position,
                transform.position,
            );
        }
        
        // Transform all vertices to world space and find the bounds
        let mut min = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut max = Vector3::new(f32::MIN, f32::MIN, f32::MIN);
        
        for vertex in &self.vertices {
            let world_vertex = transform.transform_point(*vertex);
            
            min.x = min.x.min(world_vertex.x);
            min.y = min.y.min(world_vertex.y);
            min.z = min.z.min(world_vertex.z);
            
            max.x = max.x.max(world_vertex.x);
            max.y = max.y.max(world_vertex.y);
            max.z = max.z.max(world_vertex.z);
        }
        
        Aabb::new(min, max)
    }
    
    fn get_support_point(&self, direction: Vector3) -> Vector3 {
        if self.vertices.is_empty() {
            return Vector3::zero();
        }
        
        if direction.is_zero() {
            return self.vertices[0];
        }
        
        // Find the furthest vertex in the given direction
        let mut furthest_distance = -f32::MAX;
        let mut furthest_vertex = self.vertices[0];
        
        for vertex in &self.vertices {
            let distance = vertex.dot(&direction);
            if distance > furthest_distance {
                furthest_distance = distance;
                furthest_vertex = *vertex;
            }
        }
        
        furthest_vertex
    }
    
    fn intersects_ray(&self, ray: &Ray, transform: &Transform, max_distance: f32) -> Option<f32> {
        // Ray-convex hull intersection is complex and often implemented using
        // the separating axis theorem or GJK algorithm. For simplicity, we'll
        // just check the AABB for now.
        
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Check AABB intersection
        let bounds = self.get_local_bounds();
        
        bounds.intersects_ray(&local_ray, 0.0, max_distance).map(|(t_min, _)| t_min)
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_shape(&self) -> Box<dyn Shape> {
        Box::new(self.clone())
    }
}