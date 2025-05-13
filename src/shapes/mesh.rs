use crate::shapes::Shape;
use crate::math::{Vector3, Matrix3, Aabb, Ray, Transform};
use std::any::Any;
use std::sync::Mutex;

/// A triangle in a mesh
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    /// The vertices of the triangle
    pub vertices: [Vector3; 3],
}

impl Triangle {
    /// Creates a new triangle from three vertices
    pub fn new(a: Vector3, b: Vector3, c: Vector3) -> Self {
        Self { vertices: [a, b, c] }
    }
    
    /// Returns the normal of the triangle
    pub fn normal(&self) -> Vector3 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        edge1.cross(&edge2).normalize()
    }
    
    /// Returns the area of the triangle
    pub fn area(&self) -> f32 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        edge1.cross(&edge2).length() * 0.5
    }
    
    /// Returns the center (centroid) of the triangle
    pub fn center(&self) -> Vector3 {
        (self.vertices[0] + self.vertices[1] + self.vertices[2]) / 3.0
    }
    
    /// Checks if a ray intersects this triangle
    pub fn intersects_ray(&self, ray: &Ray) -> Option<f32> {
        // Moller-Trumbore algorithm
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        
        let h = ray.direction.cross(&edge2);
        let a = edge1.dot(&h);
        
        // Check if ray is parallel to triangle
        if a.abs() < crate::math::EPSILON {
            return None;
        }
        
        let f = 1.0 / a;
        let s = ray.origin - v0;
        let u = f * s.dot(&h);
        
        // Check if intersection is outside the triangle
        if u < 0.0 || u > 1.0 {
            return None;
        }
        
        let q = s.cross(&edge1);
        let v = f * ray.direction.dot(&q);
        
        // Check if intersection is outside the triangle
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        
        // Compute the distance along the ray
        let t = f * edge2.dot(&q);
        
        if t > crate::math::EPSILON {
            Some(t)
        } else {
            None
        }
    }
}

/// A triangular mesh collision shape
#[derive(Debug)]
pub struct Mesh {
    /// The vertices of the mesh
    vertices: Vec<Vector3>,

    /// The indices of the triangles
    indices: Vec<u32>,

    /// The triangles of the mesh (derived from vertices and indices)
    triangles: Vec<Triangle>,

    /// The local AABB of the mesh (cached)
    /// Using Mutex for thread-safe caching
    local_bounds: Mutex<Option<Aabb>>,
}

impl Clone for Mesh {
    fn clone(&self) -> Self {
        let bounds = if let Ok(bounds) = self.local_bounds.lock() {
            bounds.clone()
        } else {
            None
        };

        Self {
            vertices: self.vertices.clone(),
            indices: self.indices.clone(),
            triangles: self.triangles.clone(),
            local_bounds: Mutex::new(bounds),
        }
    }
}

impl Mesh {
    /// Creates a new empty mesh
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            triangles: Vec::new(),
            local_bounds: Mutex::new(None),
        }
    }
    
    /// Creates a mesh from vertices and indices
    pub fn from_vertices_and_indices(vertices: Vec<Vector3>, indices: Vec<u32>) -> Self {
        let mut mesh = Self {
            vertices,
            indices,
            triangles: Vec::new(),
            local_bounds: Mutex::new(None),
        };

        mesh.build_triangles();
        mesh.compute_local_bounds();

        mesh
    }
    
    /// Returns the vertices of the mesh
    pub fn get_vertices(&self) -> &[Vector3] {
        &self.vertices
    }
    
    /// Returns the indices of the mesh
    pub fn get_indices(&self) -> &[u32] {
        &self.indices
    }
    
    /// Returns the triangles of the mesh
    pub fn get_triangles(&self) -> &[Triangle] {
        &self.triangles
    }
    
    /// Returns the number of triangles in the mesh
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }
    
    /// Returns the number of vertices in the mesh
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
    
    /// Adds a vertex to the mesh
    pub fn add_vertex(&mut self, vertex: Vector3) -> u32 {
        let index = self.vertices.len() as u32;
        self.vertices.push(vertex);
        // Invalidate cached bounds
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
        index
    }
    
    /// Adds a triangle to the mesh by indices
    pub fn add_triangle(&mut self, a: u32, b: u32, c: u32) {
        self.indices.push(a);
        self.indices.push(b);
        self.indices.push(c);

        // Add the triangle if all vertices exist
        if a < self.vertices.len() as u32 &&
           b < self.vertices.len() as u32 &&
           c < self.vertices.len() as u32 {
            self.triangles.push(Triangle::new(
                self.vertices[a as usize],
                self.vertices[b as usize],
                self.vertices[c as usize],
            ));
        }

        // Invalidate cached bounds
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Clears the mesh
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
        self.triangles.clear();
        if let Ok(mut bounds) = self.local_bounds.lock() {
            *bounds = None;
        }
    }
    
    /// Builds the triangles from vertices and indices
    fn build_triangles(&mut self) {
        self.triangles.clear();
        
        if self.indices.len() < 3 {
            return;
        }
        
        for i in 0..self.indices.len() / 3 {
            let a = self.indices[i * 3] as usize;
            let b = self.indices[i * 3 + 1] as usize;
            let c = self.indices[i * 3 + 2] as usize;
            
            if a < self.vertices.len() && b < self.vertices.len() && c < self.vertices.len() {
                self.triangles.push(Triangle::new(
                    self.vertices[a],
                    self.vertices[b],
                    self.vertices[c],
                ));
            }
        }
    }
    
    /// Recomputes and caches the local AABB of the mesh
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

impl Shape for Mesh {
    fn shape_type(&self) -> &'static str {
        "Mesh"
    }
    
    fn get_volume(&self) -> f32 {
        // Compute volume for a closed mesh using the divergence theorem
        // This assumes the mesh is closed and has consistent winding
        
        let mut volume = 0.0;
        
        for triangle in &self.triangles {
            let v1 = triangle.vertices[0];
            let v2 = triangle.vertices[1];
            let v3 = triangle.vertices[2];
            
            // Compute signed volume of tetrahedron formed by triangle and origin
            volume += v1.dot(&v2.cross(&v3)) / 6.0;
        }
        
        volume.abs()
    }
    
    fn get_inertia_tensor(&self, mass: f32) -> Matrix3 {
        if self.triangles.is_empty() {
            return Matrix3::zero();
        }
        
        // Compute inertia tensor by summing contribution from each triangle
        // This is a simplification - for accurate results, each triangle
        // should be treated as a tetrahedron with the origin
        
        let mut xx = 0.0;
        let mut yy = 0.0;
        let mut zz = 0.0;
        let mut xy = 0.0;
        let mut xz = 0.0;
        let mut yz = 0.0;
        
        for triangle in &self.triangles {
            let v1 = triangle.vertices[0];
            let v2 = triangle.vertices[1];
            let v3 = triangle.vertices[2];
            
            // Compute contribution to inertia tensor
            for v in &[v1, v2, v3] {
                xx += v.y * v.y + v.z * v.z;
                yy += v.x * v.x + v.z * v.z;
                zz += v.x * v.x + v.y * v.y;
                xy -= v.x * v.y;
                xz -= v.x * v.z;
                yz -= v.y * v.z;
            }
        }
        
        // Scale by mass / total number of vertices
        let scale = mass / (self.vertices.len() as f32);
        
        xx *= scale;
        yy *= scale;
        zz *= scale;
        xy *= scale;
        xz *= scale;
        yz *= scale;
        
        Matrix3::new([
            [xx, xy, xz],
            [xy, yy, yz],
            [xz, yz, zz],
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
        // Transform ray to local space
        let inv_transform = transform.inverse();
        let local_ray = ray.transform(&inv_transform.to_matrix());
        
        // Test against all triangles
        let mut closest_t = f32::MAX;
        
        for triangle in &self.triangles {
            if let Some(t) = triangle.intersects_ray(&local_ray) {
                if t > 0.0 && t < closest_t && t <= max_distance {
                    closest_t = t;
                }
            }
        }
        
        if closest_t != f32::MAX {
            Some(closest_t)
        } else {
            None
        }
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