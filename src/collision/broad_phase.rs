use crate::core::BodyHandle;
use crate::bodies::RigidBody;
use crate::math::Aabb;
use crate::collision::collision_pair::CollisionPair;

/// Trait for broad-phase collision detection algorithms
pub trait BroadPhase {
    /// Updates the broad-phase with the current body states
    fn update(&mut self, bodies: &[(BodyHandle, &RigidBody)]);
    
    /// Gets all potential collision pairs
    fn get_collision_pairs(&self) -> Vec<CollisionPair>;
}

/// Simple brute-force broad-phase algorithm
pub struct BruteForceBroadPhase {
    /// The bodies in the broad-phase
    bodies: Vec<(BodyHandle, Aabb)>,
}

impl BruteForceBroadPhase {
    /// Creates a new brute-force broad-phase
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
        }
    }
}

impl BroadPhase for BruteForceBroadPhase {
    fn update(&mut self, bodies: &[(BodyHandle, &RigidBody)]) {
        self.bodies.clear();
        
        // Store the bodies and their bounding boxes
        for (handle, body) in bodies {
            if let Some(shape) = body.get_shape() {
                let aabb = shape.get_world_bounds(&body.get_transform());
                self.bodies.push((*handle, aabb));
            }
        }
    }
    
    fn get_collision_pairs(&self) -> Vec<CollisionPair> {
        let mut pairs = Vec::new();
        
        // Check all pairs of bodies
        for i in 0..self.bodies.len() {
            let (handle_a, aabb_a) = self.bodies[i];
            
            for j in (i + 1)..self.bodies.len() {
                let (handle_b, aabb_b) = self.bodies[j];
                
                // Check if the AABBs overlap
                if aabb_a.intersects(&aabb_b) {
                    pairs.push(CollisionPair::new(handle_a, handle_b));
                }
            }
        }
        
        pairs
    }
}

/// Spatial hashing broad-phase algorithm
pub struct SpatialHashingBroadPhase {
    /// The cell size (all dimensions)
    cell_size: f32,
    
    /// The cells containing bodies
    cells: std::collections::HashMap<(i32, i32, i32), Vec<BodyHandle>>,
    
    /// The bodies in the broad-phase
    bodies: Vec<(BodyHandle, Aabb)>,
}

impl SpatialHashingBroadPhase {
    /// Creates a new spatial hashing broad-phase
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            cells: std::collections::HashMap::new(),
            bodies: Vec::new(),
        }
    }
    
    /// Gets the cell index for a position
    fn get_cell_index(&self, position: &crate::math::Vector3) -> (i32, i32, i32) {
        (
            (position.x / self.cell_size).floor() as i32,
            (position.y / self.cell_size).floor() as i32,
            (position.z / self.cell_size).floor() as i32,
        )
    }
    
    /// Gets the range of cell indices for an AABB
    fn get_cell_range(&self, aabb: &Aabb) -> ((i32, i32, i32), (i32, i32, i32)) {
        let min_cell = self.get_cell_index(&aabb.min);
        let max_cell = self.get_cell_index(&aabb.max);
        
        (min_cell, max_cell)
    }
}

impl BroadPhase for SpatialHashingBroadPhase {
    fn update(&mut self, bodies: &[(BodyHandle, &RigidBody)]) {
        self.bodies.clear();
        self.cells.clear();
        
        // Store the bodies and their bounding boxes
        for (handle, body) in bodies {
            if let Some(shape) = body.get_shape() {
                let aabb = shape.get_world_bounds(&body.get_transform());
                self.bodies.push((*handle, aabb));
                
                // Add the body to each cell it overlaps
                let (min_cell, max_cell) = self.get_cell_range(&aabb);
                
                for x in min_cell.0..=max_cell.0 {
                    for y in min_cell.1..=max_cell.1 {
                        for z in min_cell.2..=max_cell.2 {
                            self.cells.entry((x, y, z)).or_insert_with(Vec::new).push(*handle);
                        }
                    }
                }
            }
        }
    }
    
    fn get_collision_pairs(&self) -> Vec<CollisionPair> {
        let mut pairs = std::collections::HashSet::new();
        
        // Check for collisions within each cell
        for bodies in self.cells.values() {
            for i in 0..bodies.len() {
                let handle_a = bodies[i];
                
                for j in (i + 1)..bodies.len() {
                    let handle_b = bodies[j];
                    
                    // Add the pair (ensuring consistent ordering)
                    pairs.insert(CollisionPair::new(handle_a, handle_b));
                }
            }
        }
        
        // Convert to a vector
        pairs.into_iter()
            .filter(|pair| {
                // Perform AABB check
                let aabb_a = &self.bodies.iter().find(|(h, _)| *h == pair.body_a).unwrap().1;
                let aabb_b = &self.bodies.iter().find(|(h, _)| *h == pair.body_b).unwrap().1;
                
                aabb_a.intersects(aabb_b)
            })
            .collect()
    }
}