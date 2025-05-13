use crate::core::{BodyHandle, ConstraintHandle};
use std::collections::{HashSet, VecDeque};

/// An island of bodies and constraints that are connected through contacts or constraints
#[derive(Debug, Default, Clone)]
pub struct Island {
    /// The bodies in the island
    pub bodies: HashSet<BodyHandle>,
    
    /// The constraints in the island
    pub constraints: HashSet<ConstraintHandle>,
    
    /// Whether the island can go to sleep
    pub can_sleep: bool,
    
    /// The time the island has been inactive
    pub inactive_time: f32,
}

impl Island {
    /// Creates a new empty island
    pub fn new() -> Self {
        Self {
            bodies: HashSet::new(),
            constraints: HashSet::new(),
            can_sleep: true,
            inactive_time: 0.0,
        }
    }
    
    /// Adds a body to the island
    pub fn add_body(&mut self, body: BodyHandle) {
        self.bodies.insert(body);
    }
    
    /// Adds a constraint to the island
    pub fn add_constraint(&mut self, constraint: ConstraintHandle) {
        self.constraints.insert(constraint);
    }
    
    /// Returns whether the island contains a specific body
    pub fn contains_body(&self, body: BodyHandle) -> bool {
        self.bodies.contains(&body)
    }
    
    /// Returns whether the island contains a specific constraint
    pub fn contains_constraint(&self, constraint: ConstraintHandle) -> bool {
        self.constraints.contains(&constraint)
    }
    
    /// Merges another island into this one
    pub fn merge(&mut self, other: &Island) {
        for body in &other.bodies {
            self.bodies.insert(*body);
        }
        
        for constraint in &other.constraints {
            self.constraints.insert(*constraint);
        }
        
        // If either island can't sleep, the merged island can't sleep
        self.can_sleep = self.can_sleep && other.can_sleep;
        
        // The inactive time is the minimum of the two islands
        self.inactive_time = self.inactive_time.min(other.inactive_time);
    }
    
    /// Clears the island
    pub fn clear(&mut self) {
        self.bodies.clear();
        self.constraints.clear();
        self.can_sleep = true;
        self.inactive_time = 0.0;
    }
    
    /// Returns the number of bodies in the island
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    
    /// Returns the number of constraints in the island
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }
    
    /// Returns whether the island is empty
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty() && self.constraints.is_empty()
    }
}

/// Utility for building islands from a set of bodies and constraints
pub struct IslandBuilder {
    /// The islands that have been built
    islands: Vec<Island>,
    
    /// A mapping from body handles to island indices
    body_to_island: std::collections::HashMap<BodyHandle, usize>,
    
    /// A mapping from constraint handles to island indices
    constraint_to_island: std::collections::HashMap<ConstraintHandle, usize>,
}

impl IslandBuilder {
    /// Creates a new empty island builder
    pub fn new() -> Self {
        Self {
            islands: Vec::new(),
            body_to_island: std::collections::HashMap::new(),
            constraint_to_island: std::collections::HashMap::new(),
        }
    }
    
    /// Adds a connection between two bodies through a constraint
    pub fn add_connection(&mut self, body_a: BodyHandle, body_b: BodyHandle, constraint: ConstraintHandle) {
        let island_a = self.body_to_island.get(&body_a).copied();
        let island_b = self.body_to_island.get(&body_b).copied();
        let island_c = self.constraint_to_island.get(&constraint).copied();
        
        match (island_a, island_b, island_c) {
            (Some(a), Some(b), Some(c)) if a == b && b == c => {
                // All already in the same island, nothing to do
            }
            (Some(a), Some(b), Some(c)) if a == b && b != c => {
                // Bodies in same island, constraint in different island
                self.merge_islands(a, c);
            }
            (Some(a), Some(b), Some(c)) if a != b && (a == c || b == c) => {
                // One body and constraint in same island, other body in different island
                if a == c {
                    self.merge_islands(a, b);
                } else {
                    self.merge_islands(b, a);
                }
            }
            (Some(a), Some(b), Some(c)) => {
                // All in different islands
                self.merge_islands(a, b);
                self.merge_islands(a, c);
            }
            (Some(a), Some(b), None) => {
                // Bodies in islands, constraint not assigned
                if a != b {
                    self.merge_islands(a, b);
                }
                self.assign_constraint(constraint, a);
            }
            (Some(a), None, Some(c)) => {
                // Body A and constraint in islands, body B not assigned
                if a != c {
                    self.merge_islands(a, c);
                }
                self.assign_body(body_b, a);
            }
            (None, Some(b), Some(c)) => {
                // Body B and constraint in islands, body A not assigned
                if b != c {
                    self.merge_islands(b, c);
                }
                self.assign_body(body_a, b);
            }
            (Some(a), None, None) => {
                // Only body A assigned to an island
                self.assign_body(body_b, a);
                self.assign_constraint(constraint, a);
            }
            (None, Some(b), None) => {
                // Only body B assigned to an island
                self.assign_body(body_a, b);
                self.assign_constraint(constraint, b);
            }
            (None, None, Some(c)) => {
                // Only constraint assigned to an island
                self.assign_body(body_a, c);
                self.assign_body(body_b, c);
            }
            (None, None, None) => {
                // Nothing assigned yet, create a new island
                let island_idx = self.islands.len();
                self.islands.push(Island::new());
                
                self.assign_body(body_a, island_idx);
                self.assign_body(body_b, island_idx);
                self.assign_constraint(constraint, island_idx);
            }
        }
    }
    
    /// Adds a single body with no constraints
    pub fn add_single_body(&mut self, body: BodyHandle) {
        if self.body_to_island.contains_key(&body) {
            // Body already in an island
            return;
        }
        
        let island_idx = self.islands.len();
        self.islands.push(Island::new());
        
        self.assign_body(body, island_idx);
    }
    
    /// Merges two islands
    fn merge_islands(&mut self, island_a: usize, island_b: usize) {
        if island_a == island_b {
            return;
        }
        
        // Always merge the higher-indexed island into the lower-indexed one
        let (target, source) = if island_a < island_b {
            (island_a, island_b)
        } else {
            (island_b, island_a)
        };
        
        // Get the bodies from the source island
        let bodies_to_move: Vec<BodyHandle> = self.islands[source]
            .bodies
            .iter()
            .copied()
            .collect();
        
        // Get the constraints from the source island
        let constraints_to_move: Vec<ConstraintHandle> = self.islands[source]
            .constraints
            .iter()
            .copied()
            .collect();
        
        // Clone the source island to avoid borrow conflicts
        let source_island = self.islands[source].clone();

        // Update the target island
        self.islands[target].merge(&source_island);
        
        // Update the mappings
        for body in bodies_to_move {
            self.body_to_island.insert(body, target);
        }
        
        for constraint in constraints_to_move {
            self.constraint_to_island.insert(constraint, target);
        }
        
        // Clear the source island (but don't remove it to avoid reindexing)
        self.islands[source].clear();
    }
    
    /// Assigns a body to an island
    fn assign_body(&mut self, body: BodyHandle, island_idx: usize) {
        self.body_to_island.insert(body, island_idx);
        self.islands[island_idx].add_body(body);
    }
    
    /// Assigns a constraint to an island
    fn assign_constraint(&mut self, constraint: ConstraintHandle, island_idx: usize) {
        self.constraint_to_island.insert(constraint, island_idx);
        self.islands[island_idx].add_constraint(constraint);
    }
    
    /// Builds and returns the islands
    pub fn build(self) -> Vec<Island> {
        // Filter out empty islands
        self.islands
            .into_iter()
            .filter(|island| !island.is_empty())
            .collect()
    }
}