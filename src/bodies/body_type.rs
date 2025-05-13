/// Type of rigid body, determining how it behaves in the simulation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RigidBodyType {
    /// Dynamic bodies are fully simulated (affected by forces, collisions, etc.)
    Dynamic,
    
    /// Kinematic bodies are moved programmatically but affect dynamic bodies
    Kinematic,
    
    /// Static bodies don't move and aren't affected by forces or collisions
    Static,
}