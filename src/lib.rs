pub mod math;
pub mod core;
pub mod bodies;
pub mod shapes;
pub mod collision;
pub mod constraints;
pub mod forces;
pub mod integration;

/// Re-export common types for easier usage
pub use crate::core::{PhysicsWorld, SimulationConfig};
pub use crate::bodies::{RigidBody, RigidBodyType, RigidBodyHandle, Material};
pub use crate::math::Vector3;

/// Error types for the physics engine
pub mod error {
    use thiserror::Error;

    #[derive(Error, Debug)]
    pub enum PhysicsError {
        #[error("Invalid parameter: {0}")]
        InvalidParameter(String),

        #[error("Resource not found: {0}")]
        ResourceNotFound(String),

        #[error("Simulation stability error: {0}")]
        SimulationError(String),

        #[error("Internal error: {0}")]
        InternalError(String),
    }
}

/// Result type for physics engine operations
pub type Result<T> = std::result::Result<T, error::PhysicsError>;

/// Engine version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Library initialization and configuration
#[cfg(feature = "parallel")]
pub fn init() {
    // Initialize parallel processing resources if needed
}

/// Shut down the physics engine and clean up resources
#[cfg(feature = "parallel")]
pub fn shutdown() {
    // Clean up parallel processing resources if needed
}

#[cfg(test)]
mod tests {
    // Tests will be moved to dedicated test files
}
