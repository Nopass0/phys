use crate::core::GravityType;

#[cfg(feature = "serialize")]
use serde::{Serialize, Deserialize};

/// Configuration parameters for the physics simulation
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct SimulationConfig {
    /// The fixed time step for the simulation
    pub time_step: f32,
    
    /// The maximum number of substeps for variable time step simulation
    pub max_substeps: u32,
    
    /// The type of gravity in the simulation
    pub gravity: GravityType,
    
    /// The number of iterations to run for solving position constraints
    pub position_iterations: u32,
    
    /// The number of iterations to run for solving velocity constraints
    pub velocity_iterations: u32,
    
    /// Whether to use continuous collision detection
    pub use_ccd: bool,
    
    /// The linear velocity threshold below which bodies can sleep
    pub linear_sleep_threshold: f32,
    
    /// The angular velocity threshold below which bodies can sleep
    pub angular_sleep_threshold: f32,
    
    /// The time a body must be inactive before sleeping
    pub sleep_time_threshold: f32,
    
    /// Whether to allow sleeping bodies
    pub allow_sleeping: bool,
    
    /// Global damping factor for linear velocity
    pub linear_damping: f32,
    
    /// Global damping factor for angular velocity
    pub angular_damping: f32,
    
    /// The bias factor for constraint solving (Baumgarte stabilization)
    pub constraint_bias_factor: f32,
    
    /// The collision margin for shape detection
    pub collision_margin: f32,
    
    /// The factor for constraint force mixing (CFM)
    pub cfm_factor: f32,
    
    /// The factor for error reduction parameter (ERP)
    pub erp_factor: f32,
    
    /// Maximum allowed penetration before position correction
    pub contact_penetration_threshold: f32,
    
    /// The collision restitution velocity threshold
    pub restitution_velocity_threshold: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            time_step: 1.0 / 60.0,
            max_substeps: 10,
            gravity: GravityType::default(),
            position_iterations: 10,
            velocity_iterations: 8,
            use_ccd: false,
            linear_sleep_threshold: 0.001, // Reduced threshold for more accurate simulation
            angular_sleep_threshold: 0.001, // Reduced threshold for more accurate simulation
            sleep_time_threshold: 1.0, // Increased time before sleeping
            allow_sleeping: true,
            linear_damping: 0.0, // Reduced damping for more accurate test simulation
            angular_damping: 0.0, // Reduced damping for more accurate test simulation
            constraint_bias_factor: 0.2,
            collision_margin: 0.01,
            cfm_factor: 0.0,
            erp_factor: 0.2,
            contact_penetration_threshold: 0.005,
            restitution_velocity_threshold: 0.5,
        }
    }
}