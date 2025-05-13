/// Material properties for physics objects
#[derive(Debug, Clone, Copy)]
pub struct Material {
    /// Coefficient of restitution (bounciness), 0-1
    pub restitution: f32,
    
    /// Coefficient of friction, 0-1
    pub friction: f32,
    
    /// Density of the material (kg/m^3)
    pub density: f32,
    
    /// Rolling friction coefficient
    pub rolling_friction: f32,
}

impl Material {
    /// Creates a new material with the specified properties
    pub fn new(density: f32, friction: f32, restitution: f32) -> Self {
        Self {
            density,
            friction,
            restitution,
            rolling_friction: 0.0,
        }
    }
    
    /// Creates a new material with the specified properties, including rolling friction
    pub fn new_with_rolling(density: f32, friction: f32, restitution: f32, rolling_friction: f32) -> Self {
        Self {
            density,
            friction,
            restitution,
            rolling_friction,
        }
    }
    
    /// Creates a material for ice (low friction, high restitution)
    pub fn ice() -> Self {
        Self {
            density: 900.0,
            friction: 0.05,
            restitution: 0.4,
            rolling_friction: 0.01,
        }
    }
    
    /// Creates a material for rubber (medium friction, medium restitution)
    pub fn rubber() -> Self {
        Self {
            density: 1200.0,
            friction: 0.8,
            restitution: 0.7,
            rolling_friction: 0.2,
        }
    }
    
    /// Creates a material for wood (medium friction, low restitution)
    pub fn wood() -> Self {
        Self {
            density: 700.0,
            friction: 0.6,
            restitution: 0.2,
            rolling_friction: 0.05,
        }
    }
    
    /// Creates a material for metal (medium friction, medium restitution)
    pub fn metal() -> Self {
        Self {
            density: 7800.0,
            friction: 0.4,
            restitution: 0.5,
            rolling_friction: 0.02,
        }
    }
    
    /// Creates a material for concrete (high friction, low restitution)
    pub fn concrete() -> Self {
        Self {
            density: 2400.0,
            friction: 0.9,
            restitution: 0.1,
            rolling_friction: 0.4,
        }
    }
}

impl Default for Material {
    fn default() -> Self {
        Self {
            density: 1000.0,  // Density of water (kg/m^3)
            friction: 0.5,    // Medium friction
            restitution: 0.3, // Slight bounce
            rolling_friction: 0.1,
        }
    }
}