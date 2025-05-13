use crate::forces::ForceGenerator;
use crate::core::{BodyHandle, BodyStorage};
use crate::bodies::RigidBody;
use crate::math::Vector3;
use std::any::Any;

/// A force generator that simulates buoyancy forces in fluids
#[derive(Debug, Clone)]
pub struct BuoyancyForce {
    /// The body affected by buoyancy
    body: BodyHandle,
    
    /// The maximum submersion depth
    max_depth: f32,
    
    /// The volume of the object
    volume: f32,
    
    /// The height of the water plane
    water_height: f32,
    
    /// The density of the fluid
    fluid_density: f32,
    
    /// The center of buoyancy in local space
    center_of_buoyancy: Vector3,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// The bodies affected by this force generator (cached for quick lookup)
    affected_bodies: Vec<BodyHandle>,
}

impl BuoyancyForce {
    /// Creates a new buoyancy force generator
    pub fn new(
        body: BodyHandle,
        max_depth: f32,
        volume: f32,
        water_height: f32,
        fluid_density: f32,
        center_of_buoyancy: Vector3,
    ) -> Self {
        Self {
            body,
            max_depth: max_depth.max(0.0),
            volume: volume.max(0.0),
            water_height,
            fluid_density: fluid_density.max(0.0),
            center_of_buoyancy,
            enabled: true,
            affected_bodies: vec![body],
        }
    }
    
    /// Creates a new buoyancy force generator for water
    pub fn new_for_water(
        body: BodyHandle,
        max_depth: f32,
        volume: f32,
        water_height: f32,
        center_of_buoyancy: Vector3,
    ) -> Self {
        // Water density is approximately 1000 kg/m^3
        Self::new(body, max_depth, volume, water_height, 1000.0, center_of_buoyancy)
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Gets the maximum submersion depth
    pub fn get_max_depth(&self) -> f32 {
        self.max_depth
    }
    
    /// Sets the maximum submersion depth
    pub fn set_max_depth(&mut self, depth: f32) {
        self.max_depth = depth.max(0.0);
    }
    
    /// Gets the volume of the object
    pub fn get_volume(&self) -> f32 {
        self.volume
    }
    
    /// Sets the volume of the object
    pub fn set_volume(&mut self, volume: f32) {
        self.volume = volume.max(0.0);
    }
    
    /// Gets the height of the water plane
    pub fn get_water_height(&self) -> f32 {
        self.water_height
    }
    
    /// Sets the height of the water plane
    pub fn set_water_height(&mut self, height: f32) {
        self.water_height = height;
    }
    
    /// Gets the density of the fluid
    pub fn get_fluid_density(&self) -> f32 {
        self.fluid_density
    }
    
    /// Sets the density of the fluid
    pub fn set_fluid_density(&mut self, density: f32) {
        self.fluid_density = density.max(0.0);
    }
    
    /// Gets the center of buoyancy in local space
    pub fn get_center_of_buoyancy(&self) -> Vector3 {
        self.center_of_buoyancy
    }
    
    /// Sets the center of buoyancy in local space
    pub fn set_center_of_buoyancy(&mut self, center: Vector3) {
        self.center_of_buoyancy = center;
    }
}

impl ForceGenerator for BuoyancyForce {
    fn generator_type(&self) -> &'static str {
        "Buoyancy"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled {
            return;
        }
        
        let body = match bodies.get_body_mut(self.body) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        // Get the position of the center of buoyancy in world space
        let center_of_buoyancy_world = body.get_transform().transform_point(self.center_of_buoyancy);
        
        // Calculate the depth of the center of buoyancy
        let depth = center_of_buoyancy_world.y - self.water_height;
        
        // If the object is completely above water, no buoyancy force
        if depth >= 0.0 {
            return;
        }
        
        let force = Vector3::new(0.0, 0.0, 0.0);
        
        // If the object is completely underwater, full buoyancy force
        if depth <= -self.max_depth {
            // Archimedes' principle: Buoyancy force = fluid density * volume * gravity
            // Assuming gravity is in the negative y direction (0, -9.81, 0)
            let force_magnitude = self.fluid_density * self.volume * 9.81;
            let force = Vector3::new(0.0, force_magnitude, 0.0);
            
            // Apply the buoyancy force at the center of buoyancy
            body.apply_force_at_point(force, center_of_buoyancy_world);
            return;
        }
        
        // Object is partially submerged
        // Calculate the fraction of the object that is submerged
        let submerged_fraction = -depth / self.max_depth;
        
        // Calculate buoyancy force
        let force_magnitude = self.fluid_density * self.volume * 9.81 * submerged_fraction;
        let force = Vector3::new(0.0, force_magnitude, 0.0);
        
        // Apply the buoyancy force at the center of buoyancy
        body.apply_force_at_point(force, center_of_buoyancy_world);
    }
    
    fn get_affected_bodies(&self) -> &[BodyHandle] {
        &self.affected_bodies
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_generator(&self) -> Box<dyn ForceGenerator> {
        Box::new(self.clone())
    }
}

/// A more complex buoyancy force generator that simulates buoyancy using multiple buoyancy planes
#[derive(Debug, Clone)]
pub struct ComplexBuoyancyForce {
    /// The body affected by buoyancy
    body: BodyHandle,
    
    /// The density of the fluid
    fluid_density: f32,
    
    /// The buoyancy planes (defines the water surface)
    buoyancy_planes: Vec<BuoyancyPlane>,
    
    /// Whether the force generator is enabled
    enabled: bool,
    
    /// The bodies affected by this force generator (cached for quick lookup)
    affected_bodies: Vec<BodyHandle>,
}

/// A plane that defines a boundary of a fluid for buoyancy calculations
#[derive(Debug, Clone, Copy)]
pub struct BuoyancyPlane {
    /// The normal of the plane pointing out of the fluid
    normal: Vector3,
    
    /// The offset of the plane from the origin
    offset: f32,
}

impl BuoyancyPlane {
    /// Creates a new buoyancy plane
    pub fn new(normal: Vector3, offset: f32) -> Self {
        Self {
            normal: normal.normalize(),
            offset,
        }
    }
    
    /// Creates a horizontal plane at the specified height
    pub fn horizontal(height: f32) -> Self {
        Self::new(Vector3::new(0.0, 1.0, 0.0), height)
    }
    
    /// Returns the distance from a point to the plane
    /// Positive distance means the point is on the side the normal points to (outside fluid)
    /// Negative distance means the point is on the opposite side (inside fluid)
    pub fn distance_to_point(&self, point: Vector3) -> f32 {
        self.normal.dot(&point) - self.offset
    }
    
    /// Returns whether a point is inside the fluid
    pub fn is_point_submerged(&self, point: Vector3) -> bool {
        self.distance_to_point(point) < 0.0
    }
}

impl ComplexBuoyancyForce {
    /// Creates a new complex buoyancy force generator
    pub fn new(
        body: BodyHandle,
        fluid_density: f32,
    ) -> Self {
        Self {
            body,
            fluid_density: fluid_density.max(0.0),
            buoyancy_planes: Vec::new(),
            enabled: true,
            affected_bodies: vec![body],
        }
    }
    
    /// Creates a new complex buoyancy force generator for water
    pub fn new_for_water(body: BodyHandle) -> Self {
        // Water density is approximately 1000 kg/m^3
        Self::new(body, 1000.0)
    }
    
    /// Returns whether the force generator is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    /// Sets whether the force generator is enabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
    
    /// Gets the density of the fluid
    pub fn get_fluid_density(&self) -> f32 {
        self.fluid_density
    }
    
    /// Sets the density of the fluid
    pub fn set_fluid_density(&mut self, density: f32) {
        self.fluid_density = density.max(0.0);
    }
    
    /// Adds a buoyancy plane
    pub fn add_plane(&mut self, plane: BuoyancyPlane) {
        self.buoyancy_planes.push(plane);
    }
    
    /// Adds a horizontal buoyancy plane at the specified height
    pub fn add_horizontal_plane(&mut self, height: f32) {
        self.buoyancy_planes.push(BuoyancyPlane::horizontal(height));
    }
    
    /// Clears all buoyancy planes
    pub fn clear_planes(&mut self) {
        self.buoyancy_planes.clear();
    }
    
    /// Gets the number of buoyancy planes
    pub fn num_planes(&self) -> usize {
        self.buoyancy_planes.len()
    }
}

impl ForceGenerator for ComplexBuoyancyForce {
    fn generator_type(&self) -> &'static str {
        "ComplexBuoyancy"
    }
    
    fn update(&mut self, _dt: f32) {
        // Nothing to update
    }
    
    fn apply_forces(&self, bodies: &mut BodyStorage<RigidBody>, _dt: f32) {
        if !self.enabled || self.buoyancy_planes.is_empty() {
            return;
        }
        
        let body = match bodies.get_body_mut(self.body) {
            Ok(body) => body,
            Err(_) => return,
        };
        
        // For complex buoyancy, we try to approximate the shape using test points
        // Here we use a simple approach with 8 corner points of the bounding box
        // but in a real simulation, you'd want to use points that better represent the shape
        
        // Get the shape's bounding box in local space
        let shape = match body.get_shape() {
            Some(shape) => shape,
            None => return, // No shape, no buoyancy
        };
        let local_aabb = shape.as_ref().get_local_bounds();
        
        // Get the corners of the bounding box in local space
        let half_extents = (local_aabb.max - local_aabb.min) * 0.5;
        let center = (local_aabb.max + local_aabb.min) * 0.5;
        
        let corners = [
            center + Vector3::new(-half_extents.x, -half_extents.y, -half_extents.z),
            center + Vector3::new(-half_extents.x, -half_extents.y,  half_extents.z),
            center + Vector3::new(-half_extents.x,  half_extents.y, -half_extents.z),
            center + Vector3::new(-half_extents.x,  half_extents.y,  half_extents.z),
            center + Vector3::new( half_extents.x, -half_extents.y, -half_extents.z),
            center + Vector3::new( half_extents.x, -half_extents.y,  half_extents.z),
            center + Vector3::new( half_extents.x,  half_extents.y, -half_extents.z),
            center + Vector3::new( half_extents.x,  half_extents.y,  half_extents.z),
        ];
        
        // Transform corners to world space
        let transform = body.get_transform();
        let world_corners: Vec<Vector3> = corners.iter().map(|&corner| transform.transform_point(corner)).collect();
        
        // Count how many corners are submerged
        let mut submerged_count = 0;
        for &corner in &world_corners {
            // A corner is submerged if it's inside all of the fluid boundaries
            let mut is_submerged = true;
            for plane in &self.buoyancy_planes {
                if !plane.is_point_submerged(corner) {
                    is_submerged = false;
                    break;
                }
            }
            
            if is_submerged {
                submerged_count += 1;
            }
        }
        
        // If no corners are submerged, no buoyancy force
        if submerged_count == 0 {
            return;
        }
        
        // Simple model: force is proportional to proportion of corners submerged
        let submerged_fraction = submerged_count as f32 / world_corners.len() as f32;
        
        // Estimate volume based on bounding box
        let volume = 8.0 * half_extents.x * half_extents.y * half_extents.z;
        
        // Calculate buoyancy force using Archimedes' principle
        // Assuming gravity is in the negative y direction (0, -9.81, 0)
        let force_magnitude = self.fluid_density * volume * 9.81 * submerged_fraction;
        
        // The buoyancy force acts upward against gravity
        let force = Vector3::new(0.0, force_magnitude, 0.0);
        
        // Calculate the center of buoyancy (average of submerged corners)
        if submerged_count > 0 {
            let mut center_of_buoyancy = Vector3::zero();
            let mut total_count = 0;
            
            for &corner in &world_corners {
                // Check if corner is submerged
                let mut is_submerged = true;
                for plane in &self.buoyancy_planes {
                    if !plane.is_point_submerged(corner) {
                        is_submerged = false;
                        break;
                    }
                }
                
                if is_submerged {
                    center_of_buoyancy += corner;
                    total_count += 1;
                }
            }
            
            // Calculate average position
            if total_count > 0 {
                center_of_buoyancy /= total_count as f32;
                
                // Apply the buoyancy force at the center of buoyancy
                body.apply_force_at_point(force, center_of_buoyancy);
            } else {
                // Fallback: apply at center of mass
                body.apply_force(force);
            }
        } else {
            // Fallback: apply at center of mass
            body.apply_force(force);
        }
    }
    
    fn get_affected_bodies(&self) -> &[BodyHandle] {
        &self.affected_bodies
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    
    fn clone_generator(&self) -> Box<dyn ForceGenerator> {
        Box::new(self.clone())
    }
}