use phys_engine::{
    PhysicsWorld, RigidBody, Material,
    shapes::{Sphere, box_shape::BoxShape, Shape},
    math::{Vector3, Ray, Transform},
};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;
use std::io::{stdout, Write};
use rand::Rng;

// Terminal size detection
fn get_terminal_size() -> (usize, usize) {
    // Default size if terminal size detection fails
    let default_size = (80, 24);
    
    match term_size::dimensions() {
        Some((w, h)) => (w, h),
        None => default_size,
    }
}

// Characters for depth visualization (from nearest to farthest)
const DEPTH_CHARS: [char; 10] = ['@', '#', '$', 'O', '8', 'o', '*', '.', ',', ' '];

// Structure to represent a ray hit
struct RayHit {
    distance: f32,
    position: Vector3,
    normal: Vector3,
    material_id: usize, // To track different materials
}

// Simplified ray tracer for ASCII visualization
struct AsciiRayTracer {
    width: usize,
    height: usize,
    aspect_ratio: f32,
    bodies: Vec<(RigidBody, usize)>, // body and material id
    camera_pos: Vector3,
    camera_direction: Vector3,
    camera_up: Vector3,
    fov: f32,
    max_distance: f32,
}

impl AsciiRayTracer {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            aspect_ratio: width as f32 / height as f32,
            bodies: Vec::new(),
            camera_pos: Vector3::new(0.0, 0.0, -20.0),
            camera_direction: Vector3::new(0.0, 0.0, 1.0).normalize(),
            camera_up: Vector3::new(0.0, 1.0, 0.0).normalize(),
            fov: 60.0 * std::f32::consts::PI / 180.0, // 60 degrees in radians
            max_distance: 1000.0,
        }
    }
    
    fn add_body(&mut self, body: RigidBody, material_id: usize) {
        self.bodies.push((body, material_id));
    }
    
    fn set_camera(&mut self, position: Vector3, target: Vector3, up: Vector3) {
        self.camera_pos = position;
        self.camera_direction = (target - position).normalize();
        self.camera_up = up.normalize();
    }
    
    fn cast_ray(&self, ray: &Ray) -> Option<RayHit> {
        let mut closest_hit: Option<RayHit> = None;
        let mut closest_distance = self.max_distance;
        
        for (body, material_id) in &self.bodies {
            // Only process bodies with shapes
            if let Some(shape) = body.get_shape() {
                let transform = body.get_transform();
                
                // Check ray intersection with the shape
                if let Some(distance) = shape.intersects_ray(ray, &transform, self.max_distance) {
                    if distance < closest_distance {
                        closest_distance = distance;
                        
                        // Calculate hit position
                        let hit_pos = ray.point_at(distance);
                        
                        // Calculate normal - simplified approximation
                        let normal = (hit_pos - transform.position).normalize();
                        
                        closest_hit = Some(RayHit {
                            distance,
                            position: hit_pos,
                            normal,
                            material_id: *material_id,
                        });
                    }
                }
            }
        }
        
        closest_hit
    }
    
    fn render_frame(&self) -> Vec<Vec<char>> {
        let mut frame = vec![vec![' '; self.width]; self.height];
        
        // Calculate camera basis vectors
        let camera_right = self.camera_direction.cross(&self.camera_up).normalize();
        let camera_up = camera_right.cross(&self.camera_direction).normalize();
        
        // Calculate screen dimensions based on FOV
        let screen_height = 2.0 * (self.fov / 2.0).tan();
        let screen_width = screen_height * self.aspect_ratio;
        
        // Cast rays for each pixel
        for y in 0..self.height {
            for x in 0..self.width {
                // Convert pixel coordinates to normalized device coordinates
                let ndc_x = (x as f32 / self.width as f32) * 2.0 - 1.0;
                let ndc_y = 1.0 - (y as f32 / self.height as f32) * 2.0; // Flip y
                
                // Calculate ray direction
                let pixel_camera_pos = self.camera_pos + self.camera_direction; // 1 unit in front of camera
                let pixel_pos = pixel_camera_pos 
                    + camera_right * (ndc_x * screen_width / 2.0)
                    + camera_up * (ndc_y * screen_height / 2.0);
                
                let ray_dir = (pixel_pos - self.camera_pos).normalize();
                let ray = Ray::new(self.camera_pos, ray_dir);
                
                // Cast ray and determine character based on hit
                if let Some(hit) = self.cast_ray(&ray) {
                    // Normalize distance to get a depth index
                    let normalized_distance = (hit.distance / self.max_distance).clamp(0.0, 1.0);
                    let depth_index = (normalized_distance * (DEPTH_CHARS.len() - 1) as f32) as usize;
                    
                    // Get character based on depth
                    let char_to_use = DEPTH_CHARS[depth_index];
                    
                    // Store in frame buffer
                    frame[y][x] = char_to_use;
                }
            }
        }
        
        frame
    }
    
    fn render_to_console(&self) {
        let frame = self.render_frame();
        let mut stdout = stdout();
        
        // Clear screen with ANSI escape code
        print!("\x1B[2J\x1B[1;1H");
        
        // Draw frame
        for row in &frame {
            for &c in row {
                print!("{}", c);
            }
            println!();
        }
        
        // Flush output
        stdout.flush().unwrap();
    }
}

// Main function
fn main() {
    let (term_width, term_height) = get_terminal_size();
    
    // Create raytracer with terminal dimensions
    let mut raytracer = AsciiRayTracer::new(term_width, term_height - 2); // Leave space for info line
    
    // Set camera position and target
    raytracer.set_camera(
        Vector3::new(0.0, 0.0, -15.0), // Position
        Vector3::new(0.0, 0.0, 0.0),   // Target (looking at origin)
        Vector3::new(0.0, 1.0, 0.0),   // Up vector
    );
    
    // Create spheres and boxes in a nice arrangement
    let mut rng = rand::thread_rng();
    
    // Create a ground plane (large flat box)
    let ground_shape = Arc::new(BoxShape::new_with_dimensions(40.0, 1.0, 40.0));
    let mut ground = RigidBody::new_static(ground_shape, Vector3::new(0.0, -6.0, 0.0));
    ground.set_material(Material::new(1000.0, 0.8, 0.5));
    raytracer.add_body(ground, 0); // Ground material
    
    // Add some spheres
    for i in 0..8 {
        let radius = rng.gen_range(0.8..2.0);
        let x = rng.gen_range(-8.0..8.0);
        let y = rng.gen_range(-5.0..5.0);
        let z = rng.gen_range(-5.0..10.0);
        
        let sphere_shape = Arc::new(Sphere::new(radius));
        let mut sphere = RigidBody::new_dynamic(sphere_shape, Vector3::new(x, y, z));
        sphere.set_material(Material::new(1.0, 0.5, 0.8));
        
        // Assign a random material ID for variety (1-3)
        let material_id = 1 + (i % 3);
        raytracer.add_body(sphere, material_id);
    }
    
    // Add some boxes
    for i in 0..5 {
        let size_x = rng.gen_range(1.0..3.0);
        let size_y = rng.gen_range(1.0..3.0);
        let size_z = rng.gen_range(1.0..3.0);
        
        let x = rng.gen_range(-8.0..8.0);
        let y = rng.gen_range(-5.0..5.0);
        let z = rng.gen_range(-5.0..10.0);
        
        let box_shape = Arc::new(BoxShape::new_with_dimensions(size_x, size_y, size_z));
        let mut box_body = RigidBody::new_dynamic(box_shape, Vector3::new(x, y, z));
        box_body.set_material(Material::new(1.0, 0.5, 0.3));
        
        // Assign a random material ID for variety (4-6)
        let material_id = 4 + (i % 3);
        raytracer.add_body(box_body, material_id);
    }
    
    // Animation loop - move the camera in a circle around the scene
    let total_frames = 360;
    let radius = 20.0;
    
    for frame in 0..total_frames {
        // Calculate camera position on a circle
        let angle = (frame as f32 / total_frames as f32) * 2.0 * std::f32::consts::PI;
        let camera_x = radius * angle.cos();
        let camera_z = radius * angle.sin() - 5.0;
        
        // Update camera
        raytracer.set_camera(
            Vector3::new(camera_x, 2.0, camera_z),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        );
        
        // Render the current frame
        raytracer.render_to_console();
        
        // Show frame information
        println!("Frame: {}/{} - Camera Position: ({:.1}, {:.1}, {:.1})", 
                 frame + 1, total_frames, camera_x, 2.0, camera_z);
        println!("Press Ctrl+C to exit");
        
        // Wait before next frame
        sleep(Duration::from_millis(50));
    }
}