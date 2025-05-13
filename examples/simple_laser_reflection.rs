use phys_engine::math::Vector3;

// ASCII art laser reflection simulation that doesn't require interactive terminal capabilities
// This version is non-interactive but shows a rotating laser with reflections

const WIDTH: usize = 80;
const HEIGHT: usize = 24;
const MAX_REFLECTIONS: usize = 5;
const NUM_FRAMES: usize = 60;  // Number of frames to simulate

// Represents a ray with origin and direction
struct Ray {
    origin: Vector3,
    direction: Vector3,
}

impl Ray {
    fn new(origin: Vector3, direction: Vector3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }
}

// Represents a laser beam segment with start and end points
struct Beam {
    start: Vector3,
    end: Vector3,
}

// Walls/reflective surfaces
#[derive(Clone)]
struct Wall {
    min: Vector3,
    max: Vector3,
    normal: Vector3,
}

fn main() {
    // Create walls (boundaries of the console)
    let walls = vec![
        // Left wall (x=0)
        Wall {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(0.0, HEIGHT as f32, 0.0),
            normal: Vector3::new(1.0, 0.0, 0.0),
        },
        // Right wall (x=WIDTH-1)
        Wall {
            min: Vector3::new(WIDTH as f32 - 1.0, 0.0, 0.0),
            max: Vector3::new(WIDTH as f32 - 1.0, HEIGHT as f32, 0.0),
            normal: Vector3::new(-1.0, 0.0, 0.0),
        },
        // Top wall (y=0)
        Wall {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(WIDTH as f32, 0.0, 0.0),
            normal: Vector3::new(0.0, 1.0, 0.0),
        },
        // Bottom wall (y=HEIGHT-1)
        Wall {
            min: Vector3::new(0.0, HEIGHT as f32 - 1.0, 0.0),
            max: Vector3::new(WIDTH as f32, HEIGHT as f32 - 1.0, 0.0),
            normal: Vector3::new(0.0, -1.0, 0.0),
        },
    ];
    
    // Create some obstacle walls for more interesting reflections
    let obstacle_walls = vec![
        // Diagonal wall in the middle
        Wall {
            min: Vector3::new(30.0, 8.0, 0.0),
            max: Vector3::new(50.0, 16.0, 0.0),
            normal: Vector3::new(-0.7071, -0.7071, 0.0),  // 45 degree angle
        },
        // Small horizontal wall
        Wall {
            min: Vector3::new(15.0, 12.0, 0.0),
            max: Vector3::new(25.0, 12.0, 0.0),
            normal: Vector3::new(0.0, -1.0, 0.0),
        },
        // Small vertical wall
        Wall {
            min: Vector3::new(60.0, 5.0, 0.0),
            max: Vector3::new(60.0, 15.0, 0.0),
            normal: Vector3::new(-1.0, 0.0, 0.0),
        },
    ];

    // Laser cannon position (center of the screen)
    let cannon_pos = Vector3::new(WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0, 0.0);
    
    // Run through predetermined angles to show rotating laser with reflections
    for frame in 0..NUM_FRAMES {
        // Calculate angle for this frame (rotating 360 degrees)
        let angle = (frame as f32 / NUM_FRAMES as f32) * 2.0 * std::f32::consts::PI;
        
        // Calculate direction from angle
        let direction = Vector3::new(angle.cos(), angle.sin(), 0.0);
        
        // Clear active beams
        let mut active_beams = Vec::new();
        
        // Fire laser and calculate reflections
        // Combine walls and obstacles
        let mut all_walls = walls.clone();
        all_walls.extend(obstacle_walls.clone());

        fire_laser(cannon_pos, direction, &all_walls, &mut active_beams);

        // Render the current frame
        render_frame(&active_beams, cannon_pos, cannon_pos + direction * 10.0);

        // In a more sophisticated implementation, we'd add a delay here
        // But for this simple version, we just print all frames
        
        // Add a separator between frames
        println!("\n{}\n", "-".repeat(WIDTH));
    }
}

// Fires a laser beam and calculates its reflections
fn fire_laser(
    origin: Vector3,
    direction: Vector3,
    walls: &[Wall],
    beams: &mut Vec<Beam>,
) {
    // Clear previous beams
    beams.clear();
    
    // Initial ray
    let mut current_ray = Ray::new(origin, direction);
    
    // Calculate up to MAX_REFLECTIONS reflections
    for _ in 0..MAX_REFLECTIONS {
        // Find closest intersection with any wall
        if let Some((hit_point, wall)) = find_closest_intersection(&current_ray, walls) {
            // Add beam segment from ray origin to hit point
            beams.push(Beam {
                start: current_ray.origin,
                end: hit_point,
            });
            
            // Calculate reflection
            let new_direction = reflect(current_ray.direction, wall.normal);
            
            // Update ray for next reflection (offset slightly to avoid self-intersection)
            current_ray = Ray::new(hit_point + new_direction * 0.001, new_direction);
        } else {
            // No intersection, extend beam to edge of screen
            let end_point = extend_ray_to_edge(&current_ray);
            beams.push(Beam {
                start: current_ray.origin,
                end: end_point,
            });
            break;
        }
    }
}

// Find the closest intersection between a ray and any wall
fn find_closest_intersection<'a>(ray: &Ray, walls: &'a [Wall]) -> Option<(Vector3, &'a Wall)> {
    let mut closest_distance = f32::MAX;
    let mut closest_hit = None;
    let mut closest_wall = None;

    for wall in walls {
        if let Some((hit_point, distance)) = ray_wall_intersection(ray, wall) {
            if distance < closest_distance {
                closest_distance = distance;
                closest_hit = Some(hit_point);
                closest_wall = Some(wall);
            }
        }
    }

    if let (Some(hit), Some(wall)) = (closest_hit, closest_wall) {
        Some((hit, wall))
    } else {
        None
    }
}

// Calculate intersection between a ray and a wall
fn ray_wall_intersection(ray: &Ray, wall: &Wall) -> Option<(Vector3, f32)> {
    // Calculate dot product of ray direction and wall normal
    let dot = ray.direction.dot(&wall.normal);
    
    // No intersection if ray is parallel to wall or pointing away from it
    if dot.abs() < 0.0001 {
        return None;
    }
    
    // For simplicity, we'll handle specific wall orientations
    // For walls with strong x-component in normal
    if wall.normal.x.abs() > 0.7 {
        let x = wall.min.x;
        let t = (x - ray.origin.x) / ray.direction.x;
        
        // Intersection is behind the ray
        if t < 0.0 {
            return None;
        }
        
        let hit_y = ray.origin.y + t * ray.direction.y;
        
        // For diagonal walls, we need to check differently
        if wall.normal.y.abs() > 0.3 {
            // Simple line equation: y = mx + b
            // For our diagonal wall, we'll simplify with a basic check
            let min_y = wall.min.y.min(wall.max.y);
            let max_y = wall.min.y.max(wall.max.y);
            
            if hit_y >= min_y && hit_y <= max_y {
                let hit_point = Vector3::new(x, hit_y, 0.0);
                let distance = (hit_point - ray.origin).length();
                return Some((hit_point, distance));
            }
        } else {
            // Regular vertical wall check
            if hit_y >= wall.min.y && hit_y <= wall.max.y {
                let hit_point = Vector3::new(x, hit_y, 0.0);
                let distance = (hit_point - ray.origin).length();
                return Some((hit_point, distance));
            }
        }
    }
    // For walls with strong y-component in normal
    else if wall.normal.y.abs() > 0.7 {
        let y = wall.min.y;
        let t = (y - ray.origin.y) / ray.direction.y;
        
        // Intersection is behind the ray
        if t < 0.0 {
            return None;
        }
        
        let hit_x = ray.origin.x + t * ray.direction.x;
        
        // Check if hit point is within wall bounds
        if hit_x >= wall.min.x && hit_x <= wall.max.x {
            let hit_point = Vector3::new(hit_x, y, 0.0);
            let distance = (hit_point - ray.origin).length();
            return Some((hit_point, distance));
        }
    }
    // For diagonal walls (with mixed x and y components)
    else {
        // We'll use a simplified approach for diagonal walls
        // Using parametric plane equation
        let point_on_wall = wall.min;
        let t = (wall.normal.dot(&(point_on_wall - ray.origin))) / dot;
        
        if t < 0.0 {
            return None;
        }
        
        // Calculate intersection point
        let hit_point = ray.origin + ray.direction * t;
        
        // Check if the point is within the wall bounds
        // For diagonal wall, we'll use a simplified bounding box check
        if hit_point.x >= wall.min.x.min(wall.max.x) && 
           hit_point.x <= wall.min.x.max(wall.max.x) && 
           hit_point.y >= wall.min.y.min(wall.max.y) && 
           hit_point.y <= wall.min.y.max(wall.max.y) {
            let distance = (hit_point - ray.origin).length();
            return Some((hit_point, distance));
        }
    }
    
    None
}

// Calculate reflection of a vector off a surface with given normal
fn reflect(incident: Vector3, normal: Vector3) -> Vector3 {
    // r = i - 2(i·n)n
    incident - normal * 2.0 * incident.dot(&normal)
}

// Extend a ray to the edge of the screen (for rays that don't hit any walls)
fn extend_ray_to_edge(ray: &Ray) -> Vector3 {
    let mut t_values = Vec::new();
    
    // Calculate intersections with screen edges
    if ray.direction.x.abs() > 0.0001 {
        // Left edge
        let t_left = -ray.origin.x / ray.direction.x;
        if t_left > 0.0 {
            t_values.push(t_left);
        }
        
        // Right edge
        let t_right = (WIDTH as f32 - ray.origin.x) / ray.direction.x;
        if t_right > 0.0 {
            t_values.push(t_right);
        }
    }
    
    if ray.direction.y.abs() > 0.0001 {
        // Top edge
        let t_top = -ray.origin.y / ray.direction.y;
        if t_top > 0.0 {
            t_values.push(t_top);
        }
        
        // Bottom edge
        let t_bottom = (HEIGHT as f32 - ray.origin.y) / ray.direction.y;
        if t_bottom > 0.0 {
            t_values.push(t_bottom);
        }
    }
    
    // Use the smallest positive t value to find the edge intersection
    if let Some(&t) = t_values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()) {
        ray.origin + ray.direction * t
    } else {
        // Fallback
        ray.origin + ray.direction * 100.0
    }
}

// Render the current frame
fn render_frame(beams: &[Beam], cannon_pos: Vector3, _target_pos: Vector3) {
    // Create empty grid
    let mut grid = vec![vec![' '; WIDTH]; HEIGHT];
    
    // Draw walls (screen boundaries)
    for x in 0..WIDTH {
        grid[0][x] = '-';
        grid[HEIGHT - 1][x] = '-';
    }
    for y in 0..HEIGHT {
        grid[y][0] = '|';
        grid[y][WIDTH - 1] = '|';
    }
    
    // Draw obstacle walls
    for x in 30..50 {
        let y = 8 + (x - 30) / 2;
        if y < HEIGHT {
            grid[y][x] = '\\';
        }
    }
    
    for x in 15..25 {
        if 12 < HEIGHT {
            grid[12][x] = '=';
        }
    }
    
    for y in 5..15 {
        if 60 < WIDTH {
            grid[y][60] = '|';
        }
    }
    
    // Draw laser cannon
    let cannon_x = cannon_pos.x.round() as usize;
    let cannon_y = cannon_pos.y.round() as usize;
    if cannon_x < WIDTH && cannon_y < HEIGHT {
        grid[cannon_y][cannon_x] = '◎'; // Cannon symbol
    }
    
    // Draw laser beams using simplified line drawing
    for beam in beams {
        draw_line(&mut grid, beam);
    }
    
    // Print the grid
    for row in &grid {
        println!("{}", row.iter().collect::<String>());
    }
}

// Draw a line using simplified Bresenham's algorithm
fn draw_line(grid: &mut Vec<Vec<char>>, beam: &Beam) {
    let x0 = beam.start.x.round() as i32;
    let y0 = beam.start.y.round() as i32;
    let x1 = beam.end.x.round() as i32;
    let y1 = beam.end.y.round() as i32;
    
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    
    let mut x = x0;
    let mut y = y0;
    
    // Select character based on beam direction
    let beam_dir = (beam.end - beam.start).normalize();
    let char_to_use = if beam_dir.x.abs() > beam_dir.y.abs() {
        if beam_dir.x.abs() > 0.7 { '─' } else { '/' }
    } else {
        if beam_dir.y.abs() > 0.7 { '│' } else { '\\' }
    };
    
    loop {
        if x >= 0 && y >= 0 && x < WIDTH as i32 && y < HEIGHT as i32 {
            let grid_char = &mut grid[y as usize][x as usize];
            
            // Only overwrite empty spaces or previous beam characters
            if *grid_char == ' ' || *grid_char == '─' || *grid_char == '│' || 
               *grid_char == '\\' || *grid_char == '/' {
                *grid_char = char_to_use;
            }
        }
        
        if x == x1 && y == y1 {
            break;
        }
        
        let e2 = 2 * err;
        if e2 >= dy {
            if x == x1 {
                break;
            }
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            if y == y1 {
                break;
            }
            err += dx;
            y += sy;
        }
    }
}