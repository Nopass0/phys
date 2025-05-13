use crossterm::{
    cursor::{self, Hide, Show},
    event::{self, Event, KeyCode, KeyEvent, MouseButton, MouseEvent, MouseEventKind},
    execute,
    style::{Color, SetForegroundColor},
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::io::Result;
use phys_engine::math::Vector3;
use std::io::{stdout, Write};
use std::time::{Duration, Instant};

// ASCII art laser reflection simulation
// The laser will follow the mouse cursor and shoot when clicked
// Laser beams will reflect off walls up to 5 times

const WIDTH: usize = 80;
const HEIGHT: usize = 24;
const MAX_REFLECTIONS: usize = 5;

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
    color: Color,
}

// Walls/reflective surfaces
struct Wall {
    min: Vector3,
    max: Vector3,
    normal: Vector3,
}

fn main() -> Result<()> {
    // Set up terminal
    let mut stdout = stdout();
    execute!(
        stdout,
        EnterAlternateScreen,
        Hide,
        terminal::SetTitle("ASCII Laser Reflection")
    )?;
    terminal::enable_raw_mode()?;

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

    // State variables
    let mut mouse_pos = Vector3::new(WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0, 0.0);
    let cannon_pos = Vector3::new(WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0, 0.0);
    let mut active_beams: Vec<Beam> = Vec::new();
    let mut firing = false;
    let mut last_frame_time = Instant::now();
    let mut is_running = true;

    // Laser colors for different reflection levels
    let beam_colors = [
        Color::Red,
        Color::Yellow,
        Color::Green,
        Color::Cyan,
        Color::Magenta,
        Color::Blue,
    ];

    // Main loop
    while is_running {
        // Frame timing
        let current_time = Instant::now();
        let _delta_time = current_time.duration_since(last_frame_time);
        last_frame_time = current_time;

        // Handle events
        if event::poll(Duration::from_millis(1))? {
            match event::read()? {
                Event::Key(KeyEvent { code, .. }) => {
                    if code == KeyCode::Char('q') || code == KeyCode::Esc {
                        is_running = false;
                    }
                    if code == KeyCode::Char(' ') {
                        firing = true;
                    }
                }
                Event::Mouse(MouseEvent { kind, column, row, .. }) => {
                    mouse_pos = Vector3::new(column as f32, row as f32, 0.0);
                    
                    if let MouseEventKind::Down(MouseButton::Left) = kind {
                        firing = true;
                    }
                }
                _ => {}
            }
        }

        // Update laser direction based on mouse position
        let direction = (mouse_pos - cannon_pos).normalize();

        // Fire laser if trigger is active
        if firing {
            fire_laser(cannon_pos, direction, &walls, &beam_colors, &mut active_beams);
            firing = false;
        }

        // Render frame
        render_frame(&mut stdout, &active_beams, cannon_pos, mouse_pos)?;

        // Limit frame rate
        let frame_duration = current_time.elapsed();
        if frame_duration < Duration::from_millis(33) {
            std::thread::sleep(Duration::from_millis(33) - frame_duration);
        }
    }

    // Clean up
    execute!(stdout, Show, LeaveAlternateScreen)?;
    terminal::disable_raw_mode()?;
    Ok(())
}

// Fires a laser beam and calculates its reflections
fn fire_laser(
    origin: Vector3,
    direction: Vector3,
    walls: &[Wall],
    colors: &[Color],
    beams: &mut Vec<Beam>,
) {
    // Clear previous beams
    beams.clear();
    
    // Initial ray
    let mut current_ray = Ray::new(origin, direction);
    
    // Calculate up to MAX_REFLECTIONS reflections
    for reflection in 0..=MAX_REFLECTIONS {
        // Find closest intersection with any wall
        if let Some((hit_point, wall)) = find_closest_intersection(&current_ray, walls) {
            // Add beam segment from ray origin to hit point
            beams.push(Beam {
                start: current_ray.origin,
                end: hit_point,
                color: colors[reflection % colors.len()],
            });
            
            // Calculate reflection
            let new_direction = reflect(current_ray.direction, wall.normal);
            
            // Update ray for next reflection
            current_ray = Ray::new(hit_point, new_direction);
        } else {
            // No intersection, extend beam to edge of screen
            let end_point = extend_ray_to_edge(&current_ray);
            beams.push(Beam {
                start: current_ray.origin,
                end: end_point,
                color: colors[reflection % colors.len()],
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
    
    // Assume walls are axis-aligned for simplicity
    // For X-axis walls (left/right)
    if wall.normal.x.abs() > 0.9 {
        let x = wall.min.x;
        let t = (x - ray.origin.x) / ray.direction.x;
        
        // Intersection is behind the ray
        if t < 0.0 {
            return None;
        }
        
        let hit_y = ray.origin.y + t * ray.direction.y;
        
        // Check if hit point is within wall bounds
        if hit_y >= wall.min.y && hit_y <= wall.max.y {
            let hit_point = Vector3::new(x, hit_y, 0.0);
            let distance = (hit_point - ray.origin).length();
            return Some((hit_point, distance));
        }
    }
    // For Y-axis walls (top/bottom)
    else if wall.normal.y.abs() > 0.9 {
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
    
    // Calculate intersection with screen edges
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
        // Fallback (shouldn't happen, but just in case)
        ray.origin + ray.direction * 100.0
    }
}

// Render the current frame
fn render_frame(
    stdout: &mut std::io::Stdout,
    beams: &[Beam],
    cannon_pos: Vector3,
    mouse_pos: Vector3,
) -> Result<()> {
    // Create empty grid
    let mut grid = vec![vec![' '; WIDTH]; HEIGHT];
    
    // Draw cannon
    let cannon_x = cannon_pos.x.round() as usize;
    let cannon_y = cannon_pos.y.round() as usize;
    if cannon_x < WIDTH && cannon_y < HEIGHT {
        grid[cannon_y][cannon_x] = '⦿'; // Cannon symbol
    }
    
    // Draw targeting line to mouse position
    let direction = (mouse_pos - cannon_pos).normalize();
    let mut pos = cannon_pos + direction; // Start slightly away from cannon
    
    // Draw dotted line to mouse
    for _ in 0..20 {
        let x = pos.x.round() as usize;
        let y = pos.y.round() as usize;
        
        if x < WIDTH && y < HEIGHT && (x != cannon_x || y != cannon_y) {
            grid[y][x] = '·'; // Dot for targeting line
        }
        
        pos = pos + direction;
        
        // Stop if we reach mouse position
        if (pos - mouse_pos).length() < 1.0 {
            break;
        }
    }
    
    // Draw laser beams using Bresenham's line algorithm
    for beam in beams {
        draw_line(&mut grid, beam);
    }
    
    // Draw everything to the screen
    execute!(stdout, cursor::MoveTo(0, 0))?;
    
    // Draw the grid
    for row in &grid {
        for &cell in row {
            write!(stdout, "{}", cell)?;
        }
        write!(stdout, "\n\r")?;
    }
    
    // Draw instructions
    let instructions = "Click or press SPACE to fire laser | Q or ESC to quit";
    execute!(
        stdout,
        cursor::MoveTo(
            (WIDTH as u16 - instructions.len() as u16) / 2,
            HEIGHT as u16
        ),
        SetForegroundColor(Color::White)
    )?;
    write!(stdout, "{}", instructions)?;
    
    stdout.flush()?;
    Ok(())
}

// Draw a colored line using Bresenham's algorithm
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
        if beam_dir.x.abs() > 0.7 { '━' } else { '╱' }
    } else {
        if beam_dir.y.abs() > 0.7 { '┃' } else { '╲' }
    };
    
    loop {
        if x >= 0 && y >= 0 && x < WIDTH as i32 && y < HEIGHT as i32 {
            grid[y as usize][x as usize] = char_to_use;
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