use std::error::Error;
use std::f64::consts::PI;

// --- Plotters Setup ---
// Use `plotters::prelude::*` is convenient but imports many things.
// Alternatively, import specific items:
use plotters::prelude::{
    BitMapBackend, ChartBuilder, IntoDrawingArea, LineSeries, PointSeries, SeriesLabelPosition,
    BLACK, BLUE, RED, WHITE,
};
use plotters::style::{Color, TextStyle, AsRelative, FontDesc};
use plotters::coord::ranged3d::{Cartesian3d, ProjectionMatrix}; // For 3D plotting

// --- Blimp State Definition (same as before) ---
#[derive(Debug, Clone, Copy)]
struct BlimpState {
    position: (f64, f64, f64), // x, y, z
    orientation: (f64, f64, f64), // yaw, pitch, roll (radians)
    forward_speed: f64,
    angular_velocity: (f64, f64, f64), // yaw_rate, pitch_rate, roll_rate (rad/s)
}

impl BlimpState {
    fn new(
        initial_pos: (f64, f64, f64),
        initial_orientation: (f64, f64, f64),
        forward_speed: f64,
        angular_velocity: (f64, f64, f64),
    ) -> Self {
        BlimpState {
            position: initial_pos,
            orientation: initial_orientation,
            forward_speed,
            angular_velocity,
        }
    }

    fn update(&mut self, dt: f64) {
        let (mut yaw, mut pitch, mut roll) = self.orientation;
        let (yaw_rate, pitch_rate, roll_rate) = self.angular_velocity;

        // Update Orientation
        yaw += yaw_rate * dt;
        pitch += pitch_rate * dt;
        roll += roll_rate * dt;
        yaw = yaw.rem_euclid(2.0 * PI); // Normalize yaw [0, 2*PI)
        self.orientation = (yaw, pitch, roll);

        // Update Position
        let vx = self.forward_speed * yaw.cos() * pitch.cos();
        let vy = self.forward_speed * yaw.sin() * pitch.cos();
        let vz = self.forward_speed * pitch.sin(); // Positive pitch -> positive vz
        self.position.0 += vx * dt;
        self.position.1 += vy * dt;
        self.position.2 += vz * dt;
    }
}

// --- Plotting Function ---
fn plot_trajectory(
    data: &Vec<(f64, f64, f64)>,
    filename: &str,
) -> Result<(), Box<dyn Error>> {
    if data.is_empty() {
        println!("No data to plot.");
        return Ok(());
    }

    let root_area = BitMapBackend::new(filename, (1024, 768)).into_drawing_area();
    root_area.fill(&WHITE)?;

    // --- Determine Axis Ranges ---
    let (mut min_x, mut max_x) = (data[0].0, data[0].0);
    let (mut min_y, mut max_y) = (data[0].1, data[0].1);
    let (mut min_z, mut max_z) = (data[0].2, data[0].2);

    for &(x, y, z) in data.iter() {
        min_x = min_x.min(x); max_x = max_x.max(x);
        min_y = min_y.min(y); max_y = max_y.max(y);
        min_z = min_z.min(z); max_z = max_z.max(z);
    }

    // Add some padding to make sure points are not on the very edge
    let padding = 10.0; // Adjust padding as needed
    let x_range = (min_x - padding)..(max_x + padding);
    let y_range = (min_y - padding)..(max_y + padding);
    // Ensure z range has some size even if flat, prevent division by zero in plotters
    let z_padding = (max_z - min_z).max(1.0) * 0.1 + padding;
    let z_range = (min_z - z_padding)..(max_z + z_padding);


    // --- Create 3D Chart ---
    let caption_style = TextStyle::from(("sans-serif", 30).into_font()).color(&BLACK);
    let mut chart = ChartBuilder::on(&root_area)
        .caption("Blimp Spiral Trajectory (3D View)", caption_style)
        .set_all_label_area_size(50.percent()) // Adjust label area size
        .build_cartesian_3d(x_range.clone(), y_range.clone(), z_range.clone())?;

    // --- Configure Projection and Axes ---
    // Adjust pitch (elevation) and yaw (azimuth) for the best view
    chart.with_projection(|mut pb| {
        pb.pitch = 0.6; // Elevation angle ~34 degrees
        pb.yaw = 1.0;   // Azimuth angle ~57 degrees
        pb.scale = 0.8; // Zoom level
        pb.into_matrix() // Create the projection matrix
    });

    let axis_style = TextStyle::from(("sans-serif", 15).into_font()).color(&BLACK);
    chart.configure_axes()
        .light_grid_style(BLACK.mix(0.15)) // Lighter grid lines
        .max_light_lines(8) // Fewer grid lines for less clutter
        .axis_label_style(axis_style)
        .x_formatter(&|x| format!("{:.0}m", x))
        .y_formatter(&|y| format!("{:.0}m", y))
        .z_formatter(&|z| format!("{:.0}m", z))
        .x_desc("X Position (m)")
        .y_desc("Y Position (m)")
        .z_desc("Altitude Z (m)")
        .draw()?;


    // --- Draw Data Series ---
    // Draw the trajectory line
    chart
        .draw_series(LineSeries::new(
            data.iter().map(|&(x, y, z)| (x, y, z)), // Iterate over position tuples
            &BLUE.mix(0.8).stroke_width(2),         // Blue line, slightly thicker
        ))?
        .label("Trajectory")
        .legend(|(x, y)| plotters::prelude::PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Optional: Draw start and end points
    if let Some(start_pos) = data.first() {
        chart.draw_series(PointSeries::of_element(
            vec![*start_pos], // Needs a Vec or iterator
            5,                // Point size
            &RED,             // Color for start point
            &|coord, size, style| {
                plotters::prelude::EmptyElement::at(coord) // Position
                + plotters::prelude::Circle::new((0, 0), size, style.filled()) // Circle shape
                + plotters::prelude::Text::new("Start", (0, -15), ("sans-serif", 15).into_font().color(&RED)) // Label
            },
        ))?;
    }
     if let Some(end_pos) = data.last() {
         chart.draw_series(PointSeries::of_element(
            vec![*end_pos], // Needs a Vec or iterator
            5,                // Point size
            &plotters::style::colors::full_palette::PURPLE, // Color for end point
            &|coord, size, style| {
                plotters::prelude::EmptyElement::at(coord) // Position
                + plotters::prelude::Circle::new((0, 0), size, style.filled()) // Circle shape
                + plotters::prelude::Text::new("End", (0, 15), ("sans-serif", 15).into_font().color(&plotters::style::colors::full_palette::PURPLE)) // Label
            },
        ))?;
    }

    // --- Draw Legend ---
    chart
        .configure_series_labels()
        .border_style(BLACK.stroke_width(1))
        .background_style(WHITE.mix(0.8))
        .label_font(("sans-serif", 15))
        .position(SeriesLabelPosition::UpperRight) // Adjust legend position
        .draw()?;


    // --- Finalize Plot ---
    root_area.present()?; // Save the plot to the file
    println!("Plot successfully saved to {}", filename);

    Ok(())
}


// --- Main Simulation Function ---
fn main() {
    // Simulation Parameters (same as before)
    let dt: f64 = 0.1;
    let total_time: f64 = 120.0;
    let num_steps = (total_time / dt) as usize;

    // Blimp Initialization (same as before)
    let initial_position = (0.0, 0.0, 100.0);
    let initial_orientation = (0.0, 0.0, 0.0);
    let forward_speed = 5.0;
    let yaw_rate = PI / 15.0;
    let pitch_rate = PI / 60.0; // Climbing spiral
    let roll_rate = 0.0;
    let angular_velocity = (yaw_rate, pitch_rate, roll_rate);

    let mut blimp = BlimpState::new(
        initial_position,
        initial_orientation,
        forward_speed,
        angular_velocity,
    );

    // --- Data Collection ---
    let mut trajectory_data: Vec<(f64, f64, f64)> = Vec::with_capacity(num_steps + 1);

    println!("Running simulation...");
    // --- Simulation Loop ---
    for _ in 0..=num_steps {
        // Store current position BEFORE updating for the next step
        trajectory_data.push(blimp.position);
        // Update blimp state for the next time step
        blimp.update(dt);
    }
    println!("Simulation complete. Found {} data points.", trajectory_data.len());


    // --- Plotting ---
    let output_filename = "blimp_spiral_3d_plot.png";
    if let Err(e) = plot_trajectory(&trajectory_data, output_filename) {
        eprintln!("Error generating plot: {}", e);
    }
}
