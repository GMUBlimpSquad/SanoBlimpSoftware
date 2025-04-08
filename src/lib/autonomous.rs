use std::time::{Duration, Instant};

/// A simple error type for altitude-related issues.
#[derive(Debug)]
pub struct AltitudeError(pub &'static str);

/// Autonomous control system for regulating altitude and position using sensor feedback.
///
/// This struct provides methods to set a ground altitude reference, compute the altitude error
/// relative to a target height, and calculate PID control outputs based on the position error
/// from a detected object.
#[derive(Debug)]
pub struct Autonomous {
    // Previous error values for PID computations
    prev_x: f32,
    prev_y: f32,
    prev_z: f32,

    // Timestamp of the last detection
    prev_detection: Option<Instant>,

    // PID gains for each axis
    kp_x: f32,
    kd_x: f32,
    kp_y: f32,
    kd_y: f32,
    kp_z: f32,
    kd_z: f32,
    ki: f32, // Integral term, not currently used

    // Ground altitude reference (None if not set)
    ground_altitude: Option<f32>,
}

impl Autonomous {
    /// Create a new `Autonomous` controller with PID gains.
    pub fn new(kp_x: f32, kd_y: f32, kp_z: f32, kd_x: f32, kp_y: f32, kd_z: f32, ki: f32) -> Self {
        Self {
            prev_x: 0.0,
            prev_y: 0.0,
            prev_z: 0.0,
            prev_detection: None,

            kp_x,
            kd_x,
            kp_y,
            kd_y,
            kp_z,
            kd_z,
            ki,

            ground_altitude: None,
        }
    }

    /// Map a value from one numerical range to another.
    ///
    /// Equivalent to the Python `map_value` method.
    pub fn map_value(
        &self,
        value: f32,
        from_low: f32,
        from_high: f32,
        to_low: f32,
        to_high: f32,
    ) -> f32 {
        (value - from_low) / (from_high - from_low) * (to_high - to_low) + to_low
    }

    /// Set the ground altitude reference using the current sensor reading.
    ///
    /// In real code, you’d read from your BMP sensor. Here, we just mock an altitude value
    /// for demonstration. Returns `Ok(altitude)` if successful, or an `Err(...)` if something goes wrong.
    pub fn set_ground(&mut self) -> Result<f32, AltitudeError> {
        // In real code, read from the sensor:
        // let altitude = self.bmp.altitude(); // e.g. hypothetical method

        // For demonstration, we just pretend the sensor reading is 100.0:
        let altitude = 100.0_f32;
        self.ground_altitude = Some(altitude);
        Ok(altitude)
    }

    /// Compute the normalized altitude error relative to a desired altitude (offset from ground).
    ///
    /// Returns a value in [-1.0, 1.0].
    pub fn goal_height(&self, desired_altitude: f32) -> Result<f32, AltitudeError> {
        let ground_alt = match self.ground_altitude {
            Some(alt) => alt,
            None => {
                return Err(AltitudeError(
                    "Ground altitude not set. Call set_ground() first.",
                ));
            }
        };

        // Hypothetical current altitude reading:
        // In real code, read from sensor. Here, we mock 102.5 for demonstration.
        let current_altitude = 102.5_f32;

        // Our target height = ground + desired offset
        let target_height = ground_alt + desired_altitude;
        // Altitude error = current - target
        let altitude_error = current_altitude - target_height;

        // Map the error from [-10, 10] to [-1, 1]
        let mapped = self.map_value(altitude_error, -10.0, 10.0, -1.0, 1.0);
        // Clamp to [-1, 1]
        let clamped = mapped.clamp(-1.0, 1.0);

        Ok(clamped)
    }

    /// Compute the PID control output based on the position error.
    ///
    /// Given the center coordinates (x, y, z) of a detected bounding box, this method computes
    /// the error with respect to a setpoint (0, 300, 150), calculates derivative terms, and
    /// computes the PID outputs. Returns (pid_x, pid_y, pid_z), each clamped to [-1.0, 1.0].
    pub fn position(&mut self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        // Desired setpoints (mirroring the Python logic):
        // xErr = 0 - x, yErr = 300 - y, zErr = 150 - z
        let x_err = 0.0 - x;
        let y_err = self.map_value(y - 300.0, -300.0, 300.0, -1.0, 1.0);
        let z_err = self.map_value(z - 250.0, 250.0, -250.0, 1.0, -1.0);

        // Calculate time elapsed since the last detection
        let now = Instant::now();
        let dt = if let Some(prev) = self.prev_detection {
            let dur = now.duration_since(prev);
            // Avoid division by zero
            dur.as_secs_f32().max(1.0e-6)
        } else {
            1.0e-6
        };

        // Derivative of errors
        let dx = (x_err - self.prev_x) / dt;
        let dy = (y_err - self.prev_y) / dt;
        let dz = (z_err - self.prev_z) / dt;

        // PID outputs (P + D; we ignore I here)
        let pid_x = (self.kp_x * x_err) + (self.kd_x * dx);
        let pid_y = (self.kp_y * y_err) + (self.kd_y * dy);
        let pid_z = (self.kp_z * z_err) + (self.kd_z * dz);

        // Update previous values
        self.prev_x = x_err;
        self.prev_y = y_err;
        self.prev_z = z_err;
        self.prev_detection = Some(now);

        // Clamp each output to [-1, 1]
        let px = pid_x.clamp(-1.0, 1.0);
        let py = pid_y.clamp(-1.0, 1.0);
        let pz = pid_z.clamp(-1.0, 1.0);

        (px, py, pz)
    }
}

// --------------------------
// Example usage (test it out)
// --------------------------
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_autonomous() {
        let mut auto = Autonomous::new(0.1, 0.01, 0.2, 0.02, 0.3, 0.03, 0.0);

        // Set ground altitude
        let ground = auto.set_ground().expect("Failed to set ground altitude");
        println!("Ground altitude set to: {}", ground);

        // Check goal_height
        let alt_error = auto.goal_height(5.0).expect("No ground altitude set");
        println!("Altitude error (normalized) = {}", alt_error);

        // Position control
        let (px, py, pz) = auto.position(10.0, 250.0, 140.0);
        println!("PID outputs: x={}, y={}, z={}", px, py, pz);
    }
}
