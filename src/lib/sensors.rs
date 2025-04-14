// sensors.rs

use linux_embedded_hal::{Delay, I2cdev, SpidevDevice}; // Added I2cdev back in case BNO055 is uncommented later
use mint::{EulerAngles, Quaternion};
use rppal::gpio::{self, Gpio, InputPin, Trigger};
use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;
// use bno055::{BNO055OperationMode, Bno055}; // Keep commented if not used
use crate::driver::bme280::BME280;

// Constants for the ISA barometric formula (altitude in meters, pressure in Pascals)
const ALTITUDE_FACTOR: f32 = 44330.0; // Corresponds to meters
const ISA_EXPONENT: f32 = 1.0 / 5.255; // Approximately 0.190294957
const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0; // Pa - Currently unused, but kept for context

pub struct Sensors {
    pressure: f32,
    rail_pos: i8,
    rail_direction: i8, // Might need this later for update_reading logic
    rail_pin: Arc<Mutex<InputPin>>,
    altitude: f32,
    euler_angles: EulerAngles<f32, ()>,
    quaternion: Quaternion<f32>,
    // pub imu: Bno055<I2cdev>, // Keep commented if not used
    bme: BME280,
    // ground_pressure: f32, // Add back if altitude calculation is fixed
    delay: Delay,
    // dev: Arc<Mutex<I2cdev>>, // Keep commented if not used
}

impl Sensors {
    pub fn new() -> Self {
        // Read the last rail data from a file
        let content = std::fs::read_to_string("rail.pos").unwrap_or_else(|_| "0".to_string()); // Handle potential file read error

        let rail_pos = match content.trim().parse::<i8>() {
            // Trim whitespace
            Ok(res) => res,
            Err(_) => 0, // Default to 0 on parse error
        };

        // let i2c_dev = I2cdev::new("/dev/i2c-1").unwrap(); // Keep commented if not used
        let mut delay = Delay {};

        // BNO055 IMU Initialization (Keep commented if not used)
        // let mut imu = Bno055::new(i2c_dev).with_alternative_address();
        // imu.init(&mut delay)
        //     .expect("An error occurred while building the IMU");
        // imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        //     .expect("An error occurred while setting the IMU mode");
        // let status = imu.get_calibration_status().unwrap();
        // println!("The IMU's calibration status is: {:?}", status);

        // BME280 Pressure/Altitude Sensor Initialization
        let mut bme280 = BME280::new("/dev/spidev0.1");

        // Optional: Read initial measurement to potentially set ground pressure
        // let initial_measurements = bme280.measure(&mut delay).expect("Failed to get initial BME measurement");
        // let ground_pressure = initial_measurements.pressure;
        // println!("Initial ground pressure set to: {}", ground_pressure);

        Sensors {
            // dev, // Keep commented if not used
            pressure: 0.0, // Will be updated
            rail_pos,
            rail_direction: 1, // Assuming default forward direction
            rail_pin: Arc::new(Mutex::new(
                Gpio::new()
                    .expect("Failed to initialize GPIO")
                    .get(23)
                    .expect("Failed to get GPIO pin 23")
                    .into_input_pullup(),
            )),
            altitude: 0.0, // Will be updated
            euler_angles: EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]),
            quaternion: Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]), // Default quaternion
            // imu, // Keep commented if not used
            bme: bme280,
            delay,
            // ground_pressure, // Add back if altitude calculation is fixed
        }
    }

    // TODO: The logic inside the callback needs access to `self` (specifically rail_direction),
    // which is tricky with `Fn`. This might require a different approach like using channels
    // or rethinking how rail position is updated. The current implementation inside the
    // closure won't work as intended because it re-reads the file and doesn't use rail_direction.
    pub fn setup_rail_interrupt(&mut self) {
        let inpin = self.rail_pin.clone();
        // let direction = self.rail_direction; // Capture direction if needed for logic below

        let mut pin_guard = inpin.lock().expect("Failed to lock rail pin mutex");

        pin_guard
            .set_async_interrupt(
                Trigger::RisingEdge,
                // Consider debounce duration carefully
                Some(Duration::from_millis(50)), // Increased debounce
                move |_level| {
                    // Parameter is Gpio level, not event
                    // This logic runs in a separate thread and cannot directly modify `self`
                    // It also re-reads the file each time, potentially overwriting concurrent updates.
                    // A better approach might involve sending a message via a channel.
                    println!("Rail interrupt triggered!");

                    let content =
                        std::fs::read_to_string("rail.pos").unwrap_or_else(|_| "0".to_string());
                    let mut current_pos = content.trim().parse::<i8>().unwrap_or(0);

                    // Placeholder logic: Increment position (needs rail_direction ideally)
                    current_pos += 1; // Should use `direction` captured earlier if possible

                    match OpenOptions::new()
                        .write(true)
                        .truncate(true)
                        .create(true)
                        .open("rail.pos")
                    {
                        Ok(mut file) => {
                            if let Err(e) = writeln!(file, "{}", current_pos) {
                                eprintln!("Failed to write rail position: {}", e);
                            }
                        }
                        Err(e) => {
                            eprintln!("Failed to open rail.pos for writing: {}", e);
                        }
                    }
                },
            )
            .expect("Failed to set async interrupt on rail pin");
        println!("Rail interrupt setup complete.");
    }

    // Updates internal pressure reading
    pub fn update_pressure(&mut self) {
        self.pressure = self.bme.get_data().1
    }

    // Calculates altitude based on current pressure and a reference pressure
    // TODO: Requires a valid reference pressure (e.g., ground_pressure)
    pub fn update_altitude(&mut self, reference_pressure_pa: f32) {
        self.update_pressure(); // Ensure pressure reading is up-to-date

        if self.pressure.is_nan() {
            self.altitude = f32::NAN;
            return;
        }

        if reference_pressure_pa <= 0.0 {
            // eprintln!("Invalid reference pressure for altitude calculation: {}", reference_pressure_pa);
            // Keep previous altitude or set to NaN, depending on desired behavior
            // self.altitude = f32::NAN;
            return; // Cannot calculate without valid reference
        }

        // Formula: Altitude(m) = 44330.0 * [1 - (P / P₀)^(1 / 5.255)]
        let pressure_ratio = self.pressure / reference_pressure_pa;

        // Check for invalid pressure ratio (e.g., negative pressure reading)
        if pressure_ratio <= 0.0 {
            self.altitude = f32::NAN; // Cannot take powf of non-positive number
            return;
        }

        let altitude_meters = ALTITUDE_FACTOR * (1.0 - pressure_ratio.powf(ISA_EXPONENT));
        self.altitude = altitude_meters;
    }

    // Updates orientation from IMU (if used)
    // pub fn update_orientation(&mut self) {
    //     match self.imu.quat_angles() { // Or use .quaternion() depending on need
    //         Ok(val) => {
    //             self.quaternion = val.quat; // Assuming quat_angles returns struct with .quat
    //             // Optionally convert/store Euler angles if needed
    //             // self.euler_angles = ...;
    //         }
    //         Err(e) => {
    //              eprintln!("Failed to read IMU orientation: {:?}", e);
    //              // Keep previous value or set to a default error state
    //         }
    //     }
    // }

    pub fn get_orientation(&self) -> Quaternion<f32> {
        self.quaternion // Return the stored quaternion
    }

    pub fn get_altitude(&self) -> f32 {
        self.altitude // Return the stored altitude
    }

    pub fn get_pressure(&self) -> f32 {
        self.pressure
    }

    // Reads rail position directly from the file
    pub fn get_rail_pos(&self) -> i8 {
        let content = std::fs::read_to_string("rail.pos").unwrap_or_else(|_| "0".to_string());
        content.trim().parse::<i8>().unwrap_or(0) // Default to 0 on error
    }
}
