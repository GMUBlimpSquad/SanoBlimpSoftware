// use bme280::i2c::BME280;
use bno055::{BNO055OperationMode, Bno055};
use core::f64;
use gilrs::{Button, Event, GamepadId, Gilrs, PowerInfo};
use linux_embedded_hal::{Delay, I2cdev, SpidevDevice};
use mint::{EulerAngles, Quaternion};
use pwm_pca9685::{Address, Channel, Pca9685};
use rand::Rng; // Use Rng trait
use rppal::gpio::{self, Gpio, InputPin, Trigger};
use std::fs::{File, OpenOptions};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{self, Duration};
// use tokio::spawn; // Not actively used in this snippet
// use tokio::sync::mpsc::{UnboundedReceiver, unbounded_channel}; // Not actively used

use super::object_detection::Detection;
use shared_bus::{self, BusManager, I2cProxy, NullMutex}; // Import the shared-bus crate
use std::io::{self, Write};

use crate::driver::bme280::{self, Bme280, Bme280Measurements, DEFAULT_SEA_LEVEL_HPA};
use crate::{BlimpSensorData, States, lib::base_station::communication::*};

// TODO Make these a config file, and will depend on the blimp's hardware

const PWM_FREQUENCY: f32 = 60.0; // Hz for motors and servos
const MIN_PULSE_SERVO: f32 = 500.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE_SERVO: f32 = 2500.0; // Maximum pulse width in µs (Full throttle)
const NEUTRAL_ANGLE: f32 = 90.0; // Neutral position for motors and servos
//
const PWM_FREQUENCY_MOTOR: f32 = 60.0; // Hz for motors and servos
const MIN_PULSE: f32 = 600.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE: f32 = 2600.0; // Maximum pulse width in µs (Full throttle)
const MID_PULSE: f32 = 1500.0; // Neutral (90° equivalent)

// const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos
//const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos

// Constants for the ISA barometric formula (altitude in meters, pressure in Pascals)
const ALTITUDE_FACTOR: f32 = 44330.0; // Corresponds to meters
const ISA_EXPONENT: f32 = 1.0 / 5.255; // Approximately 0.190294957
const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0; // Pa

// Define Rail Limits (adjust these values as needed)
const MAX_RAIL_POS: i8 = 0;
const MIN_RAIL_POS: i8 = 0; //Negative rail position is front of blimp

/// Every blimp needs the following trait
pub trait Blimp {
    fn update(
        &mut self,
        state: Arc<Mutex<States>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData;

    fn mix(&mut self) -> Actuations;
    fn update_input(&mut self, input: (f32, f32, f32));
}

pub struct Sensors {
    pressure: f32,
    rail_pos: Arc<Mutex<i8>>, // Use Arc<Mutex> for shared position state
    rail_pin_trigger: Arc<Mutex<InputPin>>, // Keep using Arc<Mutex> for pin
    intended_direction: Arc<Mutex<i8>>, // <<< NEW: Shared state for direction command (-1, 0, 1)
    altitude: f32,
    euler_angles: EulerAngles<f32, ()>, // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    quaternion: Quaternion<f32>,        // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);
    pub imu: Bno055<I2cdev>,
    bme: Bme280,
    ground_altitude: f32,
    delay: Delay,
    //dev: Arc<Mutex<I2cdev>>, // Not used
}

impl Sensors {
    pub fn new() -> Self {
        // Read the last rail data from a file
        let initial_rail_pos = match std::fs::read_to_string("rail.pos") {
            Ok(content) => content.trim().parse::<i8>().unwrap_or(0), // Default to 0 on error/empty
            Err(_) => 0, // Default to 0 if file doesn't exist
        };
        println!("Initialized rail position: {}", initial_rail_pos);

        let dev = I2cdev::new("/dev/i2c-1").unwrap();
        //
        let mut delay = Delay {};
        //
        let mut imu = Bno055::new(dev);

        //let mut imu = Bno055::new(dev).with_alternative_address(); // IMU commented out
        //imu.init(&mut delay)
        //    .expect("An error occurred while building the IMU");
        //
        //imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        //    .expect("An error occurred while setting the IMU mode");
        //
        //let mut status = imu.get_calibration_status().unwrap();
        //println!("The IMU's calibration status is: {:?}", status);

        // Initialize SPI for BME280
        // let spi = SpidevDevice::open("/dev/spidev0.1").unwrap();
        // let mut bme280 = BME280::new(spi).unwrap();
        // bme280.init(&mut delay).unwrap(); // Initialization might be needed depending on driver version/settings
        let mut bme280 = Bme280::new("/dev/spidev0.0", DEFAULT_SEA_LEVEL_HPA).unwrap();

        // Initialize the GPIO pin for rail interrupt within an Arc<Mutex>
        let gpio = Gpio::new().expect("Failed to initialize GPIO");
        let input_pin = gpio
            .get(23) // Using GPIO Pin 23 as before
            .expect("Failed to get GPIO pin 23")
            .into_input_pullup(); // Use pull-up resistor

        let meas = bme280.read().unwrap();
        // let measurements = bme280.measure(&mut delay).unwrap(); // Initial measurement if needed

        Sensors {
            pressure: 0.0,
            rail_pos: Arc::new(Mutex::new(initial_rail_pos)), // Initialize Arc<Mutex> for position
            rail_pin_trigger: Arc::new(Mutex::new(input_pin)), // Initialize Arc<Mutex> for pin
            intended_direction: Arc::new(Mutex::new(0)), // <<< NEW: Initialize direction to 0 (neutral)
            altitude: 0.0,
            euler_angles: EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]),
            quaternion: Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]),
            imu: imu,
            bme: bme280,
            delay,
            ground_altitude: meas.altitude_m,
            // ground_pressure: measurements.pressure,
        }
    }

    // Modify get_rail_pos to work with Arc<Mutex>
    pub fn get_rail_pos(&self) -> i8 {
        match self.rail_pos.lock() {
            Ok(pos) => *pos, // Dereference the MutexGuard to get the i8 value
            Err(e) => {
                eprintln!("ERROR: Failed to lock rail_pos mutex for reading: {:?}", e);
                0 // Return default on error (consider implications)
            }
        }
    }

    // Resets the rail position to 0
    pub fn reset_rail_pos(&self) {
        match OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open("rail.pos")
        {
            Ok(mut file) => {
                if let Err(e) = writeln!(file, "{}", 0) {
                    eprintln!("ERROR: Failed to write to rail.pos in interrupt: {:?}", e);
                }
            }
            Err(e) => {
                eprintln!(
                    "ERROR: Failed to open rail.pos for writing in interrupt: {:?}",
                    e
                );
            }
        }
        // !!!!! END WARNING !!!!!
    }

    /// Sets the intended direction for the next rail count update via interrupt.
    /// `1` for positive direction, `-1` for negative, `0` for neutral/stop.
    pub fn set_intended_rail_direction(&self, direction: i8) {
        match self.intended_direction.lock() {
            Ok(mut dir_guard) => {
                *dir_guard = direction.clamp(-1, 1); // Ensure value is -1, 0, or 1
            }
            Err(e) => {
                eprintln!(
                    "ERROR: Failed to lock intended_direction mutex for setting: {:?}",
                    e
                );
            }
        }
    }

    /// Sets up the asynchronous interrupt handler for the rail position sensor.
    /// The handler now reads the `intended_direction` state to determine count direction.
    pub fn update_reading(&mut self) {
        // Clone the Arcs to move into the closure
        let rail_pos_clone = Arc::clone(&self.rail_pos);
        let pin_clone = Arc::clone(&self.rail_pin_trigger);
        let direction_clone = Arc::clone(&self.intended_direction); // <<< NEW: Clone direction Arc

        // Lock the pin mutex once to set the interrupt
        // Note: Pin access within the interrupt handler itself isn't done here,
        // only the shared state mutexes are locked inside the closure.
        let mut locked_pin = pin_clone
            .lock()
            .expect("Failed to lock rail pin mutex for setup");

        locked_pin
            .set_async_interrupt(
                Trigger::RisingEdge,             // Trigger on rising edge (adjust if sensor needs falling/both)
                Some(Duration::from_millis(50)), // Debounce timeout (adjust if needed)
                move |_level| {
                    // --- Start of Interrupt Handler ---

                    // --- Get Intended Direction ---
                    let direction_increment = match direction_clone.lock() {
                        Ok(dir_guard) => {
                             let dir_val = *dir_guard; // Read the shared direction state

                             // <<< --- DEBUG PRINT 2 --- >>>
                             println!("[Interrupt] Read direction_increment: {}", dir_val);
                             // <<< --- END DEBUG PRINT --- >>>

                             dir_val // Return the value
                        }
                        Err(e) => {
                            // Log error and default to no movement if mutex is poisoned
                            eprintln!(
                                "ERROR: Failed to lock direction_clone in interrupt: {:?}",
                                e
                            );
                            0 // Default to 0 (no change) if direction can't be read
                        }
                    };

                    // --- Update Position Only if Direction is Non-Zero ---
                    if direction_increment != 0 {
                        // Lock the shared position state
                        match rail_pos_clone.lock() {
                            Ok(mut current_pos) => {
                                // --- Perform the count update using the fetched direction ---
                                *current_pos += direction_increment;

                                // Clamp the value to prevent exceeding limits (optional but good practice)
                                // Allow one step past limits for detection in mix() logic
                                *current_pos = current_pos.clamp(MIN_RAIL_POS - 1, MAX_RAIL_POS + 1);

                                // Print the updated position and the direction used
                                println!(
                                    "Rail Interrupt! Direction: {}, New Position: {}",
                                    direction_increment, *current_pos
                                );

                                // !!!!! WARNING: FILE I/O IN INTERRUPT IS BAD PRACTICE !!!!!
                                // This can block the system and cause delays or instability.
                                // It's better to have a separate task that periodically saves
                                // the value from the Arc<Mutex> or saves on shutdown.
                                // Leaving it here for now as requested, but it should be moved.
                                match OpenOptions::new()
                                    .write(true)
                                    .truncate(true)
                                    .create(true)
                                    .open("rail.pos")
                                {
                                    Ok(mut file) => {
                                        if let Err(e) = writeln!(file, "{}", *current_pos) {
                                            eprintln!(
                                                "ERROR: Failed to write to rail.pos in interrupt: {:?}",
                                                e
                                            );
                                        }
                                    }
                                    Err(e) => {
                                        eprintln!(
                                            "ERROR: Failed to open rail.pos for writing in interrupt: {:?}",
                                            e
                                        );
                                    }
                                }
                                // !!!!! END WARNING !!!!!

                                // MutexGuard (`current_pos`) is automatically dropped here, releasing the lock.
                            }
                            Err(e) => {
                                eprintln!(
                                    "ERROR: Failed to lock rail_pos_clone in interrupt: {:?}",
                                    e
                                );
                                // Position not updated if lock fails
                            }
                        }
                    } else {
                        // Optional: Log that interrupt occurred but direction was neutral, so no count change
                        // println!("[Interrupt] Neutral direction (0), position unchanged.");
                    }
                    // --- End of Interrupt Handler ---
                },
            )
            .expect("Failed to set async interrupt");

        println!("Rail position interrupt handler set up on pin 23 (with direction control).");
        // MutexGuard (`locked_pin`) is implicitly dropped here, releasing the lock on the pin mutex.
    }

    pub fn update_altitude(&mut self) {
        // --- Get Current Pressure ---
        // It's better practice to handle the Result properly instead of unwrapping,
        // especially in embedded systems where panics can be problematic.
        let measurement = match self.bme.read() {
            Ok(m) => m,
            Err(e) => {
                // Handle the error appropriately: log it, set a default/error altitude, etc.
                // For example:
                println!("Error reading BME sensor: {:?}", e); // If using rtt-target
                self.altitude = f32::NAN; // Set altitude to Not a Number to indicate an error
                return; // Exit the function if measurement failed
            }
        };
        // println!("altitude: {:?}", measurement);
        // --- Store Result ---
        self.altitude = measurement.altitude_m - self.ground_altitude;
    }

    // pub fn update_orientation(&mut self) { // If IMU is uncommented
    //     match self.imu.quaternion() {
    //         Ok(val) => {
    //              // Convert BNO055 quaternion (usually f64) if needed
    //              // self.quaternion = Quaternion::<f32>::from([val.v.x as f32, val.v.y as f32, val.v.z as f32, val.s as f32]);
    //              // self.euler_angles = EulerAngles::<f32,()>::from(self.quaternion); // Or get Euler directly if available
    //         }
    //         Err(e) => { /* Handle error */ }
    //     }
    // }

    pub fn get_orientation(&self) -> Quaternion<f32> {
        self.quaternion // Returns the stored value (currently always 0)
    }

    pub fn get_altitude(&self) -> f32 {
        // println!("{:?}", self.altitude);
        self.altitude
    }
} // end impl Sensors

pub struct PCAActuator {
    pwm: Pca9685<I2cdev>,
    neutral_angle_motor: f32,
}

impl PCAActuator {
    pub fn new(neutral_angle_motor: f32) -> Self {
        let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device");
        let address = Address::default();
        //let address = Address::from(0x55);
        let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");

        // Calculate prescale for desired frequency (adjust formula if needed based on PCA9685 datasheet/driver)
        // Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1
        // Assuming internal oscillator is 25MHz = 25,000,000 Hz
        let osc_clock = 25_000_000.0;
        let prescale_f = (osc_clock / (4096.0 * PWM_FREQUENCY)) - 1.0;
        let prescale = prescale_f.round() as u8; // Calculate for servo freq

        // Set prescale for servos/motors (assuming same freq for Flappy's servos)
        pwm.set_prescale(prescale).expect("Failed to set prescale");
        pwm.enable().expect("Failed to enable PCA9685");

        let mut actuator = PCAActuator {
            pwm,
            neutral_angle_motor,
        };
        actuator.init_escs();
        actuator
    }

    /// Convert angle value (0-180°) to PWM pulse width and send to ESCs/Motors
    fn set_motor_speed(&mut self, channel: Channel, angle: f32) {
        // Map 0-180 angle range to MIN_PULSE - MAX_PULSE range
        let pulse_width_us =
            MIN_PULSE + ((angle.clamp(0.0, 180.0) / 180.0) * (MAX_PULSE - MIN_PULSE));
        // Convert pulse width in microseconds to PCA9685 ticks (0-4095)
        let ticks = (pulse_width_us * PWM_FREQUENCY_MOTOR * 4096.0 * 1e-6).round() as u16;
        let off = ticks.clamp(0, 4095); // Ensure value is within 12-bit range
        let on = 0; // Start pulse immediately

        self.pwm
            .set_channel_on_off(channel, on, off)
            .unwrap_or_else(|e| eprintln!("Failed to set motor speed on {:?}: {:?}", channel, e));
    }

    /// Convert angle value (0-180°) to PWM pulse width and send to Servos
    fn set_servo_speed(&mut self, channel: Channel, angle: f32) {
        // Map 0-180 angle range to MIN_PULSE_SERVO - MAX_PULSE_SERVO range
        let pulse_width_us = MIN_PULSE_SERVO
            + ((angle.clamp(0.0, 180.0) / 180.0) * (MAX_PULSE_SERVO - MIN_PULSE_SERVO));
        // Convert pulse width in microseconds to PCA9685 ticks (0-4095)
        let ticks = (pulse_width_us * PWM_FREQUENCY * 4096.0 * 1e-6).round() as u16;
        let off = ticks.clamp(0, 4095); // Ensure value is within 12-bit range
        let on = 0; // Start pulse immediately

        self.pwm
            .set_channel_on_off(channel, on, off)
            .unwrap_or_else(|e| eprintln!("Failed to set servo speed on {:?}: {:?}", channel, e));
    }

    /// ESC initialization sequence for BLHeli_S (example)
    pub fn init_escs(&mut self) {
        println!("Initializing ESCs (sending arming sequence)...");

        // Use NEUTRAL_ANGLE_MOTOR for the neutral point during init
        // let neutral_angle = NEUTRAL_ANGLE_MOTOR; // e.g., 80.0
        let min_angle = 0.0; // Corresponds to MIN_PULSE
        let max_angle = 180.0; // Corresponds to MAX_PULSE

        // Step 1: Send max throttle (using max_angle) - some ESCs need this
        println!("Sending max throttle angle ({})", max_angle);
        self.set_motor_speed(Channel::C0, max_angle);
        self.set_motor_speed(Channel::C1, max_angle);
        self.set_motor_speed(Channel::C2, max_angle);
        self.set_motor_speed(Channel::C3, max_angle);
        sleep(Duration::from_secs(1)); // Wait for ESCs to register max throttle

        // Step 2: Send min throttle (using min_angle) to arm
        println!("Sending min throttle angle ({})", min_angle);
        self.set_motor_speed(Channel::C0, min_angle);
        self.set_motor_speed(Channel::C1, min_angle);
        self.set_motor_speed(Channel::C2, min_angle);
        self.set_motor_speed(Channel::C3, min_angle);
        sleep(Duration::from_secs(2)); // Wait longer for arming confirmation beep usually

        // Step 3: Move to neutral throttle (ready to receive commands)
        println!("Setting ESCs to neutral");
        self.set_motor_speed(Channel::C0, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C1, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C2, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C3, self.neutral_angle_motor);

        println!("ESC initialization sequence complete!");
    }

    pub fn actuate(&mut self, act: Actuations) {
        // Send commands to motors (Channels 0-3)
        self.set_motor_speed(Channel::C0, act.m1);
        self.set_motor_speed(Channel::C1, act.m2);
        self.set_motor_speed(Channel::C2, act.m3);
        self.set_motor_speed(Channel::C3, act.m4);

        // Send commands to servos (Channels 4-7)
        self.set_servo_speed(Channel::C4, act.s1); // Wing 1
        self.set_servo_speed(Channel::C5, act.s2); // Wing 2
        self.set_servo_speed(Channel::C6, act.s3); // Tail
        self.set_servo_speed(Channel::C7, act.s4); // CG Rail
    }
} // end impl PCAActuator

pub struct SanoBlimp {
    state: Vec<f32>,
    input: (f32, f32, f32),
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    pub sensor: Sensors,
    manual: bool,
    pub score: bool,
    pub score_time: std::time::Instant,
    m1_mul: f32,
    m2_mul: f32,
    neutral_angle_motor: f32,
    controller_battery: f32,
}

pub struct Flappy {
    state: Vec<f32>,        // Currently unused
    input: (f32, f32, f32), // (LeftY: Fwd/Back, RightX: Turn, RightY: Up/Down)
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    pub sensor: Sensors, // Holds the sensors including rail state and direction intent
    manual: bool,
    controller_battery: f32,
    pub score: bool,
    pub score_time: std::time::Instant,
    neutral_angle_motor: f32,
    backing_out: bool,
    pub backing_out_timer: std::time::Instant,
}

impl Flappy {
    pub fn new(neutral_angle_motor: f32) -> Self {
        println!("Initializing Flappy...");
        let mut sensors = Sensors::new(); // Create sensors first
        // Call update_reading to set up the interrupt *after* Sensors is created
        sensors.update_reading(); // Setup the interrupt handler ONCE

        Flappy {
            state: Vec::new(),
            score: false,
            score_time: std::time::Instant::now(),
            neutral_angle_motor,
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(90.0),
            manual: true,
            sensor: sensors,
            controller_battery: 0.0,
            backing_out: false,
            backing_out_timer: std::time::Instant::now(),
        }
    }

    /// Calculates a sinusoidal angle for wing flapping.
    fn oscillate_wing(&self, direction: f64, freq: f64) -> f64 {
        // Base angle (center)
        let base_angle = 90.0;
        // Amplitude of oscillation
        let amplitude = 90.0;
        // Calculate current angle based on time
        let current_time_secs = time::SystemTime::now()
            .duration_since(time::SystemTime::UNIX_EPOCH)
            .unwrap_or(Duration::from_secs(0)) // Handle potential time error
            .as_secs_f64();

        let angle = base_angle
            + amplitude * direction * f64::sin(2.0 * f64::consts::PI * freq * current_time_secs);
        angle.clamp(0.0, 180.0) // Ensure angle stays within valid servo range
        // angle = self.map_value(angle, 0, 180, 5, 10) // Example mapping if needed
    }

    fn backing_out(&mut self) {
        if self.backing_out {
            // This is a timer for turning for someone who cannot read code.
            if self.backing_out_timer.elapsed() < std::time::Duration::from_secs(10) {
                self.actuator.actuate(Actuations {
                    m1: self.neutral_angle_motor,
                    m2: self.neutral_angle_motor,
                    m3: self.neutral_angle_motor,
                    m4: self.neutral_angle_motor,
                    s1: NEUTRAL_ANGLE,
                    s2: NEUTRAL_ANGLE,
                    s3: NEUTRAL_ANGLE,
                    s4: self.oscillate_wing(1.0, 1.1).clamp(90.0, 180.0) as f32,
                });
            }
        }

        // 90 0 90 0
    }

    /// Call this in the main loop to handle updates and actuation.
    pub fn run(&mut self) {
        // self.update();

        // Add sensor reads here if needed in main loop (like altitude)
        // self.sensor.update_altitude(); // Example: Read altitude periodically

        // If in manual mode, calculate and apply actuations
        if self.is_manual() {
            self.manual();
        } else {
            // Handle autonomous mode if implemented
            // let auto_actuations = self.autonomous_control();
            // self.actuator.actuate(auto_actuations);
            // For now, maybe just stop servos in non-manual mode:
            let neutral_act = Actuations {
                s1: NEUTRAL_ANGLE,
                s2: NEUTRAL_ANGLE,
                s3: NEUTRAL_ANGLE,
                s4: 95.0,             // Keep CG neutral?
                ..Default::default()  // Motors are already 0
            };
            self.actuator.actuate(neutral_act);
        }

        // Optional delay to control loop rate
        // std::thread::sleep(std::time::Duration::from_millis(10)); // e.g., 100 Hz loop
    }

    // This function seems redundant now as interrupt is set in new()
    // pub fn rail_init(&mut self) {
    //     self.sensor.update_reading();
    // }

    /// Calculates actuations based on current input and sends them.
    pub fn manual(&mut self) {
        let act = self.mix();
        self.actuator.actuate(act);
    }

    pub fn is_manual(&self) -> bool {
        self.manual
    }

    /// Utility function for mapping ranges (unused currently).
    fn map_range(
        &self,
        value: f32,
        from_low: f32,
        from_high: f32,
        to_low: f32,
        to_high: f32,
    ) -> f32 {
        // Clamp value to input range to prevent extrapolation issues
        let clamped_value = value.clamp(from_low, from_high);
        // Perform linear mapping
        to_low + (clamped_value - from_low) * (to_high - to_low) / (from_high - from_low)
    }
} // end impl Flappy

impl Blimp for Flappy {
    /// Allows external code (e.g., autonomous controller) to set inputs.
    fn update_input(&mut self, input: (f32, f32, f32)) {
        self.input = input;
        // Also update intended direction based on the new z input
        let (_, _, z) = input;
        let mut intended_direction: i8 = 0;
        if z > 0.1 {
            intended_direction = 1;
        } else if z < -0.1 {
            intended_direction = -1;
        }
        self.sensor.set_intended_rail_direction(intended_direction);
    }

    //This takes controller input and updates the blimp's state
    fn update(
        &mut self,
        state: Arc<Mutex<States>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData {
        self.sensor.update_altitude();
        let mut rng = rand::thread_rng(); // Use thread_rng() correctly
        //
        //let mut controller_battery = 0.0;
        while let Some(Event { event, id, .. }) = self.gilrs.next_event() {
            match self.gilrs.gamepad(id).power_info() {
                PowerInfo::Discharging(lvl) => self.controller_battery = lvl as f32,
                _ => (),
            };
            match event {
                gilrs::EventType::AxisChanged(axis, pos, _) => {
                    // Deadzone check could be added here
                    let deadzone = 0.05;
                    let value = if pos.abs() < deadzone { 0.0 } else { pos };

                    match axis {
                        gilrs::Axis::LeftStickY => self.input.0 = -value, // Invert Y axis if needed (depends on controller/preference)
                        gilrs::Axis::RightStickY => {
                            self.input.2 = -value; // Invert Y axis if needed ('z')
                            let current_z = -value; // Use the possibly inverted value

                            // --- Determine and Set Intended Rail Direction ---
                            let mut intended_direction: i8 = 0;
                            // Apply thresholds to avoid trying to count pulses when stick is near center
                            if current_z > 0.1 {
                                // Threshold for positive movement command
                                intended_direction = 1;
                            } else if current_z < -0.1 {
                                // Threshold for negative movement command
                                intended_direction = -1;
                            }
                            // Call the sensor method to update the shared direction state
                            self.sensor.set_intended_rail_direction(intended_direction);

                            // <<< --- DEBUG PRINT 1 --- >>>
                            // // println!(
                            //      //"[Update] z: {:.2}, Set intended_direction to: {}",
                            //      current_z, intended_direction
                            //  );
                            // <<< --- END DEBUG PRINT --- >>>
                        }

                        gilrs::Axis::RightStickX => self.input.1 = value, // Turning ('y')
                        _ => {}                                           // Ignore other axes
                    }
                }
                gilrs::EventType::ButtonPressed(button, _code) => match button {
                    gilrs::Button::Start => {
                        self.manual = !self.manual; // Toggle manual/auto mode
                        println!("Mode switched: Manual = {}", self.manual);
                        // Optionally reset inputs or stop actuators when switching mode
                    }
                    gilrs::Button::East => {
                        self.score = true;
                        self.score_time = std::time::Instant::now();
                    }
                    gilrs::Button::RightTrigger => {
                        println!("Going for ball");
                        *state.lock().unwrap() = States::Ball;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }

                    gilrs::Button::LeftTrigger => {
                        *state.lock().unwrap() = States::Goal;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }
                    gilrs::Button::West => {
                        //Capture image frame

                        *save_image = if *save_image { false } else { true };
                        println!("Toggle save image");
                    }
                    gilrs::Button::DPadDown => {
                        // Set the rail.pos to 0;
                        self.sensor.reset_rail_pos();
                    }
                    gilrs::Button::DPadRight => {
                        self.backing_out = true;
                        self.backing_out_timer = std::time::Instant::now();
                        // Timer for backing out
                    }

                    // Add other button actions if needed (e.g., Button::Select for calibration)
                    _ => {}
                },
                // Handle other events like connect/disconnect if needed
                gilrs::EventType::Connected => println!("Gamepad {:?} connected", id),
                gilrs::EventType::Disconnected => println!("Gamepad {:?} disconnected", id),
                _ => {}
            }
        }

        if self.score {
            let flap_freq = 1.1;

            if self.score_time.elapsed() > std::time::Duration::from_secs(10) {
                self.actuator.actuate(Actuations {
                    m1: self.neutral_angle_motor - 15.0,
                    m2: self.neutral_angle_motor - 15.0,
                    m3: self.neutral_angle_motor + 30.0,
                    m4: self.neutral_angle_motor + 30.0,
                    s1: self.oscillate_wing(-1.0, 0.7) as f32,
                    s2: self.oscillate_wing(1.0, flap_freq) as f32,
                    s3: self.oscillate_wing(-1.0, flap_freq) as f32,
                    s4: NEUTRAL_ANGLE,
                });
            } else {
                self.actuator.actuate(Actuations {
                    m1: self.neutral_angle_motor - 15.0,
                    m2: self.neutral_angle_motor - 15.0,
                    m3: self.neutral_angle_motor + 30.0,
                    m4: self.neutral_angle_motor + 30.0,
                    s1: self.oscillate_wing(-1.0, 0.7) as f32,
                    s2: self.oscillate_wing(1.0, flap_freq) as f32,
                    s3: NEUTRAL_ANGLE,
                    s4: NEUTRAL_ANGLE,
                });
            }

            if self.score_time.elapsed() > std::time::Duration::from_secs(20) {
                println!("Scoring stopped");
                self.score = false;

                self.actuator.actuate(Actuations {
                    m1: self.neutral_angle_motor,
                    m2: self.neutral_angle_motor,
                    m3: self.neutral_angle_motor,
                    m4: self.neutral_angle_motor,
                    s1: NEUTRAL_ANGLE,
                    s2: NEUTRAL_ANGLE,
                    s3: NEUTRAL_ANGLE,
                    s4: NEUTRAL_ANGLE,
                });
            }
        }
        let mut sensor_data = BlimpSensorData {
            battery: self.controller_battery,
            altitude: rng.gen_range(5.0..20.0),
            roll: rng.gen_range(-5.0..5.0),
            pitch: rng.gen_range(-5.0..5.0),
            yaw: rng.gen_range(0.0..360.0),
            tracking_error_x: rng.gen_range(-10.0..10.0),
            tracking_error_y: rng.gen_range(-10.0..10.0),
        };
        sensor_data
        //self.sensor.update_reading();
    }

    /// Takes current inputs (self.input) and calculates actuator commands (servos for Flappy).
    /// It checks rail limits before commanding the CG servo.
    fn mix(&mut self) -> Actuations {
        // Get current inputs (consider applying scaling/expo here if desired)
        let (x, y, z) = self.input; // x=LeftY (Fwd), y=RightX (Turn), z=RightY (Up/Down)

        // Define control parameters
        let flap_freq = 1.1; // Flapping frequency in Hz
        let cg_servo_neutral = 95.0; // Neutral position for the CG rail servo
        let wing_servo_neutral = 90.0;
        let tail_servo_neutral = 90.0;

        // Initialize servo outputs to neutral
        let mut s1_ac = wing_servo_neutral; // Servo 1 (Left Wing?)
        let mut s2_ac = wing_servo_neutral; // Servo 2 (Right Wing?)
        let mut s3_ac = tail_servo_neutral; // Servo 3 (Tail/Rudder?)
        let mut s_cg_ac = cg_servo_neutral; // Servo 4 (CG / Z-movement on rail)

        // --- Get Current Rail Position for Limit Checking ---
        let current_rail_pos = self.sensor.get_rail_pos();

        // --- Determine Z-Movement Permissions based on Rail Limits ---
        // Allow moving towards positive if not already AT or BEYOND the max limit
        let allow_z_positive = current_rail_pos <= MAX_RAIL_POS;
        // Allow moving towards negative if not already AT or BEYOND the min limit
        let allow_z_negative = current_rail_pos >= MIN_RAIL_POS;

        // --- Apply Controls ---

        // Forward Movement (x) - Left Stick Y
        // Apply threshold to prevent flapping when stick is centered
        if x > 0.1 {
            s1_ac = self.oscillate_wing(-1.0, flap_freq) as f32;
            s2_ac = self.oscillate_wing(1.0, flap_freq) as f32;
            // Flap both wings for forward motion
            // Direction -1 for one wing, 1 for the other for opposing motion
        } else if x < -0.1 {

            // Optional: Implement backward flapping or braking?
            // For now, no action if x is negative or neutral.
        }

        // Turning Movement (y) - Right Stick X
        // Apply threshold
        if y > 0.1 {
            // Turn Right: Oscillate left wing, hold right neutral, deflect tail right
            //s1_ac = self.oscillate_wing(-1.0, flap_freq) as f32;
            //s2_ac = wing_servo_neutral; // Hold right wing
            s3_ac = self.map_range(y, 0.2, 1.0, 90.0, 180.0); // Deflect tail fully right (adjust angle as needed)
        } else if y < -0.1 {
            // Turn Left: Oscillate right wing, hold left neutral, deflect tail left
            //s1_ac = wing_servo_neutral; // Hold left wing
            //s2_ac = self.oscillate_wing(1.0, flap_freq) as f32;
            s3_ac = self.map_range(y, -1.0, -0.2, 0.0, 90.0); // Deflect tail fully left (adjust angle as needed)
        }
        // Note: If moving forward AND turning, the flapping (`s1_ac`/`s2_ac`) might override turn logic here.
        // Consider combining logic if simultaneous forward + turn requires different wing behavior.

        // Vertical / Rail Servo Movement (z) - Right Stick Y
        // Apply threshold and check rail limits before commanding the SERVO
        if z > 0.2 && allow_z_positive {
            // Command servo to move towards positive end (e.g., 180 degrees)
            s_cg_ac = self.map_range(z, 0.2, 1.0, 95.0, 180.0);
        } else if z < -0.2 && allow_z_negative {
            // Command servo to move towards negative end (e.g., 0 degrees)
            s_cg_ac = self.map_range(z, -1.0, -0.2, 0.0, 95.0);
        } else {
            // Keep servo neutral if input is neutral OR movement direction is blocked by limits
            s_cg_ac = cg_servo_neutral;
            // Optional: Log if movement was attempted but blocked
            if (z > 0.1 && !allow_z_positive) || (z < -0.1 && !allow_z_negative) {
                println!(
                    "INFO: Z servo command ({:.2}) blocked by rail limit (Pos: {})",
                    z, current_rail_pos
                );
            }
        }
        // The actual rail position *counting* uses the `intended_direction` set in `update()`.
        // This `mix` function determines the *servo command* based on input and limits.

        // Construct the Actuations struct
        Actuations {
            m1: 0.0, // Flappy doesn't use motors
            m2: 0.0,
            m3: 0.0,
            m4: 0.0,
            s1: s1_ac,   // Left Wing Servo
            s2: s2_ac,   // Right Wing Servo
            s3: s3_ac,   // Tail Servo
            s4: s_cg_ac, // CG Rail Servo
        }
    }
} // end impl Blimp for Flappy

#[derive(Debug, Default, Clone, Copy)] // Added Clone, Copy for convenience
pub struct Actuations {
    m1: f32, // Motor speeds (0-180 angle equivalent)
    m2: f32,
    m3: f32,
    m4: f32,
    s1: f32, // Servo positions (0-180 degrees)
    s2: f32,
    s3: f32,
    s4: f32,
}

// --- SanoBlimp Implementation (Commented Out) ---
/*
pub struct SanoBlimp {
    state: Vec<f32>,
    input: (f32, f32, f32),
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    pub sensor: Sensors,
    manual: bool,
}

impl SanoBlimp {
    pub fn new(m1_mul: f32, m2_mul: f32, neutral_angle_motor: f32) -> Self {
        SanoBlimp {
            state: Vec::new(),
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(neutral_angle_motor),
            manual: true,
            sensor: Sensors::new(),
            score: false,
            score_time: std::time::Instant::now(),
            m1_mul,
            m2_mul,
            neutral_angle_motor,
            controller_battery: 0.0,
        }
    }

    pub fn run(&mut self) {
<<<<<<< HEAD
        self.update();
        if self.is_manual() {
            self.manual();
        }
=======
        // self.update();

>>>>>>> main
        //std::thread::sleep(std::time::Duration::from_millis(20));
    }

    pub fn manual(&mut self) {
        let act = self.mix();
        if self.score {
            return;
        }
        self.actuator.actuate(act);
    }

    pub fn is_manual(&self) -> bool {
        self.manual
    }
}

impl Blimp for SanoBlimp {
<<<<<<< HEAD
=======
    fn update(
        &mut self,
        state: Arc<Mutex<States>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData {
        self.sensor.update_altitude();
        //self.sensor.update_orientation();

        let mut rng = rand::thread_rng(); // Use thread_rng() correctly
        if self.score {
            self.actuator.actuate(Actuations {
                m1: self.neutral_angle_motor - 15.0,
                m2: self.neutral_angle_motor - 15.0,
                m3: self.neutral_angle_motor + 30.0,
                m4: self.neutral_angle_motor + 30.0,
                s1: NEUTRAL_ANGLE,
                s2: NEUTRAL_ANGLE,
                s3: NEUTRAL_ANGLE,
                s4: NEUTRAL_ANGLE,
            });

            if self.score_time.elapsed() > std::time::Duration::from_secs(3) {
                println!("Scoring stopped");
                self.score = false;

                self.actuator.actuate(Actuations {
                    m1: self.neutral_angle_motor,
                    m2: self.neutral_angle_motor,
                    m3: self.neutral_angle_motor,
                    m4: self.neutral_angle_motor,
                    s1: NEUTRAL_ANGLE,
                    s2: NEUTRAL_ANGLE,
                    s3: NEUTRAL_ANGLE,
                    s4: NEUTRAL_ANGLE,
                });
            }
        }

        //let mut controller_battery = 0.0;

        // TODO Move the controller input to a saperate thread
        while let Some(Event { event, id, .. }) = self.gilrs.next_event() {
            match self.gilrs.gamepad(id).power_info() {
                PowerInfo::Discharging(lvl) => self.controller_battery = lvl as f32,
                _ => (),
            };
            match event {
                gilrs::EventType::AxisChanged(axis, pos, _) => match axis {
                    gilrs::Axis::LeftStickY => self.input.0 = pos, // Forward/backward
                    gilrs::Axis::RightStickY => self.input.2 = pos, // Up/down
                    gilrs::Axis::RightStickX => self.input.1 = pos, // Turning
                    _ => {}
                },
                gilrs::EventType::ButtonPressed(button, code) => match button {
                    gilrs::Button::Start => {
                        self.manual = !self.manual;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }
                    gilrs::Button::East => {
                        self.score = true;
                        self.score_time = std::time::Instant::now();
                    }
                    gilrs::Button::RightTrigger => {
                        println!("Going for ball");
                        *state.lock().unwrap() = States::Ball;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }

                    gilrs::Button::LeftTrigger => {
                        *state.lock().unwrap() = States::Goal;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }
                    gilrs::Button::West => {
                        //Capture image frame

                        *save_image = if *save_image { false } else { true };
                        println!("Toggle save image");
                    }

                    _ => {}
                },
                _ => {}
            }
        }
        let mut sensor_data = BlimpSensorData {
            battery: self.controller_battery,
            altitude: self.sensor.altitude,
            roll: rng.gen_range(-5.0..5.0),
            pitch: rng.gen_range(-5.0..5.0),
            yaw: rng.gen_range(0.0..360.0),
            tracking_error_x: rng.gen_range(-10.0..10.0),
            tracking_error_y: rng.gen_range(-10.0..10.0),
        };

        sensor_data
    }

    fn mix(&mut self) -> Actuations {
        let (x, y, z) = self.input;

        //println!("{:?}", self.sensor.get_orientation());
        //

        //println!("{:?}", self.sensor.imu.gyro_data());

        let m1_mul = self.m1_mul;
        let m2_mul = self.m2_mul;

        let mut m1 = self.neutral_angle_motor - (x * 5.0) * m1_mul; // Map movement to a range (0-180°)
        let mut m2 = self.neutral_angle_motor - (x * 5.0) * m2_mul;

        //m1 += self.sensor.imu.gyro_data().unwrap().y;
        //m2 -= self.sensor.imu.gyro_data().unwrap().y;

        if z > 0.1 {
            m1 -= z * 6.0;
            m2 -= z * 6.0;
        }
        if z < -0.1 {
            m1 += z * 6.0;
            m2 += z * 6.0;
        }

        let mut s3 = NEUTRAL_ANGLE - (90.0 * z);
        let mut s4 = NEUTRAL_ANGLE + (90.0 * z);

        if y < -0.1 {
            m1 += y * 6.0;
            m2 -= y * 6.0;
        }
        if y > 0.1 {
            m1 += y * 6.0;
            m2 -= y * 6.0;
        }

        if z < -0.2 || z > 0.2 {
            if y < -0.2 {
                s4 = NEUTRAL_ANGLE;
            } else if y > 0.2 {
                s3 = NEUTRAL_ANGLE;
            }
        }

        Actuations {
            m1: m1.clamp(0.0, 180.0),
            m2: m2.clamp(0.0, 180.0),
            m3: self.neutral_angle_motor,
            m4: self.neutral_angle_motor,
            s1: NEUTRAL_ANGLE, // Keep neutral if not controlled
            s2: NEUTRAL_ANGLE,
            s3: s3.clamp(0.0, 180.0),
            s4: s4.clamp(0.0, 180.0),
        }
    }

>>>>>>> main
    fn update_input(&mut self, input: (f32, f32, f32)) {
        self.input = input;
        // SanoBlimp might need to update rail direction too if it uses the rail
        // let (_, _, z) = input;
        // let mut intended_direction: i8 = 0;
        // if z > 0.1 { intended_direction = 1; }
        // else if z < -0.1 { intended_direction = -1; }
        // self.sensor.set_intended_rail_direction(intended_direction);
    }

    fn update(&mut self) {
        // SanoBlimp would update its sensors
        // self.sensor.update_altitude();
        // self.sensor.update_orientation(); // If IMU is used

        // Process gamepad input
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
             self.active_gamepad = Some(id);
             match event {
                 gilrs::EventType::AxisChanged(axis, pos, _) => {
                      let deadzone = 0.05;
                      let value = if pos.abs() < deadzone { 0.0 } else { pos };
                      match axis {
                          gilrs::Axis::LeftStickY => self.input.0 = -value, // Forward/backward
                          gilrs::Axis::RightStickY => {
                              self.input.2 = -value; // Up/down ('z')
                              // Update rail direction if SanoBlimp uses it
                              // let current_z = -value;
                              // let mut intended_direction: i8 = 0;
                              // if current_z > 0.1 { intended_direction = 1; }
                              // else if current_z < -0.1 { intended_direction = -1; }
                              // self.sensor.set_intended_rail_direction(intended_direction);
                          }
                          gilrs::Axis::RightStickX => self.input.1 = value, // Turning ('y')
                          _ => {}
                      }
                 },
                 gilrs::EventType::ButtonPressed(button, _code) => match button {
                     gilrs::Button::Start => self.manual = !self.manual,
                     _ => {}
                 },
                 _ => {}
             }
        }
    }

    // Mixing logic for SanoBlimp (example using motors and servos differently)
    fn mix(&mut self) -> Actuations {
        let (x, y, z) = self.input; // x=Fwd, y=Turn, z=Alt

        // Example mixing - This needs to be adapted based on SanoBlimp's design
        let throttle_scale = 50.0; // Adjust scaling as needed
        let yaw_scale = 30.0;
        let altitude_scale = 40.0;
        let pitch_scale = 45.0; // For elevator servos

        let base_throttle = NEUTRAL_ANGLE_MOTOR; // e.g., 80 degrees

        // Calculate motor outputs
        let mut m1_angle = base_throttle + x * throttle_scale - y * yaw_scale + z * altitude_scale; // Front-Left?
        let mut m2_angle = base_throttle + x * throttle_scale + y * yaw_scale + z * altitude_scale; // Front-Right?
        let mut m3_angle = base_throttle + x * throttle_scale - y * yaw_scale - z * altitude_scale; // Rear-Left?
        let mut m4_angle = base_throttle + x * throttle_scale + y * yaw_scale - z * altitude_scale; // Rear-Right?

        // Add IMU stabilization if available and desired
        // if let Ok(gyro) = self.sensor.imu.gyro_data() {
        //    m1_angle += gyro.y * some_gyro_scale; // Example stabilization
        //    m2_angle -= gyro.y * some_gyro_scale;
        // }

        // Calculate servo outputs (example: using z for elevator, y for rudder)
        let elevator_angle = NEUTRAL_ANGLE + z * pitch_scale; // s1, s2 as elevators?
        let rudder_angle = NEUTRAL_ANGLE + y * yaw_scale;    // s3 as rudder?
        let cg_rail_angle = NEUTRAL_ANGLE; // s4 unused or for rail? Needs logic like Flappy if used.

        // Get rail position and apply limits if SanoBlimp uses the rail for s4
        // let current_rail_pos = self.sensor.get_rail_pos();
        // let allow_z_positive = current_rail_pos < MAX_RAIL_POS;
        // let allow_z_negative = current_rail_pos > MIN_RAIL_POS;
        // if z > 0.1 && allow_z_positive { cg_rail_angle = 180.0; }
        // else if z < -0.1 && allow_z_negative { cg_rail_angle = 0.0; }
        // else { cg_rail_angle = 95.0; /* Neutral */ }


        Actuations {
            m1: m1_angle.clamp(0.0, 180.0),
            m2: m2_angle.clamp(0.0, 180.0),
            m3: m3_angle.clamp(0.0, 180.0),
            m4: m4_angle.clamp(0.0, 180.0),
            s1: elevator_angle.clamp(0.0, 180.0), // Example: Servo 1 as elevator
            s2: elevator_angle.clamp(0.0, 180.0), // Example: Servo 2 mirrored elevator
            s3: rudder_angle.clamp(0.0, 180.0),   // Example: Servo 3 as rudder
            s4: cg_rail_angle.clamp(0.0, 180.0), // Example: Servo 4 for rail or unused
        }
    }
}
*/
