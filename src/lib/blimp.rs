// use bme280::i2c::BME280;
use bno055::{BNO055OperationMode, Bno055};
use core::f64;
use gilrs::{Button, Event, GamepadId, Gilrs};
use linux_embedded_hal::{Delay, I2cdev, SpidevDevice};
use mint::{EulerAngles, Quaternion};
use pwm_pca9685::{Address, Channel, Pca9685};
use rppal::gpio::{self, Gpio, InputPin, Trigger};
use std::fs::{File, OpenOptions};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{self, Duration};
use tokio::spawn;
use tokio::sync::mpsc::{UnboundedReceiver, unbounded_channel}; //Arc Mutex is added for the rail

use bme280::spi::BME280;
use shared_bus::{self, BusManager, I2cProxy, NullMutex}; // Import the shared-bus crate

use super::object_detection::Detection;
use std::io::{self, Write};

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
// const NEUTRAL_ANGLE_MOTOR: f32 = 83.0; // Neutral position for motors and servos
const NEUTRAL_ANGLE_MOTOR: f32 = 80.0; // Neutral position for motors and servos

// Constants for the ISA barometric formula (altitude in meters, pressure in Pascals)
const ALTITUDE_FACTOR: f32 = 44330.0; // Corresponds to meters
const ISA_EXPONENT: f32 = 1.0 / 5.255; // Approximately 0.190294957
const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0; // Pa

// Define Rail Limits (adjust these values as needed)
const MAX_RAIL_POS: i8 = 10;
const MIN_RAIL_POS: i8 = -10;

/// Every blimp needs the following trait
pub trait Blimp {
    fn update(&mut self);

    fn mix(&mut self) -> Actuations;
    fn update_input(&mut self, input: (f32, f32, f32));
}

pub struct Sensors {
    pressure: f32,
    //rail_pos: i8,
    //rail_direction: i8,
    rail_pos: Arc<Mutex<i8>>, // Use Arc<Mutex> for shared state
    rail_pin_trigger: Arc<Mutex<InputPin>>, // Keep using Arc<Mutex> for pin
    altitude: f32,
    euler_angles: EulerAngles<f32, ()>, // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    quaternion: Quaternion<f32>,        // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);
    //pub imu: Bno055<I2cdev>,
    bme: BME280<SpidevDevice>,
    //ground_pressure: f32,
    delay: Delay,
    //dev: Arc<Mutex<I2cdev>>,
}

impl Sensors {
    pub fn new() -> Self {
        // Read the last rail data from a file
        //let mut file = File::open("rail.pos").unwrap();
        let initial_rail_pos = match std::fs::read_to_string("rail.pos") {
            Ok(content) => content.trim().parse::<i8>().unwrap_or(0), // Default to 0 on error/empty
            Err(_) => 0, // Default to 0 if file doesn't exist
        };
        println!("Initialized rail position: {}", initial_rail_pos);

        //New I2C device from linux to path /dev - directory in linux
        // /i2c-1 specific i2c bus
        let dev = I2cdev::new("/dev/i2c-1").unwrap();

        let mut delay = Delay {};

        //let mut imu = Bno055::new(dev).with_alternative_address();
        //imu.init(&mut delay)
        //    .expect("An error occurred while building the IMU");

        //imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        //    .expect("An error occurred while setting the IMU mode");

        //let mut status = imu.get_calibration_status().unwrap();
        //println!("The IMU's calibration status is: {:?}", status);

        let mut spi = SpidevDevice::open("/dev/spidev0.1").unwrap();
        let mut bme280 = BME280::new(spi).unwrap();

        // Initialize the GPIO pin within an Arc<Mutex>
        let gpio = Gpio::new().expect("Failed to initialize GPIO");
        let input_pin = gpio
            .get(23) // Using GPIO Pin 23 as before
            .expect("Failed to get GPIO pin 23")
            .into_input_pullup(); // Use pull-up resistor
        //bme280.init(&mut delay).unwrap();
        //
        //let measurements = bme280.measure(&mut delay).unwrap();
        //
        Sensors {
            pressure: 0.0,
            rail_pos: Arc::new(Mutex::new(initial_rail_pos)), // Initialize Arc<Mutex>
            rail_pin_trigger: Arc::new(Mutex::new(input_pin)), // Initialize Arc<Mutex> for pin
            altitude: 0.0,
            euler_angles: EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]),
            quaternion: Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]),
            bme: bme280,
            delay,
        }
    }

    // Modify get_rail_pos to work with Arc<Mutex>
    pub fn get_rail_pos(&self) -> i8 {
        match self.rail_pos.lock() {
            Ok(pos) => *pos, // Dereference the MutexGuard to get the i8 value
            Err(e) => {
                eprintln!("ERROR: Failed to lock rail_pos mutex: {:?}", e);
                0 // Return default on error
            }
        }
        // Note: Reading the file here is redundant if the Arc<Mutex> holds the state
        // let content = std::fs::read_to_string("rail.pos").unwrap();
        // let rail_pos = match content.parse::<i8>() { /* ... */ };
        // rail_pos
    }
    pub fn update_reading(&mut self) {
        // Clone the Arcs to move into the closure
        let rail_pos_clone = Arc::clone(&self.rail_pos);
        let pin_clone = Arc::clone(&self.rail_pin_trigger);

        // Lock the pin mutex once to set the interrupt
        let mut locked_pin = pin_clone.lock().expect("Failed to lock rail pin mutex");

        locked_pin
            .set_async_interrupt(
                Trigger::RisingEdge,             // Trigger on rising edge
                Some(Duration::from_millis(20)), // Debounce timeout (adjust if needed)
                move |_level| {
                    // Closure now takes the level (unused here)
                    // --- Start of Interrupt Handler ---

                    // Lock the shared position state
                    let mut current_pos = rail_pos_clone
                        .lock()
                        .expect("Failed to lock rail_pos in interrupt");

                    // **** ASSUMPTION: Rising edge means move in positive direction ****
                    // **** You MUST adapt this if you need bidirectional counting ****
                    let direction_increment: i8 = 1; // Hardcoded direction

                    // Perform the count update
                    *current_pos += direction_increment;

                    // Clamp the value to prevent exceeding limits (optional but good practice)
                    *current_pos = current_pos.clamp(MIN_RAIL_POS - 1, MAX_RAIL_POS + 1); // Allow one step past for detection

                    // Print the updated position
                    println!("Rail Interrupt! New Position: {}", *current_pos);

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

                    // --- End of Interrupt Handler ---
                },
            )
            .expect("Failed to set async interrupt");

        println!("Rail position interrupt handler set up on pin 23.");
        // Drop the lock on the pin after setting the interrupt
        // The lock is implicitly dropped when locked_pin goes out of scope
    }

    pub fn update_altitude(&mut self) {
        // --- Get Current Pressure ---
        // It's better practice to handle the Result properly instead of unwrapping,
        // especially in embedded systems where panics can be problematic.
        let measurement = match self.bme.measure(&mut self.delay) {
            Ok(m) => m,
            Err(_e) => {
                // Handle the error appropriately: log it, set a default/error altitude, etc.
                // For example:
                // rprintln!("Error reading BME sensor: {:?}", e); // If using rtt-target
                self.altitude = f32::NAN; // Set altitude to Not a Number to indicate an error
                return; // Exit the function if measurement failed
            }
        };
        // Assuming the pressure reading from the driver is already f32 in Pascals.
        // If it's an integer (like u32), you'll need to cast it: measurement.pressure as f32
        let current_pressure_pa = measurement.pressure;

        // --- Get Reference Pressure ---
        // Use the stored ground_pressure as the reference (P₀).
        // Make sure it's a valid value.
        //let reference_pressure_pa = self.ground_pressure;
        let reference_pressure_pa = 0.0;
        if reference_pressure_pa <= 0.0 {
            // rprintln!("Invalid reference pressure: {}", reference_pressure_pa);
            self.altitude = f32::NAN; // Indicate error
            return;
        }

        // --- Calculate Altitude ---
        // Formula: Altitude(m) = 44330.0 * [1 - (P / P₀)^(1 / 5.255)]
        // Where P = current pressure, P₀ = reference pressure

        // Calculate pressure ratio (P / P₀)
        let pressure_ratio = current_pressure_pa / reference_pressure_pa;

        // Calculate altitude in meters
        let altitude_meters = ALTITUDE_FACTOR * (1.0 - pressure_ratio.powf(ISA_EXPONENT));

        // --- Store Result ---
        self.altitude = altitude_meters;

        //let ground_pressure = self.ground_pressure;
        //self.bme.measure(&mut self.delay).unwrap().pressure;
    }

    // pub fn update_orientation(&mut self) {
    //     match self.imu.quaternion() {
    //         Ok(val) => {
    //             self.quaternion = val;
    //         }
    //         Err(e) => {}
    //     }
    // }

    pub fn get_orientation(&self) -> Quaternion<f32> {
        self.quaternion
    }
    pub fn get_altitude(&self) -> f32 {
        self.altitude
    }
}

pub struct PCAActuator {
    pwm: Pca9685<I2cdev>,
}

impl PCAActuator {
    pub fn new() -> Self {
        let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device");
        let address = Address::default();
        // let address = Address::from(0x55);
        let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");
        pwm.set_prescale(100).expect("Failed to set prescale");
        pwm.enable().expect("Failed to enable PCA9685");

        let mut actuator = PCAActuator { pwm };
        actuator.init_escs();
        actuator
    }

    /// Convert throttle value (0-180°) to PWM pulse width and send to ESCs
    fn set_motor_speed(&mut self, channel: Channel, angle: f32) {
        let pulse_width = MIN_PULSE + ((angle / 180.0) * (MAX_PULSE - MIN_PULSE));
        let on = 0;
        let off = ((pulse_width * PWM_FREQUENCY_MOTOR * 4096.0 * 1e-6) as u16).clamp(0, 4095);

        self.pwm.set_channel_on_off(channel, on, off).unwrap();
    }

    /// Convert throttle value (0-180°) to PWM pulse width and send to ESCs
    fn set_servo_speed(&mut self, channel: Channel, angle: f32) {
        let pulse_width = MIN_PULSE_SERVO + ((angle / 180.0) * (MAX_PULSE_SERVO - MIN_PULSE_SERVO));
        let on = 0;
        let off = ((pulse_width * PWM_FREQUENCY * 4096.0 * 1e-6) as u16).clamp(0, 4095);

        self.pwm.set_channel_on_off(channel, on, off).unwrap();
    }

    /// ESC initialization sequence for BLHeli_S
    pub fn init_escs(&mut self) {
        println!("Initializing ESCs...");

        // Step 1: Send max throttle to enter programming mode
        println!("Sending max throttle (arming sequence)");
        self.set_motor_speed(Channel::C0, 180.0);
        self.set_motor_speed(Channel::C1, 180.0);
        self.set_motor_speed(Channel::C2, 180.0);
        self.set_motor_speed(Channel::C3, 180.0);
        sleep(Duration::from_secs(1));

        // Step 2: Send min throttle to arm the ESCs
        println!("Sending min throttle (arming)");
        self.set_motor_speed(Channel::C0, 0.0);
        self.set_motor_speed(Channel::C1, 0.0);
        self.set_motor_speed(Channel::C2, 0.0);
        self.set_motor_speed(Channel::C3, 0.0);

        sleep(Duration::from_secs(1));

        // Step 3: Move to neutral throttle (ready to receive commands)
        println!("Setting ESCs to neutral");
        self.set_motor_speed(Channel::C0, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_speed(Channel::C1, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_speed(Channel::C2, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_speed(Channel::C3, NEUTRAL_ANGLE_MOTOR);

        sleep(Duration::from_secs(2));

        println!("ESCs initialized!");
    }

    pub fn actuate(&mut self, act: Actuations) {
        self.set_motor_speed(Channel::C0, act.m1);
        self.set_motor_speed(Channel::C1, act.m2);
        self.set_motor_speed(Channel::C2, act.m3);
        self.set_motor_speed(Channel::C3, act.m4);

        self.set_servo_speed(Channel::C4, act.s1);
        self.set_servo_speed(Channel::C5, act.s2);
        self.set_servo_speed(Channel::C6, act.s3);
        self.set_servo_speed(Channel::C7, act.s4);
    }
}

pub struct SanoBlimp {
    state: Vec<f32>,
    input: (f32, f32, f32),
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    pub sensor: Sensors,
    manual: bool,
}

pub struct Flappy {
    state: Vec<f32>,
    input: (f32, f32, f32),
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    manual: bool,
    pub sensor: Sensors, // Holds the sensors including rail_pos Arc
}

impl Flappy {
    pub fn new() -> Self {
        let mut sensors = Sensors::new();
        // Call update_reading to set up the interrupt *after* Sensors is created
        sensors.update_reading(); // Setup the interrupt handler

        Flappy {
            state: Vec::new(),
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(),
            manual: true,
            sensor: sensors, // Move the sensors struct here
        }
    }

    fn oscillate_wing(&self, direction: f64, freq: f64) -> f64 {
        90.0 + 90.0
            * direction
            * f64::sin(
                (2.0 * freq * f64::consts::PI as f64)
                    * time::SystemTime::now()
                        .duration_since(time::SystemTime::UNIX_EPOCH)
                        .unwrap()
                        .as_secs_f64(),
            )
        //angle = self.map_value(angle, 0, 180, 5, 10)
    }

    pub fn run(&mut self) {
        self.update();

        //std::thread::sleep(std::time::Duration::from_millis(20));
    }

    pub fn rail_init(&mut self) {
        self.sensor.update_reading();
    }

    pub fn manual(&mut self) {
        let act = self.mix();
        self.actuator.actuate(act);
    }

    pub fn is_manual(&self) -> bool {
        self.manual
    }
    fn map_range(
        &self,
        value: f32,
        from_low: f32,
        from_high: f32,
        to_low: f32,
        to_high: f32,
    ) -> f32 {
        (value - from_low) / (from_high - from_low) * (to_high - to_low) + to_low
    }
}

impl Blimp for Flappy {
    fn update_input(&mut self, input: (f32, f32, f32)) {
        self.input = input;
    }
    //This takes controller input and updates the blimp's state
    fn update(&mut self) {
        while let Some(Event { event, .. }) = self.gilrs.next_event() {
            match event {
                gilrs::EventType::AxisChanged(axis, pos, _) => match axis {
                    gilrs::Axis::LeftStickY => self.input.0 = pos, // Forward/backward
                    gilrs::Axis::RightStickY => self.input.2 = pos, // Up/down
                    gilrs::Axis::RightStickX => self.input.1 = pos, // Turning
                    _ => {}
                },
                gilrs::EventType::ButtonPressed(button, code) => match button {
                    gilrs::Button::Start => self.manual = !self.manual,
                    _ => {}
                },
                _ => {}
            }
        }
        //self.sensor.update_reading();
    }

    //Takes in inputs and converts them into actuations
    //Options Teleop to autonomy
    //in my temrinal type "scp targets/Georgie
    fn mix(&mut self) -> Actuations {
        let (x, y, z) = self.input; // x=LeftY (Forward), y=RightX (Turn), z=RightY (Up/Down)
        let freq = 1.1;

        let mut s1_ac = 90.0; // Servo 1 (Left Wing?)
        let mut s2_ac = 90.0; // Servo 2 (Right Wing?)
        let mut s3_ac = 90.0; // Servo 3 (Tail/Rudder?)
        let mut s_cg_ac = 95.0; // Servo 4 (CG / Z-movement on rail) - Default slightly off center

        // --- Get Current Rail Position ---
        let current_rail_pos = self.sensor.get_rail_pos();

        // --- Determine Z-Movement Permissions based on Rail Limits ---
        let mut allow_z_positive = true; // Allow moving towards positive counts (assume this is z > 0.1)
        let mut allow_z_negative = true; // Allow moving towards negative counts (assume this is z < -0.1)

        if current_rail_pos >= MAX_RAIL_POS {
            allow_z_positive = false; // At positive end, block further positive Z movement
            // Optional: Print warning only once or less frequently if needed
            // println!("WARN: Rail at positive limit ({})! Blocking positive Z.", current_rail_pos);
        }
        if current_rail_pos <= MIN_RAIL_POS {
            allow_z_negative = false; // At negative end, block further negative Z movement
            // Optional: Print warning only once or less frequently if needed
            // println!("WARN: Rail at negative limit ({})! Blocking negative Z.", current_rail_pos);
        }

        // --- Apply Controls ---

        // Forward Movement (x)
        if x > 0.1 {
            s1_ac = self.oscillate_wing(-1.0, freq);
            s2_ac = self.oscillate_wing(1.0, freq);
        }

        // Turning Movement (y)
        if y > 0.2 {
            // Turn Right?
            s2_ac = 90.0;
            s1_ac = self.oscillate_wing(-1.0, freq);
            s3_ac = 180.0;
        }
        if y < -0.2 {
            // Turn Left?
            s1_ac = 90.0;
            s2_ac = self.oscillate_wing(1.0, freq);
            s3_ac = 0.0;
        }

        // Vertical / Rail Movement (z) - Apply Limits Check
        if z > 0.1 && allow_z_positive {
            // Assuming z > 0.1 corresponds to moving towards positive rail counts
            s_cg_ac = 180.0;
        } else if z < -0.1 && allow_z_negative {
            // Assuming z < -0.1 corresponds to moving towards negative rail counts
            s_cg_ac = 0.0;
        } else {
            // If input is neutral OR movement is blocked by limits, keep CG servo neutral
            s_cg_ac = 95.0; // Or 90.0 if that's the true neutral for this servo
            if (z > 0.1 && !allow_z_positive) || (z < -0.1 && !allow_z_negative) {
                // Optionally print that movement was actively blocked
                println!(
                    "INFO: Z movement blocked by rail limit (Pos: {}, Z: {:.2})",
                    current_rail_pos, z
                );
            }
        }

        Actuations {
            m1: 0.0,
            m2: 0.0,
            m3: 0.0,
            m4: 0.0,
            s1: s1_ac as f32,
            s2: s2_ac as f32,
            s3: s3_ac as f32,
            s4: s_cg_ac as f32, // Updated CG servo value
        }
    }
}

#[derive(Debug, Default)]
pub struct Actuations {
    m1: f32, // Motor speeds (0-180° where 90° is neutral)
    m2: f32,
    m3: f32,
    m4: f32,
    s1: f32, // Servo positions (0-180°)
    s2: f32,
    s3: f32,
    s4: f32,
}

// impl SanoBlimp {
//     pub fn new() -> Self {
//         SanoBlimp {
//             state: Vec::new(),
//             input: (0.0_f32, 0.0_f32, 0.0_f32),
//             gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
//             active_gamepad: None,
//             actuator: PCAActuator::new(),
//             manual: true,
//             sensor: Sensors::new(),
//         }
//     }

//     pub fn run(&mut self) {
//         self.update();

//         //std::thread::sleep(std::time::Duration::from_millis(20));
//     }

//     pub fn manual(&mut self) {
//         let act = self.mix();
//         self.actuator.actuate(act);
//     }

//     pub fn is_manual(&self) -> bool {
//         self.manual
//     }
// }

// impl Blimp for SanoBlimp {
//     fn update(&mut self) {
//         self.sensor.update_altitude();
//         self.sensor.update_orientation();
//         // TODO Move the controller input to a saperate thread
//         while let Some(Event { event, .. }) = self.gilrs.next_event() {
//             match event {
//                 gilrs::EventType::AxisChanged(axis, pos, _) => match axis {
//                     gilrs::Axis::LeftStickY => self.input.0 = pos, // Forward/backward
//                     gilrs::Axis::RightStickY => self.input.2 = pos, // Up/down
//                     gilrs::Axis::RightStickX => self.input.1 = pos, // Turning
//                     _ => {}
//                 },
//                 gilrs::EventType::ButtonPressed(button, code) => match button {
//                     gilrs::Button::Start => self.manual = !self.manual,
//                     _ => {}
//                 },
//                 _ => {}
//             }
//         }
//     }

//     fn mix(&mut self) -> Actuations {
//         let (x, y, z) = self.input;

//         //println!("{:?}", self.sensor.get_orientation());
//         //

//         println!("{:?}", self.sensor.imu.gyro_data());

//         let m1_mul = 1.3;
//         let m2_mul = 1.0;

//         let mut m1 = NEUTRAL_ANGLE_MOTOR - (x * 5.0) * m1_mul; // Map movement to a range (0-180°)
//         let mut m2 = NEUTRAL_ANGLE_MOTOR - (x * 5.0) * m2_mul;

//         m1 += self.sensor.imu.gyro_data().unwrap().y;
//         m2 -= self.sensor.imu.gyro_data().unwrap().y;

//         if z > 0.1 {
//             m1 -= z * 6.0;
//             m2 -= z * 6.0;
//         }
//         if z < -0.1 {
//             m1 += z * 6.0;
//             m2 += z * 6.0;
//         }

//         let mut s3 = NEUTRAL_ANGLE - (90.0 * z);
//         let mut s4 = NEUTRAL_ANGLE + (90.0 * z);

//         if y < -0.1 {
//             m1 += y * 6.0;
//             m2 -= y * 6.0;
//         }
//         if y > 0.1 {
//             m1 += y * 6.0;
//             m2 -= y * 6.0;
//         }

//         if z < -0.2 || z > 0.2 {
//             if y < -0.2 {
//                 s4 = NEUTRAL_ANGLE;
//             } else if y > 0.2 {
//                 s3 = NEUTRAL_ANGLE;
//             }
//         }

//         Actuations {
//             m1: m1.clamp(0.0, 180.0),
//             m2: m2.clamp(0.0, 180.0),
//             m3: m1.clamp(0.0, 180.0),
//             m4: m2.clamp(0.0, 180.0),
//             s1: NEUTRAL_ANGLE, // Keep neutral if not controlled
//             s2: NEUTRAL_ANGLE,
//             s3: s3.clamp(0.0, 180.0),
//             s4: s4.clamp(0.0, 180.0),
//         }
//     }

//     fn update_input(&mut self, input: (f32, f32, f32)) {
//         self.input = input;
//     }
// }
