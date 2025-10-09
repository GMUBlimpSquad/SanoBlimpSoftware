use ::bme280::Measurements;
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
use std::thread::sleep;
use std::time::{self, Duration};
use tokio::spawn;
use tokio::sync::mpsc::{UnboundedReceiver, unbounded_channel};

use super::object_detection::Detection;
use shared_bus::{self, BusManager, I2cProxy, NullMutex}; // Import the shared-bus crate
use std::io::{self, Write};
use std::sync::{Arc, Mutex};

use crate::driver::bme280::{self, Bme280, Bme280Measurements};
use crate::{BlimpSensorData, lib::base_station::communication::*};

// TODO Make these a config file, and will depend on the blimp's hardware

const PWM_FREQUENCY: f32 = 60.0; // Hz for motors and servos
const MIN_PULSE_SERVO: f32 = 500.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE_SERVO: f32 = 2500.0; // Maximum pulse width in µs (Full throttle)
const NEUTRAL_ANGLE: f32 = 90.0; // Neutral position for motors and servos
//
const PWM_FREQUENCY_MOTOR: f32 = 63.0; // Hz for motors and servos
const MIN_PULSE: f32 = 1000.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE: f32 = 2000.0; // Maximum pulse width in µs (Full throttle)
const MID_PULSE: f32 = 1500.0; // Neutral (90° equivalent)
// const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos
//const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos

// Constants for the ISA barometric formula (altitude in meters, pressure in Pascals)
const ALTITUDE_FACTOR: f32 = 44330.0; // Corresponds to meters
const ISA_EXPONENT: f32 = 1.0 / 5.255; // Approximately 0.190294957
const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0; // Pa

/// Every blimp needs the following trait
pub trait Blimp {
    fn update(
        &mut self,
        state: Arc<Mutex<BlimpStates>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData;

    fn mix(&mut self) -> Actuations;
    fn update_input(&mut self, input: (f32, f32, f32));
}

pub struct Sensors {
    pressure: f32,
    rail_pos: i8,
    rail_direction: i8,
    rail_pin: Arc<Mutex<InputPin>>,
    altitude: f32,
    euler_angles: EulerAngles<f32, ()>, // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    quaternion: Quaternion<f32>,        // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);
    pub imu: Bno055<I2cdev>,
    // bme: Bme280,
    ground_altitude: f32,
    delay: Delay,
    //dev: Arc<Mutex<I2cdev>>,
}

impl Sensors {
    pub fn new(version: u8) -> Self {
        // Read the last rail data from a file
        //let mut file = File::open("rail.pos").unwrap();
        let content = std::fs::read_to_string("rail.pos").unwrap();

        let rail_pos = match content.parse::<i8>() {
            Ok(res) => res,
            Err(e) => 0,
        };

        let dev = I2cdev::new("/dev/i2c-1").unwrap();
        //
        let mut delay = Delay {};
        //
        let mut imu = Bno055::new(dev).with_alternative_address();

        match imu.init(&mut delay) {
            Ok(a) => {
                imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
                    .expect("An error occurred while setting the IMU mode");
            }
            Err(e) => {
                println!("Error initializing imu");
            }
        }

        //let mut status = imu.get_calibration_status().unwrap();
        //println!("The IMU's calibration status is: {:?}", status);

        // let mut bme280 = if version == 1 {
        //     Bme280::new_spi("/dev/spidev0.0", bme280::DEFAULT_SEA_LEVEL_HPA).unwrap()
        // } else {
        //     Bme280::new_i2c("/dev/i2c-1", 0x76, bme280::DEFAULT_SEA_LEVEL_HPA).unwrap()
        // };
        let meas = 0;

        Sensors {
            //dev,
            pressure: 0.0,
            rail_pos,
            rail_direction: 1,
            rail_pin: Arc::new(Mutex::new(
                Gpio::new().unwrap().get(23).unwrap().into_input_pullup(),
            )),
            altitude: 0.0,
            euler_angles: EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]),
            quaternion: Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]),
            imu: imu,
            // bme: bme280,
            delay,
            ground_altitude: 0.0,
            //ground_pressure: measurements.pressure,
        }
    }

    pub fn update_reading(&mut self) {
        // Create a channel that receives the gpio reading and updates the pos

        let inpin = self.rail_pin.clone();

        let mut pin = inpin.lock().unwrap();

        pin.set_async_interrupt(
            Trigger::RisingEdge,
            Some(Duration::from_millis(5)),
            move |event| {
                let content = std::fs::read_to_string("rail.pos").unwrap();

                let mut rail_pos = match content.parse::<i8>() {
                    Ok(res) => res,
                    Err(e) => 0,
                };

                //rail_pos += self.rail_direction;

                let mut file = OpenOptions::new().write(true).open("rail.pos").unwrap();
                writeln!(file, "{}", rail_pos);
            },
        )
        .unwrap();
    }

    pub fn update_altitude(&mut self) {
        // --- Get Current Pressure ---
        // It's better practice to handle the Result properly instead of unwrapping,
        // especially in embedded systems where panics can be problematic.
        // let measurement = match self.bme.read() {
        //     Ok(m) => m,
        //     Err(e) => {
        //         // Handle the error appropriately: log it, set a default/error altitude, etc.
        //         // For example:
        //         eprintln!("Error reading BME sensor: {:?}", e); // If using rtt-target
        //         self.altitude = f32::NAN; // Set altitude to Not a Number to indicate an error
        //         return; // Exit the function if measurement failed
        //     }
        // };
        // //println!("{:?}", measurement);
        // --- Store Result ---
        //self.altitude = measurement.altitude_m - self.ground_altitude;
    }

    pub fn update_orientation(&mut self) {
        match self.imu.quaternion() {
            Ok(val) => {
                self.quaternion = val;
            }
            Err(e) => {}
        }
    }

    pub fn get_orientation(&self) -> Quaternion<f32> {
        self.quaternion
    }
    pub fn get_altitude(&self) -> f32 {
        self.altitude
    }

    pub fn get_rail_pos(&self) -> i8 {
        let content = std::fs::read_to_string("rail.pos").unwrap();

        let rail_pos = match content.parse::<i8>() {
            Ok(res) => res,
            Err(e) => 0,
        };

        rail_pos
    }
}

pub struct PCAActuator {
    pwm: Pca9685<I2cdev>,
    neutral_angle_motor: f32,
}

impl PCAActuator {
    pub fn new(neutral_angle_motor: f32, blimp_version: u8) -> Self {
        let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device");
        let address = if blimp_version == 1 {
            Address::default()
        } else {
            Address::from(0x40)
        };
        let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");

        let osc_clock = 25_000_000.0;
        // let prescale_val = 100;
        let prescale_val = (osc_clock / (4096.0 * PWM_FREQUENCY_MOTOR)).round() - 1.0;

        pwm.set_prescale(prescale_val as u8)
            .expect("Failed to set prescale");
        pwm.enable().expect("Failed to enable PCA9685");

        let mut actuator = PCAActuator {
            pwm,
            neutral_angle_motor,
        };
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
        sleep(Duration::from_millis(100));

        // Step 2: Send min throttle to arm the ESCs
        println!("Sending min throttle (arming)");
        self.set_motor_speed(Channel::C0, 0.0);
        self.set_motor_speed(Channel::C1, 0.0);
        self.set_motor_speed(Channel::C2, 0.0);
        self.set_motor_speed(Channel::C3, 0.0);

        sleep(Duration::from_millis(100));

        // Step 3: Move to neutral throttle (ready to receive commands)
        println!("Setting ESCs to neutral");
        self.set_motor_speed(Channel::C0, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C1, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C2, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C3, self.neutral_angle_motor);

        sleep(Duration::from_millis(100));

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
    pub score: bool,
    pub score_time: std::time::Instant,
    m1_mul: f32,
    m2_mul: f32,
    neutral_angle_motor: f32,
    controller_battery: f32,
}

pub struct Flappy {
    state: Vec<f32>,
    input: (f32, f32, f32),
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    pub actuator: PCAActuator,
    manual: bool,
    sensor: Sensors,
    controller_battery: f32,
}

impl Flappy {
    pub fn new() -> Self {
        Flappy {
            state: Vec::new(),
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(90.0, 1),
            manual: true,
            sensor: Sensors::new(1),
            controller_battery: 0.0,
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
        // self.update();

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
}

impl Blimp for Flappy {
    fn update_input(&mut self, input: (f32, f32, f32)) {
        self.input = input;
    }
    //This takes controller input and updates the blimp's state
    fn update(
        &mut self,
        state: Arc<Mutex<BlimpStates>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData {
        let mut rng = rand::thread_rng(); // Use thread_rng() correctly
        //
        //let mut controller_battery = 0.0;
        while let Some(Event { event, id, .. }) = self.gilrs.next_event() {
            match self.gilrs.gamepad(id).power_info() {
                PowerInfo::Discharging(lvl) => self.controller_battery = lvl as f32,
                _ => (),
            };
            match event {
                gilrs::EventType::AxisChanged(axis, pos, _) => match axis {
                    gilrs::Axis::LeftStickY => self.input.0 = pos, // Forward/backward
                    gilrs::Axis::RightStickY => self.input.2 = pos, // Up/down
                    gilrs::Axis::LeftStickX => self.input.1 = pos, // Turning
                    _ => {}
                },
                gilrs::EventType::ButtonPressed(button, code) => match button {
                    gilrs::Button::Start => self.manual = !self.manual,
                    _ => {}
                },
                _ => {}
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
            state: state.lock().unwrap().clone(),
        };
        sensor_data
        //self.sensor.update_reading();
    }
    //Takes in inputs and converts them into actuations
    //Options Teleop to autonomy
    //in my temrinal type "scp targets/Georgie
    fn mix(&mut self) -> Actuations {
        let (x, y, z) = self.input;

        let freq = 0.7;

        let mut s1_ac = 90.0;
        let mut s2_ac = 90.0;
        let mut s3_ac = 90.0;
        let mut s_cg_ac = 95.0;

        if x > 0.1 {
            s1_ac = self.oscillate_wing(-1.0, freq);
            s2_ac = self.oscillate_wing(1.0, freq);
        }

        if y > 0.2 {
            s2_ac = 90.0;
            s1_ac = self.oscillate_wing(-1.0, freq);
            s3_ac = 180.0;
        }
        if y < -0.2 {
            s1_ac = 90.0;
            s2_ac = self.oscillate_wing(1.0, freq);
            s3_ac = 0.0;
        }

        if z > 0.1 {
            s_cg_ac = 180.0
        }
        if z < -0.1 {
            s_cg_ac = 0.0
        }

        Actuations {
            m1: 0.0,
            m2: 0.0,
            m3: 0.0,
            m4: 0.0,
            s1: s1_ac as f32, // Keep neutral if not controlled
            s2: s2_ac as f32,
            s3: s3_ac as f32,
            s4: s_cg_ac as f32,
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

impl SanoBlimp {
    pub fn new(m1_mul: f32, m2_mul: f32, neutral_angle_motor: f32, version: u8) -> Self {
        SanoBlimp {
            state: Vec::new(),
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(neutral_angle_motor, version),
            manual: true,
            sensor: Sensors::new(version),
            score: false,
            score_time: std::time::Instant::now(),
            m1_mul,
            m2_mul,
            neutral_angle_motor,
            controller_battery: 0.0,
        }
    }

    pub fn run(&mut self) {
        // self.update();

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
    fn update(
        &mut self,
        state: Arc<Mutex<BlimpStates>>,
        state_timer: Arc<Mutex<std::time::Instant>>,
        save_image: &mut bool,
    ) -> BlimpSensorData {
        self.sensor.update_altitude();
        self.sensor.update_orientation();

        let mut rng = rand::thread_rng(); // Use thread_rng() correctly
        if self.score {
            self.actuator.actuate(Actuations {
                m1: self.neutral_angle_motor + 7.0,
                m2: self.neutral_angle_motor + 7.0,
                m3: self.neutral_angle_motor + 25.0,
                m4: self.neutral_angle_motor + 25.0,
                s1: NEUTRAL_ANGLE,
                s2: NEUTRAL_ANGLE,
                s3: NEUTRAL_ANGLE,
                s4: NEUTRAL_ANGLE,
            });

            if self.score_time.elapsed() > std::time::Duration::from_secs(3) {
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
                    gilrs::Axis::LeftStickX => self.input.1 = pos, // Turning
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
                        *state.lock().unwrap() = BlimpStates::Ball;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }

                    gilrs::Button::LeftTrigger => {
                        *state.lock().unwrap() = BlimpStates::Goal;
                        *state_timer.lock().unwrap() = std::time::Instant::now();
                    }
                    gilrs::Button::West => {
                        //Capture image frame

                        *save_image = true;
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
            //yaw: self.sensor.imu.euler_angles().unwrap().c,
            yaw: 0.0,
            tracking_error_x: rng.gen_range(-10.0..10.0),
            tracking_error_y: rng.gen_range(-10.0..10.0),
            state: state.lock().unwrap().clone(),
        };

        sensor_data
    }

    fn mix(&mut self) -> Actuations {
        let (x, y, z) = self.input;
        //
        // //println!("{:?}", self.sensor.get_orientation());
        // //
        //
        // //println!("{:?}", self.sensor.imu.gyro_data());
        //
        // let m1_mul = self.m1_mul;
        // let m2_mul = self.m2_mul;
        //
        // let mut m1 = self.neutral_angle_motor - (x * 10.0) * m1_mul; // Map movement to a range (0-180°)
        // let mut m2 = self.neutral_angle_motor - (x * 10.0) * m2_mul;
        //
        // //m1 += self.sensor.imu.gyro_data().unwrap().y;
        // //m2 -= self.sensor.imu.gyro_data().unwrap().y;
        //
        // if z > 0.1 {
        //     m1 += z * 10.0;
        //     m2 += z * 10.0;
        // }
        // if z < -0.1 {
        //     m1 -= z * 10.0;
        //     m2 -= z * 10.0;
        // }
        //
        // let mut s3 = NEUTRAL_ANGLE - (90.0 * z);
        // let mut s4 = NEUTRAL_ANGLE + (90.0 * z);
        //
        // if y < -0.1 {
        //     m1 += y * 10.0;
        //     m2 -= y * 10.0;
        // }
        // if y > 0.1 {
        //     m1 += y * 10.0;
        //     m2 -= y * 10.0;
        // }
        //
        // if z < -0.2 || z > 0.2 {
        //     if y < -0.2 {
        //         s4 = NEUTRAL_ANGLE;
        //     } else if y > 0.2 {
        //         s3 = NEUTRAL_ANGLE;
        //     }
        // }

        let mut m1 = self.neutral_angle_motor + (x * 10.0); // Map movement to a range (0-180°)
        let mut m2 = self.neutral_angle_motor + (x * 10.0);

        let mut s1 = self.neutral_angle_motor - (z * 90.0);
        let mut s2 = self.neutral_angle_motor + (z * 90.0);

        m1 += y * 10.0;
        m2 -= y * 10.0;

        let s3: f32 = 90.0;
        let s4: f32 = 90.0;
        Actuations {
            m1: m1.clamp(
                self.neutral_angle_motor - 20.0,
                self.neutral_angle_motor + 20.0,
            ),
            m2: m2.clamp(
                self.neutral_angle_motor - 20.0,
                self.neutral_angle_motor + 20.0,
            ),
            m3: self.neutral_angle_motor,
            m4: self.neutral_angle_motor,
            s1: s1.clamp(0.0, 180.0), // Keep neutral if not controlled
            s2: s2.clamp(0.0, 180.0),
            s3: s3.clamp(0.0, 180.0),
            s4: s4.clamp(0.0, 180.0),
        }
    }

    fn update_input(&mut self, input: (f32, f32, f32)) {
        self.input = input;
    }
}
