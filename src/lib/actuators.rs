// actuators.rs

use linux_embedded_hal::{Delay, I2cdev}; // Delay might be needed for waits
use pwm_pca9685::{Address, Channel, Pca9685};
use std::thread::sleep;
use std::time::Duration;

// Constants for Actuation
const PWM_FREQUENCY: f32 = 60.0; // Hz for servos
const MIN_PULSE_SERVO: f32 = 500.0; // Minimum pulse width in µs for servos
const MAX_PULSE_SERVO: f32 = 2500.0; // Maximum pulse width in µs for servos
pub const NEUTRAL_ANGLE: f32 = 90.0; // Neutral position for servos (often 90°)

const PWM_FREQUENCY_MOTOR: f32 = 60.0; // Hz for motors (can be different, often higher like 400Hz for ESCs)
const MIN_PULSE_MOTOR: f32 = 600.0; // Minimum pulse width in µs (ESC arming/zero throttle)
const MAX_PULSE_MOTOR: f32 = 2600.0; // Maximum pulse width in µs (Full throttle)
                                     //const MID_PULSE_MOTOR: f32 = 1500.0; // Neutral pulse width (often used for reversible ESCs)
                                     // Neutral angle might differ based on ESC calibration and desired zero point
pub const NEUTRAL_ANGLE_MOTOR: f32 = 83.0; // Neutral position/angle for motors (adjust based on testing)


// Represents the desired state of all motors and servos
#[derive(Debug, Default, Clone, Copy)] // Added Clone, Copy for convenience
pub struct Actuations {
    pub m1: f32, // Motor speeds/angles (0-180°)
    pub m2: f32,
    pub m3: f32,
    pub m4: f32,
    pub s1: f32, // Servo positions/angles (0-180°)
    pub s2: f32,
    pub s3: f32,
    pub s4: f32,
}

pub struct PCAActuator {
    pwm: Pca9685<I2cdev>,
    // Optional: Add Delay if needed for timing within this struct
    // delay: Delay,
}

impl PCAActuator {
    pub fn new() -> Self {
        let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device for PCA9685");
        let address = Address::default(); // Default PCA9685 address (0x40)
                                          // let address = Address::from(0x55); // Use if address is different

        let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");

        // Calculate the prescale value for the desired PWM frequency
        // Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1
        // PCA9685 internal oscillator is typically 25MHz (25,000,000 Hz)
        let osc_clock = 25_000_000.0;
        let prescale_val = (osc_clock / (4096.0 * PWM_FREQUENCY_MOTOR)).round() - 1.0;
        // Use same frequency for motors and servos for now, adjust if needed
        // let prescale_val_servo = (osc_clock / (4096.0 * PWM_FREQUENCY)).round() - 1.0;

        pwm.set_prescale(prescale_val as u8) // Cast prescale to u8
            .expect("Failed to set PCA9685 prescale");

        pwm.enable().expect("Failed to enable PCA9685");

        // Initialize all channels to a safe state (e.g., neutral) before ESC init
        let mut actuator = PCAActuator {
            pwm, /*, delay: Delay{} */
        };
        // Set servos to neutral initially
        actuator.set_servo_angle(Channel::C4, NEUTRAL_ANGLE);
        actuator.set_servo_angle(Channel::C5, NEUTRAL_ANGLE);
        actuator.set_servo_angle(Channel::C6, NEUTRAL_ANGLE);
        actuator.set_servo_angle(Channel::C7, NEUTRAL_ANGLE);
        // Motors will be set during init_escs

        actuator.init_escs(); // Initialize ESCs after setting up PCA
        actuator
    }

    /// Convert angle (0-180°) to PWM pulse width (on/off counts) for motors
    fn set_motor_angle(&mut self, channel: Channel, angle: f32) {
        // Clamp angle to valid range
        let angle_clamped = angle.clamp(0.0, 180.0);
        // Map angle (0-180) to pulse width (MIN_PULSE_MOTOR - MAX_PULSE_MOTOR)
        let pulse_width_us =
            MIN_PULSE_MOTOR + (angle_clamped / 180.0) * (MAX_PULSE_MOTOR - MIN_PULSE_MOTOR);

        // Convert pulse width in microseconds to PCA9685 'off' count
        // Formula: off_count = round((pulse_width_us / 1_000_000) * pwm_frequency * 4096)
        let off_count =
            ((pulse_width_us * 1e-6 * PWM_FREQUENCY_MOTOR * 4096.0).round() as u16).clamp(0, 4095);

        // Standard PWM: starts at count 0, ends at off_count
        let on_count = 0;

        if let Err(e) = self.pwm.set_channel_on_off(channel, on_count, off_count) {
            eprintln!("Failed to set motor channel {:?}: {:?}", channel, e);
        }
    }

    /// Convert angle (0-180°) to PWM pulse width (on/off counts) for servos
    fn set_servo_angle(&mut self, channel: Channel, angle: f32) {
        // Clamp angle to valid range
        let angle_clamped = angle.clamp(0.0, 180.0);
        // Map angle (0-180) to pulse width (MIN_PULSE_SERVO - MAX_PULSE_SERVO)
        let pulse_width_us =
            MIN_PULSE_SERVO + (angle_clamped / 180.0) * (MAX_PULSE_SERVO - MIN_PULSE_SERVO);

        // Convert pulse width in microseconds to PCA9685 'off' count
        let off_count =
            ((pulse_width_us * 1e-6 * PWM_FREQUENCY * 4096.0).round() as u16).clamp(0, 4095);

        // Standard PWM: starts at count 0, ends at off_count
        let on_count = 0;

        if let Err(e) = self.pwm.set_channel_on_off(channel, on_count, off_count) {
            eprintln!("Failed to set servo channel {:?}: {:?}", channel, e);
        }
    }

    /// ESC initialization sequence (adjust timings if needed)
    pub fn init_escs(&mut self) {
        println!("Initializing ESCs...");
        // Ensure ESCs receive min throttle first before arming sequence for safety
        println!("Sending min throttle (safety step)");
        self.set_motor_angle(Channel::C0, 0.0);
        self.set_motor_angle(Channel::C1, 0.0);
        self.set_motor_angle(Channel::C2, 0.0);
        self.set_motor_angle(Channel::C3, 0.0);
        sleep(Duration::from_secs(1)); // Wait for ESCs to recognize low signal

        // Typical arming sequence: Send max throttle, then min throttle
        // Some ESCs might only need min throttle after power-up
        // Consult your ESC documentation

        // // Optional Step 1: Send max throttle (if required by ESC for calibration/arming)
        // println!("Sending max throttle (arming sequence step 1 - if needed)");
        // self.set_motor_angle(Channel::C0, 180.0);
        // self.set_motor_angle(Channel::C1, 180.0);
        // self.set_motor_angle(Channel::C2, 180.0);
        // self.set_motor_angle(Channel::C3, 180.0);
        // sleep(Duration::from_secs(2)); // Wait for ESCs to register max throttle

        // Step 2: Send min throttle to arm the ESCs
        println!("Sending min throttle (arming)");
        self.set_motor_angle(Channel::C0, 0.0);
        self.set_motor_angle(Channel::C1, 0.0);
        self.set_motor_angle(Channel::C2, 0.0);
        self.set_motor_angle(Channel::C3, 0.0);
        sleep(Duration::from_secs(2)); // Wait for ESCs to arm (listen for beeps)

        // Step 3: Set to neutral throttle (ready state)
        println!(
            "Setting ESCs to neutral ({:.1} degrees)",
            NEUTRAL_ANGLE_MOTOR
        );
        self.set_motor_angle(Channel::C0, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_angle(Channel::C1, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_angle(Channel::C2, NEUTRAL_ANGLE_MOTOR);
        self.set_motor_angle(Channel::C3, NEUTRAL_ANGLE_MOTOR);
        sleep(Duration::from_millis(500)); // Short delay after setting neutral

        println!("ESCs should be initialized and armed!");
    }

    /// Apply the actuation commands to the motors and servos
    pub fn actuate(&mut self, act: Actuations) {
        self.set_motor_angle(Channel::C0, act.m1);
        self.set_motor_angle(Channel::C1, act.m2);
        self.set_motor_angle(Channel::C2, act.m3);
        self.set_motor_angle(Channel::C3, act.m4);

        self.set_servo_angle(Channel::C4, act.s1);
        self.set_servo_angle(Channel::C5, act.s2);
        self.set_servo_angle(Channel::C6, act.s3);
        self.set_servo_angle(Channel::C7, act.s4);
    }

    // Optional: Add a method to stop all actuators safely
    pub fn stop_all(&mut self) {
        println!("Stopping all actuators...");
        let neutral_actuations = Actuations {
            m1: NEUTRAL_ANGLE_MOTOR,
            m2: NEUTRAL_ANGLE_MOTOR,
            m3: NEUTRAL_ANGLE_MOTOR,
            m4: NEUTRAL_ANGLE_MOTOR,
            s1: NEUTRAL_ANGLE,
            s2: NEUTRAL_ANGLE,
            s3: NEUTRAL_ANGLE,
            s4: NEUTRAL_ANGLE,
        };
        self.actuate(neutral_actuations);
        // Optionally disable PWM output entirely after setting neutral
        // self.pwm.disable().expect("Failed to disable PCA9685");
    }
}

// Implement Drop to ensure actuators are stopped safely when PCAActuator goes out of scope
impl Drop for PCAActuator {
    fn drop(&mut self) {
        self.stop_all();
        // Attempt to disable the PCA9685 chip on drop
        if let Err(e) = self.pwm.disable() {
            eprintln!("Failed to disable PCA9685 on drop: {:?}", e);
        } else {
            println!("PCA9685 disabled.");
        }
    }
}
