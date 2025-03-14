use gilrs::{Button, Event, GamepadId, Gilrs};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};
use std::thread::sleep;
use std::time::Duration;
use tokio::spawn;

use super::object_detection::Detection;

const PWM_FREQUENCY: f32 = 60.0; // Hz for motors and servos
const MIN_PULSE_SERVO: f32 = 500.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE_SERVO: f32 = 2500.0; // Maximum pulse width in µs (Full throttle)
                                     //
const MIN_PULSE: f32 = 1000.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE: f32 = 2000.0; // Maximum pulse width in µs (Full throttle)
const MID_PULSE: f32 = 1500.0; // Neutral (90° equivalent)
const NEUTRAL_ANGLE: f32 = 90.0; // Neutral position for motors and servos

/// Every blimp needs the following trait
pub trait Blimp {
    fn update(&mut self);
    fn teleop(&mut self) -> Actuations;
    fn autonomous(&mut self);
    fn search(&mut self);
}

pub struct PCAActuator {
    pwm: Pca9685<I2cdev>,
}

impl PCAActuator {
    pub fn new() -> Self {
        let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device");
        let address = Address::default();
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
        let off = ((pulse_width * PWM_FREQUENCY * 4096.0 * 1e-6) as u16).clamp(0, 4095);

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
        sleep(Duration::from_secs(2));

        // Step 2: Send min throttle to arm the ESCs
        println!("Sending min throttle (arming)");
        self.set_motor_speed(Channel::C0, 0.0);
        self.set_motor_speed(Channel::C1, 0.0);
        self.set_motor_speed(Channel::C2, 0.0);
        self.set_motor_speed(Channel::C3, 0.0);
        sleep(Duration::from_secs(2));

        // Step 3: Move to neutral throttle (ready to receive commands)
        println!("Setting ESCs to neutral");
        self.set_motor_speed(Channel::C0, NEUTRAL_ANGLE);
        self.set_motor_speed(Channel::C1, NEUTRAL_ANGLE);
        self.set_motor_speed(Channel::C2, NEUTRAL_ANGLE);
        self.set_motor_speed(Channel::C3, NEUTRAL_ANGLE);
        sleep(Duration::from_secs(1));

        println!("ESCs initialized!");
    }

    fn actuate(&mut self, act: Actuations) {
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
    actuator: PCAActuator,
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
    pub fn new() -> Self {
        SanoBlimp {
            state: Vec::new(),
            input: (0.0_f32, 0.0_f32, 0.0_f32),
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            actuator: PCAActuator::new(),
        }
    }

    pub fn run(&mut self) {
        self.update();
        //self.autonomous();
        let act = self.teleop();
        self.actuator.actuate(act);

        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}

impl Blimp for SanoBlimp {
    fn update(&mut self) {
        while let Some(Event { event, .. }) = self.gilrs.next_event() {
            match event {
                gilrs::EventType::AxisChanged(axis, pos, _) => match axis {
                    gilrs::Axis::LeftStickY => self.input.0 = pos, // Forward/backward
                    gilrs::Axis::RightStickY => self.input.2 = pos, // Up/down
                    gilrs::Axis::RightStickX => self.input.1 = pos, // Turning
                    _ => {}
                },
                _ => {}
            }
        }
    }

    fn teleop(&mut self) -> Actuations {
        let (x, y, z) = self.input;

        println!("{:?}", self.input);

        let mut m1 = NEUTRAL_ANGLE - (x * 9.0); // Map movement to a range (0-180°)
        let mut m2 = NEUTRAL_ANGLE - (x * 9.0);

        if z > 0.1 {
            m1 += z * 9.0;
            m2 += z * 9.0;
        } else if z < -0.1 {
            m1 -= z * 9.0;
            m2 -= z * 9.0;
        }

        let mut s3 = NEUTRAL_ANGLE - (90.0 * z);
        let mut s4 = NEUTRAL_ANGLE + (90.0 * z);

        if y < -0.1 {
            m1 -= y * 9.0;
            m2 += y * 9.0;
        } else if y > 0.1 {
            m1 += y * 9.0;
            m2 -= y * 9.0;
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
            m3: m1.clamp(0.0, 180.0),
            m4: m2.clamp(0.0, 180.0),
            s1: NEUTRAL_ANGLE, // Keep neutral if not controlled
            s2: NEUTRAL_ANGLE,
            s3: s3.clamp(0.0, 180.0),
            s4: s4.clamp(0.0, 180.0),
        }
    }

    fn autonomous(&mut self) {}
    fn search(&mut self) {}
}
