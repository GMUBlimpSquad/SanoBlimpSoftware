use linux_embedded_hal::I2cdev;

use std::thread::sleep;
use std::time::{self, Duration};

use pwm_pca9685::{Address, Channel, Pca9685};

const PWM_FREQUENCY: f32 = 60.0; // Hz for motors and servos
const MIN_PULSE_SERVO: f32 = 500.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE_SERVO: f32 = 2500.0; // Maximum pulse width in µs (Full throttle)
const NEUTRAL_ANGLE: f32 = 90.0; // Neutral position for motors and servos
//
const PWM_FREQUENCY_MOTOR: f32 = 50.0; // Hz for motors and servos
const MIN_PULSE: f32 = 600.0; // Minimum pulse width in µs (ESC arming)
const MAX_PULSE: f32 = 2600.0; // Maximum pulse width in µs (Full throttle)
const MID_PULSE: f32 = 1500.0; // Neutral (90° equivalent)
// const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos
//const self.neutral_angle_motor: f32 = 83.0; // Neutral position for motors and servos

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
            Address::from(0x55)
        };
        let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");
        pwm.set_prescale(100).expect("Failed to set prescale");
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
        self.set_motor_speed(Channel::C0, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C1, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C2, self.neutral_angle_motor);
        self.set_motor_speed(Channel::C3, self.neutral_angle_motor);

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

fn angle_to_duty(angle: u16) -> u16 {
    (angle / 180) * 4095
}

fn main() {
    let mut actuator = PCAActuator::new(90.0, 1);

    actuator.actuate(Actuations {
        m1: 90.0,
        m2: 85.0,
        m3: 85.0,
        m4: 85.0,
        s1: 85.0,
        s2: 85.0,
        s3: 85.0,
        s4: 85.0,
    });

    loop {}
}
