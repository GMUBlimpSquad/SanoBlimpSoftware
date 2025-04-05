use linux_embedded_hal::I2cdev;

use pwm_pca9685::{Address, Channel, Pca9685};

fn angle_to_duty(angle: u16) -> u16 {
    (angle / 180) * 4095
}

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").expect("Failed to initialize I2C device");
    let address = Address::from(0x55);
    let mut pwm = Pca9685::new(dev, address).expect("Failed to create PCA9685 instance");
    pwm.set_prescale(100).expect("Failed to set prescale");
    pwm.enable().expect("Failed to enable PCA9685");

    loop {
        // Turn on channel 0 at 0.
        pwm.set_channel_on(Channel::C0, 0).unwrap();
        pwm.set_channel_on(Channel::C1, 0).unwrap();
        pwm.set_channel_on(Channel::C2, 0).unwrap();
        pwm.set_channel_on(Channel::C3, 0).unwrap();
        //
        // Turn off channel 0 at 2047, which is 50% in
        // the range `[0..4095]`.
        for i in 0..180 {
            let duty = angle_to_duty(i);
            pwm.set_channel_off(Channel::C0, duty).unwrap();
            pwm.set_channel_off(Channel::C1, duty).unwrap();
            pwm.set_channel_off(Channel::C2, duty).unwrap();
            pwm.set_channel_off(Channel::C3, duty).unwrap();
        }
    }

    let _dev = pwm.destroy(); // Get the I2C device back
}
