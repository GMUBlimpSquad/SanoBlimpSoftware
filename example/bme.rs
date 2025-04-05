use bme280::i2c::BME280;
use linux_embedded_hal::{Delay, I2cdev};

fn main() {
    // using Linux I2C Bus #1 in this example
    let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
    let mut delay = Delay; /* ..delay provider */
    // initialize the BME280 using the primary I2C address 0x76
    let mut bme280 = BME280::new_primary(i2c_bus);

    // or, initialize the BME280 using the secondary I2C address 0x77
    // let mut bme280 = BME280::new_secondary(i2c_bus);

    // or, initialize the BME280 using a custom I2C address
    // let bme280_i2c_addr = 0x88;
    // let mut bme280 = BME280::new(i2c_bus, bme280_i2c_addr);

    // initialize the sensor
    bme280.init(&mut delay).unwrap();

    // measure temperature, pressure, and humidity

    // println!("Relative Humidity = {}%", measurements.humidity);
    // println!("Temperature = {} deg C", measurements.temperature);
    loop {
        let measurements = bme280.measure(&mut delay).unwrap();
        println!("Pressure = {} pascals", measurements.pressure);
    }
}
