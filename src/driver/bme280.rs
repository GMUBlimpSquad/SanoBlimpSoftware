use std::error::Error;
use std::io;
use std::io::Write;
use std::thread;
use std::time::Duration;

// Import necessary SPI components
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

// Import necessary I2C components
use i2cdev::core::I2CDevice; // Core trait
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

// --- Constants --- (Unchanged from original)
const BME280_REG_CHIPID: u8 = 0xD0;
const BME280_CHIP_ID: u8 = 0x60; // Expected chip ID for BME280
const BME280_I2C_ADDR_PRIMARY: u16 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u16 = 0x77;

const REG_DIG_T1: u8 = 0x88;
const REG_DIG_H1: u8 = 0xA1;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5; // Note: Config register might need adjustment depending on SPI/I2C mode, but often defaults are fine.
const REG_PRESS_MSB: u8 = 0xF7;
const REG_RESET: u8 = 0xE0; // Reset register
const RESET_VALUE: u8 = 0xB6; // Value to write to trigger reset

pub const DEFAULT_SEA_LEVEL_HPA: f32 = 1013.25;

// --- Structs --- (Unchanged from original)
#[derive(Debug, Clone)]
struct CalibData {
    // Temperature
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    // Pressure
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    // Humidity
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

#[derive(Debug, Clone)]
pub struct Bme280Measurements {
    pub temperature_c: f32,
    pub pressure_hpa: f32,
    pub humidity_pct: f32,
    pub altitude_m: f32,
}

// --- Enum to represent the bus type ---
enum Bus {
    Spi(Spidev),
    I2c(LinuxI2CDevice),
}

/// Main BME280 struct supporting SPI and I2C
pub struct Bme280 {
    bus: Bus,
    calib: CalibData,
    sea_level_hpa: f32,
    t_fine: i32, // Store t_fine for compensation calculations
}

impl Bme280 {
    /// Create a new BME280 object using SPI.
    /// `spi_path`: Path to the SPI device (e.g., "/dev/spidev0.0").
    /// `sea_level_hpa`: Atmospheric pressure at sea level in hPa (used for altitude calculation).
    pub fn new_spi(spi_path: &str, sea_level_hpa: f32) -> Result<Self, Box<dyn Error>> {
        // 1) Open SPI
        let mut spi = Spidev::open(spi_path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(1_000_000) // 1 MHz, adjust if needed
            .mode(SpiModeFlags::SPI_MODE_0) // BME280 supports Mode 0 (CPOL=0, CPHA=0) and Mode 3 (CPOL=1, CPHA=1)
            .lsb_first(false)
            .build();
        spi.configure(&options)?;

        let mut bme = Bme280 {
            bus: Bus::Spi(spi),
            calib: CalibData::zero(), // Temporary zeroed calibration
            sea_level_hpa,
            t_fine: 0,
        };

        bme.common_init()?;
        Ok(bme)
    }

    /// Create a new BME280 object using I2C.
    /// `i2c_path`: Path to the I2C device (e.g., "/dev/i2c-1").
    /// `address`: The I2C address of the BME280 (usually 0x76 or 0x77).
    /// `sea_level_hpa`: Atmospheric pressure at sea level in hPa (used for altitude calculation).
    pub fn new_i2c(
        i2c_path: &str,
        address: u16,
        sea_level_hpa: f32,
    ) -> Result<Self, Box<dyn Error>> {
        // 1) Open I2C
        let mut i2c = LinuxI2CDevice::new(i2c_path, address)?;

        let mut bme = Bme280 {
            bus: Bus::I2c(i2c),
            calib: CalibData::zero(), // Temporary zeroed calibration
            sea_level_hpa,
            t_fine: 0,
        };

        bme.common_init()?;
        Ok(bme)
    }

    /// Common initialization steps for both SPI and I2C.
    fn common_init(&mut self) -> Result<(), Box<dyn Error>> {
        // Optional: Reset the device
        // self.reset()?;
        // thread::sleep(Duration::from_millis(10)); // Wait after reset

        // 2) Verify chip ID
        let chip_id = self._read_register_n(BME280_REG_CHIPID, 1)?[0];
        if chip_id != BME280_CHIP_ID {
            // Return an error instead of just printing a warning
            return Err(Box::new(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "Unexpected BME280 chip ID 0x{:02X} (expected 0x{:02X})",
                    chip_id, BME280_CHIP_ID
                ),
            )));
        }

        // 3) Read calibration data
        self.calib = self._read_calibration_data()?;

        // 4) Configure the sensor
        // Set humidity oversampling (required *before* control_meas)
        // osrs_h x1: 0b001
        self._write_register(REG_CTRL_HUM, 0x01)?;

        // Set temp/pressure oversampling and mode
        // osrs_t x1: 0b001, osrs_p x1: 0b001, mode normal: 0b11
        // Result: 0b001_001_11 = 0x27
        // Using forced mode (0b01 or 0b10) is better for single reads
        // Result: 0b001_001_01 = 0x25 (forced mode)
        self._write_register(REG_CTRL_MEAS, 0x25)?;

        // Optionally set config register (standby time, filter)
        // t_sb 1000ms: 0b101, filter off: 0b000, spi3w off: 0b0
        // Result: 0b101_000_0_0 = 0xA0
        // self._write_register(REG_CONFIG, 0xA0)?; // Example for normal mode

        // Wait for measurement to be ready if using forced mode,
        // or just a small delay for settings to apply.
        // Max measurement time (1x T, 1x P, 1x H, filter off) ~9.3ms
        // Let's wait a bit longer to be safe.
        thread::sleep(Duration::from_millis(20));

        Ok(())
    }

    /// Perform a soft reset of the sensor.
    pub fn reset(&mut self) -> Result<(), Box<dyn Error>> {
        self._write_register(REG_RESET, RESET_VALUE)?;
        thread::sleep(Duration::from_millis(10)); // Wait for reset to complete
        Ok(())
    }

    /// Read the current measurements (temperature [°C], pressure [hPa], humidity [%], altitude [m])
    pub fn read(&mut self) -> Result<Bme280Measurements, Box<dyn Error>> {
        self.calib = self._read_calibration_data()?;

        self._write_register(REG_CTRL_HUM, 0x01)?;

        self._write_register(REG_CTRL_MEAS, 0x25)?;

        // In forced mode, we need to trigger a measurement each time
        // Check if the current mode is forced mode (ends in 01 or 10)
        let ctrl_meas = self._read_register_n(REG_CTRL_MEAS, 1)?[0];
        if (ctrl_meas & 0x03) == 0x01 || (ctrl_meas & 0x03) == 0x02 {
            // Trigger a new measurement (re-write the control register)
            self._write_register(REG_CTRL_MEAS, ctrl_meas)?;
            // Wait for measurement. Calculation based on datasheet formulas or max time.
            // For 1x oversampling: T=2.3ms, P=2.3ms, H=2.3ms. Total ~7ms.
            // Add some buffer.
            thread::sleep(Duration::from_millis(15));
        }
        // In normal mode, reading automatically gets the latest measurement.

        // Read T/P/H data registers (8 bytes starting from PRESS_MSB)
        let data = self._read_register_n(REG_PRESS_MSB, 8)?;
        // data layout: [P_msb, P_lsb, P_xlsb, T_msb, T_lsb, T_xlsb, H_msb, H_lsb]

        // Unpack raw ADC values
        let adc_p = ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
        let adc_t = ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);
        let adc_h = ((data[6] as i32) << 8) | (data[7] as i32);

        // Check for invalid readings (e.g., 0x80000 for T/P, 0x8000 for H)
        if adc_t == 0x80000 {
            return Err(Box::new(io::Error::new(
                io::ErrorKind::Other,
                "Invalid temperature reading (0x80000)",
            )));
        }
        // Note: Pressure adc_p = 0x80000 might be valid if pressure is very low.
        // Note: Humidity adc_h = 0x8000 might be valid if humidity is very low.

        // 1) Temperature compensation -> t_fine (required for P and H compensation)
        self.t_fine = self.compensate_temp_fine(adc_t);
        let temperature_c = compensate_temperature(self.t_fine);

        // 2) Pressure compensation
        let pressure_pa = compensate_pressure(adc_p, &self.calib, self.t_fine);
        let pressure_hpa = pressure_pa / 100.0;

        // 3) Humidity compensation
        let humidity_pct = compensate_humidity(adc_h, &self.calib, self.t_fine);

        // 4) Altitude calculation
        let altitude_m = altitude_from_pressure(pressure_hpa, self.sea_level_hpa);

        Ok(Bme280Measurements {
            temperature_c,
            pressure_hpa,
            humidity_pct,
            altitude_m,
        })
    }

    /// If you want to adjust the sea-level pressure at runtime for altitude calculation
    pub fn set_sea_level_hpa(&mut self, hpa: f32) {
        self.sea_level_hpa = hpa;
    }

    // --- Internal Helper Methods ---

    /// Helper: return t_fine for temperature compensation
    fn compensate_temp_fine(&self, adc_t: i32) -> i32 {
        let c = &self.calib;
        // Formula from BME280 datasheet section 4.2.3
        let var1 = (((adc_t >> 3) - ((c.dig_t1 as i32) << 1)) * (c.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (c.dig_t1 as i32)) * ((adc_t >> 4) - (c.dig_t1 as i32)))
            >> 12)
            * (c.dig_t3 as i32))
            >> 14;
        var1 + var2 // This sum is t_fine
    }

    /// Internal: Read `length` bytes starting from `reg`.
    /// Handles dispatch to SPI or I2C and register address formatting.
    fn _read_register_n(&mut self, reg: u8, length: usize) -> Result<Vec<u8>, Box<dyn Error>> {
        match &mut self.bus {
            Bus::Spi(spi) => {
                // For SPI read, set MSB of register address to 1
                let read_reg = reg | 0x80;
                let mut tx_buf = vec![0; length + 1];
                tx_buf[0] = read_reg; // First byte is register address with read bit

                let mut rx_buf = vec![0; length + 1];

                let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
                spi.transfer(&mut transfer)?;

                // Data received starts from the second byte in rx_buf
                Ok(rx_buf[1..].to_vec())
            }
            Bus::I2c(i2c) => {
                // For I2C, write the register address first, then read the data.
                // Some I2C devices/controllers support combined write/read (smbus_read_i2c_block_data)
                // but the basic write-then-read approach is more universal.
                i2c.write(&[reg])?; // Write the register address we want to read from
                let mut buf = vec![0; length];
                i2c.read(&mut buf)?; // Read the data
                Ok(buf)
                // Alternative using smbus function (if available and preferred):
                // let mut buf = vec![0; length];
                // i2c.smbus_read_i2c_block_data(reg, length as u8, &mut buf)?;
                // Ok(buf)
            }
        }
    }

    /// Internal: Write a single byte `val` to register `reg`.
    /// Handles dispatch to SPI or I2C and register address formatting.
    fn _write_register(&mut self, reg: u8, val: u8) -> Result<(), Box<dyn Error>> {
        match &mut self.bus {
            Bus::Spi(spi) => {
                // For SPI write, ensure MSB of register address is 0
                let write_reg = reg & 0x7F;
                let tx_buf = [write_reg, val]; // Address byte, Data byte
                let mut rx_buf = vec![0; 2];
                spi.write(&tx_buf)?;
                Ok(())
            }
            Bus::I2c(i2c) => {
                // For I2C, send register address then data byte
                i2c.write(&[reg, val])?;
                // Alternative using smbus (more common for single byte writes):
                // i2c.smbus_write_byte_data(reg, val)?;
                Ok(())
            }
        }
    }

    /// Read all calibration data from the BME280
    fn _read_calibration_data(&mut self) -> Result<CalibData, Box<dyn Error>> {
        // Read temperature and pressure calibration (0x88..0xA1 => 26 bytes, but read in chunks usually)
        // Reading 24 bytes from 0x88..0x9F is common practice first
        let calib_tp = self._read_register_n(REG_DIG_T1, 24)?; // 0x88..0x9F

        // Read H1 (0xA1)
        let dig_h1_buf = self._read_register_n(REG_DIG_H1, 1)?; // 0xA1
        let dig_h1 = dig_h1_buf[0];

        // Read H2..H6 (0xE1..0xE7 => 7 bytes)
        let calib_h = self._read_register_n(0xE1, 7)?; // 0xE1..0xE7

        // Parse calibration data (Little Endian format)
        let dig_t1 = le_u16(&calib_tp[0..2]); // 0x88/0x89
        let dig_t2 = le_i16(&calib_tp[2..4]); // 0x8A/0x8B
        let dig_t3 = le_i16(&calib_tp[4..6]); // 0x8C/0x8D

        let dig_p1 = le_u16(&calib_tp[6..8]); // 0x8E/0x8F
        let dig_p2 = le_i16(&calib_tp[8..10]); // 0x90/0x91
        let dig_p3 = le_i16(&calib_tp[10..12]); // 0x92/0x93
        let dig_p4 = le_i16(&calib_tp[12..14]); // 0x94/0x95
        let dig_p5 = le_i16(&calib_tp[14..16]); // 0x96/0x97
        let dig_p6 = le_i16(&calib_tp[16..18]); // 0x98/0x99
        let dig_p7 = le_i16(&calib_tp[18..20]); // 0x9A/0x9B
        let dig_p8 = le_i16(&calib_tp[20..22]); // 0x9C/0x9D
        let dig_p9 = le_i16(&calib_tp[22..24]); // 0x9E/0x9F

        // Humidity parsing (more complex, based on datasheet)
        let dig_h2 = le_i16(&calib_h[0..2]); // 0xE1/0xE2
        let dig_h3 = calib_h[2]; // 0xE3

        let e4 = calib_h[3]; // 0xE4
        let e5 = calib_h[4]; // 0xE5
        let e6 = calib_h[5]; // 0xE6
        // dig_H4 = H4_msb<7:0> H4_lsb<3:0> = E4<7:0> E5<3:0>
        let dig_h4 = ((e4 as i16) << 4) | ((e5 as i16) & 0x0F);
        // dig_H5 = H5_msb<7:0> H5_lsb<3:0> = E6<7:0> E5<7:4>
        let dig_h5 = ((e6 as i16) << 4) | (((e5 as i16) >> 4) & 0x0F);

        let dig_h6 = calib_h[6] as i8; // 0xE7

        Ok(CalibData {
            dig_t1,
            dig_t2,
            dig_t3,
            dig_p1,
            dig_p2,
            dig_p3,
            dig_p4,
            dig_p5,
            dig_p6,
            dig_p7,
            dig_p8,
            dig_p9,
            dig_h1,
            dig_h2,
            dig_h3,
            dig_h4,
            dig_h5,
            dig_h6,
        })
    }
}

// --- Compensation Algorithms (Unchanged from original, but kept private) ---
// Formulas adapted from BME280 Datasheet Section 4.2.3

/// Convert t_fine to an actual temperature in °C
fn compensate_temperature(t_fine: i32) -> f32 {
    let t = (t_fine * 5 + 128) >> 8;
    (t as f32) / 100.0
}

/// Convert raw pressure to Pa, given t_fine
fn compensate_pressure(adc_p: i32, c: &CalibData, t_fine: i32) -> f32 {
    // Use i64 for intermediate calculations to avoid overflow
    let mut var1: i64;
    let mut var2: i64;
    let mut p: i64;

    var1 = (t_fine as i64) - 128000;
    var2 = var1 * var1 * (c.dig_p6 as i64);
    var2 = var2 + ((var1 * (c.dig_p5 as i64)) << 17);
    var2 = var2 + ((c.dig_p4 as i64) << 35);
    var1 = ((var1 * var1 * (c.dig_p3 as i64)) >> 8) + ((var1 * (c.dig_p2 as i64)) << 12);
    // Avoid division by zero
    if var1 == 0 {
        return 0.0; // Or handle error appropriately
    }
    // Original formula uses 1 << 47, but that requires 128-bit integers or careful handling.
    // The Datasheet notes a possible simplification/approximation if needed. Let's stick to i64 as much as possible.
    // Check datasheet variant, using the 64-bit integer version seems standard now.
    var1 = (((1_i64 << 47) + var1) * (c.dig_p1 as i64)) >> 33;

    // Avoid division by zero
    if var1 == 0 {
        return 0.0;
    }

    p = 1048576 - (adc_p as i64);
    p = (((p << 31) - var2) * 3125) / var1; // Check potential overflow here if pressure is extreme
    var1 = ((c.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((c.dig_p8 as i64) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((c.dig_p7 as i64) << 4);
    (p as f32) / 256.0 // Final pressure in Pa
}

/// Convert raw humidity to % relative humidity, given t_fine
fn compensate_humidity(adc_h: i32, c: &CalibData, t_fine: i32) -> f32 {
    // Formula from BME280 datasheet section 4.2.3
    let mut v_x1_u32r: i32;

    v_x1_u32r = t_fine - 76800;
    // Check for potential overflows with i32, consider i64 if necessary
    v_x1_u32r = (((((adc_h << 14) - ((c.dig_h4 as i32) << 20) - ((c.dig_h5 as i32) * v_x1_u32r))
        + 16384)
        >> 15)
        * (((((((v_x1_u32r * (c.dig_h6 as i32)) >> 10)
            * (((v_x1_u32r * (c.dig_h3 as i32)) >> 11) + 32768))
            >> 10)
            + 2097152)
            * (c.dig_h2 as i32)
            + 8192)
            >> 14));

    v_x1_u32r =
        v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (c.dig_h1 as i32)) >> 4);

    // Clamp results
    v_x1_u32r = v_x1_u32r.max(0); // humidity can't be < 0
    v_x1_u32r = v_x1_u32r.min(419430400); // Max value corresponds to 100% RH (419430400 = 102400 * 4096)

    (v_x1_u32r >> 12) as f32 / 1024.0 // Final humidity in %RH
}

/// Compute altitude from pressure in hPa vs. sea-level reference
/// (Standard barometric formula)
fn altitude_from_pressure(pressure_hpa: f32, sea_level_hpa: f32) -> f32 {
    // Formula: h = 44330 * [1 - (P / P0)^(1 / 5.255)]
    // 1 / 5.255 ≈ 0.1903
    44330.0 * (1.0 - (pressure_hpa / sea_level_hpa).powf(0.190294957)) // Using more precision
}

// --- Little Endian Helpers --- (Unchanged from original)
fn le_u16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}
fn le_i16(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
}

// --- Default for CalibData ---
impl CalibData {
    fn zero() -> Self {
        CalibData {
            dig_t1: 0,
            dig_t2: 0,
            dig_t3: 0,
            dig_p1: 0,
            dig_p2: 0,
            dig_p3: 0,
            dig_p4: 0,
            dig_p5: 0,
            dig_p6: 0,
            dig_p7: 0,
            dig_p8: 0,
            dig_p9: 0,
            dig_h1: 0,
            dig_h2: 0,
            dig_h3: 0,
            dig_h4: 0,
            dig_h5: 0,
            dig_h6: 0,
        }
    }
}
