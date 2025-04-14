use std::error::Error;
use std::thread;
use std::time::Duration;

use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io;

/// Key registers
const BME280_REG_CHIPID: u8 = 0xD0;
const BME280_CHIP_ID: u8 = 0x60; // Expected chip ID for BME280

// Calibration data registers
const REG_DIG_T1: u8 = 0x88; // ...through 0xA1 for temperature/pressure
const REG_DIG_H1: u8 = 0xA1; // humidity calibration part 1
                             // ...and 0xE1..0xE7 for rest of humidity calibration

// Control & data registers
const REG_CTRL_HUM: u8 = 0xF2;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_PRESS_MSB: u8 = 0xF7;

// Default sea-level pressure in hPa (used for altitude calc)
pub const DEFAULT_SEA_LEVEL_HPA: f32 = 1013.25;

/// Raw calibration values as read from the BME280
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

/// Final compensated measurements from the BME280
#[derive(Debug, Clone)]
pub struct Bme280Measurements {
    pub temperature_c: f32,
    pub pressure_hpa: f32,
    pub humidity_pct: f32,
    pub altitude_m: f32,
}

/// Main BME280 struct
pub struct Bme280 {
    spi: Spidev,
    calib: CalibData,
    sea_level_hpa: f32,
}

impl Bme280 {
    /// Create a new BME280 object, opening the given SPI device path
    /// (e.g. "/dev/spidev0.0").  
    /// `sea_level_hpa` is used for altitude calculation (default ~1013.25).
    pub fn new(spi_path: &str, sea_level_hpa: f32) -> Result<Self, Box<dyn Error>> {
        // 1) Open SPI
        let mut spi = Spidev::open(spi_path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(1_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .lsb_first(false)
            .build();
        spi.configure(&options)?;

        // 2) Verify chip ID
        let chip_id = read_register_n(&mut spi, BME280_REG_CHIPID, 1)?[0];
        if chip_id != BME280_CHIP_ID {
            eprintln!(
                "Warning: Unexpected chip ID 0x{:X} (expected 0x60)",
                chip_id
            );
        }

        // 3) Read calibration data
        let calib = read_calibration_data(&mut spi)?;

        // 4) Configure the sensor for a default oversampling & normal mode
        //    Example: humidity oversampling=1, pressure=1, temperature=1
        //    normal mode => 0xF2=0x01, 0xF4=0x27 or 0x25, etc.  Adjust as needed.
        write_register(&mut spi, REG_CTRL_HUM, 0x01)?; // osrs_h=1
        write_register(&mut spi, REG_CTRL_MEAS, 0x25)?; // osrs_p=1, osrs_t=1, mode=normal(01/11)
                                                        // Optionally config register: 0xF5 => e.g. 0xA0

        // Wait a bit
        thread::sleep(Duration::from_millis(100));

        Ok(Bme280 {
            spi,
            calib,
            sea_level_hpa,
        })
    }

    /// Read the current measurements (temperature [°C], pressure [hPa], humidity [%], altitude [m])
    pub fn read(&mut self) -> Result<Bme280Measurements, Box<dyn Error>> {
        self.calib = read_calibration_data(&mut self.spi)?;
        // 4) Configure the sensor for a default oversampling & normal mode
        //    Example: humidity oversampling=1, pressure=1, temperature=1
        //    normal mode => 0xF2=0x01, 0xF4=0x27 or 0x25, etc.  Adjust as needed.
        write_register(&mut self.spi, REG_CTRL_HUM, 0x01)?; // osrs_h=1
        write_register(&mut self.spi, REG_CTRL_MEAS, 0x25)?; // osrs_p=1, osrs_t=1, mode=normal(01/11)
                                                             // Optionally config register: 0xF5 => e.g. 0xA0

        let data = read_register_n(&mut self.spi, REG_PRESS_MSB, 8)?;
        // 8 bytes: p[0..3], t[3..6], h[6..8]

        // Unpack raw
        let adc_p = ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
        let adc_t = ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);
        let adc_h = ((data[6] as i32) << 8) | (data[7] as i32);

        // 1) Temperature compensation => t_fine
        let t_fine = self.compensate_temp_fine(adc_t);
        let temperature_c = compensate_temperature(t_fine);

        // 2) Pressure
        let pressure_pa = compensate_pressure(adc_p, &self.calib, t_fine);
        let pressure_hpa = pressure_pa / 100.0;

        // 3) Humidity
        let humidity_pct = compensate_humidity(adc_h, &self.calib, t_fine);

        // 4) Altitude
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

    /// Helper: return t_fine for temperature compensation
    fn compensate_temp_fine(&self, adc_t: i32) -> i32 {
        let c = &self.calib;
        let var1 = (((adc_t >> 3) - ((c.dig_t1 as i32) << 1)) * (c.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (c.dig_t1 as i32)) * ((adc_t >> 4) - (c.dig_t1 as i32)))
            >> 12)
            * (c.dig_t3 as i32))
            >> 14;
        var1 + var2
    }
}

/* --- Low-level read/write helpers --- */

/// Read `length` bytes starting from `reg`, setting bit7=1 for read.
fn read_register_n(spi: &mut Spidev, reg: u8, length: usize) -> io::Result<Vec<u8>> {
    let mut tx_buf = vec![0; length + 1];
    tx_buf[0] = reg | 0x80; // set bit7 => read

    let mut rx_buf = vec![0; length + 1];

    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    spi.transfer(&mut transfer)?;

    // The first byte is the echoed register
    // Actual data is in rx_buf[1..]
    Ok(rx_buf[1..].to_vec())
}

/// Write a single byte to register `reg`, bit7=0
fn write_register(spi: &mut Spidev, reg: u8, val: u8) -> io::Result<()> {
    let tx_buf = [reg & 0x7F, val];
    let mut rx_buf = [0; 2];

    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    spi.transfer(&mut transfer)?;
    Ok(())
}

/* --- Calibration reading & compensation code --- */

/// Parse little-endian u16
fn le_u16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}
/// Parse little-endian i16
fn le_i16(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
}

/// Read all calibration data from the BME280
fn read_calibration_data(spi: &mut Spidev) -> io::Result<CalibData> {
    // 1) Read 24 bytes from 0x88..0x9F
    let calib1 = read_register_n(spi, 0x88, 24)?;
    // 2) dig_H1 at 0xA1
    let dig_h1 = read_register_n(spi, 0xA1, 1)?[0];
    // 3) 0xE1..0xE7 => 7 bytes
    let calib2 = read_register_n(spi, 0xE1, 7)?;

    // parse
    let dig_t1 = le_u16(&calib1[0..2]);
    let dig_t2 = le_i16(&calib1[2..4]);
    let dig_t3 = le_i16(&calib1[4..6]);

    let dig_p1 = le_u16(&calib1[6..8]);
    let dig_p2 = le_i16(&calib1[8..10]);
    let dig_p3 = le_i16(&calib1[10..12]);
    let dig_p4 = le_i16(&calib1[12..14]);
    let dig_p5 = le_i16(&calib1[14..16]);
    let dig_p6 = le_i16(&calib1[16..18]);
    let dig_p7 = le_i16(&calib1[18..20]);
    let dig_p8 = le_i16(&calib1[20..22]);
    let dig_p9 = le_i16(&calib1[22..24]);

    let dig_h2 = le_i16(&calib2[0..2]);
    let dig_h3 = calib2[2];
    // E4, E5 => H4, H5
    let e4 = calib2[3];
    let e5 = calib2[4];
    let e6 = calib2[5];
    let dig_h4 = ((e4 as i16) << 4) | ((e5 as i16) & 0x0F);
    let dig_h5 = ((e6 as i16) << 4) | ((e5 as i16) >> 4);
    let dig_h6 = calib2[6] as i8;

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

/// Convert t_fine to an actual temperature in °C
fn compensate_temperature(t_fine: i32) -> f32 {
    let t = (t_fine * 5 + 128) >> 8;
    t as f32 / 100.0
}

/// Convert raw pressure to Pa, given t_fine
fn compensate_pressure(adc_p: i32, c: &CalibData, t_fine: i32) -> f32 {
    let mut var1 = (t_fine as i64) - 128000;
    let mut var2 = var1 * var1 * (c.dig_p6 as i64);
    var2 += (var1 * (c.dig_p5 as i64)) << 17;
    var2 += (c.dig_p4 as i64) << 35;
    var1 = ((var1 * var1 * (c.dig_p3 as i64)) >> 8) + ((var1 * (c.dig_p2 as i64)) << 12);
    var1 = (((((1i64) << 47) + var1) * (c.dig_p1 as i64)) >> 33);

    if var1 == 0 {
        return 0.0;
    }
    let mut p = 1048576 - adc_p as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((c.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((c.dig_p8 as i64) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((c.dig_p7 as i64) << 4);
    p as f32 / 256.0
}

/// Convert raw humidity to % relative humidity, given t_fine
fn compensate_humidity(adc_h: i32, c: &CalibData, t_fine: i32) -> f32 {
    let mut v_x1_u32r = t_fine - 76800;
    v_x1_u32r =
        (((((adc_h << 14) as i32) - ((c.dig_h4 as i32) << 20) - ((c.dig_h5 as i32) * v_x1_u32r)
            + 16384)
            >> 15)
            * (((((((v_x1_u32r * (c.dig_h6 as i32)) >> 10)
                * (((v_x1_u32r * (c.dig_h3 as i32)) >> 11) + 32768))
                >> 10)
                + 2097152)
                * (c.dig_h2 as i32)
                + 8192)
                >> 14));
    v_x1_u32r -= (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (c.dig_h1 as i32)) >> 4);
    if v_x1_u32r < 0 {
        v_x1_u32r = 0;
    }
    if v_x1_u32r > 419430400 {
        v_x1_u32r = 419430400;
    }
    (v_x1_u32r >> 12) as f32 / 1024.0
}

/// Compute altitude from pressure in hPa vs. sea-level reference
fn altitude_from_pressure(pressure_hpa: f32, sea_level_hpa: f32) -> f32 {
    44330.0 * (1.0 - (pressure_hpa / sea_level_hpa).powf(0.1903))
}
