/***************************************************************************
Modified BSD License
====================

Copyright © 2016, Andrei Vainik
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of the organization nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Derived from the original C source and adapted to Rust for SPI usage.
***************************************************************************/

use std::error::Error;
use std::io;
use std::mem;
use std::thread;
use std::time;
use std::time::Duration;

use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

// BME280 register constants (same as in your C code)
const BME280_REG_CHIPID: u8 = 0xD0;
const BME280_REG_SOFTRESET: u8 = 0xE0;
const BME280_RESET: u8 = 0xB6;

// Calibration data registers
const BME280_REGISTER_DIG_T1: u8 = 0x88;
const BME280_REGISTER_DIG_T2: u8 = 0x8A;
const BME280_REGISTER_DIG_T3: u8 = 0x8C;

const BME280_REGISTER_DIG_P1: u8 = 0x8E;
const BME280_REGISTER_DIG_P2: u8 = 0x90;
const BME280_REGISTER_DIG_P3: u8 = 0x92;
const BME280_REGISTER_DIG_P4: u8 = 0x94;
const BME280_REGISTER_DIG_P5: u8 = 0x96;
const BME280_REGISTER_DIG_P6: u8 = 0x98;
const BME280_REGISTER_DIG_P7: u8 = 0x9A;
const BME280_REGISTER_DIG_P8: u8 = 0x9C;
const BME280_REGISTER_DIG_P9: u8 = 0x9E;

const BME280_REGISTER_DIG_H1: u8 = 0xA1;
const BME280_REGISTER_DIG_H2: u8 = 0xE1;
const BME280_REGISTER_DIG_H3: u8 = 0xE3;
const BME280_REGISTER_DIG_H4: u8 = 0xE4;
const BME280_REGISTER_DIG_H5: u8 = 0xE5;
const BME280_REGISTER_DIG_H6: u8 = 0xE7;

// Control and data registers
const BME280_REGISTER_CONTROLHUMID: u8 = 0xF2;
const BME280_REGISTER_CONTROL: u8 = 0xF4;
const BME280_REGISTER_CONFIG: u8 = 0xF5;

const BME280_REGISTER_PRESSUREDATA: u8 = 0xF7;
const BME280_REGISTER_TEMPDATA: u8 = 0xFA;
const BME280_REGISTER_HUMIDDATA: u8 = 0xFD;

// For altitude calculation
const MEAN_SEA_LEVEL_PRESSURE: f32 = 1013.0; // hPa

/// BME280 calibration data (mirrors your C struct)
#[derive(Debug)]
struct Bme280CalibData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,

    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,

    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

/// Container for raw data
#[derive(Debug)]
struct Bme280RawData {
    temperature: u32,
    pressure: u32,
    humidity: u32,
}

/// Read N bytes from a register using spidev, setting bit7=1 for read.
fn spi_read_register(spi: &mut Spidev, reg: u8, length: usize) -> io::Result<Vec<u8>> {
    // TX buffer: [register_with_read_bit, dummy...]
    let mut tx_buf = vec![0; length + 1];
    tx_buf[0] = reg | 0x80; // set bit 7 => read

    let mut rx_buf = vec![0; length + 1];

    // Use SpidevTransfer to do a full-duplex transaction
    {
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        spi.transfer(&mut transfer)?;
    }

    // The first byte of rx_buf is the "echo" of the register address
    // The actual returned data starts at rx_buf[1..]
    Ok(rx_buf[1..].to_vec())
}

/// Write 1 byte to a register, bit7=0 for write.
fn spi_write_register(spi: &mut Spidev, reg: u8, value: u8) -> io::Result<()> {
    let mut tx_buf = [reg & 0x7F, value]; // clear bit7
                                          // We'll read these two bytes back, though typically we don't need them
    let mut rx_buf = [0; 2];

    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    spi.transfer(&mut transfer)?;
    Ok(())
}

/// Read an 8-bit register
fn spi_read8(spi: &mut Spidev, reg: u8) -> io::Result<u8> {
    let data = spi_read_register(spi, reg, 1)?;
    Ok(data[0])
}

/// Read a 16-bit little-endian register
fn spi_read16_le(spi: &mut Spidev, reg: u8) -> io::Result<u16> {
    let data = spi_read_register(spi, reg, 2)?;
    // little-endian => data[0] is low, data[1] is high
    Ok(u16::from_le_bytes([data[0], data[1]]))
}

/// Read a signed 16-bit little-endian register
fn spi_reads16_le(spi: &mut Spidev, reg: u8) -> io::Result<i16> {
    Ok(spi_read16_le(spi, reg)? as i16)
}

/// Read the BME280 calibration data
fn read_calibration_data(spi: &mut Spidev) -> io::Result<Bme280CalibData> {
    let dig_t1 = spi_read16_le(spi, BME280_REGISTER_DIG_T1)?;
    let dig_t2 = spi_reads16_le(spi, BME280_REGISTER_DIG_T2)?;
    let dig_t3 = spi_reads16_le(spi, BME280_REGISTER_DIG_T3)?;

    let dig_p1 = spi_read16_le(spi, BME280_REGISTER_DIG_P1)?;
    let dig_p2 = spi_reads16_le(spi, BME280_REGISTER_DIG_P2)?;
    let dig_p3 = spi_reads16_le(spi, BME280_REGISTER_DIG_P3)?;
    let dig_p4 = spi_reads16_le(spi, BME280_REGISTER_DIG_P4)?;
    let dig_p5 = spi_reads16_le(spi, BME280_REGISTER_DIG_P5)?;
    let dig_p6 = spi_reads16_le(spi, BME280_REGISTER_DIG_P6)?;
    let dig_p7 = spi_reads16_le(spi, BME280_REGISTER_DIG_P7)?;
    let dig_p8 = spi_reads16_le(spi, BME280_REGISTER_DIG_P8)?;
    let dig_p9 = spi_reads16_le(spi, BME280_REGISTER_DIG_P9)?;

    let dig_h1 = spi_read8(spi, BME280_REGISTER_DIG_H1)?;
    let dig_h2 = spi_reads16_le(spi, BME280_REGISTER_DIG_H2)?;
    let dig_h3 = spi_read8(spi, BME280_REGISTER_DIG_H3)?;

    // H4, H5, H6 are trickier to read – partial bits from E4..E6.  Let's do that manually:
    let h4_0 = spi_read8(spi, BME280_REGISTER_DIG_H4)?; // E4
    let h4_1 = spi_read8(spi, BME280_REGISTER_DIG_H4 + 1)?; // E5
    let h5_0 = spi_read8(spi, BME280_REGISTER_DIG_H5)?; // E5
    let h5_1 = spi_read8(spi, BME280_REGISTER_DIG_H5 + 1)?; // E6
    let dig_h4 = ((h4_0 as i16) << 4) | ((h4_1 as i16) & 0x0F);
    let dig_h5 = ((h5_1 as i16) << 4) | ((h5_0 as i16) >> 4);

    let dig_h6 = spi_read8(spi, BME280_REGISTER_DIG_H6)? as i8;

    Ok(Bme280CalibData {
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

/// Read raw sensor data from the BME280 (temperature, pressure, humidity)
fn get_raw_data(spi: &mut Spidev) -> io::Result<Bme280RawData> {
    // We can read 8 bytes starting at 0xF7:
    // F7..F9 => pressure, FA..FC => temperature, FD..FE => humidity
    let data = spi_read_register(spi, BME280_REGISTER_PRESSUREDATA, 8)?;

    // pressure is 20 bits => pmsb[0], plsb[1], pxsb[2], ignoring last 4 bits in pxsb
    let adc_p = (((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4));

    // temperature is next 20 bits => tmsb[3], tlsb[4], txsb[5]
    let adc_t = (((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4));

    // humidity => hmsb[6], hlsb[7]
    let adc_h = (((data[6] as u32) << 8) | (data[7] as u32));

    Ok(Bme280RawData {
        temperature: adc_t,
        pressure: adc_p,
        humidity: adc_h,
    })
}

/// The official BME280 datasheet function to get t_fine from raw temp
fn get_temperature_calibration(cal: &Bme280CalibData, adc_t: i32) -> i32 {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * (cal.dig_t2 as i32)) >> 11;
    let var2 = (((((adc_t >> 4) - (cal.dig_t1 as i32)) * ((adc_t >> 4) - (cal.dig_t1 as i32)))
        >> 12)
        * (cal.dig_t3 as i32))
        >> 14;
    var1 + var2
}

/// Convert t_fine into actual temperature (°C)
fn compensate_temperature(t_fine: i32) -> f32 {
    let t = (t_fine * 5 + 128) >> 8;
    t as f32 / 100.0
}

/// Convert raw pressure using t_fine
fn compensate_pressure(adc_p: i32, cal: &Bme280CalibData, t_fine: i32) -> f32 {
    let mut var1 = (t_fine as i64) - 128000;
    let mut var2 = var1 * var1 * (cal.dig_p6 as i64);
    var2 = var2 + ((var1 * (cal.dig_p5 as i64)) << 17);
    var2 = var2 + ((cal.dig_p4 as i64) << 35);
    var1 = ((var1 * var1 * (cal.dig_p3 as i64)) >> 8) + ((var1 * (cal.dig_p2 as i64)) << 12);
    var1 = ((((1i64) << 47) + var1) * (cal.dig_p1 as i64)) >> 33;

    if var1 == 0 {
        return 0.0;
    }
    let mut p = 1048576 - adc_p as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((cal.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((cal.dig_p8 as i64) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((cal.dig_p7 as i64) << 4);
    p as f32 / 256.0
}

/// Convert raw humidity using t_fine
fn compensate_humidity(adc_h: i32, cal: &Bme280CalibData, t_fine: i32) -> f32 {
    let mut v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_h << 14) as i32)
        - ((cal.dig_h4 as i32) << 20)
        - ((cal.dig_h5 as i32) * v_x1_u32r)
        + 16384)
        >> 15)
        * (((((((v_x1_u32r * (cal.dig_h6 as i32)) >> 10)
            * (((v_x1_u32r * (cal.dig_h3 as i32)) >> 11) + 32768))
            >> 10)
            + 2097152)
            * (cal.dig_h2 as i32)
            + 8192)
            >> 14));
    v_x1_u32r -= (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (cal.dig_h1 as i32)) >> 4);
    if v_x1_u32r < 0 {
        v_x1_u32r = 0;
    }
    if v_x1_u32r > 419430400 {
        v_x1_u32r = 419430400;
    }
    (v_x1_u32r >> 12) as f32 / 1024.0
}

/// Compute altitude using the standard BMP180 formula
fn get_altitude(pressure_hpa: f32) -> f32 {
    // h = 44330 * (1 - (p/p0)^(0.1903))
    44330.0 * (1.0 - (pressure_hpa / MEAN_SEA_LEVEL_PRESSURE).powf(0.190294957f32))
}

fn main() -> Result<(), Box<dyn Error>> {
    // 1) Open the SPI device (e.g. /dev/spidev0.0).  Adjust to your system.
    let mut spi = Spidev::open("/dev/spidev0.0")?;

    // 2) Configure SPI (1MHz, mode0, MSB first)
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .lsb_first(false)
        .build();
    spi.configure(&options)?;

    // 3) Read the BME280 chip ID => 0x60
    let chip_id = spi_read8(&mut spi, BME280_REG_CHIPID)?;
    println!("BME280 chip_id = 0x{:X}", chip_id);
    if chip_id != 0x60 {
        eprintln!("Unexpected chip ID, might not be a BME280!");
        // proceed or exit
    }

    loop {
        // 4) Read calibration data
        let cal = read_calibration_data(&mut spi)?;
        //println!("Calibration data: {:?}", cal);

        // 5) Configure BME280:
        //    e.g., humidity oversample = 1, pressure & temp oversample = 1, normal mode
        //    => same as '0xF2=0x01, 0xF4=0x25' in your C code
        spi_write_register(&mut spi, BME280_REGISTER_CONTROLHUMID, 0x01)?; // oversample humidity x1
        spi_write_register(&mut spi, BME280_REGISTER_CONTROL, 0x25)?; // oversample p/t x1, normal mode

        // Wait a bit for sensor to settle
        thread::sleep(Duration::from_millis(100));

        // 6) Read raw data
        let raw = get_raw_data(&mut spi)?;
        println!("Raw data: {:?}", raw);

        // 7) Apply compensation
        let adc_t = raw.temperature as i32;
        let adc_p = raw.pressure as i32;
        let adc_h = raw.humidity as i32;

        let t_fine = get_temperature_calibration(&cal, adc_t);
        let temp_c = compensate_temperature(t_fine);
        let press_pa = compensate_pressure(adc_p, &cal, t_fine);
        let hum_rh = compensate_humidity(adc_h, &cal, t_fine);

        // Pressure in hPa
        let press_hpa = press_pa / 100.0;
        let altitude_m = get_altitude(press_hpa);

        // 8) Print results (JSON style, as in your C code)
        println!(
            "{{\"sensor\":\"bme280\", \"humidity\":{:.2}, \"pressure\":{:.2},\
         \"temperature\":{:.2}, \"altitude\":{:.2}, \"timestamp\":{}}}",
            hum_rh,
            press_hpa,
            temp_c,
            altitude_m,
            time::SystemTime::now()
                .duration_since(time::SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        );

        thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
