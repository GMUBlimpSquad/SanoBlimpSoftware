# Hardware Guide

This guide provides detailed information about the hardware components, wiring, and setup for SanoBlimpSoftware.

## Overview

The SanoBlimpSoftware system is designed to run on a Raspberry Pi and interfaces with various sensors, motor controllers, and actuators to control an autonomous blimp.

## Hardware Components

### Required Components

#### 1. Raspberry Pi (Single Board Computer)
- **Model**: Raspberry Pi 4 Model B (recommended)
- **RAM**: 2GB or higher
- **Storage**: 16GB+ microSD card
- **OS**: Raspberry Pi OS (64-bit)
- **Purpose**: Main control computer running SanoBlimpSoftware

**Alternatives**: Any ARM64 Linux SBC with I2C, GPIO, and network connectivity

#### 2. PCA9685 PWM Controller
- **Type**: 16-Channel 12-bit PWM/Servo Driver
- **Interface**: I2C
- **Address**: 0x40 (default) or 0x41
- **Supply Voltage**: 5V
- **Purpose**: Controls ESCs and servos via PWM signals
- **Specifications**:
  - Frequency: 40-1000 Hz (configured to 60 Hz for servos, 63 Hz for motors)
  - Resolution: 12-bit (4096 steps)
  - Output: 16 independent PWM channels

**Where to Buy**: Adafruit PCA9685, or compatible clones

#### 3. BNO055 IMU (Inertial Measurement Unit)
- **Type**: 9-axis Absolute Orientation Sensor
- **Interface**: I2C
- **Address**: 0x28 (default) or 0x29 (alternative)
- **Supply Voltage**: 3.3V
- **Purpose**: Provides orientation, angular velocity, and acceleration data
- **Features**:
  - 3-axis accelerometer
  - 3-axis gyroscope
  - 3-axis magnetometer
  - Built-in sensor fusion (outputs quaternions or Euler angles)

**Note**: SanoBlimpSoftware uses alternative address (0x29) by default

#### 4. BME280 Barometric Sensor (Optional)
- **Type**: Combined temperature, humidity, and pressure sensor
- **Interface**: I2C or SPI
- **Address**: 0x76 or 0x77 (I2C)
- **Supply Voltage**: 3.3V
- **Purpose**: Altitude estimation via barometric pressure
- **Interface Choice**:
  - Version 1: SPI (`/dev/spidev0.0`)
  - Version 2: I2C (`/dev/i2c-1`)

**Note**: Currently commented out in latest code but supported

#### 5. Electronic Speed Controllers (ESCs)
- **Quantity**: 4
- **Type**: BLHeli_S compatible
- **Input**: PWM (1000-2000 µs pulse width)
- **Output**: 3-phase brushless motor control
- **Purpose**: Control motor speed based on PWM signals
- **Requirements**:
  - Support for 1500 µs neutral/stop position
  - Auto-calibration capable
  - Sufficient current rating for motors

#### 6. Brushless Motors
- **Quantity**: 4
- **Type**: Outrunner brushless DC motors
- **Suggested**: 
  - KV rating appropriate for blimp (low KV, high torque)
  - Lightweight
  - Compatible with ESCs
- **Configuration**:
  - M1, M2: Forward thrust
  - M3, M4: Lateral/altitude control (configuration-dependent)

#### 7. Servos (Optional)
- **Quantity**: Up to 4
- **Type**: Standard analog servos
- **Control**: PWM (500-2500 µs pulse width)
- **Purpose**: Control surfaces, rudders, or grabbers
- **Mounting**: Channels C4-C7 on PCA9685

#### 8. Gamepad Controller
- **Type**: Any USB or Bluetooth gamepad
- **Compatibility**: Must be supported by `gilrs` Rust library
- **Examples**:
  - Xbox One/Series controller
  - PlayStation DualShock 4/DualSense
  - Nintendo Switch Pro Controller
  - Generic USB gamepads
- **Connection**: USB or Bluetooth

#### 9. USB Camera (Optional, for Autonomous Mode)
- **Type**: USB webcam or Raspberry Pi Camera Module (with USB adapter)
- **Interface**: Serial communication (`/dev/ttyACM0`)
- **Purpose**: Object detection for autonomous navigation
- **Requirements**:
  - Outputs JSON detection data via serial
  - Provides base64-encoded images
  - See [Object Detection](04_object_detection___detection__.md) for protocol details

**Note**: Requires separate object detection firmware

#### 10. Power Supply
- **For Raspberry Pi**: 5V, 3A minimum (USB-C)
- **For Motors**: Separate battery (typically LiPo)
  - Voltage: Match motor/ESC specifications (e.g., 3S LiPo = 11.1V)
  - Capacity: Sufficient for flight time requirements
  - Discharge rating: Match total motor current draw
- **Important**: Keep logic power (Pi) separate from motor power to prevent voltage drops/noise

#### 11. Miscellaneous
- **Wiring**: Jumper wires, servo extension cables
- **Connectors**: For ESCs and servos
- **Mounting**: Appropriate hardware for blimp frame
- **Voltage Regulator**: If sharing battery between Pi and motors (not recommended)
- **Rail Position Sensor** (optional): GPIO-connected sensor on pin 23

## Hardware Versions

SanoBlimpSoftware supports two hardware configurations:

### Version 1
- BME280 via SPI (`/dev/spidev0.0`)
- PCA9685 at default I2C address (0x40)
- BNO055 at alternative I2C address (0x29)

### Version 2
- BME280 via I2C (if used)
- PCA9685 at address 0x40
- BNO055 at alternative I2C address (0x29)

Specify version in `config.toml`:
```toml
[blimp]
blimp_version = 2  # or 1
```

## Wiring Diagrams

### I2C Bus Connections

```
Raspberry Pi (40-pin header)
┌────────────────────────────┐
│ Pin 1  [3.3V]  [5V]  Pin 2 │
│ Pin 3  [SDA]   [5V]  Pin 4 │
│ Pin 5  [SCL]   [GND] Pin 6 │
│        ...                  │
└────────────────────────────┘

I2C Devices:
┌──────────────────────┐      ┌──────────────────────┐      ┌──────────────────────┐
│     BNO055 IMU       │      │   PCA9685 Driver     │      │   BME280 (optional)  │
├──────────────────────┤      ├──────────────────────┤      ├──────────────────────┤
│ VIN ← 3.3V (Pin 1)   │      │ VCC ← 5V (Pin 2/4)   │      │ VIN ← 3.3V           │
│ GND ← GND (Pin 6)    │      │ GND ← GND (Pin 6)    │      │ GND ← GND            │
│ SDA ← SDA (Pin 3)    │      │ SDA ← SDA (Pin 3)    │      │ SDA ← SDA (I2C mode) │
│ SCL ← SCL (Pin 5)    │      │ SCL ← SCL (Pin 5)    │      │ SCL ← SCL (I2C mode) │
│ Addr: 0x29           │      │ Addr: 0x40           │      │ Addr: 0x76/0x77      │
└──────────────────────┘      └──────────────────────┘      └──────────────────────┘

Note: All I2C devices share the same SDA/SCL bus
```

### PCA9685 to Motor/Servo Connections

```
PCA9685 Channels              ESCs / Servos
┌──────────────────┐         ┌──────────────────┐
│ C0  ─────────────┼────────→│ Motor 1 ESC      │
│ C1  ─────────────┼────────→│ Motor 2 ESC      │
│ C2  ─────────────┼────────→│ Motor 3 ESC      │
│ C3  ─────────────┼────────→│ Motor 4 ESC      │
│ C4  ─────────────┼────────→│ Servo 1 (opt)    │
│ C5  ─────────────┼────────→│ Servo 2 (opt)    │
│ C6  ─────────────┼────────→│ Servo 3 (opt)    │
│ C7  ─────────────┼────────→│ Servo 4 (opt)    │
│ C8-C15 (unused)  │         └──────────────────┘
│                  │
│ V+  ← External   │  ⚠️ Connect motor battery + (for ESC/servo power)
│     Power Supply │     Do NOT connect to Raspberry Pi power!
└──────────────────┘
```

**Important**: 
- PCA9685 V+ powers the servos/ESCs, not the PCA9685 chip itself
- PCA9685 logic is powered via I2C VCC (5V from Pi)
- ESC/Servo ground must be common with Raspberry Pi ground

### GPIO Connections

```
Raspberry Pi GPIO
┌──────────────────────┐
│ GPIO 23 ─────────────┼─────→ Rail Position Sensor (optional)
│                      │        - Input with pull-up
│                      │        - Rising edge interrupt
└──────────────────────┘
```

### Complete System Wiring

```
                         Power Supply
                              │
                    ┌─────────┴─────────┐
                    │                   │
                5V, 3A              LiPo Battery
                    │              (11.1V typical)
                    │                   │
           ┌────────▼────────┐          │
           │  Raspberry Pi 4 │          │
           │                 │          │
           │  I2C (SDA/SCL)  │          │
           └─────┬─┬─┬───────┘          │
                 │ │ │                  │
        ┌────────┘ │ └────────┐         │
        │          │          │         │
     ┌──▼──┐   ┌──▼───┐   ┌──▼───┐     │
     │BNO  │   │PCA   │   │BME   │     │
     │055  │   │9685  │   │280   │     │
     └─────┘   └──┬───┘   └──────┘     │
                  │                    │
         ┌────────┼────────┬───────┬───┴─────┐
         │        │        │       │         │
      ┌──▼─┐  ┌──▼─┐  ┌──▼─┐  ┌──▼─┐     ┌─▼──┐
      │ESC │  │ESC │  │ESC │  │ESC │     │BEC │ (optional)
      │ 1  │  │ 2  │  │ 3  │  │ 4  │     │5V  │
      └──┬─┘  └──┬─┘  └──┬─┘  └──┬─┘     └────┘
         │       │       │       │
      ┌──▼─┐  ┌──▼─┐  ┌──▼─┐  ┌──▼─┐
      │ M1 │  │ M2 │  │ M3 │  │ M4 │
      └────┘  └────┘  └────┘  └────┘
```

## Assembly Instructions

### Step 1: Prepare the Raspberry Pi
1. Install Raspberry Pi OS (64-bit recommended)
2. Enable I2C: `sudo raspi-config` → Interface Options → I2C
3. Enable SPI (if using BME280 on SPI): Interface Options → SPI
4. Update system: `sudo apt-get update && sudo apt-get upgrade`

### Step 2: Connect I2C Devices
1. **Power off** Raspberry Pi
2. Connect BNO055:
   - VIN → 3.3V (Pin 1)
   - GND → GND (Pin 6)
   - SDA → SDA (Pin 3)
   - SCL → SCL (Pin 5)
3. Connect PCA9685:
   - VCC → 5V (Pin 2 or 4)
   - GND → GND (Pin 6 or other GND pin)
   - SDA → SDA (Pin 3, shared with BNO055)
   - SCL → SCL (Pin 5, shared with BNO055)
4. (Optional) Connect BME280 similarly

### Step 3: Connect PCA9685 to ESCs
1. Connect ESC signal wires to PCA9685 channels 0-3
2. Connect ESC ground wires to PCA9685 GND
3. **Do not connect ESC power** to PCA9685 V+ yet
4. Ensure proper polarity (signal, power, ground)

### Step 4: Connect Motors to ESCs
1. Connect motor wires to ESC outputs
2. If motor spins wrong direction, swap any two motor wires

### Step 5: Power Connections
1. Connect main battery to ESC power inputs (all ESCs in parallel)
2. Optionally connect battery to PCA9685 V+ (for servo power)
3. Use a BEC or voltage regulator if powering Pi from main battery
4. **Recommended**: Use separate power supplies for Pi and motors

### Step 6: Verification
1. Power on Raspberry Pi
2. Check I2C devices: `i2cdetect -y 1`
3. Verify device addresses appear correctly

## I2C Address Configuration

Run `i2cdetect -y 1` to verify devices:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- 76 --
```

Expected devices:
- `28` or `29`: BNO055
- `40`: PCA9685
- `76` or `77`: BME280 (if present)

### Changing I2C Addresses

If you need to change addresses:

**BNO055**: Solder ADR pin to change between 0x28 and 0x29

**PCA9685**: Solder address jumpers (A0-A5) to change from 0x40

**BME280**: SDO pin determines 0x76 (low) or 0x77 (high)

## Motor Configuration

### Motor Layout

```
        Forward
          ↑
    M1         M2
     ●─────────●
     │         │
     │  Blimp  │
     │         │
     ●─────────●
    M3         M4
```

### Motor Functions

**Sano Configuration**:
- **M1 & M2**: Primary forward thrust (differential thrust for yaw control)
- **M3 & M4**: Reserved for additional control (version-dependent)

**Flappy Configuration**:
- Uses servos for wing actuation instead of quad motors
- Motor configuration differs significantly

### ESC Calibration

The software automatically calibrates ESCs on startup:

1. **Max Throttle (180°)**: Sent to all ESCs
2. **Wait**: ~100ms for ESC acknowledgment beep
3. **Min Throttle (0°)**: Arms the ESCs
4. **Neutral (configured midpoint)**: Ready for operation

**Manual Calibration** (if auto fails):
1. Disconnect motors from ESCs (safety)
2. Power on Raspberry Pi
3. Run calibration program
4. Follow ESC beep patterns
5. Power cycle ESCs

### PWM Specifications

**For Motors (ESCs)**:
- Frequency: 63 Hz
- Min pulse: 1000 µs (0% throttle)
- Mid pulse: 1500 µs (neutral/stop)
- Max pulse: 2000 µs (100% throttle)

**For Servos**:
- Frequency: 60 Hz
- Min pulse: 500 µs (0°)
- Max pulse: 2500 µs (180°)
- Neutral: 1500 µs (90°)

These are configured in `src/lib/blimp.rs`:
```rust
const PWM_FREQUENCY_MOTOR: f32 = 63.0;
const MIN_PULSE: f32 = 1000.0;
const MAX_PULSE: f32 = 2000.0;
const MID_PULSE: f32 = 1500.0;
```

## Sensor Configuration

### BNO055 IMU

**Operation Mode**: NDOF (Nine Degrees of Freedom)
- Combines accelerometer, gyroscope, and magnetometer
- Provides sensor fusion for absolute orientation

**Calibration**:
- BNO055 auto-calibrates over time
- Move blimp through various orientations during first minute
- Calibration status available via `get_calibration_status()`

**Output**:
- Quaternions (preferred for calculations)
- Euler angles (roll, pitch, yaw)
- Angular velocity
- Linear acceleration

### BME280 Barometer

**Sea Level Pressure**: Set in driver (default 101325 Pa)

**Altitude Calculation**:
```rust
const ALTITUDE_FACTOR: f32 = 44330.0;
const ISA_EXPONENT: f32 = 1.0 / 5.255;
altitude = ALTITUDE_FACTOR * (1.0 - (pressure / sea_level_pressure).powf(ISA_EXPONENT))
```

**Ground Reference**: Stored on first reading for relative altitude

## Troubleshooting

### I2C Issues

**Problem**: Devices not detected
- Check wiring connections
- Verify I2C is enabled: `sudo raspi-config`
- Check pull-up resistors (usually on breakout boards)
- Try different I2C speed: edit `/boot/config.txt`:
  ```
  dtparam=i2c_arm=on,i2c_arm_baudrate=100000
  ```

**Problem**: Bus errors or data corruption
- Shorten wire lengths
- Add stronger pull-up resistors (2.2kΩ to 4.7kΩ)
- Keep I2C wires away from motor wires

### Motor Problems

**Problem**: Motors don't spin
- Check ESC power supply
- Verify ESC calibration completed
- Check motor-to-ESC connections
- Test with a servo tester

**Problem**: Motors spin at wrong speeds
- Adjust motor multipliers in `config.toml`
- Check for loose connections
- Verify PWM signals with oscilloscope

**Problem**: Motors jitter or vibrate
- Check propeller balance
- Verify ESC firmware is updated
- Ensure adequate power supply

### Sensor Problems

**Problem**: IMU data is noisy
- Calibrate IMU properly
- Mount IMU rigidly, away from motors
- Shield from magnetic interference

**Problem**: Altitude readings drift
- Recalibrate ground altitude
- BME280 is sensitive to temperature changes
- Ensure good ventilation around sensor

## Safety Considerations

⚠️ **Important Safety Guidelines**:

1. **Power Safety**:
   - Always disconnect motors when testing
   - Use appropriate battery protections (fuses, BEC)
   - Never short battery terminals

2. **Propeller Safety**:
   - Remove propellers during initial testing
   - Keep hands away from spinning propellers
   - Use propeller guards

3. **Electrical Safety**:
   - Verify polarity before connecting power
   - Use appropriate gauge wire for current loads
   - Secure all connections

4. **Testing**:
   - Test components individually before integration
   - Start with low throttle values
   - Have an emergency stop plan

5. **Operating Environment**:
   - Adequate space for blimp operation
   - No obstacles or people nearby
   - Avoid outdoor operation in wind

## Recommended Tools

- Multimeter
- Soldering iron
- Wire strippers
- Heat shrink tubing
- Oscilloscope (for PWM debugging)
- Servo tester (for ESC testing)
- I2C scanner tool

## Additional Resources

- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [PCA9685 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

## See Also

- [Getting Started](getting_started.md) - Setup and installation
- [Hardware Actuation](02_hardware_actuation___pcaactuator__.md) - PCA9685 control details
- [Configuration](03_configuration___config__.md) - Software configuration
