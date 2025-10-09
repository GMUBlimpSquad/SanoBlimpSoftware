# Getting Started

This guide will help you set up and run SanoBlimpSoftware on your blimp hardware.

## Prerequisites

Before you begin, ensure you have the following:

### Hardware
- **Raspberry Pi 4** or compatible ARM64 single-board computer
- **PCA9685 PWM controller** - For motor and servo control
- **BNO055 IMU sensor** - 9-axis absolute orientation sensor
- **BME280 barometric sensor** (optional) - For altitude measurement
- **ESCs** - BLHeli_S compatible electronic speed controllers (4x)
- **Motors** - Brushless motors compatible with your ESCs (4x)
- **Servos** (optional) - For additional control surfaces
- **Gamepad controller** - Any controller supported by gilrs (Xbox, PlayStation, etc.)
- **USB camera** (optional) - For autonomous mode with object detection
- **Power supply** - Adequate power for Raspberry Pi and motors
- **Network connection** - WiFi or Ethernet for base station communication

### Software
- **Rust toolchain** - Version 1.70 or later
- **Git** - For cloning the repository
- **cross** (optional) - For cross-compilation from development machine

### Development Machine (Optional)
If you're cross-compiling from a non-ARM machine:
- Linux, macOS, or Windows with WSL2
- Docker (required by `cross`)

## Installation

### Step 1: Install Rust

On your development machine or Raspberry Pi:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### Step 2: Set Up Cross-Compilation (Optional)

If building on a non-ARM machine:

```bash
# Install cross-compilation tool
cargo install cross

# Add ARM64 target
rustup target add aarch64-unknown-linux-gnu
```

### Step 3: Clone the Repository

```bash
git clone git@github.com:GMUBlimpSquad/SanoBlimpSoftware.git
cd SanoBlimpSoftware
```

### Step 4: Configure the System

Edit `config.toml` to match your hardware setup:

```toml
[blimp]
blimp_type = "sano"      # "sano" or "flappy"
blimp_version = 2         # Hardware version (1 or 2)

[server]
host = "192.168.1.156"   # Your base station IP address
port = 54321              # Data port
port_stat = 54320         # Status/control port

[motor]
m1_mul = 1.0              # Motor 1 multiplier (adjust for balance)
m2_mul = 1.0              # Motor 2 multiplier
m3_mul = 1.0              # Motor 3 multiplier
m4_mul = 1.0              # Motor 4 multiplier
midpoint_angle = 90.0     # ESC neutral angle (typically 90°)

[controller]
kp_x = 1.0                # Proportional gain for X-axis
kp_y = 1.0                # Proportional gain for Y-axis (yaw)
kp_z = 1.0                # Proportional gain for Z-axis (altitude)
kd_x = 0.1                # Derivative gain for X-axis
kd_y = 0.1                # Derivative gain for Y-axis
kd_z = 0.1                # Derivative gain for Z-axis
```

### Step 5: Build the Project

#### Option A: Native Build (on Raspberry Pi)
```bash
cargo build --release
```

#### Option B: Cross-Compile (from development machine)
```bash
cross build --target aarch64-unknown-linux-gnu --release
```

### Step 6: Prepare the Raspberry Pi

Install required system packages:

```bash
ssh pi@<RASPBERRY_PI_IP>

# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install dependencies
sudo apt-get install -y \
    libdbus-1-dev \
    pkg-config \
    libudev-dev \
    i2c-tools \
    build-essential

# Add user to required groups
sudo usermod -a -G i2c,gpio,spi,dialout pi

# Enable I2C and SPI
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
# Navigate to: Interface Options -> SPI -> Enable

# Reboot to apply changes
sudo reboot
```

### Step 7: Deploy to Raspberry Pi

From your development machine:

```bash
# Copy binary
scp target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware pi@<RASPBERRY_PI_IP>:~/

# Copy configuration
scp config.toml pi@<RASPBERRY_PI_IP>:~/

# Create rail position file (required)
ssh pi@<RASPBERRY_PI_IP> "echo '0' > ~/rail.pos"
```

### Step 8: Verify Hardware Connections

Before running, verify your hardware is connected correctly:

```bash
# Check I2C devices
i2cdetect -y 1

# You should see:
# 0x28 or 0x29: BNO055 IMU
# 0x40: PCA9685 PWM controller
# 0x76 or 0x77: BME280 (if installed)
```

Expected output:
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

## Usage

### Running the Main Program

SSH into your Raspberry Pi and run:

```bash
cd ~
sudo ./SanoBlimpSoftware
```

**Note**: `sudo` is required for GPIO, I2C, and SPI access.

### First Run

On the first run, the system will:
1. Initialize the PCA9685 PWM controller
2. Calibrate ESCs (you'll hear beeps)
3. Initialize the BNO055 IMU
4. Connect to the base station
5. Wait for controller input

### Controller Layout

#### Manual Control Mode
- **Left Stick Y-axis**: Forward/backward movement
- **Left Stick X-axis**: Left/right turning (yaw)
- **Right Stick Y-axis**: Altitude control (up/down)
- **Start Button**: Toggle between manual and autonomous mode
- **Right Trigger**: Switch to ball tracking mode (autonomous)
- **Left Trigger**: Switch to goal tracking mode (autonomous)
- **East Button** (B/Circle): Execute scoring maneuver
- **West Button** (X/Square): Save current camera frame

#### Autonomous Mode Indicators
- When autonomous mode is active, the blimp will track objects based on the selected state
- Red indicator in telemetry shows autonomous mode is active
- The blimp will automatically adjust position to center on detected objects

### Testing Individual Components

Before full flight, test components individually:

#### Test IMU
```bash
cargo run --bin bno
```
Should display orientation data (Euler angles or quaternions).

#### Test Servos
```bash
cargo run --bin servo
```
Should move servos through their range.

#### Test Bluetooth (if needed)
```bash
cargo run --bin gatt_client
```

## Hardware Setup

### Wiring Diagram

#### I2C Connections
```
Raspberry Pi          Device
GPIO 2 (SDA) -------- SDA (BNO055, PCA9685)
GPIO 3 (SCL) -------- SCL (BNO055, PCA9685)
3.3V ---------------- VCC (BNO055)
5V ------------------ VCC (PCA9685)
GND ----------------- GND (All devices)
```

#### PCA9685 to ESC/Servo Connections
```
PCA9685 Channel      Connection
0 (C0) ------------- Motor 1 ESC signal
1 (C1) ------------- Motor 2 ESC signal
2 (C2) ------------- Motor 3 ESC signal
3 (C3) ------------- Motor 4 ESC signal
4 (C4) ------------- Servo 1 signal (optional)
5 (C5) ------------- Servo 2 signal (optional)
6 (C6) ------------- Servo 3 signal (optional)
7 (C7) ------------- Servo 4 signal (optional)
```

#### GPIO Connections
```
GPIO 23 ------------- Rail position sensor (optional)
```

### Motor Configuration

The Sano blimp uses a quad-motor configuration:
- **M1 & M2**: Forward thrust motors
- **M3 & M4**: Lateral/altitude motors (optional, depending on version)

Adjust `m1_mul` and `m2_mul` in `config.toml` if motors have different characteristics.

### ESC Calibration

ESCs are automatically calibrated on startup through this sequence:
1. **Max throttle** - Sent immediately (180°)
2. **Wait for ESC beep** - Indicates recognition
3. **Min throttle** - Arms the ESCs (0°)
4. **Neutral position** - Sets to configured midpoint

If ESCs don't arm, check:
- Power supply is adequate
- ESC signal wires are properly connected
- `midpoint_angle` is correct for your ESCs (usually 90°)

## Troubleshooting

### Issue: "Failed to initialize I2C device"
**Solution**: Ensure I2C is enabled:
```bash
sudo raspi-config
# Interface Options -> I2C -> Yes
sudo reboot
```

### Issue: "Permission denied" errors
**Solution**: Either run with `sudo` or add user to groups:
```bash
sudo usermod -a -G i2c,gpio,spi,dialout $USER
# Log out and back in
```

### Issue: Motors not responding
**Checklist**:
1. Check power supply is connected and adequate
2. Verify ESC connections to PCA9685
3. Check PCA9685 appears in `i2cdetect`
4. Verify ESC calibration sequence completed (listen for beeps)
5. Check `midpoint_angle` in config.toml

### Issue: IMU not found
**Solution**: Check BNO055 connections and address:
```bash
i2cdetect -y 1
# Should show 28 or 29
```
The code uses alternative address (0x29). If your IMU is at 0x28, modify `src/lib/blimp.rs`:
```rust
// Change from:
let mut imu = Bno055::new(dev).with_alternative_address();
// To:
let mut imu = Bno055::new(dev);
```

### Issue: Controller not detected
**Solution**: 
1. Check controller is connected via USB or Bluetooth
2. Verify controller is supported by gilrs
3. Check `dmesg` for USB connection messages
4. Test with a different USB port

### Issue: Can't connect to base station
**Solution**:
1. Verify network connectivity: `ping <base_station_ip>`
2. Check firewall isn't blocking UDP ports 54320 and 54321
3. Ensure base station is running and listening
4. Verify IP address in `config.toml` is correct

### Issue: "rail.pos not found"
**Solution**: Create the file:
```bash
echo '0' > ~/rail.pos
```

## Next Steps

Now that you have the system running:

1. **Tune PID Gains**: See [Autonomous Control](06_autonomous_control___autonomous__.md) for tuning guidance
2. **Configure Motors**: Adjust multipliers in config.toml for balanced flight
3. **Set Up Object Detection**: Configure camera and detection system
4. **Understand the Code**: Read through the [documentation chapters](index.md)
5. **Monitor Telemetry**: Use base station to view real-time sensor data

## Additional Resources

- [Hardware Actuation Details](02_hardware_actuation___pcaactuator__.md)
- [Configuration Reference](03_configuration___config__.md)
- [Sensor Integration](05_sensor_input_state__conceptual_.md)
- [Object Detection Setup](04_object_detection___detection__.md)
