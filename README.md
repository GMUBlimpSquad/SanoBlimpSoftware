# SanoBlimpSoftware

An autonomous blimp control system written in Rust for embedded Linux platforms (Raspberry Pi). This project provides real-time control, autonomous navigation, object detection, and telemetry for robotic blimps.

## Overview

SanoBlimpSoftware is a comprehensive control system designed for autonomous aerial blimps. It supports both manual control via gamepad and autonomous operation using computer vision-based object tracking. The system uses a PID control loop for position and altitude management, communicates with a base station over UDP, and integrates various sensors including IMU (BNO055) and barometric pressure sensors (BME280).

### Key Features

- **Dual Control Modes**: Manual control via gamepad or autonomous operation
- **Real-time Object Detection**: Serial-based object detection with bounding box visualization
- **PID-based Autonomous Control**: Position and altitude hold with configurable gains
- **Multi-sensor Integration**: IMU orientation tracking and barometric altitude measurement
- **Base Station Communication**: UDP-based telemetry and command interface
- **Hardware Abstraction**: Support for multiple blimp types (Sano, Flappy)
- **Motor Control**: ESC initialization and PWM control via PCA9685
- **Configurable**: TOML-based configuration for gains, motors, and network settings

## Hardware Requirements

### Supported Platforms
- Raspberry Pi (tested on Raspberry Pi 4)
- ARM64 Linux (aarch64-unknown-linux-gnu)

### Required Hardware
- **Motor Controller**: PCA9685 PWM controller
- **IMU**: BNO055 9-axis absolute orientation sensor
- **Barometric Sensor**: BME280 (optional, for altitude sensing)
- **ESCs**: BLHeli_S compatible electronic speed controllers
- **Gamepad**: Any controller supported by gilrs library
- **Camera**: USB camera with serial output for object detection (optional)

### Communication Interfaces
- I2C bus (`/dev/i2c-1`) for sensors and PWM controller
- SPI bus (`/dev/spidev0.0`) for BME280 (version 1)
- Serial port (`/dev/ttyACM0`) for object detection
- UDP network for base station communication

## Project Structure

```
SanoBlimpSoftware/
├── src/
│   ├── main.rs                    # Main event loop and initialization
│   ├── lib/
│   │   ├── blimp.rs              # Blimp control implementations (Sano, Flappy)
│   │   ├── autonomous.rs         # PID control and autonomous navigation
│   │   ├── object_detection.rs   # Computer vision integration
│   │   └── base_station/
│   │       └── communication.rs  # UDP protocol definitions
│   └── driver/
│       ├── bme280.rs             # BME280 driver wrapper
│       └── mod.rs
├── example/                       # Example programs
│   ├── bno055.rs                 # IMU testing
│   ├── servo.rs                  # Servo testing
│   ├── gatt_client.rs            # Bluetooth examples
│   └── ...
├── aux/                          # Auxiliary tools
│   ├── mobile_recv.rs            # Mobile receiver utility
│   └── search_sim.rs             # Search simulation
├── config.toml                   # System configuration
├── Cargo.toml                    # Rust dependencies
└── docs/                         # Documentation
```

## Installation

### Prerequisites

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM64 cross-compilation target
rustup target add aarch64-unknown-linux-gnu

# Install cross-compilation tool (optional)
cargo install cross
```

### Dependencies

The project requires several system libraries on the Raspberry Pi:

```bash
sudo apt-get update
sudo apt-get install -y \
    libdbus-1-dev \
    pkg-config \
    libudev-dev \
    build-essential
```

### Building

#### Native Build (on Raspberry Pi)
```bash
cargo build --release
```

#### Cross-compilation (from development machine)
```bash
cross build --target aarch64-unknown-linux-gnu --release
```

### Installation on Raspberry Pi

```bash
# Copy binary to Raspberry Pi
scp target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware pi@<IP_ADDRESS>:~/

# Copy configuration
scp config.toml pi@<IP_ADDRESS>:~/

# SSH into Raspberry Pi and run
ssh pi@<IP_ADDRESS>
chmod +x SanoBlimpSoftware
sudo ./SanoBlimpSoftware
```

## Configuration

Edit `config.toml` to configure your blimp:

```toml
[blimp]
blimp_type = "sano"      # "sano" or "flappy"
blimp_version = 2         # Hardware version

[server]
host = "192.168.1.156"   # Base station IP
port = 54321              # Data port
port_stat = 54320         # Status port

[motor]
m1_mul = 1.0              # Motor 1 multiplier
m2_mul = 1.0              # Motor 2 multiplier
m3_mul = 1.0              # Motor 3 multiplier
m4_mul = 1.0              # Motor 4 multiplier
midpoint_angle = 90.0     # ESC neutral angle

[controller]
kp_x = 1.0                # Proportional gain X-axis
kp_y = 1.0                # Proportional gain Y-axis
kp_z = 1.0                # Proportional gain Z-axis
kd_x = 0.1                # Derivative gain X-axis
kd_y = 0.1                # Derivative gain Y-axis
kd_z = 0.1                # Derivative gain Z-axis
```

## Usage

### Running the Main Program

```bash
sudo ./SanoBlimpSoftware
```

Note: `sudo` is required for GPIO and I2C access.

### Control Modes

#### Manual Control
- **Left Stick Y**: Forward/backward movement
- **Right Stick Y**: Altitude control (up/down)
- **Left Stick X**: Turning (yaw control)
- **Start Button**: Toggle between manual and autonomous mode
- **Right Trigger**: Switch to ball tracking mode
- **Left Trigger**: Switch to goal tracking mode
- **East Button (B/Circle)**: Execute scoring maneuver
- **West Button (X/Square)**: Save camera frame

#### Autonomous Mode
When in autonomous mode, the blimp will:
1. Track detected objects based on the current state (Ball or Goal)
2. Use PID control to maintain position relative to the target
3. Automatically adjust altitude and position
4. Send telemetry to the base station

### Example Programs

Test individual components:

```bash
# Test IMU
cargo run --bin bno

# Test servo control
cargo run --bin servo

# Test BME280 sensor
cargo run --bin bme

# Test Bluetooth GATT
cargo run --bin gatt_client
```

## Architecture

### Main Event Loop

The main event loop (`src/main.rs`) performs the following operations at each iteration:

1. **Sensor Update**: Read IMU orientation and altitude
2. **Controller Input**: Process gamepad events
3. **Object Detection**: Read detection data from serial port
4. **Control Computation**: Calculate PID outputs or process manual inputs
5. **Motor Mixing**: Convert control inputs to motor commands
6. **Actuation**: Send PWM signals to ESCs and servos
7. **Communication**: Send telemetry to base station and receive commands

### Control Flow

```
Sensors → State Estimation → Control Algorithm → Motor Mixing → Actuators
                ↓                                                    ↑
        Base Station ← UDP Telemetry                    Controller Input
```

### Blimp Trait

Both `SanoBlimp` and `Flappy` implement the `Blimp` trait:

```rust
pub trait Blimp {
    fn update(&mut self, state, state_timer, save_image) -> BlimpSensorData;
    fn mix(&mut self) -> Actuations;
    fn update_input(&mut self, input: (f32, f32, f32));
}
```

## Communication Protocol

### Messages from Blimp to Base Station

```json
// Connection
{
  "type": "connect",
  "id": "blimp_hostname"
}

// Sensor update
{
  "type": "sensor_update",
  "battery": 0.85,
  "altitude": 5.2,
  "roll": -1.2,
  "pitch": 0.8,
  "yaw": 45.0,
  "tracking_error_x": -10.5,
  "tracking_error_y": 5.3,
  "state": "Ball"
}

// Video acknowledgment
{
  "type": "ack_video",
  "streaming_to": "192.168.1.100:54321"
}
```

### Messages from Base Station to Blimp

```json
// Update configuration
{
  "type": "UpdateConfig",
  "kp_x": 1.2,
  "kd_x": 0.15,
  "kp_z": 1.0,
  "kd_z": 0.1,
  "kp_yaw": 0.8,
  "kd_yaw": 0.05,
  "m1_multiplier": 1.0,
  "m2_multiplier": 1.0,
  "motor_neutral_angle": 90.0
}

// Request video stream
{
  "type": "request_video",
  "target_port": 54321
}
```

## Development

### Running Tests

```bash
cargo test
```

### Formatting

```bash
cargo fmt
```

### Linting

```bash
cargo clippy
```

## Troubleshooting

### ESC Calibration Issues
If motors don't respond properly, ensure ESCs are calibrated:
1. ESCs receive max throttle signal on power-up
2. After beep, send min throttle
3. System sets neutral position

This is automated in the `init_escs()` function.

### I2C Communication Errors
```bash
# Check I2C devices
i2cdetect -y 1

# Common addresses:
# 0x28 or 0x29: BNO055
# 0x40: PCA9685
# 0x76 or 0x77: BME280
```

### Permission Errors
Ensure user is in required groups:
```bash
sudo usermod -a -G i2c,gpio,spi pi
```

### Serial Port Access
```bash
# Check serial device
ls -l /dev/ttyACM0

# Add user to dialout group
sudo usermod -a -G dialout pi
```

## License

See `docs/src/license.md` for licensing information.

## Contributing

Contributions are welcome! Please see `docs/src/contributing.md` for guidelines.

## Documentation

For detailed documentation, see the `docs/` directory or build the mdBook:

```bash
cd docs
mdbook serve
```

Then open http://localhost:3000 in your browser.

## Related Projects

- Base Station software (separate repository)
- Object detection firmware (separate repository)

## Acknowledgments

This project uses the following libraries:
- `rppal` - Raspberry Pi GPIO/I2C/SPI
- `gilrs` - Gamepad input
- `bno055` - IMU driver
- `pwm-pca9685` - PWM controller
- `image` and `imageproc` - Computer vision
- `tokio` - Async runtime
- `serde` - Serialization
