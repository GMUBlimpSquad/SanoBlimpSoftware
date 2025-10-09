# Installation Guide

This guide provides step-by-step instructions for installing and configuring SanoBlimpSoftware on your Raspberry Pi.

## Prerequisites

Before beginning installation, ensure you have:

### Hardware
- Raspberry Pi 4 (or compatible ARM64 SBC)
- MicroSD card (16GB minimum, 32GB recommended)
- Network connection (WiFi or Ethernet)
- All required sensors and controllers (see [Hardware Guide](hardware.md))

### Development Machine
- Computer running Linux, macOS, or Windows with WSL2
- Rust toolchain installed
- Git installed
- SSH client
- (Optional) Docker for cross-compilation

## Installation Steps

### 1. Prepare the Raspberry Pi

#### 1.1 Install Raspberry Pi OS

Download and flash Raspberry Pi OS (64-bit):

```bash
# Using Raspberry Pi Imager (recommended)
# 1. Download from https://www.raspberrypi.com/software/
# 2. Select "Raspberry Pi OS (64-bit)"
# 3. Configure hostname, SSH, WiFi in advanced options
# 4. Flash to SD card

# Or use command line with dd (Linux/macOS):
sudo dd if=raspberry-pi-os-64bit.img of=/dev/sdX bs=4M status=progress
sync
```

**Important**: Enable SSH during imaging or create an empty file named `ssh` in the boot partition.

#### 1.2 Initial Raspberry Pi Setup

Boot the Raspberry Pi and SSH into it:

```bash
ssh pi@raspberrypi.local
# Default password: raspberry (change immediately)
```

Update the system:

```bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
```

#### 1.3 Install System Dependencies

```bash
sudo apt-get install -y \
    build-essential \
    git \
    cmake \
    pkg-config \
    libdbus-1-dev \
    libudev-dev \
    i2c-tools \
    python3-smbus \
    vim \
    curl \
    wget
```

#### 1.4 Enable Required Interfaces

```bash
sudo raspi-config
```

Navigate and enable:
- **Interface Options → I2C → Yes**
- **Interface Options → SPI → Yes** (if using BME280 on SPI)
- **Interface Options → Serial Port**:
  - "Would you like a login shell over serial?" → **No**
  - "Would you like serial port hardware enabled?" → **Yes**

Exit and reboot:
```bash
sudo reboot
```

#### 1.5 Configure User Permissions

Add the user to required groups:

```bash
sudo usermod -a -G i2c,gpio,spi,dialout,video pi
```

Log out and back in for changes to take effect:
```bash
exit
ssh pi@raspberrypi.local
```

Verify group membership:
```bash
groups
# Should include: i2c gpio spi dialout video
```

#### 1.6 Verify Hardware Connections

Check I2C devices are detected:

```bash
i2cdetect -y 1
```

Expected output (example):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- 76 --
```

- `29`: BNO055 IMU
- `40`: PCA9685 PWM controller
- `76` or `77`: BME280 barometric sensor (optional)

### 2. Set Up Development Environment

#### 2.1 Install Rust on Raspberry Pi (Option A: Native Build)

If building directly on the Raspberry Pi:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

Verify installation:
```bash
rustc --version
cargo --version
```

#### 2.2 Install Rust on Development Machine (Option B: Cross-Compilation)

If cross-compiling from your development computer:

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Add ARM64 target
rustup target add aarch64-unknown-linux-gnu

# Install cross-compilation tool
cargo install cross

# Verify Docker is installed (required by cross)
docker --version
```

### 3. Clone and Build the Project

#### 3.1 Clone the Repository

On your development machine:

```bash
git clone git@github.com:GMUBlimpSquad/SanoBlimpSoftware.git
cd SanoBlimpSoftware
```

#### 3.2 Build the Project

**Option A: Native Build** (on Raspberry Pi):

```bash
# SSH into Raspberry Pi
ssh pi@raspberrypi.local

# Clone repository
git clone git@github.com:GMUBlimpSquad/SanoBlimpSoftware.git
cd SanoBlimpSoftware

# Build release version
cargo build --release

# Binary location: target/release/SanoBlimpSoftware
```

**Option B: Cross-Compilation** (from development machine):

```bash
# In SanoBlimpSoftware directory
cross build --target aarch64-unknown-linux-gnu --release

# Binary location: target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware
```

Build time:
- Native build on Pi 4: ~30-45 minutes (first build)
- Cross-compilation: ~10-20 minutes (first build)
- Subsequent builds: Much faster (incremental compilation)

#### 3.3 Build Example Programs (Optional)

```bash
# Build all examples
cargo build --release --bins

# Or build specific examples
cargo build --release --bin bno     # IMU test
cargo build --release --bin servo   # Servo test
cargo build --release --bin bme     # BME280 test
```

### 4. Configuration

#### 4.1 Create Configuration File

Edit `config.toml` in the project root:

```toml
[blimp]
blimp_type = "sano"      # Type: "sano" or "flappy"
blimp_version = 2         # Hardware version: 1 or 2

[server]
host = "192.168.1.156"   # Base station IP address
port = 54321              # Data/video port
port_stat = 54320         # Control/status port

[motor]
m1_mul = 1.0              # Motor 1 speed multiplier
m2_mul = 1.0              # Motor 2 speed multiplier
m3_mul = 1.0              # Motor 3 speed multiplier
m4_mul = 1.0              # Motor 4 speed multiplier
midpoint_angle = 90.0     # ESC neutral angle in degrees

[controller]
kp_x = 1.0                # Proportional gain - forward/back
kp_y = 1.0                # Proportional gain - yaw
kp_z = 1.0                # Proportional gain - altitude
kd_x = 0.1                # Derivative gain - forward/back
kd_y = 0.1                # Derivative gain - yaw
kd_z = 0.1                # Derivative gain - altitude
```

#### 4.2 Configuration Parameters Explained

**[blimp] section**:
- `blimp_type`: Physical blimp configuration
  - `"sano"`: Quad-motor configuration
  - `"flappy"`: Wing-flapping configuration
- `blimp_version`: Hardware version affects sensor interfaces
  - `1`: BME280 on SPI
  - `2`: BME280 on I2C (or not used)

**[server] section**:
- `host`: IP address of base station computer
- `port`: UDP port for data transmission (video, telemetry)
- `port_stat`: UDP port for control messages

**[motor] section**:
- `m1_mul`, `m2_mul`, etc.: Multipliers to balance motor thrust
  - Values typically between 0.5 and 1.5
  - Adjust if blimp drifts in one direction
- `midpoint_angle`: ESC stop/neutral position
  - Typically 90° (1500 µs pulse width)
  - Some ESCs may require adjustment

**[controller] section**:
- `kp_x`, `kp_y`, `kp_z`: Proportional gains for PID controller
  - Higher values = more aggressive response
  - Start with 1.0 and tune
- `kd_x`, `kd_y`, `kd_z`: Derivative gains for PID controller
  - Dampens oscillations
  - Typically 0.05 to 0.2

See [Configuration](03_configuration___config__.md) for detailed tuning guidance.

### 5. Deploy to Raspberry Pi

#### 5.1 Copy Files

From your development machine:

```bash
# Set Raspberry Pi IP address
RPI_IP="raspberrypi.local"  # or use IP like "192.168.1.100"

# Copy binary (use correct path based on build method)
# For cross-compilation:
scp target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware pi@$RPI_IP:~/

# Copy configuration
scp config.toml pi@$RPI_IP:~/

# Copy example binaries (optional)
scp target/aarch64-unknown-linux-gnu/release/bno pi@$RPI_IP:~/
scp target/aarch64-unknown-linux-gnu/release/servo pi@$RPI_IP:~/
```

#### 5.2 Set Permissions

SSH into Raspberry Pi:

```bash
ssh pi@$RPI_IP
cd ~
chmod +x SanoBlimpSoftware
chmod +x bno servo  # If copied
```

#### 5.3 Create Required Files

Create the rail position tracking file:

```bash
echo '0' > ~/rail.pos
```

This file tracks the position of the optional rail mechanism.

### 6. Verification and Testing

#### 6.1 Test I2C Communication

```bash
# List I2C devices
i2cdetect -y 1

# Test BNO055 (if installed standalone test program)
./bno

# Expected output: Orientation data (Euler angles or quaternions)
```

#### 6.2 Test Motor Controllers (Safely!)

**⚠️ REMOVE PROPELLERS BEFORE TESTING**

```bash
# Test servo output (servos only, no motors)
./servo

# Expected: Servos should move through their range
```

#### 6.3 Test Gamepad

```bash
# Check USB devices
lsusb

# Your controller should appear in the list
# Example: "Microsoft Corp. Xbox360 Controller"

# Test in main program (motors disabled for safety)
sudo ./SanoBlimpSoftware
# Press buttons - check for response in debug output
```

#### 6.4 Test Network Communication

On Raspberry Pi:

```bash
# Verify network connectivity to base station
ping 192.168.1.156  # Replace with your base station IP

# Check UDP connectivity (if base station is running)
# You should see "connected to the base station" message when running main program
```

### 7. Running the Software

#### 7.1 Initial Test Run

**⚠️ Keep propellers removed for first test**

```bash
cd ~
sudo ./SanoBlimpSoftware
```

Expected startup sequence:
```
Initializing ESCs...
Sending max throttle (arming sequence)
Sending min throttle (arming)
Setting ESCs to neutral
ESCs initialized!
connected to the base station
```

If you see errors, see [Troubleshooting](#troubleshooting) below.

#### 7.2 Create Startup Script (Optional)

Create a script for easier launching:

```bash
nano ~/start_blimp.sh
```

Add:
```bash
#!/bin/bash
cd /home/pi
sudo ./SanoBlimpSoftware
```

Make executable:
```bash
chmod +x ~/start_blimp.sh
```

Run with:
```bash
./start_blimp.sh
```

#### 7.3 Auto-Start on Boot (Optional)

Create a systemd service:

```bash
sudo nano /etc/systemd/system/sanoblimp.service
```

Add:
```ini
[Unit]
Description=SanoBlimp Control Software
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/pi
ExecStart=/home/pi/SanoBlimpSoftware
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable sanoblimp.service
sudo systemctl start sanoblimp.service

# Check status
sudo systemctl status sanoblimp.service

# View logs
sudo journalctl -u sanoblimp.service -f
```

Disable auto-start:
```bash
sudo systemctl disable sanoblimp.service
```

### 8. Updates and Maintenance

#### 8.1 Updating the Software

On development machine:

```bash
cd SanoBlimpSoftware
git pull origin main
cross build --target aarch64-unknown-linux-gnu --release
scp target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware pi@$RPI_IP:~/
```

On Raspberry Pi:

```bash
# If service is running
sudo systemctl stop sanoblimp.service

# Replace binary (already done by scp above)

# Restart service
sudo systemctl start sanoblimp.service
```

#### 8.2 Backing Up Configuration

```bash
# From development machine
scp pi@$RPI_IP:~/config.toml ./config_backup_$(date +%Y%m%d).toml
```

#### 8.3 System Updates

Periodically update Raspberry Pi OS:

```bash
sudo apt-get update
sudo apt-get upgrade -y
sudo reboot
```

## Troubleshooting

### Build Errors

**Problem**: `cross` fails with Docker errors
```bash
# Solution: Ensure Docker is running
sudo systemctl start docker
docker ps  # Should work without errors
```

**Problem**: Linker errors during cross-compilation
```bash
# Solution: Clean and rebuild
cargo clean
cross build --target aarch64-unknown-linux-gnu --release
```

**Problem**: Out of memory during compilation on Raspberry Pi
```bash
# Solution: Increase swap space
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=100 to CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### Runtime Errors

**Problem**: "Permission denied" when accessing I2C
```bash
# Solution: Run with sudo or fix permissions
sudo ./SanoBlimpSoftware

# Or add udev rule (then reboot):
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee /etc/udev/rules.d/99-i2c.rules
sudo reboot
```

**Problem**: "No such device" for I2C
```bash
# Solution: Verify I2C is enabled
sudo raspi-config
# Interface Options → I2C → Yes
sudo reboot

# Check kernel module is loaded
lsmod | grep i2c
# Should show i2c_bcm2835 or similar
```

**Problem**: ESCs not initializing
```bash
# Check: Are ESCs powered?
# Check: Are signal wires connected correctly?
# Check: Is midpoint_angle correct in config.toml?
# Try: Manually calibrate ESCs with a servo tester
```

**Problem**: "Failed to connect to base station"
```bash
# Check: Is base station running?
# Check: Is IP address correct in config.toml?
# Check: Firewall blocking UDP ports?
ping <BASE_STATION_IP>
# Test UDP: nc -u <BASE_STATION_IP> 54320
```

### Hardware Issues

**Problem**: I2C devices not detected
```bash
# Check wiring
# Check device addresses match code
# Try slower I2C speed
sudo nano /boot/config.txt
# Add: dtparam=i2c_arm_baudrate=50000
sudo reboot
```

**Problem**: Gamepad not recognized
```bash
# Check USB connection
lsusb
# Try different USB port
# Check permissions
ls -l /dev/input/js*  # Joystick device
```

## Next Steps

After successful installation:

1. **Test Components**: Use example programs to verify each component
2. **Tune Configuration**: See [Configuration](03_configuration___config__.md)
3. **Calibrate Sensors**: Follow [Hardware Guide](hardware.md) calibration procedures
4. **Test Flight**: Start with manual mode, then progress to autonomous
5. **Read Documentation**: Understand the system through remaining chapters

## Additional Resources

- [Getting Started Guide](getting_started.md)
- [Hardware Guide](hardware.md)
- [Configuration Guide](03_configuration___config__.md)
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
- [Rust Cross-Compilation Guide](https://rust-lang.github.io/rustup/cross-compilation.html)

## Support

For issues:
- Check [GitHub Issues](https://github.com/GMUBlimpSquad/SanoBlimpSoftware/issues)
- Review documentation thoroughly
- Verify hardware connections
- Check logs: `sudo journalctl -u sanoblimp.service`
