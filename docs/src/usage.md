# Usage Guide

This guide covers day-to-day operation of SanoBlimpSoftware, including manual and autonomous flight modes, configuration adjustments, and common scenarios.

## Starting the System

### Basic Startup

```bash
# SSH into Raspberry Pi
ssh pi@raspberrypi.local

# Navigate to project directory
cd ~

# Run the software
sudo ./SanoBlimpSoftware
```

### Startup Sequence

When started, the system performs these operations:

1. **Load Configuration** - Reads `config.toml`
2. **Initialize I2C** - Connects to sensors and PWM controller
3. **Initialize IMU** - Sets BNO055 to NDOF mode
4. **Initialize ESCs** - Calibrates electronic speed controllers
   - Sends max throttle (180°)
   - Waits for ESC beep
   - Sends min throttle (0°)
   - Sets to neutral position
5. **Connect to Base Station** - Establishes UDP communication
6. **Initialize Controllers** - Detects connected gamepads
7. **Enter Main Loop** - Ready for operation

**Expected Output**:
```
Initializing ESCs...
Sending max throttle (arming sequence)
Sending min throttle (arming)
Setting ESCs to neutral
ESCs initialized!
connected to the base station
```

## Control Modes

SanoBlimpSoftware operates in two primary modes:

### Manual Mode

**Default mode on startup**. Pilot has direct control via gamepad.

**Activating Manual Mode**:
- Press **Start button** to toggle from autonomous to manual mode
- LED/indicator on base station shows manual mode active

**Controls**:
- **Left Stick Y-axis**: Forward (up) / Backward (down)
- **Left Stick X-axis**: Turn left / Turn right (yaw control)
- **Right Stick Y-axis**: Altitude up (up) / Altitude down (down)

**Behavior**:
- Direct joystick-to-motor mapping
- No computer vision or automatic corrections
- Pilot maintains full control
- PID controller inactive

### Autonomous Mode

**Computer vision-based** navigation. Blimp tracks detected objects.

**Activating Autonomous Mode**:
- Press **Start button** to toggle from manual to autonomous mode
- Base station indicator shows autonomous mode active

**How It Works**:
1. Camera system detects objects via serial port
2. System identifies largest target matching current state
3. PID controller calculates position error
4. Control outputs adjust motors to center on target
5. Telemetry sent to base station

**Object States**:
- **Ball State**: Tracks balls (object classes 2, 3)
- **Goal State**: Tracks goals (object classes 5, 7 for orange goals)

**Switching States**:
- **Right Trigger**: Switch to Ball tracking mode
- **Left Trigger**: Switch to Goal tracking mode

## Controller Reference

### Complete Button Mapping

| Input | Function | Notes |
|-------|----------|-------|
| **Left Stick Y** | Forward/Backward | Negative = forward, Positive = backward |
| **Left Stick X** | Yaw (Turn) | Negative = left, Positive = right |
| **Right Stick Y** | Altitude | Negative = up, Positive = down |
| **Start Button** | Toggle Manual/Autonomous | Changes control mode |
| **Right Trigger** | Set Ball State | For autonomous mode |
| **Left Trigger** | Set Goal State | For autonomous mode |
| **East Button** (B/○) | Score Maneuver | Executes scoring sequence |
| **West Button** (X/□) | Save Frame | Captures current camera image |

### Input Scaling

Raw joystick values (-1.0 to 1.0) are scaled:

**Manual Mode** (Sano configuration):
```rust
// Forward/backward thrust
m1 = neutral_angle + (x * 10.0)
m2 = neutral_angle + (x * 10.0)

// Altitude control via servos
s1 = neutral_angle - (z * 90.0)  // Clamp 0-180°
s2 = neutral_angle + (z * 90.0)  // Clamp 0-180°

// Yaw control (differential thrust)
m1 += y * 10.0
m2 -= y * 10.0

// Final clamping
m1 = clamp(m1, neutral_angle - 20.0, neutral_angle + 20.0)
m2 = clamp(m2, neutral_angle - 20.0, neutral_angle + 20.0)
```

Adjust these values by modifying motor multipliers in `config.toml`.

## Autonomous Operation

### Object Detection

The system receives detection data via serial port (`/dev/ttyACM0`) from an external camera system.

**Detection Message Format** (JSON):
```json
{
  "type": 1,
  "data": {
    "boxes": [
      [x, y, width, height, confidence, class],
      ...
    ],
    "image": "base64_encoded_image"
  }
}
```

**Detection Process**:
1. Parse JSON from serial input
2. Filter boxes by target class (ball or goal)
3. Select largest matching bounding box
4. Extract center coordinates
5. Pass to PID controller

### PID Control

When a target is detected, the autonomous controller:

**Position Control**:
```rust
// Compute errors
x_err = 0.0 - x              // Target center x
y_err = 160.0 - y            // Target center y (normalized)
z_err = 122.0 - z            // Target altitude

// PID calculation
pid_x = (kp_x * x_err) + (kd_x * dx/dt)
pid_y = (kp_y * y_err) + (kd_y * dy/dt)
pid_z = (kp_z * z_err) + (kd_z * dz/dt)

// Clamp outputs to [-1.0, 1.0]
```

**Altitude Hold**:
```rust
altitude_error = current_altitude - desired_altitude
mapped = map_value(altitude_error, -10.0, 10.0, -1.0, 1.0)
output = clamp(mapped, -1.0, 1.0)
```

See [Autonomous Control](06_autonomous_control___autonomous__.md) for tuning details.

### Search Behavior

When no target is detected (empty detection result):

- **Current Implementation**: Maintains last known position
- **Commented Code**: Includes search pattern logic (rotation and altitude cycling)

To enable search behavior, uncomment search logic in `src/main.rs` around line 190.

## Special Maneuvers

### Scoring Maneuver

**Activation**: Press East Button (B/Circle)

**Behavior**:
```rust
// Move forward and up for 3 seconds
m1 = neutral_angle + 7.0
m2 = neutral_angle + 7.0
m3 = neutral_angle + 25.0
m4 = neutral_angle + 25.0
// Duration: 3 seconds
// Then return to neutral
```

Purpose: Execute goal-scoring action in competition scenarios.

### Emergency Stop

**Method 1**: Switch to manual mode (Start button) and center all sticks

**Method 2**: Power off ESCs (if accessible)

**Method 3**: Kill power to Raspberry Pi (last resort)

**Recommended**: Configure emergency stop button on gamepad if possible

## Telemetry

### Data Sent to Base Station

Every loop iteration (~50-100 Hz), the system sends:

```json
{
  "type": "sensor_update",
  "battery": 0.85,              // Controller battery level (0.0-1.0)
  "altitude": 5.2,              // Meters
  "roll": -1.2,                 // Degrees
  "pitch": 0.8,                 // Degrees
  "yaw": 45.0,                  // Degrees (0-360)
  "tracking_error_x": -10.5,    // Pixels
  "tracking_error_y": 5.3,      // Pixels
  "state": "Ball"               // "Ball" or "Goal"
}
```

### Receiving Commands

Base station can send configuration updates:

```json
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
```

System applies changes immediately without restart.

## Configuration Tuning

### Runtime Configuration Changes

The base station can update configuration in real-time:

1. Base station sends `UpdateConfig` message
2. Blimp receives via UDP on port 54320
3. `autonomous.update_gains()` applies new PID gains
4. Changes take effect immediately

### Persistent Configuration Changes

Edit `config.toml` and restart the software:

```bash
# Stop the software (Ctrl+C if running in foreground)
# Or stop service:
sudo systemctl stop sanoblimp.service

# Edit configuration
nano ~/config.toml

# Restart
sudo ./SanoBlimpSoftware
# Or:
sudo systemctl start sanoblimp.service
```

See [Configuration](03_configuration___config__.md) for parameter details.

## Common Flight Scenarios

### Scenario 1: Manual Test Flight

**Purpose**: Verify hardware and basic controls

**Steps**:
1. Remove propellers (safety first!)
2. Start software: `sudo ./SanoBlimpSoftware`
3. Connect gamepad
4. Ensure manual mode (press Start if needed)
5. Test each control axis:
   - Left stick Y: Check M1/M2 response
   - Left stick X: Check differential thrust
   - Right stick Y: Check servo/altitude response
6. If responses correct, attach propellers
7. Perform hover test in safe area

### Scenario 2: Ball Tracking

**Purpose**: Autonomous ball detection and following

**Steps**:
1. Start software with camera system active
2. Press **Start** to enter autonomous mode
3. Press **Right Trigger** to select Ball state
4. Place ball in camera view
5. Blimp should move to center ball in frame
6. Monitor telemetry on base station

**Expected Behavior**:
- Blimp moves forward if ball is ahead
- Blimp turns if ball is left/right of center
- Blimp adjusts altitude if ball is above/below center
- Close approach when ball appears large (high w/h values)

### Scenario 3: Goal Scoring

**Purpose**: Autonomous goal approach and scoring

**Steps**:
1. Start in autonomous mode
2. Press **Left Trigger** to select Goal state
3. Detect goal in camera view
4. Approach goal (automatic)
5. When close (w > 200 or h > 200 pixels), press **East** for scoring maneuver
6. Blimp executes forward-up motion for 3 seconds
7. Returns to normal operation

### Scenario 4: Tuning PID Gains

**Purpose**: Optimize autonomous control response

**Steps**:
1. Start with default gains (kp=1.0, kd=0.1)
2. Enable autonomous mode with target visible
3. Observe behavior:
   - **Slow response**: Increase kp
   - **Oscillation**: Increase kd or decrease kp
   - **Overshoot**: Increase kd
4. Use base station to send new gains
5. Test and iterate
6. Save working gains to `config.toml`

See [Autonomous Control](06_autonomous_control___autonomous__.md) for detailed tuning.

## Monitoring and Debugging

### Console Output

When running in foreground, monitor console:

```bash
sudo ./SanoBlimpSoftware

# Look for:
# - "connected to the base station" (confirms network)
# - IMU data (if enabled in code)
# - Detection results (if debug prints enabled)
```

### System Logs

When running as service:

```bash
# View live logs
sudo journalctl -u sanoblimp.service -f

# View recent logs
sudo journalctl -u sanoblimp.service -n 100

# Filter errors
sudo journalctl -u sanoblimp.service -p err
```

### Base Station Monitoring

Use base station GUI/interface to view:
- Real-time telemetry graphs
- Video stream with detection overlays
- Battery level
- Current state (Ball/Goal, Manual/Autonomous)
- Connection status

### Debug Mode

To enable additional debug output, modify source code:

```rust
// In src/main.rs, uncomment debug prints:
println!("Detection: {:?}", det);
println!("PID output: {:?}", auto_input);
println!("Current altitude: {}", altitude);
```

Rebuild and redeploy:
```bash
cross build --target aarch64-unknown-linux-gnu --release
scp target/aarch64-unknown-linux-gnu/release/SanoBlimpSoftware pi@raspberrypi.local:~/
```

## Safety Guidelines

### Pre-Flight Checklist

- [ ] Configuration file correct
- [ ] All sensors responding (check `i2cdetect`)
- [ ] Gamepad connected and responsive
- [ ] Base station running and connected
- [ ] Propellers secure and balanced
- [ ] Flight area clear of obstacles
- [ ] Emergency stop plan in place
- [ ] Battery charged (both blimp and controller)

### During Flight

- Maintain line of sight with blimp
- Keep emergency controller accessible
- Monitor battery levels
- Watch for unusual behavior (excessive vibration, drift)
- Land if battery drops below 20%
- Avoid obstacles and people

### After Flight

- Land safely before battery depletes
- Disarm ESCs (disconnect power)
- Review telemetry logs
- Note any issues for maintenance
- Charge batteries

## Troubleshooting

### Issue: Blimp doesn't respond to controller

**Check**:
1. Is gamepad connected? Check `lsusb`
2. Is software in manual mode? Press Start
3. Are sticks centered? Dead zone may be active
4. Check console for controller events

### Issue: Autonomous mode doesn't track object

**Check**:
1. Is camera system running and sending data?
2. Check serial port: `ls -l /dev/ttyACM0`
3. Is object class in target list?
4. Verify detection data with serial monitor
5. Check autonomous state matches object (Ball vs Goal)

### Issue: Blimp drifts in one direction

**Solution**:
- Adjust motor multipliers in `config.toml`
- If drifts left, reduce `m1_mul` or increase `m2_mul`
- Test incrementally (0.05 adjustments)
- May require hardware balancing

### Issue: Oscillation in autonomous mode

**Solution**:
- Increase derivative gain (kd)
- Reduce proportional gain (kp)
- Start with kd = 0.2, kp = 0.8
- Tune incrementally

### Issue: Connection lost to base station

**Check**:
1. Network connectivity: `ping <base_ip>`
2. Base station still running?
3. Firewall blocking ports?
4. Check IP address in config.toml
5. Review base station logs

### Issue: Motors won't arm

**Check**:
1. ESC power connected?
2. Check neutral_angle in config (try 90.0)
3. Listen for ESC beeps during startup
4. Try manual ESC calibration with servo tester
5. Verify PWM signals with oscilloscope

## Advanced Usage

### Custom Control Mixing

Modify `src/lib/blimp.rs`, function `mix()` to change control behavior:

```rust
fn mix(&mut self) -> Actuations {
    let (x, y, z) = self.input;
    
    // Custom mixing logic here
    // Example: Non-linear response
    let x_scaled = x.powi(3) * 10.0;  // Cubic response
    
    // Your motor calculations
    // ...
}
```

### Custom Detection Classes

Edit target classes in `src/main.rs`:

```rust
let balls = vec![2, 3, 4];           // Add class 4
let orange_goals = vec![5, 7];
let yellow_goals = vec![6, 8];       // Alternative goals
```

### Recording Flight Data

Add logging to `src/main.rs`:

```rust
use std::fs::OpenOptions;
use std::io::Write;

// In main loop:
let mut log_file = OpenOptions::new()
    .create(true)
    .append(true)
    .open("flight_log.csv")
    .unwrap();

writeln!(log_file, "{},{},{},{}", 
    sensordat.altitude, 
    sensordat.yaw, 
    auto_input.0, 
    auto_input.1
).unwrap();
```

## Performance Optimization

### Reducing Latency

- Use wired gamepad instead of Bluetooth
- Optimize network (reduce WiFi interference)
- Increase serial baud rate if camera supports
- Profile code to identify bottlenecks

### Battery Life

- Use efficient flight patterns
- Reduce hover time
- Optimize motor efficiency (prop selection)
- Monitor current draw
- Use low-battery warnings

## Next Steps

- **Tune Your System**: See [Configuration](03_configuration___config__.md)
- **Understand Control**: Read [Autonomous Control](06_autonomous_control___autonomous__.md)
- **Hardware Deep Dive**: Review [Hardware Actuation](02_hardware_actuation___pcaactuator__.md)
- **Integrate Detection**: Set up [Object Detection](04_object_detection___detection__.md)

## Additional Resources

- [Getting Started](getting_started.md)
- [Installation Guide](installation.md)
- [Hardware Guide](hardware.md)
- Base Station Documentation (separate repository)
