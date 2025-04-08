# Chapter 5: Sensor Input/State (Conceptual)

Welcome back! In [Chapter 4: Object Detection (`Detection`)](04_object_detection___detection__.md), we gave our blimp "eyes" to see the world around it, allowing it to find specific objects like markers.

But just seeing isn't enough for stable flight. Imagine trying to walk with your eyes open, but completely losing your sense of balance or whether you're going uphill or downhill. You'd stumble and fall! Our blimp needs a sense of its own **state** – its orientation (which way is up?) and its altitude (how high is it?).

This chapter explores the concept of **Sensor Input/State**. Unlike previous chapters focusing on specific code structures like `Flappy` or `PCAActuator`, this is more about *how the blimp perceives itself* using various sensors. We'll look at examples using common sensors: an IMU for balance and a pressure sensor for height. This self-awareness is absolutely vital for the blimp to fly smoothly and navigate autonomously using the logic in [Chapter 6: Autonomous Control (`Autonomous`)](06_autonomous_control___autonomous__.md).

**Our Use Case:** Imagine the blimp needs to fly autonomously towards the marker it detected in Chapter 4, but it must stay perfectly level and maintain a constant height of 2 meters above the ground while doing so. It needs sensors to constantly check: "Am I tilted?" and "Am I at the right height?".

## Key Concepts

Let's break down how the blimp achieves this self-awareness.

### 1. Self-Awareness: The Blimp's Inner Senses

Humans have an inner ear for balance and a sense of our body's position. Our blimp needs similar "inner senses" provided by electronic sensors. This information tells the control system (especially the autonomous part) about the blimp's current condition, allowing it to make corrections.

### 2. Orientation (The Inner Ear): The IMU (BNO055)

*   **What is it?** An **IMU (Inertial Measurement Unit)** is a small chip packed with sensors (like accelerometers and gyroscopes) that can figure out its orientation in 3D space. Think of it as the blimp's electronic inner ear, responsible for its sense of balance. The BNO055 is a popular IMU chip that even does some clever calculations onboard (sensor fusion) to give a stable orientation reading.
*   **What does it measure?** It tells us how the blimp is tilted:
    *   **Roll:** Tilting side to side (like an airplane banking).
    *   **Pitch:** Tilting nose up or nose down.
    *   **Yaw:** Turning left or right.
*   **How is the data represented?** Orientation can be represented as:
    *   **Euler Angles:** Three angles (roll, pitch, yaw), often in degrees. Simple to understand, but can have mathematical quirks (gimbal lock).
    *   **Quaternions:** A more complex, 4-number representation. Mathematically more robust and often preferred for calculations, avoiding gimbal lock.
*   **Why is it important?** The autonomous controller needs to know if the blimp is level or tilted to make corrections. If the blimp is supposed to fly straight but the IMU says it's rolling to the right, the controller knows it needs to apply counter-thrust or adjust control surfaces.

### 3. Altitude (The Sense of Height): The Pressure Sensor (BME280)

*   **What is it?** A **barometric pressure sensor** measures air pressure. Since air pressure generally decreases as you go higher, we can use this measurement to estimate altitude. The BME280 is a common sensor that measures pressure, temperature, and humidity.
*   **How does it work?** It detects the tiny changes in air pressure. By comparing the current pressure to a known reference pressure (like the pressure at ground level), we can calculate the approximate height difference.
*   **Why is it important?** For tasks like maintaining a specific flying height, hovering above a target, or landing gently, the blimp needs to know its altitude. The pressure sensor provides this crucial piece of information.

### 4. Data for Control

The raw data from these sensors (like quaternion values from the IMU or pressure readings in Pascals from the BME280) isn't usually used *directly* to set motor speeds. It needs to be processed and interpreted, often by the [Autonomous Control (`Autonomous`)](06_autonomous_control___autonomous__.md) logic.

*   The **IMU data** helps calculate orientation errors (e.g., "I'm tilted 5 degrees left, but I should be level").
*   The **pressure data** is converted into an altitude reading, which is then compared to the desired altitude to find the height error (e.g., "I'm at 1.8 meters, but I should be at 2.0 meters").

These calculated *errors* are what the autonomous controller uses to decide how to adjust the motors and servos via the [Blimp Control (`Blimp` Trait / `Flappy` Implementation)](01_blimp_control___blimp__trait____flappy__implementation_.md) and [Hardware Actuation (`PCAActuator`)](02_hardware_actuation___pcaactuator__.md).

## How We Use Sensor Data (Conceptually)

Integrating sensor data usually involves these steps within the main program loop:

1.  **Initialize Sensors:** When the software starts, initialize the connection to each sensor (often over an I2C bus). This might involve setting the sensor's mode or performing initial calibration.
2.  **Read Sensors Periodically:** Inside the main loop, regularly ask each sensor for its latest reading.
3.  **Process Data:** Convert raw sensor readings into useful information (e.g., pressure to altitude, quaternion to understandable angles if needed).
4.  **Feed to Controller:** Pass this state information (orientation, altitude) to the `Autonomous` controller.
5.  **Calculate Corrections:** The `Autonomous` controller uses this state information, along with target information (like desired heading or altitude), to calculate the necessary adjustments.
6.  **Actuate:** The calculated adjustments are sent to the motors/servos.

Here's how it might look conceptually in the main loop:

```rust
// --- Conceptual Main Loop ---
loop {
    // --- Read Sensors ---
    // Get orientation data (e.g., as a Quaternion) from the IMU
    let current_orientation = read_imu_orientation(); // Hypothetical function

    // Get pressure data from the pressure sensor
    let current_pressure = read_pressure_sensor(); // Hypothetical function
    // Convert pressure to altitude relative to ground
    let current_altitude = pressure_to_altitude(current_pressure, ground_pressure);

    // --- Object Detection (from Chapter 4) ---
    let target_coords = detection.detect(target_classes);

    // --- Control Logic ---
    if blimp.is_manual() {
        // Manual control based on joystick
        blimp.update(); // Read joystick
        blimp.manual(); // Mix and Actuate
    } else {
        // Autonomous Control
        // Combine sensor state and detection results
        let autonomous_input = auto.calculate_commands(
            current_orientation,
            current_altitude,
            target_coords,
            desired_altitude, // e.g., 2.0 meters
            // ... other goal parameters ...
        ); // Hypothetical function in Autonomous controller

        // Send commands to blimp
        blimp.update_input(autonomous_input);
        let actuations = blimp.mix();
        blimp.actuator.actuate(actuations);
    }

    // --- Delay ---
    // sleep(Duration::from_millis(20)); // Loop delay
}
```
This pseudo-code shows the flow: read sensors, get detection results, and then, in autonomous mode, feed *all* this information into the `Autonomous` controller (`auto.calculate_commands(...)`) to figure out the next move.

## Under the Hood: Talking to Sensors

How does the code *actually* get data from a BNO055 or BME280 chip? These sensors typically communicate using a protocol called **I2C** (Inter-Integrated Circuit). This is a simple two-wire bus that allows the main computer (like the Raspberry Pi) to talk to multiple peripheral devices (like our sensors).

Luckily, we don't usually need to handle the low-level I2C signal details ourselves. We use **libraries** (often called "drivers" or "HAL crates" in Rust) that provide a much friendlier interface.

Let's look at simplified examples based on the code provided earlier, showing how these libraries make it easier.

### Example: Interfacing with BME280 (Pressure/Altitude)

The `bme280-rs` library helps us talk to the BME280 sensor.

1.  **Initialization:** We need to tell the library which I2C bus the sensor is on and its address.

    ```rust
    // Simplified from example/bme.rs
    use bme280::i2c::BME280;
    use linux_embedded_hal::{Delay, I2cdev}; // For I2C and delay

    // Tell it which I2C bus (e.g., /dev/i2c-1 on Raspberry Pi)
    let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
    // Create a delay provider (some sensors need short pauses)
    let mut delay = Delay;

    // Create the sensor object using its default I2C address
    let mut bme280 = BME280::new_primary(i2c_bus);

    // Initialize the sensor hardware
    bme280.init(&mut delay).unwrap();
    ```
    This code sets up the communication channel and performs any necessary startup configuration on the sensor chip itself.

2.  **Reading Data:** Once initialized, reading the sensor is straightforward.

    ```rust
    // Simplified from example/bme.rs
    // Inside a loop...
    // Ask the sensor for the latest measurements
    let measurements = bme280.measure(&mut delay).unwrap();

    // Extract the pressure reading
    let pressure_pascals = measurements.pressure;

    // (You would then convert pressure_pascals to altitude)
    println!("Pressure = {} pascals", pressure_pascals);
    ```
    Calling `bme280.measure()` triggers the sensor to take readings and return them in a convenient `measurements` struct.

### Example: Interfacing with BNO055 (IMU/Orientation)

Similarly, the `bno055` crate simplifies interaction with the BNO055 IMU.

1.  **Initialization:** Set up I2C communication and configure the IMU's operating mode. The `NDOF` mode is often best, as it uses all onboard sensors for fused, stable orientation data.

    ```rust
    // Simplified from example/bno055.rs
    use bno055::{BNO055OperationMode, Bno055};
    use linux_embedded_hal::{Delay, I2cdev};

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut delay = Delay {};

    // Create the IMU object
    let mut imu = Bno055::new(dev);
    imu.init(&mut delay).expect("IMU init failed");

    // Set the operating mode (NDOF is fusion mode)
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("IMU mode set failed");

    // (Optional but recommended: Wait for calibration)
    println!("Calibrating IMU...");
    // while !imu.is_fully_calibrated().unwrap() { /* wait */ }
    println!("IMU calibrated!");
    ```
    This gets the IMU ready and sets it to the desired mode. Calibration is often needed for best accuracy.

2.  **Reading Data:** Reading orientation is then a simple function call. Quaternions are generally recommended for calculations.

    ```rust
    // Simplified from example/bno055.rs
    use mint::Quaternion; // A standard way to represent quaternions

    // Inside a loop...
    let mut orientation_quat: Quaternion<f32>;

    match imu.quaternion() {
        Ok(quat_reading) => {
            orientation_quat = quat_reading;
            println!("IMU Quaternion: {:?}", orientation_quat);
            // Use orientation_quat in the Autonomous controller
        }
        Err(e) => {
            eprintln!("Error reading IMU: {:?}", e);
            // Handle error (e.g., use last known value)
        }
    }
    ```
    Calling `imu.quaternion()` fetches the latest orientation reading directly from the sensor's fusion algorithm.

These examples show that while the underlying communication (I2C) and sensor details can be complex, well-designed libraries provide a much simpler way to get the crucial state information we need.

```mermaid
flowchart TD
    A[Main Loop] --> B{Read IMU?};
    B -- Yes --> C[IMU Library (e.g., `bno055`)];
    C --> D[I2C Communication];
    D --> E[BNO055 Chip];
    E --> D;
    D --> C;
    C --> F[Get Orientation (Quaternion)];
    F --> A;

    A --> G{Read Pressure Sensor?};
    G -- Yes --> H[Pressure Sensor Library (e.g., `bme280-rs`)];
    H --> I[I2C Communication];
    I --> J[BME280 Chip];
    J --> I;
    I --> H;
    H --> K[Get Pressure (Pascals)];
    K --> L[Convert Pressure to Altitude];
    L --> A;

    A --> M{Use Data in Autonomous?};
    M -- Yes --> N[Autonomous Controller];
    F --> N;
    L --> N;
    N --> O[Calculate Corrections];
    O --> P[Blimp Control Logic];
    P --> Q[Hardware Actuation];
    Q --> R[Motors/Servos Move];
    R --> A;
```
This diagram shows the conceptual flow: the main loop uses libraries to read sensors via I2C, processes the data (like converting pressure to altitude), and feeds the resulting state (orientation, altitude) to the autonomous controller, which then computes corrections to control the blimp.

## Conclusion

We've explored the crucial concept of **Sensor Input/State** – how the blimp perceives its own condition in the world.

*   The blimp needs **self-awareness** (orientation and altitude) for stable and autonomous flight.
*   An **IMU** (like the BNO055) acts as the "inner ear," providing **orientation** data (roll, pitch, yaw), often as robust quaternions.
*   A **pressure sensor** (like the BME280) acts as a "sense of height," allowing **altitude** estimation.
*   This sensor data is read periodically using hardware interface libraries (often abstracting **I2C** communication).
*   The processed state information (current orientation, current altitude) is essential input for the [Autonomous Control (`Autonomous`)](06_autonomous_control___autonomous__.md) system.

Now that we understand how the blimp knows *where it is* and *how it's oriented*, how does it use this information, along with data from its "eyes" ([Chapter 4: Object Detection (`Detection`)](04_object_detection___detection__.md)), to make intelligent decisions about where to go next?

In the next chapter, we'll finally dive into the brain of the autonomous flight system! Let's move on to [Chapter 6: Autonomous Control (`Autonomous`)](06_autonomous_control___autonomous__.md).


