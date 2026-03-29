// bldc_esc_softpwm_custom_ramps.rs - Controls a BLDC motor via an ESC using software-based PWM
// with specific ramp sequences.
//
// *** SAFETY WARNINGS *** (Same as before - CRITICAL)
// * ALWAYS REMOVE PROPELLERS or disconnect the motor before testing or powering up.
// * BLDC motors can spin VERY fast and draw significant current.
// * Power the ESC and motor from a SEPARATE, ADEQUATE power supply (e.g., LiPo battery).
// * DO NOT power the ESC/motor directly from the Raspberry Pi's pins.
// * Connect the GROUND (-) of the Raspberry Pi to the GROUND (-) of the ESC's signal input
//   AND the GROUND (-) of the separate power supply.
// * ESCs require an ARMING sequence (low throttle on power-up).
// * ESCs often require CALIBRATION. Values here are common defaults.
//
// *** SOFTWARE PWM LIMITATIONS ***
// Software-based PWM is inherently inaccurate due to OS scheduling.
// Use hardware PWM for precise or fast signals.
//
// Interrupting (Ctrl-C) exits immediately without guaranteeing a low throttle signal.

use std::error::Error;
use std::thread;
use std::time::Duration;

use rppal::gpio::Gpio;

// Gpio uses BCM pin numbering. BCM GPIO 23 is tied to physical pin 16.
// CHANGE THIS PIN if GPIO 23 is unavailable or you prefer another.
const GPIO_PWM: u8 = 23;

// ESC/Motor configuration.
const PERIOD_MS: u64 = 20; // 50 Hz, typical for ESCs
const THROTTLE_MIN_US: u64 = 1000; // Arming signal, zero throttle
const THROTTLE_NEUTRAL_US: u64 = 1500; // "Neutral" or mid-point for these ramps
const THROTTLE_MAX_US: u64 = 2000; // Full throttle
const ARMING_DURATION_S: u64 = 3; // Adjust based on your ESC (usually 2-5 seconds)

// Ramp parameters
const STEP_US: u64 = 5; // Pulse width change per step
const DELAY_MS: u64 = 300; // Delay between steps for smoother ramping

fn main() -> Result<(), Box<dyn Error>> {
    println!("*** Starting ESC Control with Custom Ramps (Software PWM) ***");
    println!("*** SAFETY WARNING: Ensure propeller is removed and grounds are connected! ***");
    println!("Using GPIO {}", GPIO_PWM);

    let mut pin = Gpio::new()?.get(GPIO_PWM)?.into_output();

    // --- 1. Initialization and Arming ---
    println!(
        "Initializing Software PWM on GPIO {} with {}ms period.",
        GPIO_PWM, PERIOD_MS
    );
    println!(
        "Sending MINIMUM throttle ({} µs) for arming...",
        THROTTLE_MIN_US
    );
    pin.set_pwm(
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(THROTTLE_MIN_US),
    )?;
    println!(
        "Holding minimum throttle for {} seconds for ESC arming...",
        ARMING_DURATION_S
    );
    thread::sleep(Duration::from_secs(ARMING_DURATION_S));
    println!("Arming sequence complete (hopefully!).");

    // --- 2. Set to Neutral (1500µs) after Arming ---
    println!(
        "\nSetting throttle to NEUTRAL ({} µs).",
        THROTTLE_NEUTRAL_US
    );
    pin.set_pwm(
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(THROTTLE_NEUTRAL_US),
    )?;
    thread::sleep(Duration::from_secs(2)); // Hold at neutral

    // --- 3. Ramp from 1500µs down to 1000µs ---
    println!(
        "Ramping throttle from {} µs down to {} µs...",
        THROTTLE_NEUTRAL_US, THROTTLE_MIN_US
    );
    // Iterate from THROTTLE_MIN_US up to THROTTLE_NEUTRAL_US, then reverse
    for pulse in (THROTTLE_MIN_US..=THROTTLE_NEUTRAL_US)
        .rev() // Goes from THROTTLE_NEUTRAL_US down to THROTTLE_MIN_US
        .step_by(STEP_US as usize)
    {
        if pulse < THROTTLE_MIN_US {
            continue;
        } // Ensure we don't go below min
        print!("\rSetting throttle: {} µs  ", pulse);
        pin.set_pwm(
            Duration::from_millis(PERIOD_MS),
            Duration::from_micros(pulse),
        )?;
        thread::sleep(Duration::from_millis(DELAY_MS));
    }
    // Ensure it ends precisely at THROTTLE_MIN_US
    pin.set_pwm(
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(THROTTLE_MIN_US),
    )?;
    println!(
        "\nReached MINIMUM throttle ({} µs). Holding.",
        THROTTLE_MIN_US
    );
    thread::sleep(Duration::from_secs(2));

    // --- 4. Return to Neutral (1500µs) ---
    println!(
        "\nSetting throttle back to NEUTRAL ({} µs).",
        THROTTLE_NEUTRAL_US
    );
    pin.set_pwm(
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(THROTTLE_NEUTRAL_US),
    )?;
    thread::sleep(Duration::from_secs(2)); // Hold at neutral

    // --- 5. Ramp from 1500µs up to 2000µs ---
    println!(
        "Ramping throttle from {} µs up to {} µs...",
        THROTTLE_NEUTRAL_US, THROTTLE_MAX_US
    );
    for pulse in (THROTTLE_NEUTRAL_US..=THROTTLE_MAX_US).step_by(STEP_US as usize) {
        if pulse > THROTTLE_MAX_US {
            continue;
        } // Ensure we don't go above max
        print!("\rSetting throttle: {} µs  ", pulse);
        pin.set_pwm(
            Duration::from_millis(PERIOD_MS),
            Duration::from_micros(pulse),
        )?;
        thread::sleep(Duration::from_millis(DELAY_MS));
    }
    // Ensure it ends precisely at THROTTLE_MAX_US
    pin.set_pwm(
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(THROTTLE_MAX_US),
    )?;
    println!(
        "\nReached MAXIMUM throttle ({} µs). Holding.",
        THROTTLE_MAX_US
    );
    thread::sleep(Duration::from_secs(3)); // Hold at max a bit longer

    println!("--- Motor Control Finished ---");

    // Software PWM stops automatically when `pin` goes out of scope.
    Ok(())
}

