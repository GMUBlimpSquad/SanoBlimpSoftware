use bluer::{Adapter, Address, Device, Uuid};
use evdev::{uinput::UinputDevice, AbsoluteAxisType, Device as EvDevice, InputEvent, Key};
use std::error::Error;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;

    println!("Waiting for Bluetooth connections...");

    // Setup the gamepad device
    let device = EvDevice::new().unwrap();
    let uinput = UinputDevice::create_from_device(&device).unwrap();

    // Scan for devices
    loop {
        let devices = adapter.devices().await?;
        for device in devices {
            if let Ok(true) = device.is_connected().await {
                if let Some(name) = device.name().await? {
                    println!("Connected to {}", name);

                    let services = device.uuids().await.unwrap_or(vec![]);
                    if services.contains(&Uuid::from_u128(0x1812)) {
                        println!("Joystick detected!");

                        while device.is_connected().await.unwrap_or(false) {
                            if let Some(data) = receive_joystick_data(&device).await {
                                let (x, y) = parse_joystick_data(data);

                                // Map to gamepad input
                                send_gamepad_input(&uinput, x, y).unwrap();
                            }
                            sleep(Duration::from_millis(50)).await;
                        }
                    }
                }
            }
        }
        sleep(Duration::from_secs(2)).await;
    }
}

// Function to receive joystick data over Bluetooth
async fn receive_joystick_data(device: &Device) -> Option<String> {
    let chars = device.characteristics().await.ok()?;
    for characteristic in chars {
        if let Ok(data) = characteristic.read_value().await {
            return String::from_utf8(data).ok();
        }
    }
    None
}

// Parse joystick data from Flutter
fn parse_joystick_data(data: String) -> (i16, i16) {
    let parts: Vec<&str> = data.split_whitespace().collect();
    let x = parts.get(0).unwrap().replace("X:", "").parse().unwrap_or(0);
    let y = parts.get(1).unwrap().replace("Y:", "").parse().unwrap_or(0);
    (x, y)
}

// Function to send gamepad input
fn send_gamepad_input(device: &UinputDevice, x: i16, y: i16) -> std::io::Result<()> {
    device.emit(&[
        InputEvent::new_now(AbsoluteAxisType, AbsoluteAxisType::ABS_X, x),
        InputEvent::new_now(AbsoluteAxisType, AbsoluteAxisType::ABS_Y, y),
    ])
}
