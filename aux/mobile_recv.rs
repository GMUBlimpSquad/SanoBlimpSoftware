use bluer::{Adapter, Address, Device, Uuid, Session};
// Corrected evdev imports based on compiler errors
use evdev::{
    AbsoluteAxisCode,
    BusType, // For InputId::new and BUS_BLUETOOTH
    EventType,
    InputEvent,
    InputId,
    SynchronizationCode,
    // Assuming AbsoluteInfo and UinputDevice are in the uinput module as per errors
    uinput::{AbsoluteInfo, UinputDevice},
};
// Removed BTreeSet as it's unused
use std::error::Error;
use std::ffi::CString;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let session = Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;

    println!("Setting up virtual gamepad...");

    let device_name_c = CString::new("Virtual Bluetooth Gamepad")?;
    
    let input_id = InputId::new(
        BusType::BUS_BLUETOOTH, // Corrected to BUS_BLUETOOTH
        0x1234,             
        0x5678,             
        0x0101,             
    );

    let abs_axes_to_enable = [AbsoluteAxisCode::ABS_X, AbsoluteAxisCode::ABS_Y];
    
    let uinput_device = UinputDevice::create(
        &device_name_c,
        Some(input_id),
        None, 
        0,    
        Some(&abs_axes_to_enable), 
        None, 
        None, 
        None, 
        None, 
        None, 
        None, 
        None, 
        false, 
    )?;

    let axis_min = -32767;
    let axis_max = 32767;
    let axis_initial_value = 0;
    let axis_fuzz = 0;
    let axis_flat = 0;
    let axis_resolution = 0;

    let abs_x_info = AbsoluteInfo::new(
        axis_initial_value,
        axis_min,
        axis_max,
        axis_fuzz,
        axis_flat,
        axis_resolution,
    );
    let abs_y_info = AbsoluteInfo::new(
        axis_initial_value,
        axis_min,
        axis_max,
        axis_fuzz,
        axis_flat,
        axis_resolution,
    );

    uinput_device.write_abs_info(AbsoluteAxisCode::ABS_X, abs_x_info)?;
    uinput_device.write_abs_info(AbsoluteAxisCode::ABS_Y, abs_y_info)?;

    println!("Virtual gamepad created. Waiting for Bluetooth connections...");

    loop {
        match adapter.device_addresses().await {
            Ok(addresses) => {
                for addr in addresses {
                    // Removed .await as adapter.device() returns Result directly
                    match adapter.device(addr) { 
                        Ok(device_obj) => {
                            if device_obj.is_connected().await.unwrap_or(false) {
                                if let Some(name) = device_obj.name().await? {
                                    println!("Connected to {} ({})", name, addr);

                                    let services_result = device_obj.uuids().await;
                                    let services_set = services_result?.unwrap_or_default();
                                    
                                    if services_set.contains(&Uuid::from_u128(0x1812)) { // HID Service
                                        println!("Joystick HID service detected on {}!", name);

                                        while device_obj.is_connected().await.unwrap_or(false) {
                                            if let Some(data_str) = receive_joystick_data(&device_obj).await {
                                                println!("Received from {}: {}", name, data_str);
                                                let (x, y) = parse_joystick_data(data_str);

                                                if let Err(e) = send_gamepad_input(&uinput_device, x, y) {
                                                    eprintln!("Failed to send gamepad input: {}", e);
                                                }
                                            }
                                            sleep(Duration::from_millis(20)).await;
                                        }
                                        println!("Device {} disconnected.", name);
                                    }
                                }
                            }
                        }
                        Err(e) => eprintln!("Error getting device object for {}: {}", addr, e),
                    }
                }
            }
            Err(e) => {
                eprintln!("Error scanning for device addresses: {}", e);
            }
        }
        sleep(Duration::from_secs(2)).await;
    }
}

async fn receive_joystick_data(device: &Device) -> Option<String> {
    // Changed to iterate services then characteristics, as all_gatt_characteristics() is not available
    match device.services().await {
        Ok(services_stream) => {
            for service in services_stream {
                // You might want to filter by service UUID here if you know it
                // e.g., if service.uuid().await.ok()? == known_service_uuid 
                match service.characteristics().await {
                    Ok(characteristics_stream) => {
                        for characteristic in characteristics_stream {
                            // Check if characteristic supports read
                            if let Ok(flags) = characteristic.flags().await {
                                if flags.read {
                                    match characteristic.read_value().await {
                                        Ok(data_bytes) => {
                                            match String::from_utf8(data_bytes) {
                                                Ok(s) => return Some(s),
                                                Err(e) => {
                                                    eprintln!("Failed to parse UTF-8 data from char {}: {}", characteristic.uuid().await.unwrap_or_default(), e);
                                                }
                                            }
                                        }
                                        Err(e) => {
                                            // Error reading characteristic value
                                            // eprintln!("Failed to read characteristic value from {}: {}", characteristic.uuid().await.unwrap_or_default(), e);
                                        }
                                    }
                                }
                            } else {
                                // Error getting characteristic flags
                                // eprintln!("Failed to get flags for char {}", characteristic.uuid().await.unwrap_or_default());
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Failed to get characteristics for service {}: {}", service.uuid().await.unwrap_or_default(), e);
                    }
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to get services for device {}: {}", device.address(), e);
        }
    }
    None
}

fn parse_joystick_data(data: String) -> (i16, i16) {
    let mut x = 0i16;
    let mut y = 0i16;

    for part in data.split_whitespace() {
        if let Some(val_str) = part.strip_prefix("X:") {
            if let Ok(val) = val_str.parse::<i16>() {
                x = val;
            } else {
                eprintln!("Failed to parse X value from: {}", val_str);
            }
        } else if let Some(val_str) = part.strip_prefix("Y:") {
            if let Ok(val) = val_str.parse::<i16>() {
                y = val;
            } else {
                eprintln!("Failed to parse Y value from: {}", val_str);
            }
        }
    }
    (x, y)
}

fn send_gamepad_input(uinput_device: &UinputDevice, x: i16, y: i16) -> std::io::Result<()> {
    let events = [
        InputEvent::new_now(EventType::ABSOLUTE.0, AbsoluteAxisCode::ABS_X.0, x as i32),
        InputEvent::new_now(EventType::ABSOLUTE.0, AbsoluteAxisCode::ABS_Y.0, y as i32),
        InputEvent::new_now(EventType::SYNCHRONIZATION.0, SynchronizationCode::SYN_REPORT.0, 0),
    ];
    uinput_device.emit(&events)
}

