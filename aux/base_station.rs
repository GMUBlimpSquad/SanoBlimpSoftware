use image::{GenericImageView, ImageFormat};
use minifb::{Key, Window, WindowOptions};
use std::error::Error;
use std::net::UdpSocket;

const WIDTH: usize = 640; // Default width, will be updated by first image
const HEIGHT: usize = 480; // Default height, will be updated by first image
const BUFFER_SIZE: usize = 65536; // Max UDP packet size slightly > 65535

fn main() -> Result<(), Box<dyn Error>> {
    let udp_ip = "0.0.0.0";
    let udp_port = 54321;
    let bind_addr = format!("{}:{}", udp_ip, udp_port);

    // Create and bind the UDP socket
    let socket = UdpSocket::bind(&bind_addr)?;
    println!("Listening on {}", bind_addr);

    // Buffer for receiving UDP data
    let mut receive_buffer = vec![0u8; BUFFER_SIZE];

    // Option to hold the window, created after the first image is received
    let mut window: Option<Window> = None;
    // Buffer to hold pixel data for minifb (u32 format)
    let mut pixel_buffer: Vec<u32> = Vec::new();

    loop {
        // Receive data from the UDP socket
        match socket.recv_from(&mut receive_buffer) {
            Ok((size, src_addr)) => {
                if size > 0 {
                    println!("Received {} bytes from {}", size, src_addr);
                    let image_data = &receive_buffer[..size];

                    // Attempt to decode the received bytes as a JPEG image
                    match image::load_from_memory_with_format(image_data, ImageFormat::Jpeg) {
                        Ok(img) => {
                            let (width, height) = img.dimensions();
                            println!("Decoded image: {}x{}", width, height);

                            // Convert the image to RGBA format for easier processing
                            // minifb usually expects ARGB or XRGB (u32), we'll pack it
                            let rgba_img = img.to_rgba8();

                            // Prepare the pixel buffer for minifb (XRGB format: 0xFFRRGGBB)
                            pixel_buffer.resize((width * height) as usize, 0);
                            for (x, y, pixel) in rgba_img.enumerate_pixels() {
                                let r = pixel[0] as u32;
                                let g = pixel[1] as u32;
                                let b = pixel[2] as u32;
                                // a = pixel[3] // Alpha ignored for XRGB
                                let packed_pixel = (r << 16) | (g << 8) | b; // Or 0xFF000000 | (r << 16) | (g << 8) | b for ARGB
                                let index = (y * width + x) as usize;
                                if index < pixel_buffer.len() {
                                    pixel_buffer[index] = packed_pixel;
                                }
                            }

                            // Create or update the window
                            if let Some(win) = &mut window {
                                // If window exists, update its buffer
                                // Note: This simple version doesn't resize the window if image dimensions change.
                                // It will display the new image using the original window size,
                                // potentially cropping or showing empty space.
                                win.update_with_buffer(
                                    &pixel_buffer,
                                    width as usize,
                                    height as usize,
                                )?;
                            } else {
                                // If window doesn't exist, create it with the dimensions of the first image
                                let options = WindowOptions {
                                    resize: true, // Allow user resizing
                                    ..WindowOptions::default()
                                };
                                let mut new_window = Window::new(
                                    "Received Image - Press Q to quit",
                                    width as usize,
                                    height as usize,
                                    options,
                                )?;
                                // Limit update rate to prevent excessive CPU usage
                                new_window.limit_update_rate(Some(
                                    std::time::Duration::from_micros(16600),
                                )); // ~60fps
                                new_window.update_with_buffer(
                                    &pixel_buffer,
                                    width as usize,
                                    height as usize,
                                )?;
                                window = Some(new_window);
                            }
                        }
                        Err(e) => {
                            eprintln!("Failed to decode image: {}", e);
                            // Optional: clear the window if decoding fails after success?
                            // if let Some(win) = &mut window {
                            //     pixel_buffer.fill(0); // Fill with black
                            //     win.update_with_buffer(&pixel_buffer, win.get_size().0, win.get_size().1)?;
                            // }
                        }
                    }
                }
            }
            Err(e) => {
                eprintln!("UDP receive error: {}", e);
                // Consider breaking the loop on certain errors if needed
            }
        }

        // Check window status and handle quit event
        if let Some(win) = &mut window {
            // Check if window is open and if 'Q' is pressed
            if !win.is_open() || win.is_key_down(Key::Q) {
                break; // Exit loop if window closed or Q pressed
            }

            // If we didn't update the buffer in this iteration (e.g., decode failed),
            // we might still need to call update() to process window events.
            // However, update_with_buffer usually handles event processing too.
            // If the window feels unresponsive on decode errors, uncomment the line below.
            // win.update();
        } else {
            // If the window hasn't been created yet (no valid image received),
            // we can add a small sleep to prevent the loop from spinning too fast
            // while waiting for the first image. This is optional.
            // std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    println!("Exiting...");
    Ok(()) // Socket and window are automatically closed/dropped when they go out of scope
}
