use std::error::Error;
use std::io::{BufRead, BufReader};
use std::net::UdpSocket;
use std::time::Duration;

use ab_glyph::{FontRef, PxScale};
use base64;
use image::{DynamicImage, GenericImageView, ImageFormat, Rgba, RgbaImage};
use imageproc::drawing::{draw_hollow_rect_mut, draw_text_mut};
use imageproc::rect::Rect;
use serde_json::Value;
use serialport::SerialPort;
use std::io::Cursor;
use tokio::sync::mpsc::UnboundedSender;

#[cfg(unix)]
const DEFAULT_TTY: &str = "/dev/ttyACM0";
#[cfg(windows)]
const DEFAULT_TTY: &str = "COM1";

// Detects objects
pub struct Detection {
    reader: BufReader<Box<dyn SerialPort>>,
    udp_socket: UdpSocket,
}

impl Detection {
    // TODO Make the streaming part optional
    // Get the IP from a config file
    pub fn new() -> Self {
        let port_name = "/dev/ttyACM0"; // Adjust to your serial port
        let baud_rate = 921600;
        let serial_port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs(1))
            .open()
            .unwrap();
        let mut reader = BufReader::new(serial_port);

        // --- Setup UDP sender ---
        let udp_socket = UdpSocket::bind("0.0.0.0:0").unwrap(); // binds to an ephemeral port
        Detection { reader, udp_socket }
    }

    /// Probably add a channel into the parameter to send the detection from a different thread
    pub async fn detect(&mut self, tx: UnboundedSender<Vec<i32>>) {
        let udp_dest = "192.168.8.194:54321";
        let font_data: &[u8] = include_bytes!("FiraCode-Regular.ttf");
        let font = FontRef::try_from_slice(font_data).expect("Error constructing Font");
        println!("Starting streaming");
        loop {
            let mut line = String::new();
            let n = self.reader.read_line(&mut line).unwrap();
            if n == 0 {
                continue;
            }
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }

            let json_val: Value = match serde_json::from_str(trimmed) {
                Ok(val) => val,
                Err(e) => {
                    continue;
                }
            };

            if json_val["type"] == 1 {
                let data_field = &json_val["data"];
                let boxes = data_field["boxes"].as_array().unwrap();
                let image_b64 = data_field["image"].as_str().unwrap_or("");
                if image_b64.is_empty() {
                    continue;
                }

                // --- Decode the base64 image ---
                let image_bytes = match base64::decode(image_b64) {
                    Ok(bytes) => bytes,
                    Err(e) => {
                        continue;
                    }
                };

                // --- Load image from memory ---
                let dyn_img = match image::load_from_memory(&image_bytes) {
                    Ok(img) => img,
                    Err(e) => {
                        continue;
                    }
                };

                // Convert the image to RGBA (needed for drawing)
                let mut rgba_img: RgbaImage = dyn_img.to_rgba8();

                // --- Annotate the image ---
                for b in boxes {
                    if let Some(arr) = b.as_array() {
                        if arr.len() < 6 {
                            continue;
                        }
                        let x = arr[0].as_i64().unwrap_or(0) as i32;
                        let y = arr[1].as_i64().unwrap_or(0) as i32;
                        let w = arr[2].as_i64().unwrap_or(0) as i32;
                        let h = arr[3].as_i64().unwrap_or(0) as i32;
                        let label = arr[5].to_string();
                        tx.send(
                            [x + w / 2, y + h / 2, arr[5].as_i64().unwrap_or(0) as i32].to_vec(),
                        );

                        // Draw a green rectangle.
                        let rect = Rect::at(x, y).of_size(w as u32, h as u32);
                        draw_hollow_rect_mut(&mut rgba_img, rect, Rgba([0, 255, 0, 255]));

                        // Draw a yellow label above the rectangle.
                        let scale = PxScale { x: 16.0, y: 16.0 };
                        let text_x = x;
                        let text_y = if y >= 20 { y - 20 } else { y };
                        draw_text_mut(
                            &mut rgba_img,
                            Rgba([255, 255, 0, 255]),
                            text_x,
                            text_y,
                            scale,
                            &font,
                            &label,
                        );
                    }
                }

                // --- Encode the annotated image as JPEG ---
                let mut jpeg_buf = Vec::new();
                {
                    let mut cursor = Cursor::new(&mut jpeg_buf);
                    DynamicImage::ImageRgba8(rgba_img)
                        .resize(180, 180, image::imageops::FilterType::Nearest)
                        .write_to(&mut cursor, ImageFormat::Png)
                        .unwrap();
                }
                // --- Send the JPEG bytes over UDP ---
                if let Err(e) = self.udp_socket.send_to(&jpeg_buf, udp_dest) {
                    eprintln!("UDP send error: {:?}", e);
                }
            }
        }
    }
}
