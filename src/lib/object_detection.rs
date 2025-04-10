use std::error::Error;
use std::io::{BufRead, BufReader, Cursor};
use std::net::UdpSocket;
use std::time::Duration;

use ab_glyph::{FontRef, PxScale};
use base64;
use image::{DynamicImage, GenericImageView, ImageFormat, Rgba, RgbaImage};
use imageproc::drawing::{
    draw_filled_circle_mut, draw_hollow_rect_mut, draw_line_segment_mut, draw_text_mut, Canvas,
};
use imageproc::rect::Rect;
use serde_json::Value;
use serialport::SerialPort;

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
    /// Create a new `Detection` struct by opening the serial port and binding a UDP socket.
    pub fn new() -> Self {
        let port_name = DEFAULT_TTY;
        let baud_rate = 921600;

        let serial_port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs(1))
            .open()
            .expect("Failed to open the serial port");

        let reader = BufReader::new(serial_port);

        // Binds to an ephemeral local port
        let udp_socket = UdpSocket::bind("0.0.0.0:0").expect("Failed to bind local UDP socket");

        Detection { reader, udp_socket }
    }

    /// Given a list of detection boxes and target classes, finds the largest bounding box
    /// whose class is in `target`. Returns `[x, y, w, h, conf, class]` or an empty Vec if none found.
    fn get_largest(&self, boxes: &[Value], target: &[i32]) -> Vec<i32> {
        let mut largest_area = 0;
        let mut largest_bb = Vec::new(); // Will store [x, y, w, h, conf, cls]

        for b in boxes {
            if let Some(arr) = b.as_array() {
                // Ensure we have at least 6 entries: x, y, w, h, conf, cls
                if arr.len() < 6 {
                    continue;
                }

                let x = arr[0].as_i64().unwrap_or(0) as i32;
                let y = arr[1].as_i64().unwrap_or(0) as i32;
                let w = arr[2].as_i64().unwrap_or(0) as i32;
                let h = arr[3].as_i64().unwrap_or(0) as i32;
                let conf = arr[4].as_i64().unwrap_or(0) as i32;
                let cls = arr[5].as_i64().unwrap_or(0) as i32;

                // Check if this detection's class is in our target list
                if target.contains(&cls) {
                    let area = w * h;
                    if area > largest_area {
                        largest_area = area;
                        largest_bb = vec![x, y, w, h, conf, cls];
                    }
                }
            }
        }

        largest_bb
    }

    /// Reads a JSON line from serial, extracts the largest bounding box for the given target classes,
    /// draws annotations on the image, and sends it out via UDP. Returns the `[cx, cy]` of the box center or
    /// an empty Vec if not found or if an error occurs.
    pub fn detect(&mut self, target: Vec<i32>) -> Vec<i32> {
        // Adjust this to your actual destination
        let udp_dest = "192.168.8.195:54321";

        // Embed the font file in the binary
        let font_data: &[u8] = include_bytes!("FiraCode-Regular.ttf");
        let font = FontRef::try_from_slice(font_data)
            .expect("Error constructing the font from embedded bytes");

        // Read a line from serial
        let mut line = String::new();
        if let Err(e) = self.reader.read_line(&mut line) {
            eprintln!("Error reading line from serial: {:?}", e);
            return vec![];
        }

        // Parse JSON from the line
        let trimmed = line.trim();
        let json_val: Value = match serde_json::from_str(trimmed) {
            Ok(val) => val,
            Err(e) => {
                return vec![];
            }
        };

        // We only care about messages where "type" == 1
        if json_val["type"].as_i64() == Some(1) {
            let data_field = &json_val["data"];

            // "boxes" must be an array
            let boxes = match data_field["boxes"].as_array() {
                Some(arr) => arr,
                None => {
                    eprintln!("data.boxes is not an array or missing");
                    return vec![];
                }
            };

            // Base64-encoded image data
            let image_b64 = data_field["image"].as_str().unwrap_or("");
            if image_b64.is_empty() {
                eprintln!("No image data found in JSON");
                return vec![];
            }

            // Decode base64
            let image_bytes = match base64::decode(image_b64) {
                Ok(bytes) => bytes,
                Err(e) => {
                    eprintln!("Error decoding base64: {:?}", e);
                    return vec![];
                }
            };

            // Load image from memory
            let dyn_img = match image::load_from_memory(&image_bytes) {
                Ok(img) => img,
                Err(e) => {
                    eprintln!("Error decoding image: {:?}", e);
                    return vec![];
                }
            };

            // Find the largest bounding box of the requested classes
            let arr = self.get_largest(boxes, &target);
            if arr.len() < 6 {
                // Could not find any bounding box matching our targets

                let mut rgba_img: RgbaImage = dyn_img.to_rgba8();
                // Encode the annotated image (e.g., as PNG)
                let mut img_buf = Vec::new();
                {
                    let mut cursor = Cursor::new(&mut img_buf);
                    DynamicImage::ImageRgba8(rgba_img)
                        .resize(180, 180, image::imageops::FilterType::Nearest)
                        .write_to(&mut cursor, ImageFormat::Png)
                        .expect("Failed to write image to buffer");
                }

                // Send via UDP
                if let Err(e) = self.udp_socket.send_to(&img_buf, udp_dest) {
                    eprintln!("UDP send error: {:?}", e);
                }

                return vec![];
            }

            // Extract bounding box data
            let x = arr[0];
            let y = arr[1];
            let w = arr[2];
            let h = arr[3];
            let conf = arr[4];
            let cls = arr[5];

            let cx = x + w / 2;
            let cy = y + h / 2;

            // Convert to RGBA so we can annotate
            let mut rgba_img: RgbaImage = dyn_img.to_rgba8();

            // 1) Draw a green rectangle
            let rect = Rect::at(x, y).of_size(w as u32, h as u32);
            draw_hollow_rect_mut(&mut rgba_img, rect, Rgba([0, 255, 0, 255]));

            // 2) Draw a filled circle at the center
            draw_filled_circle_mut(&mut rgba_img, (cx, cy), 20, Rgba([255, 0, 0, 255]));

            println!("height: {}, width: {}", rgba_img.height(), rgba_img.width());

            //3) Draw two line segments
            draw_line_segment_mut(
                &mut rgba_img,
                (cx as f32, cy as f32),
                ((160) as f32, cy as f32),
                Rgba([255, 0, 0, 255]),
            );
            draw_line_segment_mut(
                &mut rgba_img,
                (cx as f32, cy as f32),
                (cx as f32, (120) as f32),
                Rgba([255, 0, 0, 255]),
            );

            // 4) Draw a label above the rectangle
            let label = format!("cls={cls} conf={conf}");
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

            // Encode the annotated image (e.g., as PNG)
            let mut img_buf = Vec::new();
            {
                let mut cursor = Cursor::new(&mut img_buf);
                DynamicImage::ImageRgba8(rgba_img)
                    .resize(180, 180, image::imageops::FilterType::Nearest)
                    .write_to(&mut cursor, ImageFormat::Png)
                    .expect("Failed to write image to buffer");
            }

            // Send via UDP
            if let Err(e) = self.udp_socket.send_to(&img_buf, udp_dest) {
                eprintln!("UDP send error: {:?}", e);
            }

            // Return center coordinates
            return vec![cx, cy];
        }

        // If "type" != 1 or something else is off, return an empty vector
        vec![]
    }
}
