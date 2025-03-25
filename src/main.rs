use tokio::spawn;
mod lib;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;
use tokio::sync::mpsc::unbounded_channel;

fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[tokio::main]
async fn main() {
    let mut blimp = Arc::new(Mutex::new(blimp::SanoBlimp::new()));
    let mut detection = Detection::new();

    let mut prev_err_x = 0;
    let mut prev_err_y = 0;
    let mut time_p = std::time::Instant::now();

    let kp = 1.0;
    let kd = 0.0;

    let autonomous = Arc::new(Mutex::new(false));
    let notifier = Arc::new(Notify::new());

    let (tx, mut rx_detection) = unbounded_channel();

    let autonomous_clone = Arc::clone(&autonomous);

    // Spawn object detection task
    tokio::spawn(async move {
        detection.detect(tx).await;
    });

    let blimp_clone = blimp.clone();

    tokio::spawn(async move {
        loop {
            blimp_clone.lock().unwrap().run();
        }
    });

    let blimp_clone_autonomous = blimp.clone();
    // Process the detection and update input with the given actuations
    while let Some(det) = rx_detection.recv().await {
        if det.len() > 1 {
            let err_x = (det[0] - 300) as i32;
            let err_y = (150 - det[1]) as i32;

            println!("ex: {}, ey: {}", err_x, err_y);

            let delta_x = prev_err_x - err_x;
            let delta_y = prev_err_y - err_y;

            prev_err_y = err_y;
            prev_err_x = err_x;
            println!("dx: {}, dy: {}", delta_x, delta_y);

            let delta_t = time_p.elapsed().as_millis();

            println!("dt: {}", delta_t);

            let acc_y = err_x as f32 * kp + (kd * (delta_x as f32 / delta_t as f32)) as f32;
            let acc_z = err_y as f32 * kp + (kd * (delta_y as f32 / delta_t as f32)) as f32;

            let ac_y = map(acc_y as i32, -300, 300, -1, 1);
            let ac_z = map(acc_z as i32, -150, 150, -1, 1);

            blimp_clone_autonomous
                .lock()
                .unwrap()
                .update_input((0.3, ac_y as f32, ac_z as f32));
        }

        // Run autonomous code
        time_p = std::time::Instant::now();
    }

    // Ensure the object detection worker finishes
}
