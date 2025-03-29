use tokio::spawn;
mod lib;
use gilrs::{Button, Event, GamepadId, Gilrs};
use lib::autonomous::Autonomous;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;
use tokio::sync::mpsc::unbounded_channel;

#[tokio::main]
async fn main() {
    let mut blimp = blimp::SanoBlimp::new();
    let mut detection = Detection::new();

    let mut time_p = std::time::Instant::now();

    let mut auto = Autonomous::new(1.0, 0.8, 1.0, 0.0, 0.0, 0.0, 0.0);

    // The big event loop
    loop {
        blimp.update();
        let det = detection.detect(vec![2, 4, 5, 6, 7, 8, 9, 10]);
        if blimp.is_manual() {
            // Manual control
            blimp.manual();
        } else {
            // Autonomous
            if det.len() > 1 {
                let auto_input = auto.position(-0.5, det[0] as f32, det[1] as f32);
                println!("{:?}", auto_input);
                blimp.update_input(auto_input);
                let acc = blimp.mix();
                blimp.actuator.actuate(acc);
            } else {
                blimp.update_input((0.0, 0.0, 0.0));
                // Search
            }
        }
    }
}
