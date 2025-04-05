use tokio::spawn;
mod lib;
use gilrs::{Button, Event, GamepadId, Gilrs};
use lib::autonomous::Autonomous;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use serde::Deserialize;
use std::fs;
use std::sync::{Arc, Mutex};
use tokio::sync::Notify;
use tokio::sync::mpsc::unbounded_channel;
use tokio::time::timeout;

#[derive(Deserialize, Debug)]
struct Config {
    motor: Motor,
    controller: Controller,
    server: Server,
}

#[derive(Deserialize, Debug)]
struct Server {
    host: String,
    port: u16,
}

#[derive(Deserialize, Debug)]
struct Motor {
    m1_mul: f32,
    m2_mul: f32,
    m3_mul: f32,
    m4_mul: f32,
}

#[derive(Deserialize, Debug)]
struct Controller {
    kp_x: f32,
    kp_y: f32,
    kp_z: f32,
    kd_x: f32,
    kd_y: f32,
    kd_z: f32,
}

fn read_config() -> Config {
    // Get the path as command line input
    let conf_str = std::fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&conf_str).expect("Failed to parse config file");
    config
}

#[tokio::main]
async fn main() {
    let conf = read_config();

    let mut blimp = blimp::SanoBlimp::new();
    let mut detection = Detection::new();

    let mut time_p = std::time::Instant::now();

    let mut auto = Autonomous::new(
        conf.controller.kp_x,
        conf.controller.kp_y,
        conf.controller.kp_z,
        conf.controller.kd_x,
        conf.controller.kd_y,
        conf.controller.kd_z,
        0.0,
    );

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
                let auto_input = auto.position(-1.0, det[0] as f32, det[1] as f32);
                println!("{:?}", auto_input);
                blimp.update_input(auto_input);
                let acc = blimp.mix();
                blimp.actuator.actuate(acc);
                time_p = std::time::Instant::now();
            } else {
                blimp.update_input((0.0, 0.0, 0.0));
                // Search
                println!("Searching");

                if time_p.elapsed() > std::time::Duration::from_millis(1000) {
                    let acc = blimp.mix();
                    blimp.actuator.actuate(acc);
                }
            }
        }
    }
}
