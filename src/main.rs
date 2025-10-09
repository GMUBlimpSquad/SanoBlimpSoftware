#![allow(warnings)]

use tokio::spawn;
mod lib;
use core::time;
use gilrs::{Button, Event, GamepadId, Gilrs};
use lib::autonomous::{self, Autonomous};
use lib::base_station::communication::*;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use serde::Deserialize;
use std::fmt::format;
use std::fs;
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::thread;
use tokio::sync::Notify;
use tokio::sync::mpsc::unbounded_channel;
use tokio::time::timeout;

use rand::Rng; // Use Rng trait
mod driver;

#[derive(Deserialize, Debug)]
struct Config {
    motor: Motor,
    controller: Controller,
    server: Server,
    blimp: Blimp_Config,
}

//
#[derive(Deserialize, Debug)]
struct Blimp_Config {
    blimp_type: String,
    blimp_version: u8,
}

#[derive(Deserialize, Debug)]
struct Server {
    host: String,
    port: u16,
    port_stat: u16,
}

#[derive(Deserialize, Debug)]
struct Motor {
    m1_mul: f32,
    m2_mul: f32,
    m3_mul: f32,
    m4_mul: f32,
    midpoint_angle: f32,
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

    let mut rng = rand::thread_rng(); // Use thread_rng() correctly
    // start communication to the base station in a different thread
    let stats_socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            return;
        }
    };

    let mut save_image = false;

    stats_socket
        .set_read_timeout(Some(std::time::Duration::from_millis(50)))
        .ok();

    let base_station_control_addr = format!("{}:{}", &conf.server.host, conf.server.port_stat);

    let blimp_id = std::fs::read_to_string("/etc/hostname").unwrap();

    // Send Connect Message
    let connect_msg = BlimpToBaseMessage::Connect {
        id: blimp_id.clone(),
    };
    if let Ok(payload) = serde_json::to_vec(&connect_msg) {
        stats_socket
            .send_to(&payload, &base_station_control_addr)
            .ok();

        println!("connected to the base station");
    } else {
        /* ... error handling ... */
        println!("Error Sending message");
    }

    //let mut blimp = match conf.blimp.blimp_type {
    //    String::from("flappy") => blimp::Flappy::new(),
    //    String::from("sano") => blimp::SanoBlimp::new(),
    //    _ => blimp::SanoBlimp::new(),
    //};
    //

    // TODO Update this to take in the whole motor config
    let mut blimp = blimp::SanoBlimp::new(
        conf.motor.m1_mul,
        conf.motor.m2_mul,
        conf.motor.midpoint_angle,
        conf.blimp.blimp_version,
    );

    //std::thread::sleep(std::time::Duration::from_secs(5));

    // let mut detection = Detection::new(
    //     conf.server.host.clone(),
    //     conf.server.port.clone(),
    //     String::from("/dev/ttyACM0"),
    //     921600,
    // );
    //.unwrap();

    let mut time_p = std::time::Instant::now();
    let mut state_timer = Arc::new(Mutex::new(std::time::Instant::now()));
    let mut state = Arc::new(Mutex::new(BlimpStates::Ball));
    let mut search_timer = std::time::Instant::now();

    if state_timer.lock().unwrap().elapsed() > std::time::Duration::from_secs(120) {
        match *state.lock().unwrap() {
            BlimpStates::Ball => {
                *state.lock().unwrap() = BlimpStates::Goal;
                *state_timer.lock().unwrap() = std::time::Instant::now()
            }
            BlimpStates::Goal => {
                *state.lock().unwrap() = BlimpStates::Ball;
                *state_timer.lock().unwrap() = std::time::Instant::now()
            }
        }
    }

    let mut auto = Autonomous::new(
        conf.controller.kp_x,
        conf.controller.kp_y,
        conf.controller.kp_z,
        conf.controller.kd_x,
        conf.controller.kd_y,
        conf.controller.kd_z,
        0.0,
    );

    //blimp.rail_init();

    let mut desired_altitude = 6.0;

    let mut desired_direction = 0.0;
    // The big event loop
    loop {
        let sensordat = blimp.update(state.clone(), state_timer.clone(), &mut save_image);
        let update_msg = BlimpToBaseMessage::SensorUpdate(sensordat.clone());
        if let Ok(payload) = serde_json::to_vec(&update_msg) {
            stats_socket
                .send_to(&payload, &base_station_control_addr)
                .ok();
        }

        let balls = vec![2, 3];
        let orange_goals = vec![5, 7];
        let yellow = vec![6, 8];

        // let det = match *state.lock().unwrap() {
        //     //BlimpStates::Ball => detection.detect(balls, &stats_socket, &mut save_image),
        //     BlimpStates::Ball => {
        //         desired_altitude = 4.0;
        //         // detection.detect(balls, &stats_socket, &mut save_image)
        //         detection.detect_bb(balls)
        //     }
        //     // TODO make the goals change read from the base station
        //     BlimpStates::Goal => {
        //         desired_altitude = 5.0;
        //         detection.detect_bb(orange_goals)
        //         // detection.detect(yellow, &stats_socket, &mut save_image)
        //     } //BlimpStates::Goal => detection.detect(orange_goals, &stats_socket, &mut save_image),
        // };
        //
        // Check for Commands
        let mut command_buffer = [0u8; 1024];
        match stats_socket.recv_from(&mut command_buffer) {
            Ok((size, src)) => {
                if size > 0 {
                    match serde_json::from_slice::<BaseToBlimpMessage>(&command_buffer[..size]) {
                        Ok(command) => match command {
                            BaseToBlimpMessage::UpdateConfig(config) => {
                                println!("Updating Gains");
                                auto.update_gains(
                                    config.kp_x,
                                    config.kp_yaw,
                                    config.kp_z,
                                    config.kd_x,
                                    config.kd_yaw,
                                    config.kd_z,
                                    0.0,
                                );
                            }

                            BaseToBlimpMessage::RequestVideo { target_port } => {
                                let target_ip = src.ip();
                                if target_port == conf.server.port {
                                    let target = SocketAddr::new(target_ip, target_port);
                                    let video_target_addr = Some(target);
                                    let ack_msg = BlimpToBaseMessage::AckVideo {
                                        streaming_to: target.to_string(),
                                    };
                                    if let Ok(payload) = serde_json::to_vec(&ack_msg) {
                                        println!("Sending video ack");
                                        stats_socket
                                            .send_to(&payload, &base_station_control_addr)
                                            .ok();
                                    }
                                } else {
                                    eprintln!(
                                        "[Sim:{}] Received video request for wrong port {}",
                                        blimp_id, target_port
                                    );
                                }
                            }
                        },
                        Err(e) => {
                            eprintln!();
                        }
                    }
                }
            }

            Err(e) => {}
        };
        //let current_direction = blimp.sensor.imu.euler_angles().unwrap();
        //println!("{:?}", current_direction);
        if blimp.is_manual() {
            // Manual control
            blimp.manual();
        } else {
            // Autonomous
            // if det.len() > 1 {
            //     // let auto_input = auto.position(-0.6, det[0] as f32, det[1] as f32);
            //     let auto_input = auto.position(-0.5, det[0] as f32, det[1] as f32);
            //     //println!("{:?}", auto_input);
            //     blimp.update_input(auto_input);
            //     let acc = blimp.mix();
            //     if !blimp.score {
            //         blimp.actuator.actuate(acc);
            //     }
            //     if *state.lock().unwrap() == BlimpStates::Goal {
            //         if det[2] > 200 || det[3] > 200 {
            //             blimp.score = true;
            //             blimp.score_time = std::time::Instant::now();
            //         }
            //     }
            //     time_p = std::time::Instant::now();
            // } else {
            //     if time_p.elapsed() > std::time::Duration::from_secs(2) {
            //         // TODO make it part of the config
            //         let altitude = blimp.sensor.get_altitude();
            //
            //         let z = match auto.altitude_hold(altitude, desired_altitude) {
            //             Ok(z) => z,
            //             Err(e) => 0.0,
            //         };
            //         // println!("Searching {z}");

            //let current_direction = blimp.sensor.imu.euler_angles().unwrap().c;
            // let current_direction = blimp.sensor.imu.quaternion().unwrap().v.x;
            // let current_direction = blimp.sensor.imu.mag_data().unwrap().x;
            // println!("{:?}", current_direction);

            // if search_timer.elapsed() > std::time::Duration::from_secs(10) {
            // desired_direction += 0.1;
            //    desired_altitude += 1.0;
            //    if desired_altitude >= 10.0 {
            //        desired_altitude = 5.0;
            //    }
            //    if desired_direction >= 9.0 {
            //        desired_direction *= -1.0;
            //    }
            //
            // search_timer = std::time::Instant::now();
            // }
            //
            // let y = auto.direction_hold(current_direction, desired_direction);
            //
            // println!(" current_direction: {:?}; Y: {:?}", current_direction, y);
            //
            // blimp.update_input((0.6, y, 0.0));
            //                    blimp.update_input((0.5, 0.4, z));

            //                   let acc = blimp.mix();
            //                  blimp.actuator.actuate(acc);
            //}
            // }
        }
    }
}
