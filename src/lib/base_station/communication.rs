//! Defines the data structures and enums used for communication
//! between the base station, blimps, and GUI components via UDP and channels.

// Make sure Serialize/Deserialize traits are imported when derive is used.
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;

// --- Data Structs for Messages ---
// NEW: Define possible blimp states
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlimpStates {
    Ball,
    Goal,
}

// NEW: Define the configuration parameters
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct BlimpConfig {
    // PD Gains
    pub kp_x: f32,
    pub kd_x: f32,
    pub kp_z: f32,
    pub kd_z: f32,
    pub kp_yaw: f32,
    pub kd_yaw: f32,
    // Motor related
    pub m1_multiplier: f32,
    pub m2_multiplier: f32,
    pub motor_neutral_angle: f32, // In degrees, perhaps?
}

impl Default for BlimpConfig {
    fn default() -> Self {
        // Provide some sensible defaults
        Self {
            kp_x: 1.0,
            kd_x: 0.1,
            kp_z: 1.0,
            kd_z: 0.1,
            kp_yaw: 1.0,
            kd_yaw: 0.1,
            m1_multiplier: 1.0,
            m2_multiplier: 1.0,
            motor_neutral_angle: 0.0,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct BlimpSensorData {
    pub battery: f32,
    pub altitude: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub tracking_error_x: f32,
    pub tracking_error_y: f32,
    // UPDATED: Include state reported by the blimp
    pub state: BlimpStates,
}

/// Messages sent from a Blimp to the Base Station Control Port.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum BlimpToBaseMessage {
    #[serde(rename = "connect")]
    Connect { id: String },
    #[serde(rename = "sensor_update")]
    SensorUpdate(BlimpSensorData),
    #[serde(rename = "ack_video")]
    AckVideo { streaming_to: String },
}

/// Messages sent from the Base Station Control Port to a Blimp.
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum BaseToBlimpMessage {
    #[serde(rename = "request_video")]
    RequestVideo {
        target_port: u16,
    },
    UpdateConfig(BlimpConfig),
}

//// --- Events for Backend Thread -> GUI Communication ---
//#[derive(Debug)]
//pub enum AppEvent {
//    BlimpConnected(SocketAddr, String), // addr, id
//    SensorUpdate(SocketAddr, BlimpSensorData),
//    VideoAck(SocketAddr, String),
//    BlimpDisconnected(SocketAddr),
//    Error(String),
//}
//
//// --- Commands for GUI -> Backend Communication ---
//#[derive(Debug)]
//pub enum GuiCommand {
//    RequestVideo(SocketAddr),
//}
