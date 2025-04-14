//! Defines the data structures and enums used for communication
//! between the base station, blimps, and GUI components via UDP and channels.

// Make sure Serialize/Deserialize traits are imported when derive is used.
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;

// --- Data Structs for Messages ---

/// Represents PID controller gains (Currently unused in comms, placeholder).
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BlimpPidConfig {
    pub kp_pitch: f32,
    pub ki_pitch: f32,
    pub kd_pitch: f32,
    pub kp_roll: f32,
    pub ki_roll: f32,
    pub kd_roll: f32,
}

/// Represents sensor data sent periodically by a blimp.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BlimpSensorData {
    pub battery: f32,
    pub altitude: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub tracking_error_x: f32,
    pub tracking_error_y: f32,
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
    RequestVideo { target_port: u16 },
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
