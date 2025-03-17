use tokio::{
    spawn,
    sync::{mpsc::unbounded_channel, Notify},
};
mod lib;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use std::sync::{Arc, Mutex};

#[tokio::main]
async fn main() {
    let mut blimp = Arc::new(Mutex::new(blimp::SanoBlimp::new()));
    let mut detection = Detection::new();

    let autonomous = Arc::new(Mutex::new(false));
    let notifier = Arc::new(Notify::new());

    let (tx, mut rx_detection) = unbounded_channel();

    let autonomous_clone = Arc::clone(&autonomous);

    // Spawn object detection task
    tokio::spawn(async move {
        detection.detect(tx).await;
    });

    let blimp_clone = blimp.clone();

    // Run blimp in a separate task if `run()` is blocking
    tokio::spawn(async move {
        loop {
            blimp_clone.lock().unwrap().run();
        }
    });

    let blimp_clone_autonomous = blimp.clone();
    // Process the detection and update input with the given actuations
    while let Some(det) = rx_detection.recv().await {
        // Run autonomous code

        // IMPLEMENT PID LOGIC HERE

        blimp_clone_autonomous
            .lock()
            .unwrap()
            .update_input((0.2, 0.0, 0.0));
    }

    // Ensure the object detection worker finishes
}
