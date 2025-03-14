use tokio::spawn;
//
mod lib;
use lib::blimp::{self, Blimp};
use lib::object_detection::Detection;
use std::sync::Arc;

#[tokio::main]
async fn main() {
    let mut blimp = blimp::SanoBlimp::new();

    loop {
        blimp.run();
    }
}
