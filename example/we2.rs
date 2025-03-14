use futures::stream::StreamExt;
use std::{env, io, str};
use tokio_util::codec::{Decoder, Encoder};

use bytes::BytesMut;
use serde_json::Value;
use tokio_serial::SerialPortBuilderExt;

#[cfg(unix)]
const DEFAULT_TTY: &str = "/dev/ttyACM0";
#[cfg(windows)]
const DEFAULT_TTY: &str = "COM1";

struct LineCodec;

impl Decoder for LineCodec {
    type Item = String;
    type Error = io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if let Some(n) = src.as_ref().iter().position(|b| *b == b'\n') {
            let line = src.split_to(n + 1);
            return match str::from_utf8(line.as_ref()) {
                Ok(s) => Ok(Some(s.to_string())),
                Err(_) => Err(io::Error::new(io::ErrorKind::Other, "Invalid String")),
            };
        }
        Ok(None)
    }
}

impl Encoder<String> for LineCodec {
    type Error = io::Error;

    fn encode(&mut self, _item: String, _dst: &mut BytesMut) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    let mut args = env::args();
    let tty_path = args.nth(1).unwrap_or_else(|| DEFAULT_TTY.into());

    // Uncomment this line if you wish to log the destination.
    // println!("Streaming annotated images from serial to UDP at {}", udp_dest);

    let mut port = tokio_serial::new(tty_path, 921600).open_native_async()?;

    #[cfg(unix)]
    port.set_exclusive(false)
        .expect("Unable to set serial port exclusive to false");

    let mut reader = LineCodec.framed(port);

    while let Some(line_result) = reader.next().await {
        let line = line_result.expect("Failed to read line");
        let value: Result<Value, _> = serde_json::from_str(&line);
        match value {
            Ok(value) => {
                if value["type"] == 1 {
                    let data_field = &value["data"];
                    let boxes = data_field["boxes"].as_array();
                    println!("{:?}", boxes);
                }
            }
            Err(e) => {
                // eprintln!("Error parsing JSON: {:?}", e);
            }
        }
    }
    Ok(())
}
