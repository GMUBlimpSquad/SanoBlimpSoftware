use futures::stream::StreamExt;
use std::{env, io, str};
use tokio_util::codec::Framed;
use tokio_util::codec::{Decoder, Encoder};

use bytes::BytesMut;
use serde_json::Value;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc::UnboundedSender;
use tokio_serial::SerialPortBuilderExt;
use tokio_serial::SerialStream;

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

/// Detects objects
pub struct Detection {
    reader: Framed<SerialStream, LineCodec>,
}

impl Detection {
    pub fn new() -> Self {
        Detection {
            reader: LineCodec.framed(
                tokio_serial::new(DEFAULT_TTY, 921600)
                    .open_native_async()
                    .unwrap(),
            ),
        }
    }

    /// Probably add a channel into the parameter to send the detection from a different thread
    pub async fn detect(&mut self, tx: UnboundedSender<Vec<Value>>) {
        while let Some(line_result) = self.reader.next().await {
            let line = line_result.expect("Failed to read line");
            let value: Result<Value, _> = serde_json::from_str(&line);
            match value {
                Ok(value) => {
                    if value["type"] == 1 {
                        let data_field = &value["data"];
                        let boxes = data_field["boxes"].as_array();
                        //println!("{:?}", boxes);
                        tx.send(boxes.unwrap().to_vec());
                    }
                }
                Err(e) => {}
            }
        }
    }
}
