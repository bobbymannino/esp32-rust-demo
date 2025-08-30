mod accelerometer;
mod button;
mod led;
mod reed;
mod rgb_led;

use crate::accelerometer::{AccelerationReading, Accelerometer, AccelerometerRange};
use esp_idf_svc::hal::prelude::*;
use std::{thread, time::Duration};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;
    let mut accel = Accelerometer::new(i2c, sda, scl, AccelerometerRange::Range16G).unwrap();

    loop {
        if accel.is_connected() {
            let AccelerationReading { x, y, z } = accel.read().unwrap();
            println!("Acceleration: ({}, {}, {})", x, y, z);
            println!("");
        } else {
            println!("Accelerometer not connected");
        }

        thread::sleep(Duration::from_millis(50));
    }
}
