mod accelerometer;
mod button;
mod gy521;
mod led;
mod reed;
mod rgb_led;
mod sh1107;

use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use log::info;
use std::{thread, time::Duration};

use crate::gy521::GY521;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // Initialize GY521
    let mut gy521 = GY521::new(
        peripherals.i2c1,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
    )
    .expect("Failed to init i2c");

    info!("i2c setup");

    loop {
        if let Ok(reading) = gy521.read_movement() {
            info!(
                "Movement - X: {:.2}g, Y: {:.2}g, Z: {:.2}g",
                reading.x, reading.y, reading.z
            );
        } else {
            info!("Error reading movement data");
        }

        // Read rotation and temperature
        if let Ok(rotation) = gy521.read_rotation() {
            info!(
                "Rotation - X: {:.2}°/s, Y: {:.2}°/s, Z: {:.2}°/s",
                rotation.x, rotation.y, rotation.z
            );
        } else {
            info!("Error reading rotation data");
        }

        if let Ok(temp) = gy521.read_temperature() {
            info!("Temperature: {:.1}°C", temp);
        } else {
            info!("Error reading temperature data");
        }

        thread::sleep(Duration::from_millis(500));
    }
}
