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

use crate::sh1107::SH1107;
// Uncomment the following line to use the GY521 IMU module
// use crate::gy521::GY521;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        &I2cConfig::new().baudrate(400.kHz().into()),
    )
    .unwrap();

    let mut display = SH1107::new();
    let addr = 0x3c;

    display.set_px(2, 0, true);
    display.set_px(2, 1, true);
    display.set_px(2, 2, true);
    display.set_px(2, 3, true);
    display.set_px(2, 4, true);
    display.set_px(0, 4, true);
    display.set_px(1, 4, true);

    display.set_px(4, 0, true);
    display.set_px(5, 0, true);
    display.set_px(6, 0, true);
    display.set_px(7, 0, true);
    display.set_px(4, 2, true);
    display.set_px(5, 2, true);
    display.set_px(6, 2, true);
    display.set_px(7, 2, true);
    display.set_px(4, 1, true);
    display.set_px(7, 1, true);
    display.set_px(4, 3, true);
    display.set_px(7, 3, true);
    display.set_px(4, 4, true);
    display.set_px(7, 4, true);

    display.set_px(9, 0, true);
    display.set_px(12, 0, true);
    display.set_px(9, 1, true);
    display.set_px(10, 1, true);
    display.set_px(11, 1, true);
    display.set_px(9, 2, true);
    display.set_px(10, 2, true);
    display.set_px(11, 2, true);
    display.set_px(9, 3, true);
    display.set_px(12, 3, true);
    display.set_px(9, 4, true);
    display.set_px(13, 4, true);

    display.set_px(15, 0, true);
    display.set_px(16, 0, true);
    display.set_px(17, 0, true);
    display.set_px(18, 0, true);
    display.set_px(15, 1, true);
    display.set_px(15, 2, true);
    display.set_px(16, 2, true);
    display.set_px(17, 2, true);
    display.set_px(18, 2, true);
    display.set_px(15, 3, true);
    display.set_px(15, 4, true);
    display.set_px(16, 4, true);
    display.set_px(17, 4, true);
    display.set_px(18, 4, true);

    loop {
        thread::sleep(Duration::from_millis(50));

        display
            .flush(&mut i2c, addr)
            .expect("Failed to flush display");
    }

    /* Example usage of GY521 IMU module - uncomment to use:
    
    // Note: You need to uncomment the GY521 import at the top of this file
    // and use a different I2C peripheral or different pins than the display
    
    // Initialize GY521
    let mut gy521 = GY521::new(
        peripherals.i2c1,  // Use a different I2C peripheral if available
        peripherals.pins.gpio18,  // SDA
        peripherals.pins.gpio19,  // SCL
    ).unwrap();

    loop {
        // Read rotation, movement, and temperature
        match (gy521.read_rotation(), gy521.read_movement(), gy521.read_temperature()) {
            (Ok(rotation), Ok(movement), Ok(temp)) => {
                info!("Rotation - X: {:.2}°/s, Y: {:.2}°/s, Z: {:.2}°/s", 
                      rotation.x, rotation.y, rotation.z);
                info!("Movement - X: {:.2}g, Y: {:.2}g, Z: {:.2}g", 
                      movement.x, movement.y, movement.z);
                info!("Temperature: {:.1}°C", temp);
            }
            _ => {
                info!("Error reading GY521 sensor data");
            }
        }
        
        thread::sleep(Duration::from_millis(500));
    }
    */
}
