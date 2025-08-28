mod accelerometer;
mod button;
mod led;
mod reed;
mod rgb_led;

use crate::{button::Button, rgb_led::RgbLed};
use esp_idf_svc::hal::prelude::*;

/*
   Example usage of the accelerometer module:

   // Basic accelerometer usage:
   use crate::accelerometer::{Accelerometer, run_continuous_reading};

   fn use_accelerometer() {
       let peripherals = Peripherals::take().unwrap();
       
       // Create accelerometer instance
       let mut accel = Accelerometer::new(
           peripherals.i2c0,
           peripherals.pins.gpio21, // SDA
           peripherals.pins.gpio22, // SCL
       ).unwrap();
       
       // Read scaled measurement in g-force
       let reading = accel.read().unwrap();
       println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
       println!("Magnitude: {:.3}g", reading.magnitude());
       
       // Read raw values for custom processing
       let (x_raw, y_raw, z_raw) = accel.read_raw().unwrap();
       println!("Raw values - X: {}, Y: {}, Z: {}", x_raw, y_raw, z_raw);
       
       // Power management
       accel.sleep().unwrap();  // Save power when not needed
       accel.wake().unwrap();   // Resume measurements
       
       // Check connection
       if accel.is_connected() {
           println!("Accelerometer is connected!");
       }
   }
   
   // For continuous reading (replaces the button/LED demo):
   // fn main() {
   //     esp_idf_svc::sys::link_patches();
   //     esp_idf_svc::log::EspLogger::initialize_default();
   //     run_continuous_reading().unwrap();
   // }
*/

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let button = Button::new(peripherals.pins.gpio19).unwrap();

    let mut rgb_led = RgbLed::new(
        peripherals.pins.gpio16,
        peripherals.pins.gpio5,
        peripherals.pins.gpio17,
    )
    .unwrap();

    loop {
        if button.is_pressed() {
            rgb_led.blue().unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            rgb_led.red().unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            rgb_led.green().unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            rgb_led.on().unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
        } else {
            rgb_led.off().unwrap();
        }
    }
}
