mod accelerometer;
mod button;
mod led;
mod reed;
mod rgb_led;
mod test_accelerometer_api;

use crate::{button::Button, rgb_led::RgbLed};
use esp_idf_svc::hal::prelude::*;

/*
   Example usage of the accelerometer module with both I2C and SPI:

   // Basic accelerometer usage with I2C:
   use crate::accelerometer::{Accelerometer, AccelerometerRange, run_continuous_reading, run_continuous_reading_spi};

   fn use_accelerometer_i2c() {
       let peripherals = Peripherals::take().unwrap();
       
       // Create I2C accelerometer instance with default ±2g range
       let mut accel = Accelerometer::new(
           peripherals.i2c0,
           peripherals.pins.gpio21, // SDA
           peripherals.pins.gpio22, // SCL
           AccelerometerRange::Range2G, // ±2g range for highest resolution
       ).unwrap();
       
       // Or create with ±16g range for high acceleration measurements
       let mut accel_high_range = Accelerometer::new(
           peripherals.i2c0,
           peripherals.pins.gpio21, // SDA
           peripherals.pins.gpio22, // SCL
           AccelerometerRange::Range16G, // ±16g range for impacts/vibrations
       ).unwrap();
       
       let reading = accel.read().unwrap();
       println!("I2C - X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
   }

   fn use_accelerometer_spi() {
       let peripherals = Peripherals::take().unwrap();
       
       // Create SPI accelerometer instance with default ±2g range
       let mut accel = Accelerometer::new_spi(
           peripherals.spi2,
           peripherals.pins.gpio23, // MOSI
           peripherals.pins.gpio19, // MISO
           peripherals.pins.gpio18, // SCLK
           peripherals.pins.gpio5,  // CS
       ).unwrap();
       
       // Or create with ±16g range for high acceleration measurements
       let mut accel_high_range = Accelerometer::new_spi_with_range(
           peripherals.spi2,
           peripherals.pins.gpio23, // MOSI
           peripherals.pins.gpio19, // MISO
           peripherals.pins.gpio18, // SCLK
           peripherals.pins.gpio5,  // CS
           AccelerometerRange::Range16G, // ±16g range for impacts/vibrations
       ).unwrap();
       
       let reading = accel.read().unwrap();
       println!("SPI - X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
       
       // Both I2C and SPI accelerometers support the same methods:
       // - read() for scaled measurements
       // - read_raw() for raw values
       // - sleep() and wake() for power management
       // - is_connected() to check connection
   }
   
   // For continuous reading (replaces the button/LED demo):
   // fn main() {
   //     esp_idf_svc::sys::link_patches();
   //     esp_idf_svc::log::EspLogger::initialize_default();
   //     
   //     // Use I2C interface
   //     run_continuous_reading().unwrap();
   //     
   //     // Or use SPI interface
   //     // run_continuous_reading_spi().unwrap();
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
