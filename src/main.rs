mod button;
mod led;
mod reed;
mod rgb_led;

use crate::{button::Button, rgb_led::RgbLed};
use esp_idf_svc::hal::prelude::*;

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
