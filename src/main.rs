mod button;
mod led;
mod reed;

use crate::{button::Button, led::Led, reed::Reed};
use esp_idf_svc::hal::prelude::*;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut led = Led::new(peripherals.pins.gpio18).unwrap();
    let reed = Reed::new(peripherals.pins.gpio23).unwrap();
    let button = Button::new(peripherals.pins.gpio15).unwrap();

    loop {
        if reed.is_on() || button.is_pressed() {
            led.on().unwrap();
        } else {
            led.off().unwrap();
        }
    }
}
