use esp_idf_svc::hal::{
    gpio::{PinDriver, Pull},
    prelude::Peripherals,
};
use std::{thread, time::Duration};

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut led_pin = PinDriver::output(peripherals.pins.gpio2).unwrap();

    let mut reed_pin = PinDriver::input(peripherals.pins.gpio23).unwrap();
    reed_pin.set_pull(Pull::Up).unwrap();

    loop {
        thread::sleep(Duration::from_millis(100));

        if reed_pin.is_low() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
    }
}
