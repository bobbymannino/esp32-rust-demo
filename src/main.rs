use esp_idf_svc::hal::{gpio::PinDriver, prelude::Peripherals};
use std::{thread, time::Duration};

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut pin = PinDriver::output(peripherals.pins.gpio2).unwrap();
    pin.set_high().unwrap();

    loop {
        thread::sleep(Duration::from_millis(100));
        pin.toggle().unwrap();
    }
}
