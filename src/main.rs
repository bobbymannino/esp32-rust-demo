mod wifi;

use esp_idf_svc::hal::{
    gpio::{PinDriver, Pull},
    prelude::Peripherals,
};
mod accelerometer;
mod button;
mod led;
mod reed;
mod rgb_led;
mod sh1107;

use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use log::info;
use std::{thread, time::Duration};

use crate::sh1107::SH1107;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    connect_wifi(SSID, PWD).expect("Failed to connect Wi-Fi");
    send_email();

    loop {
        thread::sleep(Duration::from_millis(50));

        display
            .flush(&mut i2c, addr)
            .expect("Failed to flush display");
    }
}
