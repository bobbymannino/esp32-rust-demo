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
}
