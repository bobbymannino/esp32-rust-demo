mod accelerometer;
mod button;
mod email;
mod led;
mod reed;
mod rgb_led;
mod sh1107;
mod wifi;

use std::{thread, time::Duration};

use email::send_email;
use wifi::connect_wifi;

const SSID: &str = "11 Plym Street";
const PWD: &str = "AaaBbbccc1";

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    connect_wifi(SSID, PWD).expect("Failed to connect to Wi-Fi");
    send_email();

    loop {
        thread::sleep(Duration::from_millis(50));
    }
}
