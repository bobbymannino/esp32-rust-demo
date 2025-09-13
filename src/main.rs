mod accelerometer;
mod button;
mod email;
mod gy521;
mod led;
mod reed;
mod rgb_led;
mod sh1107;
mod wifi;

use log::info;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Hello there");
}
