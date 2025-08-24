use std::time::Duration;

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::prelude::Peripherals,
    nvs::EspDefaultNvsPartition,
    wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi, PmfConfiguration, ScanMethod},
};
use log::info;

const SSID: &str = "Phone";
const PWD: &str = "password";

pub fn connect_to_wifi() {
    info!("Connecting to WiFi");

    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    let mut wifi = EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs)).unwrap();

    let client_config = ClientConfiguration {
        auth_method: AuthMethod::WPA2Personal,
        ssid: SSID.parse().unwrap(),
        password: PWD.parse().unwrap(),
        bssid: None,
        channel: None,
        scan_method: ScanMethod::FastScan,
        pmf_cfg: PmfConfiguration::default(),
    };

    let wifi_configuration = Configuration::Client(client_config);

    wifi.set_configuration(&wifi_configuration).unwrap();

    wifi.start().unwrap();
    info!("Wifi started");

    wifi.connect().unwrap();
    info!("Wifi connected");

    loop {
        // get a GET request response from 153.92.211.217:5461/time
        // make a get request to a dummyjson api like

        std::thread::sleep(Duration::from_millis(500));
    }

    // use std::net::Ipv4Addr;

    // info!("Waiting for DHCP lease...");
    // let mut ip_info = wifi.wifi().sta_netif().get_ip_info().unwrap();
    // while ip_info.ip == Ipv4Addr::new(0, 0, 0, 0) {
    //     std::thread::sleep(std::time::Duration::from_millis(500));
    //     ip_info = wifi.wifi().sta_netif().get_ip_info().unwrap();
    // }

    // info!("WiFi connected successfully");
}
