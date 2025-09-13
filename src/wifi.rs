use esp_idf_hal::modem::Modem;
use esp_idf_hal::task::block_on;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{AsyncWifi, AuthMethod};
use esp_idf_svc::{
    nvs::EspDefaultNvsPartition,
    wifi::{ClientConfiguration, Configuration, EspWifi},
};
use log::info;

pub fn connect_wifi(modem: Modem, ssid: &str, pwd: &str) -> anyhow::Result<()> {
    info!("connecting to wifi: {}", ssid);

    info!("setting up system event loop");
    let sys_loop = EspSystemEventLoop::take().expect("failed to setup sys_loop");

    info!("setting up task timer service");
    let timer_svc = EspTaskTimerService::new().expect("failed to setup timer_svc");

    info!("setting up nvs partition");
    let nvs = EspDefaultNvsPartition::take().expect("failed to setup nvs");

    let async_wifi_config = EspWifi::new(modem, sys_loop.clone(), Some(nvs))
        .expect("failed to create AsyncWifi config");

    let mut wifi = AsyncWifi::wrap(async_wifi_config, sys_loop, timer_svc)
        .expect("failed to create AsyncWifi");

    block_on(connect_ssid(&mut wifi, &ssid, &pwd))?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {ip_info:?}");

    info!("Shutting down in 5s...");

    std::thread::sleep(core::time::Duration::from_secs(5));

    Ok(())
}

async fn connect_ssid(
    wifi: &mut AsyncWifi<EspWifi<'static>>,
    ssid: &str,
    pwd: &str,
) -> anyhow::Result<()> {
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: pwd.try_into().unwrap(),
        channel: None,
        ..Default::default()
    });

    info!("Setting wifi config");
    wifi.set_configuration(&wifi_configuration)
        .expect("failed to set wifi config");

    info!("Starting wifi");
    wifi.start().await.expect("failed to start wifi");
    info!("Wifi started");

    info!("Connecting to wifi");
    wifi.connect().await.expect("failed to connect to wifi");
    info!("Wifi connected");

    info!("Waiting for wifi netif to be up");
    wifi.wait_netif_up()
        .await
        .expect("failed to wait for wifi netif to be up");
    info!("Wifi netif up");

    Ok(())
}
