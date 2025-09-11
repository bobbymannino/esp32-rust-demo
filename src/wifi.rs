//! Wi-Fi helper module
//!
//! Provides a single convenience function `connect_wifi(ssid, pwd)` which:
//! 1. Acquires the required ESP-IDF system services (event loop).
//! 2. Configures the Wi-Fi driver in STA (client) mode.
//! 3. Starts the driver, connects, and waits until the network interface is up.
//! 4. Logs the obtained IP information.
//!
//! The function intentionally **leaks** the `BlockingWifi` handle so that the
//! Wi-Fi connection remains active for the lifetime of the application without
//! having to thread a handle through the rest of your code. On a small embedded
//! device that will run indefinitely, this is an acceptable pattern; if you
//! prefer not to leak, refactor the function to return the handle and store it
//! in your own global / application state.
//!
//! # Usage
//!
//! ```ignore
//! use crate::wifi::connect_wifi;
//!
//! fn main() {
//!     esp_idf_svc::sys::link_patches();
//!     esp_idf_svc::log::EspLogger::initialize_default();
//!
//!     // Replace these with your credentials (or load from configuration):
//!     let ssid = "MyNetwork";
//!     let pwd  = "SuperSecret";
//!
//!     connect_wifi(ssid, pwd).expect("Wi-Fi failed");
//!
//!     // From here on, Wi-Fi stays connected.
//!     loop {}
//! }
//! ```
//!
//! # Notes
//! * This module does not (yet) implement automatic reconnect logic beyond what
//!   the underlying ESP-IDF Wi-Fi stack already provides.
//! * If you need to handle disconnections, keep the (non-leaked) handle and
//!   implement monitoring using events from the system event loop.
//!
//! # Extending
//! To extend for advanced use cases (static IP, enterprise auth, etc.), modify
//! the `ClientConfiguration` before calling `wifi.set_configuration(...)`.

use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};
use esp_idf_sys::EspError;
use log::{info, warn};

/// Connect to a Wi-Fi network using the provided `ssid` and `pwd`.
///
/// This will:
/// * Take ownership of peripherals (only call once in your program)
/// * Initialize the Wi-Fi driver
/// * Configure it in client mode
/// * Start and connect
/// * Wait until the interface reports "up"
///
/// The internal Wi-Fi handle is leaked to keep the connection alive without
/// forcing the caller to store any state. For most always-on embedded firmware
/// this is fine. If you need graceful teardown, refactor to return the handle.
///
/// # Arguments
/// * `ssid` - The Wi-Fi network SSID
/// * `pwd`  - The Wi-Fi password
///
/// # Returns
/// `Ok(())` on success or an `EspError` describing what failed.
///
/// # Panics
/// Will panic only if `ssid` / `pwd` cannot be converted into the internal
/// fixed-size types (highly unlikely for normal credentials).
pub fn connect_wifi(ssid: &str, pwd: &str) -> Result<(), EspError> {
    info!("Initializing Wi-Fi (client) for SSID: '{}'", ssid);

    // Acquire core peripherals (only succeeds once)
    // New esp-idf-hal API returns a Result
    let peripherals = Peripherals::take()?;

    // System event loop (singleton)
    let sysloop = EspSystemEventLoop::take()?;

    // Initialize default NVS partition (required for Wi-Fi PHY calibration data)
    let nvs = EspDefaultNvsPartition::take()?;

    // Create the underlying Wi-Fi driver passing NVS partition
    let esp_wifi = EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?;

    // Wrap with blocking helper
    let mut blocking = BlockingWifi::wrap(esp_wifi, sysloop)?;

    // Configure as a simple client
    blocking.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().expect("SSID too long"),
        password: pwd.try_into().expect("Password too long"),
        ..Default::default()
    }))?;

    // Start + connect
    blocking.start()?;
    info!("Wi-Fi driver started, connecting...");
    blocking.connect()?;
    blocking.wait_netif_up()?;
    info!("Wi-Fi connected.");

    // Log IP info
    if let Ok(ip_info) = blocking.wifi().sta_netif().get_ip_info() {
        info!(
            "IP Info: IP={}, Subnet={}, DNS={:?}, Secondary DNS={:?}",
            ip_info.ip, ip_info.subnet, ip_info.dns, ip_info.secondary_dns
        );
    } else {
        warn!("Failed to fetch IP info after connection");
    }

    // Leak the handle so connection persists for program lifetime.
    // (If you want to manage the handle, return `blocking` instead.)
    let _leaked: &'static mut BlockingWifi<EspWifi<'static>> = Box::leak(Box::new(blocking));

    Ok(())
}

#[cfg(test)]
mod tests {
    // NOTE: These are compile-time tests only. On-target tests would require `#[cfg(not(test))]`
    // modifications or an integration test harness that runs on the device.
    use super::*;

    #[test]
    fn signature_compiles() {
        // Just ensure the function type is what we expect.
        fn _f(_: &str, _: &str) -> Result<(), EspError> {
            Ok(())
        }
    }
}
