mod accelerometer;
mod button;
mod led;
mod reed;
mod rgb_led;

use crate::accelerometer::{AccelerationReading, Accelerometer, AccelerometerRange};
use esp_idf_svc::hal::prelude::*;
use std::{thread, time::{Duration, Instant}};

// Autosleep configuration constants
const MOTION_THRESHOLD_G: f32 = 1.5;  // Wake up threshold in g-force
const INACTIVITY_TIMEOUT_MS: u64 = 5000;  // Sleep after 5 seconds of low activity
const SLEEP_CHECK_INTERVAL_MS: u64 = 1000;  // Check for motion every 1 second while sleeping
const ACTIVE_SAMPLE_INTERVAL_MS: u64 = 50;  // Sample rate when actively monitoring

#[derive(Debug, PartialEq)]
enum AccelerometerState {
    Awake,
    Sleeping,
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;
    let mut accel = Accelerometer::new(i2c, sda, scl, AccelerometerRange::Range16G).unwrap();

    let mut state = AccelerometerState::Awake;
    let mut last_activity_time = Instant::now();
    
    println!("Starting accelerometer with autosleep (threshold: {}g)", MOTION_THRESHOLD_G);

    loop {
        if !accel.is_connected() {
            println!("Accelerometer not connected");
            thread::sleep(Duration::from_millis(1000));
            continue;
        }

        match state {
            AccelerometerState::Awake => {
                // Continuously monitor acceleration when awake
                if let Ok(reading) = accel.read() {
                    let magnitude = reading.magnitude();
                    
                    if magnitude > MOTION_THRESHOLD_G {
                        // Motion detected above threshold - stay awake and reset timer
                        println!("Motion detected: {:.3}g (above threshold)", magnitude);
                        last_activity_time = Instant::now();
                    } else if magnitude > 1.0 {
                        // Some motion but below threshold - still log it
                        println!("Low motion: {:.3}g", magnitude);
                    }
                    
                    // Check if we should go to sleep due to inactivity
                    if last_activity_time.elapsed().as_millis() > INACTIVITY_TIMEOUT_MS as u128 {
                        println!("No significant motion for {}ms, putting accelerometer to sleep", INACTIVITY_TIMEOUT_MS);
                        if let Err(e) = accel.sleep() {
                            println!("Failed to put accelerometer to sleep: {:?}", e);
                        } else {
                            state = AccelerometerState::Sleeping;
                            println!("Accelerometer now sleeping - will check for motion every {}ms", SLEEP_CHECK_INTERVAL_MS);
                        }
                    }
                }
                
                thread::sleep(Duration::from_millis(ACTIVE_SAMPLE_INTERVAL_MS));
            }
            
            AccelerometerState::Sleeping => {
                // Periodically wake up to check for motion
                if let Err(e) = accel.wake() {
                    println!("Failed to wake accelerometer: {:?}", e);
                    thread::sleep(Duration::from_millis(SLEEP_CHECK_INTERVAL_MS));
                    continue;
                }
                
                // Give device time to wake up and take a measurement
                thread::sleep(Duration::from_millis(10));
                
                if let Ok(reading) = accel.read() {
                    let magnitude = reading.magnitude();
                    
                    if magnitude > MOTION_THRESHOLD_G {
                        // Motion above threshold - stay awake
                        println!("Wake up! Motion detected: {:.3}g (above threshold)", magnitude);
                        last_activity_time = Instant::now();
                        state = AccelerometerState::Awake;
                    } else {
                        // No significant motion - go back to sleep
                        if let Err(e) = accel.sleep() {
                            println!("Failed to put accelerometer back to sleep: {:?}", e);
                        }
                    }
                }
                
                if state == AccelerometerState::Sleeping {
                    thread::sleep(Duration::from_millis(SLEEP_CHECK_INTERVAL_MS - 10));
                }
            }
        }
    }
}
