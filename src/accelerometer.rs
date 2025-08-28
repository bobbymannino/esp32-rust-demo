//! # ADXL345 Accelerometer Driver for ESP32
//!
//! This module provides a clean, object-oriented interface for the ADXL345 3-axis accelerometer.
//! 
//! ## Key Improvements from Original Implementation:
//! - **Struct-based API**: Replaced functional approach with proper struct and methods
//! - **Clean measurements**: `AccelerationReading` struct with helper methods like `magnitude()`
//! - **Power management**: Added `sleep()` and `wake()` methods for power conservation
//! - **Proper error handling**: Uses `Result<T, EspError>` throughout
//! - **Comprehensive documentation**: Detailed comments and usage examples
//! - **Reusable design**: No more infinite loops, allows integration with other components
//!
//! ## Usage
//! ```rust
//! let mut accel = Accelerometer::new(i2c_peripheral, sda_pin, scl_pin)?;
//! let reading = accel.read()?;
//! println!("Acceleration: {:.3}g", reading.magnitude());
//! ```

use esp_idf_svc::hal::{
    delay::BLOCK,
    i2c::{I2cConfig, I2cDriver},
    prelude::Peripherals,
    units::Hertz,
};
use esp_idf_sys::EspError;
use std::{thread, time::Duration};

/// ADXL345 3-axis accelerometer I2C address
/// Found in the I2C section of the ADXL345 datasheet:
/// https://analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
const ADXL345_ADDR: u8 = 0x53;

/// ADXL345 register addresses and power control values
const REG_POWER_CTL: u8 = 0x2d;     // Power control register
const POWER_CTL_WAKEUP: u8 = 0x2;   // Wake up the device (low power mode)
const POWER_CTL_SLEEP: u8 = 0x4;    // Put device in sleep mode
const POWER_CTL_MEASURE: u8 = 0x08; // Put device in measurement mode

/// Data register addresses for X, Y, Z axis readings
/// Each axis uses 2 bytes (LSB first, then MSB)
const REG_DATAX0: u8 = 0x32;  // X-axis data LSB
const REG_DATAY0: u8 = 0x34;  // Y-axis data LSB  
const REG_DATAZ0: u8 = 0x36;  // Z-axis data LSB

/// Acceleration measurement in g-force units
#[derive(Debug, Clone, Copy)]
pub struct AccelerationReading {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl AccelerationReading {
    /// Create a new acceleration reading
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    
    /// Get the magnitude of the acceleration vector
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// ADXL345 accelerometer driver for ESP32
/// 
/// # Hardware Connections
/// * 3.3V and GND are connected between ESP32 and ADXL345
/// * SDA (data line) on ADXL345 connects to GPIO21 on ESP32
/// * SCL (clock line) on ADXL345 connects to GPIO22 on ESP32
///
/// # Usage Examples
/// 
/// ```rust
/// // Basic usage
/// let peripherals = Peripherals::take().unwrap();
/// let mut accel = Accelerometer::new(
///     peripherals.i2c0,
///     peripherals.pins.gpio21, // SDA
///     peripherals.pins.gpio22, // SCL
/// ).unwrap();
/// 
/// // Read acceleration
/// let reading = accel.read().unwrap();
/// println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
/// 
/// // Power management
/// accel.sleep().unwrap();   // Put in sleep mode to save power
/// accel.wake().unwrap();    // Wake up for measurements
/// ```
pub struct Accelerometer<'a> {
    i2c: I2cDriver<'a>,
}

impl<'a> Accelerometer<'a> {
    /// Create a new accelerometer instance
    /// 
    /// # Arguments
    /// * `i2c_peripheral` - The I2C peripheral to use (typically i2c0)
    /// * `sda_pin` - The SDA (data) pin (typically GPIO21)
    /// * `scl_pin` - The SCL (clock) pin (typically GPIO22)
    /// 
    /// # Returns
    /// Result containing the accelerometer instance or an error
    pub fn new<SDA, SCL>(
        i2c_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::i2c::I2c> + 'a,
        sda_pin: SDA,
        scl_pin: SCL,
    ) -> Result<Self, EspError>
    where
        SDA: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCL: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
    {
        // Configure I2C with 400kHz bus speed (standard for ADXL345)
        let i2c_cfg = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c = I2cDriver::new(i2c_peripheral, sda_pin, scl_pin, &i2c_cfg)?;

        let mut accelerometer = Self { i2c };

        // Initialize the accelerometer in measurement mode
        accelerometer.wake()?;

        Ok(accelerometer)
    }

    /// Read acceleration data from all three axes
    /// 
    /// Returns acceleration values in units of g-force (1g = 9.8 m/s²).
    /// The ADXL345 default range is ±2g with a resolution of approximately 0.004g per LSB.
    /// 
    /// ## Scale Factor Notes
    /// The scale factor (1/256) assumes the default ±2g range setting. If you configure
    /// the accelerometer for different ranges (±4g, ±8g, ±16g), you'll need to adjust
    /// this scale factor accordingly:
    /// - ±2g range: 1/256 (default)
    /// - ±4g range: 1/128 
    /// - ±8g range: 1/64
    /// - ±16g range: 1/32
    /// 
    /// # Returns
    /// Result containing AccelerationReading or an error
    pub fn read(&mut self) -> Result<AccelerationReading, EspError> {
        let mut x_data: [u8; 2] = [0; 2];
        let mut y_data: [u8; 2] = [0; 2]; 
        let mut z_data: [u8; 2] = [0; 2];

        // Read 2 bytes for each axis (LSB first, then MSB)
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAX0], &mut x_data, BLOCK)?;
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAY0], &mut y_data, BLOCK)?;
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAZ0], &mut z_data, BLOCK)?;

        // Convert little-endian bytes to signed 16-bit integers
        let x_raw = i16::from_le_bytes(x_data);
        let y_raw = i16::from_le_bytes(y_data);
        let z_raw = i16::from_le_bytes(z_data);

        // Convert to g-force units (assuming ±2g range, 256 LSB/g scale factor)
        // For other ranges, this scale factor would need to be adjusted
        let scale_factor = 1.0 / 256.0;
        let x = x_raw as f32 * scale_factor;
        let y = y_raw as f32 * scale_factor;
        let z = z_raw as f32 * scale_factor;

        Ok(AccelerationReading::new(x, y, z))
    }

    /// Read raw acceleration data without scale conversion
    /// 
    /// Returns the raw 16-bit signed integers from the accelerometer registers.
    /// This is useful if you want to apply custom scaling or processing.
    /// 
    /// # Returns
    /// Result containing (x_raw, y_raw, z_raw) as i16 values or an error
    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), EspError> {
        let mut x_data: [u8; 2] = [0; 2];
        let mut y_data: [u8; 2] = [0; 2]; 
        let mut z_data: [u8; 2] = [0; 2];

        // Read 2 bytes for each axis (LSB first, then MSB)
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAX0], &mut x_data, BLOCK)?;
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAY0], &mut y_data, BLOCK)?;
        self.i2c.write_read(ADXL345_ADDR, &[REG_DATAZ0], &mut z_data, BLOCK)?;

        // Convert little-endian bytes to signed 16-bit integers
        let x_raw = i16::from_le_bytes(x_data);
        let y_raw = i16::from_le_bytes(y_data);
        let z_raw = i16::from_le_bytes(z_data);

        Ok((x_raw, y_raw, z_raw))
    }

    /// Put the accelerometer into sleep mode to conserve power
    /// 
    /// In sleep mode, the device consumes much less power but cannot take measurements.
    /// Call wake() to resume normal operation.
    /// 
    /// # Usage
    /// ```rust
    /// // Put accelerometer to sleep when not in use
    /// accel.sleep().unwrap();
    /// // ... device sleeps, consuming minimal power ...
    /// accel.wake().unwrap();  // Resume measurements
    /// ```
    pub fn sleep(&mut self) -> Result<(), EspError> {
        // Set the power control register to sleep mode
        self.i2c.write(ADXL345_ADDR, &[REG_POWER_CTL, POWER_CTL_SLEEP], BLOCK)?;
        Ok(())
    }

    /// Wake the accelerometer from sleep mode and enable measurements
    /// 
    /// This puts the device in measurement mode where it actively measures acceleration.
    /// This is the normal operating mode for taking readings.
    /// 
    /// # Usage
    /// ```rust
    /// // Wake up the accelerometer for measurements
    /// accel.wake().unwrap();
    /// let reading = accel.read().unwrap();
    /// ```
    pub fn wake(&mut self) -> Result<(), EspError> {
        // Set the power control register to measurement mode
        self.i2c.write(ADXL345_ADDR, &[REG_POWER_CTL, POWER_CTL_MEASURE], BLOCK)?;
        Ok(())
    }

    /// Check if the accelerometer is responding to I2C commands
    /// 
    /// This can be used to verify the device is properly connected and functioning.
    /// Note: This will wake the device if it's sleeping.
    pub fn is_connected(&mut self) -> bool {
        // Try to read the power control register
        let mut data = [0u8; 1];
        self.i2c.write_read(ADXL345_ADDR, &[REG_POWER_CTL], &mut data, BLOCK).is_ok()
    }
}

/// Convenience function for simple accelerometer setup and continuous reading
/// 
/// This provides a simple interface similar to the original setup() function
/// but allows for more flexible usage patterns.
/// 
/// # Example
/// ```rust
/// use accelerometer::run_continuous_reading;
/// 
/// // This will run indefinitely, printing accelerometer readings
/// run_continuous_reading().unwrap();
/// ```
pub fn run_continuous_reading() -> Result<(), EspError> {
    let peripherals = Peripherals::take().unwrap();

    let mut accel = Accelerometer::new(
        peripherals.i2c0,
        peripherals.pins.gpio21, // SDA
        peripherals.pins.gpio22, // SCL
    )?;

    loop {
        match accel.read() {
            Ok(reading) => {
                println!("Acceleration - X: {:.3}g, Y: {:.3}g, Z: {:.3}g (magnitude: {:.3}g)", 
                         reading.x, reading.y, reading.z, reading.magnitude());
            }
            Err(e) => {
                println!("Error reading accelerometer: {:?}", e);
            }
        }

        // Slow down readings to avoid overwhelming the output
        thread::sleep(Duration::from_millis(50));
    }
}
