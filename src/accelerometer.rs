//! # ADXL345 Accelerometer Driver for ESP32
//!
//! This module provides a clean, object-oriented interface for the ADXL345 3-axis accelerometer.
//! Supports both I2C and SPI communication protocols.
//!
//! ## Key Improvements from Original Implementation:
//! - **Struct-based API**: Replaced functional approach with proper struct and methods
//! - **Dual Interface Support**: Works with both I2C and SPI communication
//! - **Clean measurements**: `AccelerationReading` struct with helper methods like `magnitude()`
//! - **Power management**: Added `sleep()` and `wake()` methods for power conservation
//! - **Proper error handling**: Uses `Result<T, EspError>` throughout
//! - **Comprehensive documentation**: Detailed comments and usage examples
//! - **Reusable design**: No more infinite loops, allows integration with other components
//!
//! ## Usage
//! ```rust
//! // I2C Example - Default ±2g range for highest resolution
//! let mut accel = Accelerometer::new(i2c_peripheral, sda_pin, scl_pin, AccelerometerRange::Range2G)?;
//! let reading = accel.read()?;
//! println!("Acceleration: {:.3}g", reading.magnitude());
//!
//! // SPI Example - ±16g range for high acceleration measurements  
//! let spi_driver = SpiDriver::new(peripherals.spi2, sclk, mosi, Some(miso), &SpiDriverConfig::new())?;
//! let mut accel = Accelerometer::new_spi(&spi_driver, cs_pin, AccelerometerRange::Range16G)?;
//! ```

use esp_idf_svc::hal::{
    delay::BLOCK,
    i2c::{I2cConfig, I2cDriver},
    prelude::Peripherals,
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    units::Hertz,
};
use esp_idf_sys::EspError;
use std::{thread, time::Duration};

/// ADXL345 3-axis accelerometer I2C address
/// Found in the I2C section of the ADXL345 datasheet:
/// https://analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
const ADXL345_ADDR: u8 = 0x53;

/// SPI register operation bits
/// In SPI mode, the MSB of the first byte indicates read (1) or write (0)
/// The second MSB (bit 6) indicates multi-byte operation when set
const SPI_READ_BIT: u8 = 0x80;   // Set bit 7 for read operations
const SPI_MULTI_BYTE_BIT: u8 = 0x40; // Set bit 6 for multi-byte reads

/// ADXL345 register addresses and power control values
const REG_POWER_CTL: u8 = 0x2d; // Power control register
const POWER_CTL_WAKEUP: u8 = 0x2; // Wake up the device (low power mode)
const POWER_CTL_SLEEP: u8 = 0x4; // Put device in sleep mode
const POWER_CTL_MEASURE: u8 = 0x08; // Put device in measurement mode

/// Data format register for configuring measurement range
const REG_DATA_FORMAT: u8 = 0x31; // Data format control register

/// Data register addresses for X, Y, Z axis readings
/// Each axis uses 2 bytes (LSB first, then MSB)
const REG_DATAX0: u8 = 0x32; // X-axis data LSB
const REG_DATAY0: u8 = 0x34; // Y-axis data LSB
const REG_DATAZ0: u8 = 0x36; // Z-axis data LSB

/// Communication interface for the accelerometer
/// 
/// The ADXL345 supports both I2C and SPI communication protocols.
/// This enum allows the driver to work with either interface.
#[derive(Debug)]
pub enum CommInterface<'a> {
    /// I2C communication using I2cDriver
    I2c(I2cDriver<'a>),
    /// SPI communication using SpiDeviceDriver  
    Spi(SpiDeviceDriver<'a, &'a SpiDriver<'a>>),
}

/// Accelerometer measurement range configuration
///
/// The ADXL345 supports different measurement ranges with corresponding scale factors.
/// Higher ranges can measure larger accelerations but with lower resolution.
#[derive(Debug, Clone, Copy)]
pub enum AccelerometerRange {
    /// ±2g range with highest resolution (256 LSB/g)
    Range2G,
    /// ±4g range (128 LSB/g)
    Range4G,
    /// ±8g range (64 LSB/g)
    Range8G,
    /// ±16g range with lowest resolution (32 LSB/g)
    Range16G,
}

impl AccelerometerRange {
    /// Get the scale factor for converting raw readings to g-force
    pub fn scale_factor(&self) -> f32 {
        match self {
            AccelerometerRange::Range2G => 1.0 / 256.0,
            AccelerometerRange::Range4G => 1.0 / 128.0,
            AccelerometerRange::Range8G => 1.0 / 64.0,
            AccelerometerRange::Range16G => 1.0 / 32.0,
        }
    }

    /// Get the register value for configuring the ADXL345 range
    fn register_value(&self) -> u8 {
        match self {
            AccelerometerRange::Range2G => 0x00,
            AccelerometerRange::Range4G => 0x01,
            AccelerometerRange::Range8G => 0x02,
            AccelerometerRange::Range16G => 0x03,
        }
    }
}

impl Default for AccelerometerRange {
    fn default() -> Self {
        AccelerometerRange::Range2G
    }
}

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
/// 
/// ## I2C Mode
/// * 3.3V and GND are connected between ESP32 and ADXL345
/// * SDA (data line) on ADXL345 connects to GPIO21 on ESP32
/// * SCL (clock line) on ADXL345 connects to GPIO22 on ESP32
///
/// ## SPI Mode  
/// * 3.3V and GND are connected between ESP32 and ADXL345
/// * SCLK on ADXL345 connects to SPI clock pin (e.g., GPIO18)
/// * MOSI (SDI on ADXL345) connects to SPI MOSI pin (e.g., GPIO23) 
/// * MISO (SDO on ADXL345) connects to SPI MISO pin (e.g., GPIO19)
/// * CS on ADXL345 connects to a GPIO pin for chip select (e.g., GPIO5)
///
/// # Usage Examples
///
/// ```rust
/// // I2C usage with default ±2g range
/// let peripherals = Peripherals::take().unwrap();
/// let mut accel = Accelerometer::new(
///     peripherals.i2c0,
///     peripherals.pins.gpio21, // SDA
///     peripherals.pins.gpio22, // SCL
///     AccelerometerRange::Range2G, // Optional: specify range
/// ).unwrap();
///
/// // SPI usage with ±16g range  
/// let spi_driver = SpiDriver::new(
///     peripherals.spi2,
///     peripherals.pins.gpio18, // SCLK
///     peripherals.pins.gpio23, // MOSI
///     Some(peripherals.pins.gpio19), // MISO
///     &SpiDriverConfig::new(),
/// ).unwrap();
/// let mut accel = Accelerometer::new_spi(
///     &spi_driver,
///     peripherals.pins.gpio5, // CS
///     AccelerometerRange::Range16G,
/// ).unwrap();
///
/// // Read acceleration (works the same for both I2C and SPI)
/// let reading = accel.read().unwrap();
/// println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
///
/// // Power management (works the same for both I2C and SPI)
/// accel.sleep().unwrap();   // Put in sleep mode to save power
/// accel.wake().unwrap();    // Wake up for measurements
/// ```
pub struct Accelerometer<'a> {
    comm: CommInterface<'a>,
    scale_factor: f32,
}

impl<'a> CommInterface<'a> {
    /// Write a single register value
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), EspError> {
        match self {
            CommInterface::I2c(i2c) => {
                i2c.write(ADXL345_ADDR, &[register, value], BLOCK)
            }
            CommInterface::Spi(spi) => {
                // In SPI mode, write operations have bit 7 clear (0)
                let spi_register = register & !SPI_READ_BIT;
                spi.write(&[spi_register, value])
            }
        }
    }

    /// Read a single register value
    fn read_register(&mut self, register: u8) -> Result<u8, EspError> {
        match self {
            CommInterface::I2c(i2c) => {
                let mut data = [0u8; 1];
                i2c.write_read(ADXL345_ADDR, &[register], &mut data, BLOCK)?;
                Ok(data[0])
            }
            CommInterface::Spi(spi) => {
                // In SPI mode, read operations have bit 7 set (1)
                let spi_register = register | SPI_READ_BIT;
                let mut data = [0u8; 2]; // Send register + receive data
                spi.transfer(&mut data, &[spi_register, 0x00])?;
                Ok(data[1]) // Return the second byte (the actual data)
            }
        }
    }

    /// Read multiple bytes from consecutive registers
    fn read_registers(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), EspError> {
        match self {
            CommInterface::I2c(i2c) => {
                i2c.write_read(ADXL345_ADDR, &[start_register], buffer, BLOCK)
            }
            CommInterface::Spi(spi) => {
                // In SPI mode, multi-byte reads need both read bit and multi-byte bit set
                let spi_register = start_register | SPI_READ_BIT | SPI_MULTI_BYTE_BIT;
                
                // For ADXL345, we'll never read more than 6 bytes at once (3 axes * 2 bytes each)
                // so a fixed array is sufficient and more embedded-friendly
                let mut tx_buffer = [0u8; 8]; // Max 7 bytes needed (1 cmd + 6 data)
                let mut rx_buffer = [0u8; 8];
                
                tx_buffer[0] = spi_register;
                let transfer_len = buffer.len() + 1;
                
                spi.transfer(&mut rx_buffer[..transfer_len], &tx_buffer[..transfer_len])?;
                
                // Copy the data portion (skip the first dummy byte)
                buffer.copy_from_slice(&rx_buffer[1..transfer_len]);
                Ok(())
            }
        }
    }
}

impl<'a> Accelerometer<'a> {
    /// Create a new accelerometer instance
    ///
    /// # Arguments
    /// * `i2c_peripheral` - The I2C peripheral to use (typically i2c0)
    /// * `sda_pin` - The SDA (data) pin (typically GPIO21)
    /// * `scl_pin` - The SCL (clock) pin (typically GPIO22)
    /// * `range` - The measurement range (±2g, ±4g, ±8g, or ±16g)
    ///
    /// # Returns
    /// Result containing the accelerometer instance or an error
    ///
    /// # Examples
    /// ```rust
    /// // Default ±2g range for highest resolution
    /// let accel = Accelerometer::new(i2c, sda, scl, AccelerometerRange::Range2G)?;
    ///
    /// // ±16g range for measuring high accelerations (e.g., impacts)
    /// let accel = Accelerometer::new(i2c, sda, scl, AccelerometerRange::Range16G)?;
    /// ```
    pub fn new<SDA, SCL>(
        i2c_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::i2c::I2c>
            + 'a,
        sda_pin: SDA,
        scl_pin: SCL,
        range: AccelerometerRange,
    ) -> Result<Self, EspError>
    where
        SDA: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCL: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
    {
        // Configure I2C with 400kHz bus speed (standard for ADXL345)
        let i2c_cfg = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c = I2cDriver::new(i2c_peripheral, sda_pin, scl_pin, &i2c_cfg)?;

        let mut accelerometer = Self {
            comm: CommInterface::I2c(i2c),
            scale_factor: range.scale_factor(),
        };

        // Configure the measurement range in the ADXL345 hardware
        accelerometer.comm.write_register(REG_DATA_FORMAT, range.register_value())?;

        // Initialize the accelerometer in measurement mode
        accelerometer.wake()?;

        Ok(accelerometer)
    }

    /// Create a new accelerometer instance using SPI communication
    ///
    /// # Arguments
    /// * `spi_driver` - The SPI driver instance (shared between devices)
    /// * `cs_pin` - The chip select pin for this device  
    /// * `range` - The measurement range (±2g, ±4g, ±8g, or ±16g)
    ///
    /// # Returns
    /// Result containing the accelerometer instance or an error
    ///
    /// # Examples
    /// ```rust
    /// // Create SPI driver first
    /// let spi_driver = SpiDriver::new(
    ///     peripherals.spi2,
    ///     sclk_pin,
    ///     mosi_pin,  
    ///     Some(miso_pin),
    ///     &SpiDriverConfig::new(),
    /// )?;
    ///
    /// // Create accelerometer with SPI
    /// let accel = Accelerometer::new_spi(
    ///     &spi_driver,
    ///     cs_pin,
    ///     AccelerometerRange::Range2G
    /// )?;
    /// ```
    pub fn new_spi<CS>(
        spi_driver: &'a SpiDriver<'a>,
        cs_pin: CS,
        range: AccelerometerRange,
    ) -> Result<Self, EspError>
    where
        CS: esp_idf_svc::hal::gpio::OutputPin,
    {
        // Configure SPI with appropriate settings for ADXL345
        // The ADXL345 supports up to 5MHz SPI clock
        let spi_config = SpiConfig::new().baudrate(Hertz(1_000_000)); // 1MHz for reliability
        let spi_device = SpiDeviceDriver::new(spi_driver, Some(cs_pin), &spi_config)?;

        let mut accelerometer = Self {
            comm: CommInterface::Spi(spi_device),
            scale_factor: range.scale_factor(),
        };

        // Configure the measurement range in the ADXL345 hardware
        accelerometer.comm.write_register(REG_DATA_FORMAT, range.register_value())?;

        // Initialize the accelerometer in measurement mode
        accelerometer.wake()?;

        Ok(accelerometer)
    }

    /// Read acceleration data from all three axes
    ///
    /// Returns acceleration values in units of g-force (1g = 9.8 m/s²).
    /// The scale factor is determined by the range configured in `new()`.
    ///
    /// # Returns
    /// Result containing AccelerationReading or an error
    pub fn read(&mut self) -> Result<AccelerationReading, EspError> {
        // Read all 6 bytes at once for better efficiency (X, Y, Z data)
        let mut data = [0u8; 6];
        self.comm.read_registers(REG_DATAX0, &mut data)?;

        // Convert pairs of bytes to signed 16-bit integers (little-endian)
        let x_raw = i16::from_le_bytes([data[0], data[1]]);
        let y_raw = i16::from_le_bytes([data[2], data[3]]);
        let z_raw = i16::from_le_bytes([data[4], data[5]]);

        // Convert to g-force units using the configured scale factor
        let x = x_raw as f32 * self.scale_factor;
        let y = y_raw as f32 * self.scale_factor;
        let z = z_raw as f32 * self.scale_factor;

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
        // Read all 6 bytes at once for better efficiency
        let mut data = [0u8; 6];
        self.comm.read_registers(REG_DATAX0, &mut data)?;

        // Convert pairs of bytes to signed 16-bit integers (little-endian)
        let x_raw = i16::from_le_bytes([data[0], data[1]]);
        let y_raw = i16::from_le_bytes([data[2], data[3]]);
        let z_raw = i16::from_le_bytes([data[4], data[5]]);

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
        self.comm.write_register(REG_POWER_CTL, POWER_CTL_SLEEP)?;
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
        self.comm.write_register(REG_POWER_CTL, POWER_CTL_MEASURE)?;
        Ok(())
    }

    /// Check if the accelerometer is responding to communication commands
    ///
    /// This can be used to verify the device is properly connected and functioning.
    /// Works with both I2C and SPI interfaces.
    /// Note: This will wake the device if it's sleeping.
    pub fn is_connected(&mut self) -> bool {
        // Try to read the power control register
        self.comm.read_register(REG_POWER_CTL).is_ok()
    }
}

/// Convenience function for continuous reading demonstration
/// 
/// This function shows how to use the accelerometer for continuous monitoring
/// and works with both I2C and SPI interfaces.
/// Note: This function takes ownership of peripherals, so it should be called 
/// instead of your main loop, not in addition to it.
pub fn run_continuous_reading() -> Result<(), EspError> {
    // Take peripherals - this should only be called once in your application
    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    // Example using I2C (comment out if using SPI instead)
    let mut accel = Accelerometer::new(
        peripherals.i2c0,
        peripherals.pins.gpio21, // SDA
        peripherals.pins.gpio22, // SCL  
        AccelerometerRange::Range16G,
    )?;

    // Example using SPI (uncomment to use instead of I2C)
    // let spi_driver = SpiDriver::new(
    //     peripherals.spi2,
    //     peripherals.pins.gpio18, // SCLK
    //     peripherals.pins.gpio23, // MOSI
    //     Some(peripherals.pins.gpio19), // MISO
    //     &SpiDriverConfig::new(),
    // )?;
    // let mut accel = Accelerometer::new_spi(
    //     &spi_driver,
    //     peripherals.pins.gpio5, // CS
    //     AccelerometerRange::Range16G,
    // )?;

    loop {
        if accel.is_connected() {
            if let Ok(reading) = accel.read() {
                println!(
                    "X: {:.3}g, Y: {:.3}g, Z: {:.3}g, Magnitude: {:.3}g",
                    reading.x, reading.y, reading.z, reading.magnitude()
                );
            }
        } else {
            println!("Accelerometer not connected");
        }

        thread::sleep(Duration::from_millis(50));
    }
}
