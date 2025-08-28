//! # ADXL345 Accelerometer Driver for ESP32
//!
//! This module provides a clean, object-oriented interface for the ADXL345 3-axis accelerometer.
//! The driver supports both I2C and SPI communication protocols.
//! 
//! ## Key Features:
//! - **Dual Communication**: Supports both I2C and SPI protocols
//! - **Struct-based API**: Clean object-oriented interface with proper error handling
//! - **Multiple Ranges**: Configurable measurement ranges (±2g, ±4g, ±8g, ±16g)
//! - **Power Management**: Sleep and wake methods for power conservation
//! - **Raw Data Access**: Both scaled and raw data reading capabilities
//!
//! ## Usage Examples
//! 
//! ### I2C Communication
//! ```rust
//! // I2C with default ±2g range
//! let mut accel = Accelerometer::new(i2c_peripheral, sda_pin, scl_pin, AccelerometerRange::Range2G)?;
//! let reading = accel.read()?;
//! println!("Acceleration: {:.3}g", reading.magnitude());
//! ```
//! 
//! ### SPI Communication  
//! ```rust
//! // SPI with default ±2g range
//! let mut accel = Accelerometer::new_spi(spi_peripheral, mosi_pin, miso_pin, sclk_pin, cs_pin)?;
//! 
//! // SPI with custom ±16g range for high accelerations
//! let mut accel = Accelerometer::new_spi_with_range(
//!     spi_peripheral, mosi_pin, miso_pin, sclk_pin, cs_pin, AccelerometerRange::Range16G
//! )?;
//! ```

use esp_idf_svc::hal::{
    delay::BLOCK,
    gpio::{Output, OutputPin, PinDriver},
    i2c::{I2cConfig, I2cDriver},
    prelude::Peripherals,
    spi::{SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig},
    units::Hertz,
};
use esp_idf_sys::EspError;
use std::{thread, time::Duration};

/// ADXL345 3-axis accelerometer I2C address
/// Found in the I2C section of the ADXL345 datasheet:
/// https://analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
const ADXL345_ADDR: u8 = 0x53;

/// SPI register address formatting constants
/// For SPI, register addresses need special bit formatting:
/// - Bit 7: 1 for read, 0 for write
/// - Bit 6: 1 for multi-byte, 0 for single-byte  
/// - Bits 5-0: register address
const SPI_READ_BIT: u8 = 0x80;    // Set bit 7 for read operations
const SPI_MULTI_BYTE_BIT: u8 = 0x40;  // Set bit 6 for multi-byte reads

/// ADXL345 register addresses and power control values
const REG_POWER_CTL: u8 = 0x2d;     // Power control register
const POWER_CTL_WAKEUP: u8 = 0x2;   // Wake up the device (low power mode)
const POWER_CTL_SLEEP: u8 = 0x4;    // Put device in sleep mode
const POWER_CTL_MEASURE: u8 = 0x08; // Put device in measurement mode

/// Data format register for configuring measurement range
const REG_DATA_FORMAT: u8 = 0x31;   // Data format control register

/// Data register addresses for X, Y, Z axis readings
/// Each axis uses 2 bytes (LSB first, then MSB)
const REG_DATAX0: u8 = 0x32;  // X-axis data LSB
const REG_DATAY0: u8 = 0x34;  // Y-axis data LSB  
const REG_DATAZ0: u8 = 0x36;  // Z-axis data LSB

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

/// Communication interface for the ADXL345 accelerometer
/// 
/// The ADXL345 supports both I2C and SPI communication protocols.
/// This enum abstracts the communication layer to allow the same
/// accelerometer driver to work with either protocol.
enum AccelerometerInterface<'a> {
    /// I2C communication using SDA and SCL pins
    I2c(I2cDriver<'a>),
    /// SPI communication using MOSI, MISO, SCLK, and CS pins
    Spi(SpiDeviceDriver<'a, SpiDriver<'a>>),
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
/// * MOSI (Master Out Slave In) on ESP32 connects to SDA/SDI on ADXL345
/// * MISO (Master In Slave Out) on ESP32 connects to SDO on ADXL345  
/// * SCLK (Serial Clock) on ESP32 connects to SCL/SCLK on ADXL345
/// * CS (Chip Select) on ESP32 connects to CS on ADXL345
///
/// # Usage Examples
/// 
/// ```rust
/// // I2C mode with default ±2g range
/// let peripherals = Peripherals::take().unwrap();
/// let mut accel = Accelerometer::new(
///     peripherals.i2c0,
///     peripherals.pins.gpio21, // SDA
///     peripherals.pins.gpio22, // SCL
///     AccelerometerRange::Range2G,
/// ).unwrap();
/// 
/// // SPI mode with default ±2g range
/// let mut accel = Accelerometer::new_spi(
///     peripherals.spi2,
///     peripherals.pins.gpio23, // MOSI
///     peripherals.pins.gpio19, // MISO
///     peripherals.pins.gpio18, // SCLK
///     peripherals.pins.gpio5,  // CS
/// ).unwrap();
/// 
/// // Read acceleration
/// let reading = accel.read().unwrap();
/// println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
/// ```
pub struct Accelerometer<'a> {
    interface: AccelerometerInterface<'a>,
    scale_factor: f32,
}

impl<'a> Accelerometer<'a> {
    /// Create a new accelerometer instance using I2C communication
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
        i2c_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::i2c::I2c> + 'a,
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
            interface: AccelerometerInterface::I2c(i2c),
            scale_factor: range.scale_factor(),
        };

        // Configure the measurement range in the ADXL345 hardware
        accelerometer.write_register(REG_DATA_FORMAT, range.register_value())?;

        // Initialize the accelerometer in measurement mode
        accelerometer.wake()?;

        Ok(accelerometer)
    }

    /// Create a new accelerometer instance using SPI communication with default ±2g range
    /// 
    /// # Arguments
    /// * `spi_peripheral` - The SPI peripheral to use (typically spi2)
    /// * `mosi_pin` - The MOSI (Master Out Slave In) pin 
    /// * `miso_pin` - The MISO (Master In Slave Out) pin
    /// * `sclk_pin` - The SCLK (Serial Clock) pin
    /// * `cs_pin` - The CS (Chip Select) pin
    /// 
    /// # Returns
    /// Result containing the accelerometer instance or an error
    /// 
    /// # Examples
    /// ```rust
    /// let accel = Accelerometer::new_spi(
    ///     peripherals.spi2,
    ///     peripherals.pins.gpio23, // MOSI
    ///     peripherals.pins.gpio19, // MISO
    ///     peripherals.pins.gpio18, // SCLK
    ///     peripherals.pins.gpio5,  // CS
    /// )?;
    /// ```
    pub fn new_spi<MOSI, MISO, SCLK, CS>(
        spi_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::spi::Spi> + 'a,
        mosi_pin: MOSI,
        miso_pin: MISO, 
        sclk_pin: SCLK,
        cs_pin: CS,
    ) -> Result<Self, EspError>
    where
        MOSI: esp_idf_svc::hal::gpio::OutputPin,
        MISO: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCLK: esp_idf_svc::hal::gpio::OutputPin,
        CS: esp_idf_svc::hal::gpio::OutputPin,
    {
        Self::new_spi_with_range(spi_peripheral, mosi_pin, miso_pin, sclk_pin, cs_pin, AccelerometerRange::Range2G)
    }

    /// Create a new accelerometer instance using SPI communication with specified range
    /// 
    /// # Arguments
    /// * `spi_peripheral` - The SPI peripheral to use (typically spi2)
    /// * `mosi_pin` - The MOSI (Master Out Slave In) pin 
    /// * `miso_pin` - The MISO (Master In Slave Out) pin
    /// * `sclk_pin` - The SCLK (Serial Clock) pin
    /// * `cs_pin` - The CS (Chip Select) pin
    /// * `range` - The measurement range (±2g, ±4g, ±8g, or ±16g)
    /// 
    /// # Returns
    /// Result containing the accelerometer instance or an error
    /// 
    /// # Examples
    /// ```rust
    /// // High-range SPI configuration for measuring large accelerations
    /// let accel = Accelerometer::new_spi_with_range(
    ///     peripherals.spi2,
    ///     peripherals.pins.gpio23, // MOSI
    ///     peripherals.pins.gpio19, // MISO
    ///     peripherals.pins.gpio18, // SCLK
    ///     peripherals.pins.gpio5,  // CS
    ///     AccelerometerRange::Range16G,
    /// )?;
    /// ```
    pub fn new_spi_with_range<MOSI, MISO, SCLK, CS>(
        spi_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::spi::Spi> + 'a,
        mosi_pin: MOSI,
        miso_pin: MISO,
        sclk_pin: SCLK,
        cs_pin: CS,
        range: AccelerometerRange,
    ) -> Result<Self, EspError>
    where
        MOSI: esp_idf_svc::hal::gpio::OutputPin,
        MISO: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCLK: esp_idf_svc::hal::gpio::OutputPin,
        CS: esp_idf_svc::hal::gpio::OutputPin,
    {
        // Configure SPI with appropriate settings for ADXL345
        // ADXL345 SPI supports up to 5MHz clock speed
        let spi_config = SpiDriverConfig::new();
        let spi_driver = SpiDriver::new(spi_peripheral, sclk_pin, mosi_pin, Some(miso_pin), &spi_config)?;
        
        // Configure SPI device settings for ADXL345
        let spi_device_config = SpiConfig::new()
            .baudrate(Hertz(1_000_000)) // 1MHz for reliable communication
            .data_mode(esp_idf_svc::hal::spi::config::Mode {
                polarity: esp_idf_svc::hal::spi::config::Polarity::IdleLow,
                phase: esp_idf_svc::hal::spi::config::Phase::CaptureOnFirstTransition,
            });
        
        let spi_device = SpiDeviceDriver::new(spi_driver, Some(cs_pin), &spi_device_config)?;

        let mut accelerometer = Self {
            interface: AccelerometerInterface::Spi(spi_device),
            scale_factor: range.scale_factor(),
        };

        // Configure the measurement range in the ADXL345 hardware
        accelerometer.write_register(REG_DATA_FORMAT, range.register_value())?;

        // Initialize the accelerometer in measurement mode
        accelerometer.wake()?;

        Ok(accelerometer)
    }

    /// Write a value to a register (internal helper method)
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), EspError> {
        match &mut self.interface {
            AccelerometerInterface::I2c(i2c) => {
                i2c.write(ADXL345_ADDR, &[register, value], BLOCK)
            }
            AccelerometerInterface::Spi(spi) => {
                // For SPI write: bit 7 = 0 (write), bit 6 = 0 (single byte)
                let spi_register = register & 0x3F; // Clear bits 7 and 6
                spi.write(&[spi_register, value])
            }
        }
    }

    /// Read a single byte from a register (internal helper method)
    fn read_register(&mut self, register: u8) -> Result<u8, EspError> {
        match &mut self.interface {
            AccelerometerInterface::I2c(i2c) => {
                let mut data = [0u8; 1];
                i2c.write_read(ADXL345_ADDR, &[register], &mut data, BLOCK)?;
                Ok(data[0])
            }
            AccelerometerInterface::Spi(spi) => {
                // For SPI read: bit 7 = 1 (read), bit 6 = 0 (single byte)
                let spi_register = register | SPI_READ_BIT;
                let mut tx_data = [spi_register, 0x00];
                let mut rx_data = [0u8; 2];
                spi.transfer(&mut rx_data, &tx_data)?;
                Ok(rx_data[1]) // Data is in the second byte
            }
        }
    }

    /// Read multiple bytes from consecutive registers (internal helper method)
    fn read_registers(&mut self, start_register: u8, data: &mut [u8]) -> Result<(), EspError> {
        match &mut self.interface {
            AccelerometerInterface::I2c(i2c) => {
                i2c.write_read(ADXL345_ADDR, &[start_register], data, BLOCK)
            }
            AccelerometerInterface::Spi(spi) => {
                // For SPI multi-byte read: bit 7 = 1 (read), bit 6 = 1 (multi-byte)
                let spi_register = start_register | SPI_READ_BIT | SPI_MULTI_BYTE_BIT;
                
                // Create tx buffer: register address + dummy bytes for each data byte
                let mut tx_data = vec![0u8; data.len() + 1];
                tx_data[0] = spi_register;
                
                // Create rx buffer to receive data
                let mut rx_data = vec![0u8; data.len() + 1];
                
                spi.transfer(&mut rx_data, &tx_data)?;
                
                // Copy received data (skip first byte which is dummy)
                data.copy_from_slice(&rx_data[1..]);
                Ok(())
            }
        }
    }

    /// Read acceleration data from all three axes
    /// 
    /// Returns acceleration values in units of g-force (1g = 9.8 m/s²).
    /// The scale factor is determined by the range configured in constructor.
    /// 
    /// # Returns
    /// Result containing AccelerationReading or an error
    pub fn read(&mut self) -> Result<AccelerationReading, EspError> {
        let mut x_data: [u8; 2] = [0; 2];
        let mut y_data: [u8; 2] = [0; 2]; 
        let mut z_data: [u8; 2] = [0; 2];

        // Read 2 bytes for each axis (LSB first, then MSB)
        self.read_registers(REG_DATAX0, &mut x_data)?;
        self.read_registers(REG_DATAY0, &mut y_data)?;
        self.read_registers(REG_DATAZ0, &mut z_data)?;

        // Convert little-endian bytes to signed 16-bit integers
        let x_raw = i16::from_le_bytes(x_data);
        let y_raw = i16::from_le_bytes(y_data);
        let z_raw = i16::from_le_bytes(z_data);

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
        let mut x_data: [u8; 2] = [0; 2];
        let mut y_data: [u8; 2] = [0; 2]; 
        let mut z_data: [u8; 2] = [0; 2];

        // Read 2 bytes for each axis (LSB first, then MSB)
        self.read_registers(REG_DATAX0, &mut x_data)?;
        self.read_registers(REG_DATAY0, &mut y_data)?;
        self.read_registers(REG_DATAZ0, &mut z_data)?;

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
        self.write_register(REG_POWER_CTL, POWER_CTL_SLEEP)
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
        self.write_register(REG_POWER_CTL, POWER_CTL_MEASURE)
    }

    /// Check if the accelerometer is responding to commands
    /// 
    /// This can be used to verify the device is properly connected and functioning.
    /// Note: This will wake the device if it's sleeping.
    pub fn is_connected(&mut self) -> bool {
        // Try to read the power control register
        self.read_register(REG_POWER_CTL).is_ok()
    }
}

/// Convenience function for simple I2C accelerometer setup and continuous reading
/// 
/// This provides a simple interface similar to the original setup() function
/// but allows for more flexible usage patterns.
/// 
/// # Example
/// ```rust
/// use accelerometer::run_continuous_reading;
/// 
/// // This will run indefinitely, printing accelerometer readings via I2C
/// run_continuous_reading().unwrap();
/// ```
pub fn run_continuous_reading() -> Result<(), EspError> {
    let peripherals = Peripherals::take().unwrap();

    let mut accel = Accelerometer::new(
        peripherals.i2c0,
        peripherals.pins.gpio21, // SDA
        peripherals.pins.gpio22, // SCL
        AccelerometerRange::Range2G, // Use default ±2g range
    )?;

    loop {
        match accel.read() {
            Ok(reading) => {
                println!("I2C Acceleration - X: {:.3}g, Y: {:.3}g, Z: {:.3}g (magnitude: {:.3}g)", 
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

/// Convenience function for simple SPI accelerometer setup and continuous reading
/// 
/// This demonstrates how to use the ADXL345 accelerometer via SPI protocol.
/// 
/// # Example
/// ```rust
/// use accelerometer::run_continuous_reading_spi;
/// 
/// // This will run indefinitely, printing accelerometer readings via SPI
/// run_continuous_reading_spi().unwrap();
/// ```
pub fn run_continuous_reading_spi() -> Result<(), EspError> {
    let peripherals = Peripherals::take().unwrap();

    let mut accel = Accelerometer::new_spi(
        peripherals.spi2,
        peripherals.pins.gpio23, // MOSI
        peripherals.pins.gpio19, // MISO
        peripherals.pins.gpio18, // SCLK
        peripherals.pins.gpio5,  // CS
    )?;

    loop {
        match accel.read() {
            Ok(reading) => {
                println!("SPI Acceleration - X: {:.3}g, Y: {:.3}g, Z: {:.3}g (magnitude: {:.3}g)", 
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
