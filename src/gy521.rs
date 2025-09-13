//! # GY521 (MPU6050) IMU Driver for ESP32
//!
//! This module provides a clean, object-oriented interface for the GY521 module,
//! which contains the MPU6050 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) with temperature sensor.
//!
//! ## Key Features:
//! - **6-axis IMU**: 3-axis accelerometer and 3-axis gyroscope
//! - **Temperature sensor**: Built-in temperature measurement
//! - **Separate reading functions**: Individual functions for rotation, movement, and temperature
//! - **Configurable ranges**: Adjustable accelerometer and gyroscope ranges
//! - **Power management**: Sleep/wake functionality for power conservation
//! - **Proper error handling**: Uses `Result<T, EspError>` throughout
//!
//! ## Usage
//! ```rust
//! // Initialize GY521 with default ranges
//! let mut gy521 = GY521::new(i2c_peripheral, sda_pin, scl_pin)?;
//! 
//! // Read rotation (gyroscope data)
//! let rotation = gy521.read_rotation()?;
//! println!("Rotation - X: {:.2}°/s, Y: {:.2}°/s, Z: {:.2}°/s", rotation.x, rotation.y, rotation.z);
//! 
//! // Read movement (accelerometer data)
//! let movement = gy521.read_movement()?;
//! println!("Movement - X: {:.2}g, Y: {:.2}g, Z: {:.2}g", movement.x, movement.y, movement.z);
//! 
//! // Read temperature
//! let temp = gy521.read_temperature()?;
//! println!("Temperature: {:.1}°C", temp);
//! ```

use esp_idf_svc::hal::{
    delay::BLOCK,
    i2c::{I2cConfig, I2cDriver},
    units::Hertz,
};
use esp_idf_sys::EspError;

/// MPU6050 I2C address (when AD0 pin is low)
/// The GY521 module typically has AD0 connected to GND, giving address 0x68
const MPU6050_ADDR: u8 = 0x68;

/// MPU6050 register addresses
const REG_PWR_MGMT_1: u8 = 0x6B;      // Power management register
const REG_PWR_MGMT_2: u8 = 0x6C;      // Power management register 2
const REG_GYRO_CONFIG: u8 = 0x1B;     // Gyroscope configuration
const REG_ACCEL_CONFIG: u8 = 0x1C;    // Accelerometer configuration
const REG_TEMP_OUT_H: u8 = 0x41;      // Temperature high byte
const REG_TEMP_OUT_L: u8 = 0x42;      // Temperature low byte

// Accelerometer data registers (high byte first)
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_ACCEL_XOUT_L: u8 = 0x3C;
const REG_ACCEL_YOUT_H: u8 = 0x3D;
const REG_ACCEL_YOUT_L: u8 = 0x3E;
const REG_ACCEL_ZOUT_H: u8 = 0x3F;
const REG_ACCEL_ZOUT_L: u8 = 0x40;

// Gyroscope data registers (high byte first)
const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_XOUT_L: u8 = 0x44;
const REG_GYRO_YOUT_H: u8 = 0x45;
const REG_GYRO_YOUT_L: u8 = 0x46;
const REG_GYRO_ZOUT_H: u8 = 0x47;
const REG_GYRO_ZOUT_L: u8 = 0x48;

/// Accelerometer range configuration
/// Different ranges provide different sensitivity and measurement limits
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    /// ±2g range (highest sensitivity: 16384 LSB/g)
    Range2G,
    /// ±4g range (8192 LSB/g)
    Range4G,
    /// ±8g range (4096 LSB/g)  
    Range8G,
    /// ±16g range (lowest sensitivity: 2048 LSB/g)
    Range16G,
}

impl AccelRange {
    /// Get the scale factor for converting raw readings to g-force
    fn scale_factor(&self) -> f32 {
        match self {
            AccelRange::Range2G => 1.0 / 16384.0,
            AccelRange::Range4G => 1.0 / 8192.0,
            AccelRange::Range8G => 1.0 / 4096.0,
            AccelRange::Range16G => 1.0 / 2048.0,
        }
    }

    /// Get the register value for configuring the MPU6050 accelerometer range
    fn register_value(&self) -> u8 {
        match self {
            AccelRange::Range2G => 0x00,
            AccelRange::Range4G => 0x08,
            AccelRange::Range8G => 0x10,
            AccelRange::Range16G => 0x18,
        }
    }
}

impl Default for AccelRange {
    fn default() -> Self {
        AccelRange::Range2G
    }
}

/// Gyroscope range configuration
/// Different ranges provide different sensitivity and measurement limits
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    /// ±250°/s range (highest sensitivity: 131 LSB/(°/s))
    Range250,
    /// ±500°/s range (65.5 LSB/(°/s))
    Range500,
    /// ±1000°/s range (32.8 LSB/(°/s))
    Range1000,
    /// ±2000°/s range (lowest sensitivity: 16.4 LSB/(°/s))
    Range2000,
}

impl GyroRange {
    /// Get the scale factor for converting raw readings to degrees per second
    fn scale_factor(&self) -> f32 {
        match self {
            GyroRange::Range250 => 1.0 / 131.0,
            GyroRange::Range500 => 1.0 / 65.5,
            GyroRange::Range1000 => 1.0 / 32.8,
            GyroRange::Range2000 => 1.0 / 16.4,
        }
    }

    /// Get the register value for configuring the MPU6050 gyroscope range
    fn register_value(&self) -> u8 {
        match self {
            GyroRange::Range250 => 0x00,
            GyroRange::Range500 => 0x08,
            GyroRange::Range1000 => 0x10,
            GyroRange::Range2000 => 0x18,
        }
    }
}

impl Default for GyroRange {
    fn default() -> Self {
        GyroRange::Range250
    }
}

/// 3-axis rotation reading in degrees per second
#[derive(Debug, Clone, Copy)]
pub struct RotationReading {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl RotationReading {
    /// Create a new rotation reading
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Get the magnitude of the rotation vector
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// 3-axis movement/acceleration reading in g-force units
#[derive(Debug, Clone, Copy)]
pub struct MovementReading {
    pub x: f32,
    pub y: f32,  
    pub z: f32,
}

impl MovementReading {
    /// Create a new movement reading
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Get the magnitude of the movement vector
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// GY521 (MPU6050) IMU driver for ESP32
/// 
/// # Hardware Connections
/// * VCC: 3.3V or 5V (GY521 module has voltage regulation)
/// * GND: Ground
/// * SDA: GPIO21 (I2C data line)
/// * SCL: GPIO22 (I2C clock line)
/// * AD0: Usually connected to GND on GY521 modules (sets I2C address to 0x68)
///
/// # Usage Examples
/// ```rust
/// // Initialize with default ranges
/// let peripherals = Peripherals::take().unwrap();
/// let mut gy521 = GY521::new(
///     peripherals.i2c0,
///     peripherals.pins.gpio21, // SDA
///     peripherals.pins.gpio22, // SCL
/// ).unwrap();
///
/// // Read different sensor data
/// let rotation = gy521.read_rotation().unwrap();
/// let movement = gy521.read_movement().unwrap();
/// let temp = gy521.read_temperature().unwrap();
/// 
/// println!("Rotation: {:.2}°/s", rotation.magnitude());
/// println!("Movement: {:.2}g", movement.magnitude());
/// println!("Temperature: {:.1}°C", temp);
/// ```
pub struct GY521<'a> {
    i2c: I2cDriver<'a>,
    accel_scale: f32,
    gyro_scale: f32,
}

impl<'a> GY521<'a> {
    /// Create a new GY521 instance with default ranges
    ///
    /// Uses default ranges: ±2g for accelerometer, ±250°/s for gyroscope
    ///
    /// # Arguments
    /// * `i2c_peripheral` - The I2C peripheral to use (typically i2c0)
    /// * `sda_pin` - The SDA (data) pin (typically GPIO21)
    /// * `scl_pin` - The SCL (clock) pin (typically GPIO22)
    ///
    /// # Returns
    /// Result containing the GY521 instance or an error
    pub fn new<SDA, SCL>(
        i2c_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::i2c::I2c>
            + 'a,
        sda_pin: SDA,
        scl_pin: SCL,
    ) -> Result<Self, EspError>
    where
        SDA: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCL: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
    {
        Self::new_with_ranges(
            i2c_peripheral,
            sda_pin,
            scl_pin,
            AccelRange::default(),
            GyroRange::default(),
        )
    }

    /// Create a new GY521 instance with custom ranges
    ///
    /// # Arguments
    /// * `i2c_peripheral` - The I2C peripheral to use (typically i2c0)
    /// * `sda_pin` - The SDA (data) pin (typically GPIO21)
    /// * `scl_pin` - The SCL (clock) pin (typically GPIO22)
    /// * `accel_range` - Accelerometer measurement range
    /// * `gyro_range` - Gyroscope measurement range
    ///
    /// # Returns
    /// Result containing the GY521 instance or an error
    pub fn new_with_ranges<SDA, SCL>(
        i2c_peripheral: impl esp_idf_svc::hal::peripheral::Peripheral<P = impl esp_idf_svc::hal::i2c::I2c>
            + 'a,
        sda_pin: SDA,
        scl_pin: SCL,
        accel_range: AccelRange,
        gyro_range: GyroRange,
    ) -> Result<Self, EspError>
    where
        SDA: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
        SCL: esp_idf_svc::hal::gpio::InputPin + esp_idf_svc::hal::gpio::OutputPin,
    {
        // Configure I2C with 400kHz bus speed (standard for MPU6050)
        let i2c_cfg = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c = I2cDriver::new(i2c_peripheral, sda_pin, scl_pin, &i2c_cfg)?;

        let mut gy521 = Self {
            i2c,
            accel_scale: accel_range.scale_factor(),
            gyro_scale: gyro_range.scale_factor(),
        };

        // Initialize the MPU6050
        gy521.init(accel_range, gyro_range)?;

        Ok(gy521)
    }

    /// Initialize the MPU6050 chip
    fn init(&mut self, accel_range: AccelRange, gyro_range: GyroRange) -> Result<(), EspError> {
        // Wake up the MPU6050 (it starts in sleep mode)
        self.i2c.write(MPU6050_ADDR, &[REG_PWR_MGMT_1, 0x00], BLOCK)?;

        // Set accelerometer range
        self.i2c.write(
            MPU6050_ADDR,
            &[REG_ACCEL_CONFIG, accel_range.register_value()],
            BLOCK,
        )?;

        // Set gyroscope range  
        self.i2c.write(
            MPU6050_ADDR,
            &[REG_GYRO_CONFIG, gyro_range.register_value()],
            BLOCK,
        )?;

        Ok(())
    }

    /// Read rotation data from the gyroscope
    ///
    /// Returns rotation rates in degrees per second for all three axes.
    /// Positive values indicate rotation in the right-hand rule direction.
    ///
    /// # Returns
    /// Result containing RotationReading or an error
    pub fn read_rotation(&mut self) -> Result<RotationReading, EspError> {
        let mut data: [u8; 6] = [0; 6];

        // Read all 6 gyroscope registers in one transaction
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_XOUT_H], &mut data, BLOCK)?;

        // Convert big-endian bytes to signed 16-bit integers
        let x_raw = i16::from_be_bytes([data[0], data[1]]);
        let y_raw = i16::from_be_bytes([data[2], data[3]]);
        let z_raw = i16::from_be_bytes([data[4], data[5]]);

        // Convert to degrees per second using the configured scale factor
        let x = x_raw as f32 * self.gyro_scale;
        let y = y_raw as f32 * self.gyro_scale;
        let z = z_raw as f32 * self.gyro_scale;

        Ok(RotationReading::new(x, y, z))
    }

    /// Read movement data from the accelerometer
    ///
    /// Returns acceleration values in g-force units for all three axes.
    /// 1g = 9.8 m/s² (acceleration due to gravity at sea level).
    ///
    /// # Returns
    /// Result containing MovementReading or an error
    pub fn read_movement(&mut self) -> Result<MovementReading, EspError> {
        let mut data: [u8; 6] = [0; 6];

        // Read all 6 accelerometer registers in one transaction
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_XOUT_H], &mut data, BLOCK)?;

        // Convert big-endian bytes to signed 16-bit integers
        let x_raw = i16::from_be_bytes([data[0], data[1]]);
        let y_raw = i16::from_be_bytes([data[2], data[3]]);
        let z_raw = i16::from_be_bytes([data[4], data[5]]);

        // Convert to g-force units using the configured scale factor
        let x = x_raw as f32 * self.accel_scale;
        let y = y_raw as f32 * self.accel_scale;
        let z = z_raw as f32 * self.accel_scale;

        Ok(MovementReading::new(x, y, z))
    }

    /// Read temperature from the built-in temperature sensor
    ///
    /// Returns temperature in degrees Celsius.
    /// The MPU6050 temperature sensor is reasonably accurate for ambient temperature.
    ///
    /// # Returns
    /// Result containing temperature in °C or an error
    pub fn read_temperature(&mut self) -> Result<f32, EspError> {
        let mut data: [u8; 2] = [0; 2];

        // Read temperature registers
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_TEMP_OUT_H], &mut data, BLOCK)?;

        // Convert big-endian bytes to signed 16-bit integer
        let temp_raw = i16::from_be_bytes(data);

        // Convert to Celsius using the formula from the datasheet:
        // Temperature in degrees C = (TEMP_OUT Register Value as a signed number) / 340 + 36.53
        let temp_celsius = (temp_raw as f32) / 340.0 + 36.53;

        Ok(temp_celsius)
    }

    /// Put the MPU6050 into sleep mode to conserve power
    ///
    /// In sleep mode, the device consumes much less power but cannot take measurements.
    /// Call wake() to resume normal operation.
    pub fn sleep(&mut self) -> Result<(), EspError> {
        // Set the sleep bit in power management register 1
        self.i2c.write(MPU6050_ADDR, &[REG_PWR_MGMT_1, 0x40], BLOCK)?;
        Ok(())
    }

    /// Wake the MPU6050 from sleep mode and enable measurements
    ///
    /// This puts the device in normal operating mode where it actively measures
    /// acceleration, rotation, and temperature.
    pub fn wake(&mut self) -> Result<(), EspError> {
        // Clear the sleep bit in power management register 1
        self.i2c.write(MPU6050_ADDR, &[REG_PWR_MGMT_1, 0x00], BLOCK)?;
        Ok(())
    }

    /// Check if the MPU6050 is responding to I2C commands
    ///
    /// This can be used to verify the device is properly connected and functioning.
    pub fn is_connected(&mut self) -> bool {
        // Try to read the power management register
        let mut data = [0u8; 1];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_PWR_MGMT_1], &mut data, BLOCK)
            .is_ok()
    }
}

/// Convenience function to run continuous reading from GY521 sensors
///
/// This function demonstrates how to use the GY521 driver in a continuous loop.
/// It reads rotation, movement, and temperature data every 100ms.
/// 
/// # Example Output
/// ```
/// Rotation - X: 1.23°/s, Y: -0.45°/s, Z: 0.78°/s | Magnitude: 1.52°/s
/// Movement - X: 0.12g, Y: 0.05g, Z: 1.02g | Magnitude: 1.03g  
/// Temperature: 24.3°C
/// ```
pub fn run_continuous_reading() -> Result<(), EspError> {
    use esp_idf_svc::hal::prelude::Peripherals;
    use std::{thread, time::Duration};

    let peripherals = Peripherals::take().unwrap();
    let mut gy521 = GY521::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
    )?;

    loop {
        match (gy521.read_rotation(), gy521.read_movement(), gy521.read_temperature()) {
            (Ok(rotation), Ok(movement), Ok(temp)) => {
                println!(
                    "Rotation - X: {:.2}°/s, Y: {:.2}°/s, Z: {:.2}°/s | Magnitude: {:.2}°/s",
                    rotation.x, rotation.y, rotation.z, rotation.magnitude()
                );
                println!(
                    "Movement - X: {:.2}g, Y: {:.2}g, Z: {:.2}g | Magnitude: {:.2}g",
                    movement.x, movement.y, movement.z, movement.magnitude()
                );
                println!("Temperature: {:.1}°C", temp);
                println!("---");
            }
            _ => {
                println!("Error reading GY521 sensor data");
            }
        }

        thread::sleep(Duration::from_millis(100));
    }
}