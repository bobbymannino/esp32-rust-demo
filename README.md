# ESP32 Rust Demo

A very simple demo of programming an ESP32 in Rust

## Components

This demo includes drivers for:
- Button input (GPIO19)
- RGB LED output (GPIO16, GPIO5, GPIO17)
- Reed switch input
- **ADXL345 Accelerometer** - 3-axis accelerometer via I2C or SPI

## Accelerometer (ADXL345) Usage

The accelerometer module provides a clean, object-oriented API for interfacing with the ADXL345 3-axis accelerometer over I2C or SPI.

### Hardware Connections

#### I2C Mode (Default)
- **VCC**: 3.3V
- **GND**: Ground  
- **SDA**: GPIO21 (I2C data line)
- **SCL**: GPIO22 (I2C clock line)

#### SPI Mode (Alternative)
- **VCC**: 3.3V
- **GND**: Ground
- **SCLK**: GPIO18 (SPI clock line)
- **MOSI (SDI)**: GPIO23 (SPI data input to ADXL345)
- **MISO (SDO)**: GPIO19 (SPI data output from ADXL345)  
- **CS**: GPIO5 (Chip select, any available GPIO)

### Code Examples

```rust
use crate::accelerometer::{Accelerometer, AccelerationReading, AccelerometerRange};
use esp_idf_svc::hal::{spi::{SpiDriver, SpiDriverConfig}, prelude::*};

// I2C Example (Default method)
let peripherals = Peripherals::take().unwrap();
let mut accel = Accelerometer::new(
    peripherals.i2c0,
    peripherals.pins.gpio21, // SDA
    peripherals.pins.gpio22, // SCL
    AccelerometerRange::Range16G,
).unwrap();

// SPI Example (Alternative method)
let spi_driver = SpiDriver::new(
    peripherals.spi2,
    peripherals.pins.gpio18, // SCLK
    peripherals.pins.gpio23, // MOSI
    Some(peripherals.pins.gpio19), // MISO
    &SpiDriverConfig::new(),
).unwrap();
let mut accel = Accelerometer::new_spi(
    &spi_driver,
    peripherals.pins.gpio5, // CS
    AccelerometerRange::Range16G,
).unwrap();

// Reading data works the same for both I2C and SPI
let reading = accel.read().unwrap();
println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
println!("Magnitude: {:.3}g", reading.magnitude());

// Power management
accel.sleep().unwrap();  // Put device to sleep to save power
accel.wake().unwrap();   // Wake up for measurements

// Check connection
if accel.is_connected() {
    println!("Accelerometer is connected and responding!");
}
```

### Power Management

The accelerometer supports sleep/wake functionality to conserve power:

- **`sleep()`**: Puts the device in low-power sleep mode. The device stops taking measurements but maintains its configuration.
- **`wake()`**: Wakes the device and puts it in measurement mode. Required before taking readings.

Power management is especially useful in battery-powered applications where the accelerometer isn't needed continuously.

### Continuous Reading

For applications that need continuous acceleration monitoring:

```rust
use crate::accelerometer::run_continuous_reading;

// This runs an infinite loop printing acceleration readings every 50ms
run_continuous_reading().unwrap();
```

## Commands

See connected serial devices, you should see something like
`/dev/cu.usbserial-110`

```shell
ls /dev/tyy.*
```

Run the project with dev profile on the board

```shell
cargo run
```

Run the project in release on the board

```shell
cargo run --release
```
