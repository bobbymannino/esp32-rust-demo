# ESP32 Rust Demo

A very simple demo of programming an ESP32 in Rust

## Components

This demo includes drivers for:
- Button input (GPIO19)
- RGB LED output (GPIO16, GPIO5, GPIO17)
- Reed switch input
- **ADXL345 Accelerometer** - 3-axis accelerometer via I2C

## Accelerometer (ADXL345) Usage

The accelerometer module provides a clean, object-oriented API for interfacing with the ADXL345 3-axis accelerometer over I2C.

### Hardware Connections
- **VCC**: 3.3V
- **GND**: Ground  
- **SDA**: GPIO21 (I2C data line)
- **SCL**: GPIO22 (I2C clock line)

### Code Examples

```rust
use crate::accelerometer::{Accelerometer, AccelerationReading};

// Initialize accelerometer
let peripherals = Peripherals::take().unwrap();
let mut accel = Accelerometer::new(
    peripherals.i2c0,
    peripherals.pins.gpio21, // SDA
    peripherals.pins.gpio22, // SCL
).unwrap();

// Read acceleration data
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

### Autosleep Feature

The main application now includes intelligent autosleep functionality:

- **Motion Threshold**: Automatically wakes up when acceleration exceeds **1.5g**
- **Inactivity Timeout**: Goes to sleep after 5 seconds of motion below threshold
- **Sleep Monitoring**: Checks for motion every 1 second while sleeping
- **Power Savings**: Significantly reduces power consumption during periods of low activity

The autosleep feature provides optimal power efficiency while ensuring responsive motion detection above the 1.5g threshold.

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
