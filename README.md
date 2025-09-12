# ESP32 Rust Demo

A very simple demo of programming an ESP32 in Rust

## Components

This demo includes drivers for:
- Button input (GPIO19)
- RGB LED output (GPIO16, GPIO5, GPIO17)
- Reed switch input
- **ADXL345 Accelerometer** - 3-axis accelerometer via I2C
- **GY521 (MPU6050) IMU** - 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) with temperature sensor via I2C

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

### Continuous Reading

For applications that need continuous acceleration monitoring:

```rust
use crate::accelerometer::run_continuous_reading;

// This runs an infinite loop printing acceleration readings every 50ms
run_continuous_reading().unwrap();
```

## GY521 (MPU6050) IMU Usage

The GY521 module provides a clean, object-oriented API for interfacing with the MPU6050 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) with built-in temperature sensor over I2C.

### Hardware Connections
- **VCC**: 3.3V or 5V (GY521 module has voltage regulation)
- **GND**: Ground
- **SDA**: GPIO21 (I2C data line)
- **SCL**: GPIO22 (I2C clock line)
- **AD0**: Usually connected to GND on GY521 modules (sets I2C address to 0x68)

### Code Examples

```rust
use crate::gy521::GY521;

// Initialize GY521 with default ranges
let peripherals = Peripherals::take().unwrap();
let mut gy521 = GY521::new(
    peripherals.i2c0,
    peripherals.pins.gpio21, // SDA
    peripherals.pins.gpio22, // SCL
).unwrap();

// Read rotation data (gyroscope)
let rotation = gy521.read_rotation().unwrap();
println!("Rotation - X: {:.2}°/s, Y: {:.2}°/s, Z: {:.2}°/s", 
         rotation.x, rotation.y, rotation.z);
println!("Rotation Magnitude: {:.2}°/s", rotation.magnitude());

// Read movement data (accelerometer)  
let movement = gy521.read_movement().unwrap();
println!("Movement - X: {:.2}g, Y: {:.2}g, Z: {:.2}g", 
         movement.x, movement.y, movement.z);
println!("Movement Magnitude: {:.2}g", movement.magnitude());

// Read temperature
let temp = gy521.read_temperature().unwrap();
println!("Temperature: {:.1}°C", temp);

// Power management
gy521.sleep().unwrap();  // Put device to sleep to save power
gy521.wake().unwrap();   // Wake up for measurements

// Check connection
if gy521.is_connected() {
    println!("GY521 is connected and responding!");
}
```

### Advanced Configuration

```rust
use crate::gy521::{GY521, AccelRange, GyroRange};

// Initialize with custom ranges for high-performance applications
let mut gy521 = GY521::new_with_ranges(
    peripherals.i2c0,
    peripherals.pins.gpio21,
    peripherals.pins.gpio22,
    AccelRange::Range8G,    // ±8g accelerometer range
    GyroRange::Range1000,   // ±1000°/s gyroscope range
).unwrap();
```

### Continuous Reading

For applications that need continuous IMU monitoring:

```rust
use crate::gy521::run_continuous_reading;

// This runs an infinite loop printing rotation, movement, and temperature every 100ms
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
