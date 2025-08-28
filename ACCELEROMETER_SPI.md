# SPI Support for ADXL345 Accelerometer

This document describes the newly added SPI support for the ADXL345 accelerometer driver.

## Summary of Changes

The accelerometer driver now supports both I2C and SPI communication protocols with the ADXL345 accelerometer chip.

### New Methods Added

As requested, **2 new methods** have been added:

1. **`new_spi()`** - Creates an accelerometer instance using SPI with default ±2g range
2. **`new_spi_with_range()`** - Creates an accelerometer instance using SPI with specified range

### API Changes

#### Before (I2C only):
```rust
let accel = Accelerometer::new(i2c_peripheral, sda_pin, scl_pin, range)?;
```

#### After (I2C + SPI):
```rust
// I2C (unchanged)
let accel = Accelerometer::new(i2c_peripheral, sda_pin, scl_pin, range)?;

// SPI (new methods)
let accel = Accelerometer::new_spi(spi_peripheral, mosi_pin, miso_pin, sclk_pin, cs_pin)?;
let accel = Accelerometer::new_spi_with_range(spi_peripheral, mosi_pin, miso_pin, sclk_pin, cs_pin, range)?;
```

### Hardware Connections

#### I2C Mode (existing):
- VCC → 3.3V
- GND → GND  
- SDA → GPIO21
- SCL → GPIO22

#### SPI Mode (new):
- VCC → 3.3V
- GND → GND
- SDO (MISO) → GPIO19
- SDA (MOSI) → GPIO23  
- SCL (SCLK) → GPIO18
- CS → GPIO5

### Technical Implementation

1. **Communication Abstraction**: Created `AccelerometerInterface` enum to handle both I2C and SPI protocols
2. **Register Access**: Added helper methods that automatically handle the different register addressing schemes:
   - I2C: Direct register addresses
   - SPI: Bit-formatted addresses (bit 7 for read/write, bit 6 for multi-byte)
3. **Protocol Support**: All existing methods (read, read_raw, sleep, wake, is_connected) work with both protocols
4. **SPI Configuration**: Uses 1MHz clock with appropriate mode settings for ADXL345 compatibility

### Usage Examples

```rust
use crate::accelerometer::{Accelerometer, AccelerometerRange};

// SPI with default ±2g range
let mut accel = Accelerometer::new_spi(
    peripherals.spi2,
    peripherals.pins.gpio23, // MOSI
    peripherals.pins.gpio19, // MISO  
    peripherals.pins.gpio18, // SCLK
    peripherals.pins.gpio5,  // CS
)?;

// SPI with ±16g range for high accelerations
let mut accel = Accelerometer::new_spi_with_range(
    peripherals.spi2,
    peripherals.pins.gpio23, // MOSI
    peripherals.pins.gpio19, // MISO
    peripherals.pins.gpio18, // SCLK
    peripherals.pins.gpio5,  // CS
    AccelerometerRange::Range16G,
)?;

// Reading data works the same for both I2C and SPI
let reading = accel.read()?;
println!("X: {:.3}g, Y: {:.3}g, Z: {:.3}g", reading.x, reading.y, reading.z);
```

### Backwards Compatibility

All existing I2C functionality remains unchanged. Existing code will continue to work without modifications.

### Testing

- Added API validation tests in `test_accelerometer_api.rs`
- Created example usage patterns for both protocols
- Updated documentation with SPI examples
- Added convenience function `run_continuous_reading_spi()` for demo purposes

This implementation provides a clean, unified interface for ADXL345 communication via either I2C or SPI, giving developers flexibility in their hardware designs while maintaining the same high-level API.