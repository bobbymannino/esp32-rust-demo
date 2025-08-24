use esp_idf_svc::hal::{
    delay::BLOCK,
    i2c::{I2cConfig, I2cDriver},
    prelude::Peripherals,
    units::Hertz,
};
use std::{thread, time::Duration};

// find this address from the I2C section in the pdf listed below
const ADXL345_ADDR: u8 = 0x53;

// ADXL345 register addresses
const REG_POWER_CTL: u8 = 0x2d;
const POWER_CTL_WAKEUP: u8 = 0x2;
const POWER_CTL_SLEEP: u8 = 0x4;
const POWER_CTL_MEASURE: u8 = 0x08;

// The register addresses for the x/y/z data, can find in:
// analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
const REG_DATAX0: u8 = 0x32;
const REG_DATAY0: u8 = 0x34;
const REG_DATAZ0: u8 = 0x36;

/// ESP32 & ADXL345
///
/// 3.3V and GND are connected.
/// SDA on the ADXL is connected to GPIO21 on the ESP
/// SCL on the ADXL is connected to GPIO22 on the EPS

pub fn setup() {
    let peripherals = Peripherals::take().unwrap();

    let scl = peripherals.pins.gpio22;
    let sda = peripherals.pins.gpio21;

    // Sets the rate of the I2C bus (data transfer speed)
    let i2c_cfg = I2cConfig::new().baudrate(Hertz(400_000));
    let mut i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_cfg).unwrap();

    set_to_measure(&mut i2c);

    loop {
        let (x, y, z) = read_accelerometer(&mut i2c);
        println!("X = {:.3}, Y = {:.3}, Z = {:.3}", x, y, z);

        // Slow down logging a little
        thread::sleep(Duration::from_millis(50));
    }
}

fn set_to_measure(i2c: &mut I2cDriver) {
    // set the value of address: REG_POWER_CTL to POWER_CTL_MEASURE
    // BLOCK means the operation will block until the write is complete
    // ADXL345_ADDR is the address of the ADXL345 accelerometer
    i2c.write(ADXL345_ADDR, &[REG_POWER_CTL, POWER_CTL_MEASURE], BLOCK)
        .unwrap();
}

fn read_accelerometer(i2c: &mut I2cDriver) -> (f32, f32, f32) {
    let mut x_data: [u8; 2] = [0; 2];
    let mut y_data: [u8; 2] = [0; 2];
    let mut z_data: [u8; 2] = [0; 2];

    i2c.write_read(ADXL345_ADDR, &[REG_DATAX0], &mut x_data, BLOCK)
        .unwrap();
    i2c.write_read(ADXL345_ADDR, &[REG_DATAY0], &mut y_data, BLOCK)
        .unwrap();
    i2c.write_read(ADXL345_ADDR, &[REG_DATAZ0], &mut z_data, BLOCK)
        .unwrap();

    let x = i16::from_le_bytes(x_data);
    let y = i16::from_le_bytes(y_data);
    let z = i16::from_le_bytes(z_data);

    let x = x as f32;
    let y = y as f32;
    let z = z as f32;

    (x, y, z)
}
