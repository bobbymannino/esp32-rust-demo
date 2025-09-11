use embedded_hal::i2c::I2c;
use log::info;

const WIDTH: usize = 128;
const HEIGHT: usize = 128;

const BUFFER_SIZE: usize = WIDTH * HEIGHT / 8;
const PAGES: usize = HEIGHT / 8;

pub struct SH1107 {
    /// the buffer is an array of pages, each page is 8 pixels high and 128 pixels wide
    /// if you want to change px at 20, 20 you would set bit 20 % 8 = 4 in page 20 / 8 = 2
    buffer: [u8; BUFFER_SIZE],
}

impl SH1107 {
    pub fn new() -> Self {
        Self {
            buffer: [0x00; BUFFER_SIZE],
        }
    }

    pub fn set_px(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }

        // the 128x128 screen is split into pages
        // each page is 8 pixels high and 128 pixels wide

        let page = y / 8; // integer division
        let bit = y % 8; // remainder only
        let idx = page * WIDTH + x;
        let mask = 1 << bit;

        if on {
            // bitwise or
            self.buffer[idx] |= mask;
        } else {
            // bitwise and
            self.buffer[idx] &= !mask;
        }
    }

    pub fn get_buffer(&self) -> &[u8; BUFFER_SIZE] {
        &self.buffer
    }
}

impl SH1107 {
    pub fn flush<I2C>(&self, i2c: &mut I2C, address: u8) -> Result<(), I2C::Error>
    where
        I2C: I2c,
    {
        for page in 0..PAGES {
            // Set page address
            i2c.write(address, &[0x00, (0xB0 | page as u8)])?;
            // Set column address to 0
            i2c.write(address, &[0x00, 0x10])?; // high nibble = 0
            i2c.write(address, &[0x00, 0x00])?; // low nibble = 0

            // Slice of 128 bytes for this page
            let start = page * WIDTH;
            let end = start + WIDTH;
            let page_data = &self.buffer[start..end];

            // Control byte 0x40 = "data stream follows"
            let mut buf = [0u8; WIDTH + 1];
            buf[0] = 0x40;
            buf[1..].copy_from_slice(page_data);

            i2c.write(address, &buf)?;

            // turn display on
            i2c.write(address, &[0x00, 0xaf])
                .expect("failed to turn display on"); // turn display on
        }

        Ok(())
    }
}
