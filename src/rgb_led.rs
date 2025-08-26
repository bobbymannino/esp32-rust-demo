use esp_idf_svc::hal::gpio::{Output, OutputPin, PinDriver};
use esp_idf_sys::EspError;

pub struct RgbLed<'a, R, G, B>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
{
    red: PinDriver<'a, R, Output>,
    green: PinDriver<'a, G, Output>,
    blue: PinDriver<'a, B, Output>,
}

impl<'a, R, G, B> RgbLed<'a, R, G, B>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
{
    pub fn new(red: R, green: G, blue: B) -> Result<Self, EspError> {
        let mut r = PinDriver::output(red)?;
        let mut g = PinDriver::output(green)?;
        let mut b = PinDriver::output(blue)?;

        r.set_low()?;
        g.set_low()?;
        b.set_low()?;

        Ok(Self {
            red: r,
            green: g,
            blue: b,
        })
    }

    /// Turns on all 3 pins
    pub fn on(&mut self) -> Result<(), EspError> {
        self.red.set_high()?;
        self.green.set_high()?;
        self.blue.set_high()?;

        Ok(())
    }

    /// Turns off all 3 pins
    pub fn off(&mut self) -> Result<(), EspError> {
        self.red.set_low()?;
        self.green.set_low()?;
        self.blue.set_low()?;

        Ok(())
    }

    pub fn red(&mut self) -> Result<(), EspError> {
        self.red.set_high()?;
        self.green.set_low()?;
        self.blue.set_low()?;

        Ok(())
    }

    pub fn green(&mut self) -> Result<(), EspError> {
        self.red.set_low()?;
        self.green.set_high()?;
        self.blue.set_low()?;

        Ok(())
    }

    pub fn blue(&mut self) -> Result<(), EspError> {
        self.red.set_low()?;
        self.green.set_low()?;
        self.blue.set_high()?;

        Ok(())
    }
}
