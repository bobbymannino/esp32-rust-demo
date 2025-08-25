use esp_idf_svc::hal::gpio::{Output, OutputPin, PinDriver};
use esp_idf_sys::EspError;

pub struct Led<'a, P>
where
    P: OutputPin,
{
    pin: PinDriver<'a, P, Output>,
    is_on: bool,
}

impl<'a, P> Led<'a, P>
where
    P: OutputPin,
{
    pub fn new(pin: P) -> Result<Led<'a, P>, EspError> {
        let mut pin = PinDriver::output(pin)?;

        pin.set_low()?;

        Ok(Self { pin, is_on: false })
    }

    pub fn on(&mut self) -> Result<(), EspError> {
        self.pin.set_high()?;
        self.is_on = true;
        Ok(())
    }

    pub fn off(&mut self) -> Result<(), EspError> {
        self.pin.set_low()?;
        self.is_on = false;
        Ok(())
    }

    pub fn toggle(&mut self) -> Result<(), EspError> {
        if self.is_on {
            self.off()
        } else {
            self.on()
        }
    }

    pub fn is_on(&self) -> bool {
        self.is_on
    }
}
