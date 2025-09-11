use esp_idf_svc::hal::gpio::{Input, InputPin, OutputPin, PinDriver, Pull};
use esp_idf_sys::EspError;

pub struct Reed<'a, P>
where
    P: OutputPin + InputPin,
{
    pin: PinDriver<'a, P, Input>,
}

impl<'a, P> Reed<'a, P>
where
    P: OutputPin + InputPin,
{
    pub fn new(pin: P) -> Result<Self, EspError> {
        let mut pin = PinDriver::input(pin)?;

        pin.set_pull(Pull::Up)?;

        Ok(Self { pin })
    }

    pub fn is_on(&self) -> bool {
        self.pin.is_high()
    }
}
