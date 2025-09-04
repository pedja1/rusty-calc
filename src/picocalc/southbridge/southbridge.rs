use defmt::debug;
use embedded_hal::i2c::I2c;

enum SbRegister {
    Fw = 0x00,
    FiFo = 0x09,
}

enum SbAddress {
    SouthBridge = 0x1f,
}

#[derive(Debug, Clone, Copy)]
pub enum KeyboardState {
    Idle = 0,
    Pressed = 1,
    Hold = 2,
    Released = 3,
}

pub struct KeyboardKey {
    pub key_state: KeyboardState,
    pub key_code: u8,
}

pub struct SouthBridge<I2C>
where
    I2C: I2c,
{
    /// I2C
    i2c: I2C,
    kb_value: [u8; 2],
}

impl<I2C> SouthBridge<I2C>
where
    I2C: I2c,
{
    /// Creates a new driver instance that uses hardware I2C.
    pub fn new(
        i2c: I2C,
    ) -> Self {
        Self {
            i2c,
            kb_value: [0u8; 2]
        }
    }

    /// Runs commands to initialize the southbridge.
    pub fn init(&mut self) -> Result<(), ()> {
        // nothing for now
        Ok(())
    }

    pub fn read_keyboard_key(&mut self) -> Result<KeyboardKey, ()> {
        //TODO mutex guard
        self.i2c
            .write_read(SbAddress::SouthBridge as u8, &[SbRegister::FiFo as u8], &mut self.kb_value)
            .map_err(|_| ()).map(|_| KeyboardKey {
            key_state: self.kb_value[0].into(),
            key_code: self.kb_value[1],
        })
    }

    pub fn read_firmware_version(&mut self) -> Result<u8, ()> {
        let mut fw_value = [0u8; 1];
        self.i2c
            .write_read(SbAddress::SouthBridge as u8, &[SbRegister::Fw as u8], &mut fw_value)
            .map_err(|_| ())?;
        Ok(fw_value[0])
    }
}

impl From<u8> for KeyboardState {
    fn from(value: u8) -> Self {
        match value {
            1 => KeyboardState::Pressed,
            2 => KeyboardState::Hold,
            3 => KeyboardState::Released,
            0 | _ => KeyboardState::Idle,
        }
    }
}
