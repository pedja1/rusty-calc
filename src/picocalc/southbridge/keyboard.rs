use crate::picocalc::southbridge::southbridge::{KeyboardState, SouthBridge};
use core::cell::RefCell;
use defmt::debug;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::SpiDevice;
use rp_pico::hal::gpio::bank0::{Gpio6, Gpio7};
use rp_pico::hal::gpio::{FunctionI2c, Pin, PullUp};
use rp_pico::hal::I2C;
use rp_pico::pac::I2C1;
use slint::platform::Key;

// Key constants
const KEY_BACKSPACE: u8 = 0x08;
const KEY_TAB: u8 = 0x09;
const KEY_ENTER: u8 = 0x0A;
const KEY_RETURN: u8 = 0x0D;
const KEY_SPACE: u8 = 0x20;

const KEY_ESC: u8 = 0xB1;
const KEY_UP: u8 = 0xB5;
const KEY_DOWN: u8 = 0xB6;
const KEY_LEFT: u8 = 0xB4;
const KEY_RIGHT: u8 = 0xB7;

const KEY_BREAK: u8 = 0xD0;
const KEY_INSERT: u8 = 0xD1;
const KEY_HOME: u8 = 0xD2;
const KEY_DEL: u8 = 0xD4;
const KEY_END: u8 = 0xD5;
const KEY_PAGE_UP: u8 = 0xD6;
const KEY_PAGE_DOWN: u8 = 0xD7;

const KEY_CAPS_LOCK: u8 = 0xC1;

const KEY_F1: u8 = 0x81;
const KEY_F2: u8 = 0x82;
const KEY_F3: u8 = 0x83;
const KEY_F4: u8 = 0x84;
const KEY_F5: u8 = 0x85;
const KEY_F6: u8 = 0x86;
const KEY_F7: u8 = 0x87;
const KEY_F8: u8 = 0x88;
const KEY_F9: u8 = 0x89;
const KEY_F10: u8 = 0x90;

pub const KEY_POWER: u8 = 0x91;

const KEY_ALT: u8 = 0xA1;
const KEY_SHL: u8 = 0xA2;
const KEY_SHR: u8 = 0xA3;
const KEY_SYM: u8 = 0xA4;
const KEY_CTRL: u8 = 0xA5;

pub fn slint_key_from_u8(code: u8) -> Option<Key> {
    match code {
        KEY_BACKSPACE => Some(Key::Backspace),
        KEY_TAB => Some(Key::Tab),
        KEY_ENTER => Some(Key::Return),
        KEY_RETURN => Some(Key::Return),
        KEY_SPACE => Some(Key::Space),
        KEY_ESC => Some(Key::Escape),
        KEY_UP => Some(Key::UpArrow),
        KEY_DOWN => Some(Key::DownArrow),
        KEY_LEFT => Some(Key::LeftArrow),
        KEY_RIGHT => Some(Key::RightArrow),
        KEY_BREAK => Some(Key::Pause),
        KEY_INSERT => Some(Key::Insert),
        KEY_HOME => Some(Key::Home),
        KEY_DEL => Some(Key::Delete),
        KEY_END => Some(Key::End),
        KEY_PAGE_UP => Some(Key::PageUp),
        KEY_PAGE_DOWN => Some(Key::PageDown),
        KEY_CAPS_LOCK => Some(Key::CapsLock),
        KEY_F1 => Some(Key::F1),
        KEY_F2 => Some(Key::F2),
        KEY_F3 => Some(Key::F3),
        KEY_F4 => Some(Key::F4),
        KEY_F5 => Some(Key::F5),
        KEY_F6 => Some(Key::F6),
        KEY_F7 => Some(Key::F7),
        KEY_F8 => Some(Key::F8),
        KEY_F9 => Some(Key::F9),
        KEY_F10 => Some(Key::F10),
        KEY_SHL => Some(Key::Shift),
        KEY_SHR => Some(Key::ShiftR),
        KEY_CTRL => Some(Key::Control),
        KEY_ALT => Some(Key::Alt),
        // KEY_POWER => Some(Key::Power), // not defined in slint
        _ => None,
    }
}