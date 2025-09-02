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
pub const KEY_BACKSPACE: u8 = 0x08;
pub const KEY_TAB: u8 = 0x09;
pub const KEY_ENTER: u8 = 0x0A;
pub const KEY_RETURN: u8 = 0x0D;
pub const KEY_SPACE: u8 = 0x20;

pub const KEY_ESC: u8 = 0xB1;
pub const KEY_UP: u8 = 0xB5;
pub const KEY_DOWN: u8 = 0xB6;
pub const KEY_LEFT: u8 = 0xB4;
pub const KEY_RIGHT: u8 = 0xB7;

pub const KEY_BREAK: u8 = 0xD0;
pub const KEY_INSERT: u8 = 0xD1;
pub const KEY_HOME: u8 = 0xD2;
pub const KEY_DEL: u8 = 0xD4;
pub const KEY_END: u8 = 0xD5;
pub const KEY_PAGE_UP: u8 = 0xD6;
pub const KEY_PAGE_DOWN: u8 = 0xD7;

pub const KEY_CAPS_LOCK: u8 = 0xC1;

pub const KEY_F1: u8 = 0x81;
pub const KEY_F2: u8 = 0x82;
pub const KEY_F3: u8 = 0x83;
pub const KEY_F4: u8 = 0x84;
pub const KEY_F5: u8 = 0x85;
pub const KEY_F6: u8 = 0x86;
pub const KEY_F7: u8 = 0x87;
pub const KEY_F8: u8 = 0x88;
pub const KEY_F9: u8 = 0x89;
pub const KEY_F10: u8 = 0x90;

pub const KEY_POWER: u8 = 0x91;

const KEYBOARD_MODIFIER_ALT: u8 = 0xA1;
const KEYBOARD_MODIFIER_SHL: u8 = 0xA2;
const KEYBOARD_MODIFIER_SHR: u8 = 0xA3;
const KEYBOARD_MODIFIER_SYM: u8 = 0xA4;
const KEYBOARD_MODIFIER_CTRL: u8 = 0xA5;

pub fn slint_key_from_u8(code: u8) -> Key {
    match code {
        KEY_BACKSPACE => Key::Backspace,
        KEY_TAB => Key::Tab,
        KEY_ENTER => Key::Return,
        KEY_RETURN => Key::Return,
        KEY_SPACE => Key::Space,
        KEY_ESC => Key::Escape,
        KEY_UP => Key::UpArrow,
        KEY_DOWN => Key::DownArrow,
        KEY_LEFT => Key::LeftArrow,
        KEY_RIGHT => Key::RightArrow,
        KEY_BREAK => Key::Pause,
        KEY_INSERT => Key::Insert,
        KEY_HOME => Key::Home,
        KEY_DEL => Key::Delete,
        KEY_END => Key::End,
        KEY_PAGE_UP => Key::PageUp,
        KEY_PAGE_DOWN => Key::PageDown,
        KEY_CAPS_LOCK => Key::CapsLock,
        KEY_F1 => Key::F1,
        KEY_F2 => Key::F2,
        KEY_F3 => Key::F3,
        KEY_F4 => Key::F4,
        KEY_F5 => Key::F5,
        KEY_F6 => Key::F6,
        KEY_F7 => Key::F7,
        KEY_F8 => Key::F8,
        KEY_F9 => Key::F9,
        KEY_F10 => Key::F10,
        KEY_POWER => Key::Escape, // Map power key to Escape as fallback
        _ => Key::Escape, // Default fallback for unknown keys
    }
}