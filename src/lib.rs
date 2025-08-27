#![cfg_attr(context = "mcu", no_std)]

extern crate alloc;

pub mod controller;
pub mod slint_backend;

//pub mod log;

#[cfg(context = "mcu")]
pub mod picocalc;

#[cfg(context = "mcu")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(context = "simulator")]
pub mod simulator;

#[cfg(context = "simulator")]
pub use log::{debug, error, info, trace, warn};