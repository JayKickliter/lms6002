#![deny(unsafe_code)]

#[cfg(test)]
#[macro_use]
extern crate quickcheck;

#[macro_use]
extern crate bitfield;

#[macro_use]
extern crate log;

mod algo;
mod error;
mod interface;
mod lms6002;
pub mod reg;

pub use error::*;
pub use interface::*;
pub use lms6002::*;
