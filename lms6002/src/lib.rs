#![deny(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate core as std;

#[cfg(all(test, feature = "std"))]
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
