#[cfg(feature = "spi")]
mod hidden {
    use spidev::{SpiTransfer, Spidev};
    use std::io;
    use std::path::Path;

    pub struct Device {
        spidev: Spidev,
    }

    impl Device {
        fn open(path: impl AsRef<Path>) -> io::Result<Self> {
            let spidev = Spidev::open(path)?;
            let mut options = SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(4_000_000)
                .mode(SPI_MODE_0);
            spidev.configure(&options)?;
            Ok(Self { spidev })
        }
        fn read(&self, addr: u8) -> Result<u8, ()> {
            let mut buf = [0u8; 2];
            let mut xfer = SpidevTransfer::write(&buf);
            try!(spi.transfer(&mut transfer));
            println!("{:?}", transfer.rx_buf);
        }
        fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            let mut buf = [0u8; 2];
        }
    }

    impl ::lms6002::Interface for Device {
        fn read(&self, addr: u8) -> Result<u8, ()> {
            self.read(addr)
        }

        fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            self.write(addr, val)
        }
    }
}

#[cfg(not(feature = "spi"))]
mod hidden {
    use std::cell::RefCell;
    use std::fs::File;
    use std::io;
    use std::path::Path;

    pub struct Device(RefCell<File>);

    impl Device {
        pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
            let f = ::std::fs::OpenOptions::new()
                .create(true)
                .read(true)
                .write(true)
                .open(path)?;
            Ok(Device(RefCell::new(f)))
        }

        pub fn read(&self, addr: u8) -> Result<u8, ()> {
            use std::io::{Read, Seek, SeekFrom};
            let mut f = self.0.borrow_mut();
            f.seek(SeekFrom::Start(addr as u64)).or(Err(()))?;
            let mut buf = [0xfe; 1];
            f.read(&mut buf).or(Err(()))?;
            Ok(buf[0])
        }

        pub fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            use std::io::{Seek, SeekFrom, Write};
            let mut f = self.0.borrow_mut();
            f.seek(SeekFrom::Start(addr as u64)).or(Err(()))?;
            let buf = [val; 1];
            f.write(&buf).or(Err(()))?;
            Ok(())
        }
    }

    impl ::lms6002::Interface for Device {
        fn read(&self, addr: u8) -> Result<u8, ()> {
            self.read(addr)
        }

        fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            self.write(addr, val)
        }
    }
}

pub use self::hidden::*;
