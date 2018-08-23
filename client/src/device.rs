pub use self::detail::*;

#[cfg(feature = "spi")]
mod detail {
    use spidev::*;
    use std::io;
    use std::path::Path;

    pub struct Device {
        spi: Spidev,
    }

    impl Device {
        pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
            let mut spi = Spidev::open(path)?;
            let mut options = SpidevOptions::new();
            options
                .bits_per_word(8)
                .max_speed_hz(4_000_000)
                .mode(SPI_MODE_0);
            spi.configure(&options)?;
            Ok(Self { spi })
        }

        pub fn read(&self, addr: u8) -> Result<u8, ()> {
            let tx_buf = [addr];
            let mut rx_buf = [0; 2];
            {
                let mut xfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
                self.spi.transfer(&mut xfer).or(Err(()))?;
            }
            Ok(rx_buf[1])
        }

        pub fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            let tx_buf = [
                // Set upper bit to 1 to indicate a write operation.
                addr & 0b1000_0000,
                val,
            ];
            let mut xfer = SpidevTransfer::write(&tx_buf);
            self.spi.transfer(&mut xfer).or(Err(()))?;
            Ok(())
        }
    }
}

#[cfg(not(feature = "spi"))]
mod detail {
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
            f.seek(SeekFrom::Start(u64::from(addr))).or(Err(()))?;
            let mut buf = [0xfe; 1];
            f.read(&mut buf).or(Err(()))?;
            Ok(buf[0])
        }

        pub fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
            use std::io::{Seek, SeekFrom, Write};
            let mut f = self.0.borrow_mut();
            f.seek(SeekFrom::Start(u64::from(addr))).or(Err(()))?;
            let buf = [val; 1];
            f.write(&buf).or(Err(()))?;
            Ok(())
        }
    }
}

impl ::lms6002::Interface for Device {
    fn read(&self, addr: u8) -> Result<u8, ()> {
        assert!(addr < 128);
        self.read(addr)
    }

    fn write(&self, addr: u8, val: u8) -> Result<(), ()> {
        assert!(addr < 128);
        self.write(addr, val)
    }
}
