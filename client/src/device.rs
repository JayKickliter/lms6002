#[cfg(target_os = "linux")]
mod hidden {
    use spidev::Spidev;
    use std::io;
    use std::path::Path;

    pub struct Device {
        spidev: Spidev,
    }

    impl Device {
        fn open(path: impl AsRef<Path>) -> io::Result<Self> {}
        fn read(&self, addr: u8) -> Result<u8, ()> {}
        fn write(&self, addr: u8, val: u8) -> Result<(), ()> {}
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

#[cfg(not(target_os = "linux"))]
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
