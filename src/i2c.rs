// https://github.com/astro/stm32f429-hal/blob/master/src/i2c.rs
//
// https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/i2c.rs
//
// https://github.com/therealprof/stm32f767-hal/blob/master/src/i2c.rs

use core::cmp;
use gpio::gpiob::{PB6, PB7, PB8, PB9};
use gpio::AF4;
use hal::blocking::i2c::{Write, WriteRead};
use rcc::{Clocks, APB1};
use stm32f7x7::{I2C1, RCC};
use time::{KiloHertz, U32Ext};

/// I2C abstraction
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

pub trait Pins<I2c> {}

// SclPin, SdaPin
impl Pins<I2C1> for (PB6<AF4>, PB7<AF4>) {}
impl Pins<I2C1> for (PB8<AF4>, PB9<AF4>) {}

#[derive(Debug)]
pub enum Error {
    OVERRUN,
    NACK,
}

impl<PINS> I2c<I2C1, PINS> {
    pub fn i2c1(i2c: I2C1, pins: PINS, speed: KiloHertz, _clocks: Clocks, apb: &mut APB1) -> Self
    where
        PINS: Pins<I2C1>,
    {
        // Disable clock, set clock source to HSI 16 MHz
        apb.enr().modify(|_, w| w.i2c1en().clear_bit());
        let rcc = unsafe { &*RCC::ptr() };
        rcc.dkcfgr2.modify(|_, w| unsafe { w.i2c1sel().bits(0b10) });

        // Enable clock for I2C1
        apb.enr().modify(|_, w| w.i2c1en().set_bit());

        // Reset I2C1
        apb.rstr().modify(|_, w| w.i2c1rst().set_bit());
        apb.rstr().modify(|_, w| w.i2c1rst().clear_bit());

        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // Calculate settings for I2C speed modes
        let presc;
        let scldel;
        let sdadel;
        let sclh;
        let scll;

        // We're using HSI here which runs at a fixed 16MHz
        const FREQ: u32 = 16_000_000;

        // Normal I2C speeds use a different scaling than fast mode below
        if speed <= 100_u32.khz() {
            presc = 3;
            scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
            sclh = scll - 4;
            sdadel = 2;
            scldel = 4;
        } else {
            presc = 1;
            scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
            sclh = scll - 6;
            sdadel = 2;
            scldel = 3;
        }

        // Enable I2C signal generator, and configure I2C for 400KHz full speed
        i2c.timingr.write(|w| unsafe {
            w.presc()
                .bits(presc)
                .scldel()
                .bits(scldel)
                .sdadel()
                .bits(sdadel)
                .sclh()
                .bits(sclh)
                .scll()
                .bits(scll)
        });

        // Enable the I2C processing
        i2c.cr1.modify(|_, w| w.pe().set_bit());

        I2c { i2c, pins }
    }

    pub fn release(self) -> (I2C1, PINS) {
        (self.i2c, self.pins)
    }

    fn send_byte(&self, byte: &u8) -> Result<(), Error> {
        // Wait until we're ready for sending
        while self.i2c.isr.read().txis().bit_is_clear() {}

        // Push out a byte of data
        self.i2c.txdr.write(|w| unsafe { w.bits(u32::from(*byte)) });

        // If we received a NACK, then this is an error
        if self.i2c.isr.read().nackf().bit_is_set() {
            self.i2c
                .icr
                .write(|w| w.stopcf().set_bit().nackcf().set_bit());
            return Err(Error::NACK);
        }

        Ok(())
    }

    fn recv_byte(&self) -> Result<u8, Error> {
        while self.i2c.isr.read().rxne().bit_is_clear() {}
        let value = self.i2c.rxdr.read().bits() as u8;
        Ok(value)
    }
}

impl<PINS> WriteRead for I2c<I2C1, PINS> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // Set up current address, we're trying a "read" command and not going to set
        // anything and make sure we end a non-NACKed read (i.e. if we found a
        // device) properly
        self.i2c.cr2.modify(|_, w| unsafe {
            w.sadd()
                .bits(addr as u16)
                .nbytes()
                .bits(bytes.len() as u8)
                .rd_wrn()
                .clear_bit()
                .autoend()
                .clear_bit()
        });

        // Send a START condition
        self.i2c.cr2.modify(|_, w| w.start().set_bit());

        // Wait until the transmit buffer is empty and there hasn't been either a NACK
        // or STOP being received
        let mut isr;
        while {
            isr = self.i2c.isr.read();
            isr.txis().bit_is_clear()
                && isr.nackf().bit_is_clear()
                && isr.stopf().bit_is_clear()
                && isr.tc().bit_is_clear()
        } {}

        // If we received a NACK, then this is an error
        if isr.nackf().bit_is_set() {
            self.i2c
                .icr
                .write(|w| w.stopcf().set_bit().nackcf().set_bit());
            return Err(Error::NACK);
        }

        for c in bytes {
            self.send_byte(c)?;
        }

        // Wait until data was sent
        while self.i2c.isr.read().tc().bit_is_clear() {}

        // Set up current address, we're trying a "read" command and not going to set
        // anything and make sure we end a non-NACKed read (i.e. if we found a
        // device) properly
        self.i2c.cr2.modify(|_, w| unsafe {
            w.sadd()
                .bits(addr as u16)
                .nbytes()
                .bits(buffer.len() as u8)
                .rd_wrn()
                .set_bit()
        });

        // Send a START condition
        self.i2c.cr2.modify(|_, w| w.start().set_bit());

        // Send the autoend after setting the start to get a restart
        self.i2c.cr2.modify(|_, w| w.autoend().set_bit());

        // Read in all bytes
        for c in buffer.iter_mut() {
            *c = self.recv_byte()?;
        }

        // Clear flags if they somehow ended up set
        self.i2c
            .icr
            .write(|w| w.stopcf().set_bit().nackcf().set_bit());

        Ok(())
    }
}

impl<PINS> Write for I2c<I2C1, PINS> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Set up current address, we're trying a "read" command and not going to set
        // anything and make sure we end a non-NACKed read (i.e. if we found a
        // device) properly
        self.i2c.cr2.modify(|_, w| unsafe {
            w.sadd()
                .bits(addr as u16)
                .nbytes()
                .bits(bytes.len() as u8)
                .rd_wrn()
                .clear_bit()
                .autoend()
                .set_bit()
        });

        // Send a START condition
        self.i2c.cr2.modify(|_, w| w.start().set_bit());

        for c in bytes {
            self.send_byte(c)?;
        }

        // Fallthrough is success
        self.i2c
            .icr
            .write(|w| w.stopcf().set_bit().nackcf().set_bit());
        Ok(())
    }
}
