//! Reset and Clock Control
#![allow(dead_code)]

use core::cmp;

use cast::u32;
use stm32f7x7::{rcc, RCC};

use flash::ACR;
use time::Hertz;

/// LSI clock frequency is approximately 32 KHz.
///
/// NOTE - this is not very accurate.
/// It is recommended to use TIM5 to measure the LSI frequency
/// for accurate
pub const LSI: u32 = 32_000_000;

/// HSI default clock speed is 16 MHz
pub const HSI: u32 = 16_000_000;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other
    /// abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1 { _0: () },
            ahb2: AHB2 { _0: () },
            ahb3: AHB3 { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus 1 (AHB1) registers
    pub ahb1: AHB1,
    /// AMBA High-performance Bus 2 (AHB2) registers
    pub ahb2: AHB2,
    /// AMBA High-performance Bus 3 (AHB3) registers
    pub ahb3: AHB3,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration
    pub cfgr: CFGR,
}

/// AMBA High-performance Bus 1 (AHB1) registers
pub struct AHB1 {
    _0: (),
}

impl AHB1 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
}

pub struct AHB2 {
    _0: (),
}

impl AHB2 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2rstr }
    }
}

/// AMBA High-performance Bus 3 (AHB3) registers
pub struct AHB3 {
    _0: (),
}

impl AHB3 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

/// Clock configuration
pub struct CFGR {
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

impl CFGR {
    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    // @brief  System Clock Configuration
    //         The system Clock is configured as follow :
    //            System Clock source            = PLL (HSE)
    //            SYSCLK(Hz)                     = 216000000
    //            HCLK(Hz)                       = 216000000
    //            AHB Prescaler                  = 1
    //            APB1 Prescaler                 = 4
    //            APB2 Prescaler                 = 2
    //            HSE Frequency(Hz)              = 25000000
    //            PLL_M                          = 8
    //            PLL_N                          = 432
    //            PLL_P                          = 2
    //            PLL_Q                          = 9
    //            PLL_R                          = 7
    //            VDD(V)                         = 3.3
    //            Main regulator output voltage  = Scale1 mode
    //            Flash Latency(WS)              = 7
    //
    // TODO - configs/timeout/result?
    pub fn freeze_max(self, acr: &mut ACR) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };
        let pll_m = 8;
        let pll_n = 432;
        let pll_q = 9;
        let pll_r = 7;

        // enable power control clock
        rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

        // TODO - needed?
        // enable voltage scaling

        // enable HSE oscillator and activate PLL with HSE as source
        rcc.cr.modify(|_, w| w.hseon().set_bit());

        // wait until HSE is ready
        while rcc.cr.read().hserdy().bit() == false {}

        // if the PLL is not used as system clock
        if rcc.cfgr.read().sws().bits() != 0b10 {
            // disable main PLL
            rcc.cr.modify(|_, w| w.pllon().clear_bit());

            // configure main PLL clock source
            rcc.pllcfgr.modify(|_, w| unsafe {
                w
                    // HSE PLL source
                    .pllsrc()
                    .set_bit()
                    .pllm()
                    .bits(pll_m)
                    .plln()
                    .bits(pll_n)
                    // PLLP_DIV2
                    .pllp()
                    .bits(0b00)
                    .pllq()
                    .bits(pll_q)
                    .pllr()
                    .bits(pll_r)
            });

            // enable main PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());

            // wait until PLL is ready
            while rcc.cr.read().pllrdy().bit() == false {}
        }

        // TODO - not neede for voltage scale 1?
        // activate OverDrive
        // HAL_PWREx_EnableOverDrive

        // set flash latency wait states
        acr.acr().modify(|_, w| w.latency().bits(7));

        // TODO - should read back out and check?

        // HCLK config
        // no prescaler
        rcc.cfgr.modify(|_, w| unsafe { w.hpre().bits(0b0000) });

        // SYSCLK config
        // wait for PLL ready
        while rcc.cr.read().pllrdy().bit() == false {}
        // set clock source, PLL
        rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(0b10) });

        // wait for it
        while rcc.cfgr.read().sws().bits() != 0b10 {}

        rcc.cfgr.modify(|_, w| unsafe {
            w
                // PCLK1, DIV4
                .ppre1()
                .bits(0b101)
                // PCLK2, DIV2
                .ppre2()
                .bits(0b100)
        });

        // TODO
        let sysclk = 216_000_000;
        let hclk = sysclk;
        let ppre1: u32 = 4;
        let ppre2: u32 = 2;

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(hclk / ppre1),
            pclk2: Hertz(hclk / ppre2),
            ppre1: ppre1 as _,
            ppre2: ppre2 as _,
            sysclk: Hertz(sysclk),
        }
    }

    /// Freezes the clock configuration, making it effective
    /// TODO - this needs work
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let pllmul = (2 * self.sysclk.unwrap_or(HSI)) / HSI;
        let pllmul = cmp::min(cmp::max(pllmul, 2), 16);
        let pllmul_bits = if pllmul == 2 {
            None
        } else {
            Some(pllmul as u8 - 2)
        };

        let sysclk = pllmul * HSI / 2;

        assert!(sysclk <= 72_000_000);

        let hpre_bits = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3...5 => 0b1001,
                6...11 => 0b1010,
                12...39 => 0b1011,
                40...95 => 0b1100,
                96...191 => 0b1101,
                192...383 => 0b1110,
                _ => 0b1111,
            }).unwrap_or(0b0111);

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        assert!(hclk <= 72_000_000);

        let ppre1_bits = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            }).unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 45_000_000);

        let ppre2_bits = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            }).unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 90_000_000);

        // adjust flash wait states
        acr.acr().write(|w| {
            w.latency().bits(if sysclk <= 24_000_000 {
                0b000
            } else if sysclk <= 48_000_000 {
                0b001
            } else {
                0b010
            })
        });

        let rcc = unsafe { &*RCC::ptr() };
        if let Some(pllmul_bits) = pllmul_bits {
            // use PLL as source
            rcc.cfgr
                .modify(|_, w| unsafe { w.hpre().bits(pllmul_bits) });
            // rcc.cfgr.write(|w| unsafe { w.pllmul().bits(pllmul_bits) });

            // Enable PLL
            rcc.cr.write(|w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}

            // SW: PLL selected as system clock
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(0b10)
            });
        } else {
            // use HSI as source

            // SW: HSI selected as system clock
            rcc.cfgr.write(|w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(0b00)
            });
        }

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no
/// longer be changed
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    // TODO remove `allow`
    #[allow(dead_code)]
    ppre2: u8,
    sysclk: Hertz,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
}

/// TODO
#[derive(Copy, Clone, Debug)]
pub struct ResetConditions {
    pub low_power: bool,
    pub window_watchdog: bool,
    pub independent_watchdog: bool,
    pub software: bool,
    pub por_pdr: bool,
    pub pin: bool,
    pub bor: bool,
}

impl ResetConditions {
    pub fn read_and_clear() -> Self {
        let csr = unsafe { &(*RCC::ptr()).csr };
        let rc = ResetConditions {
            low_power: csr.read().lpwrrstf().bit(),
            window_watchdog: csr.read().wwdgrstf().bit(),
            independent_watchdog: csr.read().wdgrstf().bit(),
            software: csr.read().sftrstf().bit(),
            por_pdr: csr.read().porrstf().bit(),
            pin: csr.read().padrstf().bit(),
            bor: csr.read().borrstf().bit(),
        };

        // clear the reset flags
        csr.modify(|_, w| w.rmvf().set_bit());

        rc
    }
}
