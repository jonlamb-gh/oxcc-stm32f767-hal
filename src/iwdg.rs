/// Independent watchdog
///
/// Note: may need a way to debug freeze the IWDG peripheral
/// during debugging/halted/etc:
///     Disable IWDG if core is halted
///     DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
///     ... continue with enabling IWDG
/// Also requires enabling DBGMCU clock on APB2
use stm32f7x7::IWDG;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WatchdogTimeout {
    /// 20 ms timeout
    Wdto20ms,
    /// 50 ms timeout
    Wdto50ms,
    /// 250 ms timeout
    Wdto250ms,
    /// 500 ms timeout
    Wdto500ms,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Prescaler {
    Prescaler4,
    Prescaler8,
    Prescaler16,
    Prescaler32,
    Prescaler64,
    Prescaler128,
    Prescaler256,
}

/// IWDG reload counter enable
const KEY_RELOAD: u16 = 0xAAAA;

/// IWDG peripheral enable
const KEY_ENABLE: u16 = 0xCCCC;

/// IWDG KR write access enable
const KEY_WRITE_ACCESS_ENABLE: u16 = 0x5555;

/// IWDG KR write access disable
const KEY_WRITE_ACCESS_DISABLE: u16 = 0x0000;

/// Default/reset value of the reload register
const DEFAULT_RELOAD_VALUE: u16 = 0x0FFF;

pub struct IwdgConfig {
    pub reload: u16,
    pub prescaler: Prescaler,
}

pub struct Iwdg<IWDG> {
    iwdg: IWDG,
}

impl Iwdg<IWDG> {
    pub fn new(iwdg: IWDG, config: IwdgConfig) -> Self {
        // enable IWDG, LSI is turned on automatically
        iwdg.kr.write(|w| unsafe { w.key().bits(KEY_ENABLE) });

        // enable write access
        iwdg.kr
            .write(|w| unsafe { w.key().bits(KEY_WRITE_ACCESS_ENABLE) });

        // set the prescaler and reload values
        iwdg.pr
            .write(|w| unsafe { w.pr().bits(u8::from(config.prescaler)) });
        iwdg.rlr
            .write(|w| unsafe { w.rl().bits(config.reload & DEFAULT_RELOAD_VALUE) });

        // TODO - timeout
        // wait for completion
        while iwdg.sr.read().bits() != 0 {}

        // reload IWDG counter
        iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });

        // disable write access
        iwdg.kr
            .write(|w| unsafe { w.key().bits(KEY_WRITE_ACCESS_DISABLE) });

        Iwdg { iwdg }
    }

    pub fn refresh(&self) {
        // reload IWDG counter
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
    }
}

impl From<Prescaler> for u8 {
    fn from(p: Prescaler) -> u8 {
        match p {
            Prescaler::Prescaler4 => 0b000,
            Prescaler::Prescaler8 => 0b001,
            Prescaler::Prescaler16 => 0b010,
            Prescaler::Prescaler32 => 0b011,
            Prescaler::Prescaler64 => 0b100,
            Prescaler::Prescaler128 => 0b101,
            Prescaler::Prescaler256 => 0b110,
        }
    }
}

impl IwdgConfig {
    pub fn new() -> Self {
        IwdgConfig {
            reload: 0xFA,
            prescaler: Prescaler::Prescaler32,
        }
    }
}

/// TODO - this can be calculated, currently it's just taking
/// advantage of the ~32 KHz LSI and prescaler 32 such that
/// the reload value is approximate equal to milliseconds
/// NOTE: based on the approximate LSI frequency, not very accurate
impl From<WatchdogTimeout> for IwdgConfig {
    fn from(to: WatchdogTimeout) -> IwdgConfig {
        match to {
            WatchdogTimeout::Wdto20ms => IwdgConfig {
                reload: 20,
                prescaler: Prescaler::Prescaler32,
            },
            WatchdogTimeout::Wdto50ms => IwdgConfig {
                reload: 50,
                prescaler: Prescaler::Prescaler32,
            },
            WatchdogTimeout::Wdto250ms => IwdgConfig {
                reload: 250,
                prescaler: Prescaler::Prescaler32,
            },
            WatchdogTimeout::Wdto500ms => IwdgConfig {
                reload: 500,
                prescaler: Prescaler::Prescaler32,
            },
        }
    }
}
