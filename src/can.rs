// This is more or less a copy of the STM32 HAL driver
// https://github.com/jonlamb-gh/STM32Cube_FW_F7_V1.8.0/blob/master/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_can.c
//
// TODO
// - assert_eq!(self.can.esr.read().bits(), 0), check for error registers, when?
// - add rx/tx timeouts?, currently blocking
// - error/result types
// - provide rx timestamp/counter?
// - do something with the rx filter match index?
// - conversion (using clock) data rate to/from bit timing
// - macro out the defintions for CANX
#![allow(dead_code)]

use gpio::gpiob::{PB12, PB13};
use gpio::gpiod::{PD0, PD1};
use gpio::AF9;
use rcc::APB1;
use stm32f7x7::{can1, CAN1, CAN2};

// use time::Hertz;

pub use embedded_types::can::{
    BaseID, CanFrame, DataFrame, ExtendedDataFrame, ExtendedID, RemoteFrame, ID,
};

// TODO
// currently only used in the Tx logic to wait for completion
// that would normally lock up if the bus wasn't connected for example
// 10 us at 16 MHz
pub const MAX_BLOCK_TICKS: u32 = 16 * 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanError {
    BufferExhausted,
    ConfigurationFailed,
    InvalidFrame,
    Timeout,
}

pub struct CanConfig {
    /// Enable or disable loop back mode (debug).
    pub loopback_mode: bool,

    /// Enable or disable silent mode (debug).
    pub silent_mode: bool,

    /// Enable or disable the time triggered communication mode.
    pub ttcm: bool,

    /// Enable or disable the automatic bus-off management.
    pub abom: bool,

    /// Enable or disable the automatic wake-up mode.
    pub awum: bool,

    /// Enable or disable the non-automatic retransmission mode.
    pub nart: bool,

    /// Enable or disable the receive FIFO Locked mode.
    pub rflm: bool,

    /// Enable or disable the transmit FIFO priority.
    pub txfp: bool,

    // TODO - to/from time::Hertz
    pub bit_timing: CanBitTiming,
}

impl Default for CanConfig {
    fn default() -> Self {
        CanConfig {
            loopback_mode: false,
            silent_mode: false,
            ttcm: false,
            abom: true,
            awum: false,
            nart: false,
            rflm: false,
            txfp: false,
            // 500K, assumes 216 MHz system clock /= 4 = 54 MHz pclk1
            bit_timing: CanBitTiming {
                prescaler: 5, // 6
                sjw: 0,       // CAN_SJW_1TQ
                bs1: 14,      // CAN_BS1_15TQ
                bs2: 1,       // CAN_BS2_2TQ
            },
        }
    }
}

pub struct CanBitTiming {
    /// Specifies the length of a time quantum.
    pub prescaler: u16,

    /// Specifies the maximum number of time quanta
    /// the CAN hardware is allowed to lengthen or
    /// shorten a bit to perform resynchronization.
    pub sjw: u8,

    /// Specifies the number of time quanta in Bit Segment 1.
    pub bs1: u8,

    /// Specifies the number of time quanta in Bit Segment 2.
    pub bs2: u8,
}

pub enum TxMailbox {
    Mailbox0,
    Mailbox1,
    Mailbox2,
}

pub enum RxFifo {
    Fifo0,
    Fifo1,
}

pub enum FilterMode {
    IdMask,
    IdList,
}

pub enum FilterScale {
    Fs16Bit,
    Fs32Bit,
}

/// NOTE: for 16 bit ID list mode filters, ID needs to be shifted left by 5.
///
/// Example:
/// filter.scale = FilterScale::Fs16Bit;
/// filter.filter_mask_id_low = 0x22 << 5;
/// filter.filter_id_low = 0x23 << 5;
/// filter.filter_mask_id_high = 0x24 << 5;
/// filter.filter_id_high = 0x25 << 5;
pub struct CanFilterConfig {
    pub filter_number: u8,
    pub bank_number: u8,
    pub fifo_assignment: RxFifo,
    pub mode: FilterMode,
    pub scale: FilterScale,
    pub filter_id_high: u32,
    pub filter_id_low: u32,
    pub filter_mask_id_high: u32,
    pub filter_mask_id_low: u32,
    pub enabled: bool,
}

/// Default is a filter that matches all messages.
impl Default for CanFilterConfig {
    fn default() -> Self {
        CanFilterConfig {
            filter_number: 0,
            bank_number: 14,
            fifo_assignment: RxFifo::Fifo0,
            mode: FilterMode::IdMask,
            scale: FilterScale::Fs32Bit,
            filter_id_high: 0,
            filter_id_low: 0,
            filter_mask_id_high: 0,
            filter_mask_id_low: 0,
            enabled: true,
        }
    }
}

// FIXME these should be "closed" traits
/// TX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait TxPin<CAN> {}

/// RX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait RxPin<CAN> {}

unsafe impl TxPin<CAN1> for PD1<AF9> {}
unsafe impl TxPin<CAN2> for PB13<AF9> {}

unsafe impl RxPin<CAN1> for PD0<AF9> {}
unsafe impl RxPin<CAN2> for PB12<AF9> {}

// CAN abstraction
pub struct Can<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

macro_rules! hal {
    ($(
        $CANX:ident: ($canX:ident, $canXen:ident, $canXrst:ident, $init_wait:ident),
    )+) => {
        $(
impl<TX, RX> Can<$CANX, (TX, RX)> {
    pub fn $canX(
        can: $CANX,
        pins: (TX, RX),
        apb: &mut APB1,
        settings: &CanConfig,
    ) -> Result<Self, CanError>
    where
        TX: TxPin<$CANX>,
        RX: RxPin<$CANX>,
    {
        // enable
        apb.enr().modify(|_, w| w.$canXen().set_bit());

        // reset
        apb.rstr().modify(|_, w| w.$canXrst().set_bit());
        apb.rstr().modify(|_, w| w.$canXrst().clear_bit());

        // master CAN reset
        can.mcr.modify(|_, w| w.reset().set_bit());

        // exit from sleep mode
        can.mcr.modify(|_, w| w.sleep().clear_bit());
        // TODO - ref impl doesn't wait
        //while can.msr.read().slak().bit() == true {}

        // request initialization mode
        can.mcr.modify(|_, w| w.inrq().set_bit());

        // TODO - timeout?
        // wait for ack
        while can.msr.read().inak().bit() == false {}

        // clear wakeup interrupt
        can.msr.modify(|_, w| w.wkui().clear_bit());

        // apply settings/configurations
        can.mcr.modify(|_, w| w.ttcm().bit(settings.ttcm));
        can.mcr.modify(|_, w| w.abom().bit(settings.abom));
        can.mcr.modify(|_, w| w.awum().bit(settings.awum));
        can.mcr.modify(|_, w| w.nart().bit(settings.nart));
        can.mcr.modify(|_, w| w.rflm().bit(settings.rflm));
        can.mcr.modify(|_, w| w.txfp().bit(settings.txfp));

        // disable debug freeze
        can.mcr.modify(|_, w| w.dbf().clear_bit());

        can.btr.modify(|_, w| w.silm().bit(settings.silent_mode));
        can.btr.modify(|_, w| w.lbkm().bit(settings.loopback_mode));

        can.btr
            .modify(|_, w| unsafe { w.sjw().bits(settings.bit_timing.sjw) });
        can.btr
            .modify(|_, w| unsafe { w.ts2().bits(settings.bit_timing.bs2) });
        can.btr
            .modify(|_, w| unsafe { w.ts1().bits(settings.bit_timing.bs1) });
        can.btr
            .modify(|_, w| unsafe { w.brp().bits(settings.bit_timing.prescaler) });

        // request to leave inialization mode
        can.mcr.modify(|_, w| w.inrq().clear_bit());

        // TODO - timeout?
        // wait for ack
        // TODO - conditional added because it seems only master (CAN1)
        // responds with INAK, needs more testing
        if $init_wait {
            while can.msr.read().inak().bit() == true {}
        }

        Ok(Can { can, pins })
    }

    pub fn configure_filter(&self, config: &CanFilterConfig) -> Result<(), CanError> {
        // CAN1/2 share the same filters, so CAN2 is actually
        // accessing CAN1 IP block
        let can = unsafe { &*CAN1::ptr() };

        let filter_num_bitpos = 1 << config.filter_number;

        // enter filter initialization mode
        can.fmr.modify(|_, w| w.finit().set_bit());

        // select start slave bank
        can
            .fmr
            .modify(|_, w| unsafe { w.can2sb().bits(config.bank_number) });

        // filter deactivation
        can
            .fa1r
            .modify(|r, w| unsafe { w.bits(r.bits() & !filter_num_bitpos) });

        // filter scale
        if let Err(e) = self.set_filter_scale(can, config) {
            // leave initialization mode
            can.fmr.modify(|_, w| w.finit().clear_bit());

            return Err(e);
        }

        // filter mode
        match config.mode {
            FilterMode::IdMask => can
                .fm1r
                .modify(|r, w| unsafe { w.bits(r.bits() & !filter_num_bitpos) }),
            FilterMode::IdList => can
                .fm1r
                .modify(|r, w| unsafe { w.bits(r.bits() | filter_num_bitpos) }),
        }

        // FIFO assignment
        match config.fifo_assignment {
            RxFifo::Fifo0 => can
                .ffa1r
                .modify(|r, w| unsafe { w.bits(r.bits() & !filter_num_bitpos) }),
            RxFifo::Fifo1 => can
                .ffa1r
                .modify(|r, w| unsafe { w.bits(r.bits() | filter_num_bitpos) }),
        }

        // filter activation
        if config.enabled {
            can
                .fa1r
                .modify(|r, w| unsafe { w.bits(r.bits() | filter_num_bitpos) });
        }

        // leave initialization mode
        can.fmr.modify(|_, w| w.finit().clear_bit());

        Ok(())
    }

    // Sets the appropriate FiRx registers based on the configuration
    fn set_filter_scale(&self, can: &can1::RegisterBlock, config: &CanFilterConfig) -> Result<(), CanError> {
        let filter_num_bitpos = 1 << config.filter_number;

        match config.scale {
            FilterScale::Fs16Bit => {
                // dual 16 bit scale
                can
                    .fs1r
                    .modify(|r, w| unsafe { w.bits(r.bits() & !filter_num_bitpos) });

                // first 16 bit id and first 16 bit mask
                // or
                // first 16 bit id and second 16 bit id
                // resets FiR1 state
                match config.filter_number {
                    0 => can.f0r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    1 => can.f1r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    2 => can.f2r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    3 => can.f3r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    4 => can.f4r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    5 => can.f5r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    6 => can.f6r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    7 => can.f7r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    8 => can.f8r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    9 => can.f9r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    10 => can.f10r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    11 => can.f11r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    12 => can.f12r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    13 => can.f13r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    14 => can.f14r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    15 => can.f15r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    16 => can.f16r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    17 => can.f17r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    18 => can.f18r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    19 => can.f19r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    20 => can.f20r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    21 => can.f21r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    22 => can.f22r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    23 => can.f23r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    24 => can.f24r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    25 => can.f25r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    26 => can.f26r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    27 => can.f27r1.write(|w| unsafe {
                        w.bits((config.filter_mask_id_low << 16) | (config.filter_id_low))
                    }),
                    _ => return Err(CanError::ConfigurationFailed),
                }

                // second 16 bit id and second 16 bit mask
                // or
                // third 16 bit id and fourth 16 bit id
                // resets FiR2 state
                match config.filter_number {
                    0 => can.f0r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    1 => can.f1r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    2 => can.f2r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    3 => can.f3r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    4 => can.f4r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    5 => can.f5r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    6 => can.f6r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    7 => can.f7r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    8 => can.f8r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    9 => can.f9r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    10 => can.f10r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    11 => can.f11r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    12 => can.f12r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    13 => can.f13r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    14 => can.f14r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    15 => can.f15r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    16 => can.f16r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    17 => can.f17r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    18 => can.f18r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    19 => can.f19r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    20 => can.f20r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    21 => can.f21r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    22 => can.f22r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    23 => can.f23r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    24 => can.f24r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    25 => can.f25r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    26 => can.f26r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    27 => can.f27r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_id_high))
                    }),
                    _ => return Err(CanError::ConfigurationFailed),
                }
            }
            FilterScale::Fs32Bit => {
                // single 32 bit scale
                can
                    .fs1r
                    .modify(|r, w| unsafe { w.bits(r.bits() | filter_num_bitpos) });

                // 32 bit id or first 32 bit id
                match config.filter_number {
                    0 => can.f0r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    1 => can.f1r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    2 => can.f2r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    3 => can.f3r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    4 => can.f4r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    5 => can.f5r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    6 => can.f6r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    7 => can.f7r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    8 => can.f8r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    9 => can.f9r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    10 => can.f10r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    11 => can.f11r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    12 => can.f12r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    13 => can.f13r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    14 => can.f14r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    15 => can.f15r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    16 => can.f16r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    17 => can.f17r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    18 => can.f18r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    19 => can.f19r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    20 => can.f20r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    21 => can.f21r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    22 => can.f22r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    23 => can.f23r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    24 => can.f24r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    25 => can.f25r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    26 => can.f26r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    27 => can.f27r1.write(|w| unsafe {
                        w.bits((config.filter_id_high << 16) | (config.filter_id_low))
                    }),
                    _ => return Err(CanError::ConfigurationFailed),
                }

                // 32 bit mask or second 32 bit id
                match config.filter_number {
                    0 => can.f0r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    1 => can.f1r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    2 => can.f2r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    3 => can.f3r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    4 => can.f4r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    5 => can.f5r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    6 => can.f6r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    7 => can.f7r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    8 => can.f8r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    9 => can.f9r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    10 => can.f10r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    11 => can.f11r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    12 => can.f12r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    13 => can.f13r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    14 => can.f14r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    15 => can.f15r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    16 => can.f16r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    17 => can.f17r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    18 => can.f18r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    19 => can.f19r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    20 => can.f20r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    21 => can.f21r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    22 => can.f22r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    23 => can.f23r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    24 => can.f24r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    25 => can.f25r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    26 => can.f26r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    27 => can.f27r2.write(|w| unsafe {
                        w.bits((config.filter_mask_id_high << 16) | (config.filter_mask_id_high))
                    }),
                    _ => return Err(CanError::ConfigurationFailed),
                }
            }
        }

        Ok(())
    }

    pub fn transmit(&self, frame: &CanFrame) -> Result<(), CanError> {
        // select an empty tx mailbox
        if self.can.tsr.read().tme0().bit() {
            self.transmit_mb0(frame)
        } else if self.can.tsr.read().tme1().bit() {
            self.transmit_mb1(frame)
        } else if self.can.tsr.read().tme2().bit() {
            self.transmit_mb2(frame)
        } else {
            // all mailboxes are in use
            Err(CanError::BufferExhausted)
        }
    }

    pub fn receive(&self, fifo: &RxFifo) -> Result<CanFrame, CanError> {
        match fifo {
            RxFifo::Fifo0 => self.receive_fifo0(),
            RxFifo::Fifo1 => self.receive_fifo1(),
        }
    }

    pub fn receive_fifo0(&self) -> Result<CanFrame, CanError> {
        // gather relevant registers
        let (rfr, rir, rdtr, rdlr, rdhr) = (
            &self.can.rf0r,
            &self.can.ri0r,
            &self.can.rdt0r,
            &self.can.rdl0r,
            &self.can.rdh0r,
        );

        let pending = rfr.read().fmp0().bits();

        if pending == 0 {
            return Err(CanError::BufferExhausted);
        }

        let ext_id = rir.read().ide().bit();

        // get ID
        let id = if ext_id {
            ID::ExtendedID(ExtendedID::new(rir.read().bits() >> 3 as u32))
        } else {
            ID::BaseID(BaseID::new(rir.read().stid().bits()))
        };

        let remote_frame = rir.read().rtr().bit();
        let dlc = rdtr.read().dlc().bits() as usize;

        let frame = if remote_frame {
            CanFrame::from(RemoteFrame::new(id))
        } else {
            let mut data_frame = DataFrame::new(id);

            data_frame.set_data_length(dlc);

            for i in 0..dlc {
                match i {
                    0 => data_frame.data_as_mut()[i] = rdlr.read().data0().bits(),
                    1 => data_frame.data_as_mut()[i] = rdlr.read().data1().bits(),
                    2 => data_frame.data_as_mut()[i] = rdlr.read().data2().bits(),
                    3 => data_frame.data_as_mut()[i] = rdlr.read().data3().bits(),
                    4 => data_frame.data_as_mut()[i] = rdhr.read().data4().bits(),
                    5 => data_frame.data_as_mut()[i] = rdhr.read().data5().bits(),
                    6 => data_frame.data_as_mut()[i] = rdhr.read().data6().bits(),
                    7 => data_frame.data_as_mut()[i] = rdhr.read().data7().bits(),
                    _ => return Err(CanError::InvalidFrame),
                }
            }

            CanFrame::from(data_frame)
        };

        rfr.modify(|_, w| {
            w
            // release the FIFO
            .rfom0().set_bit()
            // clear FIFO overrun
            .fovr0().clear_bit()
            // clear FIFO full
            .full0().clear_bit()
        });

        Ok(frame)
    }

    pub fn receive_fifo1(&self) -> Result<CanFrame, CanError> {
        // gather relevant registers
        let (rfr, rir, rdtr, rdlr, rdhr) = (
            &self.can.rf1r,
            &self.can.ri1r,
            &self.can.rdt1r,
            &self.can.rdl1r,
            &self.can.rdh1r,
        );

        let pending = rfr.read().fmp1().bits();

        if pending == 0 {
            return Err(CanError::BufferExhausted);
        }

        let ext_id = rir.read().ide().bit();

        // get ID
        let id = if ext_id {
            ID::ExtendedID(ExtendedID::new(
                rir.read().exid().bits() | (rir.read().stid().bits() << 18) as u32,
            ))
        } else {
            ID::BaseID(BaseID::new(rir.read().stid().bits()))
        };

        let remote_frame = rir.read().rtr().bit();
        let dlc = rdtr.read().dlc().bits() as usize;

        let frame = if remote_frame {
            CanFrame::from(RemoteFrame::new(id))
        } else {
            let mut data_frame = DataFrame::new(id);

            data_frame.set_data_length(dlc);

            for i in 0..dlc {
                match i {
                    0 => data_frame.data_as_mut()[i] = rdlr.read().data0().bits(),
                    1 => data_frame.data_as_mut()[i] = rdlr.read().data1().bits(),
                    2 => data_frame.data_as_mut()[i] = rdlr.read().data2().bits(),
                    3 => data_frame.data_as_mut()[i] = rdlr.read().data3().bits(),
                    4 => data_frame.data_as_mut()[i] = rdhr.read().data4().bits(),
                    5 => data_frame.data_as_mut()[i] = rdhr.read().data5().bits(),
                    6 => data_frame.data_as_mut()[i] = rdhr.read().data6().bits(),
                    7 => data_frame.data_as_mut()[i] = rdhr.read().data7().bits(),
                    _ => return Err(CanError::InvalidFrame),
                }
            }

            CanFrame::from(data_frame)
        };

        rfr.modify(|_, w| {
            w
            // release the FIFO
            .rfom1().set_bit()
            // clear FIFO overrun
            .fovr1().clear_bit()
            // clear FIFO full
            .full1().clear_bit()
        });

        Ok(frame)
    }

    fn get_tx_status(&self, mb: &TxMailbox) -> bool {
        match mb {
            TxMailbox::Mailbox0 => {
                let rqcp = self.can.tsr.read().rqcp0().bit();
                let txok = self.can.tsr.read().txok0().bit();
                let tme = self.can.tsr.read().tme0().bit();

                rqcp && txok && tme
            }
            TxMailbox::Mailbox1 => {
                let rqcp = self.can.tsr.read().rqcp1().bit();
                let txok = self.can.tsr.read().txok1().bit();
                let tme = self.can.tsr.read().tme1().bit();

                rqcp && txok && tme
            }
            TxMailbox::Mailbox2 => {
                let rqcp = self.can.tsr.read().rqcp2().bit();
                let txok = self.can.tsr.read().txok2().bit();
                let tme = self.can.tsr.read().tme2().bit();

                rqcp && txok && tme
            }
        }
    }

    fn transmit_mb0(&self, frame: &CanFrame) -> Result<(), CanError> {
        // gather relevant registers
        let (tir, tdtr, tdlr, tdhr) = (
            &self.can.ti0r,
            &self.can.tdt0r,
            &self.can.tdl0r,
            &self.can.tdh0r,
        );

        // setup ID, start from TIxR reset
        if let ID::ExtendedID(_) = frame.id() {
            // extented
            tir.write(|w| unsafe { w.ide().set_bit().exid().bits(frame.id().into()) });
        } else {
            // std
            tir.write(|w| unsafe {
                w.ide()
                    .clear_bit()
                    .stid()
                    .bits(u32::from(frame.id()) as u16)
            });
        }

        // set RTR
        if let CanFrame::RemoteFrame(_) = *frame {
            tir.modify(|_, w| w.rtr().set_bit());
        }

        // setup DLC, start from TDTxR reset
        if let CanFrame::DataFrame(df) = *frame {
            tdtr.write(|w| unsafe { w.dlc().bits(df.data().len() as _) });

            // setup data
            for index in 0..df.data().len() {
                match index {
                    0 => tdlr.modify(|_, w| unsafe { w.data0().bits(df.data()[index]) }),
                    1 => tdlr.modify(|_, w| unsafe { w.data1().bits(df.data()[index]) }),
                    2 => tdlr.modify(|_, w| unsafe { w.data2().bits(df.data()[index]) }),
                    3 => tdlr.modify(|_, w| unsafe { w.data3().bits(df.data()[index]) }),
                    4 => tdhr.modify(|_, w| unsafe { w.data4().bits(df.data()[index]) }),
                    5 => tdhr.modify(|_, w| unsafe { w.data5().bits(df.data()[index]) }),
                    6 => tdhr.modify(|_, w| unsafe { w.data6().bits(df.data()[index]) }),
                    7 => tdhr.modify(|_, w| unsafe { w.data7().bits(df.data()[index]) }),
                    _ => return Err(CanError::InvalidFrame),
                };
            }
        } else {
            tdtr.write(|w| unsafe { w.dlc().bits(0) });
        }

        // don't transmit global time
        tdtr.modify(|_, w| w.tgt().clear_bit());

        // request transmission
        tir.modify(|_, w| w.txrq().set_bit());

        // TODO - timeout and cancel?
        // wait for completion
        let mut ticks: u32 = 0;
        while self.get_tx_status(&TxMailbox::Mailbox0) == false {
            ticks += 1;
            if ticks >= MAX_BLOCK_TICKS {
                // cancel transmit
                self.can.tsr.modify(|_, w| w.abrq0().set_bit());

                return Err(CanError::Timeout);
            }
        }

        Ok(())
    }

    fn transmit_mb1(&self, frame: &CanFrame) -> Result<(), CanError> {
        // gather relevant registers
        let (tir, tdtr, tdlr, tdhr) = (
            &self.can.ti1r,
            &self.can.tdt1r,
            &self.can.tdl1r,
            &self.can.tdh1r,
        );

        // setup ID, start from TIxR reset
        if let ID::ExtendedID(_) = frame.id() {
            // extented
            tir.write(|w| unsafe { w.ide().set_bit().exid().bits(frame.id().into()) });
        } else {
            // std
            tir.write(|w| unsafe {
                w.ide()
                    .clear_bit()
                    .stid()
                    .bits(u32::from(frame.id()) as u16)
            });
        }

        // set RTR
        if let CanFrame::RemoteFrame(_) = *frame {
            tir.modify(|_, w| w.rtr().set_bit());
        }

        // setup DLC, start from TDTxR reset
        if let CanFrame::DataFrame(df) = *frame {
            tdtr.write(|w| unsafe { w.dlc().bits(df.data().len() as _) });

            // setup data
            for index in 0..df.data().len() {
                match index {
                    0 => tdlr.modify(|_, w| unsafe { w.data0().bits(df.data()[index]) }),
                    1 => tdlr.modify(|_, w| unsafe { w.data1().bits(df.data()[index]) }),
                    2 => tdlr.modify(|_, w| unsafe { w.data2().bits(df.data()[index]) }),
                    3 => tdlr.modify(|_, w| unsafe { w.data3().bits(df.data()[index]) }),
                    4 => tdhr.modify(|_, w| unsafe { w.data4().bits(df.data()[index]) }),
                    5 => tdhr.modify(|_, w| unsafe { w.data5().bits(df.data()[index]) }),
                    6 => tdhr.modify(|_, w| unsafe { w.data6().bits(df.data()[index]) }),
                    7 => tdhr.modify(|_, w| unsafe { w.data7().bits(df.data()[index]) }),
                    _ => return Err(CanError::InvalidFrame),
                };
            }
        } else {
            tdtr.write(|w| unsafe { w.dlc().bits(0) });
        }

        // don't transmit global time
        tdtr.modify(|_, w| w.tgt().clear_bit());

        // request transmission
        tir.modify(|_, w| w.txrq().set_bit());

        // TODO - timeout and cancel?
        // wait for completion
        let mut ticks: u32 = 0;
        while self.get_tx_status(&TxMailbox::Mailbox1) == false {
            ticks += 1;
            if ticks >= MAX_BLOCK_TICKS {
                // cancel transmit
                self.can.tsr.modify(|_, w| w.abrq1().set_bit());

                return Err(CanError::Timeout);
            }
        }

        Ok(())
    }

    fn transmit_mb2(&self, frame: &CanFrame) -> Result<(), CanError> {
        // gather relevant registers
        let (tir, tdtr, tdlr, tdhr) = (
            &self.can.ti2r,
            &self.can.tdt2r,
            &self.can.tdl2r,
            &self.can.tdh2r,
        );

        // setup ID, start from TIxR reset
        if let ID::ExtendedID(_) = frame.id() {
            // extented
            tir.write(|w| unsafe { w.ide().set_bit().exid().bits(frame.id().into()) });
        } else {
            // std
            tir.write(|w| unsafe {
                w.ide()
                    .clear_bit()
                    .stid()
                    .bits(u32::from(frame.id()) as u16)
            });
        }

        // set RTR
        if let CanFrame::RemoteFrame(_) = *frame {
            tir.modify(|_, w| w.rtr().set_bit());
        }

        // setup DLC, start from TDTxR reset
        if let CanFrame::DataFrame(df) = *frame {
            tdtr.write(|w| unsafe { w.dlc().bits(df.data().len() as _) });

            // setup data
            for index in 0..df.data().len() {
                match index {
                    0 => tdlr.modify(|_, w| unsafe { w.data0().bits(df.data()[index]) }),
                    1 => tdlr.modify(|_, w| unsafe { w.data1().bits(df.data()[index]) }),
                    2 => tdlr.modify(|_, w| unsafe { w.data2().bits(df.data()[index]) }),
                    3 => tdlr.modify(|_, w| unsafe { w.data3().bits(df.data()[index]) }),
                    4 => tdhr.modify(|_, w| unsafe { w.data4().bits(df.data()[index]) }),
                    5 => tdhr.modify(|_, w| unsafe { w.data5().bits(df.data()[index]) }),
                    6 => tdhr.modify(|_, w| unsafe { w.data6().bits(df.data()[index]) }),
                    7 => tdhr.modify(|_, w| unsafe { w.data7().bits(df.data()[index]) }),
                    _ => return Err(CanError::InvalidFrame),
                };
            }
        } else {
            tdtr.write(|w| unsafe { w.dlc().bits(0) });
        }

        // don't transmit global time
        tdtr.modify(|_, w| w.tgt().clear_bit());

        // request transmission
        tir.modify(|_, w| w.txrq().set_bit());

        // TODO - timeout and cancel?
        // wait for completion
        let mut ticks: u32 = 0;
        while self.get_tx_status(&TxMailbox::Mailbox2) == false {
            ticks += 1;
            if ticks >= MAX_BLOCK_TICKS {
                // cancel transmit
                self.can.tsr.modify(|_, w| w.abrq2().set_bit());

                return Err(CanError::Timeout);
            }
        }

        Ok(())
    }
}
)+
    }
}

hal! {
    CAN1: (can1, can1en, can1rst, true),
    CAN2: (can2, can2en, can2rst, false),
}
