// TODO - this needs a refactor
// do something similar to PWM here:
// https://github.com/japaric/stm32f103xx-hal/blob/master/src/pwm.rs
// - need to enforce that the ADC clock does not exceed 30 MHz

use cortex_m;
use rcc::APB2;
use stm32f7x7::{ADC1, ADC2, ADC3, C_ADC};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AdcSampleTime {
    Cycles3,
    Cycles15,
    Cycles28,
    Cycles56,
    Cycles84,
    Cycles112,
    Cycles144,
    Cycles480,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AdcChannel {
    Adc123In3,
    Adc123In10,
    Adc123In13,
    Adc12In9,
    Adc3In9,
    Adc3In15,
    Adc3In8,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AdcPrescaler {
    Prescaler2,
    Prescaler4,
    Prescaler6,
    Prescaler8,
}

impl From<AdcSampleTime> for u8 {
    fn from(s: AdcSampleTime) -> u8 {
        match s {
            AdcSampleTime::Cycles3 => 0b000,
            AdcSampleTime::Cycles15 => 0b001,
            AdcSampleTime::Cycles28 => 0b010,
            AdcSampleTime::Cycles56 => 0b011,
            AdcSampleTime::Cycles84 => 0b100,
            AdcSampleTime::Cycles112 => 0b101,
            AdcSampleTime::Cycles144 => 0b110,
            AdcSampleTime::Cycles480 => 0b111,
        }
    }
}

impl From<AdcChannel> for u8 {
    fn from(c: AdcChannel) -> u8 {
        match c {
            AdcChannel::Adc123In3 => 3,
            AdcChannel::Adc123In10 => 10,
            AdcChannel::Adc123In13 => 13,
            AdcChannel::Adc12In9 => 9,
            AdcChannel::Adc3In9 => 9,
            AdcChannel::Adc3In15 => 15,
            AdcChannel::Adc3In8 => 8,
        }
    }
}

impl From<AdcPrescaler> for u8 {
    fn from(p: AdcPrescaler) -> u8 {
        match p {
            AdcPrescaler::Prescaler2 => 0b00,
            AdcPrescaler::Prescaler4 => 0b01,
            AdcPrescaler::Prescaler6 => 0b10,
            AdcPrescaler::Prescaler8 => 0b11,
        }
    }
}

pub struct Adc<ADC> {
    adc: ADC,
}

macro_rules! hal {
    ($(
        $ADCX:ident: ($adcX:ident, $adcXen:ident, $adc_master:ident),
    )+) => {
        $(
impl Adc<$ADCX> {
    pub fn $adcX(adc: $ADCX, c_adc: &mut C_ADC, apb: &mut APB2, prescaler: AdcPrescaler) -> Self {
        // reset ADC on ADC1 (master), applies to all
        if $adc_master {
            apb.rstr().modify(|_, w| w.adcrst().set_bit());
            apb.rstr().modify(|_, w| w.adcrst().clear_bit());
        }

        // enable ADCx peripheral clocks
        apb.enr().modify(|_, w| w.$adcXen().set_bit());

        // stop conversions while being configured
        adc.cr2.modify(|_, w| w.swstart().clear_bit());

        // set ADC prescaler
        // $adc_master (bool) is used to only let ADC1 (master)
        // set the prescaler as it is global to ADC1/2/3
        if $adc_master {
            // TODO - must not exceed 30 MHz
            //assert!(adcclock <= 30_000_000);
            c_adc.ccr.write(|w| unsafe { w.adcpre().bits(u8::from(prescaler)) });
        }

        adc.cr1.write(|w| {
            w
                // disable overrun interrupt
                .ovrie().clear_bit()
                // 12-bit resolution
                .res().bits(0b00)
                // disable scan mode
                .scan().clear_bit()
                // disable analog watchdog
                .awden().clear_bit()
                .jawden().clear_bit()
                // disable end of conversion interrupt
                .eocie().clear_bit()
                // disable discontinuous mode
                .discen().clear_bit()
        });

        adc.cr2.write(|w| {
            w
                // trigger detection disabled
                .exten().bits(0b00)
                // right alignment
                .align().clear_bit()
                // EOC set at the end of each regular conversion
                .eocs().set_bit()
                // disable continuous conversion mode
                .cont().clear_bit()
                // disable DMA
                .dds().clear_bit()
                .dma().clear_bit()
        });

        // single conversion
        adc.sqr1.write(|w| w.l().bits(0b0000));

        // enable the ADC peripheral if needed, stabilizing if so
        if adc.cr2.read().adon().bit() == false {
            adc.cr2.modify(|_, w| w.adon().set_bit());
            // TODO - counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000));
            cortex_m::asm::delay(100);
        }

        // clear regular group conversion flag and overrun flag
        adc.sr.modify(|_, w| w.ovr().clear_bit().eoc().clear_bit());

        Adc { adc }
    }

    pub fn read(&self, channel: AdcChannel, sample_time: AdcSampleTime) -> u16 {
        let smpt = u8::from(sample_time);

        // single conversion, uses the 1st conversion in the sequence
        self.adc
            .sqr3
            .write(|w| unsafe { w.sq1().bits(u8::from(channel)) });

        // sample time in cycles
        // channel 10:18 uses SMPR1
        // channel 0:9 uses SMPR2
        match channel {
            AdcChannel::Adc123In3 => self.adc.smpr2.write(|w| unsafe { w.smp3().bits(smpt) }),
            AdcChannel::Adc3In8 => self.adc.smpr2.write(|w| unsafe { w.smp8().bits(smpt) }),
            AdcChannel::Adc3In9 => self.adc.smpr2.write(|w| w.smp9().bits(smpt)),
            AdcChannel::Adc12In9 => self.adc.smpr2.write(|w| w.smp9().bits(smpt)),
            AdcChannel::Adc123In10 => self.adc.smpr1.write(|w| unsafe { w.smp10().bits(smpt) }),
            AdcChannel::Adc123In13 => self.adc.smpr1.write(|w| unsafe { w.smp13().bits(smpt) }),
            AdcChannel::Adc3In15 => self.adc.smpr1.write(|w| unsafe { w.smp15().bits(smpt) }),
        };

        // start conversion
        self.adc.cr2.modify(|_, w| w.swstart().set_bit());

        // wait for conversion to complete
        while !self.adc.sr.read().eoc().bit() {}

        self.adc.sr.modify(|_, w| {
            w
            // clear regular channel start flag
            .strt().clear_bit()
            // clear end of conversion flag
            .eoc().clear_bit()
        });

        // return data register contents
        self.adc.dr.read().data().bits()
    }
}
)+
    }
}

hal! {
    ADC1: (adc1, adc1en, true),
    ADC2: (adc2, adc2en, false),
    ADC3: (adc3, adc3en, false),
}
