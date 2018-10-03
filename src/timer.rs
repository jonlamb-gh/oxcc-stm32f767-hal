//! Timers

use cast::{u16, u32};
use hal::timer::{CountDown, Periodic};
use nb;
//use stm32f7x7::{TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5,
// TIM6, TIM7, TIM9};
use stm32f7x7::{TIM2, TIM3, TIM4, TIM5, TIM6, TIM7};
use void::Void;

//use rcc::{Clocks, APB1, APB2};
use rcc::{Clocks, APB1};
use time::Hertz;

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

pub trait OnePulse: CountDown {
    fn reconfigure_one_pulse_mode(&mut self);

    fn reset(&mut self);
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $APB:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission
                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // restart counter
                    self.tim.cnt.reset();

                    self.timeout = timeout.into();

                    let frequency = self.timeout.0;

                    let ticks =
                        self.clocks.pclk1().0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finnished. Since this is not the case,
                    // it should be cleared
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl OnePulse for Timer<$TIM> {
                fn reconfigure_one_pulse_mode(&mut self) {
                    // disable
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    // one-pulse mode
                    self.tim.cr1.modify(|_, w| w.opm().set_bit());

                    // enable
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn reset(&mut self) {
                    // disable
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    // restart counter/flags
                    self.tim.cnt.reset();
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    // enable
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }
            }

            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: Clocks, apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer {
                        clocks,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn free(self) -> $TIM {
                    // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, APB1, tim2en, tim2rst),
    TIM3: (tim3, APB1, tim3en, tim3rst),
    TIM4: (tim4, APB1, tim4en, tim4rst),
    TIM5: (tim5, APB1, tim5en, tim5rst),
    TIM6: (tim6, APB1, tim6en, tim6rst),
    TIM7: (tim7, APB1, tim7en, tim7rst),
    /*
    TIM9: (tim9, APB2, tim9en, tim9rst),
    TIM10: (tim10, APB2, tim10en, tim10rst),
    TIM11: (tim11, APB2, tim11en, tim11rst),
    TIM12: (tim12, APB1, tim12en, tim12rst),
    TIM13: (tim13, APB1, tim13en, tim13rst),
    TIM14: (tim14, APB1, tim14en, tim14rst),
    */
}
