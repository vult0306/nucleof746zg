#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32f7xx_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let gpiob = p.GPIOB.split();
    let mut led = gpiob.pb7.into_push_pull_output();

    // Constrain clocking registers
    let rcc = p.RCC.constrain();

    // Configure clock and freeze it
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high().expect("Turn on LED");
        delay.delay_ms(500_u16);

        led.set_low().expect("Turn off LED");
        delay.delay_ms(500_u16);
    }
}
