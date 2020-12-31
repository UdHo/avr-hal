#![no_std]
#![no_main]

extern crate panic_halt;

//use hal::pac::{CorePeripherals, Peripherals};
use teensy2::prelude::*;

#[teensy2::entry]
fn main() -> ! {
    let dp = teensy2::Peripherals::take().unwrap();

    let mut pins = teensy2::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD, dp.PORTE, dp.PORTF);

    let mut led0 = pins.d6.into_output(&mut pins.ddr);
    //    let mut led1 = pins.led_tx.into_output(&mut pins.ddr);

    led0.set_high().void_unwrap();

    //    let mut core = dp::take().unwrap();

    let mut time: u16 = 0;
    loop {
        if time % 2 == 0 {
            led0.set_low().void_unwrap();
        } else {
            led0.set_high().void_unwrap();
        }

        teensy2::delay_ms(10);
        time = time.wrapping_add(1);
    }
}
