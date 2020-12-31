use crate::hal::port::PortExt;

avr_hal_generic::impl_board_pins! {
    #[port_defs]
    use crate::hal::port;

    /// Generic DDR that works for all ports
    pub struct DDR {
        portb: crate::pac::PORTB,
        portc: crate::pac::PORTC,
        portd: crate::pac::PORTD,
        porte: crate::pac::PORTE,
        portf: crate::pac::PORTF,
    }

    /// Reexport of the Pro Micro's pins, with the names they have on the board
    pub struct Pins {
        /// `D0` / `RX`
        ///
        /// * `RX` (UART)
        /// * `INT2`: External Interrupt
        pub d2 : portd::pd2::PD2,
        /// `D1` / `TX`
        ///
        /// * `TX` (UART)
        /// * `INT3`: External Interrupt
        pub d3 : portd::pd3::PD3,
        /// `D2` / `SDA`
        ///
        /// * `SDA`: i2c/twi data
        /// * `INT1`: External Interrupt
        pub d1 : portd::pd1::PD1,
        /// `D3` / `SCL`
        ///
        /// * **PWM**: [atmega32u4_hal::timer::Timer0Pwm]
        /// * `SCL`: i2c/twi clock
        /// * `INT0`: External Interrupt
        /// * `OC0B`: Output Compare Channel `B` for Timer/Counter0
        pub d0 : portd::pd0::PD0,
        /// `D4`
        pub d4: portd::pd4::PD4,
        /// `D5`
        ///
        /// * **PWM**: [atmega32u4_hal::timer::Timer3Pwm]
        /// * `OC3A`: Output Compare Channel `A` for Timer/Counter3
        /// * `#OC4A`: Inverted Output Compare Channel `A` for Timer/Counter4 (Not implemented)
        pub c6: portc::pc6::PC6,
        /// `D6`
        ///
        /// Led
        /// * **PWM**: [atmega32u4_hal::timer::Timer4Pwm]
        /// * `OC4D`: Output Compare Channel `D` for Timer/Counter4
        pub d6: portd::pd6::PD6,
        /// `D7`
        ///
        /// * `INT6`: External Interrupt
        pub e6: porte::pe6::PE6,
        /// `D8`
        pub b4: portb::pb4::PB4,
        /// `D9`
        ///
        /// * **PWM**: [atmega32u4_hal::timer::Timer1Pwm]
        /// * `OC1A`: Output Compare Channel `A` for Timer/Counter1
        /// * `#OC4B`: Inverted Output Compare Channel `B` for Timer/Counter4 (Not implemented)
        pub b5: portb::pb5::PB5,
        /// `D10`
        ///
        /// * **PWM**: [atmega32u4_hal::timer::Timer1Pwm]
        /// * `OC1B`: Output Compare Channel `B` for Timer/Counter1
        /// * `OC4B`: Output Compare Channel `B` for Timer/Counter4 (Not implemented)
        pub b6: portb::pb6::PB6,
        /// `RX`
        ///
        ///
        pub b0: portb::pb0::PB0,
        /// `TX`
        ///
        ///
        pub d5: portd::pd5::PD5,
        /// `SCLK`
        ///
        /// ICSP SCLK pin
        pub sck: portb::pb1::PB1,
        /// `MOSI`
        ///
        /// ICSP MOSI pin
        pub mosi: portb::pb2::PB2,
        /// `MISO`
        ///
        /// ICSP MISO pin
        pub miso: portb::pb3::PB3,
        /// `A0`
        ///
        /// * `ADC7` channel
        pub f7: portf::pf7::PF7,
        /// `A1`
        ///
        /// * `ADC6` channel
        pub f6: portf::pf6::PF6,
        /// `A2`
        ///
        /// * `ADC5` channel
        pub f5: portf::pf5::PF5,
        /// `A3`
        ///
        /// * `ADC4` channel
        pub f4: portf::pf4::PF4,
    }
}
