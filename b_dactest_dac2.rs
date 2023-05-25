// this is a rust implementation of the dactest c code 
// it uses a different DAC chip MCP4725 which uses I2C 
// rather than SPI
#![no_std]
#![no_main]

// The trait used by formatting macros like write! and writeln!
use core::fmt::Write as FmtWrite;

// The macro for our start-up function
use rp_pico::entry;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;

// Use MCP4725 Crate to perform I2C work
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write, WriteIter};

use mcp4725::*;

// for hz abbreviations
use fugit::RateExtU32;

// for sin function

extern crate micromath;
use micromath::F32Ext;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

const SIN_TBL_SIZ : usize  = 256;

// Setup some blinking codes:
const BLINK_OK_LONG: [u8; 1] = [8u8];
const BLINK_OK_SHORT_LONG: [u8; 4] = [1u8, 0u8, 6u8, 0u8];
const BLINK_OK_SHORT_SHORT_LONG: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_OK_SHORT_SHORT_SHORT_LONG: [u8;8] =[1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_ERR_2_SHORT: [u8; 4] = [1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_3_SHORT: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_4_SHORT: [u8; 8] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_5_SHORT: [u8; 10] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_6_SHORT: [u8; 12] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];

fn blink_signals(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) {
    for bit in sig {
        if *bit != 0 {
            pin.set_high().unwrap();
        } else {
            pin.set_low().unwrap();
        }

        let length = if *bit > 0 { *bit } else { 1 };

        for _ in 0..length {
            delay.delay_ms(150);
        }
    }

    pin.set_low().unwrap();

}

fn blink_signals_loop(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8]
) {
    for _i in 0..1 {
        blink_signals(pin, delay, sig);
        delay.delay_ms(500);
    }
}

// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 I2C & DAC peripherals then sets DAC to a value in sin_table
/// then blinks the LED every pass through the sin_table in an 
/// infinite loop.

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

// The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

// Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

// Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio26.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio27.into_mode::<hal::gpio::FunctionI2C>();
// let not_an_scl_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();

//  sin_table[ii] = (int)(2047*sin((float)ii*6.283/(float)sine_table_size));

    let  mut sin_table : [i16;SIN_TBL_SIZ] = [0;SIN_TBL_SIZ];

    for idx in 0..SIN_TBL_SIZ {

	    sin_table[idx] = ((2047.0*((idx as f32) * 6.283/(SIN_TBL_SIZ as f32)).sin()) as i16) + 2048;	

//	    if idx % 2 == 0 {
//		sin_table[idx] = 0;
//	    } else {
//		sin_table[idx]=4095;
//	   }; 


      // turns out to use println requires inclusion of some macros that are not defined
	   // in the normal rp-hal environment go figure. so if you uncomment this line
	   // you will get undefined symbol errors for _defmt_write, _defmt_acquire & _defmt_release
	   // and there's nothing you can do to define these for the rp-pico

	   // defmt::println!("IDX: {0}, Sin_entry: {1}", idx,  sin_table[idx]) 
	   }

// Create the I²C drive, using the two pre-configured pins. This will fail
// at compile time if the pins are in the wrong mode, or if this I²C
// peripheral isn't available on these pins!
// some pins are i2c0 and some are i2c1, this is defined in i2c.rs in rp-hal/rp2040-hal/src

   let mut i2c = hal::I2C::i2c1(
	pac.I2C1,
	sda_pin,
	scl_pin,
	400.kHz(),
	&mut pac.RESETS,
	&clocks.system_clock,
	);

    blink_signals_loop(&mut led_pin, &mut delay, &BLINK_OK_LONG);

// Write three bytes to the I²C device with 7-bit address 0x00

    let mut count : i64 = 0; 
    let mut freq : u32 = 0;
    let freq_mult : u32 = 1;
    let mut i2c_load : [u8;3] = [0;3];

// Configure the MCP4725 DAC
    let mut dac = MCP4725::new(i2c, 0b000);  // uses address 000 binary 

    loop {
        for idx in 0..SIN_TBL_SIZ {
	   let res = dac.set_dac(PowerDown::Normal, sin_table[idx] as u16);
	   let res = match res  {
		Ok(rtn_code) =>  rtn_code,
		Err(err) => 
			blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_2_SHORT),
	    }; // match end

	    delay.delay_ms(freq*freq_mult);  // add delay to see if we can change sound frequency

	    count +=1;
	    if count %  1_048_576 == 0 {
	       blink_signals_loop(&mut led_pin, &mut delay, &BLINK_OK_SHORT_SHORT_SHORT_LONG);
	       freq +=1;
	    } else if count % 131_072 == 0 {
               blink_signals_loop(&mut led_pin, &mut delay, &BLINK_OK_SHORT_SHORT_LONG);
	       freq +=1;
            } else if count % 16_384 == 0 {
               blink_signals_loop(&mut led_pin, &mut delay, &BLINK_OK_SHORT_LONG);
	       freq +=1;
            }
        } // for idx loop end
    } // loop end
} // main end

