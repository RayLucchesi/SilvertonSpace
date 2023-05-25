// this is a rust implementation of the dactest c code 
#![no_std]
#![no_main]

// The trait used by formatting macros like write! and writeln!
use core::fmt::Write as FmtWrite;
use core::cell::RefCell;

use rp2040_hal as hal;
use hal::pac;

// The macro for our start-up function
use rp_pico::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use rp_pico::hal::prelude::*;
// use rp_pico::hal::pac;
// use rp_pico::hal;
use rp2040_hal::timer::Alarm;
use critical_section::Mutex;
use pac::interrupt;

// Use MCP4725 Crate to perform I2C work
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write, WriteIter};

use mcp4725::*;

// for hz abbreviations
use fugit::RateExtU32;
use fugit::MicrosDurationU32;

// for sin function

extern crate micromath;
use micromath::F32Ext;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

const SIN_TBL_SIZ : usize  = 256;
static mut SIN_TABLE : [i16; SIN_TBL_SIZ] = [0; SIN_TBL_SIZ];
static mut IDX : Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0)); 
static mut COUNT : Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

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

// Give our LED and Alarm a type alias to make it easier to refer to them
type LedAndAlarmAndDac = (
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::PushPullOutput>,
    hal::timer::Alarm0,
//    MCP4725<hal::I2C0>, // dac component 
);

// Place our LED and Alarm type in a static variable, so we can access it from interrupts
static mut LED_AND_ALARM_AND_DAC: Mutex<RefCell<Option<LedAndAlarmAndDac>>> = Mutex::new(RefCell::new(None));

// Period that each of the alarms will be set for - 1 second and 300ms respectively
const SLOW_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::secs(1);
const FAST_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(300);


// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 I2C & DAC peripherals then sets DAC to a value in sin_table
/// then blinks the LED every pass through the sin_table in an 
/// infinite loop.

#[rp2040_hal::entry]
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
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

// Set the LED to be an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.toggle().unwrap();
    delay.delay_ms(500);
    led_pin.toggle().unwrap();

// Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio26.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio27.into_mode::<hal::gpio::FunctionI2C>();
// let not_an_scl_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();

    for x in 0..SIN_TBL_SIZ {

	    unsafe { 
		SIN_TABLE[x] = ((2047.0*((x as f32) * 6.283/(SIN_TBL_SIZ as f32)).sin()) as i16) + 2048;	
	    }
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

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    critical_section::with(|cs| {
        
        // Configure the MCP4725 DAC
        let mut dac = MCP4725::new(i2c, 0b000);  // uses address 000 binary 

        let mut alarm = timer.alarm_0().unwrap();
        
        // Schedule an alarm in 1 second
        let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US); 
       
        // Move alarm into ALARM, so that it can be accessed from interrupts
        unsafe {
            LED_AND_ALARM_AND_DAC.borrow(cs).replace(Some((led_pin, alarm)));
        }

        // Enable generating an interrupt on alarm
        alarm.enable_interrupt();
   });   
// Unmask the timer0 IRQ so that it will generate an interrupt
    unsafe {
        rp2040_hal::pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }


     loop {
    
     } // loop end
} // main end


// Regular interrupt handler for Alarm0. The `interrupt` macro will perform some transformations to ensure
// that this interrupt entry ends up in the vector table.
#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        // Temporarily take our LED_AND_ALARM_AND_DAC
        let ledalarm = unsafe { LED_AND_ALARM_AND_DAC.borrow(cs).take() };
        if let Some((mut led, mut alarm)) = ledalarm {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            alarm.clear_interrupt();
            // Schedule a new alarm after SLOW_BLINK_INTERVAL_US have passed (1 second)
            let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US);
            // Blink the LED so we know we hit this interrupt
            led.toggle().unwrap();
            // Return LED_AND_ALARM into our static variable
            unsafe {
                LED_AND_ALARM_AND_DAC
                    .borrow(cs)
                    .replace_with(|_| Some((led, alarm)));
            }
        }
    });
}

