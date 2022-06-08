//! blinky timer using interrupts on TIM2, adapted from blinky_timer_irq.rs example from
//! stm32f1xx-hal
//!
//! This assumes that a LED is connected to pa5 (sck/d13) as is the case on most nucleo board.

#![no_main]
#![no_std]

use panic_halt as _;

use stm32f4xx_hal as hal;

use crate::hal::{
    gpio::{self, Output, PushPull},
    pac::{interrupt, Interrupt, Peripherals, TIM2},
    prelude::*,
    timer::{CounterUs, Event},
};

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

// NOTE You can uncomment 'hprintln' here and in the code below for a bit more
// verbosity at runtime, at the cost of throwing off the timing of the blink
// (using 'semihosting' for printing debug info anywhere slows program
// execution down)
//use cortex_m_semihosting::hprintln;

// A type definition for the GPIO pin to be used for our LED
// For the onboard nucleo LED, use PA5 or PB13 depending your model
type LedPin = gpio::PB12<Output<PushPull>>;
type LedPin2 = gpio::PB13<Output<PushPull>>;

// Make LED pin globally available
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static G_LED2: Mutex<RefCell<Option<LedPin2>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> =
    Mutex::new(RefCell::new(None));

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the timer TIM2 times out
#[interrupt]
fn TIM2() {
    static mut LED: Option<LedPin> = None;
    static mut LED2: Option<LedPin2> = None;
    static mut TIM: Option<CounterUs<TIM2>> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED.borrow(cs).replace(None).unwrap()
        })
    });
    let led2 = LED2.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED2.borrow(cs).replace(None).unwrap()
        })
    });

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });

    let _ = led.toggle();
    let _ = led2.toggle();
    let _ = tim.wait();
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(16.MHz()).pclk1(8.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // Configure PA5 pin to blink LED
    let mut led = gpiob.pb12.into_push_pull_output();
    let _ = led.set_high(); // Turn off
    let mut led2 = gpiob.pb13.into_push_pull_output();
    let _ = led2.set_low(); // Turn on

    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| {
        *G_LED.borrow(cs).borrow_mut() = Some(led);
        *G_LED2.borrow(cs).borrow_mut() = Some(led2);
    });

    // Set up a timer expiring after 1s
    let mut timer = dp.TIM2.counter(&clocks);
    timer.start(100.millis()).unwrap();

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| {
        *G_TIM.borrow(cs).borrow_mut() = Some(timer)
    });

    //enable TIM2 interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    // define RX/TX pins

    let tx_pin = gpioa.pa9.into_alternate();

    // configure serial
    // let mut tx = Serial::tx(dp.USART1, tx_pin, 9600.bps(), &clocks).unwrap();
    // or
    let mut tx = dp.USART1.tx(tx_pin, 9600.bps(), &clocks).unwrap();

    let mut delay = dp.TIM1.delay_ms(&clocks);

    let mut value: u8 = 0;

    loop {
        writeln!(tx, "value: {:02}\r", value).unwrap();
        value = value.wrapping_add(1);
        delay.delay(2.secs());
    }
}
