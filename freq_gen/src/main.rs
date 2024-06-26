//! # Pico PWM Blink Example
//!
//! Fades the LED on a Pico board using the PWM peripheral.
//!
//! This will fade in/out the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::asm::wfi;
use cortex_m::delay;
use cortex_m::singleton;
use defmt::println;
use embedded_hal::digital::OutputPin;
use fugit::ExtU64;
use hal::dma::SingleChannel;

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::pwm::SetDutyCycle;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt;
use defmt_serial as _;
use panic_halt as _;

use rp_pico::hal::adc::AdcPin;
use rp_pico::hal::dma::single_buffer;
use rp_pico::hal::dma::DMAExt;
// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use core::fmt::Write;
use fugit::RateExtU32;
use rp_pico::hal;
use rp_pico::hal::uart::DataBits;
use rp_pico::hal::uart::StopBits;
use rp_pico::hal::uart::UartConfig;
use static_cell::StaticCell;

type FullUart = hal::uart::UartPeripheral<
    hal::uart::Enabled,
    pac::UART0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
    ),
>;

static SERIAL: StaticCell<FullUart> = StaticCell::new();

const DUTY_CYCLE_PERCENT: u16 = 30;

const BIN_WIDTH: usize = 32;

const SAMPLE_BATCH_SIZE: usize = 256 * 4;

fn average_samples(samples: &[u16; BIN_WIDTH]) -> u32 {
    let sum: u32 = samples.iter().map(|&x| x as u32).sum();
    sum >> 6
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );

    // Create a UART driver
    let uart: FullUart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(256000.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let serial = SERIAL.init(uart);
    defmt_serial::defmt_serial(serial);

    let mut led_pin = pins.led.into_push_pull_output();
    let mut led_pin_state = false;
    led_pin.set_high().unwrap();

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm1;

    // Setup frequency data
    // Want 1MHz PWM frequency, base clock is 125MHz
    // 125/((42)*(3 + 0/16)) = 0.9920634921% accurate
    // Count up to 42, use a 3.0 divisor
    pwm.set_top(42u16);
    pwm.set_div_int(20u8);
    pwm.set_div_frac(0u8);

    // Do NOT want phase correct
    // Phase correct will cause the counter to count back down from the top
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio2);

    // We want a 30% duty cycle to start, so that's based on the top value
    // of 42, so 42 * 0.3 = 12.6, so 13
    let _ = channel.set_duty_cycle(42);
    // let _ = channel.set_duty_cycle(26);

    // Initialize DMA
    let mut dma = pac.DMA.split(&mut pac.RESETS);

    // ADC Setup
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    let mut current_sensor_pin = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    // we'll capture 1000 samples in total (500 per channel)
    // NOTE: when calling `shift_8bit` below, the type here must be changed from `u16` to `u8`
    let buf_for_samples = singleton!(: [u16; BIN_WIDTH] = [0; BIN_WIDTH]).unwrap();

    // To average, we sum all samples into a u32 then shift right by 7 (divide by 128)

    // Configure free-running mode:
    let mut adc_fifo = adc
        .build_fifo()
        // Set clock divider to target a sample rate of 96,000 samples per second (1ksps).
        // The value was calculated by `(48MHz / 96ksps) - 1 = 499.0`
        // Please check the `clock_divider` method documentation for details.
        .clock_divider(0, 0)
        // sample the temperature sensor first
        .set_channel(&mut current_sensor_pin)
        // Uncomment this line to produce 8-bit samples, instead of 12 bit (lower bits are discarded)
        //.shift_8bit()
        // Enable DMA transfers for the FIFO
        .enable_dma()
        // Create the FIFO, but don't start it just yet
        .start_paused();

    dma.ch0.enable_irq0();

    // Start a DMA transfer (must happen before resuming the ADC FIFO)
    let mut dma_transfer =
        single_buffer::Config::new(dma.ch0, adc_fifo.dma_read_target(), buf_for_samples).start();

    // Resume the FIFO to start capturing
    adc_fifo.resume();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut next_time = timer.get_counter() + 500.millis();

    let double_buf = singleton!(: [u16; BIN_WIDTH] = [0; BIN_WIDTH]).unwrap();

    let mut answer_buffer: [u16; SAMPLE_BATCH_SIZE] = [0; SAMPLE_BATCH_SIZE];

    let mut counter = 0;

    loop {
        if timer.get_counter() > next_time && !led_pin_state {
            led_pin.set_high().unwrap();
            led_pin_state = true;
        }

        if dma_transfer.is_done() {
            let (ch, read_target, buf_for_samples) = dma_transfer.wait();
            buf_for_samples.swap_with_slice(double_buf);
            dma_transfer = single_buffer::Config::new(ch, read_target, buf_for_samples).start();
            adc_fifo.resume();
            let average = average_samples(double_buf);

            // // If the average is less than AVERAGE_CUTOFF, set the PWN duty cycle to 0 for half a second and write 0 to the answer buffer
            // if average < 1300 {
            //     channel.set_duty_cycle(42).unwrap();
            //     led_pin.set_low().unwrap();
            //     led_pin_state = false;
            //     next_time = timer.get_counter() + 200.millis();
            // }

            answer_buffer[counter] = average as u16;
            counter += 1;
        }

        if counter == SAMPLE_BATCH_SIZE {
            println!("{=[?; 1024]}", answer_buffer);
            counter = 0;
        }
    }
}
