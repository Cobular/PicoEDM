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

use core::cell::Cell;

use critical_section::Mutex;
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::pac::interrupt;
use cortex_m::singleton;
use embedded_hal::digital::OutputPin;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::pwm::SetDutyCycle;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use usb_device::{class_prelude::*, prelude::*};

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
use rp_pico::hal;
use usbd_serial::SerialPort;

const DUTY_CYCLE_PERCENT: u16 = 30;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static MESSAGE_QUEUE: Mutex<Cell<Option<([u8; 64], usize)>>> = Mutex::new(Cell::new(None));

fn write(msg: &str) {
    let mut buf = [0u8; 64];
    let len = msg.as_bytes().len();
    buf.iter_mut().zip(msg.as_bytes()).for_each(|(b, c)| {
        *b = *c;
    });
    critical_section::with(|cs| {
        MESSAGE_QUEUE.borrow(cs).set(Some((buf, len)));
    });
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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let ClocksManager { usb_clock, system_clock, peripheral_clock, .. } = clocks;

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    let nvic = &mut pac.i;

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, system_clock.freq().to_Hz());

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
    let _ = channel.set_duty_cycle(26);

    // Initialize DMA
    let dma = pac.DMA.split(&mut pac.RESETS);

    // ADC Setup
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    let mut current_sensor_pin = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    // we'll capture 1000 samples in total (500 per channel)
    // NOTE: when calling `shift_8bit` below, the type here must be changed from `u16` to `u8`
    let buf_for_samples = singleton!(: [u16; 1000] = [0; 1000]).unwrap();

    // Configure free-running mode:
    let mut adc_fifo = adc
        .build_fifo()
        // Set clock divider to target a sample rate of 96,000 samples per second (1ksps).
        // The value was calculated by `(48MHz / 96ksps) - 1 = 499.0`
        // Please check the `clock_divider` method documentation for details.
        .clock_divider(499, 0)
        // sample the temperature sensor first
        .set_channel(&mut current_sensor_pin)
        // Uncomment this line to produce 8-bit samples, instead of 12 bit (lower bits are discarded)
        //.shift_8bit()
        // Enable DMA transfers for the FIFO
        .enable_dma()
        // Create the FIFO, but don't start it just yet
        .start_paused();

    // Start a DMA transfer (must happen before resuming the ADC FIFO)
    let dma_transfer =
        single_buffer::Config::new(dma.ch0, adc_fifo.dma_read_target(), buf_for_samples).start();

    // Resume the FIFO to start capturing
    adc_fifo.resume();

    let start = timer.get_counter();


    let (_ch, _adc_read_target, buf_for_samples) = dma_transfer.wait();

    let time_taken = timer.get_counter() - start;

    // serial.write(b"Done sampling, printing results:\r\n");

    // Infinite loop, fading LED up and down
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        write("Hello, World!\r\n");
    }
}

// End of file


/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    critical_section::with(|cs| {
        if let Some((msg, len)) = MESSAGE_QUEUE.borrow(cs).take() {
            let mut wr_ptr = &msg[..len];
            while !wr_ptr.is_empty() {
                let _ = serial.write(wr_ptr).map(|len| {
                    wr_ptr = &wr_ptr[len..];
                });
            }
        }
    });
    serial.flush().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                }
            }
        }
    }
}
