#![no_std]
#![no_main]
/// Main entry point for the async application.
///
/// This function initializes the necessary peripherals, sets up the I2C bus,
/// and spawns the display, pressure, and button tasks. It also sets up channels
/// for communication between tasks and runs an infinite loop with a delay.
///
/// # Arguments
///
/// * `spawner` - The task spawner used to spawn async tasks.

type I2cBus = Mutex<NoopRawMutex, I2c<'static, Async>>;
extern crate alloc;

use alloc::string::String;
use defmt::{debug, error};

use core::fmt::Write;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::*, mutex::Mutex};
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{renderer::TextRenderer, Baseline, Text},
};
use esp_println as _;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Pull},
    i2c::master::I2c,
    Async,
};
use ssd1306::{mode::DisplayConfigAsync, prelude::DisplayRotation, size::DisplaySize128x32};

use static_cell::StaticCell;

type ReadingPressure = f32;
type ReadingCounter = u32;

static CHANNEL_PRESSURE: StaticCell<Channel<NoopRawMutex, ReadingPressure, 3>> = StaticCell::new();
static CHANNEL_COUNTER: StaticCell<Channel<NoopRawMutex, ReadingCounter, 3>> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size:1024);

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

    let i2c = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default()
            .with_frequency(esp_hal::time::RateExtU32::kHz(400u32)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7)
    .into_async();
    let i2c_bus = &*I2C_BUS.init(Mutex::new(i2c));

    let button: Input<'_> = Input::new(peripherals.GPIO2, Pull::Up);
    let channel_pressure: &'static mut _ = CHANNEL_PRESSURE.init(Channel::new());
    let c_receiver_pressure = channel_pressure.receiver();
    let c_sender_pressure = channel_pressure.sender();

    let channel_counter: &'static mut _ = CHANNEL_COUNTER.init(Channel::new());
    let c_receiver_counter = channel_counter.receiver();
    let c_sender_counter = channel_counter.sender();
    // Spawn tasks
    spawner.must_spawn(display_task(
        i2c_bus,
        c_receiver_pressure,
        c_receiver_counter,
    ));
    spawner.must_spawn(pressure_task(i2c_bus, c_sender_pressure));
    spawner.must_spawn(button_task(button, c_sender_counter));
    // Run forever and do nothing
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10000)).await;
    }
}

/// Task which reveives messages from the pressure sensor and the button task
/// and displays them on the OLED display.
///
/// The task initializes the display, and then waits for messages from the  pressure
/// sensor and the button task. When a message is received, it updates the display
/// with the new value. The display is updated every time a new message is received.
#[embassy_executor::task]
async fn display_task(
    i2c_bus: &'static I2cBus,
    receiver_pressure: Receiver<'static, NoopRawMutex, ReadingPressure, 3>,
    receiver_counter: Receiver<'static, NoopRawMutex, ReadingCounter, 3>,
) {
    debug!("Display_task created");
    let i2c_dev: I2cDevice<'_, NoopRawMutex, I2c<'_, Async>> = I2cDevice::new(i2c_bus);
    let display_interface = ssd1306::I2CDisplayInterface::new(i2c_dev);
    let mut display = ssd1306::Ssd1306Async::new(
        display_interface,
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    let _ = display.init().await;
    debug!("Display initialized!");

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    let mut output_string = String::new();
    let text_position_pressure = Point::new(0, 30);
    let text_position_counter = Point::new(0, 15);
    loop {
        // Wait for message from sensor
        let reading = receiver_pressure.receive().await;
        //got for message from sensor
        output_string.clear();
        write!(output_string, " value:{:.2}", reading).unwrap();

        //Clear old text
        let metrics =
            text_style.measure_string(&output_string, text_position_pressure, Baseline::Bottom);
        let _ = display.fill_solid(&metrics.bounding_box, BinaryColor::Off);

        //new Text
        Text::with_baseline(
            output_string.as_str(),
            text_position_pressure,
            text_style,
            Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();

        //Check for new counter value
        let reading_counter = receiver_counter.try_receive();
        //If new counter value is available, update the display
        if let Ok(reading_counter) = reading_counter {
            output_string.clear();
            write!(output_string, " Counter:{:3}", reading_counter).unwrap();
            let metrics =
                text_style.measure_string(&output_string, text_position_counter, Baseline::Bottom);
            // Clear old text
            let _ = display.fill_solid(&metrics.bounding_box, BinaryColor::Off);
            // New text
            Text::with_baseline(
                output_string.as_str(),
                text_position_counter,
                text_style,
                Baseline::Bottom,
            )
            .draw(&mut display)
            .unwrap();
        }
        let _ = display.flush().await;
    }
}

/// Task which reads the pressure sensor and sends the value over a channel to the
/// display task.
///
/// The task reads the pressure sensor in a loop and sends the value over a channel to
/// the display task. The pressure sensor is read every 100ms.
///
#[embassy_executor::task]
async fn pressure_task(
    i2c_bus: &'static I2cBus,
    sender: Sender<'static, NoopRawMutex, ReadingPressure, 3>,
) {
    debug!("Pressure task created");
    let i2c_device_pressure =
        embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_bus);
    let mut pressure_sensor = ms5611_i2c::Ms5611::new(i2c_device_pressure, None).await;
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        let pressure_sample = pressure_sensor
            .read_sample(ms5611_i2c::OversamplingRatio::Opt4096)
            .await;
        if let Ok(pressure) = pressure_sample {
            sender.send(pressure.pressure_mbar).await;
        } else {
            error!("Error reading pressure");
        }
    }
}

/// Task which increments a counter each time the button is pressed and sends it
/// over a channel to the display task.
///
/// The button is read in a loop, and each time it is pressed, it sends a new
/// counter value over the channel. The display task will receive this value

#[embassy_executor::task]
async fn button_task(
    mut button: Input<'static>,
    sender: Sender<'static, NoopRawMutex, ReadingCounter, 3>,
) {
    debug!("button Task created");
    let mut button_counter: u32 = 0;
    loop {
        button.wait_for_falling_edge().await;
        button_counter += 1;
        debug!("Button pressed: {}", button_counter);
        sender.send(button_counter).await;
    }
}
