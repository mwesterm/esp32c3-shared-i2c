#![no_std]
#![no_main]

use alloc::string::ToString;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyle},
    pixelcolor::{BinaryColor, Rgb555},
    prelude::*,
    text::{renderer::TextRenderer, Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, Async};
use log::info;
use ssd1306::{mode::DisplayConfig, prelude::DisplayRotation, size::DisplaySize128x32, Ssd1306};

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    let config = esp_hal::i2c::master::Config::default();
    let mut i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, config)
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();

    info!("Embassy initialized!");

    spawner.must_spawn(worker_task(i2c));
    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(100)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

#[embassy_executor::task]
async fn worker_task(i2c: esp_hal::i2c::master::I2c<'static, Async>) {
    info!("worker Task created");

    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::with_baseline("Running!", Point::new(00, 30), text_style, Baseline::Bottom)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    #[allow(unused_variables)]
    let text_position = Point::new(0, 15);
    let mut counter: i32 = 0;
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        counter += 1;
        let metrics = text_style.measure_string("000", text_position, Baseline::Bottom);

        display.fill_solid(&metrics.bounding_box, BinaryColor::Off);

        Text::with_baseline(
            &counter.to_string(),
            text_position,
            text_style,
            Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();
        info!("counter: {}", counter);
    }
}
