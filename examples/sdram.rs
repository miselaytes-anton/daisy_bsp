#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use micromath::F32Ext;

use cortex_m::asm;
use cortex_m_rt::entry;
use daisy_bsp as daisy;

use daisy::hal;
use hal::delay::Delay;
use hal::prelude::*;

use daisy::pac;
use pac::interrupt;

use daisy::audio;
use daisy::led::Led;
use daisy::logger;
use daisy::sdram;
use log::info;

// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    logger::init();

    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
        dp.GPIOH.split(ccdr.peripheral.GPIOH),
        dp.GPIOI.split(ccdr.peripheral.GPIOI),
    );

    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, ccdr.clocks);
    let sdram: &mut [f32] = sdram::Sdram::new(
        dp.FMC,
        ccdr.peripheral.FMC,
        &ccdr.clocks,
        &mut delay,
        &mut cp.SCB,
        &mut cp.MPU,
        pins.SDRAM,
    )
    .into();

    let sdram_size_bytes = sdram::Sdram::bytes();
    let sdram_size = sdram_size_bytes / core::mem::size_of::<u32>();

    info!(
        "SDRAM size: {} bytes, {} words starting at {:?}",
        sdram_size_bytes, sdram_size, &sdram[0] as *const _,
    );

    // Make sure that we're not reading memory from a previous test run
    info!("Clear memory...");
    for item in sdram.iter_mut().take(sdram_size) {
        *item = 0.0;
    }

    info!("Write test pattern...");
    let mut data: f32 = 0.0;
    for item in sdram.iter_mut().take(sdram_size) {
        *item = data;
        data = (data + 1.0) % core::f32::MAX;
    }

    info!("Read test pattern...");
    let percent = (sdram_size as f64 / 100.0) as f32;
    data = 0.0;
    for (i, item) in sdram.iter_mut().enumerate().take(sdram_size) {
        assert!((*item - data).abs() < f32::EPSILON);
        data = (data + 1.0) % core::f32::MAX;

        if (i as f32 % (10.0 * percent)) == 0.0 {
            info!("{}% done", i as f32 / percent);
        }
    }
    info!("Test Success!");

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}
