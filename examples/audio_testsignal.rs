#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _;

use daisy_bsp as daisy;

use daisy::hal;
use hal::prelude::*;

use daisy::pac;
use pac::interrupt;

use daisy::audio;
use daisy::led::Led;
use daisy::loggit;

mod dsp;
use dsp::osc;

// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));

// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
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
    );

    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);

    // let pins = (pins.AK4556.PDN.into_push_pull_output(),
    // pins.AK4556.MCLK_A.into_alternate_af6(),
    // pins.AK4556.SCK_A.into_alternate_af6(),
    // pins.AK4556.FS_A.into_alternate_af6(),
    // pins.AK4556.SD_A.into_alternate_af6(),
    // pins.AK4556.SD_B.into_alternate_af6());

    let i2c2_pins = (
        pins.WM8731.SCL.into_alternate_af4(),
        pins.WM8731.SDA.into_alternate_af4(),
    );

    let sai1_pins = (
        pins.WM8731.MCLK_A.into_alternate_af6(),
        pins.WM8731.SCK_A.into_alternate_af6(),
        pins.WM8731.FS_A.into_alternate_af6(),
        pins.WM8731.SD_A.into_alternate_af6(),
        pins.WM8731.SD_B.into_alternate_af6(),
    );

    let sai1_prec = ccdr
        .peripheral
        .SAI1
        .kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P);

    let i2c2_prec = ccdr.peripheral.I2C2;

    let audio_interface = audio::Interface::init(
        &ccdr.clocks,
        sai1_prec,
        sai1_pins,
        i2c2_prec, // added i2c init
        i2c2_pins,
        ccdr.peripheral.DMA1,
    )
    .unwrap();

    let sdram = crate::sdram::Sdram::new(
        device.FMC,
        ccdr.peripheral.FMC,
        &ccdr.clocks,
        &mut delay,
        &mut core.SCB,
        &mut core.MPU,
        gpiod.pd0,
        gpiod.pd1,
        gpiod.pd8,
        gpiod.pd9,
        gpiod.pd10,
        gpiod.pd14,
        gpiod.pd15,
        gpioe.pe0,
        gpioe.pe1,
        gpioe.pe7,
        gpioe.pe8,
        gpioe.pe9,
        gpioe.pe10,
        gpioe.pe11,
        gpioe.pe12,
        gpioe.pe13,
        gpioe.pe14,
        gpioe.pe15,
        gpiof.pf0,
        gpiof.pf1,
        gpiof.pf2,
        gpiof.pf3,
        gpiof.pf4,
        gpiof.pf5,
        gpiof.pf11,
        gpiof.pf12,
        gpiof.pf13,
        gpiof.pf14,
        gpiof.pf15,
        gpiog.pg0,
        gpiog.pg1,
        gpiog.pg2,
        gpiog.pg4,
        gpiog.pg5,
        gpiog.pg8,
        gpiog.pg15,
        gpioh.ph2,
        gpioh.ph3,
        gpioh.ph5,
        gpioh.ph8,
        gpioh.ph9,
        gpioh.ph10,
        gpioh.ph11,
        gpioh.ph12,
        gpioh.ph13,
        gpioh.ph14,
        gpioh.ph15,
        gpioi.pi0,
        gpioi.pi1,
        gpioi.pi2,
        gpioi.pi3,
        gpioi.pi4,
        gpioi.pi5,
        gpioi.pi6,
        gpioi.pi7,
        gpioi.pi9,
        gpioi.pi10,
    )
    .into();

    // - audio callback -------------------------------------------------------

    // handle callback with function pointer
    #[cfg(not(feature = "alloc"))]
    let audio_interface = {
        fn callback(fs: f32, block: &mut audio::Block) {
            static mut OSC_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
            static mut OSC_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
            unsafe { OSC_1.dx = (1. / fs) * 110.00 };
            unsafe { OSC_2.dx = (1. / fs) * 110.00 };
            for frame in block {
                *frame = (unsafe { OSC_1.step() }, unsafe { OSC_2.step() });
            }
        }

        audio_interface.spawn(callback)
    };

    // handle callback with closure (needs alloc)
    #[cfg(any(feature = "alloc"))]
    let audio_interface = {
        let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
        let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);

        audio_interface.spawn(move |fs, block| {
            osc_1.dx = (1. / fs) * 110.00;
            osc_2.dx = (1. / fs) * 110.00;
            for frame in block {
                *frame = (osc_1.step(), osc_2.step());
            }
        })
    };

    let audio_interface = match audio_interface {
        Ok(audio_interface) => audio_interface,
        Err(e) => {
            loggit!("Failed to start audio interface: {:?}", e);
            loop {}
        }
    };

    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    });

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}

// - interrupts ---------------------------------------------------------------

/// interrupt handler for: dma1, stream1
#[interrupt]
fn DMA1_STR1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str1() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt: {:?}", e);
                }
            };
        }
    });
}
