//! Sdram
use stm32_fmc::devices::as4c16m32msa_6;
use stm32h7xx_hal::{hal::blocking::delay::DelayUs, prelude::*, rcc, stm32};

use crate::pins::SDRAMPins;

/// Configure pins for the FMC controller
macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .set_speed(stm32h7xx_hal::gpio::Speed::VeryHigh)
                    .into_alternate_af12()
                    .internal_pull_up(true)
            ),*
        )
    };
}

/// Struct that owns the sdram
pub struct Sdram {
    inner: *mut u32,
}

impl Sdram {
    /// Initialize the sdram
    pub fn new<D: DelayUs<u8>>(
        fmc_d: stm32::FMC,
        fmc_p: rcc::rec::Fmc,
        clocks: &rcc::CoreClocks,
        delay: &mut D,
        scb: &mut cortex_m::peripheral::SCB,
        mpu: &mut cortex_m::peripheral::MPU,
        pins: SDRAMPins,
    ) -> Self {
        let sdram_pins = fmc_pins! {
            // A0-A12
            pins.ff0, pins.ff1, pins.ff2, pins.ff3,
            pins.ff4, pins.ff5, pins.ff12, pins.ff13,
            pins.ff14, pins.ff15, pins.gg0, pins.gg1,
            pins.gg2,
            // BA0-BA1
            pins.gg4, pins.gg5,
            // D0-D31
            pins.dd14, pins.dd15, pins.dd0, pins.dd1,
            pins.ee7, pins.ee8, pins.ee9, pins.ee10,
            pins.ee11, pins.ee12, pins.ee13, pins.ee14,
            pins.ee15, pins.dd8, pins.dd9, pins.dd10,
            pins.hh8, pins.hh9, pins.hh10, pins.hh11,
            pins.hh12, pins.hh13, pins.hh14, pins.hh15,
            pins.ii0, pins.ii1, pins.ii2, pins.ii3,
            pins.ii6, pins.ii7, pins.ii9, pins.ii10,
            // NBL0 - NBL3
            pins.ee0, pins.ee1, pins.ii4, pins.ii5,
            pins.hh2,   // SDCKE0
            pins.gg8,   // SDCLK
            pins.gg15,  // SDNCAS
            pins.hh3,   // SDNE0
            pins.ff11,  // SDRAS
            pins.hh5    // SDNWE
        };

        let ram_ptr = fmc_d
            .sdram(sdram_pins, as4c16m32msa_6::As4c16m32msa {}, fmc_p, clocks)
            .init(delay);
        crate::mpu::sdram_init(mpu, scb, ram_ptr, Self::bytes());
        Self { inner: ram_ptr }
    }

    /// Get the total number of bytes that this ram has.
    pub const fn bytes() -> usize {
        64 * 1024 * 1024
    }

    /// Get a pointer to the first word of the ram.
    pub fn inner(&self) -> *mut u32 {
        self.inner
    }
}

impl<T: Sized> Into<&'static mut [T]> for Sdram {
    fn into(self) -> &'static mut [T] {
        unsafe {
            core::slice::from_raw_parts_mut(
                self.inner as *mut T,
                Self::bytes() / core::mem::size_of::<T>(),
            )
        }
    }
}
