#[cfg(any(feature = "alloc"))] extern crate alloc;
#[cfg(any(feature = "alloc"))] use alloc::prelude::v1::Box;

use stm32h7xx_hal as hal;
use hal::gpio;
use hal::time;
use hal::dma;
use hal::sai::{ self, SaiI2sExt, SaiChannel, I2sUsers };

// use hal::hal as embedded_hal;
// use embedded_hal::digital::v2::OutputPin;

use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use hal::i2c; // to make the i2c2.write() work

use hal::pac;
use cortex_m::asm;

use num_enum::IntoPrimitive;

// - global constants ---------------------------------------------------------

pub const BLOCK_LENGTH: usize = 32;                             // 32 samples
pub const HALF_DMA_BUFFER_LENGTH: usize = BLOCK_LENGTH * 2;     //  2 channels
pub const DMA_BUFFER_LENGTH:usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

pub const FS: time::Hertz = time::Hertz(48_000);
pub const I2C_FS: time::Hertz = time::Hertz(100_000);

// - static data --------------------------------------------------------------

#[link_section = ".sram1_bss"]
static mut TX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];
#[link_section = ".sram1_bss"]
static mut RX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];


// - types --------------------------------------------------------------------

pub type Frame = (f32, f32);
pub type Block = [Frame; BLOCK_LENGTH];

pub type Sai1Pins = (
    // gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>,  // PDN
    gpio::gpioe::PE2<gpio::Alternate<gpio::AF6>>,     // MCLK_A
    gpio::gpioe::PE5<gpio::Alternate<gpio::AF6>>,     // SCK_A
    gpio::gpioe::PE4<gpio::Alternate<gpio::AF6>>,     // FS_A
    gpio::gpioe::PE6<gpio::Alternate<gpio::AF6>>,     // SD_A
    gpio::gpioe::PE3<gpio::Alternate<gpio::AF6>>,     // SD_B
);

pub type I2C2Pins = (
    gpio::gpioh::PH4<gpio::Alternate<gpio::AF4>>,  // SCL
    gpio::gpiob::PB11<gpio::Alternate<gpio::AF4>>, // SDA
);

type TransferDma1Str0 = dma::Transfer<dma::dma::Stream0<pac::DMA1>,
                                      pac::SAI1,
                                      dma::MemoryToPeripheral,
                                      &'static mut [u32; DMA_BUFFER_LENGTH],
                                      dma::DBTransfer>;

type TransferDma1Str1 = dma::Transfer<dma::dma::Stream1<pac::DMA1>,
                                      pac::SAI1,
                                      dma::PeripheralToMemory,
                                      &'static mut [u32; DMA_BUFFER_LENGTH],
                                      dma::DBTransfer>;


// - Error --------------------------------------------------------------------

#[derive(Debug)]
pub enum Error {
    I2c,
    Dma,
}


// - audio::Interface ---------------------------------------------------------

pub struct Interface<'a> {
    pub fs: time::Hertz,

    #[cfg(not(feature = "alloc"))] function_ptr: Option<fn (f32, &mut Block)>,
    #[cfg(any(feature = "alloc"))] closure: Option<Box<dyn FnMut (f32, &mut Block) + Send + Sync + 'a>>,

    // ak4556_reset: Option<gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>>,
    hal_dma1_stream0: Option<TransferDma1Str0>,
    hal_dma1_stream1: Option<TransferDma1Str1>,
    hal_sai1: Option<hal::sai::Sai<pac::SAI1, hal::sai::I2S>>,
    hal_i2c2: Option<hal::i2c::I2c<pac::I2C2>>,

    _marker: core::marker::PhantomData<&'a ()>,
}


impl<'a> Interface<'a> {
    pub fn init(
        clocks: &hal::rcc::CoreClocks,
        sai1_rec: hal::rcc::rec::Sai1, // reset and enable control
        sai1_pins: Sai1Pins,
        i2c2_rec: hal::rcc::rec::I2c2, // reset and enable control
        i2c2_pins: I2C2Pins,
        dma1_rec: hal::rcc::rec::Dma1
    ) -> Result<Interface<'a>, Error> {

        // - configure dma1 ---------------------------------------------------

        let dma1_streams = dma::dma::StreamsTuple::new(unsafe { pac::Peripherals::steal().DMA1 }, dma1_rec);

        // dma1 stream 0
        let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] = unsafe { &mut RX_BUFFER };
        let dma_config = dma::dma::DmaConfig::default()
            .priority(dma::config::Priority::High)
            .memory_increment(true)
            .peripheral_increment(false)
            .circular_buffer(true)
            .fifo_enable(false);

        // is later overwritten to be a P2M stream! (HAL doesn't support this yet)
        let dma1_str0: dma::Transfer<_, _, dma::MemoryToPeripheral, _, _> = dma::Transfer::init(
            dma1_streams.0,
            unsafe { pac::Peripherals::steal().SAI1 },
            rx_buffer,
            None,
            dma_config,
        );

        // dma1 stream 1
        let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] = unsafe { &mut TX_BUFFER };
        let dma_config = dma_config.transfer_complete_interrupt(true)
                                   .half_transfer_interrupt(true);

        // is later overwritten to be a M2P stream! (HAL doesn't support this yet)
        let dma1_str1: dma::Transfer<_, _, dma::PeripheralToMemory, _, _> = dma::Transfer::init(
            dma1_streams.1,
            unsafe { pac::Peripherals::steal().SAI1 },
            tx_buffer,
            None,
            dma_config,
        );

        // - configure sai1 ---------------------------------------------------

        let sai1_rx_config = sai::I2SChanConfig::new(sai::I2SDir::Rx)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(sai::I2SClockStrobe::Falling);

        let sai1_tx_config = sai::I2SChanConfig::new(sai::I2SDir::Tx)
            .set_sync_type(sai::I2SSync::Internal)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(sai::I2SClockStrobe::Rising);

        let sai1_pins = (
            sai1_pins.0,
            sai1_pins.1,
            sai1_pins.2,
            sai1_pins.3,
            Some(sai1_pins.4),
        );

        let sai1 = unsafe { pac::Peripherals::steal().SAI1 }.i2s_ch_a(
            sai1_pins,
            FS,
            sai::I2SDataSize::BITS_24,
            sai1_rec,
            clocks,
            I2sUsers::new(sai1_rx_config).add_slave(sai1_tx_config),
        );

        // manually configure Channel B as transmit stream
        let dma1_reg = unsafe { pac::Peripherals::steal().DMA1 };
        dma1_reg.st[0].cr.modify(|_ , w | w.dir().peripheral_to_memory());
        // manually configure Channel A as receive stream
        dma1_reg.st[1].cr.modify(|_ , w | w.dir().memory_to_peripheral());

        // - configure i2c ---------------------------------------------------

        let i2c2 = i2c::I2cExt::i2c(
            unsafe { pac::Peripherals::steal().I2C2 },
            i2c2_pins,
            I2C_FS,
            i2c2_rec,
            clocks,
        );

        Ok(Self {
            fs: FS,

            #[cfg(not(feature = "alloc"))] function_ptr: None,
            #[cfg(any(feature = "alloc"))] closure: None,

            // ak4556_reset: Some(pins.0),
            hal_dma1_stream0: Some(dma1_str0),
            hal_dma1_stream1: Some(dma1_str1),
            hal_sai1: Some(sai1),
            hal_i2c2: Some(i2c2),

            _marker: core::marker::PhantomData,
        })
    }


    /// assign function pointer for interrupt callback and start audio
    #[cfg(not(feature = "alloc"))]
    pub fn spawn(mut self, function_ptr:fn (f32, &mut Block)) -> Result<Self, Error> {
        self.function_ptr = Some(function_ptr);
        self.start()?;
        Ok(self) // TODO type state for started audio interface
    }

    /// assign closure for interrupt callback and start audio
    #[cfg(any(feature = "alloc"))]
    pub fn spawn<F: FnMut(f32, &mut Block) + Send + Sync + 'a>(mut self, closure: F) -> Result<Self, Error> {
        self.closure = Some(Box::new(closure));
        self.start()?;
        Ok(self) // TODO type state for started audio interface
    }


    fn start(&mut self) -> Result<(), Error> {
        // - AK4556 -----------------------------------------------------------

        // let ak4556_reset = self.ak4556_reset.as_mut().unwrap();
        // ak4556_reset.set_low().unwrap();
        // use cortex_m::asm;
        // asm::delay(480_000); // ~ 1ms (datasheet specifies minimum 150ns)
        // ak4556_reset.set_high().unwrap();

        // - WM8731 -----------------------------------------------------------

        let i2c2 = self.hal_i2c2.as_mut().unwrap();

        let codec_i2c_address: u8 = 0x1a; // or 0x1b if CSB is high

        // Go through configuration setup
        for (register, value) in REGISTER_CONFIG {
            let register: u8 = (*register).into();
            let value: u8 = (*value).into();
            let byte1: u8 = ((register << 1) & 0b1111_1110) | ((value >> 7) & 0b0000_0001u8);
            let byte2: u8 = value & 0b1111_1111;
            let bytes = [byte1, byte2];

            i2c2.write(codec_i2c_address, &bytes).unwrap_or_default();
            
            // wait ~10us
            asm::delay(5_000);

        }

        // - start audio ------------------------------------------------------

        // unmask interrupt handler for dma 1, stream 1
        unsafe { pac::NVIC::unmask(pac::Interrupt::DMA1_STR1); }

        let dma1_str0 = self.hal_dma1_stream0.as_mut().unwrap();
        let dma1_str1 = self.hal_dma1_stream1.as_mut().unwrap();
        let sai1 = self.hal_sai1.as_mut().unwrap();

        dma1_str1.start(|_sai1_rb| {
            sai1.enable_dma(SaiChannel::ChannelA); // rx
        });

        dma1_str0.start(|sai1_rb| {
            sai1.enable_dma(SaiChannel::ChannelB); // tx

            // wait until sai1's fifo starts to receive data
            while sai1_rb.chb.sr.read().flvl().is_empty() { }

            sai1.enable();

            use stm32h7xx_hal::traits::i2s::FullDuplex;
            sai1.try_send(0, 0).unwrap();
        });

        Ok(())
    }

    pub fn handle_interrupt_dma1_str1(&mut self) -> Result<(), Error> {
        let transfer = self.hal_dma1_stream1.as_mut().unwrap();

        let skip = if transfer.get_half_transfer_flag() {
            transfer.clear_half_transfer_interrupt();
            (0, HALF_DMA_BUFFER_LENGTH)

        } else if transfer.get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();
            (HALF_DMA_BUFFER_LENGTH, 0)

        } else {
            // TODO handle error flags once HAL supports them
            return Err(Error::Dma);
        };

        // callback buffer
        let mut block: Block = [(0., 0.); BLOCK_LENGTH];

        // convert & copy rx buffer to callback buffer
        let mut dma_index: usize = 0;
        let mut block_index: usize = 0;
        while dma_index < HALF_DMA_BUFFER_LENGTH {
            let rx0: usize = dma_index + skip.1;
            let rx1: usize = rx0 + 1;

            let rx_y0 = unsafe { RX_BUFFER[rx0] };
            let rx_y1 = unsafe { RX_BUFFER[rx1] };

            let y0 = u24_to_f32(rx_y0);
            let y1 = u24_to_f32(rx_y1);
            block[block_index] = (y1, y0);

            dma_index += 2;
            block_index += 1;
        }

        // invoke audio callback
        self.invoke_callback(&mut block);

        // convert & copy callback buffer to tx buffer
        let mut dma_index: usize = 0;
        let mut block_index: usize = 0;
        while dma_index < HALF_DMA_BUFFER_LENGTH {
            let tx0: usize = dma_index + skip.0;
            let tx1: usize = tx0 + 1;

            let (y0, y1) = block[block_index];
            unsafe { TX_BUFFER[tx0] = f32_to_u24(y0) };
            unsafe { TX_BUFFER[tx1] = f32_to_u24(y1) };

            dma_index += 2;
            block_index += 1;
        }

        Ok(())
    }


    fn invoke_callback(&mut self, block: &mut Block) {
        #[cfg(not(feature = "alloc"))]
        if let Some(function_ptr) = self.function_ptr.as_mut() {
            function_ptr(self.fs.0 as f32, block);
        }

        #[cfg(any(feature = "alloc"))]
        if let Some(closure) = self.closure.as_mut() {
            closure(self.fs.0 as f32, block);
        }
    }
}


// - conversion helpers -------------------------------------------------------

use core::num::Wrapping;

/// convert audio data from u24 to f32
#[inline(always)]
fn u24_to_f32(y: u32) -> f32 {
    let y = (Wrapping(y) + Wrapping(0x0080_0000)).0 & 0x00FF_FFFF; // convert to i32
    let y = (y as f32 / 8_388_608.) - 1.;  // (2^24) / 2
    y
}

/// convert audio data from f32 to u24
#[inline(always)]
fn f32_to_u24(x: f32) -> u32 {
    //return (int16_t) __SSAT((int32_t) (x * 32767.f), 16);
    let x = x * 8_388_607.;
    let x = if x > 8_388_607. {
        8_388_607.
    } else if x < -8_388_608. {
        -8_388_608.
    } else {
        x
    };
    (x as i32) as u32
}

// - WM8731 codec register addresses -------------------------------------------------

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, IntoPrimitive)]
#[repr(u8)]
enum Register {
    LINVOL = 0x00,
    RINVOL = 0x01,
    LOUT1V = 0x02,
    ROUT1V = 0x03,
    APANA = 0x04,
    APDIGI = 0x05, // 0000_0101
    PWR = 0x06,
    IFACE = 0x07,  // 0000_0111
    SRATE = 0x08,  // 0000_1000
    ACTIVE = 0x09, // 0000_1001
    RESET = 0x0F,
}

const REGISTER_CONFIG: &[(Register, u8)] = &[
    // reset Codec
    (Register::RESET, 0x00), 

    // set line inputs 0dB
    (Register::LINVOL, 0x17),
    (Register::RINVOL, 0x17),

    // set headphone to mute
    (Register::LOUT1V, 0x00),
    (Register::ROUT1V, 0x00),

    // set analog and digital routing
    (Register::APANA, 0x12),
    (Register::APDIGI, 0x01),

    // configure power management
    (Register::PWR, 0x42),

    // configure digital format
    (Register::IFACE, 0x0A),
    
    // set samplerate
    (Register::SRATE, 0x00),

    (Register::ACTIVE, 0x00),
    (Register::ACTIVE, 0x01),
];
