// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use core::convert::Infallible;
use defmt::debug;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::peripherals::{SPI1, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15};
use embassy_time::{Duration, Instant, Timer};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_futures::block_on;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use renderer::Rgb565Pixel;
use slint::platform::{software_renderer as renderer};

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static TIMER_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// 16ns for serial clock cycle (write), page 43 of https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf
const SPI_ST7789VW_MAX_FREQ: u32 = 75_000_000;

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 320);

/// The Pixel type of the backing store
pub type TargetPixel = Rgb565Pixel;

type DisplaySpi = Spi<'static, SPI1, Async>;

#[derive(Clone)]
struct SharedSpiWithFreq<CS> {
    spi: &'static Mutex<CriticalSectionRawMutex, RefCell<DisplaySpi>>,
    cs: CS,
    freq: u32,
}

impl<CS> ErrorType for SharedSpiWithFreq<CS> {
    type Error = embassy_rp::spi::Error;
}

impl<CS: OutputPin<Error = Infallible>> SpiDevice for SharedSpiWithFreq<CS> {
    #[inline]
    fn transaction(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Self::Error> {
        self.cs.set_low().ok();
        let result = self.spi.lock(|spi_cell| {
            let mut spi = spi_cell.borrow_mut();
            for op in operations {
                match op {
                    Operation::Read(words) => {
                        block_on(spi.read(words))?;
                    }
                    Operation::Write(words) => {
                        block_on(spi.write(words))?;
                    }
                    Operation::Transfer(read, write) => {
                        block_on(spi.transfer(read, write))?;
                    }
                    Operation::TransferInPlace(words) => {
                        block_on(spi.transfer_in_place(words))?;
                    }
                    Operation::DelayNs(_) => unimplemented!(),
                }
            }
            Ok::<(), embassy_rp::spi::Error>(())
        });
        self.cs.set_high().ok();
        result
    }
}

static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<DisplaySpi>>> = StaticCell::new();
static LINE_BUFFER: StaticCell<[TargetPixel; 320]> = StaticCell::new();
static MIPIDSI_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();

// Global storage for direct SPI and GPIO access during pixel writing
static mut RAW_SPI_FOR_DMA: Option<*mut DisplaySpi> = None;
static mut RAW_DC_PIN: Option<*mut Output<'static>> = None;
static mut RAW_CS_PIN: Option<*mut Output<'static>> = None;

// Simple global state for display initialization
static DISPLAY_READY: embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::Cell<bool>> =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(false));

// Public initialization function that can be called from the main binary
pub async fn init(spawner: &Spawner) {
    let p = embassy_rp::init(Default::default());

    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // Initialize GPIO pins
    let rst = Output::new(p.PIN_15, Level::High);
    let mut dc = Output::new(p.PIN_14, Level::Low);
    let mut cs = Output::new(p.PIN_13, Level::High);

    // Initialize SPI
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = SPI_ST7789VW_MAX_FREQ;
    spi_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    spi_config.polarity = embassy_rp::spi::Polarity::IdleHigh;

    let mut spi = Spi::new(
        p.SPI1,
        p.PIN_10, // sclk
        p.PIN_11, // mosi
        p.PIN_12, // miso
        p.DMA_CH0,
        p.DMA_CH1,
        spi_config,
    );

    // Store raw pointers for DMA access (similar to original "stolen" pins approach)
    unsafe {
        RAW_SPI_FOR_DMA = Some(&mut spi as *mut _);
        RAW_DC_PIN = Some(&mut dc as *mut _);
        RAW_CS_PIN = Some(&mut cs as *mut _);
    }

    let spi_bus = SPI_BUS.init(Mutex::new(RefCell::new(spi)));
    let mipidsi_buffer = MIPIDSI_BUFFER.init([0u8; 512]);

    let display_spi = SharedSpiWithFreq {
        spi: spi_bus,
        cs,
        freq: SPI_ST7789VW_MAX_FREQ
    };

    let di = mipidsi::interface::SpiInterface::new(display_spi, dc, mipidsi_buffer);
    let mut delay = embassy_time::Delay;

    let display = mipidsi::Builder::new(mipidsi::models::ST7796, di)
        .reset_pin(rst)
        .display_size(DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _)
        .orientation(mipidsi::options::Orientation::new().flip_horizontal().rotate(mipidsi::options::Rotation::Deg0))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut delay)
        .unwrap();

    // Mark display as ready
    DISPLAY_READY.lock(|ready| ready.set(true));

    // Drop the display since we'll use direct SPI access for pixel writing
    drop(display);

    slint::platform::set_platform(Box::new(PicoBackend {
        window: Default::default(),
        buffer_provider: RefCell::new(DrawBuffer {
            buffer: LINE_BUFFER.init_with(|| [Rgb565Pixel::default(); 320]),
        }),
    }))
    .expect("backend already initialized");

    debug!("finished initializing screen");

    // Spawn timer task
    spawner.spawn(timer_task()).unwrap();
}

#[embassy_executor::task]
async fn timer_task() {
    loop {
        Timer::after(Duration::from_millis(16)).await; // ~60 FPS
        TIMER_SIGNAL.signal(());
    }
}

async fn run_event_loop_embassy() -> ! {
    loop {
        slint::platform::update_timers_and_animations();

        // Use the platform's existing window handling
        if let Some(duration) = slint::platform::duration_until_next_timer_update() {
            let millis = duration.as_millis() as u64;
            if millis > 0 {
                Timer::after(Duration::from_millis(millis)).await;
            } else {
                Timer::after(Duration::from_micros(duration.as_micros() as u64)).await;
            }
        } else {
            TIMER_SIGNAL.wait().await;
        }
    }
}

struct PicoBackend<DrawBuffer> {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
    buffer_provider: RefCell<DrawBuffer>,
}

impl<DrawBuffer> slint::platform::Platform for PicoBackend<DrawBuffer>
where
    for<'a> &'a mut DrawBuffer: renderer::LineBufferProvider<TargetPixel = TargetPixel>,
{
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window =
            renderer::MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        // Set window size
        self.window.borrow().as_ref().unwrap().set_size(DISPLAY_SIZE);

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = self.buffer_provider.borrow_mut();
                    renderer.render_by_line(&mut *buffer_provider);
                });

                if window.has_active_animations() {
                    continue;
                }
            }

            // Simple blocking wait - not ideal but works for basic functionality
            // In a real embassy implementation, this would yield properly
            break;
        }
        Ok(())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        let now = Instant::now();
        core::time::Duration::from_micros(now.as_micros())
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

struct DrawBuffer {
    buffer: &'static mut [TargetPixel; 320],
}

impl renderer::LineBufferProvider for &mut DrawBuffer {
    type TargetPixel = TargetPixel;

    fn process_line(
        &mut self,
        _line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [TargetPixel]),
    ) {
        // Check if display is ready using the blocking mutex
        let is_ready = DISPLAY_READY.lock(|ready| ready.get());

        if is_ready {
            // Render pixels to the buffer that's part of the struct
            render_fn(&mut self.buffer[range.clone()]);

            // Convert from little endian to big endian for display
            for pixel in &mut self.buffer[range.clone()] {
                *pixel = Rgb565Pixel(pixel.0.to_be());
            }

            // Write pixels to display using DMA
            //DrawBuffer::write_pixels_dma_static(pixel_bytes);
        }
    }
}

impl DrawBuffer {
    fn write_pixels_dma_static(pixel_bytes: &[u8]) {
        // Access the SPI bus that was initialized with DMA channels
        // We'll access it through an unsafe approach similar to the original "stolen" pins
        unsafe {
            if let Some(spi_ptr) = RAW_SPI_FOR_DMA {
                let spi = &mut *spi_ptr;

                // This is where we actually write the pixel data using DMA
                // The SPI was initialized with DMA channels, so this write will use DMA

                // In the original implementation, this would also:
                // 1. Set DC pin high for data mode
                // 2. Set CS pin low
                // 3. Write pixel data via async SPI (which uses DMA)
                // 4. Set CS pin high

                // Since we're in a sync context but SPI is async, use block_on
                if let Err(_) = block_on(spi.write(pixel_bytes)) {
                    // Handle error silently - in a real implementation you might want logging
                }

                // The embassy SPI write() method automatically uses the DMA channels
                // that were configured during SPI initialization (DMA_CH0, DMA_CH1)
            }
        }
    }

    fn write_pixels_dma(&mut self, pixels: &[TargetPixel]) {
        // Convert pixels to byte array for DMA transfer
        let pixel_bytes = unsafe {
            core::slice::from_raw_parts(
                pixels.as_ptr() as *const u8,
                pixels.len() * core::mem::size_of::<TargetPixel>(),
            )
        };

        Self::write_pixels_dma_static(pixel_bytes);
    }
}

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // For panic handler, we'll use a simplified approach without full embassy initialization
    let p = unsafe { embassy_rp::Peripherals::steal() };
    let mut led = Output::new(p.PIN_25, Level::High); // Built-in LED

    loop {
        led.set_high();
        // Simple busy wait delay
        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }
        led.set_low();
        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }
    }
}
