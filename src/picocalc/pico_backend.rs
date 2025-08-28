// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::{Cell, RefCell};
use core::convert::Infallible;
use cortex_m::delay::Delay;
use cortex_m::singleton;
use critical_section::CriticalSection;
use defmt::debug;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::DrawTarget;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use fugit::{Hertz, Instant, RateExtU32};
use hal::dma::{DMAExt, SingleChannel, WriteTarget};
use hal::gpio::{self, Interrupt as GpioInterrupt};
use hal::timer::{Alarm, Alarm0};
use pac::interrupt;
use {defmt_rtt as _, panic_probe as _};
use renderer::Rgb565Pixel;
use rp_pico::hal::{self, pac, prelude::*, Timer};
use slint::platform::{software_renderer as renderer, PointerEventButton, WindowEvent};
use crate::picocalc::display::st7365p::ST7365P;

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static ALARM0: Mutex<CriticalSectionRawMutex, RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

// 16ns for serial clock cycle (write), page 43 of https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(62_500_000);

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 320);

/// The Pixel type of the backing store
pub type TargetPixel = Rgb565Pixel;

type PicoDisplay<SPI, DC, RST> = ST7365P<SPI, DC, RST, Timer>;

pub fn init() {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
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

    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let rst = pins.gpio15.into_push_pull_output();

    let dc = pins.gpio14.into_push_pull_output();
    let cs = pins.gpio13.into_push_pull_output();

    let spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ST7789VW_MAX_FREQ,
        &embedded_hal::spi::MODE_3,
    );

    // SAFETY: This is not safe :-(  But we need to access the SPI and its control pins for the PIO
    let (dc_copy, cs_copy) =
        unsafe { (core::ptr::read(&dc as *const _), core::ptr::read(&cs as *const _)) };
    let stolen_spi = unsafe { core::ptr::read(&spi as *const _) };

    let display_spi = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    let mut display = ST7365P::new(
        display_spi,
        dc,
        Some(rst),
        false,
        true,
        DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _,
        timer,
    );
    display.init().unwrap();
    display.set_custom_orientation(0x40).unwrap();
    display.set_on().unwrap();

    //display.clear(Rgb565::new(0, 0, 0)).unwrap();

    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();

    critical_section::with(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
        TIMER.borrow(cs).replace(Some(timer));
    });

    critical_section::with(|cs| {
        let alarm = DRIVER.alarms.borrow(cs);
        alarm.timestamp.set(u64::MAX);
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let dma = pac.DMA.split(&mut pac.RESETS);
    let pio = PioTransfer::Idle(
        dma.ch0,
        vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        stolen_spi,
    );
    let buffer_provider = DrawBuffer {
        display,
        buffer: vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        pio: Some(pio),
        stolen_pin: (dc_copy, cs_copy),
    };

    slint::platform::set_platform(Box::new(PicoBackend {
        window: Default::default(),
        buffer_provider: buffer_provider.into(),
        timer
    }))
    .expect("backend already initialized");

    debug!("finished initializing screen")
}

struct PicoBackend<DrawBuffer> {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
    buffer_provider: RefCell<DrawBuffer>,
    timer: Timer,
}

impl<
    SPI: SpiDevice,
    RST: OutputPin<Error = Infallible>,
    TO: WriteTarget<TransmittedWord = u8> + embedded_hal_nb::spi::FullDuplex,
    CH: SingleChannel,
    DC: OutputPin<Error = Infallible>,
    CS_: OutputPin<Error = Infallible>,
> slint::platform::Platform
for PicoBackend<
    DrawBuffer<PicoDisplay<SPI, DC, RST>, PioTransfer<TO, CH>, (DC, CS_)>,
>
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

        self.window.borrow().as_ref().unwrap().set_size(DISPLAY_SIZE);

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                let start = self.timer.get_counter();
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = self.buffer_provider.borrow_mut();
                    renderer.render_by_line(&mut *buffer_provider);
                    buffer_provider.flush_frame();
                });
                debug!("render time: {:?}", (self.timer.get_counter() - start).ticks());


                if window.has_active_animations() {
                    continue;
                }
            }

            let sleep_duration = match slint::platform::duration_until_next_timer_update() {
                None => None,
                Some(d) => {
                    let micros = d.as_micros() as u32;
                    if micros < 10 {
                        // Cannot wait for less than 10µs, or `schedule()` panics
                        continue;
                    } else {
                        Some(fugit::MicrosDurationU32::micros(micros))
                    }
                }
            };

            critical_section::with(|cs| {
                if let Some(duration) = sleep_duration {
                    ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule(duration).unwrap();
                }
            });
            cortex_m::asm::wfe();
        }
    }

    fn duration_since_start(&self) -> core::time::Duration {
        let counter = critical_section::with(|cs| {
            TIMER.borrow(cs).borrow().as_ref().map(|t| t.get_counter().ticks()).unwrap_or_default()
        });
        core::time::Duration::from_micros(counter)
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

enum PioTransfer<TO: WriteTarget, CH: SingleChannel> {
    Idle(CH, &'static mut [TargetPixel], TO),
    Running(hal::dma::single_buffer::Transfer<CH, PartialReadBuffer, TO>),
}

impl<TO: WriteTarget<TransmittedWord = u8>, CH: SingleChannel> PioTransfer<TO, CH> {
    fn wait(self) -> (CH, &'static mut [TargetPixel], TO) {
        match self {
            PioTransfer::Idle(a, b, c) => (a, b, c),
            PioTransfer::Running(dma) => {
                let (a, b, to) = dma.wait();
                (a, b.0, to)
            }
        }
    }
}

struct DrawBuffer<Display, PioTransfer, Stolen> {
    display: Display,
    buffer: &'static mut [TargetPixel],
    pio: Option<PioTransfer>,
    stolen_pin: Stolen,
}

impl<
    SPI: SpiDevice,
    RST: OutputPin<Error = Infallible>,
    TO: WriteTarget<TransmittedWord = u8>,
    CH: SingleChannel,
    DC: OutputPin<Error = Infallible>,
    CS_: OutputPin<Error = Infallible>,
> renderer::LineBufferProvider
for &mut DrawBuffer<PicoDisplay<SPI, DC, RST>, PioTransfer<TO, CH>, (DC, CS_)>
{
    type TargetPixel = TargetPixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [TargetPixel]),
    ) {
        render_fn(&mut self.buffer[range.clone()]);

        //-- Send the pixel without DMA
        /*self.display.set_pixels(
            range.start as _,
            line as _,
            range.end as _,
            line as _,
            self.buffer[range.clone()]
                .iter()
                .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
        );
        return;*/

        // convert from little to big endian before sending to the DMA channel
        for x in &mut self.buffer[range.clone()] {
            *x = Rgb565Pixel(x.0.to_be())
        }
        let (ch, mut b, spi) = self.pio.take().unwrap().wait();
        core::mem::swap(&mut self.buffer, &mut b);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                core::iter::empty(),
            )
            .unwrap();

        self.stolen_pin.1.set_low().unwrap();
        self.stolen_pin.0.set_high().unwrap();
        let mut dma = hal::dma::single_buffer::Config::new(ch, PartialReadBuffer(b, range), spi);
        dma.pace(hal::dma::Pace::PreferSink);
        self.pio = Some(PioTransfer::Running(dma.start()));
        /*let (a, b, c) = dma.start().wait();
        self.pio = Some(PioTransfer::Idle(a, b.0, c));*/
    }
}

impl<
    SPI: SpiDevice,
    RST: OutputPin<Error = Infallible>,
    TO: WriteTarget<TransmittedWord = u8> + embedded_hal_nb::spi::FullDuplex,
    CH: SingleChannel,
    DC: OutputPin<Error = Infallible>,
    CS_: OutputPin<Error = Infallible>,
> DrawBuffer<PicoDisplay<SPI, DC, RST>, PioTransfer<TO, CH>, (DC, CS_)>
{
    fn flush_frame(&mut self) {
        let (ch, b, mut spi) = self.pio.take().unwrap().wait();
        self.stolen_pin.1.set_high().unwrap();

        // After the DMA operated, we need to empty the receive FIFO, otherwise the touch screen
        // driver will pick wrong values.
        // Continue to read as long as we don't get a Err(WouldBlock)
        while !spi.read().is_err() {}

        self.pio = Some(PioTransfer::Idle(ch, b, spi));
    }
}

struct PartialReadBuffer(&'static mut [Rgb565Pixel], core::ops::Range<usize>);
unsafe impl embedded_dma::ReadBuffer for PartialReadBuffer {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const <Self as embedded_dma::ReadBuffer>::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    DRIVER.check_alarm();
}

struct AlarmState {
    timestamp: Cell<u64>,
}
unsafe impl Send for AlarmState {}

struct TimerDriver {
    alarms: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            TIMER.borrow(cs).borrow().as_ref().map(|t| t.get_counter().ticks()).unwrap_or_default()
        })
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl TimerDriver {
    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let now = self.now();
        if timestamp <= now {
            false
        } else {
            ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule_at(hal::timer::Instant::from_ticks(timestamp)).unwrap();
            alarm.timestamp.set(u64::MAX);
            true
        }
    }

    fn check_alarm(&self) {
        let n = 0;
        critical_section::with(|cs| {
            // clear the irq
            ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().clear_interrupt();

            let alarm = &self.alarms.borrow(cs);
            let timestamp = alarm.timestamp.get();
            if timestamp <= self.now() {
                self.trigger_alarm(cs)
            } else {
                ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule_at(hal::timer::Instant::from_ticks(timestamp)).unwrap();
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver{
    alarms:  Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState {
        timestamp: Cell::new(0),
    }),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Safety: it's ok to steal here since we are in the panic handler, and the rest of the code will not be run anymore
    let mut pac = unsafe { pac::Peripherals::steal() };

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.led.into_push_pull_output();
    led.set_high().unwrap();

    // Re-init the display
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
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

    let spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        4_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let rst = pins.gpio15.into_push_pull_output();
    let mut bl = pins.gpio13.into_push_pull_output();
    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();
    bl.set_high().unwrap();
    let spi = singleton!(:SpiRefCell = SpiRefCell::new((spi, 0.Hz()))).unwrap();
    let display_spi = SharedSpiWithFreq { refcell: spi, cs, freq: SPI_ST7789VW_MAX_FREQ };
    let mut buffer = [0_u8; 512];
    let di = mipidsi::interface::SpiInterface::new(display_spi, dc, &mut buffer);
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7796, di)
        .reset_pin(rst)
        .display_size(DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut timer)
        .unwrap();

    use core::fmt::Write;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        text::Text,
    };

    display.fill_solid(&display.bounding_box(), Rgb565::new(0x00, 0x25, 0xff)).unwrap();

    struct WriteToScreen<'a, D> {
        x: i32,
        y: i32,
        width: i32,
        style: MonoTextStyle<'a, Rgb565>,
        display: &'a mut D,
    }
    let mut writer = WriteToScreen {
        x: 0,
        y: 1,
        width: display.bounding_box().size.width as i32 / 6 - 1,
        style: MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE),
        display: &mut display,
    };
    impl<'a, D: DrawTarget<Color = Rgb565>> Write for WriteToScreen<'a, D> {
        fn write_str(&mut self, mut s: &str) -> Result<(), core::fmt::Error> {
            while !s.is_empty() {
                let (x, y) = (self.x, self.y);
                let end_of_line = s
                    .find(|c| {
                        if c == '\n' || self.x > self.width {
                            self.x = 0;
                            self.y += 1;
                            true
                        } else {
                            self.x += 1;
                            false
                        }
                    })
                    .unwrap_or(s.len());
                let (line, rest) = s.split_at(end_of_line);
                let sz = self.style.font.character_size;
                Text::new(line, Point::new(x * sz.width as i32, y * sz.height as i32), self.style)
                    .draw(self.display)
                    .map_err(|_| core::fmt::Error)?;
                s = rest.strip_prefix('\n').unwrap_or(rest);
            }
            Ok(())
        }
    }
    write!(writer, "{}", info).unwrap();

    loop {
        use embedded_hal::delay::DelayNs as _;
        timer.delay_ms(100);
        led.set_low().unwrap();
        timer.delay_ms(100);
        led.set_high().unwrap();
    }
}
