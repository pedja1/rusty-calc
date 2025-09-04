// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

extern crate alloc;

use crate::picocalc::display::st7365p::ST7365P;
use crate::picocalc::southbridge::keyboard::slint_key_from_u8;
use crate::picocalc::southbridge::southbridge::{KeyboardState, SouthBridge};
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::{Cell, RefCell};
use core::convert::Infallible;
use core::ops::DerefMut;
use cortex_m::delay::Delay;
use cortex_m::singleton;
use critical_section::CriticalSection;
use defmt::debug;
use embassy_executor::{Executor, Spawner};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::DrawTarget;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use fugit::{Hertz, Instant, RateExtU32};
use hal::dma::{DMAExt, SingleChannel, WriteTarget};
use hal::gpio::{self, Interrupt as GpioInterrupt};
use hal::timer::{Alarm, Alarm0};
use pac::interrupt;
use renderer::Rgb565Pixel;
use rp_pico::hal::dma::{CH0, Channel};
use rp_pico::hal::gpio::bank0::{Gpio6, Gpio7, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15};
use rp_pico::hal::gpio::{
    AnyPin, FunctionI2c, FunctionSio, FunctionSpi, Pin, PullDown, PullUp, SioOutput,
};
use rp_pico::hal::i2c::Controller;
use rp_pico::hal::multicore::{Multicore, Stack};
use rp_pico::hal::spi::Enabled;
use rp_pico::hal::{self, I2C, Spi, Timer, pac, prelude::*};
use rp_pico::pac::{I2C1, SPI1};
use slint::platform::{Key, PointerEventButton, WindowEvent, software_renderer as renderer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static ALARM0: Mutex<CriticalSectionRawMutex, RefCell<Option<Alarm0>>> =
    Mutex::new(RefCell::new(None));
static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<Timer>>> =
    Mutex::new(RefCell::new(None));

// 16ns for serial clock cycle (write), page 43 of https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(62_500_000);

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 320);

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// The Pixel type of the backing store
pub type TargetPixel = Rgb565Pixel;

type PicoDisplay<SPI, DC, RST> = ST7365P<SPI, DC, RST, Timer>;

type PicoDisplayPioTransfer = PioTransfer<
    Spi<
        Enabled,
        SPI1,
        (
            Pin<Gpio11, FunctionSpi, PullDown>,
            Pin<Gpio12, FunctionSpi, PullDown>,
            Pin<Gpio10, FunctionSpi, PullDown>,
        ),
    >,
    Channel<CH0>,
>;
type PicoCalcSouthBridge = SouthBridge<
    I2C<
        I2C1,
        (
            Pin<Gpio6, FunctionI2c, PullUp>,
            Pin<Gpio7, FunctionI2c, PullUp>,
        ),
    >,
>;

type PicoDisplayDCPin = Pin<Gpio14, FunctionSio<SioOutput>, PullDown>;
type PicoDisplayCSPin = Pin<Gpio13, FunctionSio<SioOutput>, PullDown>;

#[allow(static_mut_refs)]
pub fn init(core0_run: fn(Spawner) -> (), core1_run: fn(Spawner) -> ()) {
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

    let mut sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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
    let (dc_copy, cs_copy) = unsafe {
        (
            core::ptr::read(&dc as *const _),
            core::ptr::read(&cs as *const _),
        )
    };
    let stolen_spi = unsafe { core::ptr::read(&spi as *const _) };

    let display_spi = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    let mut display = ST7365P::new(
        display_spi,
        dc,
        Some(rst),
        false,
        true,
        DISPLAY_SIZE.height as _,
        DISPLAY_SIZE.width as _,
        timer,
    );
    display.init().unwrap();
    display.set_custom_orientation(0x40).unwrap();
    display.set_on().unwrap();

    let sda_pin = pins.gpio6.reconfigure();
    let scl_pin = pins.gpio7.reconfigure();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        10.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut sb = SouthBridge::new(i2c);
    sb.init().unwrap();

    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();

    critical_section::with(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
        TIMER.borrow(cs).replace(Some(timer));

        let alarm = DRIVER.alarms.borrow(cs);
        alarm.timestamp.set(u32::MAX.into());
    });

    unsafe {
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

    let window = renderer::MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);

    slint::platform::set_platform(Box::new(PicoBackend {
        window: window.clone(),
    }))
    .expect("backend already initialized");

    let _core = pac::CorePeripherals::take().unwrap();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(core1_run);
        })
        .unwrap();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner
            .spawn(event_loop_task(window, buffer_provider.into(), sb.into()))
            .expect("failed to spawn render loop");
        core0_run(spawner);
    });
}

#[embassy_executor::task]
async fn event_loop_task(
    window: Rc<renderer::MinimalSoftwareWindow>,
    buffer_provider: RefCell<
        DrawBuffer<
            ST7365P<
                ExclusiveDevice<
                    Spi<
                        Enabled,
                        SPI1,
                        (
                            Pin<Gpio11, FunctionSpi, PullDown>,
                            Pin<Gpio12, FunctionSpi, PullDown>,
                            Pin<Gpio10, FunctionSpi, PullDown>,
                        ),
                    >,
                    Pin<Gpio13, FunctionSio<SioOutput>, PullDown>,
                    NoDelay,
                >,
                Pin<Gpio14, FunctionSio<SioOutput>, PullDown>,
                Pin<Gpio15, FunctionSio<SioOutput>, PullDown>,
                Timer,
            >,
            PicoDisplayPioTransfer,
            (PicoDisplayDCPin, PicoDisplayCSPin),
        >,
    >,
    south_bridge: RefCell<PicoCalcSouthBridge>,
) {
    debug!("run_event_loop");

    window.set_size(DISPLAY_SIZE);

    let fw = south_bridge.borrow_mut().read_firmware_version().unwrap();
    debug!("firmware id: {}", fw);

    loop {
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            let mut buffer_provider = buffer_provider.borrow_mut();
            renderer.render_by_line(&mut *buffer_provider);
            buffer_provider.flush_frame();
        });
        //debug!("render time: {:?}", (self.timer.get_counter() - start).ticks());

        loop {
            let key = south_bridge.borrow_mut().read_keyboard_key().unwrap();
            match key.key_state {
                KeyboardState::Idle => break,
                KeyboardState::Pressed => {
                    //debug!("key pressed: {}", key.key_code as char);
                    if let Some(key) = slint_key_from_u8(key.key_code) {
                        window.dispatch_event(WindowEvent::KeyPressed { text: key.into() })
                    }
                }
                KeyboardState::Hold => {}
                KeyboardState::Released => {
                    //debug!("key released: {}", key.key_code as char);
                    if let Some(key) = slint_key_from_u8(key.key_code) {
                        window.dispatch_event(WindowEvent::KeyReleased { text: key.into() })
                    }
                }
            }
        }

        if window.has_active_animations() {
            continue;
        }

        let sleep_duration = match slint::platform::duration_until_next_timer_update() {
            None => Some(fugit::MicrosDurationU32::millis(50)),
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

        //debug!("sleep_duration: {}", sleep_duration.map(|d| d.to_micros()).unwrap_or(0));

        /*critical_section::with(|cs| {
            if let Some(duration) = sleep_duration {
                ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule(duration).unwrap();
            }
        });
        cortex_m::asm::wfe();*/
        if let Some(duration) = sleep_duration {
            embassy_time::Timer::after_ticks(duration.ticks() as u64).await;
        }
    }
}

pub fn used_heap() -> usize {
    ALLOCATOR.used()
}

pub fn free_heap() -> usize {
    ALLOCATOR.free()
}

struct PicoBackend {
    window: Rc<renderer::MinimalSoftwareWindow>,
}

impl slint::platform::Platform for PicoBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        let counter = critical_section::with(|cs| {
            TIMER
                .borrow(cs)
                .borrow()
                .as_ref()
                .map(|t| t.get_counter().ticks())
                .unwrap_or_default()
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
        (
            act_slice.as_ptr() as *const u8,
            act_slice.len() * core::mem::size_of::<Rgb565Pixel>(),
        )
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
            //debug!("now()");
            TIMER
                .borrow(cs)
                .borrow()
                .as_ref()
                .map(|t| t.get_counter().ticks())
                .unwrap_or_default()
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
            alarm.timestamp.set(4294967295);
            false
        } else {
            ALARM0
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .schedule_at(hal::timer::Instant::from_ticks((timestamp as u32) as u64))
                .unwrap();
            true
        }
    }

    fn check_alarm(&self) {
        critical_section::with(|cs| {
            // clear the irq
            ALARM0
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .clear_interrupt();

            let alarm = &self.alarms.borrow(cs);
            let timestamp = alarm.timestamp.get();
            if timestamp <= self.now() {
                self.trigger_alarm(cs)
            } else {
                ALARM0
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .schedule_at(hal::timer::Instant::from_ticks((timestamp as u32) as u64))
                    .unwrap();
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
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
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
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
    let display_spi = SharedSpiWithFreq {
        refcell: spi,
        cs,
        freq: SPI_ST7789VW_MAX_FREQ,
    };
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
        mono_font::{MonoTextStyle, ascii::FONT_6X10},
        pixelcolor::Rgb565,
        prelude::*,
        text::Text,
    };

    display
        .fill_solid(&display.bounding_box(), Rgb565::new(0x00, 0x25, 0xff))
        .unwrap();

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
                Text::new(
                    line,
                    Point::new(x * sz.width as i32, y * sz.height as i32),
                    self.style,
                )
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
