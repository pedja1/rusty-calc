#![no_std]
#![no_main]
#![macro_use]

extern crate alloc;

use alloc::{boxed::Box, rc::Rc};
use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;
use cortex_m::singleton;
use defmt::{debug, info, unwrap};
use embassy_executor::{Executor, SpawnError, Spawner};
use embassy_futures::{block_on, poll_once, yield_now};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi;
use embassy_rp::spi::{Async, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Timer};
use rusty_calc::{
    controller::{Controller},
    picocalc::{hardware::HardwareMcu},
    slint_backend::{StmBackend, DISPLAY_HEIGHT, DISPLAY_WIDTH},
};
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel},
    },
    ComponentHandle,
};
use slint_generated::MainWindow;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
//use rusty_calc::log::init;
use rusty_calc::picocalc::display::st7365p::ST7365P;

use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::DrawTarget;
use mipidsi::Display;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7796;
use rusty_calc::picocalc::spi::exclusive_device::ExclusiveDevice;

//type PicoDisplay = ST7365P<ExclusiveDevice<spi::Spi<'static, SPI1, Async>, Output<'static>, Delay>, Output<'static>, Output<'static>, Delay>;
type PicoDisplay = Display<SpiInterface<'static, ExclusiveDevice<spi::Spi<'static, SPI1, Async>, Output<'static>, Delay>, Output<'static>>, ST7796, Output<'static>>;

const HEAP_SIZE: usize = 100 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
pub static ALLOCATOR: Heap = Heap::empty();

const SCREEN_WIDTH: usize = 320;
const SCREEN_HEIGHT: usize = 320;

static SHARED: Signal<CriticalSectionRawMutex, [Rgb565Pixel; DISPLAY_WIDTH]> = Signal::new();

//static LINE_BUFFER: Mutex<CriticalSectionRawMutex, RefCell<[Rgb565Pixel; DISPLAY_WIDTH]>> = Mutex::new(RefCell::new([Rgb565Pixel(0); DISPLAY_WIDTH]));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    //init();
    debug!("main start");
    let p = embassy_rp::init(Default::default());

    unsafe { ALLOCATOR.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    let mut config = spi::Config::default();
    config.frequency = 75_000_000;
    let spi1 = spi::Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, config,
    );

    let spi_device = ExclusiveDevice::new(spi1, Output::new(p.PIN_13, Level::Low), Delay).unwrap();
    /*let mut display: PicoDisplay = ST7365P::new(
        spi_device,
        Output::new(p.PIN_14, Level::Low),
        Some(Output::new(p.PIN_15, Level::High)),
        false,
        true,
        SCREEN_WIDTH as u32,
        SCREEN_HEIGHT as u32,
        Delay,
    );*/

    let mipidsi_buffer = singleton!(:[u8; 512] = [0; 512]).unwrap();
    let di = mipidsi::interface::SpiInterface::new(spi_device, Output::new(p.PIN_14, Level::Low), mipidsi_buffer);
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7796, di)
        .reset_pin(Output::new(p.PIN_15, Level::High))
        .display_size(SCREEN_WIDTH as _, SCREEN_HEIGHT as _)
        .orientation(mipidsi::options::Orientation::new().flip_horizontal().rotate(mipidsi::options::Rotation::Deg0))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut Delay)
        .unwrap();


    display.clear(Rgb565::new(0, 0, 0)).unwrap();


    /*display.init().await.unwrap();
    display.set_custom_orientation(0x40).await.unwrap();
    display.set_on().await.unwrap();
    display.clear(0).await.unwrap();
    */

    /*display.init().unwrap();
    display.set_custom_orientation(0x40).unwrap();
    display.set_on().unwrap();
    display.clear(0).unwrap();
*/
    let line_buffer = [Rgb565Pixel(0); DISPLAY_WIDTH];
    //let line_buffer: &'static mut [Rgb565Pixel] = LINE_BUFFER.init(line_buffer);

    // create a slint window and register it with slint
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    window.set_size(slint::PhysicalSize::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32));
    let backend = Box::new(StmBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("slint gui setup complete");

    // TASK: run the gui render loop
    unwrap!(spawner.spawn(render_loop(window, line_buffer, display)));

    //unwrap!(spawner.spawn(read_task()));
    //unwrap!(spawner.spawn(write_task(line_buffer)));

    let main_window = MainWindow::new().unwrap();
    main_window.show().expect("unable to show main window");

    let hardware = HardwareMcu {  };

    // run the controller event loop
    let mut controller = Controller::new(&main_window, hardware);
    controller.run().await;
}

#[embassy_executor::task()]
async fn read_task() {
    loop {
        let line_buffer = SHARED.wait().await;

        debug!("read_task: [{},{},{},{},{},{},{},{},{},{},]", line_buffer[0].0, line_buffer[1].0,
            line_buffer[2].0, line_buffer[3].0, line_buffer[4].0, line_buffer[5].0, line_buffer[6].0,
            line_buffer[7].0, line_buffer[8].0, line_buffer[9].0);
    }
}

#[embassy_executor::task()]
async fn write_task(
    mut line_buffer: [Rgb565Pixel; DISPLAY_WIDTH],
) {
    let mut iter = 0;
    loop {
        line_buffer[iter] = Rgb565Pixel(iter as u16);
        SHARED.signal(line_buffer);
        iter+=1;
        if(iter > 10) {
            return;
        }
    }
}

#[embassy_executor::task()]
pub async fn render_loop(
    window: Rc<MinimalSoftwareWindow>,
    mut line_buffer: [Rgb565Pixel; DISPLAY_WIDTH],
    mut display: PicoDisplay,
) {

    loop {
        slint::platform::update_timers_and_animations();

        // process touchscreen events
        //process_touch(&touch, &mut i2c, &mut last_touch, window.clone());

/*        window.draw_async_if_needed(async |renderer| {
            let instant = embassy_time::Instant::now();
            renderer.render_by_line_async(DisplayWrapper {
                display: &mut display,
                line_buffer: &mut line_buffer,
            }).await;
            debug!("render time: {:?}ms", instant.elapsed().as_millis());
        }).await;*/
        window.draw_if_needed(|renderer| {
            let instant = embassy_time::Instant::now();
            renderer.render_by_line(DisplayWrapper {
                display: &mut display,
                line_buffer: &mut line_buffer,
            });
            debug!("render time: {:?}ms", instant.elapsed().as_millis());
        });

        if window.has_active_animations() {
            continue;
        }

        if let Some(duration) = slint::platform::duration_until_next_timer_update() {
            debug!("Waiting for next timer update: {:?}", duration);
            Timer::after_millis(duration.as_millis() as u64).await;
        }

    }
}

struct DisplayWrapper<'a, T>{
    display: &'a mut T,
    line_buffer: &'a mut [Rgb565Pixel],
}

impl<'a, T> DisplayWrapper<'a, T> {}

impl slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, PicoDisplay>
{
    type TargetPixel = Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        //debug!("before send to dislay");
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        //SHARED.signal(self.line_buffer);

        //let instant = embassy_time::Instant::now();
         self.display.set_pixels(
            range.start as _,
            line as _,
            range.end as _,
            line as _,
            self.line_buffer[range.clone()].iter()
                .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
        ).unwrap();
        //poll_once(future);
        //debug!("process line: {:?}", instant.elapsed());
        //debug!("after send to dislay")

        //self.executor.run(|spawner|{
            //self.spawner.spawn(main_loop_task(Box::pin(future))).unwrap();
            /*let result = self.spawner.spawn(test_task());
        match result {
            Ok(_) => {
                debug!("spawned test task");
            }
            Err(error) => {
                panic!("failed to spawn test task {}", error);
            }
        }*/
            //block_on(future).unwrap()
        //})
        //let _ = poll_once(future);
    }
}

#[embassy_executor::task()]
async fn main_loop_task(
    run_fn: core::pin::Pin<Box<dyn Future<Output = Result<(), ()>> + 'static>>,
) {
    run_fn.await.unwrap();
}