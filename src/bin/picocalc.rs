#![no_std]
#![no_main]
#![macro_use]
extern crate alloc;

use defmt::{debug, unwrap};
use embassy_executor::Executor;
use embassy_time::{Instant, Timer};
use rp_pico::{entry, hal, pac};
use rp_pico::hal::multicore::{Multicore, Stack};
//use defmt::{debug, info};
//use embassy_executor::Spawner;
use slint::{run_event_loop_until_quit, ComponentHandle};
use slint_generated::{MainWindow, SplashWindow, LauncherWindow};
use static_cell::StaticCell;
use rusty_calc::picocalc::pico_backend;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[entry]
#[allow(static_mut_refs)]
fn main() -> ! {
    // Initialize the embassy-based display and platform
    pico_backend::init();

    // steal here because already used in pico_backend::init()
    let mut pac = unsafe { pac::Peripherals::steal() };

    let _core = pac::CorePeripherals::take().unwrap();

    let mut sio = hal::sio::Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| spawner.spawn(core1_main()).unwrap());
    }).unwrap();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.spawn(core0_main()).unwrap());

}

#[embassy_executor::task]
async fn core0_main() {

    //info!("Starting embassy-based Slint application");

    debug!("Ticks: {}", Instant::now().as_millis());
    Timer::after_millis(1000).await;
    debug!("Ticks after: {}", Instant::now().as_millis());


    // Create the main window
    let main_window = LauncherWindow::new().unwrap();
    main_window.show().expect("unable to show main window");
    //let splash_window = SplashWindow::new().unwrap();
    //splash_window.show().expect("unable to show main window");
    run_event_loop_until_quit().expect("event loop failed");

    // TASK: run the gui render loop
    //spawner.spawn(update_progress(main_window)).unwrap();

    // The event loop is now handled by the embassy-based platform
    panic!("The event loop should not return");
}


#[embassy_executor::task]
async fn core1_main() {
    loop {
        //debug!("run core1_main loop");
        Timer::after_millis(1000).await;
    }
}
