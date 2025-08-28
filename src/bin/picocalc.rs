#![no_std]
#![no_main]
#![macro_use]
extern crate alloc;

use cortex_m_rt::entry;
use defmt::debug;
use embassy_time::Instant;
//use defmt::{debug, info};
//use embassy_executor::Spawner;
use slint::{run_event_loop_until_quit, ComponentHandle};
use slint_generated::SplashWindow;
use rusty_calc::picocalc::pico_backend;

#[entry]
fn main() -> !{
    // Initialize the embassy-based display and platform
    pico_backend::init();

    //info!("Starting embassy-based Slint application");

    debug!("Ticks: {}", Instant::now().as_millis());

    // Create the main window
    let main_window = SplashWindow::new().unwrap();
    main_window.show().expect("unable to show main window");
    run_event_loop_until_quit().expect("event loop failed");

    // TASK: run the gui render loop
    //spawner.spawn(update_progress(main_window)).unwrap();

    // The event loop is now handled by the embassy-based platform
    panic!("The event loop should not return");
}

/*#[embassy_executor::task()]
pub async fn update_progress(
    _window: MainWindow,
) {
    loop {
        debug!("run update_progress loop");
        //Timer::after_millis(1000).await;
    }
}*/
