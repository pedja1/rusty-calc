#![no_std]
#![no_main]
#![macro_use]
extern crate alloc;

use defmt::{debug, unwrap};
use embassy_executor::{Executor, Spawner};
use embassy_time::{Instant, Timer};
use rp_pico::{entry, hal, pac};
use rp_pico::hal::multicore::{Multicore, Stack};
//use defmt::{debug, info};
//use embassy_executor::Spawner;
use slint::{run_event_loop_until_quit, ComponentHandle, PlatformError};
use slint_generated::{MainWindow, SplashWindow, LauncherWindow};
use static_cell::StaticCell;
//use rusty_calc::controller::Controller;
use rusty_calc::picocalc::pico_backend;


#[entry]
fn main() -> ! {

    pico_backend::init(core0_run, core1_run);

    panic!("The main function should not return");
}

fn core0_run(spawner: Spawner) {
    spawner.spawn(ui_task()).unwrap();
    spawner.spawn(event_loop_task()).unwrap();
}
fn core1_run(spawner: Spawner) {
    spawner.spawn(core1_task()).unwrap()
}

#[embassy_executor::task]
async fn event_loop_task() {

    loop {
        //debug!("free heap: {}, used heap: {}", pico_backend::free_heap(), pico_backend::used_heap());
        Timer::after_millis(1000).await;
    }
    panic!("The event loop should not return");
}

#[embassy_executor::task]
async fn ui_task() {
    debug!("ui _tasl");
    // Create the main window
    //let main_window = MainWindow::new().unwrap();
    let splash_window = SplashWindow::new().unwrap();
    splash_window.show().unwrap();

    Timer::after_millis(2000).await;
    debug!("hiding splash");
    splash_window.hide().unwrap();

    let launcher_window = LauncherWindow::new().unwrap();
    launcher_window.show().unwrap();

    Timer::after_millis(2000).await;
    debug!("hiding launcher");
    launcher_window.hide().unwrap();

    loop {
        Timer::after_millis(1000).await;
    }

    //main_window.show().expect("unable to show main window");

    //let mut controller = Controller::new(&main_window, hardware);
    //controller.run().await;
}


#[embassy_executor::task]
async fn core1_task() {
    loop {
        //debug!("run core1_main loop");
        Timer::after_millis(1000).await;
    }
}
