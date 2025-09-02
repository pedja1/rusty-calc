
use std::{
    rc::Rc,
    slice,
    sync::mpsc::{self, Receiver, Sender},
    thread::{self},
    vec::Vec,
};

use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Timer};
use log::*;
use rusty_calc::{
    controller::{self, Action, Controller},
    simulator::hardware::HardwareSim,
};
use object_pool::{Pool, Reusable};
use sdl2::{
    event::Event, keyboard::Keycode, mouse::MouseButton, pixels::PixelFormatEnum, rect::Rect,
};
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        PointerEventButton, WindowAdapter, WindowEvent,
    },
    ComponentHandle,
};
use slint_generated::MainWindow;
use static_cell::StaticCell;
use rusty_calc::simulator::slint_backend::{StmBackend, TargetPixelType, DISPLAY_HEIGHT, DISPLAY_WIDTH};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
static POOL: StaticCell<Pool<Vec<TargetPixelType>>> = StaticCell::new();

fn main() {
    env_logger::builder().filter_level(log::LevelFilter::Debug).format_timestamp_nanos().init();

    thread::scope(|scope| {
        let (tx_render, rx_render) = mpsc::channel();
        let (tx_event, rx_event) = mpsc::channel();

        let pool = POOL.init(Pool::new(4, || {
            vec![TargetPixelType::default(); DISPLAY_WIDTH * DISPLAY_HEIGHT]
        }));

        scope.spawn(move || sdl2_render_loop(rx_render, tx_event).unwrap());
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(main_task(spawner, tx_render, rx_event, pool)).unwrap();
        });
    });
}

fn sdl2_render_loop(
    rx_render: Receiver<Reusable<'static, Vec<TargetPixelType>>>,
    tx_event: Sender<WindowEvent>,
) -> Result<(), String> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let window = video_subsystem
        .window("Demo", DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _)
        .position_centered()
        .opengl()
        .build()
        .map_err(|e| e.to_string())?;

    let mut canvas = window.into_canvas().build().map_err(|e| e.to_string())?;
    let texture_creator = canvas.texture_creator();

    let mut texture = texture_creator
        .create_texture_streaming(PixelFormatEnum::RGB565, DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _)
        .map_err(|e| e.to_string())?;

    canvas.clear();
    canvas.copy(&texture, None, Some(Rect::new(0, 0, DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _)))?;
    canvas.present();

    let mut event_pump = sdl_context.event_pump()?;

    loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    std::process::exit(0)
                }
                Event::KeyDown { keycode: Some(Keycode::LSHIFT), .. } => {
                    //controller::send_action(Action::HardwareUserBtnPressed(true))
                }
                Event::KeyUp { keycode: Some(Keycode::LSHIFT), .. } => {
                    //controller::send_action(Action::HardwareUserBtnPressed(false))
                }
                Event::MouseButtonDown {
                    timestamp: _timestamp,
                    window_id: _window_id,
                    which: _which,
                    mouse_btn,
                    clicks: _clicks,
                    x,
                    y,
                } => {
                    if mouse_btn == MouseButton::Left {
                        let button = PointerEventButton::Left;
                        let position = slint::PhysicalPosition::new(x, y).to_logical(1.0);
                        let event = WindowEvent::PointerPressed { position, button };
                        tx_event.send(event).unwrap();
                    }
                }
                Event::MouseButtonUp {
                    timestamp: _timestamp,
                    window_id: _window_id,
                    which: _which,
                    mouse_btn,
                    clicks: _clicks,
                    x,
                    y,
                } => {
                    if mouse_btn == MouseButton::Left {
                        let button = PointerEventButton::Left;
                        let position = slint::PhysicalPosition::new(x, y).to_logical(1.0);
                        let event = WindowEvent::PointerReleased { position, button };
                        tx_event.send(event).unwrap();
                    }
                }
                Event::MouseMotion {
                    timestamp: _timestamp,
                    window_id: _window_id,
                    which: _which,
                    mousestate,
                    x,
                    y,
                    xrel: _xrel,
                    yrel: _yrel,
                } => {
                    if mousestate.is_mouse_button_pressed(MouseButton::Left) {
                        let position = slint::PhysicalPosition::new(x, y).to_logical(1.0);
                        let event = WindowEvent::PointerMoved { position };
                        tx_event.send(event).unwrap();
                    }
                }

                _ => {}
            }
        }

        'render_buffers: loop {
            match rx_render.try_recv() {
                Ok(buf) => {
                    texture.with_lock(None, |buffer: &mut [u8], _pitch: usize| {
                        let buf_ptr = buf.as_ptr() as *const u8;
                        let buf_slice = unsafe { slice::from_raw_parts(buf_ptr, buf.len() * 2) };
                        buffer.copy_from_slice(buf_slice);
                        drop(buf); // returns buffer to pool
                    })?;
                    canvas.clear();
                    canvas.copy_ex(
                        &texture,
                        None,
                        Some(Rect::new(0, 0, DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _)),
                        0.0,
                        None,
                        false,
                        false,
                    )?;
                    canvas.present();
                }
                _ => {
                    // ignore
                    break 'render_buffers;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn main_task(
    spawner: Spawner,
    tx_render: Sender<Reusable<'static, Vec<TargetPixelType>>>,
    rx_event: Receiver<WindowEvent>,
    pool: &'static Pool<Vec<TargetPixelType>>,
) {
    let window = MinimalSoftwareWindow::new(RepaintBufferType::SwappedBuffers);
    window.set_size(slint::PhysicalSize::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32));
    let backend = Box::new(StmBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("slint gui setup complete");

    spawner.spawn(embassy_render_loop(window, tx_render, rx_event, pool)).unwrap();

    // give the render loop time to come up (otherwise it will draw a blank screen)
    Timer::after(Duration::from_millis(200)).await;
    let main_window = MainWindow::new().unwrap();
    main_window.show().expect("unable to show main window");

    info!("press LEFT SHIFT to simulate a hardware button press");

    let hardware = HardwareSim {};

    // run the gui controller loop
    let mut controller = Controller::new(&main_window, hardware);
    controller.run().await;
}

#[embassy_executor::task]
async fn embassy_render_loop(
    window: Rc<MinimalSoftwareWindow>,
    tx_render: Sender<Reusable<'static, Vec<TargetPixelType>>>,
    rx_event: Receiver<WindowEvent>,
    pool: &'static Pool<Vec<TargetPixelType>>,
) {
    info!("embassy_render_loop");

    loop {
        slint::platform::update_timers_and_animations();

        'event: loop {
            match rx_event.try_recv() {
                Ok(e) => {
                    window.dispatch_event(e);
                }
                Err(_) => break 'event,
            }
        }

        // redraw the entire window (otherwise we get partial redraws which are more complicated to deal with)
        window.request_redraw();

        let _is_dirty = window.draw_if_needed(|renderer| match pool.try_pull() {
            Some(mut buffer) => {
                renderer.render(&mut buffer, DISPLAY_WIDTH as _);
                tx_render.send(buffer).ok();
            }
            None => {
                // this happens when the MainWindow hasn't yet been created or if it has been closed by the user
            }
        });

        // for approx 60fps
        Timer::after(Duration::from_millis(16)).await;
    }
}