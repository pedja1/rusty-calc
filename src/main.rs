#![no_std]
#![no_main]

slint::slint!(export { MainWindow } from "src/main.slint";);

fn main() {
    let main_window = MainWindow::new().unwrap();
    main_window.run().unwrap();
}
