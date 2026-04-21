use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::peripherals::Peripherals;
use std::thread;
use std::time::Duration;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut led = PinDriver::output(peripherals.pins.gpio2).unwrap();

    loop {
        led.set_high().unwrap();
        log::info!("LED on");
        thread::sleep(Duration::from_millis(500));

        led.set_low().unwrap();
        log::info!("LED off");
        thread::sleep(Duration::from_millis(500));
    }
}
