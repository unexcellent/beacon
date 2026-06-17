use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{InterruptType, PinDriver, Pull},
    peripherals::Peripherals,
};

static EDGES: AtomicU32 = AtomicU32::new(0);

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // DE held LOW = receive mode on the THVD1424RGTR
    let mut de = PinDriver::output(peripherals.pins.gpio39).unwrap();
    de.set_low().unwrap();

    let mut rx = PinDriver::input(peripherals.pins.gpio37, Pull::Floating).unwrap();
    rx.set_interrupt_type(InterruptType::AnyEdge).unwrap();
    unsafe {
        rx.subscribe(|| {
            EDGES.fetch_add(1, Ordering::Relaxed);
        })
        .unwrap();
    }
    rx.enable_interrupt().unwrap();

    log::info!("Monitoring G37 for edges via interrupt (DE=G39 held LOW)");

    let mut last = 0u32;
    loop {
        FreeRtos::delay_ms(100);
        rx.enable_interrupt().unwrap(); // re-arm after each ISR firing

        let total = EDGES.load(Ordering::Relaxed);
        if total != last {
            log::info!("G37: {} new edge(s) (total {})", total - last, total);
            last = total;
        }
    }
}
