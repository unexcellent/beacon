mod csp;
mod kiss;
mod ota;

use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, Output, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};

const BAUD_RATE: u32 = 115_200;
const TX_PIN: i32 = 38;
const RX_PIN: i32 = 37;
const DE_PIN: i32 = 39;

fn send_beacon(uart: &UartDriver, de: &mut PinDriver<'_, Output>) {
    let frame = csp::CspStack::beacon_frame();
    de.set_high().unwrap();
    let mut sent = 0;
    while sent < frame.len() {
        sent += uart.write(&frame[sent..]).unwrap();
    }
    uart.flush_write().unwrap();
    // Wait one byte-period for the shift register to drain before releasing the bus.
    delay::FreeRtos::delay_ms(1);
    de.set_low().unwrap();
    log::info!("Beacon sent ({} bytes)", frame.len());
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut de = PinDriver::output(peripherals.pins.gpio39).unwrap();
    de.set_low().unwrap();

    let config = uart::config::Config::new().baudrate(Hertz(BAUD_RATE));

    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    )
    .unwrap();

    log::info!(
        "Listening for UART at {} baud on GPIO {} (TX on GPIO {}, DE on GPIO {} held LOW)",
        BAUD_RATE, RX_PIN, TX_PIN, DE_PIN,
    );

    let csp = csp::CspStack::new();
    let mut kiss = kiss::KissDecoder::new();
    let mut ota = ota::OtaState::new();

    send_beacon(&uart, &mut de);

    let mut buf = [0u8; 64];

    loop {
        if let Ok(n) = uart.read(&mut buf, delay::BLOCK) {
            for &b in &buf[..n] {
                if let Some(frame) = kiss.push(b) {
                    csp.inject(&frame);
                    csp.route_work();
                    if let Some(pkt) = csp.recv_ota() {
                        ota.handle(pkt.data());
                    }
                }
            }
        }
    }
}
