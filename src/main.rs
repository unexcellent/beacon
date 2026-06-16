mod csp;
mod kiss;
mod ota;

use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartRxDriver},
    units::Hertz,
};

const BAUD_RATE: u32 = 115_200;
const RX_PIN: i32 = 37;
const DE_PIN: i32 = 38;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut de = PinDriver::output(peripherals.pins.gpio38).unwrap();
    de.set_low().unwrap();

    let config = uart::config::Config::new().baudrate(Hertz(BAUD_RATE));

    let uart = UartRxDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio37,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    )
    .unwrap();

    log::info!(
        "Listening for UART at {} baud on GPIO {} (DE on GPIO {} held LOW)",
        BAUD_RATE,
        RX_PIN,
        DE_PIN,
    );

    let csp = csp::CspStack::new();
    let mut kiss = kiss::KissDecoder::new();
    let mut ota = ota::OtaState::new();
    let mut buf = [0u8; 64];

    loop {
        match uart.read(&mut buf, delay::BLOCK) {
            Ok(n) => {
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
            Err(e) => log::warn!("UART read error: {:?}", e),
        }
    }
}
