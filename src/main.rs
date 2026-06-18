use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};

const BAUD_RATE: u32 = 115_200;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // DE held LOW = receive mode on the THVD1424RGTR
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

    log::info!("Listening on UART1 at {} baud (RX=G37, DE=G39 held LOW)", BAUD_RATE);

    let mut buf = [0u8; 64];

    loop {
        if let Ok(n) = uart.read(&mut buf, delay::BLOCK) {
            log::info!("RX {} byte(s): {:02x?}", n, &buf[..n]);
        }
    }
}
