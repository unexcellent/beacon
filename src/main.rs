use esp_idf_sys::{
    EspError, esp,
    uart_config_t,
    uart_driver_install,
    uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE as HW_FLOWCTRL_DISABLE,
    uart_mode_t_UART_MODE_RS485_HALF_DUPLEX as RS485_HALF_DUPLEX,
    uart_param_config,
    uart_parity_t_UART_PARITY_DISABLE as PARITY_DISABLE,
    uart_port_t_UART_NUM_1 as UART1,
    uart_read_bytes,
    uart_set_mode,
    uart_set_pin,
    uart_stop_bits_t_UART_STOP_BITS_1 as STOP_BITS_1,
    uart_word_length_t_UART_DATA_8_BITS as DATA_8_BITS,
};

const BAUD_RATE: i32 = 9600;
const TX_PIN: i32 = 30;
const DE_PIN: i32 = 31; // RTS — controls THVD1424 DE line in RS485 half-duplex mode
const RX_PIN: i32 = 32;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("RS485 receive test — UART1 / {} baud", BAUD_RATE);

    unsafe {
        let config = uart_config_t {
            baud_rate: BAUD_RATE,
            data_bits: DATA_8_BITS,
            parity: PARITY_DISABLE,
            stop_bits: STOP_BITS_1,
            flow_ctrl: HW_FLOWCTRL_DISABLE,
            rx_flow_ctrl_thresh: 0,
            ..Default::default()
        };

        esp!(uart_driver_install(UART1, 1024, 0, 0, std::ptr::null_mut(), 0)).unwrap();
        esp!(uart_param_config(UART1, &config)).unwrap();
        esp!(uart_set_pin(UART1, TX_PIN, RX_PIN, DE_PIN, -1)).unwrap();
        esp!(uart_set_mode(UART1, RS485_HALF_DUPLEX)).unwrap();
    }

    log::info!("Listening for RS485 data…");

    let mut buf = [0u8; 64];
    loop {
        let len = unsafe {
            uart_read_bytes(
                UART1,
                buf.as_mut_ptr().cast(),
                buf.len() as u32,
                100, // ticks (~100 ms at default tick rate)
            )
        };

        if len > 0 {
            log::info!("Received {} byte(s): {:02x?}", len, &buf[..len as usize]);
        }
    }
}
