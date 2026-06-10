use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use esp_idf_sys::{
    esp,
    gpio_config,
    gpio_config_t,
    gpio_get_level,
    gpio_install_isr_service,
    gpio_int_type_t_GPIO_INTR_ANYEDGE as GPIO_INTR_ANYEDGE,
    gpio_isr_handler_add,
    gpio_mode_t_GPIO_MODE_INPUT as GPIO_INPUT,
    gpio_mode_t_GPIO_MODE_OUTPUT as GPIO_OUTPUT,
    gpio_set_level,
    vTaskDelay,
};

const MONITOR_PINS: [i32; 2] = [37, 39];
const DE_PIN: i32 = 38;

static EDGES: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
static LEVEL: [AtomicU8; 2] = [AtomicU8::new(0), AtomicU8::new(0)];

unsafe extern "C" fn on_edge(arg: *mut core::ffi::c_void) {
    let idx = arg as usize;
    let level = gpio_get_level(MONITOR_PINS[idx]) as u8;
    LEVEL[idx].store(level, Ordering::Relaxed);
    EDGES[idx].fetch_add(1, Ordering::Relaxed);
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    unsafe {
        // Configure GPIO 38 as output and drive it HIGH
        let out_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << DE_PIN,
            mode: GPIO_OUTPUT,
            pull_up_en: 0,
            pull_down_en: 0,
            intr_type: 0,
            ..Default::default()
        };
        esp!(gpio_config(&out_cfg)).unwrap();
        esp!(gpio_set_level(DE_PIN, 0)).unwrap();

        // Configure GPIO 37 and 39 as inputs with edge interrupts
        let in_mask: u64 = MONITOR_PINS.iter().fold(0u64, |acc, &p| acc | (1u64 << p));
        let in_cfg = gpio_config_t {
            pin_bit_mask: in_mask,
            mode: GPIO_INPUT,
            pull_up_en: 0,
            pull_down_en: 0,
            intr_type: GPIO_INTR_ANYEDGE,
            ..Default::default()
        };
        esp!(gpio_config(&in_cfg)).unwrap();
        esp!(gpio_install_isr_service(0)).unwrap();
        for (idx, &pin) in MONITOR_PINS.iter().enumerate() {
            esp!(gpio_isr_handler_add(pin, Some(on_edge), idx as *mut core::ffi::c_void)).unwrap();
        }
    }

    log::info!("GPIO {} (DE) held LOW; watching for voltage changes on GPIOs {:?}", DE_PIN, MONITOR_PINS);

    let mut last_edges = [0u32; 2];
    loop {
        unsafe { vTaskDelay(500) };
        for (idx, &pin) in MONITOR_PINS.iter().enumerate() {
            let edges = EDGES[idx].load(Ordering::Relaxed);
            if edges != last_edges[idx] {
                let level = LEVEL[idx].load(Ordering::Relaxed);
                let direction = if level == 1 { "HIGH (rising)" } else { "LOW (falling)" };
                log::info!(
                    "Voltage change on GPIO {}: {} — total edges: {}",
                    pin, direction, edges
                );
                last_edges[idx] = edges;
            }
        }
    }
}
