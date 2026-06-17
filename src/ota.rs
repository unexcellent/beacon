use esp_idf_sys::{
    esp_err_t,
    esp_ota_begin,
    esp_ota_end,
    esp_ota_get_next_update_partition,
    esp_ota_handle_t,
    esp_ota_set_boot_partition,
    esp_ota_write,
    esp_partition_t,
    esp_restart,
    ESP_OK,
};

// Tells esp_ota_begin that writes will be sequential (faster internal path).
const OTA_WITH_SEQUENTIAL_WRITES: usize = 0xFFFFFFFE;

pub enum OtaState {
    Idle,
    Writing(OtaWriter),
}

pub struct OtaWriter {
    handle: esp_ota_handle_t,
    partition: *const esp_partition_t,
    total: u32,
    received: u32,
}

// Safety: OtaWriter is only ever accessed from the single main task.
unsafe impl Send for OtaWriter {}

impl OtaState {
    pub fn new() -> Self {
        Self::Idle
    }

    /// Dispatch an incoming OTA payload (first byte = command).
    pub fn handle(&mut self, payload: &[u8]) {
        if payload.is_empty() {
            return;
        }
        match payload[0] {
            0x00 => log::info!("OTA: incoming firmware update announced"),
            0x01 => self.cmd_begin(&payload[1..]),
            0x02 => self.cmd_data(&payload[1..]),
            0x03 => self.cmd_end(),
            cmd  => log::warn!("OTA: unknown command 0x{:02x}", cmd),
        }
    }

    fn cmd_begin(&mut self, data: &[u8]) {
        if data.len() < 4 {
            log::error!("OTA BEGIN: payload too short ({} bytes)", data.len());
            return;
        }
        let total = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);

        if matches!(self, OtaState::Writing(_)) {
            log::warn!("OTA BEGIN: aborting previous session");
            *self = OtaState::Idle;
        }

        unsafe {
            let partition = esp_ota_get_next_update_partition(core::ptr::null());
            if partition.is_null() {
                log::error!("OTA: no update partition available (partition table missing OTA slots?)");
                return;
            }
            let mut handle: esp_ota_handle_t = 0;
            let err = esp_ota_begin(partition, OTA_WITH_SEQUENTIAL_WRITES, &mut handle);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA esp_ota_begin failed: 0x{:08x}", err);
                return;
            }
            *self = OtaState::Writing(OtaWriter { handle, partition, total, received: 0 });
        }

        log::info!("OTA: session started, expecting {} bytes", total);
    }

    fn cmd_data(&mut self, data: &[u8]) {
        let OtaState::Writing(w) = self else {
            log::warn!("OTA DATA: no active session");
            return;
        };
        if data.len() < 5 {
            log::error!("OTA DATA: payload too short ({} bytes)", data.len());
            return;
        }
        // First 4 bytes are offset (for progress display only; writes are sequential).
        let offset = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let chunk = &data[4..];

        unsafe {
            let err = esp_ota_write(w.handle, chunk.as_ptr() as *const _, chunk.len());
            if err != ESP_OK as esp_err_t {
                log::error!("OTA write failed at offset {}: 0x{:08x}", offset, err);
                *self = OtaState::Idle;
                return;
            }
        }

        w.received += chunk.len() as u32;
        let pct = w.received as f32 / w.total as f32 * 100.0;
        log::info!("OTA: {}/{} bytes ({:.0}%)", w.received, w.total, pct);
    }

    fn cmd_end(&mut self) {
        let OtaState::Writing(w) = self else {
            log::warn!("OTA END: no active session");
            return;
        };

        log::info!("OTA: finalizing ({} bytes written)...", w.received);

        unsafe {
            let err = esp_ota_end(w.handle);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA esp_ota_end failed: 0x{:08x} (image corrupt or incomplete?)", err);
                *self = OtaState::Idle;
                return;
            }

            let err = esp_ota_set_boot_partition(w.partition);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA set_boot_partition failed: 0x{:08x}", err);
                *self = OtaState::Idle;
                return;
            }
        }

        log::info!("OTA: success — rebooting into new firmware");
        unsafe { esp_restart() };
    }
}
