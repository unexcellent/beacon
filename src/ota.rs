use esp_idf_sys::{
    esp_err_t, esp_ota_begin, esp_ota_end, esp_ota_get_next_update_partition,
    esp_ota_handle_t, esp_ota_set_boot_partition, esp_ota_write,
    esp_partition_t, esp_restart, ESP_OK,
};

// Sequential-write mode: esp-idf erases sectors on demand as writes arrive.
const OTA_WITH_SEQUENTIAL_WRITES: usize = 0xFFFFFFFE;

const CMD_ANNOUNCE: u8 = 0x00;
const CMD_BEGIN:    u8 = 0x01;
const CMD_DATA:     u8 = 0x02;
const CMD_END:      u8 = 0x03;

pub enum OtaState {
    // u16 = chunk_size from the last ANNOUNCE (0 = not yet announced)
    Idle(u16),
    Writing(OtaWriter),
}

pub struct OtaWriter {
    handle:       esp_ota_handle_t,
    partition:    *const esp_partition_t,
    total:        u32,
    received:     u32,
    chunk_size:   u16,
    // Buffered first copy of the current chunk; cleared after the second copy arrives.
    pending:      Option<(u32, Vec<u8>)>,
    // Set when both copies of a chunk have wrong size; blocks further writes until
    // a new ANNOUNCE resets the session.
    offset_error: bool,
}

// OtaWriter is only ever accessed from the single main task.
unsafe impl Send for OtaWriter {}

impl OtaState {
    pub fn new() -> Self { Self::Idle(0) }

    pub fn handle(&mut self, payload: &[u8]) {
        if payload.is_empty() { return; }
        match payload[0] {
            CMD_ANNOUNCE => self.cmd_announce(&payload[1..]),
            CMD_BEGIN    => self.cmd_begin(&payload[1..]),
            CMD_DATA     => self.cmd_data(&payload[1..]),
            CMD_END      => self.cmd_end(),
            cmd          => log::warn!("OTA: unknown command 0x{:02x}", cmd),
        }
    }

    fn cmd_announce(&mut self, data: &[u8]) {
        let chunk_size = if data.len() >= 2 {
            u16::from_le_bytes([data[0], data[1]])
        } else {
            0
        };
        if matches!(self, OtaState::Writing(_)) {
            log::warn!("OTA: aborting in-progress session on new announce");
        }
        *self = OtaState::Idle(chunk_size);
        log::info!("OTA: update announced (chunk_size={})", chunk_size);
    }

    fn cmd_begin(&mut self, data: &[u8]) {
        if data.len() < 4 {
            log::error!("OTA BEGIN: payload too short ({} bytes)", data.len());
            return;
        }
        let total      = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let chunk_size = if let OtaState::Idle(sz) = self { *sz } else { 0 };

        if matches!(self, OtaState::Writing(_)) {
            log::warn!("OTA: aborting previous session");
        }

        unsafe {
            let partition = esp_ota_get_next_update_partition(core::ptr::null());
            if partition.is_null() {
                log::error!("OTA: no update partition (partition table missing OTA slots?)");
                return;
            }
            let mut handle: esp_ota_handle_t = 0;
            let err = esp_ota_begin(partition, OTA_WITH_SEQUENTIAL_WRITES, &mut handle);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA esp_ota_begin failed: 0x{:08x}", err);
                return;
            }
            *self = OtaState::Writing(OtaWriter {
                handle,
                partition,
                total,
                received: 0,
                chunk_size,
                pending: None,
                offset_error: false,
            });
        }
        log::info!("OTA: session started, expecting {} bytes (chunk_size={})", total, chunk_size);
    }

    fn cmd_data(&mut self, data: &[u8]) {
        let OtaState::Writing(w) = self else {
            log::warn!("OTA DATA: no active session");
            return;
        };

        if w.offset_error {
            log::error!("OTA DATA: offset error — send a new announce to reset");
            return;
        }

        // Packet format: [offset: u32 LE][chunk...]
        if data.len() < 5 {
            log::error!("OTA DATA: payload too short ({} bytes)", data.len());
            return;
        }
        let offset = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let raw    = &data[4..];

        let remaining = w.total.saturating_sub(offset);
        let expected  = (w.chunk_size as u32).min(remaining) as usize;

        // Trim to expected size if the transport appended extra bytes; detect
        // truncation (too few bytes) by leaving the short slice as-is.
        let chunk: Vec<u8> = if raw.len() >= expected {
            raw[..expected].to_vec()
        } else {
            raw.to_vec()
        };

        // First copy: buffer it and wait for the second.
        if w.pending.is_none() {
            w.pending = Some((offset, chunk));
            return;
        }

        // Second copy arrived — decide which to use.
        let (first_offset, first_chunk) = w.pending.take().unwrap();

        if offset != first_offset {
            log::error!(
                "OTA DATA: expected second copy at {:#x}, got {:#x} — offset error",
                first_offset, offset
            );
            w.offset_error = true;
            return;
        }

        let first_ok  = first_chunk.len() == expected;
        let second_ok = chunk.len() == expected;

        let to_write: Vec<u8> = if first_ok {
            first_chunk
        } else if second_ok {
            log::warn!(
                "OTA DATA: first copy at {:#x} dropped bytes ({}/{}), using second",
                offset, first_chunk.len(), expected
            );
            chunk
        } else {
            log::error!(
                "OTA DATA: both copies at {:#x} have wrong size (first={}, second={}, expected={}) — using second, offset error from now on",
                offset, first_chunk.len(), chunk.len(), expected
            );
            w.offset_error = true;
            chunk
        };

        let n = to_write.len();
        unsafe {
            let err = esp_ota_write(w.handle, to_write.as_ptr() as *const _, n);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA write failed at {:#x}: 0x{:08x}", offset, err);
                *self = OtaState::Idle(0);
                return;
            }
        }

        let OtaState::Writing(w) = self else { return; };
        w.received += n as u32;
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
                log::error!(
                    "OTA esp_ota_end failed: 0x{:08x} (image corrupt or incomplete?)",
                    err
                );
                *self = OtaState::Idle(0);
                return;
            }
            let err = esp_ota_set_boot_partition(w.partition);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA set_boot_partition failed: 0x{:08x}", err);
                *self = OtaState::Idle(0);
                return;
            }
        }

        log::info!("OTA: success — rebooting into new firmware");
        unsafe { esp_restart() };
    }
}
