use esp_idf_sys::{
    ESP_OK, esp_err_t, esp_ota_begin, esp_ota_end, esp_ota_get_next_update_partition,
    esp_ota_handle_t, esp_ota_set_boot_partition, esp_ota_write, esp_partition_t, esp_restart,
};

use crate::devices::kiss::fmt_payload;

const OTA_WITH_SEQUENTIAL_WRITES: usize = 0xFFFFFFFE;

const CMD_ANNOUNCE: u8 = 0x00;
const CMD_BEGIN: u8 = 0x01;
const CMD_DATA: u8 = 0x02;
const CMD_END: u8 = 0x03;

pub enum OtaState {
    Idle(u16),
    Writing(OtaWriter),
}

pub struct OtaWriter {
    handle: esp_ota_handle_t,
    partition: *const esp_partition_t,
    total: u32,
    received: u32,
    chunk_size: u16,
}

unsafe impl Send for OtaWriter {}

impl OtaState {
    pub fn new() -> Self {
        Self::Idle(0)
    }

    pub fn handle(&mut self, payload: &[u8]) {
        if payload.is_empty() {
            return;
        }
        match payload[0] {
            CMD_ANNOUNCE => self.cmd_announce(&payload[1..]),
            CMD_BEGIN => self.cmd_begin(&payload[1..]),
            CMD_DATA => self.cmd_data(&payload[1..]),
            CMD_END => self.cmd_end(),
            cmd => log::warn!("OTA: unknown command 0x{:02x}", cmd),
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
        let total = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
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
            });
        }
        log::info!(
            "OTA: session started, expecting {} bytes (chunk_size={})",
            total,
            chunk_size
        );
    }

    fn cmd_data(&mut self, data: &[u8]) {
        let OtaState::Writing(w) = self else {
            log::warn!("OTA DATA: no active session");
            return;
        };

        if data.len() < 5 {
            log::error!("OTA DATA: payload too short ({} bytes)", data.len());
            *self = OtaState::Idle(0);
            return;
        }

        let base_offset = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let raw = &data[4..];

        if base_offset != w.received {
            log::error!(
                "OTA DATA: gap detected — expected offset {:#x}, got {:#x} — aborting",
                w.received,
                base_offset
            );
            *self = OtaState::Idle(0);
            return;
        }

        // The relay may append 4 bytes (CRC without flag) and/or batch multiple
        // consecutive OTA packets into one CSP frame. Strip trailing bytes that
        // are relay overhead: anything beyond the last complete chunk, unless the
        // remaining firmware bytes are smaller than chunk_size (final chunk).
        let payload = if w.chunk_size > 0 {
            let cs = w.chunk_size as usize;
            let remaining_total = w.total.saturating_sub(base_offset) as usize;
            if remaining_total <= cs {
                // Final chunk — take only the remaining firmware bytes.
                let take = remaining_total.min(raw.len());
                &raw[..take]
            } else {
                // Mid-transfer — strip to a whole number of full chunks.
                let n_chunks = raw.len() / cs;
                if n_chunks > 0 {
                    &raw[..n_chunks * cs]
                } else {
                    raw
                }
            }
        } else {
            raw
        };

        let mut pos = 0usize;
        while pos < payload.len() {
            let offset = base_offset + pos as u32;
            let remaining = w.total.saturating_sub(offset);
            if remaining == 0 {
                break;
            }
            let expected = if w.chunk_size > 0 {
                (w.chunk_size as u32).min(remaining) as usize
            } else {
                payload.len() - pos
            };
            let slice = &payload[pos..pos + expected.min(payload.len() - pos)];
            let is_last = offset + slice.len() as u32 >= w.total;

            if w.chunk_size > 0 && slice.len() < expected && !is_last {
                log::error!(
                    "OTA DATA: chunk at {:#x} too short (got={}, expected={}) — aborting",
                    offset,
                    slice.len(),
                    expected
                );
                *self = OtaState::Idle(0);
                return;
            }

            if !Self::write_chunk(w, offset, slice) {
                *self = OtaState::Idle(0);
                return;
            }

            pos += slice.len();
        }
    }

    fn write_chunk(w: &mut OtaWriter, offset: u32, data: &[u8]) -> bool {
        let n = data.len();
        unsafe {
            let err = esp_ota_write(w.handle, data.as_ptr() as *const _, n);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA write failed at {:#x}: 0x{:08x}", offset, err);
                return false;
            }
        }
        w.received += n as u32;
        let pct = w.received as f32 / w.total as f32 * 100.0;

        if n < 33 {
            log::info!(
                "{:.0}% | {} b arrived: {:02x?}",
                pct.floor(),
                n,
                fmt_payload(data)
            );
        } else {
            log::info!("{:.0}% | {} b arrived", pct, n);
        }

        true
    }

    fn cmd_end(&mut self) {
        let OtaState::Writing(w) = self else {
            log::warn!("OTA END: no active session");
            return;
        };

        if w.received != w.total {
            log::error!(
                "OTA END: incomplete — received {} of {} bytes — aborting",
                w.received,
                w.total
            );
            *self = OtaState::Idle(0);
            return;
        }

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
