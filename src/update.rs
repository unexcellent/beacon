//! Firmware update (OTA): receives the announced image chunk by chunk via the
//! payload link, writes it to the next OTA partition and reboots into it.

use core::ptr;

use esp_idf_sys::{
    ESP_OK, esp_err_t, esp_ota_begin, esp_ota_end, esp_ota_get_next_update_partition,
    esp_ota_handle_t, esp_ota_set_boot_partition, esp_ota_write, esp_partition_t, esp_restart,
};

use crate::devices::kiss::fmt_payload;
use crate::devices::link::{Command, PayloadLink};
use crate::error::{Error, Result};

const OTA_WITH_SEQUENTIAL_WRITES: usize = 0xFFFFFFFE;

/// Run one firmware update session after its announcement. Keeps polling the
/// link until the transfer completes (which reboots into the new firmware) or
/// fails. SSTV commands received in the meantime are ignored; ping stays alive
/// since it is serviced inside [`PayloadLink::poll`].
pub fn update(mut chunk_size: u16, link: &mut PayloadLink) -> Result<()> {
    log::info!("OTA: update announced (chunk_size={chunk_size})");
    let mut writer: Option<Writer> = None;

    loop {
        for cmd in link.poll() {
            match cmd {
                Command::UpdateAnnounced(size) => {
                    if writer.take().is_some() {
                        log::warn!("OTA: aborting in-progress session on new announce");
                    }
                    chunk_size = size;
                    log::info!("OTA: update announced (chunk_size={chunk_size})");
                }
                Command::UpdateBegin(total) => {
                    if writer.take().is_some() {
                        log::warn!("OTA: aborting previous session");
                    }
                    writer = Some(Writer::begin(total, chunk_size)?);
                }
                Command::UpdateData { offset, data } => match writer.as_mut() {
                    Some(w) => {
                        if let Err(e) = w.write_data(offset, &data) {
                            return Err(e);
                        }
                    }
                    None => log::warn!("OTA DATA: no active session"),
                },
                Command::UpdateEnd => match writer.take() {
                    Some(w) => w.finish()?,
                    None => log::warn!("OTA END: no active session"),
                },
                _ => log::warn!("command ignored during firmware update"),
            }
        }
    }
}

/// An open esp_ota session writing sequentially to the next update partition.
struct Writer {
    handle: esp_ota_handle_t,
    partition: *const esp_partition_t,
    total: u32,
    received: u32,
    chunk_size: u16,
}

impl Writer {
    fn begin(total: u32, chunk_size: u16) -> Result<Self> {
        unsafe {
            let partition = esp_ota_get_next_update_partition(ptr::null());
            if partition.is_null() {
                log::error!("OTA: no update partition (partition table missing OTA slots?)");
                return Err(Error::Update);
            }
            let mut handle: esp_ota_handle_t = 0;
            let err = esp_ota_begin(partition, OTA_WITH_SEQUENTIAL_WRITES, &mut handle);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA esp_ota_begin failed: 0x{:08x}", err);
                return Err(Error::Update);
            }
            log::info!(
                "OTA: session started, expecting {} bytes (chunk_size={})",
                total,
                chunk_size
            );
            Ok(Self {
                handle,
                partition,
                total,
                received: 0,
                chunk_size,
            })
        }
    }

    /// Validate and flash one DATA payload starting at `base_offset`.
    fn write_data(&mut self, base_offset: u32, raw: &[u8]) -> Result<()> {
        if base_offset != self.received {
            log::error!(
                "OTA DATA: gap detected — expected offset {:#x}, got {:#x} — aborting",
                self.received,
                base_offset
            );
            return Err(Error::Update);
        }

        // The relay may append 4 bytes (CRC without flag) and/or batch multiple
        // consecutive OTA packets into one CSP frame. Strip trailing bytes that
        // are relay overhead: anything beyond the last complete chunk, unless the
        // remaining firmware bytes are smaller than chunk_size (final chunk).
        let payload = if self.chunk_size > 0 {
            let cs = self.chunk_size as usize;
            let remaining_total = self.total.saturating_sub(base_offset) as usize;
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
            let remaining = self.total.saturating_sub(offset);
            if remaining == 0 {
                break;
            }
            let expected = if self.chunk_size > 0 {
                (self.chunk_size as u32).min(remaining) as usize
            } else {
                payload.len() - pos
            };
            let slice = &payload[pos..pos + expected.min(payload.len() - pos)];
            let is_last = offset + slice.len() as u32 >= self.total;

            if self.chunk_size > 0 && slice.len() < expected && !is_last {
                log::error!(
                    "OTA DATA: chunk at {:#x} too short (got={}, expected={}) — aborting",
                    offset,
                    slice.len(),
                    expected
                );
                return Err(Error::Update);
            }

            self.write_chunk(offset, slice)?;
            pos += slice.len();
        }
        Ok(())
    }

    fn write_chunk(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        let n = data.len();
        unsafe {
            let err = esp_ota_write(self.handle, data.as_ptr() as *const _, n);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA write failed at {:#x}: 0x{:08x}", offset, err);
                return Err(Error::Update);
            }
        }
        self.received += n as u32;
        let pct = self.received as f32 / self.total as f32 * 100.0;

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
        Ok(())
    }

    /// Verify completeness, validate the image and reboot into it.
    fn finish(self) -> Result<()> {
        if self.received != self.total {
            log::error!(
                "OTA END: incomplete — received {} of {} bytes — aborting",
                self.received,
                self.total
            );
            return Err(Error::Update);
        }

        log::info!("OTA: finalizing ({} bytes written)...", self.received);

        unsafe {
            let err = esp_ota_end(self.handle);
            if err != ESP_OK as esp_err_t {
                log::error!(
                    "OTA esp_ota_end failed: 0x{:08x} (image corrupt or incomplete?)",
                    err
                );
                return Err(Error::Update);
            }
            let err = esp_ota_set_boot_partition(self.partition);
            if err != ESP_OK as esp_err_t {
                log::error!("OTA set_boot_partition failed: 0x{:08x}", err);
                return Err(Error::Update);
            }
        }

        log::info!("OTA: success — rebooting into new firmware");
        unsafe { esp_restart() };
    }
}
