//! Firmware update: receives the announced image chunk by chunk via the
//! payload link, writes it to the next update partition and reboots into it.

use core::ptr;

use esp_idf_sys::{
    ESP_OK, esp_err_t, esp_ota_begin, esp_ota_end, esp_ota_get_next_update_partition,
    esp_ota_handle_t, esp_ota_set_boot_partition, esp_ota_write, esp_partition_t, esp_restart,
};

use crate::error::{Error, Result};
use crate::link::{Command, CommandLink};

/// ESP-IDF's `OTA_WITH_SEQUENTIAL_WRITES` sentinel for `esp_ota_begin`: image
/// size unknown, written sequentially.
const UPDATE_WITH_SEQUENTIAL_WRITES: usize = 0xFFFFFFFE;

/// Run one firmware update session after its announcement. Keeps polling the
/// link until the transfer completes (which reboots into the new firmware) or
/// fails. SSTV commands received in the meantime are ignored; ping stays alive
/// since it is serviced inside [`CommandLink::receive`].
pub fn update<L: CommandLink>(chunk_size: u16, link: &mut L) -> Result<()> {
    let mut state = UpdateState::Announced;

    loop {
        match link.receive()? {
            Some(Command::UpdateAnnounced(size)) => {
                log::warn!("update: aborting in-progress session on new announce");
                update(size, link)?;
            }
            Some(Command::UpdateBegin(total)) => state.begin(total, chunk_size)?,
            Some(Command::UpdateData { offset, data }) => state.write_data(offset, &data)?,
            Some(Command::UpdateEnd) => state.finish()?,
            Some(_) => log::warn!("command ignored during firmware update"),
            None => continue,
        }
    }
}

enum UpdateState {
    Announced,
    InProgress(Writer),
    Done,
}

impl UpdateState {
    pub fn begin(&mut self, total: u32, chunk_size: u16) -> Result<()> {
        *self = Self::InProgress(Writer::begin(total, chunk_size)?);
        Ok(())
    }

    pub fn write_data(&mut self, base_offset: u32, raw: &[u8]) -> Result<()> {
        match self {
            Self::InProgress(writer) => writer.write_data(base_offset, raw),
            _ => Err(Error::UpdateNotInProgress),
        }
    }

    fn finish(&mut self) -> Result<()> {
        match core::mem::replace(self, Self::Done) {
            Self::InProgress(writer) => writer.finish(),
            state => {
                *self = state;
                Err(Error::UpdateNotInProgress)
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
    pub fn begin(total: u32, chunk_size: u16) -> Result<Self> {
        unsafe {
            let partition = esp_ota_get_next_update_partition(ptr::null());
            if partition.is_null() {
                return Err(Error::UpdatePartition);
            }

            let mut handle: esp_ota_handle_t = 0;
            let result = esp_ota_begin(partition, UPDATE_WITH_SEQUENTIAL_WRITES, &mut handle);
            if result != ESP_OK as esp_err_t {
                return Err(Error::UpdateBegin);
            }

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
    pub fn write_data(&mut self, base_offset: u32, raw: &[u8]) -> Result<()> {
        if base_offset != self.received {
            return Err(Error::UpdatePackageOffset(self.received, base_offset));
        }

        let payload = self.strip_relay_overhead(base_offset, raw);

        // The payload may batch several consecutive chunks: flash them one by one.
        let mut pos = 0;
        while pos < payload.len() {
            let offset = base_offset + pos as u32;
            let remaining_image = self.total.saturating_sub(offset) as usize;
            if remaining_image == 0 {
                break; // bytes beyond the image length are relay overhead
            }

            let expected = if self.chunk_size > 0 {
                remaining_image.min(self.chunk_size as usize)
            } else {
                payload.len() - pos // chunk size unknown: take everything
            };
            let chunk = &payload[pos..pos + expected.min(payload.len() - pos)];

            self.check_chunk_complete(offset, chunk.len(), expected)?;
            self.write_chunk(offset, chunk)?;
            pos += chunk.len();
        }
        Ok(())
    }

    /// The relay may append 4 bytes (CRC without flag) and/or batch multiple
    /// consecutive update packets into one CSP frame. Strip trailing bytes that
    /// are relay overhead: anything beyond the last complete chunk, unless the
    /// remaining firmware bytes are smaller than chunk_size (final chunk).
    fn strip_relay_overhead<'a>(&self, base_offset: u32, raw: &'a [u8]) -> &'a [u8] {
        let chunk_size = self.chunk_size as usize;
        if chunk_size == 0 {
            return raw;
        }

        let remaining_total = self.total.saturating_sub(base_offset) as usize;
        if remaining_total <= chunk_size {
            &raw[..remaining_total.min(raw.len())]
        } else {
            let n_chunks = raw.len() / chunk_size;
            if n_chunks > 0 {
                &raw[..n_chunks * chunk_size]
            } else {
                raw
            }
        }
    }

    /// A chunk shorter than expected is only acceptable as the image's last chunk.
    fn check_chunk_complete(&self, offset: u32, got: usize, expected: usize) -> Result<()> {
        let is_last = offset + got as u32 >= self.total;
        let chunk_is_incomplete = self.chunk_size > 0 && got < expected;
        if chunk_is_incomplete && !is_last {
            return Err(Error::UpdateChunkIncomplete(
                offset,
                got as u32,
                expected as u32,
            ));
        }
        Ok(())
    }

    fn write_chunk(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        let data_len = data.len();
        unsafe {
            let err = esp_ota_write(self.handle, data.as_ptr() as *const _, data_len);
            if err != ESP_OK as esp_err_t {
                return Err(Error::UpdateWrite(offset, err));
            }
        }
        self.received += data_len as u32;
        let percentage = self.received as f32 / self.total as f32 * 100.0;

        log::info!("{:.0}% | {} b arrived", percentage, data_len);
        Ok(())
    }

    /// Verify completeness, validate the image and reboot into it.
    pub fn finish(self) -> Result<()> {
        if self.received != self.total {
            return Err(Error::UpdateIncomplete(self.received, self.total));
        }

        unsafe {
            let err = esp_ota_end(self.handle);
            if err != ESP_OK as esp_err_t {
                return Err(Error::UpdateCorrupt);
            }

            let err = esp_ota_set_boot_partition(self.partition);
            if err != ESP_OK as esp_err_t {
                return Err(Error::UpdateCorrupt);
            }
        }

        unsafe { esp_restart() };
    }
}
