//! Inbound commands from the payload bus and the update wire-format parsing.

// `parse_update_packet` is reached only from the ESP-only link collector; on the
// host it is compiled for tests to target, so silence dead-code there.
#![cfg_attr(not(target_os = "espidf"), allow(dead_code))]

// update wire protocol: first payload byte selects the command, the rest is
// little-endian command data.
const UPDATE_CMD_ANNOUNCE: u8 = 0x00;
const UPDATE_CMD_BEGIN: u8 = 0x01;
const UPDATE_CMD_DATA: u8 = 0x02;
const UPDATE_CMD_END: u8 = 0x03;

/// A command received from the payload board, returned by the payload link's
/// `receive` for the application to execute.
pub enum Command {
    Sstv,
    /// A firmware update was announced with the given chunk size.
    UpdateAnnounced(u16),
    /// A firmware update session starts, expecting this many bytes in total.
    UpdateBegin(u32),
    /// One block of firmware bytes starting at `offset`.
    UpdateData {
        offset: u32,
        data: Vec<u8>,
    },
    /// The sender considers the firmware transfer complete.
    UpdateEnd,
}

/// Parse one packet from the update socket into a [`Command`], or None (with a log)
/// if it is malformed.
pub(super) fn parse_update_packet(payload: &[u8]) -> Option<Command> {
    match *payload.first()? {
        UPDATE_CMD_ANNOUNCE => {
            let chunk_size = match payload.get(1..3) {
                Some(b) => u16::from_le_bytes([b[0], b[1]]),
                None => 0,
            };
            Some(Command::UpdateAnnounced(chunk_size))
        }
        UPDATE_CMD_BEGIN => match payload.get(1..5) {
            Some(b) => Some(Command::UpdateBegin(u32::from_le_bytes([
                b[0], b[1], b[2], b[3],
            ]))),
            None => {
                log::error!(
                    "update BEGIN: payload too short ({} bytes)",
                    payload.len() - 1
                );
                None
            }
        },
        UPDATE_CMD_DATA => match payload.get(1..5) {
            Some(b) if payload.len() > 5 => Some(Command::UpdateData {
                offset: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
                data: payload[5..].to_vec(),
            }),
            _ => {
                log::error!(
                    "update DATA: payload too short ({} bytes)",
                    payload.len() - 1
                );
                None
            }
        },
        UPDATE_CMD_END => Some(Command::UpdateEnd),
        cmd => {
            log::warn!("update: unknown command 0x{cmd:02x}");
            None
        }
    }
}
