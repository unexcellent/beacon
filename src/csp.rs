/// CSP v1 packet header (32-bit big-endian).
///
/// Layout: priority[31:30] src[29:25] dst[24:20] dport[19:14] sport[13:8] flags[7:0]
pub struct CspHeader {
    pub priority: u8,
    pub src: u8,
    pub dst: u8,
    pub dport: u8,
    pub sport: u8,
    pub flags: u8,
}

impl CspHeader {
    /// Parse the first 4 bytes of `data` as a CSP v1 header.
    /// Returns the header and the remaining payload slice on success.
    pub fn parse(data: &[u8]) -> Option<(Self, &[u8])> {
        if data.len() < 4 {
            return None;
        }
        let word = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        Some((
            Self {
                priority: ((word >> 30) & 0x03) as u8,
                src: ((word >> 25) & 0x1F) as u8,
                dst: ((word >> 20) & 0x1F) as u8,
                dport: ((word >> 14) & 0x3F) as u8,
                sport: ((word >> 8) & 0x3F) as u8,
                flags: (word & 0xFF) as u8,
            },
            &data[4..],
        ))
    }
}
