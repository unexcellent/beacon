const FEND: u8 = 0xC0;
const FESC: u8 = 0xDB;
const TFEND: u8 = 0xDC;
const TFESC: u8 = 0xDD;

enum State {
    Idle,
    Command,
    Data,
    Escape,
}

/// KISS deframer. Feed bytes via `push`; returns a completed frame payload when
/// the closing FEND arrives. Only data frames (command byte 0x00) are returned.
pub struct KissDecoder {
    state: State,
    buf: Vec<u8>,
}

impl KissDecoder {
    pub fn new() -> Self {
        Self {
            state: State::Idle,
            buf: Vec::new(),
        }
    }

    pub fn push(&mut self, b: u8) -> Option<Vec<u8>> {
        match self.state {
            State::Idle => {
                if b == FEND {
                    self.state = State::Command;
                }
                None
            }
            State::Command => {
                match b {
                    FEND => None, // consecutive FENDs are allowed
                    0x00 => {
                        self.buf.clear();
                        self.state = State::Data;
                        None
                    }
                    _ => {
                        // Non-data command — skip to next FEND
                        self.state = State::Idle;
                        None
                    }
                }
            }
            State::Data => {
                match b {
                    FEND => {
                        if self.buf.is_empty() {
                            return None;
                        }
                        let frame = self.buf.clone();
                        self.buf.clear();
                        self.state = State::Command;
                        Some(frame)
                    }
                    FESC => {
                        self.state = State::Escape;
                        None
                    }
                    _ => {
                        self.buf.push(b);
                        None
                    }
                }
            }
            State::Escape => {
                let decoded = match b {
                    TFEND => FEND,
                    TFESC => FESC,
                    _ => {
                        self.state = State::Idle;
                        return None;
                    }
                };
                self.buf.push(decoded);
                self.state = State::Data;
                None
            }
        }
    }
}
