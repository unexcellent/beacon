//! Shared fakes for the host-side behavioural tests of [`idle`](crate::idle)
//! and [`transmit_sstv`](crate::transmit_sstv). Compiled only under `cfg(test)`.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use sstv::RgbPixel;

use crate::audio::{AudioChannel, AudioError};
use crate::camera::{Camera, Image};
use crate::error::Result;
use crate::link::{Command, CommandLink, Message};

/// Robot36 expects a 320x240 frame, so build one that `Encoder::new` accepts.
pub(crate) fn fake_robot36_frame() -> Image {
    let (w, h) = (320, 240);
    Image::from_pixels(w, h, vec![RgbPixel::new(0, 128, 255); w * h])
}

/// The order of lifecycle calls a [`FakeCamera`] received, e.g.
/// `["power_on", "calibrate", "receive_frame", "power_off"]`. Stays readable
/// after the camera is handed off to the code under test.
#[derive(Clone)]
pub(crate) struct CallLog(Rc<RefCell<Vec<&'static str>>>);

impl CallLog {
    pub(crate) fn new() -> Self {
        Self(Rc::new(RefCell::new(Vec::new())))
    }

    pub(crate) fn append_call(&self, call: &'static str) {
        self.0.borrow_mut().push(call);
    }

    pub(crate) fn calls(&self) -> Vec<&'static str> {
        self.0.borrow().clone()
    }
}

/// A camera slot holding a working [`FakeCamera`] whose call log is not needed.
pub(crate) fn working_camera() -> Option<Box<dyn Camera>> {
    Some(FakeCamera::boxed().0)
}

/// A camera slot whose camera failed to initialize.
pub(crate) fn missing_camera() -> Option<Box<dyn Camera>> {
    None
}

/// A camera that records its lifecycle calls and yields a fixed frame.
pub(crate) struct FakeCamera {
    log: CallLog,
}

impl FakeCamera {
    /// Returns the boxed camera and a handle to its call log.
    pub(crate) fn boxed() -> (Box<dyn Camera>, CallLog) {
        let log = CallLog::new();
        (Box::new(Self { log: log.clone() }), log)
    }
}

impl Camera for FakeCamera {
    fn power_on(&mut self) {
        self.log.append_call("power_on");
    }
    fn power_off(&mut self) {
        self.log.append_call("power_off");
    }
    fn calibrate(&mut self) {
        self.log.append_call("calibrate");
    }
    fn receive_frame(&mut self) -> Image {
        self.log.append_call("receive_frame");
        fake_robot36_frame()
    }
}

/// Counts the samples and flushes an [`AudioChannel`] receives, and can be told
/// to start failing `transmit` after a given number of samples.
pub(crate) struct FakeAudio {
    pub(crate) samples: usize,
    flushes: usize,
    fail_after: Option<usize>,
}

impl FakeAudio {
    /// Number of completed SSTV transmissions. `transmit_image` ends every
    /// image with exactly one `flush()`, so counting flushes counts images.
    pub(crate) fn completed_transmissions(&self) -> usize {
        self.flushes
    }

    pub(crate) fn new() -> Self {
        Self {
            samples: 0,
            flushes: 0,
            fail_after: None,
        }
    }

    /// A channel whose `transmit` fails once `n` samples have been accepted.
    pub(crate) fn failing_after(n: usize) -> Self {
        Self {
            samples: 0,
            flushes: 0,
            fail_after: Some(n),
        }
    }
}

impl AudioChannel for FakeAudio {
    fn sample_rate(&self) -> u32 {
        8000
    }

    fn transmit(&mut self, _sample: i16) -> core::result::Result<(), AudioError> {
        if self.fail_after.is_some_and(|n| self.samples >= n) {
            return Err(AudioError::Transmission);
        }
        self.samples += 1;
        Ok(())
    }

    fn flush(&mut self) -> core::result::Result<(), AudioError> {
        self.flushes += 1;
        Ok(())
    }
}

/// Sentinel a [`ScriptedLink`] panics with once its script is exhausted, so a
/// `-> !` loop driven under `catch_unwind` terminates.
pub(crate) const SCRIPT_EXHAUSTED: &str = "scripted link exhausted (test sentinel)";

/// A [`CommandLink`] that returns a scripted sequence of `receive` results and
/// records every message sent.
pub(crate) struct FakeLink {
    inbound: VecDeque<Result<Option<Command>>>,
    sent: RefCell<Vec<Message>>,
}

impl FakeLink {
    pub(crate) fn new(inbound: Vec<Result<Option<Command>>>) -> Self {
        Self {
            inbound: inbound.into(),
            sent: RefCell::new(Vec::new()),
        }
    }

    /// Labels of the messages sent so far, for order-sensitive assertions.
    pub(crate) fn sent_kinds(&self) -> Vec<&'static str> {
        self.sent.borrow().iter().map(message_kind).collect()
    }
}

impl CommandLink for FakeLink {
    fn send(&self, message: Message) {
        self.sent.borrow_mut().push(message);
    }

    fn receive(&mut self) -> Result<Option<Command>> {
        self.inbound
            .pop_front()
            .unwrap_or_else(|| panic!("{SCRIPT_EXHAUSTED}"))
    }
}

fn message_kind(m: &Message) -> &'static str {
    match m {
        Message::Available => "AVAILABLE",
        Message::Busy => "BUSY",
        Message::Booted(_) => "BOOTED",
        Message::Error(_) => "ERROR",
    }
}

/// Convenience for scripting a successfully received command.
pub(crate) fn ok(cmd: Command) -> Result<Option<Command>> {
    Ok(Some(cmd))
}
