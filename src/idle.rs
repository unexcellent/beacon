//! The beacon's main loop: wait for commands on the payload link and service
//! them — capture-and-transmit SSTV on request, and apply firmware updates.

use crate::audio::AudioChannel;
use crate::camera::Camera;

use crate::error::ReportIfErr;
use crate::link::{Command, CommandLink, Message};
use crate::transmit_sstv::transmit_sstv;
#[cfg(target_os = "espidf")]
use crate::update::update;

/// Run the beacon's main loop: block on `link` and service each command.
///
/// - [`Command::Sstv`] captures and transmits an SSTV image per camera (see
///   [`transmit_sstv`]), bracketed by [`Busy`](Message::Busy) /
///   [`Available`](Message::Available) status messages.
/// - [`Command::UpdateAnnounced`] hands off to the firmware [`update`] session,
///   which reboots into the new image on success.
/// - Any other command, and any link error, is ignored after reporting.
///
/// Link errors and per-command failures are logged and downlinked via
/// [`ReportIfErr`] rather than propagated. Owns `cameras` so it can re-lend them
/// on every request. Never returns.
pub fn idle<L: CommandLink>(
    link: &mut L,
    mut cameras: Vec<Option<Box<dyn Camera>>>,
    audio: &mut impl AudioChannel,
) -> ! {
    loop {
        match link.receive().report_if_err(&*link) {
            Ok(Some(Command::Sstv)) => {
                link.send(Message::Busy);
                let _ = transmit_sstv(&mut cameras, audio).report_if_err(&*link);
                link.send(Message::Available);
            }
            #[cfg(target_os = "espidf")]
            Ok(Some(Command::UpdateAnnounced(chunk_size))) => {
                let _ = update(chunk_size, link).report_if_err(&*link);
            }
            _ => (),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::panic::{AssertUnwindSafe, catch_unwind};

    use super::idle;
    use crate::camera::Camera;
    use crate::error::{Error, Result};
    use crate::link::Command;
    use crate::test_support::{FakeAudio, ScriptedLink, ok};

    /// Drive `idle` (which never returns) until the scripted link is exhausted
    /// and panics the loop, then hand back the link for message inspection.
    /// Uses no cameras so the SSTV path is a fast no-op and only the command
    /// dispatch is under test.
    fn run_idle(script: Vec<Result<Option<Command>>>) -> ScriptedLink {
        let mut link = ScriptedLink::new(script);
        let mut audio = FakeAudio::new();
        let cameras: Vec<Option<Box<dyn Camera>>> = Vec::new();

        let outcome = catch_unwind(AssertUnwindSafe(|| {
            idle(&mut link, cameras, &mut audio);
        }));
        assert!(
            outcome.is_err(),
            "idle should only end via the sentinel panic"
        );
        link
    }

    #[test]
    fn sstv_command_is_bracketed_by_busy_and_available() {
        let link = run_idle(vec![ok(Command::Sstv)]);
        assert_eq!(link.sent_kinds(), ["BUSY", "AVAILABLE"]);
    }

    #[test]
    fn link_error_is_reported_and_loop_continues() {
        let link = run_idle(vec![Err(Error::UartReceive), ok(Command::Sstv)]);
        assert_eq!(link.sent_kinds(), ["ERROR", "BUSY", "AVAILABLE"]);
    }

    #[test]
    fn idle_poll_sends_nothing() {
        let link = run_idle(vec![Ok(None)]);
        assert!(link.sent_kinds().is_empty());
    }
}
