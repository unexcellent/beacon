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
