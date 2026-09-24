# beacon

Library crate for the Slow-Scan Television Payload of the MOVE-III satellite on
the ESP32-P4: device drivers (camera, audio), the CSP/KISS payload link, the
Robot36 SSTV transmitter, OTA firmware update, and the idle command loop.

The per-carrier firmware binaries live in their own repos and depend on this
crate:

- [`beacon-on-moveiiia`](../beacon-on-moveiiia) — the MOVE-IIIa payload carrier.
- [`beacon-on-tab5`](../beacon-on-tab5) — the M5Stack Tab5 development board.

Each supplies its own board bring-up (transports, sensor pin maps, `main`) and
imports the mission logic here unchanged.
