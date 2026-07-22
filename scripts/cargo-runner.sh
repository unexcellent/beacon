#!/usr/bin/env bash
#
# cargo-runner.sh — the `cargo run` runner for this project.
#
# `cargo run` invokes this with the built ELF as $1. We flash with the esp-idf-built
# bootloader and partition table from the ELF's own profile dir, because the project
# uses a non-default config (HEX/200 MHz PSRAM for the camera buffer and a two-OTA
# partition layout). espflash's built-in default bootloader/partition table do NOT
# match, so a plain `espflash flash` boots a firmware whose PSRAM and partitions are
# misconfigured. This mirrors what local/capture_frame_via_usb_c.sh flashes.
#
# Falls back to espflash's defaults if the artifacts aren't present yet, so a fresh
# checkout still flashes something rather than erroring.
set -euo pipefail

ELF="$1"; shift || true
BIN_DIR="$(dirname "$ELF")"

ARGS=(--chip esp32p4)
[ -f "$BIN_DIR/bootloader.bin" ]      && ARGS+=(--bootloader "$BIN_DIR/bootloader.bin")
[ -f "$BIN_DIR/partition-table.bin" ] && ARGS+=(--partition-table "$BIN_DIR/partition-table.bin")

exec espflash flash --monitor "${ARGS[@]}" "$ELF"
