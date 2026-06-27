# Changelog

All notable changes to this project are documented in this file.

The current focus of this repository is the **T-Watch Ultra Arduino port**
(`arduino/DJI_TWatch_Remote/`). Earlier `v1.x` entries describe the original
ESP-IDF firmware for M5Stack / Waveshare boards that this project was forked
from; that firmware still lives in the repo (`protocol/`, `ble/`, `logic/`,
`main/`, `sdkconfig.defaults.*`) and serves as the protocol reference.

## [v2.0.0] — 2026-06 — T-Watch Ultra Arduino port

A from-scratch Arduino implementation for the **LILYGO T-Watch Ultra**
(ESP32-S3R8) built on LilyGoLib, controlling a **DJI Osmo Action 5 Pro**.
The DJI BLE protocol was reverse-engineered from the original ESP-IDF codebase;
the watch application itself is new.

### Added

- **BLE camera control** — Start/stop recording from the wrist
  (CMD_SET `0x1D` / CMD_ID `0x03`) over the DJI BLE protocol.
- **Real-time GPS injection** — Live coordinates pushed to the camera every
  second (CMD_SET `0x00` / CMD_ID `0x17`, 45-byte struct). GPS overlay
  confirmed working in DJI Mimo.
- **GPX track logger** — Track points and waypoints (REC START/STOP markers,
  manual "BITE" markers) written to SD card, one file per day.
- **Two-zone touch UI (LVGL)** — Upper 2/3 controls the camera, lower 1/3
  controls the logger; double-tap to connect / save a waypoint.
- **Display auto-sleep + shake-to-wake** — Screen sleeps after 1 minute, wakes
  on wrist shake via the BHI260AP IMU (delta > 3.0).
- **GPS via LilyGoLib** — Uses `instance.gps` (u-blox MIA-M10Q, UART1 @ 38400),
  satellite/HDOP signal-strength indicator, and a speed noise filter
  (< 2 km/h reported as 0).
- **Verified library set** — Pinned, compatible versions documented in
  `WORKING_LIBRARIES.md` (esp32 core 3.3.8, LilyGoLib 0.1.0, NimBLE 2.5.0,
  LVGL 9.4.0, TinyGPSPlus).

### Fixed

- **GPX truncation** — `FILE_WRITE` (`"w"`) truncates on esp32 core 3.x;
  switched to seek-based append with `"r+"` so headers and prior points survive.
- **