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
- **Waypoint/trackpoint collision** — A single seek-from-end append let each new
  trackpoint overwrite the previous waypoint; waypoints now go to a separate
  `_wpt.gpx` file per session.
- **Signal indicator glyphs** — `◆/◇` symbols aren't included in the Montserrat
  LVGL build; replaced with ASCII bars (`|||||` … `.....`).
- **HDOP parsing** — The MIA-M10Q emits `$GNGGA`, from which TinyGPSPlus does not
  parse HDOP; the indicator falls back to satellite count.

### Notes

- **BLE on a FreeRTOS task** — All BLE work runs in a dedicated task; calling
  `connect()` from `setup()` hangs the system. UI is updated via flags only.
- **Field validation** —
  Moscow (06.06.2026): 5.7 m average drift, 18.5 m max — no RTK.
  Kamchatka ascent (20.06.2026): 142 points over 2 h 21 min, 21 km, +1290 m
  (413 → 1703 m), with no dropped points.

---

## Inherited ESP-IDF firmware (M5Stack / Waveshare)

The following releases predate the T-Watch Ultra port and describe the upstream
ESP-IDF firmware retained in this repository.

### [v1.2.0]

#### Added

- **Waveshare ESP32-S3-LCD-1.9 support** — New HAL, ESP32-S3 target,
  320x170 landscape IPS display (ST7789V2).
- **Dual-target release builds** — Two merged binaries produced per release,
  one per hardware target.
- **UI layout for 320x170** — Adaptive layout system supporting both
  320x240 (M5Stack) and 320x170 (Waveshare) screen resolutions.
- **GPS Kconfig** — GPS UART pins and baud rate configurable per board
  via Kconfig (no hardcoded pin assignments).

#### Removed

- **Manual flash ZIP and scripts** — Web-flash only from this release onward.

### [v1.1.0]

#### Added

- **LVGL 9.5.0 UI rendering** — All screens replaced with LVGL widget-based
  rendering via esp_lvgl_port 2.7.2. Replaces manual TFT/SPI drawing.
- **NimBLE BLE stack** — Apache NimBLE replaces Bluedroid as the GATT client
  stack.
- **Boot splash screen** — LVGL-rendered splash screen replaces the raw bitmap
  boot logo.
- **Release packaging automation** — GitHub Actions workflow and
  `build_release.sh` produce merged binary and ZIP for every tagged release.

#### Fixed

- **NimBLE scan fixes** — Device names now shown during scan; duplicate filter
  disabled; reconnect-on-unpair prevented.

#### Changed

- **Production log level** — Default log level set to ERROR for production
  builds.

### [v1.0.0]

Initial public release. Firmware for M5Stack Basic V2.7 (ESP32).

#### Features

- Control up to three DJI Osmo Action cameras simultaneously over BLE
- Live GPS forwarding to all connected cameras (10 Hz)
- Start/stop recording, highlight tags, sleep/wake, snapshot-while-sleeping
- Mode switching via QS button emulation
- Automatic boot-time scanning and reconnection
- Multi-camera action coordination with sequential wake queue
- Optional external hardware buttons (GPIO26, GPIO21, GPIO22)
- Supported cameras: Action 4, Action 5 Pro, Action 6, Osmo 360
