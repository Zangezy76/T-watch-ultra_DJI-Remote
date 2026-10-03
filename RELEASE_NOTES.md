# Release Notes

## DJI-Remote v2.1.0 — Camera state, confirmed recording, GPX per day

A custom BLE remote for the **DJI Osmo Action 5 Pro**, running on the
**LILYGO T-Watch Ultra** (ESP32-S3R8) as an Arduino sketch built on LilyGoLib.
It controls recording from the wrist, injects real-time GPS for overlay in
DJI Mimo, and logs GPX tracks to SD card.

### What's New in v2.1.0

- **Real camera state** — `Cam:72%` (awake, camera battery), `Cam:zz` (switched
  off but still linked over BLE), `Cam:wake`, `Cam:~~` (link lost).
- **Confirmed recording** — REC lights up only after the camera confirms.
- **Wake-on-REC** — A tap wakes a sleeping camera and starts recording (~3 s).
- **One GPX track per day** — A new segment per logger session; BITE marks are
  saved even with the logger off.
- **Logger = hold 1.5 s** — No more accidental toggles from slow taps or a wet
  screen.
- **Stability** — Thread-safety rework, phantom-tap and dark-screen-tap fixes,
  logger resume after a crash, no more log flood from GPS writes to a sleeping
  camera.
- **GPS injection in DJI's 48-byte layout** — From DJI's official demo; not
  field-verified yet (the old layout is one `#define` away).
- **Logs over USB** — `ls` / `cat <path>` on the serial port, no need to remove
  the SD card.

See [`CHANGELOG.md`](CHANGELOG.md) for the full list.

### What was in v2.0.0

- **BLE camera control** — Start/stop recording from the wrist.
- **Real-time GPS injection** — Coordinates sent to the camera every second;
  GPS overlay confirmed working in DJI Mimo.
- **GPX track logger** — Track points and waypoints (REC markers, manual "BITE"
  markers) saved to SD card, one file per day.
- **Two-zone touch UI** — Upper 2/3 controls the camera, lower 1/3 the logger.
- **Display auto-sleep + shake-to-wake** — Sleeps after 1 minute, wakes on a
  wrist shake (BHI260AP IMU).

### Hardware

| Component | Details |
|-----------|---------|
| Watch | LILYGO T-Watch Ultra (ESP32-S3R8, BLE 5.0) |
| Display | 2.01" AMOLED 410×502 (CO5300) |
| GPS | u-blox MIA-M10Q (built-in) |
| IMU | BHI260AP (shake-to-wake) |
| Camera | DJI Osmo Action 5 Pro |

### Flash Instructions (Arduino IDE)

This release ships as an Arduino sketch, not a pre-built web-flash binary.

1. Install **Arduino IDE 2.x** and the **esp32 by Espressif Systems** core
   **3.3.12** via Boards Manager (v2.0.0 was built with 3.3.8).
2. Install the libraries at the **exact** versions listed in
   [`WORKING_LIBRARIES.md`](WORKING_LIBRARIES.md) — do **not** let Arduino IDE
   auto-update them (LilyGoLib 0.1.0, SensorLib 0.3.3, RadioLib 7.4.0,
   LVGL 9.4.0, NimBLE-Arduino 2.5.0, TinyGPSPlus 1.1.0).
3. Open `arduino/DJI_Remote_T-Watch/DJI_Remote_T-Watch.ino`.
4. Select board: **LILYGO T-Watch Ultra (SX1262)**.
5. Set your camera's BLE MAC in the sketch:
   ```cpp
   static const char* CAMERA_MAC = "xx:xx:xx:xx:xx:xx";
   ```
   (Find it in DJI Mimo → Camera Settings → About.)
6. Upload via the COM port (auto-detected, no button press needed). If the
   sketch file was changed outside the IDE, reopen it first — an open tab may
   upload stale text.

### Field Validation

- **Moscow, 06.06.2026** — 5.7 m average drift, 18.5 m max (no RTK).
- **Kamchatka ascent, 20.06.2026** — 142 points over 2 h 21 min, 21 km,
  +1290 m of gain (413 → 1703 m), no dropped points.

---

### About the ESP-IDF firmware (M5Stack / Waveshare)

The original ESP-IDF firmware for M5Stack Basic V2.7 and Waveshare
ESP32-S3-LCD-1.9 remains in this repository as the protocol reference. Its
release history is documented in [`CHANGELOG.md`](CHANGELOG.md) under the
`v1.x` entries.
