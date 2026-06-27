# Release Notes

## DJI-Remote v2.0.0 — T-Watch Ultra Arduino port

A custom BLE remote for the **DJI Osmo Action 5 Pro**, running on the
**LILYGO T-Watch Ultra** (ESP32-S3R8) as an Arduino sketch built on LilyGoLib.
It controls recording from the wrist, injects real-time GPS for overlay in
DJI Mimo, and logs GPX tracks to SD card.

### What's New in v2.0.0

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
   **3.3.8** via Boards Manager.
2. Install the libraries at the **exact** versions listed in
   [`WORKING_LIBRARIES.md`](WORKING_LIBRARIES.md) — do **not** let Arduino IDE
   auto-update them (LilyGoLib 0.1.0, SensorLib 0.3.3, RadioLib 7.4.0,
   LVGL 9.4.0, NimBLE-Arduino 2.5.0, TinyGPSPlus).
3. Open `arduino/DJI_TWatch_R