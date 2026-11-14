# Release Notes

## DJI-Remote v2.1.1 — T-Watch Ultra release

A custom BLE remote for the **DJI Osmo Action 5 Pro**, running on the
**LILYGO T-Watch Ultra** (ESP32-S3R8) as an Arduino sketch built on LilyGoLib.
It controls recording from the wrist, injects real-time GPS for overlay in
DJI Mimo, and logs GPX tracks to SD card.

### What's in v2.1.1

- **GPX timestamps at midnight** — Track points wait for a matching GPS date
  and UTC time; the logger writes the point when the receiver confirms them.
- **Waypoints before UTC is available** — BITE and REC marks retain their
  coordinates without a `<time>` element. With the logger off, undated BITE
  marks go to `/track_undated_wpt.gpx`.
- **T-Watch-only repository** — Removed the inherited ESP-IDF firmware and its
  M5Stack/Waveshare documentation. The watch photo is now easier to read.

The release also includes the T-Watch features introduced in v2.1.0:

- **Real camera state** — `Cam:72%` (awake, camera battery), `Cam:zz` (switched
  off but still linked over BLE), `Cam:wake`, `Cam:~~` (link lost).
- **Confirmed recording** — REC lights up only after the camera confirms.
- **Wake-on-REC, the DJI-remote way** — One press wakes a sleeping camera and
  starts recording (~4–5 s); after STOP the camera goes back to sleep by itself.
  Works even after the link was lost while the camera slept (~10 s).
- **Side button** — The lower left button records (click), marks BITE (double
  click) and toggles the logger (hold 1.5 s). Rain and a sleeve cuff caused
  hundreds of false REC taps on the touch screen; the button needs a real press.
- **One GPX track per day** — A new segment per logger session; BITE marks are
  saved even with the logger off.
- **Logger = hold 1.5 s** — No more accidental toggles from slow taps or a wet
  screen.
- **Stability** — Thread-safety rework, phantom-tap and dark-screen-tap fixes,
  no false REC resets, no stalls on a PC's USB port, logger resume after a crash,
  no more log flood from GPS writes to a sleeping camera.
- **GPS injection in DJI's 48-byte layout** — From DJI's official demo;
  confirmed in the field (overlay in DJI Mimo).
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
5. Replace the example address below with the BLE MAC shown for your camera
   in DJI Mimo → Camera Settings → About.

   ```cpp
   #define CAMERA_MAC_ADDR "AA:BB:CC:DD:EE:FF"
   ```

6. Upload via the COM port (auto-detected, no button press needed). If the
   sketch file was changed outside the IDE, reopen it first — an open tab may
   upload stale text.

### Field Validation

- **Moscow, 06.06.2026** — 5.7 m average drift, 18.5 m max (no RTK).
- **Kamchatka ascent, 20.06.2026** — 142 points over 2 h 21 min, 21 km,
  +1290 m of gain (413 → 1703 m), no dropped points.

The v2.1.1 GPX fixes passed host-side checks and the T-Watch sketch compiled
with ESP32 core 3.3.12. The GPX fixes have not yet been tested on the watch.
