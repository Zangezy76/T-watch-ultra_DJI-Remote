/**
 * DJI Osmo Action 5 Pro BLE Remote on LILYGO T-Watch Ultra
 *
 * Features:
 *   - BLE camera control: REC confirmed by the camera's reply; a sleeping
 *     camera is woken up first; camera awake/asleep + battery % on screen
 *   - Real-time GPS injection → overlay in DJI Mimo
 *   - GPX track logger: one file per day, new segment per session,
 *     BITE waypoints saved even with the logger off (SD card)
 *   - Two touch zones: upper=camera, lower=logger (hold 1.5 s) / BITE
 *   - Display auto-sleep + wake by wrist shake (BHI260AP)
 *   - Read-only SD access over USB serial ("ls", "cat <path>")
 *
 * Hardware:
 *   - LILYGO T-Watch Ultra (ESP32-S3R8, BLE 5.0)
 *   - DJI Osmo Action 5 Pro
 *
 * Author: Zangezy76
 * License: MIT
 * Repository: https://github.com/Zangezy76/T-watch-ultra_DJI-Remote
 */

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <NimBLEDevice.h>
#include <SD.h>
#include <FS.h>
#include <bosch/BoschSensorDataHelper.hpp>

// ─── Camera BLE address and device ID ─────────────────────
// Set your camera's BLE MAC address here
// Find it in DJI Mimo → Camera Settings → About
static const char* CAMERA_MAC   = "xx:xx:xx:xx:xx:xx";  // <-- set your DJI camera BLE MAC
static const uint32_t DEVICE_ID = 0x00000015;

// ─── GPX file format ──────────────────────────────────────
// Creation: FILE_APPEND — creates file with header + closing tags
// Track points / waypoints: "r+" + seek to (size - close_len)
// "r+" = open read+write, no truncate. FILE_WRITE ("w") truncates on esp32 core 3.x!
// instance.begin() already mounts SD internally — use SD.exists("/") to check.
#define TRKSEG_CLOSE     "    </trkseg>\n  </trk>\n"
#define GPX_CLOSE        "</gpx>\n"
// Lengths derived from the strings (hard-coded 22/29 were off by one)
#define TRKSEG_CLOSE_LEN (sizeof(TRKSEG_CLOSE)-1)
#define GPX_CLOSE_LEN    (sizeof(GPX_CLOSE)-1)
#define FULL_CLOSE_LEN   (TRKSEG_CLOSE_LEN+GPX_CLOSE_LEN)
static char  gpxPath[32] = "";
static bool  sdReady     = false;
static bool  gpxLogging  = false;
static int   trackSession = 0;
static uint32_t lastTrackMs = 0;
#define TRACK_INTERVAL_MS 60000UL
static volatile bool wantConnected = false;
static char  wptPath[36] = "";

// ─── SD event log ─────────────────────────────────────────
static char logPath[32] = "";
static int  lastLogDay  = -1;
static uint32_t startMs = 0;
static bool firstFix    = false;
static uint32_t lastBatLogMs = 0;
#define BAT_LOG_INTERVAL_MS 1800000UL

// ─── Display sleep/wake ───────────────────────────────────
#define DISPLAY_TIMEOUT_MS 60000UL
static uint32_t lastActivityMs = 0;
static bool displayOn = true;

// ─── Accelerometer for shake-to-wake ─────────────────────
SensorXYZ accel(SensorBHI260AP::ACCEL_PASSTHROUGH, instance.sensor);
static float prevAccelMag = 0.0f;
static bool  accelPrimed  = false;   // first sample after sleep is only a baseline
static uint32_t sleepStartMs = 0;

// ─── UI widgets ───────────────────────────────────────────
#define ZONE_SPLIT 318   // touch boundary; the blue zone line is drawn here (was 335 vs line at 318)
static lv_obj_t* lblTime   = nullptr;
static lv_obj_t* lblRec    = nullptr;
static lv_obj_t* lblCoords = nullptr;
static lv_obj_t* lblSpeed  = nullptr;
static lv_obj_t* lblCamSat = nullptr;
static lv_obj_t* lblLog    = nullptr;
static lv_obj_t* lblBatSd  = nullptr;

// ─── BLE state ────────────────────────────────────────────
// Thread ownership (LVGL, SD and TinyGPS are not thread-safe):
//   loop()       — LVGL, SD, instance.gps, isRecording
//   bleTask      — pClient, pWriteChr, seqNum (every write to the camera)
//   onDisconnect — NimBLE host task, only sets flags
static NimBLEClient*               pClient   = nullptr;
static NimBLERemoteCharacteristic* pWriteChr = nullptr;
static volatile bool connected      = false;
static volatile bool camLost        = false;   // link dropped, reconnecting (UI: orange "~~")
static volatile int  disconnectReason = 0;
static bool          isRecording    = false;
static uint16_t      seqNum         = 0;
static volatile bool triggerConnect = false;
static volatile bool wakeReq        = false;   // loop() → bleTask: send wake-up advertising

// Camera awake/asleep. The camera keeps the BLE link while switched off, so
// "connected" alone means nothing. While awake it pushes its battery state once
// a second (DUML 0D/02, see onCamNotify); silence = asleep. A 3 s gap was seen
// while awake, hence 6 s. If no push was ever seen on this link (other firmware?)
// the state is unknown and the watch behaves as before.
#define CAM_HEARTBEAT_MS 6000UL
static volatile uint32_t camHeartbeatMs = 0;   // millis() of the last battery push, 0 = none yet
static volatile uint32_t camLinkMs      = 0;   // millis() when the current link came up
static volatile uint8_t  camBattPct     = 0;
static volatile bool     camWaking      = false;   // bleTask is waking the camera for REC

// REC reply handoff: onCamNotify (NimBLE host task) → bleTask
static volatile bool     recRespNew = false;
static volatile uint16_t recRespSeq = 0;
static volatile uint8_t  recRespRet = 0;

// BLE dump for step 24 research (see "BLE dump" below). Declared up here because
// Arduino inserts its generated function prototypes before the first function.
#define DEBUG_BLE_DUMP 0
#define DUMP_MAX_LEN   160
enum { DUMP_RX = 0, DUMP_TX = 1, DUMP_NOTE = 2 };
struct DumpItem { uint32_t ms; uint8_t dir; uint16_t len; uint8_t data[DUMP_MAX_LEN]; };

// REC command handoff: loop() → bleTask → loop()
#define REC_FAILED 2
static volatile int8_t recCmd  = 0;   // +1 start, -1 stop; set by loop(), cleared by bleTask
static volatile int8_t recDone = 0;   // +1/-1 sent OK, REC_FAILED; set by bleTask, cleared by loop()

// GPS injection handoff: loop() snapshots instance.gps, bleTask sends it.
// Two layouts of the same data, see buildGpsPayload(); GPS_FRAME_LEGACY picks the one in use.
#define GPS_FRAME_LEGACY 0
static portMUX_TYPE  gpsMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t       gpsPayload[48];         // DJI demo layout
static uint8_t       gpsPayloadLegacy[45];   // our layout up to v20
static volatile bool gpsPayloadReady = false;
// Camera's opinion of each layout: both are sent with a reply request once per link
static volatile bool     gpsProbePending = false;   // set by bleTask on connect
static volatile bool     gpsReplyNew = false;       // onCamNotify → bleTask
static volatile uint16_t gpsReplySeq = 0;
static volatile uint8_t  gpsReplyRet = 0;
static uint32_t      lastGpsPayloadMs = 0;

static uint32_t lastUiUpdate = 0;

// ─── Touch state ──────────────────────────────────────────
#define LOGGER_HOLD_MS 1500   // was 600: slow taps / wet screen toggled the logger by accident
static uint32_t touchStartMs = 0;
static bool touchActive = false;
static bool touchWasOn = false;       // display was on when this touch began
static bool touchHoldFired = false;   // logger hold already handled for this touch
static int16_t touchX = 0, touchY = 0;
static uint32_t lastTouchMs = 0;

// ═══════════════════════════════════════════════════════════
// DJI CRC16 — Fletcher variant, init=0x3AA3
// ═══════════════════════════════════════════════════════════
static const uint16_t crc16_table[256] = {
    0x0000,0xc0c1,0xc181,0x0140,0xc301,0x03c0,0x0280,0xc241,
    0xc601,0x06c0,0x0780,0xc741,0x0500,0xc5c1,0xc481,0x0440,
    0xcc01,0x0cc0,0x0d80,0xcd41,0x0f00,0xcfc1,0xce81,0x0e40,
    0x0a00,0xcac1,0xcb81,0x0b40,0xc901,0x09c0,0x0880,0xc841,
    0xd801,0x18c0,0x1980,0xd941,0x1b00,0xdbc1,0xda81,0x1a40,
    0x1e00,0xdec1,0xdf81,0x1f40,0xdd01,0x1dc0,0x1c80,0xdc41,
    0x1400,0xd4c1,0xd581,0x1540,0xd701,0x17c0,0x1680,0xd641,
    0xd201,0x12c0,0x1380,0xd341,0x1100,0xd1c1,0xd081,0x1040,
    0xf001,0x30c0,0x3180,0xf141,0x3300,0xf3c1,0xf281,0x3240,
    0x3600,0xf6c1,0xf781,0x3740,0xf501,0x35c0,0x3480,0xf441,
    0x3c00,0xfcc1,0xfd81,0x3d40,0xff01,0x3fc0,0x3e80,0xfe41,
    0xfa01,0x3ac0,0x3b80,0xfb41,0x3900,0xf9c1,0xf881,0x3840,
    0x2800,0xe8c1,0xe981,0x2940,0xeb01,0x2bc0,0x2a80,0xea41,
    0xee01,0x2ec0,0x2f80,0xef41,0x2d00,0xedc1,0xec81,0x2c40,
    0xe401,0x24c0,0x2580,0xe541,0x2700,0xe7c1,0xe681,0x2640,
    0x2200,0xe2c1,0xe381,0x2340,0xe101,0x21c0,0x2080,0xe041,
    0xa001,0x60c0,0x6180,0xa141,0x6300,0xa3c1,0xa281,0x6240,
    0x6600,0xa6c1,0xa781,0x6740,0xa501,0x65c0,0x6480,0xa441,
    0x6c00,0xacc1,0xad81,0x6d40,0xaf01,0x6fc0,0x6e80,0xae41,
    0xaa01,0x6ac0,0x6b80,0xab41,0x6900,0xa9c1,0xa881,0x6840,
    0x7800,0xb8c1,0xb981,0x7940,0xbb01,0x7bc0,0x7a80,0xba41,
    0xbe01,0x7ec0,0x7f80,0xbf41,0x7d00,0xbdc1,0xbc81,0x7c40,
    0xb401,0x74c0,0x7580,0xb541,0x7700,0xb7c1,0xb681,0x7640,
    0x7200,0xb2c1,0xb381,0x7340,0xb101,0x71c0,0x7080,0xb041,
    0x5000,0x90c1,0x9181,0x5140,0x9301,0x53c0,0x5280,0x9241,
    0x9601,0x56c0,0x5780,0x9741,0x5500,0x95c1,0x9481,0x5440,
    0x9c01,0x5cc0,0x5d80,0x9d41,0x5f00,0x9fc1,0x9e81,0x5e40,
    0x5a00,0x9ac1,0x9b81,0x5b40,0x9901,0x59c0,0x5880,0x9841,
    0x8801,0x48c0,0x4980,0x8941,0x4b00,0x8bc1,0x8a81,0x4a40,
    0x4e00,0x8ec1,0x8f81,0x4f40,0x8d01,0x4dc0,0x4c80,0x8c41,
    0x4400,0x84c1,0x8581,0x4540,0x8701,0x47c0,0x4680,0x8641,
    0x8201,0x42c0,0x4380,0x8341,0x4100,0x81c1,0x8081,0x4040
};
uint16_t dji_crc16(const uint8_t* d, size_t l) {
    uint16_t crc = 0x3AA3;
    while (l--) { uint8_t i=(crc^*d++)&0xFF; crc=(crc16_table[i]^(crc>>8))&0xFFFF; }
    return crc;
}

// ═══════════════════════════════════════════════════════════
// DJI CRC32 — Fletcher variant, init=0x00003AA3
// ═══════════════════════════════════════════════════════════
static const uint32_t crc32_table[256] = {
    0x00000000,0x77073096,0xee0e612c,0x990951ba,0x076dc419,0x706af48f,0xe963a535,0x9e6495a3,
    0x0edb8832,0x79dcb8a4,0xe0d5e91e,0x97d2d988,0x09b64c2b,0x7eb17cbd,0xe7b82d07,0x90bf1d91,
    0x1db71064,0x6ab020f2,0xf3b97148,0x84be41de,0x1adad47d,0x6ddde4eb,0xf4d4b551,0x83d385c7,
    0x136c9856,0x646ba8c0,0xfd62f97a,0x8a65c9ec,0x14015c4f,0x63066cd9,0xfa0f3d63,0x8d080df5,
    0x3b6e20c8,0x4c69105e,0xd56041e4,0xa2677172,0x3c03e4d1,0x4b04d447,0xd20d85fd,0xa50ab56b,
    0x35b5a8fa,0x42b2986c,0xdbbbc9d6,0xacbcf940,0x32d86ce3,0x45df5c75,0xdcd60dcf,0xabd13d59,
    0x26d930ac,0x51de003a,0xc8d75180,0xbfd06116,0x21b4f4b5,0x56b3c423,0xcfba9599,0xb8bda50f,
    0x2802b89e,0x5f058808,0xc60cd9b2,0xb10be924,0x2f6f7c87,0x58684c11,0xc1611dab,0xb6662d3d,
    0x76dc4190,0x01db7106,0x98d220bc,0xefd5102a,0x71b18589,0x06b6b51f,0x9fbfe4a5,0xe8b8d433,
    0x7807c9a2,0x0f00f934,0x9609a88e,0xe10e9818,0x7f6a0dbb,0x086d3d2d,0x91646c97,0xe6635c01,
    0x6b6b51f4,0x1c6c6162,0x856530d8,0xf262004e,0x6c0695ed,0x1b01a57b,0x8208f4c1,0xf50fc457,
    0x65b0d9c6,0x12b7e950,0x8bbeb8ea,0xfcb9887c,0x62dd1ddf,0x15da2d49,0x8cd37cf3,0xfbd44c65,
    0x4db26158,0x3ab551ce,0xa3bc0074,0xd4bb30e2,0x4adfa541,0x3dd895d7,0xa4d1c46d,0xd3d6f4fb,
    0x4369e96a,0x346ed9fc,0xad678846,0xda60b8d0,0x44042d73,0x33031de5,0xaa0a4c5f,0xdd0d7cc9,
    0x5005713c,0x270241aa,0xbe0b1010,0xc90c2086,0x5768b525,0x206f85b3,0xb966d409,0xce61e49f,
    0x5edef90e,0x29d9c998,0xb0d09822,0xc7d7a8b4,0x59b33d17,0x2eb40d81,0xb7bd5c3b,0xc0ba6cad,
    0xedb88320,0x9abfb3b6,0x03b6e20c,0x74b1d29a,0xead54739,0x9dd277af,0x04db2615,0x73dc1683,
    0xe3630b12,0x94643b84,0x0d6d6a3e,0x7a6a5aa8,0xe40ecf0b,0x9309ff9d,0x0a00ae27,0x7d079eb1,
    0xf00f9344,0x8708a3d2,0x1e01f268,0x6906c2fe,0xf762575d,0x806567cb,0x196c3671,0x6e6b06e7,
    0xfed41b76,0x89d32be0,0x10da7a5a,0x67dd4acc,0xf9b9df6f,0x8ebeeff9,0x17b7be43,0x60b08ed5,
    0xd6d6a3e8,0xa1d1937e,0x38d8c2c4,0x4fdff252,0xd1bb67f1,0xa6bc5767,0x3fb506dd,0x48b2364b,
    0xd80d2bda,0xaf0a1b4c,0x36034af6,0x41047a60,0xdf60efc3,0xa867df55,0x316e8eef,0x4669be79,
    0xcb61b38c,0xbc66831a,0x256fd2a0,0x5268e236,0xcc0c7795,0xbb0b4703,0x220216b9,0x5505262f,
    0xc5ba3bbe,0xb2bd0b28,0x2bb45a92,0x5cb36a04,0xc2d7ffa7,0xb5d0cf31,0x2cd99e8b,0x5bdeae1d,
    0x9b64c2b0,0xec63f226,0x756aa39c,0x026d930a,0x9c0906a9,0xeb0e363f,0x72076785,0x05005713,
    0x95bf4a82,0xe2b87a14,0x7bb12bae,0x0cb61b38,0x92d28e9b,0xe5d5be0d,0x7cdcefb7,0x0bdbdf21,
    0x86d3d2d4,0xf1d4e242,0x68ddb3f8,0x1fda836e,0x81be16cd,0xf6b9265b,0x6fb077e1,0x18b74777,
    0x88085ae6,0xff0f6a70,0x66063bca,0x11010b5c,0x8f659eff,0xf862ae69,0x616bffd3,0x166ccf45,
    0xa00ae278,0xd70dd2ee,0x4e048354,0x3903b3c2,0xa7672661,0xd06016f7,0x4969474d,0x3e6e77db,
    0xaed16a4a,0xd9d65adc,0x40df0b66,0x37d83bf0,0xa9bcae53,0xdebb9ec5,0x47b2cf7f,0x30b5ffe9,
    0xbdbdf21c,0xcabac28a,0x53b39330,0x24b4a3a6,0xbad03605,0xcdd70693,0x54de5729,0x23d967bf,
    0xb3667a2e,0xc4614ab8,0x5d681b02,0x2a6f2b94,0xb40bbe37,0xc30c8ea1,0x5a05df1b,0x2d02ef8d
};
uint32_t dji_crc32(const uint8_t* d, size_t l) {
    uint32_t crc = 0x00003AA3;
    while (l--) { uint8_t i=(crc^*d++)&0xFF; crc=(crc32_table[i]^(crc>>8))&0xFFFFFFFF; }
    return crc;
}

// CmdType byte (DJI Osmo-GPS-Controller-Demo, enums_logic.h):
// bits 4..0 = response policy, bit 5 = 1 for a response frame.
// The camera answers a command with the same SEQ.
#define DJI_CMD_NO_RESPONSE   0x00
#define DJI_CMD_RESPONSE_OPT  0x01   // response requested, absence is not an error
#define DJI_CMD_WAIT_RESULT   0x02   // response mandatory
#define DJI_FRAME_IS_RESPONSE 0x20

size_t dji_build_frame(uint8_t* buf, uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type,
                       const uint8_t* data, size_t data_len, uint16_t seq) {
    size_t total=14+data_len+4, off=0;
    buf[off++]=0xAA; buf[off++]=total&0xFF; buf[off++]=(total>>8)&0xFF;
    buf[off++]=cmd_type; buf[off++]=0x00;
    buf[off++]=0x00; buf[off++]=0x00; buf[off++]=0x00;
    buf[off++]=seq&0xFF; buf[off++]=(seq>>8)&0xFF;
    uint16_t c16=dji_crc16(buf,off);
    buf[off++]=c16&0xFF; buf[off++]=(c16>>8)&0xFF;
    buf[off++]=cmd_set; buf[off++]=cmd_id;
    if(data&&data_len>0){memcpy(buf+off,data,data_len);off+=data_len;}
    uint32_t c32=dji_crc32(buf,off);
    buf[off++]=c32&0xFF; buf[off++]=(c32>>8)&0xFF;
    buf[off++]=(c32>>16)&0xFF; buf[off++]=(c32>>24)&0xFF;
    return off;
}

// Camera state from the battery heartbeat (see CAM_HEARTBEAT_MS). A link with no
// push for CAM_HEARTBEAT_MS since it came up = connected to an already sleeping camera.
static bool camHeartbeatSeen() { return camHeartbeatMs != 0; }
static bool camAwake()  { uint32_t t=camHeartbeatMs; return t && millis()-t < CAM_HEARTBEAT_MS; }
static bool camAsleep() {
    uint32_t t=camHeartbeatMs;
    if (t) return millis()-t >= CAM_HEARTBEAT_MS;
    return connected && millis()-camLinkMs >= CAM_HEARTBEAT_MS;
}

// ═══════════════════════════════════════════════════════════
// GPS fix freshness
// TinyGPSPlus location.isValid() stays true forever after the first fix
// (location is committed only from sentences with a fix), so without an age
// check a lost signal keeps returning the last position. Receiver runs at 1 Hz.
// ═══════════════════════════════════════════════════════════
#define GPS_FIX_MAX_AGE_MS 3000UL
static bool gpsFixLive() {
    return instance.gps.location.isValid() && instance.gps.location.age() < GPS_FIX_MAX_AGE_MS;
}

// TinyGPSPlus also marks an empty/zero date as valid (it produced log_2000_00_00.txt
// and track_2000_00_00_0000.gpx), so check the values too
static bool gpsDateOk() {
    return instance.gps.date.isValid() && instance.gps.date.year() >= 2024 &&
           instance.gps.date.month() >= 1 && instance.gps.date.day() >= 1;
}

// ═══════════════════════════════════════════════════════════
// Haptics — non-blocking: pulse sequences are played from loop(),
// so touch handlers never delay() (it stalled LVGL and NMEA parsing).
// A new sequence replaces one still pending.
// ═══════════════════════════════════════════════════════════
#define DOUBLE_TAP_MS 400
static uint8_t  hapticEffect = 0, hapticLeft = 0;
static uint16_t hapticGapMs  = 0;
static uint32_t hapticNextMs = 0;

static void hapticSeq(uint8_t effect, uint8_t count, uint16_t gapMs, uint16_t startDelayMs) {
    hapticEffect=effect; hapticLeft=count; hapticGapMs=gapMs;
    hapticNextMs=millis()+startDelayMs;
}
static void haptic(uint8_t effect) { hapticSeq(effect,1,0,0); }
static void serviceHaptics() {
    if (hapticLeft && (int32_t)(millis()-hapticNextMs)>=0) {
        instance.setHapticEffects(hapticEffect); instance.vibrator();
        hapticLeft--; hapticNextMs=millis()+hapticGapMs;
    }
}

// Long 750 ms buzz (DRV2605 effect 15) = "action not done"
static void hapticError() { haptic(15); }

// ═══════════════════════════════════════════════════════════
// Display
// ═══════════════════════════════════════════════════════════
void wakeDisplay() {
    if (!displayOn) {
        displayOn = true;
        updateUI();                 // UI is not refreshed while dark — show fresh data at once
        lastUiUpdate = millis();
        instance.setBrightness(200);
    }
    lastActivityMs = millis();
}

// ═══════════════════════════════════════════════════════════
// SD Event Log
// Lines from other tasks are queued and written by loop(), so SD and the
// GPS timestamp are touched from one thread only.
// ═══════════════════════════════════════════════════════════
#define LOG_MSG_LEN 128
static TaskHandle_t  mainTask = nullptr;
static QueueHandle_t logQueue = nullptr;

void logWrite(const char* msg) {
    if (xTaskGetCurrentTaskHandle() != mainTask) {
        char item[LOG_MSG_LEN];
        strncpy(item, msg, sizeof(item)-1); item[sizeof(item)-1] = '\0';
        if (!logQueue || xQueueSend(logQueue, item, 0) != pdTRUE) Serial.println(msg);
        return;
    }
    Serial.println(msg);
    if (!sdReady) return;
    if (gpsDateOk() && instance.gps.date.day() != lastLogDay) {
        lastLogDay = instance.gps.date.day();
        snprintf(logPath, sizeof(logPath), "/log_%04d_%02d_%02d.txt",
            instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day());
    }
    if (logPath[0] == '\0') snprintf(logPath, sizeof(logPath), "/log_nodate.txt");
    File f = SD.open(logPath, FILE_APPEND);
    if (!f) return;
    if (instance.gps.time.isValid())
        f.printf("[%02d:%02d:%02d] %s\n",
            instance.gps.time.hour(), instance.gps.time.minute(),
            instance.gps.time.second(), msg);
    else f.printf("[%08lu] %s\n", millis(), msg);
    f.flush();
    f.close();
}
void logWritef(const char* fmt, ...) {
    char buf[LOG_MSG_LEN]; va_list a; va_start(a,fmt);
    vsnprintf(buf,sizeof(buf),fmt,a); va_end(a);
    logWrite(buf);
}
static void drainLogQueue() {
    char item[LOG_MSG_LEN];
    while (logQueue && xQueueReceive(logQueue, item, 0) == pdTRUE) logWrite(item);
}

// ═══════════════════════════════════════════════════════════
// BLE dump (step 24 research): every camera notification on FFF4, every
// command we send (except routine GPS pushes) and user MARKs go to
// /ble_dump.txt, decoded where the format is known. Same pattern as the log:
// any task queues, loop() writes. Set DEBUG_BLE_DUMP to 0 for a normal build.
// ═══════════════════════════════════════════════════════════
#if DEBUG_BLE_DUMP
#define DUMP_PATH    "/ble_dump.txt"
static QueueHandle_t dumpQueue = nullptr;

static void dumpFrame(uint8_t dir, const uint8_t* data, size_t len) {
    if (!dumpQueue) return;
    DumpItem it; it.ms=millis(); it.dir=dir; it.len=(uint16_t)len;
    memcpy(it.data, data, len<DUMP_MAX_LEN?len:DUMP_MAX_LEN);
    xQueueSend(dumpQueue, &it, 0);
}
static void dumpNote(const char* text) {
    dumpFrame(DUMP_NOTE, (const uint8_t*)text, strnlen(text, DUMP_MAX_LEN-1)+1);
}

// Status push 1D02 payload offsets (camera_status_push_command_frame, packed, 38 bytes)
static bool dumpIsStatus(const DumpItem& it, const uint8_t** p, int* plen) {
    if (it.dir!=DUMP_RX || it.len<18 || it.len>DUMP_MAX_LEN || it.data[0]!=0xAA) return false;
    if ((it.data[3]&DJI_FRAME_IS_RESPONSE) || it.data[12]!=0x1D || it.data[13]!=0x02) return false;
    *p=it.data+14; *plen=(int)it.len-18;
    return true;
}

static bool dumpIsBattery(const DumpItem& it) {
    return it.dir==DUMP_RX && it.len>=33 && it.len<=DUMP_MAX_LEN && it.data[0]==0x55 &&
           it.data[9]==0x0D && it.data[10]==0x02;
}

static void dumpDecode(File& f, const DumpItem& it) {
    const uint8_t* d=it.data;
    if (it.len>=13 && it.len<=DUMP_MAX_LEN && d[0]==0x55) {
        bool crcOk = duml_crc16(d,it.len-2)==(uint16_t)(d[it.len-2]|(d[it.len-1]<<8));
        f.printf("    DUML %02X%02X seq=%u crc=%s", d[9], d[10], d[6]|(d[7]<<8), crcOk?"ok":"BAD");
        if (dumpIsBattery(it))
            f.printf(" BATTERY %u%% %ld mV %ld mA", d[31],
                (long)(int32_t)(d[12]|(d[13]<<8)|(d[14]<<16)|((uint32_t)d[15]<<24)),
                (long)(int32_t)(d[16]|(d[17]<<8)|(d[18]<<16)|((uint32_t)d[19]<<24)));
        f.print("\n");
        return;
    }
    if (it.len<18 || it.len>DUMP_MAX_LEN || d[0]!=0xAA) { f.print("    (not a full DJI frame)\n"); return; }
    uint16_t flen=(d[1]|(d[2]<<8))&0x03FF;
    uint8_t  type=d[3], set=d[12], id=d[13];
    uint16_t seq=d[8]|(d[9]<<8);
    const uint8_t* p=d+14; int plen=(int)flen-18;
    bool crcOk = flen==it.len &&
        dji_crc16(d,10)==(uint16_t)(d[10]|(d[11]<<8)) &&
        dji_crc32(d,flen-4)==((uint32_t)d[flen-4]|((uint32_t)d[flen-3]<<8)|
                              ((uint32_t)d[flen-2]<<16)|((uint32_t)d[flen-1]<<24));
    f.printf("    %s %02X%02X seq=%u type=0x%02X len=%u crc=%s",
        (type&DJI_FRAME_IS_RESPONSE)?"RESP":"CMD ", set, id, seq, type, flen, crcOk?"ok":"BAD");
    if (type&DJI_FRAME_IS_RESPONSE) {
        if (plen>0) f.printf(" ret=%u", p[0]);
    } else if (set==0x1D && id==0x02 && plen>=38) {
        f.printf(" STATUS mode=0x%02X status=0x%02X rec_t=%us user=%u power=%u next=0x%02X temp=%u loop=%u bat=%u%%",
            p[0], p[1], p[5]|(p[6]<<8), p[27], p[28], p[29], p[30], p[35]|(p[36]<<8), p[37]);
    } else if (set==0x1D && id==0x06 && plen>=2 && p[0]==0x01) {
        int n=p[1]<plen-2?p[1]:plen-2;
        f.print(" MODE \""); f.write(p+2,n); f.print("\"");
    }
    f.print("\n");
}

static void drainDump() {
    if (!dumpQueue || uxQueueMessagesWaiting(dumpQueue)==0) return;
    static uint8_t  lastStatus[DUMP_MAX_LEN];
    static int      lastStatusLen = -1;
    static uint32_t sameStatus = 0;
    File f;
    if (sdReady) f = SD.open(DUMP_PATH, FILE_APPEND);
    DumpItem it;
    while (xQueueReceive(dumpQueue, &it, 0) == pdTRUE) {
        if (!f) continue;
        const uint8_t* p; int plen;
        bool status = dumpIsStatus(it, &p, &plen);
        if (status && plen==lastStatusLen && memcmp(p,lastStatus,plen)==0) { sameStatus++; continue; }
        if (sameStatus) { f.printf("    (+%lu identical status pushes)\n", (unsigned long)sameStatus); sameStatus=0; }
        // Battery heartbeat (1 Hz): keep only changes of % and gaps > 3 s
        static uint8_t  lastPct = 0xFF;
        static uint32_t lastBattMs = 0, skippedBatt = 0;
        if (dumpIsBattery(it)) {
            bool keep = it.data[31]!=lastPct || it.ms-lastBattMs>3000;
            lastPct=it.data[31]; lastBattMs=it.ms;
            if (!keep) { skippedBatt++; continue; }
        }
        if (skippedBatt) { f.printf("    (+%lu battery pushes)\n", (unsigned long)skippedBatt); skippedBatt=0; }
        if (status && plen>0 && plen<=DUMP_MAX_LEN) { memcpy(lastStatus,p,plen); lastStatusLen=plen; }

        char ts[12];
        if (instance.gps.time.isValid())
            snprintf(ts,sizeof(ts),"%02d:%02d:%02d",instance.gps.time.hour(),instance.gps.time.minute(),instance.gps.time.second());
        else snprintf(ts,sizeof(ts),"--:--:--");
        if (it.dir==DUMP_NOTE) {
            it.data[DUMP_MAX_LEN-1]='\0';
            f.printf("[%s %lu] ===== %s =====\n", ts, (unsigned long)it.ms, (const char*)it.data);
            continue;
        }
        static const char hx[]="0123456789ABCDEF";
        char hex[2*DUMP_MAX_LEN+1]; int n=it.len<DUMP_MAX_LEN?it.len:DUMP_MAX_LEN;
        for (int i=0;i<n;i++){ hex[2*i]=hx[it.data[i]>>4]; hex[2*i+1]=hx[it.data[i]&15]; }
        hex[2*n]='\0';
        f.printf("[%s %lu] %s %u%s: %s\n", ts, (unsigned long)it.ms, it.dir==DUMP_RX?"RX":"TX",
            it.len, it.len>DUMP_MAX_LEN?" (truncated)":"", hex);
        dumpDecode(f, it);
    }
    if (f) { f.flush(); f.close(); }
}
#else
static inline void dumpFrame(uint8_t, const uint8_t*, size_t) {}
static inline void dumpNote(const char*) {}
#endif

// ═══════════════════════════════════════════════════════════
// DJI Command: Start/Stop Recording
// CMD_SET=0x1D, CMD_ID=0x03
// ═══════════════════════════════════════════════════════════
bool sendRecordCommand(bool start, uint16_t* seqOut) {
    if (!pWriteChr || !connected) return false;
    uint8_t payload[9];
    payload[0]=DEVICE_ID&0xFF; payload[1]=(DEVICE_ID>>8)&0xFF;
    payload[2]=(DEVICE_ID>>16)&0xFF; payload[3]=(DEVICE_ID>>24)&0xFF;
    payload[4]=start?0x00:0x01;
    payload[5]=payload[6]=payload[7]=payload[8]=0x00;
    uint8_t frame[64];
    // Response requested (as in DJI's demo) — the reply on FFF4 carries ret_code
    *seqOut=++seqNum;
    size_t len=dji_build_frame(frame,0x1D,0x03,DJI_CMD_RESPONSE_OPT,payload,sizeof(payload),*seqOut);
    bool ok=pWriteChr->writeValue(frame,len,false);
    dumpFrame(DUMP_TX,frame,len);
    if (!ok) logWritef("REC %s: write FAIL",start?"START":"STOP");
    return ok;
}

// bleTask: send REC and wait for the camera's reply with the same SEQ.
// Measured: START from idle ~1 s, STOP or START from pre-rec ~40 ms;
// a sleeping camera does not answer. Returns ret_code (0 = done) or -1.
static int recCommandWithReply(bool start, uint32_t timeoutMs) {
    recRespNew=false;
    uint16_t seq;
    if (!sendRecordCommand(start,&seq)) return -1;
    uint32_t t0=millis();
    while (millis()-t0<timeoutMs && connected) {
        if (recRespNew && recRespSeq==seq) {
            int ret=recRespRet;
            logWritef("REC %s: ret=%d (%lu ms)",start?"START":"STOP",ret,(unsigned long)(millis()-t0));
            return ret;
        }
        sendGpsPayload();   // keep the 1 Hz overlay going while waiting
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
    logWritef("REC %s: no reply",start?"START":"STOP");
    return -1;
}

// ═══════════════════════════════════════════════════════════
// DJI Command: GPS Injection
// CMD_SET=0x00, CMD_ID=0x17, sent at 1Hz
// buildGpsPayload() runs in loop() (owns instance.gps), sendGpsPayload() in bleTask.
//
// DJI demo layout (gps_logic.c / gps_data_push_command_frame), 48 bytes:
//   int32 date YYYYMMDD | int32 time (hour+8)*10000+MMSS — UTC+8, hour NOT wrapped, as
//   in the demo | int32 lon*1e7 | int32 lat*1e7 | int32 height mm | float v_north,
//   v_east, v_down cm/s | uint32 v_acc mm, h_acc mm, speed_acc cm/s | uint32 satellites
// Legacy layout (up to v20), 45 bytes: UTC time, accuracies as floats (2.0f reads as
// 1073741824 mm), satellites as one byte. The overlay in Mimo worked with it too.
// ═══════════════════════════════════════════════════════════
void buildGpsPayload() {
    if (!gpsFixLive()||!gpsDateOk()||!instance.gps.time.isValid()) return;
    if (instance.gps.satellites.value()==0) return;
    int32_t ymd=instance.gps.date.year()*10000+instance.gps.date.month()*100+instance.gps.date.day();
    int32_t mmss=instance.gps.time.minute()*100+instance.gps.time.second();
    int32_t hmsUtc=instance.gps.time.hour()*10000+mmss;
    int32_t hmsDji=(instance.gps.time.hour()+8)*10000+mmss;
    int32_t lon=(int32_t)(instance.gps.location.lng()*1e7);
    int32_t lat=(int32_t)(instance.gps.location.lat()*1e7);
    int32_t alt=instance.gps.altitude.isValid()?(int32_t)(instance.gps.altitude.meters()*1000.0f):0;
    float spd=instance.gps.speed.isValid()?(float)instance.gps.speed.mps():0.0f;
    if(spd<0.5f) spd=0.0f;
    float crs=instance.gps.course.isValid()?(float)(instance.gps.course.deg()*3.14159265f/180.0f):0.0f;
    float sn=spd*cosf(crs)*100.0f, se=spd*sinf(crs)*100.0f, sd=0.0f;
    uint32_t sats=instance.gps.satellites.value();

    uint8_t dji[sizeof(gpsPayload)], *p=dji;
    auto put=[&p](const void* v){ memcpy(p,v,4); p+=4; };
    const uint32_t vAcc=1000, hAcc=1000, sAcc=10;   // demo defaults: 1 m, 1 m, 10 cm/s
    put(&ymd); put(&hmsDji); put(&lon); put(&lat); put(&alt);
    put(&sn); put(&se); put(&sd); put(&vAcc); put(&hAcc); put(&sAcc); put(&sats);

    uint8_t old[sizeof(gpsPayloadLegacy)];
    p=old;
    const float va=2.0f, ha=2.0f, sa=0.1f;
    put(&ymd); put(&hmsUtc); put(&lon); put(&lat); put(&alt);
    put(&sn); put(&se); put(&sd); put(&va); put(&ha); put(&sa);
    *p=(uint8_t)sats;

    portENTER_CRITICAL(&gpsMux);
    memcpy(gpsPayload,dji,sizeof(gpsPayload));
    memcpy(gpsPayloadLegacy,old,sizeof(gpsPayloadLegacy));
    gpsPayloadReady=true;
    portEXIT_CRITICAL(&gpsMux);
}

// bleTask only: SEQs of the last reply-requesting push of each layout, and how many
// replies of this link to log in full (later ones only when ret != 0)
static uint16_t gpsProbeSeqDji = 0, gpsProbeSeqLegacy = 0;
static uint8_t  gpsVerboseReplies = 0;

static void sendGpsFrame(const uint8_t* pl, size_t n, bool reply, uint16_t* seqOut) {
    uint8_t frame[80];
    uint16_t seq=++seqNum;
    size_t len=dji_build_frame(frame,0x00,0x17,reply?DJI_CMD_RESPONSE_OPT:DJI_CMD_NO_RESPONSE,pl,n,seq);
    bool ok=pWriteChr->writeValue(frame,len,false);
    if (reply) { dumpFrame(DUMP_TX,frame,len); *seqOut=seq; }
    if (!ok) logWrite("GPS inject FAIL");
}

void sendGpsPayload() {
    uint8_t dji[sizeof(gpsPayload)], old[sizeof(gpsPayloadLegacy)];
    portENTER_CRITICAL(&gpsMux);
    bool ready=gpsPayloadReady;
    if (ready) {
        memcpy(dji,gpsPayload,sizeof(dji));
        memcpy(old,gpsPayloadLegacy,sizeof(old));
        gpsPayloadReady=false;
    }
    portEXIT_CRITICAL(&gpsMux);
    if (!ready || !pWriteChr || !connected) return;
    // A sleeping camera queues writes and replays them on wake-up — don't feed it
    // (only when its heartbeat is known: never cut GPS on a camera that doesn't push it)
    if (camHeartbeatSeen() && camAsleep()) return;
    const bool legacy = GPS_FRAME_LEGACY;
    static uint32_t count=0;
    bool probe = (++count%60==0);   // the layout in use asks for a reply now and then
    if (gpsProbePending) {
        // Once per link: the other layout first, then the one in use, both with a reply,
        // so the camera ends up with the layout in use
        gpsProbePending=false;
        probe=true;
        if (legacy) sendGpsFrame(dji,sizeof(dji),true,&gpsProbeSeqDji);
        else        sendGpsFrame(old,sizeof(old),true,&gpsProbeSeqLegacy);
    }
    if (legacy) sendGpsFrame(old,sizeof(old),probe,&gpsProbeSeqLegacy);
    else        sendGpsFrame(dji,sizeof(dji),probe,&gpsProbeSeqDji);
}

// bleTask: log the camera's replies to GPS pushes (ret 0 = accepted)
static void serviceGpsReplies() {
    if (!gpsReplyNew) return;
    gpsReplyNew=false;
    uint16_t seq=gpsReplySeq;
    uint8_t  ret=gpsReplyRet;
    if (gpsVerboseReplies==0 && ret==0) return;
    if (gpsVerboseReplies) gpsVerboseReplies--;
    logWritef("GPS reply (%s): ret=%u", seq==gpsProbeSeqDji?"DJI 48 B":seq==gpsProbeSeqLegacy?"legacy 45 B":"?", ret);
}

// ═══════════════════════════════════════════════════════════
// GPX File Management
//
// Confirmed working approach (2026-05-24, 15 track points):
//   gpxCreateFile  → FILE_APPEND: creates file + writes full header with closing tags
//   gpxWriteTrackPoint → "r+" + seek: overwrites closing tags, adds trkpt
//   gpxWriteWaypoint   → "r+" + seek: overwrites </gpx>, adds wpt
//
// IMPORTANT: FILE_WRITE ("w") truncates on esp32 core 3.x — "r+" does not.
// SD is already mounted by instance.begin() — use SD.exists("/") to verify.
// ═══════════════════════════════════════════════════════════
// One track and one waypoint file per day: /track_YYYY_MM_DD.gpx + /track_YYYY_MM_DD_wpt.gpx.
// Restarting the logger the same day continues the file with a new <trkseg>, so an
// accidental stop/start no longer splits the trip into many files.
static bool gpxDailyPaths(char* trk, size_t trkLen, char* wpt, size_t wptLen) {
    if (!gpsDateOk()) return false;
    int y=instance.gps.date.year(), m=instance.gps.date.month(), d=instance.gps.date.day();
    snprintf(trk, trkLen, "/track_%04d_%02d_%02d.gpx", y, m, d);
    snprintf(wpt, wptLen, "/track_%04d_%02d_%02d_wpt.gpx", y, m, d);
    return true;
}

// Waypoint file: header + </gpx> on first use; waypoints are inserted before </gpx>
static bool gpxEnsureWptFile(const char* path) {
    File fw = SD.open(path, FILE_APPEND);
    if (!fw) return false;
    if (fw.size()==0) {
        fw.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fw.print("<gpx version=\"1.1\" creator=\"TWatch-DJI\"\n");
        fw.print("  xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
        fw.print(GPX_CLOSE);
        fw.flush();
    }
    fw.close();
    return true;
}

// Re-open an existing track for a new session: it must still end with the closing
// tags (not cut by a power loss mid-write). Starts a new <trkseg> unless the last one
// is still empty. false = file not usable, the caller starts a new one.
#define TRKSEG_OPEN     "    <trkseg>\n"
#define TRKSEG_OPEN_LEN (sizeof(TRKSEG_OPEN)-1)
static bool gpxContinueFile(const char* path) {
    File f = SD.open(path, "r+");
    if (!f) return false;
    uint32_t size = f.size();
    char tail[TRKSEG_OPEN_LEN+FULL_CLOSE_LEN];
    bool ok = size >= sizeof(tail) && f.seek(size-sizeof(tail)) &&
              f.read((uint8_t*)tail, sizeof(tail)) == sizeof(tail) &&
              memcmp(tail+TRKSEG_OPEN_LEN, TRKSEG_CLOSE GPX_CLOSE, FULL_CLOSE_LEN) == 0;
    if (ok && memcmp(tail, TRKSEG_OPEN, TRKSEG_OPEN_LEN) != 0) {
        f.seek(size-FULL_CLOSE_LEN);
        f.print("    </trkseg>\n" TRKSEG_OPEN TRKSEG_CLOSE GPX_CLOSE);
        f.flush();
    }
    f.close();
    return ok;
}

bool gpxCreateFile(const char* path) {
    File f = SD.open(path, FILE_APPEND);
    if (!f) { logWritef("GPX create FAIL: %s", path); return false; }
    f.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    f.print("<gpx version=\"1.1\" creator=\"TWatch-DJI\"\n");
    f.print("  xmlns=\"http://www.topografix.com/GPX/1/1\"\n");
    f.print("  xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
    f.print("  xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 ");
    f.print("http://www.topografix.com/GPX/1/1/gpx.xsd\">\n");
    char header[64];
    if (gpsDateOk())
        snprintf(header, sizeof(header), "  <trk>\n    <name>%04d-%02d-%02d</name>\n" TRKSEG_OPEN,
            instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day());
    else
        snprintf(header, sizeof(header), "  <trk>\n    <name>session</name>\n" TRKSEG_OPEN);
    f.print(header);
    f.print(TRKSEG_CLOSE);
    f.print(GPX_CLOSE);
    f.flush();
    f.close();
    logWritef("GPX created: %s", path);
    return true;
}

// Write track point — "r+" + seek: open without truncation, seek to closing tags position
// No live fix → no point (a gap is better than the last position with a new timestamp)
void gpxWriteTrackPoint() {
    if (!sdReady || gpxPath[0]=='\0' || !gpsFixLive()) return;
    File f = SD.open(gpxPath, "r+");
    if (!f) { logWrite("GPX trkpt open FAIL"); return; }
    uint32_t size = f.size();
    if (size < FULL_CLOSE_LEN) { f.close(); return; }
    f.seek(size - FULL_CLOSE_LEN);
    f.print("      <trkpt lat=\""); f.print(instance.gps.location.lat(),6);
    f.print("\" lon=\""); f.print(instance.gps.location.lng(),6); f.print("\">\n");
    f.print("        <ele>"); f.print(instance.gps.altitude.isValid()?instance.gps.altitude.meters():0.0,1); f.print("</ele>\n");
    char tbuf[48];
    snprintf(tbuf,sizeof(tbuf),"        <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day(),
        instance.gps.time.hour(),instance.gps.time.minute(),instance.gps.time.second());
    f.print(tbuf);
    f.print("      </trkpt>\n");
    f.print(TRKSEG_CLOSE);
    f.print(GPX_CLOSE);
    f.flush();
    f.close();
}

// Write waypoint — separate *_wpt.gpx file, seek before </gpx>
// NOTE: wpt cannot go in the main track file because the next trkpt write
// (seek from end - FULL_CLOSE_LEN) would overwrite the wpt data.
// Logger on → this session's waypoint file. Logger off → nothing, unless anyTime
// (BITE): then the day's waypoint file, so a mark is never lost.
// Returns false if nothing was written (no live fix, no date, SD error).
bool gpxWriteWaypoint(const char* name, bool anyTime) {
    if (!sdReady || !gpsFixLive()) return false;
    char path[sizeof(wptPath)];
    if (wptPath[0]) {
        strcpy(path, wptPath);
    } else {
        char trk[sizeof(gpxPath)];
        if (!anyTime || !gpxDailyPaths(trk,sizeof(trk),path,sizeof(path)) || !gpxEnsureWptFile(path)) return false;
    }
    File f = SD.open(path, "r+");
    if (!f) { logWritef("GPX wpt open FAIL: %s", name); return false; }
    uint32_t size = f.size();
    if (size < GPX_CLOSE_LEN) { f.close(); return false; }
    f.seek(size - GPX_CLOSE_LEN);
    f.print("  <wpt lat=\""); f.print(instance.gps.location.lat(),6);
    f.print("\" lon=\""); f.print(instance.gps.location.lng(),6); f.print("\">\n");
    f.print("    <ele>"); f.print(instance.gps.altitude.isValid()?instance.gps.altitude.meters():0.0,1); f.print("</ele>\n");
    char tbuf[48];
    snprintf(tbuf,sizeof(tbuf),"    <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day(),
        instance.gps.time.hour(),instance.gps.time.minute(),instance.gps.time.second());
    f.print(tbuf);
    f.print("    <name>"); f.print(name); f.print("</name>\n");
    f.print("  </wpt>\n");
    f.print(GPX_CLOSE);
    f.flush();
    f.close();
    logWritef("GPX wpt: %s", name);
    return true;
}

// ═══════════════════════════════════════════════════════════
// Recording toggle
// ═══════════════════════════════════════════════════════════
// Built-in Montserrat has only ASCII, °, • and the LV_SYMBOL_* icons —
// ● ■ ▶ ─ were drawn as placeholder boxes, so icons are used instead.
void renderRec() {
    if (isRecording) {
        lv_label_set_text(lblRec, LV_SYMBOL_VIDEO " REC");
        lv_obj_set_style_text_color(lblRec, lv_palette_main(LV_PALETTE_RED), 0);
    } else {
        lv_label_set_text(lblRec, LV_SYMBOL_VIDEO);
        lv_obj_set_style_text_color(lblRec, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    }
}

void renderLog() {
    if (gpxLogging) {
        lv_label_set_text(lblLog, LV_SYMBOL_PLAY " LOG: ON");
        lv_obj_set_style_text_color(lblLog, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else {
        lv_label_set_text(lblLog, LV_SYMBOL_STOP " LOG: OFF");
        lv_obj_set_style_text_color(lblLog, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    }
}

// ─── Logger resume after a crash ──────────────────────────
// RTC_NOINIT memory survives software resets (panic, watchdog, brownout) but
// not power-off: a crashed watch continues the same track, a watch that was
// switched off starts with the logger OFF as before.
#define LOG_RESUME_MAGIC 0x4C4F4731UL
static RTC_NOINIT_ATTR uint32_t rtcLogMagic;
static RTC_NOINIT_ATTR char     rtcGpxPath[sizeof(gpxPath)];
static RTC_NOINIT_ATTR char     rtcWptPath[sizeof(wptPath)];

static void logResumeSave() {
    if (gpxLogging) {
        memcpy(rtcGpxPath, gpxPath, sizeof(rtcGpxPath));
        memcpy(rtcWptPath, wptPath, sizeof(rtcWptPath));
        rtcLogMagic = LOG_RESUME_MAGIC;
    } else {
        rtcLogMagic = 0;
    }
}

static void logResumeTry() {
    esp_reset_reason_t rr = esp_reset_reason();
    bool crashed = rr==ESP_RST_PANIC || rr==ESP_RST_INT_WDT || rr==ESP_RST_TASK_WDT ||
                   rr==ESP_RST_WDT   || rr==ESP_RST_BROWNOUT;
    rtcGpxPath[sizeof(rtcGpxPath)-1] = '\0';
    rtcWptPath[sizeof(rtcWptPath)-1] = '\0';
    if (rtcLogMagic != LOG_RESUME_MAGIC || !crashed || !sdReady || !SD.exists(rtcGpxPath)) {
        rtcLogMagic = 0;
        return;
    }
    memcpy(gpxPath, rtcGpxPath, sizeof(gpxPath));   // keep appending to the same files
    memcpy(wptPath, rtcWptPath, sizeof(wptPath));
    gpxLogging = true;
    lastTrackMs = millis();
    renderLog();
    logWritef("Logger RESUMED after reset (reason=%d): %s", (int)rr, gpxPath);
}

// 1 buzz = started, 2 buzzes = stopped, long buzz = camera did not confirm.
// Soft bump right away = camera is asleep and is being woken up first.
// The BLE work happens in bleTask; its result comes back through recDone → onRecordResult().
void toggleRecording() {
    if (!connected) {
        // Delayed: if this was the 1st tap of a "connect" double tap, the 2nd tap replaces it
        hapticSeq(15,1,0,DOUBLE_TAP_MS);
        logWrite("REC ignored: camera not connected");
        return;
    }
    if (recCmd != 0 || recDone != 0) return;   // previous command still in flight
    if (!isRecording && camAsleep()) {
        haptic(7);
        lv_label_set_text(lblCamSat, "Cam: waking...");
    }
    recCmd = isRecording ? -1 : +1;
}

void onRecordResult(int8_t r) {
    if (r == REC_FAILED) { hapticError(); return; }   // state flips only on success
    isRecording = (r > 0);
    hapticSeq(14, isRecording?1:2, 200, 0);
    gpxWriteWaypoint(isRecording ? "REC START" : "REC STOP", false);   // only while logging
    renderRec();
}

// ═══════════════════════════════════════════════════════════
// Logger toggle
// ═══════════════════════════════════════════════════════════
void toggleLogger() {
    if (!gpxLogging && !sdReady) {
        hapticError();
        logWrite("Logger not started: no SD card");
        return;
    }
    gpxLogging = !gpxLogging;
    if (gpxLogging) {
        // Day file (continued with a new segment if it already exists); without a GPS
        // date yet — a separate session file
        bool daily = gpxDailyPaths(gpxPath, sizeof(gpxPath), wptPath, sizeof(wptPath));
        bool continued = daily && SD.exists(gpxPath) && gpxContinueFile(gpxPath);
        if (!continued) {
            if (!daily) {
                trackSession++;
                snprintf(gpxPath, sizeof(gpxPath), "/track_session_%03d.gpx", trackSession);
            }
            // Never reuse an existing file: FILE_APPEND would add a second header → invalid GPX
            // (damaged day file, or track_session_001 again after reboot)
            if (SD.exists(gpxPath)) {
                char base[32];
                strncpy(base, gpxPath, sizeof(base)-1); base[sizeof(base)-1] = '\0';
                char* ext = strrchr(base, '.');
                if (ext) *ext = '\0';
                for (int n=2; n<100; n++) {
                    snprintf(gpxPath, sizeof(gpxPath), "%s_%d.gpx", base, n);
                    if (!SD.exists(gpxPath)) break;
                }
            }
            if (!daily) {   // session file gets its own companion wpt file
                strncpy(wptPath, gpxPath, sizeof(wptPath)-1); wptPath[sizeof(wptPath)-1] = '\0';
                char* dot = strrchr(wptPath, '.');
                if (dot) strcpy(dot, "_wpt.gpx");
            }
            if (!gpxCreateFile(gpxPath)) {
                gpxLogging = false;
                gpxPath[0] = wptPath[0] = '\0';
                hapticError();
                return;
            }
        }
        // Companion wpt file (waypoints can't share the track file due to the seek writes)
        gpxEnsureWptFile(wptPath);
        haptic(14);
        gpxWriteTrackPoint();   // первая точка сразу, не ждать 60 сек
        lastTrackMs = millis();
        logResumeSave();
        renderLog();
        logWritef("Logger START: %s%s", gpxPath, continued?" (continued, new segment)":"");
    } else {
        haptic(14);
        wptPath[0] = '\0';
        logResumeSave();
        renderLog();
        logWrite("Logger STOP");
    }
}

// ═══════════════════════════════════════════════════════════
// Touch zone handler
// Upper (y < ZONE_SPLIT): tap=record, double tap=connect camera
//                         [debug build] long press=wake-up advertising
// Lower (y >= ZONE_SPLIT): hold 1.5 s=logger (see loop), double tap=BITE waypoint
//                         [debug build] every single tap writes a MARK to the dump
// ═══════════════════════════════════════════════════════════
void handleTouchEnd(int16_t x, int16_t y, uint32_t duration) {
    if (millis()-lastTouchMs < 80) return;   // 250 swallowed the 2nd tap of a fast double tap
    lastTouchMs = millis();

    if (y < ZONE_SPLIT) {
#if DEBUG_BLE_DUMP
        if (duration >= 600) {
            wakeReq = true;
            hapticSeq(7,3,100,0);
            logWrite("Long press: wake camera");
            return;
        }
#endif
        if (duration < 600) {
            static uint32_t lastTopTapMs = 0;
            static int topTapCount = 0;
            if (millis()-duration-lastTopTapMs < DOUBLE_TAP_MS) topTapCount++;   // gap: release → next press
            else topTapCount = 1;
            lastTopTapMs = millis();
            if (topTapCount >= 2) {
                topTapCount = 0;
                if (!connected) {
                    wantConnected = true;
                    triggerConnect = true;
                    hapticSeq(7,2,100,0);   // also cancels the 1st tap's pending error buzz
                    lv_label_set_text(lblCamSat, "Cam: connecting...");
                    logWrite("Double tap: connect camera");
                }
            } else {
                toggleRecording();
            }
        }
    } else {
        // Logger hold (LOGGER_HOLD_MS) is handled in loop() while the finger is down;
        // every shorter touch here is a tap, even a slow one (wet finger, glove)
        {
            static uint32_t lastBotTapMs = 0;
            static int botTapCount = 0;
            if (millis()-duration-lastBotTapMs < DOUBLE_TAP_MS) botTapCount++;   // gap: release → next press
            else botTapCount = 1;
            lastBotTapMs = millis();
            if (botTapCount >= 2) {
                botTapCount = 0;
                static int biteCount = 0;
                char name[16];
                snprintf(name, sizeof(name), "BITE %d", biteCount+1);
                if (gpxWriteWaypoint(name, true)) {   // logger off → day's waypoint file
                    biteCount++;
                    hapticSeq(14,3,150,0);
                    logWritef("Waypoint: %s %.5f,%.5f%s", name,
                        instance.gps.location.lat(), instance.gps.location.lng(),
                        gpxLogging?"":" (logger off, day file)");
                } else {
                    hapticError();
                    logWrite(!gpsFixLive() ? "BITE ignored: no GPS fix"
                           : !gpsDateOk()  ? "BITE ignored: no GPS date"
                                           : "BITE ignored: SD write error");
                }
            } else {
                haptic(1);
#if DEBUG_BLE_DUMP
                static int markCount = 0;
                char mark[16];
                snprintf(mark, sizeof(mark), "MARK %d", ++markCount);
                dumpNote(mark);
                logWrite(mark);
#endif
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════
// Signal quality bars using ASCII only (◆◇ U+25C6/25C7 not in LVGL Montserrat subset)
// ||||. = good,  ..... = no fix
const char* hdopBars() {
    if (instance.gps.hdop.isValid() && instance.gps.hdop.value() > 0) {
        float h = instance.gps.hdop.value() / 100.0f;
        if (h < 1.0f)  return "|||||";
        if (h < 2.0f)  return "||||.";
        if (h < 5.0f)  return "|||..";
        if (h < 10.0f) return "||...";
        return "|....";
    }
    // Fallback: satellite count as signal quality proxy
    if (!instance.gps.satellites.isValid()) return ".....";
    int s = (int)instance.gps.satellites.value();
    if (s >= 10) return "|||||";
    if (s >= 8)  return "||||.";
    if (s >= 6)  return "|||..";
    if (s >= 4)  return "||...";
    if (s >= 1)  return "|....";
    return ".....";
}

// ═══════════════════════════════════════════════════════════
// UI update
// ═══════════════════════════════════════════════════════════
void updateUI() {
    char buf[128];
    if (instance.gps.time.isValid())
        snprintf(buf,sizeof(buf),"%02d:%02d UTC",
            instance.gps.time.hour(),instance.gps.time.minute());
    else snprintf(buf,sizeof(buf),"--:--");
    lv_label_set_text(lblTime,buf);

    if (instance.gps.location.isValid()) {
        // white = live fix, orange = last known position (fix lost)
        lv_obj_set_style_text_color(lblCoords,
            gpsFixLive()?lv_color_white():lv_palette_main(LV_PALETTE_ORANGE),0);
        snprintf(buf,sizeof(buf),"%.5f\n%.5f",
            instance.gps.location.lat(),instance.gps.location.lng());
    } else {
        lv_obj_set_style_text_color(lblCoords,lv_palette_main(LV_PALETTE_YELLOW),0);
        snprintf(buf,sizeof(buf),"GPS\nconnecting...");
    }
    lv_label_set_text(lblCoords,buf);

    if (gpsFixLive()) {
        float kmh=instance.gps.speed.isValid()?(float)instance.gps.speed.kmph():0.0f;
        if(kmh<2.0f) kmh=0.0f;
        snprintf(buf,sizeof(buf),"%.1f km/h   %.0f m",
            kmh,instance.gps.altitude.isValid()?(float)instance.gps.altitude.meters():0.0f);
    } else snprintf(buf,sizeof(buf),"-- km/h   -- m");
    lv_label_set_text(lblSpeed,buf);

    // Cam:72% = awake (camera battery), zz = asleep / switched off, wake (yellow) = being
    // woken for REC, OK = linked but state unknown, -- = searching, ~~ (orange) = link lost
    bool lost = !connected && camLost;
    char cam[8];
    lv_color_t camColor = lv_palette_darken(LV_PALETTE_GREY,1);
    if (connected) {
        if (camWaking)         { strcpy(cam,"wake"); camColor=lv_palette_main(LV_PALETTE_YELLOW); }
        else if (camAwake())   snprintf(cam,sizeof(cam),"%u%%",(unsigned)camBattPct);
        else if (camAsleep())  { strcpy(cam,"zz"); camColor=lv_palette_main(LV_PALETTE_BLUE_GREY); }
        else                   strcpy(cam,"OK");   // just linked, first push not in yet
    } else {
        strcpy(cam, lost?"~~":"--");
        if (lost) camColor=lv_palette_main(LV_PALETTE_ORANGE);
    }
    snprintf(buf,sizeof(buf),"Cam:%s GPS:%s %d", cam, hdopBars(),
        instance.gps.satellites.isValid()?(int)instance.gps.satellites.value():0);
    lv_obj_set_style_text_color(lblCamSat,camColor,0);
    lv_label_set_text(lblCamSat,buf);

    snprintf(buf,sizeof(buf),"Bat: %d%%    SD: %s",
        instance.pmu.getBatteryPercent(),sdReady?(DEBUG_BLE_DUMP?"DUMP":"OK"):"--");
    lv_label_set_text(lblBatSd,buf);
}

// ═══════════════════════════════════════════════════════════
// BLE Callbacks
// ═══════════════════════════════════════════════════════════
// Runs in the NimBLE host task: flags only — bleTask logs and cleans up, loop() redraws
class CameraCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* c, int reason) override {
        disconnectReason=reason;
        connected=false;
    }
};
static CameraCallbacks camCallbacks;   // one instance, reused by every client (no leak per retry)

// The camera's own pushes use DJI's older DUML v1 framing (SOF 0x55): CRC-16
// init 0x3692, reflected poly 0x8408 over the whole frame minus the CRC.
static uint16_t duml_crc16(const uint8_t* d, size_t n) {
    uint16_t c=0x3692;
    while (n--) { c^=*d++; for (int i=0;i<8;i++) c=(c&1)?(c>>1)^0x8408:(c>>1); }
    return c;
}

// FFF4 notifications — NimBLE host task: validate, update flags, nothing else.
//  • DUML 0D/02 (47 bytes, 1 Hz while awake): camera battery; byte 31 = %   → heartbeat
//  • R SDK reply to REC (AA…, CmdType bit5): payload[0] = ret_code (0 = done)
static void onCamNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
    dumpFrame(DUMP_RX, d, len);
    if (len<13) return;
    size_t flen=(d[1]|(d[2]<<8))&0x03FF;
    if (flen!=len) return;
    if (d[0]==0x55) {
        if (duml_crc16(d,len-2)!=(uint16_t)(d[len-2]|(d[len-1]<<8))) return;
        if (d[9]==0x0D && d[10]==0x02 && len>=33) {
            camBattPct=d[31];
            camHeartbeatMs=millis()|1;   // never 0 (0 = "no push yet")
        }
    } else if (d[0]==0xAA && len>=19 && (d[3]&DJI_FRAME_IS_RESPONSE)) {
        uint32_t c32=(uint32_t)d[len-4]|((uint32_t)d[len-3]<<8)|((uint32_t)d[len-2]<<16)|((uint32_t)d[len-1]<<24);
        if (dji_crc16(d,10)!=(uint16_t)(d[10]|(d[11]<<8)) || dji_crc32(d,len-4)!=c32) return;
        if (d[12]==0x1D && d[13]==0x03) {
            recRespSeq=d[8]|(d[9]<<8);
            recRespRet=d[14];
            recRespNew=true;
        } else if (d[12]==0x00 && d[13]==0x17) {
            gpsReplySeq=d[8]|(d[9]<<8);
            gpsReplyRet=d[14];
            gpsReplyNew=true;
        }
    }
}

// Ask the camera to push its status (1D02): mode 3 = periodic 2 Hz + once on every change
static void subscribeCameraStatus() {
    const uint8_t sub[6] = {3, 20, 0, 0, 0, 0};   // push_mode, push_freq (0.1 Hz units, only 20 allowed), reserved
    uint8_t frame[32];
    size_t len=dji_build_frame(frame,0x1D,0x05,DJI_CMD_NO_RESPONSE,sub,sizeof(sub),++seqNum);
    bool ok=pWriteChr->writeValue(frame,len,false);
    dumpFrame(DUMP_TX,frame,len);
    logWritef("Status subscription: %s", ok?"sent":"FAIL");
}

// Wake-up as in DJI's demo (ble.c): advertise manufacturer data "WKP" + camera MAC
// in reverse byte order for 2 s. Runs in bleTask.
static void serviceWakeRequest() {
    if (!wakeReq) return;
    wakeReq = false;
    wakeCamera();
}
static void wakeCamera() {
    uint8_t md[9] = {'W','K','P'};
    for (int i=0;i<6;i++) md[3+5-i]=(uint8_t)strtoul(CAMERA_MAC+3*i,nullptr,16);
    NimBLEAdvertisementData ad;
    ad.setManufacturerData(md,sizeof(md));
    NimBLEAdvertising* adv=NimBLEDevice::getAdvertising();
    adv->stop();
    bool ok=adv->setAdvertisementData(ad) && adv->start(2000);
    dumpNote(ok?"WAKE advertising started (2 s)":"WAKE advertising FAILED");
    logWritef("Wake advertising: %s", ok?"started":"FAIL");
}

// bleTask: carry out a REC request from loop(). Returns cmd on success, REC_FAILED otherwise.
// Never write REC to a sleeping camera: it queues the writes and replays them all on
// wake-up (seen: START, STOP, START → not recording). Wake it, wait for the battery
// heartbeat, then send. Right after wake-up it may still refuse (ret 223) — retry.
static int8_t handleRecCommand(int8_t cmd) {
    bool start = cmd>0;
    bool woke = false;
    if (camAsleep()) {
        if (!start) { logWrite("REC STOP: camera asleep, not recording"); return cmd; }
        camWaking = true;
        uint32_t t0=millis();
        wakeCamera();
        while (!camAwake() && millis()-t0<12000 && connected) vTaskDelay(50/portTICK_PERIOD_MS);
        if (camAwake()) {
            logWritef("Wake: camera awake after %lu ms", (unsigned long)(millis()-t0));
            woke = true;
            vTaskDelay(800/portTICK_PERIOD_MS);
        } else if (camHeartbeatSeen()) {
            camWaking=false; logWrite("Wake: no heartbeat, giving up"); return REC_FAILED;
        } else {
            logWrite("Wake: this link never had a heartbeat - sending REC anyway");   // other firmware?
        }
    }
    int ret=-1;
    for (int attempt=0; attempt<(woke?4:2) && connected; attempt++) {
        ret=recCommandWithReply(start, 2500);
        if (ret==0) break;
        vTaskDelay(700/portTICK_PERIOD_MS);
    }
    camWaking = false;
    return ret==0 ? cmd : REC_FAILED;
}

// ═══════════════════════════════════════════════════════════
// BLE Task
// ═══════════════════════════════════════════════════════════
void bleTask(void* pvParameters) {
    vTaskDelay(2000/portTICK_PERIOD_MS);
    NimBLEDevice::init("TWatch-DJI");
    NimBLEDevice::setPower(3);
    NimBLEDevice::setMTU(512);
    NimBLEAddress addr(std::string(CAMERA_MAC),BLE_ADDR_PUBLIC);
    while (true) {
        while (!triggerConnect) { serviceWakeRequest(); vTaskDelay(200/portTICK_PERIOD_MS); }
        triggerConnect = false;
        pClient=NimBLEDevice::createClient();
        pClient->setClientCallbacks(&camCallbacks,false);
        pClient->setConnectTimeout(10000);
        logWrite("BLE connecting...");
        if (!pClient->connect(addr)) {
            logWrite("BLE connect FAIL");
            NimBLEDevice::deleteClient(pClient); pClient=nullptr;
            if (wantConnected) {
                vTaskDelay(5000/portTICK_PERIOD_MS);
                triggerConnect = true;  // auto-retry
            }
            continue;
        }
        auto svc=pClient->getService("0000fff0-0000-1000-8000-00805f9b34fb");
        pWriteChr=svc?svc->getCharacteristic("0000fff3-0000-1000-8000-00805f9b34fb"):nullptr;
        if (!pWriteChr) {
            logWrite(svc?"BLE FFF3 not found":"BLE FFF0 not found");
            pClient->disconnect();NimBLEDevice::deleteClient(pClient);pClient=nullptr;
            if (wantConnected) {
                vTaskDelay(5000/portTICK_PERIOD_MS);
                triggerConnect = true;  // auto-retry (was missing → reconnect stalled)
            }
            continue;
        }
        auto nc=svc->getCharacteristic("0000fff4-0000-1000-8000-00805f9b34fb");
        bool notifyOk = nc && nc->canNotify() && nc->subscribe(true, onCamNotify);
        portENTER_CRITICAL(&gpsMux); gpsPayloadReady=false; portEXIT_CRITICAL(&gpsMux);  // no stale fix
        gpsProbePending=true;   // first GPS push of this link asks about both layouts
        gpsVerboseReplies=2;
        camHeartbeatMs=0;   // camera state unknown until its first battery push
        camLinkMs=millis();
        camLost=false;
        connected=true;
        logWritef("BLE connected to camera (FFF4 notify: %s)", notifyOk?"on":"FAIL");
        dumpNote("BLE connected");
        vTaskDelay(300/portTICK_PERIOD_MS);
        subscribeCameraStatus();
        // REC requests are picked up within ~20 ms, GPS as loop() publishes it (1 Hz).
        // isConnected() also catches a disconnect that fired before connected=true.
        while (connected && pClient->isConnected()) {
            int8_t cmd=recCmd;
            if (cmd) { recDone=handleRecCommand(cmd); recCmd=0; }
            sendGpsPayload();
            serviceGpsReplies();
            serviceWakeRequest();
            vTaskDelay(20/portTICK_PERIOD_MS);
        }
        connected=false;
        if (recCmd) { recDone=REC_FAILED; recCmd=0; }   // command raced the disconnect
        camLost=true;
        logWritef("BLE lost, reason=%d. Reconnecting in 3s...",disconnectReason);
        dumpNote("BLE lost");
        pWriteChr=nullptr;
        vTaskDelay(200/portTICK_PERIOD_MS);   // let the host task finish the disconnect event
        NimBLEDevice::deleteClient(pClient); pClient=nullptr;
        if (wantConnected) {
            vTaskDelay(3000/portTICK_PERIOD_MS);
            triggerConnect = true;
        }
    }
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════
// GNSS constellation: GPS + Galileo + GLONASS (BeiDou OFF)
//
// The u-blox MIA-M10Q tracks a MAXIMUM of 4 GNSS at once (QZSS rides on GPS and
// does not count as a separate system). Factory default is GPS+Galileo+BeiDou
// (+QZSS), so enabling GLONASS REQUIRES freeing a slot — we disable BeiDou to
// make room. GPS and Galileo are left at their defaults (enabled).
//
// Why GLONASS instead of BeiDou here: at high / northern latitudes and in
// terrain (e.g. Kamchatka) GLONASS gives better satellite geometry, whereas
// BeiDou is optimised for Asia — so GLONASS yields a more stable fix for us.
//
// M10 uses the modern config interface UBX-CFG-VALSET (0x06 0x8A); the legacy
// UBX-CFG-GNSS (0x06 0x3E) is NOT supported on this chip generation.
// Config is written to RAM+BBR layers: the watch keeps GPS backup power
// (AXP2101 LDO1 / VRTC) permanently on, so the setting survives reboots. We also
// re-send it on every boot, so it is robust even if backup power is ever lost.
// LilyGoLib drives the GPS on the global Serial1 @ 38400 (see LilyGoLib
// initGPS()), so we can write UBX frames straight to Serial1.
// NOTE: changing the constellation set restarts the GNSS engine, so the first
// fix right after a config change is effectively a cold start (longer TTFF once).
// ═══════════════════════════════════════════════════════════
static void ubxSend(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[6] = {0xB5,0x62,cls,id,(uint8_t)(len&0xFF),(uint8_t)(len>>8)};
    uint8_t ckA=0, ckB=0;
    for (int i=2;i<6;i++){ ckA+=hdr[i]; ckB+=ckA; }             // 8-bit Fletcher over
    for (uint16_t i=0;i<len;i++){ ckA+=payload[i]; ckB+=ckA; }  // class..end of payload
    Serial1.write(hdr,6);
    if (len) Serial1.write(payload,len);
    Serial1.write(ckA); Serial1.write(ckB);
    Serial1.flush();
}

// Wait for a UBX-ACK-ACK (0x05 0x01) / ACK-NAK (0x05 0x00) matching the given
// message class/id. Scans the Serial1 byte stream (which also carries NMEA) for
// the UBX sync 0xB5 0x62 and verifies the checksum. Runs in setup() before the
// main loop, so there is no contention with instance.gps.loop().
// Returns: 1 = ACK, 0 = NAK, -1 = timeout.
static int ubxWaitAck(uint8_t forCls, uint8_t forId, uint32_t timeoutMs) {
    uint32_t start = millis();
    uint8_t  st = 0, mCls = 0, mId = 0, p0 = 0, p1 = 0, ckA = 0, ckB = 0, rxA = 0;
    uint16_t len = 0, idx = 0;
    while (millis() - start < timeoutMs) {
        if (!Serial1.available()) { delay(2); continue; }
        uint8_t c = Serial1.read();
        switch (st) {
        case 0: if (c == 0xB5) st = 1; break;
        case 1: st = (c == 0x62) ? 2 : (c == 0xB5 ? 1 : 0); break;
        case 2: mCls = c; ckA = c; ckB = c; st = 3; break;
        case 3: mId = c; ckA += c; ckB += ckA; st = 4; break;
        case 4: len = c; ckA += c; ckB += ckA; st = 5; break;
        case 5: len |= (uint16_t)c << 8; ckA += c; ckB += ckA; idx = 0; st = (len == 0) ? 7 : 6; break;
        case 6:
            if (idx == 0) p0 = c; else if (idx == 1) p1 = c;
            ckA += c; ckB += ckA;
            if (++idx >= len) st = 7;
            break;
        case 7: rxA = c; st = 8; break;                 // CK_A
        case 8:                                          // CK_B
            if (rxA == ckA && c == ckB && mCls == 0x05 &&
                (mId == 0x01 || mId == 0x00) && p0 == forCls && p1 == forId) {
                return (mId == 0x01) ? 1 : 0;
            }
            st = 0;   // not our ack (or bad checksum) → keep scanning
            break;
        }
    }
    return -1;
}

static void configGnssGlonass() {
    // UBX-CFG-VALSET: version=0, layers=RAM|BBR(0x03), reserved(2), key/value pairs.
    // Keys are 4-byte little-endian; value for an L(bool) item is 1 byte.
    //   CFG-SIGNAL-BDS_ENA (0x10310022) = 0  -> BeiDou OFF
    //   CFG-SIGNAL-GLO_ENA (0x10310025) = 1  -> GLONASS ON
    static const uint8_t payload[] = {
        0x00, 0x03, 0x00, 0x00,
        0x22, 0x00, 0x31, 0x10, 0x00,   // BDS_ENA = 0
        0x25, 0x00, 0x31, 0x10, 0x01,   // GLO_ENA = 1
    };
    delay(100);   // let the module settle after power-up before configuring
    ubxSend(0x06, 0x8A, payload, sizeof(payload));
    int ack = ubxWaitAck(0x06, 0x8A, 1500);   // CFG-VALSET is class 0x06, id 0x8A
    logWrite(ack == 1 ? "GNSS: BeiDou off, GLONASS on (ACK)"
           : ack == 0 ? "GNSS config REJECTED by module (NAK)"
                      : "GNSS config: no ACK (timeout)");
}

// ═══════════════════════════════════════════════════════════
// USB serial file access (read-only): the PC pulls logs and dumps without the
// SD card being taken out. One command per line:
//   ls          — files in / with sizes
//   cat <path>  — file contents between "<<<BEGIN path size>>>" and "<<<END>>>"
// Runs in loop(), which owns SD.
// ═══════════════════════════════════════════════════════════
static void serviceSerialCommands() {
    static char line[64];
    static uint8_t n = 0;
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (n < sizeof(line)-1) line[n++] = (char)c; continue; }
        line[n] = '\0'; n = 0;
        if (!sdReady) { Serial.println("<<<ERR no SD>>>"); continue; }
        if (strcmp(line, "ls") == 0) {
            File root = SD.open("/");
            Serial.println("<<<LS>>>");
            for (File f = root.openNextFile(); f; f = root.openNextFile()) {
                Serial.printf("%s%s %lu\n", f.name(), f.isDirectory()?"/":"", (unsigned long)f.size());
                f.close();
            }
            root.close();
            Serial.println("<<<END>>>");
        } else if (strncmp(line, "cat ", 4) == 0) {
            File f = SD.open(line+4, FILE_READ);
            if (!f || f.isDirectory()) { Serial.printf("<<<ERR cannot open %s>>>\n", line+4); if (f) f.close(); continue; }
            Serial.printf("<<<BEGIN %s %lu>>>\n", line+4, (unsigned long)f.size());
            uint8_t buf[512];
            size_t r;
            while ((r = f.read(buf, sizeof(buf))) > 0) Serial.write(buf, r);
            f.close();
            Serial.println("\n<<<END>>>");
        }
    }
}

// Horizontal bar centred on screen, top edge at y
static lv_obj_t* hLine(int y, int w, int h, lv_color_t color) {
    lv_obj_t* o=lv_obj_create(lv_scr_act());
    lv_obj_set_size(o,w,h);
    lv_obj_set_style_bg_color(o,color,0);
    lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(o,0,0);
    lv_obj_set_style_pad_all(o,0,0);
    lv_obj_set_style_radius(o,0,0);
    lv_obj_align(o,LV_ALIGN_TOP_MID,0,y);
    return o;
}

// ═══════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    mainTask = xTaskGetCurrentTaskHandle();   // setup() and loop() share this task
    logQueue = xQueueCreate(16, LOG_MSG_LEN);
#if DEBUG_BLE_DUMP
    dumpQueue = xQueueCreate(24, sizeof(DumpItem));
#endif

    // instance.begin() initializes all hardware including SD card mount
    instance.begin();
    beginLvglHelper(instance);
    instance.setBrightness(200);
    startMs = millis();
    lastActivityMs = millis();

    // Accelerometer for shake-to-wake at 25Hz
    if (instance.getDeviceProbe() & HW_BHI260AP_ONLINE) {
        accel.enable(25.0f, 0);
    }

    // Switch GNSS to GPS + Galileo + GLONASS (BeiDou off) — see notes above configGnssGlonass()
    if (instance.getDeviceProbe() & HW_GPS_ONLINE) {
        configGnssGlonass();
    }

    // SD check — instance.begin() already mounts SD internally.
    // SD.exists("/") is the reliable check (proven working 2026-05-24).
    // Do NOT call installSD() again — it would try to remount and fail.
    sdReady = SD.exists("/");
    logWritef("=== TWatch-DJI START (reset reason %d) ===", (int)esp_reset_reason());
    logWritef("SD: %s", sdReady?"OK":"NOT FOUND");
#if DEBUG_BLE_DUMP
    if (sdReady) {
        File f = SD.open(DUMP_PATH, FILE_APPEND);
        if (f) { f.printf("\n=== BOOT (reset reason %d) ===\n", (int)esp_reset_reason()); f.close(); }
    }
#endif

    lv_obj_set_style_bg_color(lv_scr_act(),lv_color_black(),0);
    lv_obj_set_style_bg_opa(lv_scr_act(),LV_OPA_COVER,0);

    lblTime=lv_label_create(lv_scr_act());
    lv_label_set_text(lblTime,"--:--");
    lv_obj_set_style_text_color(lblTime,lv_color_white(),0);
    lv_obj_set_style_text_font(lblTime,&lv_font_montserrat_28,0);
    lv_obj_align(lblTime,LV_ALIGN_TOP_LEFT,70,15);

    lblRec=lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(lblRec,&lv_font_montserrat_28,0);
    lv_obj_align(lblRec,LV_ALIGN_TOP_RIGHT,-90,15);
    renderRec();

    hLine(61,340,2,lv_palette_darken(LV_PALETTE_GREY,3));

    lblCoords=lv_label_create(lv_scr_act());
    lv_label_set_text(lblCoords,"GPS\nconnecting...");
    lv_obj_set_style_text_color(lblCoords,lv_palette_main(LV_PALETTE_YELLOW),0);
    lv_obj_set_style_text_font(lblCoords,&lv_font_montserrat_40,0);
    lv_obj_set_style_text_align(lblCoords,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblCoords,390);
    lv_obj_align(lblCoords,LV_ALIGN_TOP_MID,0,75);

    hLine(207,340,2,lv_palette_darken(LV_PALETTE_GREY,3));

    lblSpeed=lv_label_create(lv_scr_act());
    lv_label_set_text(lblSpeed,"-- km/h   -- m");
    lv_obj_set_style_text_color(lblSpeed,lv_palette_main(LV_PALETTE_CYAN),0);
    lv_obj_set_style_text_font(lblSpeed,&lv_font_montserrat_34,0);
    lv_obj_set_style_text_align(lblSpeed,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblSpeed,390);
    lv_obj_align(lblSpeed,LV_ALIGN_TOP_MID,0,220);

    lblCamSat=lv_label_create(lv_scr_act());
    lv_label_set_text(lblCamSat,"Cam: --   Sat: -");
    lv_obj_set_style_text_color(lblCamSat,lv_palette_darken(LV_PALETTE_GREY,1),0);
    lv_obj_set_style_text_font(lblCamSat,&lv_font_montserrat_30,0);
    lv_obj_set_style_text_align(lblCamSat,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblCamSat,390);
    lv_obj_align(lblCamSat,LV_ALIGN_TOP_MID,0,270);

    hLine(ZONE_SPLIT-1,410,3,lv_palette_main(LV_PALETTE_BLUE_GREY));   // drawn exactly at the touch boundary

    lblLog=lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(lblLog,&lv_font_montserrat_36,0);
    lv_obj_set_style_text_align(lblLog,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblLog,390);
    lv_obj_align(lblLog,LV_ALIGN_TOP_MID,0,345);
    renderLog();

    lblBatSd=lv_label_create(lv_scr_act());
    lv_label_set_text(lblBatSd,"Bat: --%    SD: --");
    lv_obj_set_style_text_color(lblBatSd,lv_palette_darken(LV_PALETTE_GREY,1),0);
    lv_obj_set_style_text_font(lblBatSd,&lv_font_montserrat_30,0);
    lv_obj_set_style_text_align(lblBatSd,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblBatSd,390);
    lv_obj_align(lblBatSd,LV_ALIGN_TOP_MID,0,425);

    logResumeTry();         // continue the track if we rebooted after a crash

    wantConnected = true;   // auto-connect on boot, retry every 5s until camera found
    triggerConnect = true;
    xTaskCreate(bleTask,"ble",8192,NULL,1,NULL);
}

// ═══════════════════════════════════════════════════════════
// Main Loop
// ═══════════════════════════════════════════════════════════
void loop() {
    instance.gps.loop();
    instance.loop();

    serviceSerialCommands();

    // ── Events from bleTask ──
    drainLogQueue();
#if DEBUG_BLE_DUMP
    drainDump();
#endif
    int8_t rd=recDone;
    if (rd) { recDone=0; onRecordResult(rd); }
    if (!connected && isRecording) { isRecording=false; renderRec(); }   // link dropped
    if (isRecording && camHeartbeatSeen() && camAsleep() && recCmd==0 && !camWaking) {
        isRecording=false; renderRec();                                   // switched off while recording
        logWrite("Camera asleep: REC reset");
    }
    if (connected && millis()-lastGpsPayloadMs>=1000) {
        lastGpsPayloadMs=millis();
        buildGpsPayload();
    }

    serviceHaptics();

    if (displayOn && millis()-lastActivityMs>DISPLAY_TIMEOUT_MS) {
        displayOn=false;
        sleepStartMs=millis();
        accelPrimed=false;
        instance.setBrightness(0);
        logWrite("Display sleep");
    }

    if (!displayOn && millis()-sleepStartMs>2000 && accel.hasUpdated()) {
        float x=accel.getX(),y=accel.getY(),z=accel.getZ();
        float mag=sqrtf(x*x+y*y+z*z);
        float delta=fabsf(mag-prevAccelMag);
        prevAccelMag=mag;
        if (!accelPrimed) accelPrimed=true;   // was compared against 0 / a stale value
        else if (delta>2.0f) { wakeDisplay(); logWrite("Wake by shake"); }
    }

    if (!firstFix && instance.gps.location.isValid()) {
        firstFix=true;
        uint32_t t=(millis()-startMs)/1000;
        logWritef("GPS first fix: %.5f,%.5f %d sats, %lu sec",
            instance.gps.location.lat(),instance.gps.location.lng(),
            (int)instance.gps.satellites.value(),t);
    }

    if (displayOn && millis()-lastUiUpdate>2000) {   // no redraws while the screen is dark
        lastUiUpdate=millis();
        updateUI();
    }

    if (gpxLogging && millis()-lastTrackMs>TRACK_INTERVAL_MS) {
        lastTrackMs=millis();
        gpxWriteTrackPoint();
    }

    if (millis()-lastBatLogMs>BAT_LOG_INTERVAL_MS) {
        lastBatLogMs=millis();
        logWritef("Battery: %d%%",instance.pmu.getBatteryPercent());
    }

    bool isTouched=instance.getTouched();
    // getTouched() only reports the touch IRQ flag. Right after lift-off the panel raises
    // one more IRQ without a point; taken as a touch, it became a phantom tap with bogus
    // coordinates (upper zone → REC after a logger hold). Only a reported point is a touch.
    if (isTouched && !touchActive && instance.getPoint(&touchX,&touchY) > 0) {
        touchActive=true;
        touchStartMs=millis();
        touchWasOn=displayOn;
        touchHoldFired=false;
    }
    // Logger hold in the lower zone fires while the finger is still down, so its buzz
    // says "done, let go"; the release of that touch is then ignored
    if (isTouched && touchActive && touchWasOn && !touchHoldFired &&
        touchY>=ZONE_SPLIT && millis()-touchStartMs>=LOGGER_HOLD_MS) {
        touchHoldFired=true;
        toggleLogger();
    }
    if (!isTouched && touchActive) {
        touchActive=false;
        uint32_t dur=millis()-touchStartMs;
        bool wasOn=displayOn;
        wakeDisplay();
        if (wasOn && !touchHoldFired) handleTouchEnd(touchX,touchY,dur);   // tap on dark screen only wakes it
        else if (touchHoldFired) lastTouchMs=millis();   // the 80 ms debounce also follows a hold
    }

    lv_task_handler();
    delay(5);
}