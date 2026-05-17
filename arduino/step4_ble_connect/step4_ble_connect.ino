#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <NimBLEDevice.h>

static const char* CAMERA_MAC   = "04:a8:5a:8c:bc:6c";
static const uint32_t DEVICE_ID = 0x00000015;

static lv_obj_t* lblStatus  = nullptr;
static lv_obj_t* lblInfo    = nullptr;
static char statusText[128] = "Starting...";
static char infoText[128]   = "";
static bool statusChanged   = false;
static bool infoChanged     = false;

static NimBLEClient*               pClient   = nullptr;
static NimBLERemoteCharacteristic* pWriteChr = nullptr;
static bool    connected   = false;
static bool    isRecording = false;
static bool    triggerRec  = false;
static uint16_t seqNum     = 0;

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

uint16_t dji_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x3AA3;
    while (len--) {
        uint8_t idx = (crc ^ *data++) & 0xFF;
        crc = (crc16_table[idx] ^ (crc >> 8)) & 0xFFFF;
    }
    return crc;
}

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

uint32_t dji_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0x00003AA3;
    while (len--) {
        uint8_t idx = (crc ^ *data++) & 0xFF;
        crc = (crc32_table[idx] ^ (crc >> 8)) & 0xFFFFFFFF;
    }
    return crc;
}

size_t dji_build_frame(uint8_t* buf, uint8_t cmd_set, uint8_t cmd_id,
                       const uint8_t* data, size_t data_len, uint16_t seq) {
    size_t total = 14 + data_len + 4;
    size_t off = 0;
    buf[off++] = 0xAA;
    buf[off++] = total & 0xFF;
    buf[off++] = (total >> 8) & 0xFF;
    buf[off++] = 0x00;
    buf[off++] = 0x00;
    buf[off++] = 0x00;
    buf[off++] = 0x00;
    buf[off++] = 0x00;
    buf[off++] = seq & 0xFF;
    buf[off++] = (seq >> 8) & 0xFF;
    uint16_t crc16 = dji_crc16(buf, off);
    buf[off++] = crc16 & 0xFF;
    buf[off++] = (crc16 >> 8) & 0xFF;
    buf[off++] = cmd_set;
    buf[off++] = cmd_id;
    if (data && data_len > 0) { memcpy(buf + off, data, data_len); off += data_len; }
    uint32_t crc32 = dji_crc32(buf, off);
    buf[off++] = crc32 & 0xFF;
    buf[off++] = (crc32 >> 8) & 0xFF;
    buf[off++] = (crc32 >> 16) & 0xFF;
    buf[off++] = (crc32 >> 24) & 0xFF;
    return off;
}

void sendRecordCommand(bool start) {
    if (!pWriteChr || !connected) return;
    uint8_t payload[9];
    payload[0] = DEVICE_ID & 0xFF;
    payload[1] = (DEVICE_ID >> 8) & 0xFF;
    payload[2] = (DEVICE_ID >> 16) & 0xFF;
    payload[3] = (DEVICE_ID >> 24) & 0xFF;
    payload[4] = start ? 0x00 : 0x01;
    payload[5] = payload[6] = payload[7] = payload[8] = 0x00;
    uint8_t frame[64];
    size_t len = dji_build_frame(frame, 0x1D, 0x03, payload, sizeof(payload), ++seqNum);
    Serial.printf("[TX] %s (%zu bytes): ", start ? "START" : "STOP", len);
    for (size_t i = 0; i < len; i++) Serial.printf("%02X ", frame[i]);
    Serial.println();
    bool ok = pWriteChr->writeValue(frame, len, false);
    Serial.printf("[TX] %s\n", ok ? "OK" : "FAIL");
}

class CameraCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* c, int reason) override {
        connected = false; isRecording = false; pWriteChr = nullptr;
        snprintf(statusText, sizeof(statusText), "DISCONNECTED\n%d", reason);
        statusChanged = true;
    }
};

void bleTask(void* pvParameters) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    NimBLEDevice::init("TWatch-DJI");
    NimBLEDevice::setPower(3);
    NimBLEDevice::setMTU(512);
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new CameraCallbacks(), false);
    pClient->setConnectTimeout(10000);
    NimBLEAddress addr(std::string(CAMERA_MAC), BLE_ADDR_PUBLIC);
    snprintf(statusText, sizeof(statusText), "Connecting...");
    statusChanged = true;
    if (!pClient->connect(addr)) {
        snprintf(statusText, sizeof(statusText), "FAILED!\nCamera off?");
        statusChanged = true;
        vTaskDelete(NULL); return;
    }
    auto svc = pClient->getService("0000fff0-0000-1000-8000-00805f9b34fb");
    if (!svc) {
        snprintf(statusText, sizeof(statusText), "SVC FFF0\nnot found!");
        statusChanged = true; pClient->disconnect(); vTaskDelete(NULL); return;
    }
    pWriteChr = svc->getCharacteristic("0000fff3-0000-1000-8000-00805f9b34fb");
    if (!pWriteChr) {
        snprintf(statusText, sizeof(statusText), "CHR FFF3\nnot found!");
        statusChanged = true; pClient->disconnect(); vTaskDelete(NULL); return;
    }
    auto notifyChr = svc->getCharacteristic("0000fff4-0000-1000-8000-00805f9b34fb");
    if (notifyChr && notifyChr->canNotify()) {
        notifyChr->subscribe(true, [](NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
            Serial.printf("[RX] %zu bytes: %02X %02X %02X %02X\n", len,
                len>0?data[0]:0, len>1?data[1]:0, len>2?data[2]:0, len>3?data[3]:0);
        });
    }
    connected = true;
    snprintf(statusText, sizeof(statusText), "Connected!\nPWR = REC");
    snprintf(infoText, sizeof(infoText), "RSSI: %d dBm\nREC: OFF", pClient->getRssi());
    statusChanged = true; infoChanged = true;
    while (connected) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        if (connected) {
            snprintf(infoText, sizeof(infoText), "RSSI: %d dBm\nREC: %s",
                pClient->getRssi(), isRecording ? "ON" : "OFF");
            infoChanged = true;
        }
    }
    vTaskDelete(NULL);
}

void onEvent(DeviceEvent_t event, void* params, void* user_data) {
    if (event == POWER_EVENT) {
        auto pmu = instance.getPMUEventType(params);
        if (pmu == PMU_EVENT_KEY_CLICKED) {
            instance.setHapticEffects(14);
            instance.vibrator();
            triggerRec = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    instance.begin();
    beginLvglHelper(instance);
    instance.setBrightness(200);
    instance.onEvent(onEvent, NULL, ALL_EVENT_MAX);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    lblStatus = lv_label_create(lv_scr_act());
    lv_label_set_text(lblStatus, statusText);
    lv_obj_set_style_text_color(lblStatus, lv_color_white(), 0);
    lv_obj_set_style_text_font(lblStatus, &lv_font_montserrat_20, 0);
    lv_obj_set_width(lblStatus, 390);
    lv_obj_set_style_text_align(lblStatus, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblStatus, LV_ALIGN_TOP_MID, 0, 30);
    lblInfo = lv_label_create(lv_scr_act());
    lv_label_set_text(lblInfo, "");
    lv_obj_set_style_text_color(lblInfo, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(lblInfo, &lv_font_montserrat_14, 0);
    lv_obj_set_width(lblInfo, 390);
    lv_obj_set_style_text_align(lblInfo, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblInfo, LV_ALIGN_TOP_MID, 0, 180);
    xTaskCreate(bleTask, "ble", 8192, NULL, 1, NULL);
}

void loop() {
    instance.loop();
    if (statusChanged) { statusChanged = false; lv_label_set_text(lblStatus, statusText); }
    if (infoChanged)   { infoChanged   = false; lv_label_set_text(lblInfo, infoText); }
    if (triggerRec) {
        triggerRec = false;
        if (connected && pWriteChr) {
            isRecording = !isRecording;
            sendRecordCommand(isRecording);
            snprintf(statusText, sizeof(statusText), isRecording ? "● RECORDING" : "Connected!\nPWR = REC");
            lv_obj_set_style_text_color(lblStatus,
                isRecording ? lv_palette_main(LV_PALETTE_RED) : lv_color_white(), 0);
            statusChanged = true;
        }
    }
    lv_task_handler();
    delay(5);
}