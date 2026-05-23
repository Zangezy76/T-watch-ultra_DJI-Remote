#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <NimBLEDevice.h>
#include <SD.h>
#include <FS.h>
#include <bosch/BoschSensorDataHelper.hpp>

static const char* CAMERA_MAC   = "04:a8:5a:8c:bc:6c";
static const uint32_t DEVICE_ID = 0x00000015;

#define TRKSEG_CLOSE     "    </trkseg>\n  </trk>\n"
#define GPX_CLOSE        "</gpx>\n"
#define TRKSEG_CLOSE_LEN 22
#define GPX_CLOSE_LEN    7
#define FULL_CLOSE_LEN   29
static char  gpxPath[32] = "";
static int   lastGpxDay  = -1;
static bool  sdReady     = false;
static bool  gpxLogging  = false;
static uint32_t lastTrackMs = 0;
#define TRACK_INTERVAL_MS 60000UL

static char logPath[32] = "";
static int  lastLogDay  = -1;
static uint32_t startMs = 0;
static bool firstFix    = false;
static uint32_t lastBatLogMs = 0;
#define BAT_LOG_INTERVAL_MS 1800000UL

#define DISPLAY_TIMEOUT_MS 60000UL
static uint32_t lastActivityMs = 0;
static bool displayOn = true;
static float prevAccelMag = 0.0f;
static uint32_t sleepStartMs = 0;

SensorXYZ accel(SensorBHI260AP::ACCEL_PASSTHROUGH, instance.sensor);

#define ZONE_SPLIT 335
static lv_obj_t* lblTime   = nullptr;
static lv_obj_t* lblRec    = nullptr;
static lv_obj_t* lblCoords = nullptr;
static lv_obj_t* lblSpeed  = nullptr;
static lv_obj_t* lblCamSat = nullptr;
static lv_obj_t* lblLog    = nullptr;
static lv_obj_t* lblBatSd  = nullptr;

static NimBLEClient*               pClient   = nullptr;
static NimBLERemoteCharacteristic* pWriteChr = nullptr;
static bool    connected     = false;
static bool    isRecording   = false;
static uint16_t seqNum       = 0;
static bool    triggerConnect = false;

static uint32_t lastUiUpdate = 0;

static uint32_t touchStartMs = 0;
static bool touchActive = false;
static int16_t touchX = 0, touchY = 0;
static uint32_t lastTouchMs = 0;

// ═══════════════════════════════════════════════════════════
// CRC16 / CRC32
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
    uint16_t crc=0x3AA3;
    while(l--){uint8_t i=(crc^*d++)&0xFF;crc=(crc16_table[i]^(crc>>8))&0xFFFF;}
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
uint32_t dji_crc32(const uint8_t* d, size_t l) {
    uint32_t crc=0x00003AA3;
    while(l--){uint8_t i=(crc^*d++)&0xFF;crc=(crc32_table[i]^(crc>>8))&0xFFFFFFFF;}
    return crc;
}
size_t dji_build_frame(uint8_t* buf, uint8_t cmd_set, uint8_t cmd_id,
                       const uint8_t* data, size_t data_len, uint16_t seq) {
    size_t total=14+data_len+4, off=0;
    buf[off++]=0xAA; buf[off++]=total&0xFF; buf[off++]=(total>>8)&0xFF;
    buf[off++]=0x00; buf[off++]=0x00;
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

// ═══════════════════════════════════════════════════════════
// Дисплей — только яркость, без sleepDisplay()
// ═══════════════════════════════════════════════════════════
void wakeDisplay() {
    if (!displayOn) {
        displayOn = true;
        instance.setBrightness(200);
    }
    lastActivityMs = millis();
    Serial.printf("[WAKE] lastActivity reset, displayOn=%d\n", displayOn);
}

// ═══════════════════════════════════════════════════════════
// LOG
// ═══════════════════════════════════════════════════════════
void logWrite(const char* msg) {
    Serial.println(msg);
    if(!sdReady) return;
    if(instance.gps.date.isValid()&&instance.gps.date.day()!=lastLogDay){
        lastLogDay=instance.gps.date.day();
        snprintf(logPath,sizeof(logPath),"/log_%04d_%02d_%02d.txt",
            instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day());
    }
    if(logPath[0]=='\0') snprintf(logPath,sizeof(logPath),"/log_nodate.txt");
    File f=SD.open(logPath,FILE_APPEND);
    if(!f) return;
    if(instance.gps.time.isValid())
        f.printf("[%02d:%02d:%02d] %s\n",
            instance.gps.time.hour(),instance.gps.time.minute(),
            instance.gps.time.second(),msg);
    else f.printf("[%08lu] %s\n",millis(),msg);
    f.close();
}
void logWritef(const char* fmt,...) {
    char buf[128]; va_list a; va_start(a,fmt);
    vsnprintf(buf,sizeof(buf),fmt,a); va_end(a);
    logWrite(buf);
}

// ═══════════════════════════════════════════════════════════
// BLE команды
// ═══════════════════════════════════════════════════════════
void sendRecordCommand(bool start) {
    if(!pWriteChr||!connected) return;
    uint8_t payload[9];
    payload[0]=DEVICE_ID&0xFF; payload[1]=(DEVICE_ID>>8)&0xFF;
    payload[2]=(DEVICE_ID>>16)&0xFF; payload[3]=(DEVICE_ID>>24)&0xFF;
    payload[4]=start?0x00:0x01;
    payload[5]=payload[6]=payload[7]=payload[8]=0x00;
    uint8_t frame[64];
    size_t len=dji_build_frame(frame,0x1D,0x03,payload,sizeof(payload),++seqNum);
    bool ok=pWriteChr->writeValue(frame,len,false);
    logWritef("REC %s: %s",start?"START":"STOP",ok?"OK":"FAIL");
}

void sendGpsData() {
    if(!pWriteChr||!connected) return;
    if(!instance.gps.location.isValid()||!instance.gps.date.isValid()||!instance.gps.time.isValid()) return;
    if(instance.gps.satellites.value()==0) return;
    uint8_t payload[45]; memset(payload,0,sizeof(payload)); uint8_t* p=payload;
    int32_t ymd=instance.gps.date.year()*10000+instance.gps.date.month()*100+instance.gps.date.day();
    memcpy(p,&ymd,4); p+=4;
    int32_t hms=instance.gps.time.hour()*10000+instance.gps.time.minute()*100+instance.gps.time.second();
    memcpy(p,&hms,4); p+=4;
    int32_t lon=(int32_t)(instance.gps.location.lng()*1e7); memcpy(p,&lon,4); p+=4;
    int32_t lat=(int32_t)(instance.gps.location.lat()*1e7); memcpy(p,&lat,4); p+=4;
    int32_t alt=instance.gps.altitude.isValid()?(int32_t)(instance.gps.altitude.meters()*1000.0f):0;
    memcpy(p,&alt,4); p+=4;
    float spd=instance.gps.speed.isValid()?(float)instance.gps.speed.mps():0.0f;
    if(spd<0.5f) spd=0.0f;
    float crs=instance.gps.course.isValid()?(float)(instance.gps.course.deg()*3.14159265f/180.0f):0.0f;
    float sn=spd*cosf(crs)*100.0f,se=spd*sinf(crs)*100.0f,sd=0.0f;
    memcpy(p,&sn,4); p+=4; memcpy(p,&se,4); p+=4; memcpy(p,&sd,4); p+=4;
    float va=2.0f,ha=2.0f,sa=0.1f;
    memcpy(p,&va,4); p+=4; memcpy(p,&ha,4); p+=4; memcpy(p,&sa,4); p+=4;
    *p=(uint8_t)instance.gps.satellites.value();
    uint8_t frame[80];
    size_t len=dji_build_frame(frame,0x00,0x17,payload,sizeof(payload),++seqNum);
    bool ok=pWriteChr->writeValue(frame,len,false);
    if(!ok) logWrite("GPS inject FAIL");
}

// ═══════════════════════════════════════════════════════════
// GPX
// ═══════════════════════════════════════════════════════════
void gpxCreateFile(const char* path) {
    File f=SD.open(path,FILE_WRITE);
    if(!f){logWritef("GPX create FAIL: %s",path);return;}
    f.printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    f.printf("<gpx version=\"1.1\" creator=\"TWatch-DJI\">\n");
    f.printf("  <trk>\n    <name>%04d-%02d-%02d</name>\n    <trkseg>\n",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day());
    f.print(TRKSEG_CLOSE); f.print(GPX_CLOSE); f.close();
    logWritef("GPX created: %s",path);
}
void gpxCheckDay() {
    if(!sdReady||!instance.gps.date.isValid()) return;
    if(instance.gps.date.day()==lastGpxDay) return;
    lastGpxDay=instance.gps.date.day();
    snprintf(gpxPath,sizeof(gpxPath),"/track_%04d_%02d_%02d.gpx",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day());
    if(!SD.exists(gpxPath)) gpxCreateFile(gpxPath);
    else logWritef("GPX using: %s",gpxPath);
}
void gpxWriteTrackPoint() {
    if(!sdReady||gpxPath[0]=='\0'||!instance.gps.location.isValid()) return;
    File f=SD.open(gpxPath,FILE_WRITE);
    if(!f) return;
    uint32_t size=f.size();
    if(size<FULL_CLOSE_LEN){f.close();return;}
    f.seek(size-FULL_CLOSE_LEN);
    f.printf("      <trkpt lat=\"%.6f\" lon=\"%.6f\">\n",
        instance.gps.location.lat(),instance.gps.location.lng());
    f.printf("        <ele>%.1f</ele>\n",
        instance.gps.altitude.isValid()?instance.gps.altitude.meters():0.0);
    f.printf("        <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day(),
        instance.gps.time.hour(),instance.gps.time.minute(),instance.gps.time.second());
    if(instance.gps.speed.isValid())
        f.printf("        <speed>%.2f</speed>\n",instance.gps.speed.mps());
    f.printf("      </trkpt>\n");
    f.print(TRKSEG_CLOSE); f.print(GPX_CLOSE); f.close();
}
void gpxWriteWaypoint(const char* name) {
    if(!sdReady||gpxPath[0]=='\0'||!instance.gps.location.isValid()) return;
    File f=SD.open(gpxPath,FILE_WRITE);
    if(!f) return;
    uint32_t size=f.size();
    if(size<GPX_CLOSE_LEN){f.close();return;}
    f.seek(size-GPX_CLOSE_LEN);
    f.printf("  <wpt lat=\"%.6f\" lon=\"%.6f\">\n",
        instance.gps.location.lat(),instance.gps.location.lng());
    f.printf("    <ele>%.1f</ele>\n",
        instance.gps.altitude.isValid()?instance.gps.altitude.meters():0.0);
    f.printf("    <time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n",
        instance.gps.date.year(),instance.gps.date.month(),instance.gps.date.day(),
        instance.gps.time.hour(),instance.gps.time.minute(),instance.gps.time.second());
    f.printf("    <name>%s</name>\n",name);
    f.printf("  </wpt>\n");
    f.print(GPX_CLOSE); f.close();
    logWritef("GPX wpt: %s",name);
}

// ═══════════════════════════════════════════════════════════
// Управление записью и логгером
// ═══════════════════════════════════════════════════════════
void toggleRecording() {
    instance.setHapticEffects(14); instance.vibrator();
    if(!connected||!pWriteChr) return;
    isRecording=!isRecording;
    sendRecordCommand(isRecording);
    gpxWriteWaypoint(isRecording?"REC START":"REC STOP");
    if(!isRecording){delay(200);instance.setHapticEffects(14);instance.vibrator();}
    if(isRecording){
        lv_label_set_text(lblRec,"● REC");
        lv_obj_set_style_text_color(lblRec,lv_palette_main(LV_PALETTE_RED),0);
    } else {
        lv_label_set_text(lblRec,"●");
        lv_obj_set_style_text_color(lblRec,lv_palette_darken(LV_PALETTE_GREY,2),0);
    }
}

void toggleLogger() {
    instance.setHapticEffects(14); instance.vibrator();
    gpxLogging=!gpxLogging;
    if(gpxLogging){
        gpxCheckDay();
        lv_label_set_text(lblLog,"▶ LOG: ON");
        lv_obj_set_style_text_color(lblLog,lv_palette_main(LV_PALETTE_GREEN),0);
        logWrite("Logger START");
    } else {
        lv_label_set_text(lblLog,"■ LOG: OFF");
        lv_obj_set_style_text_color(lblLog,lv_palette_darken(LV_PALETTE_GREY,2),0);
        logWrite("Logger STOP");
    }
}

// ═══════════════════════════════════════════════════════════
// Обработка тапов
// ═══════════════════════════════════════════════════════════
void handleTouchEnd(int16_t x, int16_t y, uint32_t duration) {
    if(millis()-lastTouchMs<250) return;
    lastTouchMs=millis();

    if(y < ZONE_SPLIT) {
        if(duration < 600) {
            static uint32_t lastTopTapMs = 0;
            static int topTapCount = 0;
            if(millis()-lastTopTapMs < 400) topTapCount++;
            else topTapCount = 1;
            lastTopTapMs = millis();
            if(topTapCount >= 2) {
                topTapCount = 0;
                if(!connected) {
                    triggerConnect = true;
                    instance.setHapticEffects(7); instance.vibrator();
                    delay(100);
                    instance.setHapticEffects(7); instance.vibrator();
                    lv_label_set_text(lblCamSat,"Cam: connecting...");
                    logWrite("Double tap: connect camera");
                }
            } else {
                toggleRecording();
            }
        }
    } else {
        if(duration >= 600) {
            toggleLogger();
        } else {
            instance.setHapticEffects(1); instance.vibrator();
        }
    }
}

// ═══════════════════════════════════════════════════════════
// UI
// ═══════════════════════════════════════════════════════════
void updateUI() {
    char buf[128];
    if(instance.gps.time.isValid())
        snprintf(buf,sizeof(buf),"%02d:%02d UTC",
            instance.gps.time.hour(),instance.gps.time.minute());
    else snprintf(buf,sizeof(buf),"--:--");
    lv_label_set_text(lblTime,buf);

    if(instance.gps.location.isValid()){
        lv_obj_set_style_text_color(lblCoords,lv_color_white(),0);
        snprintf(buf,sizeof(buf),"%.5f\n%.5f",
            instance.gps.location.lat(),instance.gps.location.lng());
    } else {
        lv_obj_set_style_text_color(lblCoords,lv_palette_main(LV_PALETTE_YELLOW),0);
        snprintf(buf,sizeof(buf),"GPS\nconnecting...");
    }
    lv_label_set_text(lblCoords,buf);

    if(instance.gps.location.isValid()){
        float kmh=instance.gps.speed.isValid()?(float)instance.gps.speed.kmph():0.0f;
        if(kmh<2.0f) kmh=0.0f;
        snprintf(buf,sizeof(buf),"%.1f km/h   %.0f m",
            kmh,instance.gps.altitude.isValid()?(float)instance.gps.altitude.meters():0.0f);
    } else {
        snprintf(buf,sizeof(buf),"-- km/h   -- m");
    }
    lv_label_set_text(lblSpeed,buf);

    int sats=(int)instance.gps.satellites.value();
    if(connected)
        snprintf(buf,sizeof(buf),"Cam: OK   Sat: %d",sats);
    else
        snprintf(buf,sizeof(buf),"Cam: --   Sat: %d",sats);
    lv_label_set_text(lblCamSat,buf);

    snprintf(buf,sizeof(buf),"Bat: %d%%    SD: %s",
        instance.pmu.getBatteryPercent(), sdReady?"OK":"--");
    lv_label_set_text(lblBatSd,buf);
}

// ═══════════════════════════════════════════════════════════
// BLE
// ═══════════════════════════════════════════════════════════
class CameraCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* c, int reason) override {
        connected=false; isRecording=false; pWriteChr=nullptr;
        lv_label_set_text(lblRec,"●");
        lv_obj_set_style_text_color(lblRec,lv_palette_darken(LV_PALETTE_GREY,2),0);
        logWritef("BLE disconnected %d",reason);
    }
};

void bleTask(void* pvParameters) {
    vTaskDelay(2000/portTICK_PERIOD_MS);
    NimBLEDevice::init("TWatch-DJI");
    NimBLEDevice::setPower(3);
    NimBLEDevice::setMTU(512);
    NimBLEAddress addr(std::string(CAMERA_MAC),BLE_ADDR_PUBLIC);
    while(true){
        while(!triggerConnect) vTaskDelay(200/portTICK_PERIOD_MS);
        triggerConnect=false;
        pClient=NimBLEDevice::createClient();
        pClient->setClientCallbacks(new CameraCallbacks(),false);
        pClient->setConnectTimeout(10000);
        logWrite("BLE connecting...");
        if(!pClient->connect(addr)){
            logWrite("BLE FAIL");
            NimBLEDevice::deleteClient(pClient); pClient=nullptr;
            lv_label_set_text(lblCamSat,"Cam: FAIL (2x tap)");
            continue;
        }
        auto svc=pClient->getService("0000fff0-0000-1000-8000-00805f9b34fb");
        if(!svc){pClient->disconnect();NimBLEDevice::deleteClient(pClient);pClient=nullptr;continue;}
        pWriteChr=svc->getCharacteristic("0000fff3-0000-1000-8000-00805f9b34fb");
        if(!pWriteChr){pClient->disconnect();NimBLEDevice::deleteClient(pClient);pClient=nullptr;continue;}
        auto nc=svc->getCharacteristic("0000fff4-0000-1000-8000-00805f9b34fb");
        if(nc&&nc->canNotify())
            nc->subscribe(true,[](NimBLERemoteCharacteristic*,uint8_t*,size_t,bool){});
        connected=true;
        logWrite("BLE connected!");
        while(connected){vTaskDelay(1000/portTICK_PERIOD_MS);sendGpsData();}
        logWrite("BLE lost.");
        pWriteChr=nullptr;
        NimBLEDevice::deleteClient(pClient); pClient=nullptr;
    }
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════
void setup(){
    Serial.begin(115200);
    instance.begin();
    beginLvglHelper(instance);
    instance.setBrightness(200);
    startMs=millis();
    lastActivityMs=millis();

    if(instance.getDeviceProbe() & HW_BHI260AP_ONLINE){
        accel.enable(25.0f, 0);
        Serial.println("[IMU] Accel enabled");
    }

    sdReady=SD.exists("/");
    logWrite("=== TWatch-DJI START ===");
    logWritef("SD: %s",sdReady?"OK":"NOT FOUND");

    lv_obj_set_style_bg_color(lv_scr_act(),lv_color_black(),0);
    lv_obj_set_style_bg_opa(lv_scr_act(),LV_OPA_COVER,0);

    lblTime=lv_label_create(lv_scr_act());
    lv_label_set_text(lblTime,"--:--");
    lv_obj_set_style_text_color(lblTime,lv_color_white(),0);
    lv_obj_set_style_text_font(lblTime,&lv_font_montserrat_28,0);
    lv_obj_align(lblTime,LV_ALIGN_TOP_LEFT,70,15);

    lblRec=lv_label_create(lv_scr_act());
    lv_label_set_text(lblRec,"●");
    lv_obj_set_style_text_color(lblRec,lv_palette_darken(LV_PALETTE_GREY,2),0);
    lv_obj_set_style_text_font(lblRec,&lv_font_montserrat_28,0);
    lv_obj_align(lblRec,LV_ALIGN_TOP_RIGHT,-90,15);

    lv_obj_t* l1=lv_label_create(lv_scr_act());
    lv_label_set_text(l1,"────────────────");
    lv_obj_set_style_text_color(l1,lv_palette_darken(LV_PALETTE_GREY,3),0);
    lv_obj_set_style_text_font(l1,&lv_font_montserrat_14,0);
    lv_obj_align(l1,LV_ALIGN_TOP_MID,0,55);

    lblCoords=lv_label_create(lv_scr_act());
    lv_label_set_text(lblCoords,"GPS\nconnecting...");
    lv_obj_set_style_text_color(lblCoords,lv_palette_main(LV_PALETTE_YELLOW),0);
    lv_obj_set_style_text_font(lblCoords,&lv_font_montserrat_30,0);
    lv_obj_set_style_text_align(lblCoords,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblCoords,390);
    lv_obj_align(lblCoords,LV_ALIGN_TOP_MID,0,75);

    lv_obj_t* l2=lv_label_create(lv_scr_act());
    lv_label_set_text(l2,"────────────────");
    lv_obj_set_style_text_color(l2,lv_palette_darken(LV_PALETTE_GREY,3),0);
    lv_obj_set_style_text_font(l2,&lv_font_montserrat_14,0);
    lv_obj_align(l2,LV_ALIGN_TOP_MID,0,200);

    lblSpeed=lv_label_create(lv_scr_act());
    lv_label_set_text(lblSpeed,"-- km/h   -- m");
    lv_obj_set_style_text_color(lblSpeed,lv_palette_main(LV_PALETTE_CYAN),0);
    lv_obj_set_style_text_font(lblSpeed,&lv_font_montserrat_26,0);
    lv_obj_set_style_text_align(lblSpeed,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblSpeed,390);
    lv_obj_align(lblSpeed,LV_ALIGN_TOP_MID,0,220);

    lblCamSat=lv_label_create(lv_scr_act());
    lv_label_set_text(lblCamSat,"Cam: --   Sat: -");
    lv_obj_set_style_text_color(lblCamSat,lv_palette_darken(LV_PALETTE_GREY,1),0);
    lv_obj_set_style_text_font(lblCamSat,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_align(lblCamSat,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblCamSat,390);
    lv_obj_align(lblCamSat,LV_ALIGN_TOP_MID,0,270);

    lv_obj_t* zoneLine=lv_obj_create(lv_scr_act());
    lv_obj_set_size(zoneLine,410,3);
    lv_obj_set_style_bg_color(zoneLine,lv_palette_main(LV_PALETTE_BLUE_GREY),0);
    lv_obj_set_style_bg_opa(zoneLine,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(zoneLine,0,0);
    lv_obj_set_style_pad_all(zoneLine,0,0);
    lv_obj_set_style_radius(zoneLine,0,0);
    lv_obj_align(zoneLine,LV_ALIGN_TOP_MID,0,318);

    lblLog=lv_label_create(lv_scr_act());
    lv_label_set_text(lblLog,"■ LOG: OFF");
    lv_obj_set_style_text_color(lblLog,lv_palette_darken(LV_PALETTE_GREY,2),0);
    lv_obj_set_style_text_font(lblLog,&lv_font_montserrat_28,0);
    lv_obj_set_style_text_align(lblLog,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblLog,390);
    lv_obj_align(lblLog,LV_ALIGN_TOP_MID,0,345);

    lblBatSd=lv_label_create(lv_scr_act());
    lv_label_set_text(lblBatSd,"Bat: --%    SD: --");
    lv_obj_set_style_text_color(lblBatSd,lv_palette_darken(LV_PALETTE_GREY,1),0);
    lv_obj_set_style_text_font(lblBatSd,&lv_font_montserrat_20,0);
    lv_obj_set_style_text_align(lblBatSd,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_width(lblBatSd,390);
    lv_obj_align(lblBatSd,LV_ALIGN_TOP_MID,0,425);

    xTaskCreate(bleTask,"ble",8192,NULL,1,NULL);
}

// ═══════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════
void loop(){
    instance.gps.loop();
    instance.loop();

// Гасить дисплей
if(displayOn && millis()-lastActivityMs>DISPLAY_TIMEOUT_MS){
    displayOn=false;
    sleepStartMs=millis();
    instance.setBrightness(0);
    Serial.println("[SLEEP]");
}

// Пробуждение по встряхиванию — только после 2 сек сна
// Детектируем ИЗМЕНЕНИЕ ускорения, не абсолютное значение
if(!displayOn && millis()-sleepStartMs>2000 && accel.hasUpdated()){
    float x=accel.getX(), y=accel.getY(), z=accel.getZ();
    float mag=sqrtf(x*x+y*y+z*z);
    float delta=fabsf(mag-prevAccelMag);
    prevAccelMag=mag;
    Serial.printf("[ACCEL] mag=%.1f delta=%.1f\n", mag, delta);
    if(delta>3.0f){
        wakeDisplay();
    }
}

    // Первый GPS фикс
    if(!firstFix&&instance.gps.location.isValid()){
        firstFix=true;
        uint32_t t=(millis()-startMs)/1000;
        logWritef("GPS fix: %.5f,%.5f %dsats %lus",
            instance.gps.location.lat(),instance.gps.location.lng(),
            (int)instance.gps.satellites.value(),t);
        gpxCheckDay();
    }

    // UI каждые 2 сек
    if(millis()-lastUiUpdate>2000){
        lastUiUpdate=millis();
        updateUI();
    }

    // GPX трекпоинт
    if(gpxLogging&&millis()-lastTrackMs>TRACK_INTERVAL_MS){
        lastTrackMs=millis();
        gpxCheckDay();
        gpxWriteTrackPoint();
    }

    // Батарея каждые 30 мин
    if(millis()-lastBatLogMs>BAT_LOG_INTERVAL_MS){
        lastBatLogMs=millis();
        logWritef("Battery: %d%%",instance.pmu.getBatteryPercent());
    }

    // Обработка касания
    bool isTouched=instance.getTouched();
    if(isTouched&&!touchActive){
        touchActive=true;
        touchStartMs=millis();
        instance.getPoint(&touchX,&touchY);
    }
    if(!isTouched&&touchActive){
        touchActive=false;
        uint32_t dur=millis()-touchStartMs;
        wakeDisplay();  // будим только при завершении тапа
        if(displayOn){
            handleTouchEnd(touchX,touchY,dur);
        }
    }

    lv_task_handler();
    delay(5);
}