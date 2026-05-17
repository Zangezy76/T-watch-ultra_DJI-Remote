/*
 * LILYGO T-Watch Ultra Hardware Abstraction Layer
 * Stub implementation - LilyGoLib will be integrated when device arrives
 */
#include "twatch_ultra_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char *TAG = "TWATCH_HAL";
static bool s_initialized = false;

int twatch_ultra_init(void)
{
    ESP_LOGI(TAG, "T-Watch Ultra HAL stub - init");
    s_initialized = true;
    return 0;
}

lv_display_t* twatch_ultra_lvgl_init(void)
{
    ESP_LOGI(TAG, "T-Watch Ultra HAL stub - lvgl_init");
    return lv_display_get_default();
}

void twatch_ultra_backlight_on(void)  {}
void twatch_ultra_backlight_off(void) {}
void twatch_ultra_set_brightness(uint8_t level) {}

bool twatch_ultra_crown_pressed(void)      { return false; }
bool twatch_ultra_touch_a_pressed(void)    { return false; }
bool twatch_ultra_touch_b_pressed(void)    { return false; }

void twatch_ultra_vibrate(uint32_t ms)     {}
void twatch_ultra_shutdown(void)           {}

uint8_t twatch_ultra_get_battery_percent(void) { return 100; }