/*
 * LILYGO T-Watch Ultra Hardware Abstraction Layer Header
 */
#ifndef TWATCH_ULTRA_HAL_H
#define TWATCH_ULTRA_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "lvgl.h"

/* Display resolution - T-Watch Ultra AMOLED */
#define TWATCH_LCD_H_RES    410
#define TWATCH_LCD_V_RES    502

/* Brightness levels */
#define TWATCH_BRIGHTNESS_MAX   255
#define TWATCH_BRIGHTNESS_MIN   0
#define TWATCH_BRIGHTNESS_DEFAULT 128

/* Function prototypes */
int twatch_ultra_init(void);
lv_display_t* twatch_ultra_lvgl_init(void);
void twatch_ultra_backlight_on(void);
void twatch_ultra_backlight_off(void);
void twatch_ultra_set_brightness(uint8_t level);

/* Buttons */
bool twatch_ultra_crown_pressed(void);
bool twatch_ultra_touch_a_pressed(void);
bool twatch_ultra_touch_b_pressed(void);

/* Vibration feedback */
void twatch_ultra_vibrate(uint32_t ms);

/* Power */
void twatch_ultra_shutdown(void);
uint8_t twatch_ultra_get_battery_percent(void);

#endif /* TWATCH_ULTRA_HAL_H */