#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "connect_logic.h"
#include "status_logic.h"

#define TAG "LOGIC_LIGHT"

/* LED GPIO — board-specific via Kconfig board selector */
#if defined(CONFIG_BOARD_M5STACK_BASIC_V27)
#define LED_GPIO 2
#elif defined(CONFIG_BOARD_WAVESHARE_S3_LCD19)
#define LED_GPIO 15
#elif defined(CONFIG_BOARD_TWATCH_ULTRA)
#define LED_GPIO -1               /* T-Watch Ultra has no WS2812 LED */
#else
#define LED_GPIO 2
#endif

#define LED_STRIP_LENGTH 1

static led_strip_handle_t led_strip = NULL;

static void init_rgb_led(void) {
#if defined(CONFIG_BOARD_TWATCH_ULTRA)
    ESP_LOGI(TAG, "T-Watch Ultra: no RGB LED, skipping init");
    return;
#endif
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "RGB LED initialized");
}

static void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
#if defined(CONFIG_BOARD_TWATCH_ULTRA)
    return;
#endif
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

uint8_t led_red = 0, led_green = 0, led_blue = 0;
bool led_blinking = false;
bool current_led_on = false;

static void update_led_state() {
    connect_state_t current_connect_state = connect_logic_get_state();
    bool current_camera_recording = is_camera_recording();

    led_blinking = false;

    switch (current_connect_state) {
        case BLE_NOT_INIT:
            led_red = 13;
            led_green = 0;
            led_blue = 0;
            break;

        case BLE_INIT_COMPLETE:
            led_red = 13;
            led_green = 13;
            led_blue = 0;
            break;

        case BLE_SEARCHING:
            led_blinking = true;
            led_red = 0;
            led_green = 0;
            led_blue = 13;
            break;

        case BLE_CONNECTED:
            led_red = 0;
            led_green = 0;
            led_blue = 13;
            break;

        case PROTOCOL_CONNECTED:
            if (current_camera_recording) {
                led_blinking = true;
                led_red = 0;
                led_green = 13;
                led_blue = 0;
            } else {
                led_red = 0;
                led_green = 13;
                led_blue = 0;
            }
            break;

        default:
            led_red = 0;
            led_green = 0;
            led_blue = 0;
            break;
    }
}

static void led_state_timer_callback(TimerHandle_t xTimer) {
    update_led_state();
}

static void led_blink_timer_callback(TimerHandle_t xTimer) {
    if (led_blinking) {
        if (current_led_on) {
            set_rgb_color(0, 0, 0);
        } else {
            set_rgb_color(led_red, led_green, led_blue);
        }
        current_led_on = !current_led_on;
    } else {
        set_rgb_color(led_red, led_green, led_blue);
    }
}

int init_light_logic() {
#if defined(CONFIG_BOARD_TWATCH_ULTRA)
    ESP_LOGI(TAG, "T-Watch Ultra: light logic stub, no RGB LED");
    return 0;
#endif
    init_rgb_led();

    TimerHandle_t led_state_timer = xTimerCreate("led_state_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_state_timer_callback);

    if (led_state_timer != NULL) {
        xTimerStart(led_state_timer, 0);
        ESP_LOGI(TAG, "LED state timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED state timer");
        return -1;
    }

    TimerHandle_t led_blink_timer = xTimerCreate("led_blink_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_blink_timer_callback);

    if (led_blink_timer != NULL) {
        xTimerStart(led_blink_timer, 0);
        ESP_LOGI(TAG, "LED blink timer started successfully");
        return 0;
    } else {
        ESP_LOGE(TAG, "Failed to create LED blink timer");
        return -1;
    }
}