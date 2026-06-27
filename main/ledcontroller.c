/*
 * ledcontroller.c
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#include "ledcontroller.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "ringbuf.h"

static const char *TAG = "led_ctrl";

// ── Private state ─────────────────────────────────────────────────────────────
static volatile uint16_t s_last_pot    = 0;
static volatile uint16_t s_last_sensor = 0;
static volatile uint8_t  s_brightness_pct = 70;  // Start at 70%

static ring_buf_t        *s_rb   = NULL;
static SemaphoreHandle_t  s_wake = NULL;

// ── LEDC helpers ──────────────────────────────────────────────────────────────
#define LEDC_FREQ_HZ    5000
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT    // 0..255

static void ledc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Red → GPIO2 → channel 0
    ledc_channel_config_t ch_red = {
        .gpio_num   = LED_GPIO_RED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_red));

    // Green → GPIO2 → channel 1
    ledc_channel_config_t ch_green = {
        .gpio_num   = LED_GPIO_GREEN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_green));

    // Blue → GPIO2 → channel 2
    ledc_channel_config_t ch_blue = {
        .gpio_num   = LED_GPIO_BLUE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_blue));

    ESP_LOGI(TAG, "LEDC initialized: R=GPIO%d G=GPIO%d B=GPIO%d",
             LED_GPIO_RED, LED_GPIO_GREEN, LED_GPIO_BLUE);
}

// Set RGB duty (0..255 each)
static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// ── Brightness calculation based on current sensor ──────────────────────────
// 
// Mapping: sensor_raw (0-4095) → brightness (50% to 100%)
// 
// Idle/normal: ~70% brightness
// Current increases: brightness increases toward 100%
// Current decreases: brightness decreases toward 50%
// 
// The mapping uses a sigmoid-like curve for smooth transition
// with hysteresis to prevent flickering

#define BRIGHTNESS_MIN_PCT  50   // 50% - minimum brightness
#define BRIGHTNESS_IDLE_PCT 70   // 70% - normal/idle brightness
#define BRIGHTNESS_MAX_PCT  100  // 100% - maximum brightness

// ADC range for current sensor (adjust based on your sensor)
#define SENSOR_MIN_RAW     0     // Minimum ADC reading (0V)
#define SENSOR_IDLE_RAW    2048  // Midpoint (~1.65V for ACS712 at 0A)
#define SENSOR_MAX_RAW     4095  // Maximum ADC reading (3.3V)

static uint8_t calculate_brightness(uint16_t sensor_raw)
{
    // Clamp sensor value to valid range
    if (sensor_raw < SENSOR_MIN_RAW) sensor_raw = SENSOR_MIN_RAW;
    if (sensor_raw > SENSOR_MAX_RAW) sensor_raw = SENSOR_MAX_RAW;

    // Normalize sensor reading to 0.0 - 1.0 range
    float normalized = (float)(sensor_raw - SENSOR_MIN_RAW) / 
                       (float)(SENSOR_MAX_RAW - SENSOR_MIN_RAW);

    // Map normalized value to brightness range:
    // normalized = 0.0 → 50% (min)
    // normalized = 0.5 → 70% (idle)
    // normalized = 1.0 → 100% (max)
    //
    // Using a cubic curve for smooth transition:
    // brightness = 50 + 20 * (2*normalized - 1) + 30 * (2*normalized - 1)^3
    // 
    // This gives:
    // - Gentle slope around idle point
    // - Smooth approach to limits
    // - Natural feel

    float x = (2.0f * normalized) - 1.0f;  // Map to -1.0 to +1.0 range
    
    // Cubic mapping: y = a*x + b*x^3
    // Where a and b control the slope and curve shape
    // At x=0 (idle): y=0 → brightness = 70%
    // At x=-1 (min): y=-1 → brightness = 50%
    // At x=+1 (max): y=+1 → brightness = 100%
    float y = (0.7f * x) + (0.3f * x * x * x);
    
    // Map y (-1 to +1) to brightness percentage (50% to 100%)
    float brightness_pct = BRIGHTNESS_IDLE_PCT + (y * (BRIGHTNESS_MAX_PCT - BRIGHTNESS_MIN_PCT) / 2.0f);
    
    // Clamp to valid range
    if (brightness_pct < BRIGHTNESS_MIN_PCT) brightness_pct = BRIGHTNESS_MIN_PCT;
    if (brightness_pct > BRIGHTNESS_MAX_PCT) brightness_pct = BRIGHTNESS_MAX_PCT;
    
    return (uint8_t)brightness_pct;
}

// Apply brightness to all RGB channels (white light)
static void apply_brightness(uint16_t pot_raw, uint16_t sensor_raw)
{
    // Calculate brightness based on current sensor
    uint8_t brightness_pct = calculate_brightness(sensor_raw);
    
    // Store for monitor task
    s_brightness_pct = brightness_pct;
    
    // Convert percentage to 8-bit duty (0-255)
    uint8_t duty = (brightness_pct * 255) / 100;
    
    // Set all channels to same value for white light
    // (or you can use pot to adjust color temperature)
    rgb_set(duty, duty, duty);
    
    ESP_LOGD(TAG, "Sensor: %d, Brightness: %d%%", sensor_raw, brightness_pct);
}

// ── Consumer task ─────────────────────────────────────────────────────────────
static void led_controller_task(void *arg)
{
    adc_sample_t sample;
    
    // Variables for averaging
    uint32_t sensor_sum = 0;
    uint32_t count = 0;

    while (1) {
        // Block until ISR gives semaphore
        xSemaphoreTake(s_wake, portMAX_DELAY);

        // Drain all available samples
        while (rb_pop(s_rb, &sample)) {
            sensor_sum += sample.current_sensor;
            count++;
        }

        // Apply averaged value to LED
        if (count > 0) {
            uint16_t sensor_avg = (uint16_t)(sensor_sum / count);
            
            // Store for monitor task
            s_last_sensor = sensor_avg;
            
            // Apply brightness based on sensor
            apply_brightness(sample.potentiometer, sensor_avg);
            
            // Reset accumulators
            sensor_sum = 0;
            count = 0;
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────
void led_controller_start(ring_buf_t *rb, SemaphoreHandle_t wake_sem)
{
    s_rb   = rb;
    s_wake = wake_sem;

    ledc_init();

    // Priority 5, core 1
    xTaskCreatePinnedToCore(
        led_controller_task,
        "led_ctrl",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "LED controller task started on core 1, priority 5");
    ESP_LOGI(TAG, "Brightness mapping: Min=50%%, Idle=70%%, Max=100%%");
}

uint16_t led_get_last_sensor_raw(void) { return s_last_sensor; }
uint8_t  led_get_brightness_pct(void)  { return s_brightness_pct; }